#include <3ds.h>
#include "hid.hpp"
#include "ipc.hpp"
#include "services/mcuhid.hpp"
#include "services/irrst.hpp"
#include <cstdio>
#include <cstring>
#include <atomic>
extern "C" {
#include "csvc.h"
#include "services.h"
}

#define ONERRSVCBREAK(ret) if(R_FAILED(ret)) svcBreak(USERBREAK_ASSERT);
#define OS_REMOTE_SESSION_CLOSED MAKERESULT(RL_STATUS,    RS_CANCELED, RM_OS, 26)
#define OS_INVALID_HEADER        MAKERESULT(RL_PERMANENT, RS_WRONGARG, RM_OS, 47)
#define OS_INVALID_IPC_PARAMATER MAKERESULT(RL_PERMANENT, RS_WRONGARG, RM_OS, 48)

extern volatile uint32_t diag_ir_require_commands;
extern volatile uint32_t diag_ir_disconnect_commands;
extern volatile uint32_t diag_ir_disconnect_successes;
extern volatile uint32_t diag_ir_disconnect_redundant;
extern volatile uint32_t diag_ir_disconnect_queue_last;
extern volatile uint32_t diag_ir_disconnect_queue_max;
extern volatile uint32_t diag_ir_input_requests;
extern volatile uint32_t diag_ir_first_input_requests;
extern volatile uint32_t diag_ir_subsequent_input_requests;

static volatile uint32_t diag_sleep_entries = 0;
static volatile uint32_t diag_sleep_exits = 0;
static volatile uint32_t diag_home_presses = 0;

extern volatile uint32_t diag_pad_wakeups;
extern volatile uint32_t diag_pad_successes;
extern volatile uint32_t diag_pad_failures;
extern volatile Result diag_last_pad_result;
extern std::atomic<uint32_t> diag_accel_remote_written;
extern std::atomic<uint32_t> diag_gyro_received;
extern std::atomic<uint32_t> diag_gyro_moving;
extern std::atomic<uint32_t> diag_gyro_remote_written;
extern std::atomic<uint32_t> diag_gyro_local_written;
extern std::atomic<uint32_t> diag_pad_raw_seen;
extern std::atomic<uint32_t> diag_pad_ring_seen;
extern std::atomic<uint32_t> diag_pad_pressed_seen;

#ifdef ENABLE_EXTERNAL_CCP
extern uint32_t diag_cpp_ipc_packets;
extern uint32_t diag_cpp_ipc_active_packets;
extern uint32_t diag_cpp_ipc_last_active;
extern uint32_t diag_cpp_report_stick;
extern uint32_t diag_cpp_report_zl;
extern uint32_t diag_cpp_report_zr;
extern uint32_t diag_cpp_report_last_stick;
extern uint32_t diag_cpp_queue_rejections;
extern uint64_t diag_cpp_batch_last_tick;
extern uint64_t diag_cpp_batch_ticks[4];
extern uint32_t diag_cpp_batch_next;
#endif

static void LogNotificationDiagnostics(uint32_t notid) {
    const bool home = notid == 0x204;
    const uint32_t padRaw = home ? diag_pad_raw_seen.exchange(0, std::memory_order_relaxed) : 0;
    const uint32_t padRing = home ? diag_pad_ring_seen.exchange(0, std::memory_order_relaxed) : 0;
    const uint32_t padPressed = home ? diag_pad_pressed_seen.exchange(0, std::memory_order_relaxed) : 0;
#ifdef ENABLE_EXTERNAL_CCP
    const uint64_t notification_tick = svcGetSystemTick();
#endif
    Handle file;
    u32 written = 0;
    u64 offset = 0;

    static const char path[] = "/rehid_notification_debug.txt";

    FS_Path emptyPath = fsMakePath(PATH_EMPTY, NULL);
    FS_Path filePath = fsMakePath(PATH_ASCII, path);

    Result ret = FSUSER_OpenFileDirectly(
        &file, ARCHIVE_SDMC, emptyPath, filePath, FS_OPEN_WRITE, 0
    );

    if (R_FAILED(ret)) {
        ret = FSUSER_OpenFileDirectly(
            &file, ARCHIVE_SDMC, emptyPath, filePath,
            FS_OPEN_WRITE | FS_OPEN_CREATE, 0
        );
    }

    if (R_FAILED(ret))
        return;

    ret = FSFILE_GetSize(file, &offset);

    if (R_SUCCEEDED(ret)) {
        char line[1024];

        int len = snprintf(
            line, sizeof(line),
            "Notification=%03lX sleep_in=%lu sleep_out=%lu home=%lu ir_require=%lu ir_disconnect=%lu disconnect_ok=%lu disconnect_redundant=%lu disconnect_queue_last=%lu disconnect_queue_max=%lu input_requests=%lu input_first=%lu input_subsequent=%lu pad_wakeups=%lu pad_successes=%lu pad_failures=%lu pad_last_result=%08lX\n",
            (unsigned long)notid,
            (unsigned long)diag_sleep_entries,
            (unsigned long)diag_sleep_exits,
            (unsigned long)diag_home_presses,
            (unsigned long)diag_ir_require_commands,
            (unsigned long)diag_ir_disconnect_commands,
            (unsigned long)diag_ir_disconnect_successes,
            (unsigned long)diag_ir_disconnect_redundant,
            (unsigned long)diag_ir_disconnect_queue_last,
            (unsigned long)diag_ir_disconnect_queue_max,
            (unsigned long)diag_ir_input_requests,
            (unsigned long)diag_ir_first_input_requests,
            (unsigned long)diag_ir_subsequent_input_requests,
            (unsigned long)diag_pad_wakeups,
            (unsigned long)diag_pad_successes,
            (unsigned long)diag_pad_failures,
            (unsigned long)diag_last_pad_result
        );

#ifdef ENABLE_EXTERNAL_CCP
        if (len > 0 && len < (int)sizeof(line)) {
            len += snprintf(
                line + len, sizeof(line) - len,
                "CPP IPC packets=%lu active=%lu last_active=%08lX\n",
                (unsigned long)diag_cpp_ipc_packets,
                (unsigned long)diag_cpp_ipc_active_packets,
                (unsigned long)diag_cpp_ipc_last_active
            );
        }
        if (len > 0 && len < (int)sizeof(line)) {
            len += snprintf(
                line + len, sizeof(line) - len,
                "CPP reports stick=%lu zl=%lu zr=%lu last_stick=%08lX\n",
                (unsigned long)diag_cpp_report_stick,
                (unsigned long)diag_cpp_report_zl,
                (unsigned long)diag_cpp_report_zr,
                (unsigned long)diag_cpp_report_last_stick
            );
        }
        if (len > 0 && len < (int)sizeof(line)) {
            const uint32_t next = diag_cpp_batch_next;
            len += snprintf(
                line + len, sizeof(line) - len,
                "CPP timing queue_rejections=%lu batch256_ticks=%llu,%llu,%llu,%llu tail_ticks=%llu tail_reports=%lu\n",
                (unsigned long)diag_cpp_queue_rejections,
                (unsigned long long)diag_cpp_batch_ticks[(next + 3) & 3],
                (unsigned long long)diag_cpp_batch_ticks[(next + 2) & 3],
                (unsigned long long)diag_cpp_batch_ticks[(next + 1) & 3],
                (unsigned long long)diag_cpp_batch_ticks[next],
                (unsigned long long)(diag_cpp_batch_last_tick != 0
                    ? notification_tick - diag_cpp_batch_last_tick : 0),
                (unsigned long)(diag_ir_input_requests & 255)
            );
        }
#endif

        if (home && len > 0 && len < (int)sizeof(line)) {
            len += snprintf(
                line + len, sizeof(line) - len,
                "PAD trace raw=%08lX ring=%08lX pressed=%08lX\n",
                (unsigned long)padRaw, (unsigned long)padRing, (unsigned long)padPressed
            );
        }

        if (home && len > 0 && len < (int)sizeof(line)) {
            len += snprintf(line + len, sizeof(line) - len,
                "GYRO trace received=%lu nonzero=%lu remote_written=%lu local_written=%lu accel_written=%lu\n",
                (unsigned long)diag_gyro_received.load(std::memory_order_relaxed),
                (unsigned long)diag_gyro_moving.load(std::memory_order_relaxed),
                (unsigned long)diag_gyro_remote_written.load(std::memory_order_relaxed),
                (unsigned long)diag_gyro_local_written.load(std::memory_order_relaxed),
                (unsigned long)diag_accel_remote_written.load(std::memory_order_relaxed));
        }

        if (len > 0 && len < (int)sizeof(line)) {
            Result write_result = FSFILE_Write(
                file, &written, offset, line, (u32)len, FS_WRITE_FLUSH
            );

            if (R_SUCCEEDED(write_result))
                offset += written;
        }

        uint32_t history_count = diag_ir_command_history_count;
        uint32_t history_next = diag_ir_command_history_next;
        uint32_t history_start = (history_next + 16 - history_count) & 15;

        for (uint32_t i = 0; i < history_count; i++) {
            uint32_t slot = (history_start + i) & 15;

            uint64_t tick = diag_ir_command_history[slot].tick;
            uint32_t command = diag_ir_command_history[slot].command;
            uint32_t detail = diag_ir_command_history[slot].detail;
            uint32_t result = diag_ir_command_history[slot].result;

            char history_line[112];

            int history_len = snprintf(
                history_line, sizeof(history_line),
                "IR tick=%llu cmd=%02lX detail=%08lX result=%08lX\n",
                (unsigned long long)tick,
                (unsigned long)command,
                (unsigned long)detail,
                (unsigned long)result
            );

            if (history_len <= 0 || history_len >= (int)sizeof(history_line))
                continue;

            written = 0;
            Result history_write_result = FSFILE_Write(
                file, &written, offset, history_line,
                (u32)history_len, FS_WRITE_FLUSH
            );

            if (R_FAILED(history_write_result))
                break;

            offset += written;
        }
    }

    FSFILE_Close(file);
}

static Result HandleNotifications(Hid *hid, int *exit) {
    uint32_t notid = 0;
    Result ret = srvReceiveNotification(&notid);

    if (R_FAILED(ret))
        return ret;

    switch (notid) {
        case 0x100: { // Exit
            *exit = 1;
            break;
        }

        case 0x104: { // Entering SleepMode
            diag_sleep_entries++;
            hid->EnteringSleepMode();
            LogNotificationDiagnostics(notid);
            break;
        }

        case 0x105: { // Exiting SleepMode
            diag_sleep_exits++;
            hid->ExitingSleepMode();
            LogNotificationDiagnostics(notid);
            break;
        }

        case 0x10C: { // Regular Application started
            break;
        }

        case 0x204: { // Home button pressed
            diag_home_presses++;
            LogNotificationDiagnostics(notid);
            break;
        }

        case 0x213: { // Shell Opened
            hid->IsShellOpened(true);
            break;
        }

        case 0x214: { // Shell Closed
            hid->IsShellOpened(false);
            break;
        }

        case 0x110: { // Application terminated
            if (!hid->GetRemapperObject()->isGlobal())
                hid->GetRemapperObject()->Reset();

            break;
        }

    }

    return 0;
}

extern "C"
{
    extern u32 __ctru_heap, __ctru_heap_size, __ctru_linear_heap, __ctru_linear_heap_size;
    extern char *fake_heap_start;
    extern char *fake_heap_end;
    extern 

    // this is called before main
    void __system_allocateHeaps(void) {
        u32 tmp = 0;
        __ctru_heap_size = 0x8000;
        // Allocate the application heap
        __ctru_heap = 0x08000000;
        svcControlMemoryEx(&tmp, __ctru_heap, 0x0, __ctru_heap_size, MEMOP_ALLOC, (MemPerm)(MEMPERM_READWRITE | MEMREGION_BASE), false);
        // Set up newlib heap
        fake_heap_start = (char*)__ctru_heap;
        fake_heap_end = fake_heap_start + __ctru_heap_size;
    }

    struct _reent* __wrap___syscall_getreent(void) {
        return _impure_ptr;
    }

    void __appInit() {
        srvSysInit();
        fsSysInit();
        Result ret = mcuHidInit();

        if (ret != 0)
            *(u32*)ret = 0xFFAA;

        ptmSysmInit();
    }

    // this is called after main exits
    void __appExit() {
        ptmSysmExit();
        mcuHidExit();
        srvSysExit();
    }

    // stubs for non-needed pre-main functions
    void __sync_init();
    void __sync_fini();
    void __system_initSyscalls();
    void __libc_init_array(void);
    void __libc_fini_array(void);

    void initSystem(void (*retAddr)(void)) {
        __libc_init_array();
        __sync_init();
        __system_initSyscalls();
        __system_allocateHeaps();
        __appInit();
    }

    void __ctru_exit(int rc) {
        __appExit();
        __sync_fini();
        __libc_fini_array();
        svcExitProcess();
    }
}

uint8_t sttbuf[0x100];

int main() {
    Hid hid;

    const char *srvnames[] = {"", "hid:SPVR", "hid:USER", "hid:QTM", "hid:NFC", "ir:USER"};

    hid.CheckIfIRPatchExists();
    hid.CreateAndMapMemoryBlock();
    hid.CreateRingsOnSharedmemoryBlock();
    hid.InitializePad();
    hid.InitializeAccelerometer();
    hid.InitializeGyroscope();
    hid.InitializeDebugPad();
    hid.StartThreadsForSampling();

    const s32 SERVICECOUNT = 5;
    const s32 INDEXMAX = SERVICECOUNT * 7 + 1;
    const s32 REMOTESESSIONINDEX = SERVICECOUNT + 1;

    Handle sessionhandles[INDEXMAX];

    u32 serviceindexes[7 * SERVICECOUNT];

    s32 handlecount = SERVICECOUNT + 1;

    for (int i = 1; i <= SERVICECOUNT; i++)
        ONERRSVCBREAK(srvRegisterService(&sessionhandles[i], srvnames[i], 6));

    ONERRSVCBREAK(srvEnableNotification(&sessionhandles[0]));

    ONERRSVCBREAK(srvSubscribe(0x104));
    ONERRSVCBREAK(srvSubscribe(0x105));
    ONERRSVCBREAK(srvSubscribe(0x10C)); // This is not subscribed to by official hid
    ONERRSVCBREAK(srvSubscribe(0x110)); // application termination, not subscribed by official hid
    ONERRSVCBREAK(srvSubscribe(0x204)); // Home Button pressed, not subscribed by official hid
    ONERRSVCBREAK(srvSubscribe(0x213));
    ONERRSVCBREAK(srvSubscribe(0x214));

    Handle target = 0;
    s32 targetindex = -1;
    int terminationflag = 0;


    // This is required to recieve buffers for ir:USER commands
    u32 *staticbuf = getThreadStaticBuffers();
    staticbuf[0] = IPC_Desc_StaticBuffer(0x100, 0);
    staticbuf[1] = (uint32_t)sttbuf;

    for (;;) {
        s32 index;

        if (!target) {
            if (terminationflag && handlecount == REMOTESESSIONINDEX)
                break;
            else
                *getThreadCommandBuffer() = 0xFFFF0000;
        }

        Result res = svcReplyAndReceive(&index, sessionhandles, handlecount, target);
        s32 lasttargetindex = targetindex;
        target = 0;
        targetindex = -1;

        if (R_FAILED(res)) {

            if (res != OS_REMOTE_SESSION_CLOSED) {
                ONERRSVCBREAK(0xd9875);
            } else if (index == -1) {
                if (lasttargetindex == -1) {
                    ONERRSVCBREAK(0xd9874);
                } else
                    index = lasttargetindex;
            }

            else if (index >= handlecount)
                ONERRSVCBREAK(-1);

            svcCloseHandle(sessionhandles[index]);
            handlecount--;

            for (s32 i = index - REMOTESESSIONINDEX; i < handlecount - REMOTESESSIONINDEX; i++) {
                sessionhandles[REMOTESESSIONINDEX + i] = sessionhandles[REMOTESESSIONINDEX + i + 1];
                serviceindexes[i] = serviceindexes[i + 1];
            }

            continue;
        }

        if (index == 0)
            HandleNotifications(&hid, &terminationflag);

        else if (index >= 1 && index < REMOTESESSIONINDEX) {
            Handle newsession = 0;
            ONERRSVCBREAK(svcAcceptSession(&newsession, sessionhandles[index]));

            if (handlecount >= INDEXMAX) {
                svcCloseHandle(newsession);
                continue;
            }

            sessionhandles[handlecount] = newsession;
            serviceindexes[handlecount - REMOTESESSIONINDEX] = index - 1;
            handlecount++;

        } else if (index >= REMOTESESSIONINDEX && index < INDEXMAX) {
            int32_t idx = serviceindexes[index - REMOTESESSIONINDEX];
            if (idx == 4)
                IPC::HandleMockIRUserCommands(&hid);
            else
                IPC::HandleHidCommands(&hid);

            target = sessionhandles[index];
            targetindex = index;

        } else {
            ONERRSVCBREAK(-4);
        }
    }

    for (int i = 1; i <= SERVICECOUNT; i++) {
        srvUnregisterService(srvnames[i]);
        svcCloseHandle(sessionhandles[i + 1]);
    }

    svcCloseHandle(sessionhandles[0]);
}
