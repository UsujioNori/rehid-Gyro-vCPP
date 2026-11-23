#pragma once
#include <3ds.h>
#include "../CPPDefs.hpp"
#include "../CirclePad.hpp"

static constexpr uint8_t CONNECTION_EVENT_HANDLE_IDX = 0;
static constexpr uint8_t RECV_EVENT_HANDLE_IDX = 1;
static constexpr uint8_t SEND_EVENT_HANDLE_IDX = 2;

class IRUserMock {

public:
    IRUserMock() {
        LightLock_Init(&m_inputlock);
        LightLock_Init(&m_lock);
    }

    Result InitializeIrNopShared(size_t sharedbufsize, size_t recvbufsize,
       size_t recvbufpacketcount, size_t sendbufsize, size_t sendbufpacketcount,
       uint8_t baudrate, Handle shmemhandle);

    Result IrNopFinalize();
    Result ClearReceiveBuffer();

    Result RequireConnection(uint8_t machineid);
    Result Disconnect();

    uint32_t GetReceivePacketCount() {
        LightLock_Lock(&m_lock);
        uint32_t count = m_shmem->bufferinfo.packetcount;
        LightLock_Unlock(&m_lock);
        return count;
    }

    Result SendIrnop(uint8_t *buf, size_t size);
    void SignalReceiveIfPending();

    Result GetConnectionStatusEvent(Handle *event) {
        *event = m_handles[CONNECTION_EVENT_HANDLE_IDX];
        return 0;
    }

    Result GetRecvEvent(Handle *event) {
        *event = m_handles[RECV_EVENT_HANDLE_IDX];
        return 0;
    }

    Result GetSendEvent(Handle *event) {
        *event = m_handles[SEND_EVENT_HANDLE_IDX];
        return 0;
    }

    Result ReleaseRecievedData(uint32_t count);

    void CreateCalibrationPayload(CPPCalibrationRequest *request, CPPCalibrationResponse *response);
    void CreateInputDataPayload(CPPInputResponse *response);
    bool AddPayload(uint8_t *buffer, size_t length);

    void SetLatestInputData(CirclePadEntry *entry, uint32_t buttons) {
        LightLock_Lock(&m_inputlock);
        m_latestdata = *entry;
        m_latestbuttons = buttons;
        LightLock_Unlock(&m_inputlock);
    }

    bool IsConnected() {
        return m_connected;
    }

    void DoSleep(uint8_t sleep) {
        if (sleep) {
            LightLock_Lock(&m_inputlock);
            m_latestdata.x = m_calibration.xoff;
            m_latestdata.y = m_calibration.yoff;
            m_latestbuttons = 0;
            LightLock_Unlock(&m_inputlock);
        }
    }

    void SetCalibrationData(CPPCalibrationData *calibdata);

private:
    bool AddPayloadLocked(uint8_t *buffer, size_t length);

#ifdef ENABLE_EXTERNAL_CCP
    uint32_t m_pendingInputReports = 0;
#ifdef TEST_CPP_REPORT_PAD_SAMPLES
    unsigned int m_padSamplesUntilReport = 0;
#endif
#endif

    size_t m_shmemsize = 0;
    size_t m_recvbufsize = 0;
    size_t m_recvbufcount = 0;
    size_t m_sendbufsize = 0;
    size_t m_sendbufcount = 0;
    size_t m_baudrate = 0;

    Handle m_handles[3] {0};

    CPPCalibrationData m_calibration {0x800, 0x800, 1.0f, 1.0f};
    CirclePadEntry m_latestdata {0, 0};
    uint32_t m_latestbuttons = 0;
    LightLock m_inputlock;

    Handle m_shmemhandle;
    uint32_t m_shmemaddr;

    uint8_t m_initialized = 0;

    #ifdef ENABLE_EXTERNAL_CCP
    bool m_connected = false;
    #else
    bool m_connected = true;
    #endif

    uint8_t *m_curr = nullptr;
    uint8_t *m_end = nullptr;
    uint8_t *m_start = nullptr;

    LightLock m_lock;

    CPPSharedMem *m_shmem = nullptr;
};
