#include <3ds.h>
#include <cstdio>
#include <cstring>
#include "IRUserMock.hpp"
#include "../crc8.hpp"

char buf[0x100];

volatile uint32_t diag_ir_input_requests = 0;
volatile uint32_t diag_ir_first_input_requests = 0;
volatile uint32_t diag_ir_subsequent_input_requests = 0;

#ifdef ENABLE_EXTERNAL_CCP
uint32_t diag_cpp_report_stick = 0;
uint32_t diag_cpp_report_zl = 0;
uint32_t diag_cpp_report_zr = 0;
uint32_t diag_cpp_report_last_stick = 0;
uint32_t diag_cpp_queue_rejections = 0;
uint64_t diag_cpp_batch_last_tick = 0;
uint64_t diag_cpp_batch_ticks[4] = {};
uint32_t diag_cpp_batch_next = 0;
#endif

static void LogCPPRequest(const uint8_t *data, size_t size) {
    Handle file;
    u32 written = 0;
    u64 offset = 0;

    static const char path[] = "/rehid_cpp_debug.txt";

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
        int len = snprintf(buf, sizeof(buf),
                           "SendIrnop size=%u data=",
                           (unsigned)size);

        for (size_t i = 0; i < size && len > 0 &&
            len < (int)sizeof(buf) - 5; i++) {
            len += snprintf(buf + len, sizeof(buf) - len,
                            "%02X ", data[i]);
            }

            len += snprintf(buf + len, sizeof(buf) - len, "\n");

        if (len > 0 && len < (int)sizeof(buf)) {
            FSFILE_Write(file, &written, offset, buf,
                         (u32)len, FS_WRITE_FLUSH);
        }
    }

    FSFILE_Close(file);
}

static void LogCPPText(const char *message) {
    Handle file;
    u32 written = 0;
    u64 offset = 0;

    static const char path[] = "/rehid_cpp_debug.txt";

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
        char line[0x100];
        int len = snprintf(line, sizeof(line), "%s\n", message);

        if (len > 0 && len < (int)sizeof(line)) {
            FSFILE_Write(file, &written, offset, line,
                         (u32)len, FS_WRITE_FLUSH);
        }
    }

    FSFILE_Close(file);
}

Result IRUserMock::InitializeIrNopShared(size_t sharedbufsize, size_t recvbufsize,
       size_t recvbufpacketcount, size_t sendbufsize, size_t sendbufpacketcount,
       uint8_t baudrate, Handle shmemhandle) {
    if (m_initialized) {
        return 0xD8210FF9;
    }

    Result ret = 0;

    m_shmemhandle = shmemhandle;

    m_shmemaddr = (uint32_t)mappableAlloc(sharedbufsize);
    // sprintf(buf, "m_sharedmemaddr: %08X, m_sharedmemend: %08X\n", m_shmemaddr, m_shmemaddr + sharedbufsize);
    // svcOutputDebugString(buf, strlen(buf));

    // sprintf(buf, "recvbufsize: %08X, recvbufpacketcount: %08X  \n", recvbufsize, recvbufpacketcount);
    // svcOutputDebugString(buf, strlen(buf));

    if (R_FAILED(ret = svcMapMemoryBlock(m_shmemhandle, m_shmemaddr, (MemPerm)(MEMPERM_READ | MEMPERM_WRITE), MEMPERM_READ))) {
        mappableFree((void *)m_shmemaddr);
        m_shmemaddr = 0;
        return ret;
    }

    LightLock_Lock(&m_lock);
    m_shmem = (CPPSharedMem *)m_shmemaddr;

    for (int i = 0; i < 3; i++) {
        svcCreateEvent(&m_handles[i], RESET_ONESHOT);
    }

    m_recvbufcount = recvbufpacketcount;
    m_recvbufsize = recvbufsize;

    /* skip to the end of the struct and to start of where actual packets live */
    m_start = ((uint8_t *)&m_shmem->packetinfos[m_recvbufcount]);
    m_end = (uint8_t *)m_shmemaddr + m_recvbufsize + sizeof(CPPBufferInfo) + sizeof(CPPSharedMemHeader);
    m_curr = m_start;

    m_shmem->header.recieveresult = 0;
    m_shmem->header.sendresult = 0;
    m_shmem->header.connectionstatus = IRUSER_ConnectionStatus::Disconnected;
    m_shmem->header.tryingtoconnectstatus = 0;
    m_shmem->header.initialized = 0;
    m_shmem->header.connectionrole = 0;
    m_shmem->header.connected = 0;
    m_shmem->header.networkid = 0xAA; // This should be randomly generated but we do not care

    m_shmem->bufferinfo.packetcount = 0;
    m_shmem->bufferinfo.beginindex = 0;
    m_shmem->bufferinfo.endindex = 0;

    m_connected = false;
    m_initialized = 1;
    LightLock_Unlock(&m_lock);
    return 0;
}

void IRUserMock::SignalReceiveIfPending() {
    LightLock_Lock(&m_lock);
    if (m_shmem &&
        m_shmem->header.connectionstatus == IRUSER_ConnectionStatus::Connected) {
#ifdef ENABLE_EXTERNAL_CCP
        bool publishNow = true;
#ifdef TEST_CPP_REPORT_PAD_SAMPLES
        publishNow = m_padSamplesUntilReport == 0;
        if (publishNow)
            m_padSamplesUntilReport = TEST_CPP_REPORT_PAD_SAMPLES - 1;
        else
            --m_padSamplesUntilReport;
#endif
        if (publishNow && m_pendingInputReports != 0) {
            CPPInputResponse response {};
            CreateInputDataPayload(&response);
            if (AddPayloadLocked(reinterpret_cast<uint8_t *>(&response), sizeof(response)))
                --m_pendingInputReports;
        }
#endif
        if (m_shmem->bufferinfo.packetcount != 0)
            svcSignalEvent(m_handles[RECV_EVENT_HANDLE_IDX]);
    }
    LightLock_Unlock(&m_lock);
}

Result IRUserMock::IrNopFinalize() {
    if (!m_initialized) {
        return 0xD8210FF8;
    }

    Result ret = 0;
    m_connected = false;

    LightLock_Lock(&m_lock);
#ifdef ENABLE_EXTERNAL_CCP
    m_pendingInputReports = 0;
#endif
    m_recvbufcount = 0;
    m_recvbufsize = 0;

    m_start = nullptr;
    m_end = nullptr;
    m_curr = nullptr;

    m_shmem->header.recieveresult = 0;
    m_shmem->header.sendresult = 0;
    m_shmem->header.connectionstatus = IRUSER_ConnectionStatus::Disconnected;
    m_shmem->header.tryingtoconnectstatus = 0;
    m_shmem->header.initialized = 0;
    m_shmem->header.connectionrole = 0;
    m_shmem->header.connected = 0;
    m_shmem->header.networkid = 0; // This should be randomly generated but we do not care
    m_shmem = nullptr;

    if (R_FAILED(ret = svcUnmapMemoryBlock(m_shmemhandle, m_shmemaddr))) {
        svcBreak(USERBREAK_ASSERT);
    }

    svcCloseHandle(m_shmemhandle);
    m_shmemhandle = 0;
    m_shmemaddr = 0;

    for (int i = 0; i < 3; i++) {
        svcCloseHandle(m_handles[i]);
    }

    LightLock_Unlock(&m_lock);

    m_initialized = 0;
    return 0;
}

Result IRUserMock::ClearReceiveBuffer() {
    LightLock_Lock(&m_lock);
#ifdef ENABLE_EXTERNAL_CCP
    m_pendingInputReports = 0;
#endif
    m_recvbufcount = 0;
    m_recvbufsize = 0;

    m_start = nullptr;
    m_end = nullptr;
    m_curr = nullptr;
    LightLock_Unlock(&m_lock);

    return 0;
}

Result IRUserMock::RequireConnection(uint8_t machineid) {
    if (machineid != 1)
        svcBreak(USERBREAK_ASSERT); // Trying to connect to a device other than cpp is not supported

    LightLock_Lock(&m_lock);
    if (m_shmem->header.connectionstatus == IRUSER_ConnectionStatus::Connected) {
        LightLock_Unlock(&m_lock);
        return 0xC8A10C0B; // We are already connected
    }

    m_shmem->header.machineid = machineid;

    // Mark network as connected, even when not
    m_shmem->header.connectionstatus = IRUSER_ConnectionStatus::Connected;
    m_shmem->header.connected = 1;
    m_shmem->header.initialized = 1;

    // signal that we got connected
    svcSignalEvent(m_handles[CONNECTION_EVENT_HANDLE_IDX]);
    LightLock_Unlock(&m_lock);
    return 0;
}

Result IRUserMock::SendIrnop(uint8_t *buff, size_t size) {
    if (m_shmem->header.connectionstatus != IRUSER_ConnectionStatus::Connected) {
        return 0xC8A10C0D;
    }

    // sprintf(buf, "SendIrnop Called type : %X\n", buff[0]);
    // svcOutputDebugString(buf, strlen(buf));

    if (!buff) {
        svcBreak(USERBREAK_ASSERT);
        return static_cast<Result>(-1);
    }

//    if (buff[0] != 1) {
//        LogCPPRequest(buff, size);
//    }

    switch (buff[0]) {
        case 3: {
            char diagnostic[0x100];

            snprintf(diagnostic, sizeof(diagnostic),
                     "Request 3: received size=%u",
                     (unsigned)size);
            //LogCPPText(diagnostic);

            if (size >= sizeof(CPPCalibrationRequest)) {
                CPPCalibrationRequest *request =
                reinterpret_cast<CPPCalibrationRequest *>(buff);

                snprintf(diagnostic, sizeof(diagnostic),
                         "Request 3: offset=%u requested_size=%u expected_size=%u",
                         request->offset, request->size,
                         (unsigned)(sizeof(CPPCalibrationRequest) + request->size));
                //LogCPPText(diagnostic);

                if (request->offset == 0 &&
                    request->size == sizeof(CPPCalibrationResponseData) &&
                    size == sizeof(CPPCalibrationRequest) + request->size) {

                    CPPCalibrationResponseData *candidate =
                    reinterpret_cast<CPPCalibrationResponseData *>(
                        buff + sizeof(CPPCalibrationRequest)
                    );

                uint8_t calculatedCRC = crc8(candidate, 15);

                snprintf(diagnostic, sizeof(diagnostic),
                         "Request 3: CRC received=%02X calculated=%02X",
                         candidate->crc8, calculatedCRC);
                //LogCPPText(diagnostic);

                if (calculatedCRC == candidate->crc8) {
                    CPPCalibrationData calibration {};
                    calibration.xoff = candidate->xoff;
                    calibration.yoff = candidate->yoff;
                    calibration.xscale = candidate->xscale;
                    calibration.yscale = candidate->yscale;

                    SetCalibrationData(&calibration);

                    snprintf(diagnostic, sizeof(diagnostic),
                             "Request 3: STORED xoff=%u yoff=%u xscale=%f yscale=%f",
                             calibration.xoff, calibration.yoff,
                             calibration.xscale, calibration.yscale);
                    //LogCPPText(diagnostic);
                } else {
                    //LogCPPText("Request 3: REJECTED - CRC mismatch");
                }
                    } else {
                        //LogCPPText("Request 3: REJECTED - offset/size mismatch");
                    }
            } else {
                //LogCPPText("Request 3: REJECTED - request header too short");
            }

            uint8_t response[2] = {0x12, 0x00};

            if (!AddPayload(response, sizeof(response)))
                return static_cast<Result>(-1);

            svcSignalEvent(m_handles[RECV_EVENT_HANDLE_IDX]);
            break;
        }

        case 4: {
            uint8_t response[2] = {0x13, 0x00};

            if (!AddPayload(response, sizeof(response)))
                return static_cast<Result>(-1);

            svcSignalEvent(m_handles[RECV_EVENT_HANDLE_IDX]);
            break;
        }

        case 2: {
            CPPCalibrationRequest *request = (CPPCalibrationRequest *)buff;
            CPPCalibrationResponse response {};

            if (request->size > sizeof(response.candidates))
                return static_cast<Result>(-1);

            CreateCalibrationPayload(request, &response);

            if (request->size == sizeof(CPPCalibrationResponseData)) {
                char diagnostic[0x100];
                int len = snprintf(diagnostic, sizeof(diagnostic),
                                   "Request 2 readback response:");

                const uint8_t *bytes =
                reinterpret_cast<const uint8_t *>(&response);

                for (size_t i = 0; i < 5 + request->size &&
                    len > 0 && len < (int)sizeof(diagnostic) - 4; i++) {
                    len += snprintf(diagnostic + len,
                                    sizeof(diagnostic) - len,
                                    " %02X", bytes[i]);
                    }

                    //LogCPPText(diagnostic);
            }

            size_t responseSize =
            sizeof(response.header) +
            sizeof(response.offset) +
            sizeof(response.size) +
            request->size;

            if (!AddPayload((uint8_t *)&response, responseSize))
                return static_cast<Result>(-1);

            svcSignalEvent(m_handles[RECV_EVENT_HANDLE_IDX]);
            break;
        }

        case 1: {
            CPPInputResponse response {};
            CreateInputDataPayload(&response);

#ifdef ENABLE_EXTERNAL_CCP
            LightLock_Lock(&m_lock);
            const bool hasSpace = m_shmem->bufferinfo.packetcount + m_pendingInputReports < m_recvbufcount;
            if (hasSpace)
                ++m_pendingInputReports;
            LightLock_Unlock(&m_lock);
            if (!hasSpace) {
                ++diag_cpp_queue_rejections;
                return static_cast<Result>(-1);
            }
#else
            if (!AddPayload(reinterpret_cast<uint8_t *>(&response), sizeof(response)))
                return static_cast<Result>(-1);
#endif

#ifdef ENABLE_EXTERNAL_CCP
            if (response.x != 0x800 || response.y != 0x800) {
                ++diag_cpp_report_stick;
                diag_cpp_report_last_stick =
                    (static_cast<uint32_t>(response.x) << 16) | response.y;
            }
            if (!response.zlup)
                ++diag_cpp_report_zl;
            if (!response.zrup)
                ++diag_cpp_report_zr;
#endif

            diag_ir_input_requests++;

#ifdef ENABLE_EXTERNAL_CCP
            if (diag_ir_input_requests == 1 || (diag_ir_input_requests & 255) == 0) {
                const uint64_t tick = svcGetSystemTick();
                if (diag_cpp_batch_last_tick != 0 && diag_ir_input_requests > 256) {
                    diag_cpp_batch_ticks[diag_cpp_batch_next] = tick - diag_cpp_batch_last_tick;
                    diag_cpp_batch_next = (diag_cpp_batch_next + 1) & 3;
                }
                diag_cpp_batch_last_tick = tick;
            }
#endif

            if (m_connected)
                diag_ir_subsequent_input_requests++;
            else
                diag_ir_first_input_requests++;

            m_connected = true;
            break;
        }
    }

    // Notify that the outgoing IRNOP request was processed.
    svcSignalEvent(m_handles[SEND_EVENT_HANDLE_IDX]);
    return 0;
}

Result IRUserMock::Disconnect() {
    LightLock_Lock(&m_lock);
#ifdef ENABLE_EXTERNAL_CCP
    m_pendingInputReports = 0;
#endif
    if (m_shmem->header.connectionstatus == IRUSER_ConnectionStatus::Connected) {
        m_shmem->header.connectionstatus = IRUSER_ConnectionStatus::Disconnected;
        m_shmem->header.connected = 0;
        m_shmem->header.initialized = 0;
        m_connected = false;

        // Signal that we got disconnected
        svcSignalEvent(m_handles[CONNECTION_EVENT_HANDLE_IDX]);
    } else {
        LightLock_Unlock(&m_lock);
        return 0xC8A10C0D;
    }

    LightLock_Unlock(&m_lock);
    return 0;
}

void IRUserMock::CreateCalibrationPayload(CPPCalibrationRequest *request, CPPCalibrationResponse *response) {

    CPPCalibrationData calibration;

    LightLock_Lock(&m_lock);
    calibration = m_calibration;
    LightLock_Unlock(&m_lock);

    response->header = 0x11;

    /* as per 3dbrew, this remains the same */
    response->offset = request->offset;
    response->size = request->size;

    /* Write the default calibration for now, when possible this should come from cpp */
    for (int i = 0; i < 4; i++) {
        CPPCalibrationResponseData *candidate = &response->candidates[i];

        candidate->pad1 = 0;
        candidate->pad2[0] = 0xFF;
        candidate->pad2[1] = 0xFF;
        candidate->pad2[2] = 0xFF;

        candidate->xoff = calibration.xoff;
        candidate->yoff = calibration.yoff;
        candidate->xscale = calibration.xscale;
        candidate->yscale = calibration.yscale;
        candidate->crc8 = crc8(candidate, 15);
    }

}

void IRUserMock::CreateInputDataPayload(CPPInputResponse *response) {
    CirclePadEntry latestdata;
    uint32_t latestbuttons;

    LightLock_Lock(&m_inputlock);
    latestdata = m_latestdata;
    latestbuttons = m_latestbuttons;
    LightLock_Unlock(&m_inputlock);

    response->header = 0x10;
    response->x = latestdata.x;
    response->y = latestdata.y;

    response->batterylevel = 0x1F; // max battery

    // CPP report button fields are active-low.
    response->zlup = !(latestbuttons & KEY_ZL);
    response->zrup = !(latestbuttons & KEY_ZR);
    response->rup = !(latestbuttons & KEY_R);
}

bool IRUserMock::AddPayload(uint8_t *buffer, size_t length) {
    LightLock_Lock(&m_lock);
    const bool added = AddPayloadLocked(buffer, length);
    LightLock_Unlock(&m_lock);
    return added;
}

bool IRUserMock::AddPayloadLocked(uint8_t *buffer, size_t length) {
    if (m_shmem->bufferinfo.packetcount >= m_recvbufcount)
        return false;

    /* Calculate the packet header for the payload */
    uint8_t header[4] = {0};
    header[0] = 0xA5;
    header[1] = 0x1;

    /* max possible length is 0xFFFF */
    uint8_t headersize = 0;
    if (length < 0x40) {
        header[2] = length;
        headersize = 3;
    } else {
        header[2] = (1 << 6) | ((length & 0xFF00) >> 8); // higher byte of total length
        header[3] = (length & 0xFF); // lower byte of total length
        headersize = 4;
    }

    auto write = [&](uint8_t *buf2, size_t size) {
        for (size_t i = 0; i < size; i++) {
            if (m_curr >= m_end) {
                m_curr = m_start;
            }

            // sprintf(buf, "Writing %02X to %08X | Remaining size: %d\n", buf2[i], m_curr, size - i - 1);
            // svcOutputDebugString(buf, strlen(buf));

            *m_curr = buf2[i];
            m_curr++;
        }
    };

    /* Normalize the write pointer before recording its ring offset. */
    if (m_curr >= m_end) {
        m_curr = m_start;
    }
    uint32_t start = static_cast<uint32_t>(m_curr - m_start);
    uint32_t size = 0;

    // sprintf(buf, "m_curr: %08X, m_start: %08X, offset: %08X, m_end: %08X\n", m_curr, m_start, start, m_end);
    // svcOutputDebugString(buf, strlen(buf));

    /* first write the header */
    write(header, headersize);
    size += headersize;

    /* Now write the payload */
    write(buffer, length);
    size += length;

    /* Finally the crc */
    uint8_t crc = crc8_loop(m_start, m_end - m_start, m_start + start, size);
    write(&crc, 1);
    size += 1;

    CPPPacketInfo *info = &m_shmem->packetinfos[m_shmem->bufferinfo.endindex];
    info->offset = start;
    info->size = size;

    m_shmem->bufferinfo.endindex = (m_shmem->bufferinfo.endindex + 1) % m_recvbufcount;
    m_shmem->bufferinfo.packetcount++;
    return true;
}

Result IRUserMock::ReleaseRecievedData(uint32_t count) {
    Result res = 0;

    // sprintf(buf, "ReleaseRecievedData called count : %d\n", count);
    // svcOutputDebugString(buf, strlen(buf));

    if (m_shmem->header.connectionstatus == IRUSER_ConnectionStatus::Connected) {
        LightLock_Lock(&m_lock);
        if (m_shmem->bufferinfo.packetcount) {
            m_shmem->bufferinfo.beginindex = (m_shmem->bufferinfo.beginindex + count) % m_recvbufcount;
            m_shmem->bufferinfo.packetcount -= count;
        } else {
            res = 0xC8810FEF; // There is no data to release
        }
        LightLock_Unlock(&m_lock);
    } else {
        res = 0xC8A10C0D; // Can't use in the current state
    }

    return res;
}

void IRUserMock::SetCalibrationData(CPPCalibrationData *data) {
    LightLock_Lock(&m_lock);
    m_calibration = *data;
    LightLock_Unlock(&m_lock);
}
