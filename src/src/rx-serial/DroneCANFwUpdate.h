#pragma once

#include "targets.h"

#if defined(TARGET_RX) && defined(HAS_DRONECAN_FW_UPDATE)

#include "canard.h"
#include "CANTypes.h"
#include "dsdl/uavcan.protocol.file.BeginFirmwareUpdate.h"
#include "dsdl/uavcan.protocol.file.Read.h"

// Firmware update over DroneCAN (RP2): the image is fetched with file.Read from the file server named in
// BeginFirmwareUpdate and written straight over the application in flash, safe as the app runs from RAM.
// Sector 0 holds the boot block, it is erased first and written last, so an interrupted update boots to BOOTSEL.
// The receiver reboots as soon as the image is complete and verified
class DroneCANFwUpdate
{
public:
    // Returns a BeginFirmwareUpdate response error, ERROR_OK when the update has started.
    // Refused while connected to a TX, the only time the link can commit config to flash
    uint8_t begin(uint8_t serverNode, const uavcan_protocol_file_Path &path, bool connected, uint32_t nowMs);
    // Sends file.Read requests and handles their timeouts, call from every service()
    void service(CanardInstance *canard, uint32_t nowMs, bool canfd);
    void onReadResponse(const CanardRxTransfer *transfer, const uavcan_protocol_file_ReadResponse &resp, uint32_t nowMs);

    bool active() const { return state == FETCHING; }
    // Flash no longer holds a bootable image, it comes back only with a completed update
    bool invalidated() const { return flashInvalidated; }
    uint32_t progressKB() const { return offset / 1024; }
    // Why the last update stopped, nullptr if it did not fail; cleared once taken
    const char *takeError() { const char *why = failReason; failReason = nullptr; return why; }

private:
    enum State : uint8_t { IDLE, FETCHING };

    void fail(const char *why);
    void sendRead(CanardInstance *canard, uint32_t nowMs, bool canfd);
    bool acceptData(const uint8_t *data, uint32_t len);
    bool finish();

    State state = IDLE;
    bool flashInvalidated = false;
    const char *failReason = nullptr;
    uint8_t server = 0;
    uavcan_protocol_file_Path path {};

    uint32_t offset = 0;            // bytes of the image received
    bool pending = false;
    uint8_t transferId = 0;         // libcanard advances it once a request is queued
    uint8_t pendingTransferId = 0;
    uint32_t sentMs = 0;
    uint32_t progressMs = 0;        // last accepted response, or the start

    uint8_t requestBuf[UAVCAN_PROTOCOL_FILE_READ_REQUEST_MAX_SIZE];
};

#endif
