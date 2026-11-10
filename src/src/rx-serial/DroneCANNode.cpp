#include "DroneCANNode.h"

#if defined(TARGET_RX) && defined(HAS_DRONECAN)

#include "common.h"
#include "helpers.h"
#include "options.h"
#include "logging.h"

#if defined(PLATFORM_ESP32)
#include "TWAIDriver.h"
#include "esp_mac.h"
#include "esp_timer.h"
using CANDriver = TWAIDriver;
#elif defined(PLATFORM_RP2)
#include <pico/time.h>
#include <pico/unique_id.h>
#if defined(CAN_MCP251XFD)
#include "MCP251XFDDriver.h"
using CANDriver = MCP251XFDDriver;
#else
#include "CAN2040Driver.h"
using CANDriver = CAN2040Driver;
#endif
#endif

// ArduPilot's MAVLink-over-CAN needs a fixed node id (CAN_Dx_UC_S1_NOD)
#define DRONECAN_NODE_ID            68
#define DRONECAN_TUNNEL_KEEPALIVE_MS 500
#define DRONECAN_TUNNEL_MAX_PAYLOAD 120
#define DRONECAN_MAX_RX_PER_CALL    32
// With no node acknowledging, frames retry forever and RC would reach the FC late once it returns
#define DRONECAN_TX_STALL_MS        50

// Acceptance filter mask: everything but priority (28..24) and source node (6..0)
#define CAN_ID_MASK_TYPE_DEST       0x00FFFF80
// Destination and service bit only
#define CAN_ID_MASK_SERVICE_DEST    0x00007F80

static_assert(DRONECAN_BUF_SIZE >= UAVCAN_TUNNEL_TARGETTED_MAX_SIZE, "DRONECAN_BUF_SIZE too small");
static_assert(DRONECAN_BUF_SIZE >= DRONECAN_SENSORS_RC_RCINPUT_MAX_SIZE, "DRONECAN_BUF_SIZE too small");
static_assert(DRONECAN_BUF_SIZE >= UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE, "DRONECAN_BUF_SIZE too small");
#if defined(HAS_DRONECAN_FW_UPDATE)
static_assert(DRONECAN_BUF_SIZE >= UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_MAX_SIZE, "DRONECAN_BUF_SIZE too small");
#endif

static uint64_t micros64()
{
#if defined(PLATFORM_ESP32)
    return esp_timer_get_time();
#else
    return time_us_64();
#endif
}

static bool shouldAcceptTransfer(const CanardInstance *ins, uint64_t *outSignature, uint16_t dataTypeId,
                                 CanardTransferType transferType, uint8_t sourceNodeId)
{
    return ((DroneCANNode *)canardGetUserReference(ins))->shouldAcceptTransfer(outSignature, dataTypeId, transferType);
}

static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    ((DroneCANNode *)canardGetUserReference(ins))->onTransferReceived(ins, transfer);
}

DroneCANNode::DroneCANNode(int8_t txPin, int8_t rxPin) : txPin(txPin), rxPin(rxPin)
{
#if defined(PLATFORM_RP2)
    critical_section_init(&crit);
#endif
    canardInit(&canard, canardPool, sizeof(canardPool), ::onTransferReceived, ::shouldAcceptTransfer, this);
    canardSetLocalNodeID(&canard, DRONECAN_NODE_ID);
}

DroneCANNode::~DroneCANNode()
{
#if defined(PLATFORM_RP2)
    critical_section_deinit(&crit);
#endif
}

void DroneCANNode::lock()
{
#if defined(PLATFORM_RP2)
    critical_section_enter_blocking(&crit);
#endif
}

void DroneCANNode::unlock()
{
#if defined(PLATFORM_RP2)
    critical_section_exit(&crit);
#endif
}

void DroneCANNode::start()
{
    if (txPin == UNDEF_PIN || rxPin == UNDEF_PIN)
        return;

    // Only accept tunnel.Targetted broadcasts (DTID, service bit clear) and GetNodeInfo
    // requests to us (service type, request, destination, service bit set)
    const CANFilter filter {
        UAVCAN_TUNNEL_TARGETTED_ID << 8, CAN_ID_MASK_TYPE_DEST,
#if defined(HAS_DRONECAN_FW_UPDATE)
        // Any service frame to us: also BeginFirmwareUpdate requests and file.Read responses
        (DRONECAN_NODE_ID << 8) | (1 << 7), CAN_ID_MASK_SERVICE_DEST,
#else
        (UAVCAN_PROTOCOL_GETNODEINFO_ID << 16) | (1 << 15) | (DRONECAN_NODE_ID << 8) | (1 << 7), CAN_ID_MASK_TYPE_DEST,
#endif
    };
    if (CANDriver::begin(txPin, rxPin, filter))
        started.store(true, std::memory_order_release);
}

void DroneCANNode::stop()
{
    if (started.load(std::memory_order_relaxed))
        CANDriver::end();
    started.store(false, std::memory_order_release);
}

void DroneCANNode::postRC(const dronecan_sensors_rc_RCInput &rc)
{
    lock();
    rcPosted = rc;
    rcPostedSeq++;
    unlock();
}

bool DroneCANNode::takeStats(DroneCANNodeStats &out)
{
    lock();
    const bool fresh = publishedFresh;
    out = published;
    publishedFresh = false;
    unlock();
    return fresh;
}

void DroneCANNode::service()
{
    if (!started.load(std::memory_order_relaxed))
        return;

    CANDriver::poll();
    receiveFrames();
    sendRC();

    const uint32_t now = millis();
    if (now - lastNodeStatusMs >= 1000)
    {
        lastNodeStatusMs = now;
        canardCleanupStaleTransfers(&canard, micros64());
        // Sent even when inactive, an acknowledgement is how another node is detected
        sendNodeStatus();
        publishStats();
    }

#if defined(HAS_DRONECAN_FW_UPDATE)
    fwUpdate.service(&canard, now, peerFd);
#endif

    if (serverNodeId() == 0 || !busActive())
    {
        // Nowhere to send until the FC has opened the tunnel and is acknowledging
        rxToFc.discard();
        lastTunnelMs = now;
    }
    else if (rxToFc.size() > 0 || now - lastTunnelMs > DRONECAN_TUNNEL_KEEPALIVE_MS)
    {
        lastTunnelMs = now;
        sendTunnel();
    }

    processTxQueue();
}

void DroneCANNode::publishStats()
{
    stats.driver = CANDriver::takeStats();
    stats.state = CANDriver::state();
    stats.tec = CANDriver::txErrorCounter();
    stats.rec = CANDriver::rxErrorCounter();
#if defined(HAS_DRONECAN_FW_UPDATE)
    stats.fwKB = fwUpdate.active() ? fwUpdate.progressKB() : 0;
    stats.fwError = fwUpdate.takeError();
#endif
    lock();
    published = stats;
    publishedFresh = true;
    unlock();
    stats = {};
}

void DroneCANNode::sendRC()
{
    lock();
    const bool fresh = rcPostedSeq != rcSentSeq;
    if (fresh)
    {
        scratch.rcInput = rcPosted;
        rcSentSeq = rcPostedSeq;
    }
    unlock();
    if (!fresh)
        return;

    const uint32_t len = dronecan_sensors_rc_RCInput_encode(&scratch.rcInput, encodeBuf TAO_ARG(peerFd));
    canardBroadcast(&canard, DRONECAN_SENSORS_RC_RCINPUT_SIGNATURE, DRONECAN_SENSORS_RC_RCINPUT_ID,
                    &rcInputTransferId, CANARD_TRANSFER_PRIORITY_HIGH, encodeBuf, len CANFD_ARG(peerFd));
    stats.rcSent++;
}

void DroneCANNode::receiveFrames()
{
    CANFrame msg;
    if (!CANDriver::receive(msg))
        return;
    active.store(true, std::memory_order_relaxed);

    // One timestamp per batch, libcanard only uses it for multi-frame transfer timeouts
    const uint64_t nowUs = micros64();
    unsigned count = 0;
    do
    {
        CanardCANFrame frame {};
        frame.id = msg.id | CANARD_CAN_FRAME_EFF;
        frame.data_len = msg.dlc;
        memcpy(frame.data, msg.data, msg.dlc);
#if CANARD_ENABLE_CANFD
        frame.canfd = msg.fd;
#endif
        // Reassembly failures (toggle, TID, CRC) mean frames were lost; a missed start is normal when joining mid-transfer
        const int16_t res = canardHandleRxFrame(&canard, &frame, nowUs);
        if (res <= -CANARD_ERROR_RX_WRONG_TOGGLE || res == -CANARD_ERROR_OUT_OF_MEMORY)
            stats.rxTransferErrors++;
    } while (++count < DRONECAN_MAX_RX_PER_CALL && CANDriver::receive(msg));
    stats.rxFrames += count;
}

void DroneCANNode::processTxQueue()
{
    const uint32_t now = millis();
    const uint32_t completed = CANDriver::txCompletedCount();
    if (completed != lastTxCompleted)
    {
        lastTxCompleted = completed;
        lastTxProgressMs = now;
        active.store(true, std::memory_order_relaxed);
    }
    else if (!CANDriver::txQueued())
    {
        lastTxProgressMs = now;
    }
    else if (now - lastTxProgressMs > DRONECAN_TX_STALL_MS)
    {
        // Nobody is acknowledging, drop everything queued rather than deliver it late
        lastTxProgressMs = now;
        active.store(false, std::memory_order_relaxed);
        stats.txStalls++;
        while (canardPeekTxQueue(&canard) != nullptr)
            canardPopTxQueue(&canard);
        CANDriver::flushTx();
        return;
    }

    const CanardCANFrame *frame;
    while ((frame = canardPeekTxQueue(&canard)) != nullptr)
    {
        CANFrame msg;
        msg.id = frame->id & CANARD_CAN_EXT_ID_MASK;
        msg.dlc = frame->data_len;
        memcpy(msg.data, frame->data, frame->data_len);
#if CANARD_ENABLE_CANFD
        msg.fd = frame->canfd;
#endif

        if (!CANDriver::transmit(msg))
            break;  // driver queue full, retry on the next call
        canardPopTxQueue(&canard);
        stats.txFrames++;
    }
}

void DroneCANNode::fillNodeStatus(uavcan_protocol_NodeStatus &status)
{
    status.uptime_sec = millis() / 1000;
    status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
#if defined(HAS_DRONECAN_FW_UPDATE)
    if (fwUpdate.active())
    {
        status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE;
        status.vendor_specific_status_code = fwUpdate.progressKB();
    }
    // Running from RAM with no bootable image in flash, the next power cycle lands in BOOTSEL
    if (fwUpdate.invalidated())
        status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_CRITICAL;
#endif
}

void DroneCANNode::sendNodeStatus()
{
    uavcan_protocol_NodeStatus status {};
    fillNodeStatus(status);

    const uint32_t len = uavcan_protocol_NodeStatus_encode(&status, encodeBuf TAO_ARG(peerFd));
    canardBroadcast(&canard, UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE, UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &nodeStatusTransferId, CANARD_TRANSFER_PRIORITY_LOW, encodeBuf, len CANFD_ARG(peerFd));
}

void DroneCANNode::handleGetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer)
{
    uavcan_protocol_GetNodeInfoResponse &info = scratch.nodeInfo;
    memset(&info, 0, sizeof(info));

    fillNodeStatus(info.status);

    // strtoul rather than sscanf, which links ~25KB of newlib scanf/strtod
    char *end;
    info.software_version.major = strtoul(version, &end, 10);
    info.software_version.minor = *end == '.' ? strtoul(end + 1, nullptr, 10) : 0;
    info.software_version.vcs_commit = strtoul(commit, nullptr, 16);
    info.software_version.optional_field_flags = UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_VCS_COMMIT;

#if defined(PLATFORM_ESP32)
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    memcpy(info.hardware_version.unique_id, mac, sizeof(mac));
#else
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    memcpy(info.hardware_version.unique_id, id.id, sizeof(id.id));
#endif

    // e.g. org.expresslrs.radiomaster_xr1_dual_band_rx, node names are lowercase [a-z0-9._]
    const int nameLen = snprintf((char *)info.name.data, sizeof(info.name.data), "org.expresslrs.%s", product_name[0] ? product_name : "rx");
    info.name.len = std::min<int>(nameLen, sizeof(info.name.data) - 1);
    for (uint8_t i = sizeof("org.expresslrs.") - 1; i < info.name.len; ++i)
    {
        const char c = tolower(info.name.data[i]);
        info.name.data[i] = isalnum(c) ? c : '_';
    }

    stats.nodeInfoRequests++;
    const uint32_t len = uavcan_protocol_GetNodeInfoResponse_encode(&info, encodeBuf TAO_ARG(transfer->canfd));
    canardRequestOrRespond(ins, transfer->source_node_id, UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID, &transfer->transfer_id, transfer->priority,
                           CanardResponse, encodeBuf, len CANFD_ARG(transfer->canfd));
}

void DroneCANNode::handleTunnel(CanardRxTransfer *transfer)
{
    uavcan_tunnel_Targetted &tunnel = scratch.tunnel;
    if (uavcan_tunnel_Targetted_decode(transfer, &tunnel))
        return;
    if (transfer->source_node_id == 0 || tunnel.target_node != DRONECAN_NODE_ID || tunnel.serial_id != 0)
        return;
    // ArduPilot <= 4.5 leaves protocol undefined
    if (tunnel.protocol.protocol != UAVCAN_TUNNEL_PROTOCOL_UNDEFINED &&
        tunnel.protocol.protocol != UAVCAN_TUNNEL_PROTOCOL_MAVLINK2)
        return;

    serverNode.store(transfer->source_node_id, std::memory_order_relaxed);
    if (tunnel.buffer.len == 0)
        return;
    if (fcToRx.free() < tunnel.buffer.len)
    {
        stats.tunnelInDropped += tunnel.buffer.len;
        return;
    }
    fcToRx.push(tunnel.buffer.data, tunnel.buffer.len);
    stats.tunnelInBytes += tunnel.buffer.len;
}

void DroneCANNode::sendTunnel()
{
    uavcan_tunnel_Targetted &tunnel = scratch.tunnel;
    tunnel.protocol.protocol = UAVCAN_TUNNEL_PROTOCOL_MAVLINK2;
    tunnel.target_node = serverNodeId();
    tunnel.serial_id = 0;
    tunnel.options = UAVCAN_TUNNEL_TARGETTED_OPTION_LOCK_PORT;
    tunnel.baudrate = 0;  // ignored by ArduPilot
    tunnel.buffer.len = std::min<uint32_t>(rxToFc.size(), DRONECAN_TUNNEL_MAX_PAYLOAD);
    rxToFc.pop(tunnel.buffer.data, tunnel.buffer.len);

    const uint32_t len = uavcan_tunnel_Targetted_encode(&tunnel, encodeBuf TAO_ARG(peerFd));
    canardBroadcast(&canard, UAVCAN_TUNNEL_TARGETTED_SIGNATURE, UAVCAN_TUNNEL_TARGETTED_ID,
                    &tunnelTransferId, CANARD_TRANSFER_PRIORITY_MEDIUM, encodeBuf, len CANFD_ARG(peerFd));
}

bool DroneCANNode::shouldAcceptTransfer(uint64_t *outSignature, uint16_t dataTypeId, CanardTransferType transferType)
{
    if (transferType == CanardTransferTypeRequest && dataTypeId == UAVCAN_PROTOCOL_GETNODEINFO_ID)
    {
        *outSignature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
        return true;
    }
    if (transferType == CanardTransferTypeBroadcast && dataTypeId == UAVCAN_TUNNEL_TARGETTED_ID)
    {
        *outSignature = UAVCAN_TUNNEL_TARGETTED_SIGNATURE;
        return true;
    }
#if defined(HAS_DRONECAN_FW_UPDATE)
    if (transferType == CanardTransferTypeRequest && dataTypeId == UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID)
    {
        *outSignature = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE;
        return true;
    }
    if (transferType == CanardTransferTypeResponse && dataTypeId == UAVCAN_PROTOCOL_FILE_READ_ID)
    {
        *outSignature = UAVCAN_PROTOCOL_FILE_READ_SIGNATURE;
        return true;
    }
#endif
    return false;
}

void DroneCANNode::onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
#if CANARD_ENABLE_CANFD
    // Only the FC's (or file server's) transfers pass the filter: send in whichever format it last used
    peerFd = transfer->canfd;
#endif
    if (transfer->transfer_type == CanardTransferTypeRequest && transfer->data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID)
    {
        handleGetNodeInfo(ins, transfer);
    }
    else if (transfer->transfer_type == CanardTransferTypeBroadcast && transfer->data_type_id == UAVCAN_TUNNEL_TARGETTED_ID)
    {
        handleTunnel(transfer);
    }
#if defined(HAS_DRONECAN_FW_UPDATE)
    else if (transfer->transfer_type == CanardTransferTypeRequest && transfer->data_type_id == UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID)
    {
        handleBeginFirmwareUpdate(ins, transfer);
    }
    else if (transfer->transfer_type == CanardTransferTypeResponse && transfer->data_type_id == UAVCAN_PROTOCOL_FILE_READ_ID)
    {
        if (!uavcan_protocol_file_ReadResponse_decode(transfer, &scratch.fileRead))
            fwUpdate.onReadResponse(transfer, scratch.fileRead, millis());
    }
#endif
}

#if defined(HAS_DRONECAN_FW_UPDATE)
void DroneCANNode::handleBeginFirmwareUpdate(CanardInstance *ins, CanardRxTransfer *transfer)
{
    uavcan_protocol_file_BeginFirmwareUpdateRequest &req = scratch.beginFwUpdate;
    if (uavcan_protocol_file_BeginFirmwareUpdateRequest_decode(transfer, &req))
        return;

    // A source node id of 0 means the caller is the file server
    const uint8_t fileServer = req.source_node_id ? req.source_node_id : transfer->source_node_id;
    uavcan_protocol_file_BeginFirmwareUpdateResponse resp {};
    resp.error = fwUpdate.begin(fileServer, req.image_file_remote_path, connectionState == connected, millis());

    const uint32_t len = uavcan_protocol_file_BeginFirmwareUpdateResponse_encode(&resp, encodeBuf TAO_ARG(transfer->canfd));
    canardRequestOrRespond(ins, transfer->source_node_id, UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE,
                           UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID, &transfer->transfer_id, transfer->priority,
                           CanardResponse, encodeBuf, len CANFD_ARG(transfer->canfd));
}
#endif

#endif
