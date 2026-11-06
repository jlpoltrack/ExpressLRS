#pragma once

#include "targets.h"

#if defined(TARGET_RX) && defined(HAS_DRONECAN)

#include <atomic>

#include "CANTypes.h"
#include "SPSCRing.h"
#include "canard.h"
#include "dsdl/dronecan.sensors.rc.RCInput.h"
#include "dsdl/uavcan.protocol.GetNodeInfo.h"
#include "dsdl/uavcan.tunnel.Targetted.h"
#if defined(PLATFORM_RP2)
#include "rp2_core1.h"
#include <pico/critical_section.h>
#endif

#define DRONECAN_TUNNEL_FIFO_LEN    1024
#define DRONECAN_BUF_SIZE           UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE

// One second of counters, published by the node for the main loop to log
struct DroneCANNodeStats {
    uint32_t rxFrames, txFrames, rcSent, tunnelInDropped, txStalls, tunnelInBytes, rxTransferErrors, nodeInfoRequests;
    CANStats driver;
    CANState state;
    uint8_t tec, rec;
};

// The DroneCAN node: libcanard and the CAN driver, run on core 1 (RP2) or from the main loop (ESP32).
// The main loop uses only the tunnel rings, postRC() and the getters, which are safe from the other core
class DroneCANNode
#if defined(PLATFORM_RP2)
    final : public RP2Core1Task
#endif
{
public:
    DroneCANNode(int8_t txPin, int8_t rxPin);
    ~DroneCANNode();

    // CAN context: start/stop the driver, and do all the CAN work
    void start();
    void stop();
    void service();

    // Main loop side. RC is latest wins, sent once by the next service()
    void postRC(const dronecan_sensors_rc_RCInput &rc);
    bool driverStarted() const { return started.load(std::memory_order_acquire); }
    // Another node is acknowledging our frames, or has been heard
    bool busActive() const { return active.load(std::memory_order_relaxed); }
    // Node id of the FC, learnt from its first tunnel message, 0 until then
    uint8_t serverNodeId() const { return serverNode.load(std::memory_order_relaxed); }
    // Returns true with the last second's counters, once per second
    bool takeStats(DroneCANNodeStats &out);

    // Tunnel bytes from the FC (node produces), and to the FC (main loop produces)
    SPSCRing<DRONECAN_TUNNEL_FIFO_LEN> fcToRx;
    SPSCRing<DRONECAN_TUNNEL_FIFO_LEN> rxToFc;

    // libcanard callbacks
    bool shouldAcceptTransfer(uint64_t *outSignature, uint16_t dataTypeId, CanardTransferType transferType);
    void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer);

private:
    void lock();
    void unlock();
    void receiveFrames();
    void processTxQueue();
    void sendRC();
    void sendNodeStatus();
    void sendTunnel();
    void handleGetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer);
    void handleTunnel(CanardRxTransfer *transfer);
    void publishStats();

    const int8_t txPin, rxPin;
    std::atomic<bool> started {false};
    std::atomic<bool> active {false};
    std::atomic<uint8_t> serverNode {0};

#if defined(PLATFORM_RP2)
    // Guards rcPosted and published, which are copied across cores
    critical_section_t crit;
#endif
    dronecan_sensors_rc_RCInput rcPosted {};
    uint32_t rcPostedSeq = 0;
    uint32_t rcSentSeq = 0;
    DroneCANNodeStats published {};
    bool publishedFresh = false;

    // Everything below is only touched from the CAN context
    CanardInstance canard {};
    uint8_t canardPool[4096] __attribute__((aligned(8)));
    uint8_t encodeBuf[DRONECAN_BUF_SIZE];
    // Decode/encode scratch, handlers run one at a time from service()
    union {
        uavcan_protocol_GetNodeInfoResponse nodeInfo;
        uavcan_tunnel_Targetted tunnel;
        dronecan_sensors_rc_RCInput rcInput;
    } scratch;

    uint32_t lastNodeStatusMs = 0;
    uint32_t lastTunnelMs = 0;
    uint32_t lastTxCompleted = 0;
    uint32_t lastTxProgressMs = 0;

    uint8_t nodeStatusTransferId = 0;
    uint8_t rcInputTransferId = 0;
    uint8_t tunnelTransferId = 0;

    DroneCANNodeStats stats {};
};

#endif
