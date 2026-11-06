#pragma once

#include "targets.h"

#if defined(TARGET_RX) && defined(HAS_DRONECAN)

#include "SerialMavlink.h"
#include "DroneCANNode.h"

// Stream adapter for the DroneCAN tunnel.Targetted byte streams: reads return bytes from the FC,
// writes queue bytes for the FC
class DroneCANTunnelStream final : public Stream {
public:
    explicit DroneCANTunnelStream(DroneCANNode &node) : node(node) {}

    int available() override { return node.fcToRx.size(); }
    int read() override;
    int peek() override { return node.fcToRx.peek(); }
#if defined(PLATFORM_ESP32)
    // Bulk pop, Stream's default reads byte by byte with a millis() timeout check each (not virtual on RP2)
    using Stream::readBytes;
    size_t readBytes(char *buffer, size_t length) override;
#endif
    size_t write(uint8_t c) override { return write(&c, 1); }
    size_t write(const uint8_t *buf, size_t len) override;
    int availableForWrite() override { return node.rxToFc.free(); }
    void flush() override {}

    uint32_t rxToFcDropped = 0;

private:
    DroneCANNode &node;
};

// Private base so the node and stream are constructed before the SerialMavlink base that is handed the stream
struct DroneCANNodeHolder {
    DroneCANNodeHolder(int8_t txPin, int8_t rxPin) : node(txPin, rxPin), tunnelStream(node) {}
    DroneCANNode node;
    DroneCANTunnelStream tunnelStream;
};

// MAVLink over DroneCAN: RC goes as dronecan.sensors.rc.RCInput and MAVLink through uavcan.tunnel.Targetted,
// so RC_CHANNELS_OVERRIDE is never sent. Main loop side only, DroneCANNode does the CAN work (core 1 on RP2)
class SerialDroneCAN final : private DroneCANNodeHolder, public SerialMavlink {
public:
    SerialDroneCAN(int8_t txPin, int8_t rxPin);
    ~SerialDroneCAN() override;

    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;
    void processSerialInput() override;
    void sendQueuedData(uint32_t maxBytesToSend) override;

private:
    // Runs the node inline, or wakes core 1 to run it
    void kickNode();
    uint8_t rcInputQuality(uint16_t &status);
    void logStats();

    // Extended RCInput stats, decoded by ArduPilot >= 4.7. Older versions still get RC and MAVLink,
    // but show these as a meaningless RSSI (display only, it does not feed RC failsafe)
    uint8_t qualitySlot = 0;
    uint8_t lastTxPower = 0;
    uint32_t lastTxPowerMs = 0;

    uint16_t inputBufferPeak = 0;
};

#endif
