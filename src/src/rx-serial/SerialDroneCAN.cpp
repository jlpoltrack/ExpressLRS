#include "SerialDroneCAN.h"

#if defined(TARGET_RX) && defined(HAS_DRONECAN)

#include "common.h"
#include "config.h"
#include "helpers.h"
#include "logging.h"

// RC is sent at 100Hz to match MAVLink over UART, not at the OTA packet rate
#define DRONECAN_RC_INTERVAL_MS     10
#define DRONECAN_TX_POWER_REPORT_MS 2500

int DroneCANTunnelStream::read()
{
    uint8_t c;
    if (node.fcToRx.size() == 0)
        return -1;
    node.fcToRx.pop(&c, 1);
    return c;
}

size_t DroneCANTunnelStream::write(const uint8_t *buf, size_t len)
{
    if (node.rxToFc.free() < len)
    {
        rxToFcDropped += len;
        return 0;
    }
    node.rxToFc.push(buf, len);
    return len;
}

#if defined(PLATFORM_ESP32)
size_t DroneCANTunnelStream::readBytes(char *buffer, size_t length)
{
    const size_t len = std::min<size_t>(length, node.fcToRx.size());
    node.fcToRx.pop((uint8_t *)buffer, len);
    return len;
}
#endif

SerialDroneCAN::SerialDroneCAN(int8_t txPin, int8_t rxPin)
    : DroneCANNodeHolder(txPin, rxPin), SerialMavlink(tunnelStream, tunnelStream)
{
    if (txPin == UNDEF_PIN || rxPin == UNDEF_PIN)
    {
        ERRLN("DroneCAN requires both CAN TX and RX pins");
        return;
    }
#if defined(PLATFORM_RP2)
    RP2Core1::start(&node);
#else
    node.start();
#endif
    if (!node.driverStarted())
    {
        ERRLN("DroneCAN driver start failed");
        return;
    }
    DBGLN("DroneCAN started tx=%d rx=%d", txPin, rxPin);
}

SerialDroneCAN::~SerialDroneCAN()
{
#if defined(PLATFORM_RP2)
    RP2Core1::stop();
#else
    node.stop();
#endif
}

void SerialDroneCAN::kickNode()
{
#if defined(PLATFORM_RP2)
    RP2Core1::wake();
#else
    node.service();
#endif
}

uint32_t SerialDroneCAN::sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData)
{
    // Wait for the next OTA frame, then send at most every 10ms (frameAvailable latches in between)
    if (!node.driverStarted() || !node.busActive() || (!frameAvailable && !failsafe))
        return DURATION_IMMEDIATELY;
    if (failsafe && config.GetFailsafeMode() == FAILSAFE_NO_PULSES)
        return DRONECAN_RC_INTERVAL_MS;

    dronecan_sensors_rc_RCInput rcInput {};
    if (failsafe)
    {
        rcInput.status |= DRONECAN_SENSORS_RC_RCINPUT_STATUS_FAILSAFE;
    }
    else
    {
        rcInput.quality = rcInputQuality(rcInput.status);
        rcInput.status |= DRONECAN_SENSORS_RC_RCINPUT_STATUS_QUALITY_VALID;
    }
    rcInput.rcin.len = CRSF_NUM_CHANNELS;
    for (unsigned ch = 0; ch < CRSF_NUM_CHANNELS; ++ch)
    {
        rcInput.rcin.data[ch] = CRSF_to_US(channelData[ch]);
    }

    node.postRC(rcInput);
    kickNode();

    return DRONECAN_RC_INTERVAL_MS;
}

void SerialDroneCAN::processSerialInput()
{
    if (!node.driverStarted())
        return;

#if !defined(PLATFORM_RP2)
    node.service();
#endif
    logStats();

    // Moves bytes received from the FC into the MAVLink downlink buffer
    SerialIO::processSerialInput();
    inputBufferPeak = std::max(inputBufferPeak, inputBufferUsed());
}

void SerialDroneCAN::sendQueuedData(uint32_t maxBytesToSend)
{
    if (!node.driverStarted())
        return;

    // Writes RADIO_STATUS (100Hz) and uplinked MAVLink into the tunnel stream
    const uint32_t queued = node.rxToFc.size();
    SerialMavlink::sendQueuedData(maxBytesToSend);
    if (node.rxToFc.size() != queued)
        kickNode();
}

uint8_t SerialDroneCAN::rcInputQuality(uint16_t &status)
{
    // Round robin of RSSI dBm, LQ + antenna, and SNR or TX power
    qualitySlot = (qualitySlot + 1) % 3;
    if (qualitySlot == 0)
    {
        status |= DRONECAN_SENSORS_RC_RCINPUT_QUALITY_TYPE_RSSI_DBM;
        return linkStats.active_antenna ? linkStats.uplink_RSSI_2 : linkStats.uplink_RSSI_1;
    }
    if (qualitySlot == 1)
    {
        status |= DRONECAN_SENSORS_RC_RCINPUT_QUALITY_TYPE_LQ_ACTIVE_ANTENNA;
        return linkStats.uplink_Link_quality | (linkStats.active_antenna ? 0x80 : 0);
    }

    const uint32_t now = millis();
    if (linkStats.uplink_TX_Power != lastTxPower || now - lastTxPowerMs > DRONECAN_TX_POWER_REPORT_MS)
    {
        // TX power in 5mW units, indexed by CRSF power level
        static const uint16_t crsfPowerMw[] = {0, 10, 25, 100, 500, 1000, 2000, 250, 50};
        lastTxPower = linkStats.uplink_TX_Power;
        lastTxPowerMs = now;
        status |= DRONECAN_SENSORS_RC_RCINPUT_QUALITY_TYPE_TX_POWER;
        const uint16_t mw = lastTxPower < ARRAY_SIZE(crsfPowerMw) ? crsfPowerMw[lastTxPower] : 0;
        return std::min(mw / 5, 255);
    }
    status |= DRONECAN_SENSORS_RC_RCINPUT_QUALITY_TYPE_SNR;
    return 128 + linkStats.uplink_SNR;
}

void SerialDroneCAN::logStats()
{
#if defined(DEBUG_LOG)
    // Per second; st: 1=running 2=bus-off 3=recovering 4=errata reset, drop: rx overrun/rx queue full/tx failed,
    // berr: can2040 bus errors, rst: errata resets, stall: TX flushes, lost: tunnel bytes dropped from/to FC,
    // buf: peak downlink buffer use, in: tunnel bytes from FC, xerr: transfers lost to missing frames, ni: node info requests
    DroneCANNodeStats s;
    if (!node.takeStats(s))
        return;
    DBGLN("DC st=%u tec=%u rec=%u rx=%u tx=%u rc=%u fs=%u fc=%u drop=%u/%u/%u boff=%u berr=%u rst=%u stall=%u lost=%u/%u buf=%u in=%u xerr=%u ni=%u",
          s.state, s.tec, s.rec, s.rxFrames, s.txFrames, s.rcSent, failsafe, node.serverNodeId(), s.driver.rxOverrun,
          s.driver.rxQueueFull, s.driver.txFailed, s.driver.busOff, s.driver.busErrors, s.driver.errataResets,
          s.txStalls, s.tunnelInDropped, tunnelStream.rxToFcDropped, inputBufferPeak, s.tunnelInBytes,
          s.rxTransferErrors, s.nodeInfoRequests);
    inputBufferPeak = 0;
    tunnelStream.rxToFcDropped = 0;
#endif
}

#endif
