#include "DroneCANFwUpdate.h"

#if defined(TARGET_RX) && defined(HAS_DRONECAN_FW_UPDATE)

#include <string.h>

#include <Arduino.h>
#include <hardware/flash.h>
#include <hardware/regs/addressmap.h>
#include <hardware/sync.h>

// A read is resent after READ_TIMEOUT_MS; the update fails after READ_DEADLINE_MS without a usable response
#define READ_TIMEOUT_MS     1000
#define READ_DEADLINE_MS    6000

// Sector 0 is held in RAM and written last: the bootrom only boots an image whose sector 0 is valid
// (the RP2350's boot block, the RP2040's second-stage bootloader), so a partial update never boots
#define HEAD_LEN            FLASH_SECTOR_SIZE
// This image's own vector table, at the same offset in any image built with the same core. It starts
// sector 3, so a bad one is only caught once flash has been touched
extern uint32_t __vectors[];
#define APP_VECTORS         ((uint32_t)__vectors - XIP_BASE)
#if defined(PICO_RP2350)
#define CHIP_NAME           "RP2350"
// The SDK's boot/picobin.h is not on the Arduino-Pico include path
#define PICOBIN_BLOCK_MARKER_START 0xFFFFDED3u
#else
#define CHIP_NAME           "RP2040"
// The bootrom checks a CRC32 at the end of the 256-byte second-stage bootloader
#define BOOT2_LEN           256
#endif

// The image ends before the LittleFS partition, which holds options.json and hardware.json
extern uint8_t _FS_start;
#define MAX_IMAGE_LEN       ((uint32_t)&_FS_start - XIP_BASE)

static uint8_t head[HEAD_LEN];
static uint8_t page[FLASH_PAGE_SIZE];

// Interrupts are off so core 0's flash writes (config, LittleFS), which idle this core through an
// IRQ, wait for this one to finish instead of interleaving with it
static bool flashWritePage(uint32_t addr, const uint8_t *data)
{
    const uint32_t save = save_and_disable_interrupts();
    if ((addr & (FLASH_SECTOR_SIZE - 1)) == 0)
        flash_range_erase(addr, FLASH_SECTOR_SIZE);
    flash_range_program(addr, data, FLASH_PAGE_SIZE);
    restore_interrupts(save);
    // Read back around the XIP cache
    return memcmp((const void *)(XIP_NOCACHE_NOALLOC_BASE + addr), data, FLASH_PAGE_SIZE) == 0;
}

static void flashEraseSector(uint32_t addr)
{
    const uint32_t save = save_and_disable_interrupts();
    flash_range_erase(addr, FLASH_SECTOR_SIZE);
    restore_interrupts(save);
}

#if defined(PICO_RP2350)
// An RP2350 Arm image has a boot block in sector 0
static bool headValid()
{
    for (uint32_t i = 0; i + 4 <= FLASH_SECTOR_SIZE; i += 4)
    {
        uint32_t word;
        memcpy(&word, &head[i], 4);
        if (word == PICOBIN_BLOCK_MARKER_START)
            return true;
    }
    return false;
}
#else
// An RP2040 image starts with a second-stage bootloader whose CRC32 (MPEG-2 variant) the bootrom checks
static bool headValid()
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < BOOT2_LEN - 4; ++i)
    {
        crc ^= (uint32_t)head[i] << 24;
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80000000) ? (crc << 1) ^ 0x04C11DB7 : crc << 1;
    }
    uint32_t stored;
    memcpy(&stored, &head[BOOT2_LEN - 4], 4);
    return crc == stored;
}
#endif

static bool vectorsValid(const uint8_t *vectors)
{
    uint32_t sp, reset;
    memcpy(&sp, vectors, 4);
    memcpy(&reset, vectors + 4, 4);
    return sp > SRAM_BASE && sp <= SRAM_END && (reset & 1) &&
           reset > XIP_BASE + APP_VECTORS && reset < XIP_BASE + MAX_IMAGE_LEN;
}

uint8_t DroneCANFwUpdate::begin(uint8_t serverNode, const uavcan_protocol_file_Path &imagePath, bool connected, uint32_t nowMs)
{
    if (active())
        return UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_IN_PROGRESS;
    if (connected)
        return UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_INVALID_MODE;
    if (serverNode == 0 || imagePath.path.len == 0)
        return UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_UNKNOWN;

    server = serverNode;
    path = imagePath;
    offset = 0;
    pending = false;
    progressMs = nowMs;
    failReason = nullptr;
    state = FETCHING;
    return UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;
}

void DroneCANFwUpdate::fail(const char *why)
{
    // Flash may already be invalidated: the app keeps running from RAM, and a new update can be started
    failReason = why;
    state = IDLE;
    pending = false;
}

void DroneCANFwUpdate::sendRead(CanardInstance *canard, uint32_t nowMs, bool canfd)
{
    uavcan_protocol_file_ReadRequest req {};
    req.offset = offset;
    req.path = path;
    const uint32_t len = uavcan_protocol_file_ReadRequest_encode(&req, requestBuf TAO_ARG(canfd));

    const uint8_t id = transferId;
    if (canardRequestOrRespond(canard, server, UAVCAN_PROTOCOL_FILE_READ_SIGNATURE, UAVCAN_PROTOCOL_FILE_READ_ID,
                               &transferId, CANARD_TRANSFER_PRIORITY_MEDIUM, CanardRequest, requestBuf, len CANFD_ARG(canfd)) < 0)
        return;  // TX queue full, retried on the next call until the deadline
    pendingTransferId = id;
    pending = true;
    sentMs = nowMs;
}

void DroneCANFwUpdate::service(CanardInstance *canard, uint32_t nowMs, bool canfd)
{
    if (state != FETCHING)
        return;

    if (nowMs - progressMs > READ_DEADLINE_MS)
    {
        fail("read timeout");
        return;
    }
    if (pending && nowMs - sentMs > READ_TIMEOUT_MS)
        pending = false;
    if (!pending)
        sendRead(canard, nowMs, canfd);
}

void DroneCANFwUpdate::onReadResponse(const CanardRxTransfer *transfer, const uavcan_protocol_file_ReadResponse &resp, uint32_t nowMs)
{
    if (state != FETCHING || !pending || transfer->source_node_id != server || transfer->transfer_id != pendingTransferId)
        return;
    pending = false;
    progressMs = nowMs;

    if (resp.error.value != UAVCAN_PROTOCOL_FILE_ERROR_OK)
    {
        fail("server error");
        return;
    }
    if (!acceptData(resp.data.data, resp.data.len))
        return;

    // A short read is the end of the file
    if (resp.data.len < sizeof(resp.data.data) && finish())
        rp2040.reboot();
}

bool DroneCANFwUpdate::acceptData(const uint8_t *data, uint32_t len)
{
    if (offset + len > MAX_IMAGE_LEN)
    {
        fail("image too large");
        return false;
    }

    for (uint32_t i = 0; i < len; ++i, ++offset)
    {
        if (offset < HEAD_LEN)
        {
            head[offset] = data[i];
            continue;
        }
        if (offset == HEAD_LEN)
        {
            // Last chance to back out with the old firmware intact
            if (!headValid())
            {
                fail("not an " CHIP_NAME " image");
                return false;
            }
            flashEraseSector(0);
            flashInvalidated = true;
        }
        page[offset % FLASH_PAGE_SIZE] = data[i];
        if (offset == APP_VECTORS + 7 && !vectorsValid(&page[APP_VECTORS % FLASH_PAGE_SIZE]))
        {
            fail("bad vector table");
            return false;
        }
        if (offset % FLASH_PAGE_SIZE == FLASH_PAGE_SIZE - 1 && !flashWritePage(offset - (FLASH_PAGE_SIZE - 1), page))
        {
            fail("flash verify");
            return false;
        }
    }
    return true;
}

bool DroneCANFwUpdate::finish()
{
    if (offset < APP_VECTORS + 8)
    {
        fail("image too small");
        return false;
    }

    // Partial last page, padded as erased flash
    const uint32_t tail = offset % FLASH_PAGE_SIZE;
    if (tail)
    {
        memset(page + tail, 0xFF, FLASH_PAGE_SIZE - tail);
        if (!flashWritePage(offset - tail, page))
        {
            fail("flash verify");
            return false;
        }
    }

    // Sector 0, erased when the update started, makes the image bootable again. Top down, so the pages the
    // bootrom needs (RP2350 block loop in pages 0-1, RP2040 boot2 in page 0) are the last written
    const uint32_t save = save_and_disable_interrupts();
    for (uint32_t addr = FLASH_SECTOR_SIZE; addr > 0; addr -= FLASH_PAGE_SIZE)
        flash_range_program(addr - FLASH_PAGE_SIZE, &head[addr - FLASH_PAGE_SIZE], FLASH_PAGE_SIZE);
    restore_interrupts(save);
    if (memcmp((const void *)XIP_NOCACHE_NOALLOC_BASE, head, FLASH_SECTOR_SIZE) != 0)
    {
        fail("flash verify");
        return false;
    }
    flashInvalidated = false;
    return true;
}

#endif
