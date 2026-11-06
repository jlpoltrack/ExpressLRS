#!/usr/bin/env python3
"""
Update an ELRS RP2350 receiver's firmware over DroneCAN and validate it.

Runs a DroneCAN node on the FC's CAN bus through MAVLink CAN forwarding (pydronecan 'mavcan:'), serves the image
with uavcan.protocol.file.Read, starts the update with BeginFirmwareUpdate, waits for the receiver to reboot, and
optionally reads the flash back over USB with picotool to compare it with the image.

  tools/dronecan_fw_update.py src/.pio/build/<env>/firmware.bin --readback      # or firmware.uf2
  tools/dronecan_fw_update.py bad.bin --expect-reject          # must be refused, receiver keeps running
  tools/dronecan_fw_update.py firmware.bin --abort-at-kb 64    # stop serving mid-update (power-loss test)
"""
import argparse
import functools
import os
import struct
import subprocess
import sys
import tempfile
import time

import dronecan
import serial.tools.list_ports
from dronecan import uavcan

print = functools.partial(print, flush=True)

FC_USB_VID = 0x1209
RP2_USB_VID = 0x2E8A
PICOTOOL = os.path.expanduser('~/.platformio/packages/tool-picotool-rp2040-earlephilhower/picotool')
XIP_BASE = 0x10000000
MODES = {0: 'operational', 1: 'initialization', 2: 'maintenance', 3: 'software_update', 7: 'offline'}
HEALTH = {0: 'ok', 1: 'warning', 2: 'error', 3: 'critical'}
IMAGE_PATH = 'fw.bin'
UF2_MAGIC = (0x0A324655, 0x9E5D5157, 0x0AB16F30)
UF2_FLAG_NOT_MAIN_FLASH = 0x1
UF2_FLAG_FAMILY_ID = 0x2000
UF2_FAMILY_RP2350_ARM_S = 0xE48BFF59
FLASH_END = XIP_BASE + 0x1000000


def uf2_to_image(data):
    """Flatten the RP2350 Arm flash blocks of a UF2 into a binary image starting at XIP_BASE, gaps as erased flash."""
    blocks = {}
    for i in range(0, len(data) - 511, 512):
        m0, m1, flags, addr, size, _, _, family = struct.unpack_from('<8I', data, i)
        if (m0, m1) != UF2_MAGIC[:2] or struct.unpack_from('<I', data, i + 508)[0] != UF2_MAGIC[2]:
            sys.exit(f'FAIL: bad UF2 block at offset {i}')
        # Skips e.g. the RP2350-E10 workaround block, which uses the 'absolute' family
        if flags & UF2_FLAG_NOT_MAIN_FLASH or (flags & UF2_FLAG_FAMILY_ID and family != UF2_FAMILY_RP2350_ARM_S):
            continue
        if not XIP_BASE <= addr < FLASH_END:
            continue
        blocks[addr - XIP_BASE] = data[i + 32:i + 32 + size]
    if not blocks:
        sys.exit('FAIL: no RP2350 Arm flash blocks in the UF2')
    end = max(off + len(b) for off, b in blocks.items())
    image = bytearray(b'\xff' * end)
    for off, b in blocks.items():
        image[off:off + len(b)] = b
    return bytes(image)


def load_image(path):
    data = open(path, 'rb').read()
    if data[:4] == b'\x7fELF':
        sys.exit('FAIL: an .elf has no options or hardware layout, which are appended after linking; '
                 'use firmware.bin or firmware.uf2 from the same build folder')
    if len(data) >= 512 and struct.unpack_from('<I', data)[0] == UF2_MAGIC[0]:
        return uf2_to_image(data)
    return data


def find_fc_port():
    ports = sorted(p.device for p in serial.tools.list_ports.comports() if p.vid == FC_USB_VID)
    if not ports:
        sys.exit('no FC (VID 0x1209) found, pass --port')
    return ports[0]


class Updater:
    def __init__(self, node, target, image):
        self.node = node
        self.target = target
        self.image = image
        self.status = None          # last NodeStatus from the target
        self.status_time = 0
        self.max_uptime = 0
        self.rebooted = False
        self.served = 0             # end of the last chunk served
        self.last_read = 0
        self.abort_at = None
        self.transfer_errors = 0
        node.add_handler(uavcan.protocol.NodeStatus, self.on_status)
        node.add_handler(uavcan.protocol.file.Read, self.on_read)

    def on_status(self, e):
        if e.transfer.source_node_id != self.target:
            return
        up = e.message.uptime_sec
        if self.status is not None and up + 2 < self.max_uptime:
            self.rebooted = True
        self.max_uptime = max(self.max_uptime, up)
        self.status = e.message
        self.status_time = time.monotonic()

    def on_read(self, e):
        resp = uavcan.protocol.file.Read.Response()
        if e.request.path.path.decode() != IMAGE_PATH:
            resp.error.value = resp.error.NOT_FOUND
            return resp
        off = e.request.offset
        if self.abort_at is not None and off >= self.abort_at:
            raise SystemExit(f'aborted at offset {off} as requested, file server gone')
        resp.data = bytearray(self.image[off:off + 256])
        resp.error.value = resp.error.OK
        self.served = off + len(resp.data)
        self.last_read = time.monotonic()
        return resp

    def spin(self, timeout):
        # Forwarded frames can be lost, pydronecan raises on the broken transfers that leaves
        try:
            self.node.spin(timeout)
        except dronecan.transport.TransferError as ex:
            self.transfer_errors += 1
            if self.transfer_errors <= 3:
                print(f'  ignored broken transfer: {ex}')

    def spin_until(self, cond, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin(0.05)
            if cond():
                return
        sys.exit(f'FAIL: timed out after {timeout}s waiting for {what}')

    def call(self, request, timeout=3):
        result = []
        self.node.request(request, self.target, lambda e: result.append(e), timeout=timeout)
        self.spin_until(lambda: result, timeout + 1, type(request).__name__)
        if result[0] is None:
            sys.exit(f'FAIL: no response to {type(request).__name__}')
        return result[0].response

    def describe(self, label):
        info = self.call(uavcan.protocol.GetNodeInfo.Request())
        sw = info.software_version
        print(f'{label}: {info.name.decode()} v{sw.major}.{sw.minor} vcs {sw.vcs_commit:08x} '
              f'uptime {info.status.uptime_sec}s mode {MODES.get(info.status.mode)} health {HEALTH.get(info.status.health)}')
        return info


def enter_bootsel():
    """Arduino-Pico has no picotool reset interface: a 1200 baud touch on its USB serial port reboots to BOOTSEL."""
    port = next((p.device for p in serial.tools.list_ports.comports() if p.vid == RP2_USB_VID), None)
    if not port:
        sys.exit('FAIL: receiver USB port (VID 0x2E8A) not found for readback')
    s = serial.Serial(port, 1200)
    s.dtr = False
    time.sleep(0.2)
    s.close()
    for _ in range(20):
        if subprocess.run([PICOTOOL, 'info'], capture_output=True).returncode == 0:
            return
        time.sleep(0.5)
    sys.exit('FAIL: receiver did not enter BOOTSEL')


def readback(image):
    """Read the flash back over USB with picotool and compare it with the image, then reboot to the app."""
    enter_bootsel()
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'readback.bin')
        r = subprocess.run([PICOTOOL, 'save', '-r', hex(XIP_BASE), hex(XIP_BASE + len(image)), out, '-t', 'bin'],
                           capture_output=True, text=True)
        subprocess.run([PICOTOOL, 'reboot'], capture_output=True)
        if r.returncode != 0:
            sys.exit(f'FAIL: picotool save failed: {r.stdout}{r.stderr}')
        data = open(out, 'rb').read()
    if data != image:
        diff = next((i for i in range(min(len(data), len(image))) if data[i] != image[i]), None)
        sys.exit(f'FAIL: flash does not match the image (len {len(data)}/{len(image)}, first difference at {diff})')
    print(f'readback: flash matches the image ({len(image)} bytes)')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('image', nargs='?', help='firmware.bin or firmware.uf2 from the build folder (options and hardware layout appended)')
    ap.add_argument('--status', action='store_true', help='only print the receiver\'s node info')
    ap.add_argument('--port', help='FC MAVLink USB port, default the first VID 0x1209 port')
    ap.add_argument('--bus', type=int, default=1, help='FC CAN bus to forward (1-based)')
    ap.add_argument('--target', type=int, default=68, help='receiver node id')
    ap.add_argument('--node-id', type=int, default=100, help='node id of this file server')
    ap.add_argument('--timeout', type=float, default=600, help='seconds allowed for the transfer')
    ap.add_argument('--readback', action='store_true', help='verify flash over USB with picotool afterwards')
    ap.add_argument('--expect-reject', action='store_true', help='the receiver must refuse the image and keep running')
    ap.add_argument('--abort-at-kb', type=int, help='stop serving at this offset to simulate a lost file server')
    args = ap.parse_args()

    if not args.image and not args.status:
        ap.error('an image is required unless --status')
    image = load_image(args.image) if args.image else b''
    port = args.port or find_fc_port()
    what = f'image {args.image} ({len(image)} bytes)' if image else 'status'
    print(f'{what}, FC {port} bus {args.bus}, receiver node {args.target}')

    node = dronecan.make_node('mavcan:' + port, node_id=args.node_id, bitrate=1000000, bus_number=args.bus)
    up = Updater(node, args.target, image)
    if args.abort_at_kb is not None:
        up.abort_at = args.abort_at_kb * 1024

    try:
        up.spin_until(lambda: up.status is not None, 10, f'NodeStatus from node {args.target}')
        up.describe('node' if args.status else 'before')
        if args.status:
            return

        req = uavcan.protocol.file.BeginFirmwareUpdate.Request()
        req.source_node_id = args.node_id
        req.image_file_remote_path.path = IMAGE_PATH
        resp = up.call(req)
        print(f'BeginFirmwareUpdate: error {resp.error} {resp.optional_error_message.decode()}')
        if resp.error != 0:
            sys.exit('FAIL: update refused')

        start = time.monotonic()
        up.last_read = start
        last_print = start
        while not up.rebooted:
            up.spin(0.05)
            now = time.monotonic()
            if now - last_print >= 2:
                last_print = now
                st = up.status
                rate = up.served / max(now - start, 1e-3) / 1024
                print(f'  {up.served / 1024:6.1f}/{len(image) / 1024:.1f} KB  {rate:5.2f} KB/s  '
                      f'mode {MODES.get(st.mode)} health {HEALTH.get(st.health)} vendor {st.vendor_specific_status_code}')
                if args.expect_reject and st.mode == 0 and up.served:
                    break
            served_all = up.served >= len(image)
            if served_all and not up.rebooted and up.status.mode == 0 and up.status_time > up.last_read and not args.expect_reject:
                sys.exit('FAIL: receiver left software update mode without rebooting, the final flash verify failed')
            if now - up.last_read > 15 and not served_all:
                if args.expect_reject:
                    break
                sys.exit('FAIL: receiver stopped reading')
            if now - start > args.timeout:
                sys.exit('FAIL: transfer timed out')

        if args.expect_reject:
            up.spin_until(lambda: up.status.mode == 0, 10, 'receiver back to operational')
            if up.rebooted:
                sys.exit('FAIL: receiver rebooted on an image it should have refused')
            up.describe('after reject')
            print('PASS: image refused, receiver still running')
            return

        took = time.monotonic() - start
        print(f'transfer done: {len(image)} bytes in {took:.1f}s ({len(image) / took / 1024:.2f} KB/s), receiver rebooted')
        up.spin_until(lambda: up.status is not None and up.status.mode == 0 and up.status_time > start, 15, 'operational')
        after = up.describe('after')
        if after.status.health != 0:
            sys.exit('FAIL: receiver unhealthy after the update')
    finally:
        node.close()

    if args.readback:
        # Spaced from the update reboot: ELRS counts quick reboots towards bind mode
        time.sleep(8)
        readback(image)
    print('PASS')


if __name__ == '__main__':
    main()
