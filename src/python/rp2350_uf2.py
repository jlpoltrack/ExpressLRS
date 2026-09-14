import os
import subprocess

# picotool infers the UF2 family from an ELF, but a raw binary carries no architecture
# information and it silently falls back to the RP2040 family, which BOOTSEL rejects.
MCU_FAMILY = {
    'rp2040': 'rp2040',
    'rp2350': 'rp2350-arm-s',
}

XIP_BASE = 0x10000000


def _picotool(env):
    tool = env.PioPlatform().get_package_dir('tool-picotool-rp2040-earlephilhower')
    return os.path.join(tool, 'picotool') if tool else 'picotool'


def convertToUF2(source, target, env):
    """
    Rebuild firmware.uf2 from the configured firmware.bin.

    The platform generates the UF2 from the ELF as soon as it is linked, which is before
    the options/hardware layout is appended to the binary, so that image is always bare.
    """
    bin_file = str(target[0])
    uf2_file = os.path.splitext(bin_file)[0] + '.uf2'

    mcu = env.BoardConfig().get('build.mcu', '').lower()
    if mcu not in MCU_FAMILY:
        print(f'Cannot build a UF2 for unknown MCU "{mcu}"')
        env.Exit(1)

    # The whole image is converted in one pass so the appended configuration stays
    # contiguous with the firmware; UF2 payloads are 256-byte aligned and the options
    # start at __flash_binary_end, which is not.
    subprocess.run([
        _picotool(env), 'uf2', 'convert', bin_file, '-t', 'bin', uf2_file,
        '-o', hex(XIP_BASE), '--family', MCU_FAMILY[mcu]
    ], check=True)
