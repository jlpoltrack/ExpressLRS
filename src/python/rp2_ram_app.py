import functools
import os

from elftools.elf.elffile import ELFFile
from elftools.elf.sections import SymbolTableSection

MOVED = '/* moved to RAM by rp2_ram_app.py */'
# Rules in the core's flash .text/.rodata that claim the application's code and constants. Without
# them those input sections fall through to the *(.text*)/*(.rodata*) catch-alls in .data, copied to RAM by crt0
LD_PATCHES = {
    'rp2350': (
        ('*(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a:) .text*)', MOVED),
        ('*(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a:) .rodata*)', MOVED),
    ),
    'rp2040': (
        ('*(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a: *interp.c.obj *divider.S.obj *PicoDVI.cpp.o *dvi.c.o) .text*)', MOVED),
        ('*(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a:) .rodata* .big_const*)', MOVED),
        # .data has no catch-all for .big_const
        ('        *(.rodata*)\n', '        *(.rodata* .big_const*)\n'),
    ),
}

# The RP2040's main RAM; its two 4 KB scratch banks hold the stacks. Every RP2 build must fit it with this
# much heap left, so the RP2350 builds stay portable to the RP2040
RP2040_RAM = 256 * 1024
MIN_HEAP = 32 * 1024
# M0+ code is larger than M33 code: +3.7% measured across the app objects (docs/rp2040-alignment.md)
M0PLUS_GROWTH = 0.05


def _make_ram_ldscript(target, source, env, mcu):
    with open(str(source[0])) as f:
        script = f.read()
    for old, new in LD_PATCHES[mcu]:
        if script.count(old) != 1:
            # A core update changed the script, fail rather than silently run from flash
            print(f'rp2_ram_app: "{old.strip()}" not found once in {source[0]}')
            return 1
        script = script.replace(old, new)
    with open(str(target[0]), 'w') as f:
        f.write(script)
    return None


def _check_ram_budget(target, source, env, mcu):
    with open(str(target[0]), 'rb') as f:
        symtab = ELFFile(f).get_section_by_name('.symtab')
        assert isinstance(symtab, SymbolTableSection)
        sym = {s: symtab.get_symbol_by_name(s)[0]['st_value'] for s in ('__data_start__', '__data_end__', '__end__')}
    ram_base = 0x20000000
    used = sym['__end__'] - ram_base
    if mcu != 'rp2040':
        used += int((sym['__data_end__'] - sym['__data_start__']) * M0PLUS_GROWTH)
    heap = RP2040_RAM - used
    kind = 'heap' if mcu == 'rp2040' else 'estimated RP2040 heap'
    print(f'rp2_ram_app: {kind} {heap / 1024:.1f} KB (minimum {MIN_HEAP // 1024} KB)')
    if heap < MIN_HEAP:
        print('rp2_ram_app: the RAM image no longer fits the RP2040 budget, see docs/rp2040-alignment.md')
        return 1
    return None


def runFromRam(env):
    """
    Link the application to run from RAM; only boot-time code (vectors, crt0, the .ota stub) stays in flash.
    The core bakes $BUILD_DIR/memmap_default.ld into LINKFLAGS, so derive a patched copy and swap the -T argument.
    """
    mcu = env.BoardConfig().get('build.mcu', '').lower()
    if mcu not in LD_PATCHES:
        print(f'rp2_ram_app: no linker patches for MCU "{mcu}"')
        env.Exit(1)
    default_ld = env.subst(os.path.join('$BUILD_DIR', 'memmap_default.ld'))
    ram_ld = env.subst(os.path.join('$BUILD_DIR', 'memmap_ram.ld'))
    if default_ld not in env['LINKFLAGS']:
        print(f'rp2_ram_app: {default_ld} is not in LINKFLAGS, cannot link the app to RAM')
        env.Exit(1)

    make_ld = functools.partial(_make_ram_ldscript, mcu=mcu)
    env.Command(ram_ld, default_ld, env.VerboseAction(make_ld, 'Generating linkerscript $TARGET'))
    elf = os.path.join('$BUILD_DIR', '${PROGNAME}.elf')
    env.Depends(elf, ram_ld)
    env['LINKFLAGS'] = [ram_ld if f == default_ld else f for f in env['LINKFLAGS']]
    env.AddPostAction(elf, env.VerboseAction(functools.partial(_check_ram_budget, mcu=mcu), 'Checking RAM against the RP2040 budget'))
