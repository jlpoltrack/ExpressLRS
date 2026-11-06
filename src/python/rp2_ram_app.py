import os

# Rules in the core's flash .text/.rodata that claim the application's code and constants. Without
# them those input sections fall through to the *(.text*)/*(.rodata*) catch-alls in .data, copied to RAM by crt0
FLASH_RULES = (
    '*(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a:) .text*)',
    '*(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a:) .rodata*)',
)


def _make_ram_ldscript(target, source, env):
    with open(str(source[0])) as f:
        script = f.read()
    for rule in FLASH_RULES:
        if script.count(rule) != 1:
            # A core update changed the script, fail rather than silently run from flash
            print(f'rp2_ram_app: "{rule}" not found once in {source[0]}')
            return 1
        script = script.replace(rule, '/* moved to RAM by rp2_ram_app.py */')
    with open(str(target[0]), 'w') as f:
        f.write(script)
    return None


def runFromRam(env):
    """
    Link the application to run from RAM; only boot-time code (vectors, crt0, the .ota stub) stays in flash.
    The core bakes $BUILD_DIR/memmap_default.ld into LINKFLAGS, so derive a patched copy and swap the -T argument.
    """
    default_ld = env.subst(os.path.join('$BUILD_DIR', 'memmap_default.ld'))
    ram_ld = env.subst(os.path.join('$BUILD_DIR', 'memmap_ram.ld'))
    if default_ld not in env['LINKFLAGS']:
        print(f'rp2_ram_app: {default_ld} is not in LINKFLAGS, cannot link the app to RAM')
        env.Exit(1)

    env.Command(ram_ld, default_ld, env.VerboseAction(_make_ram_ldscript, 'Generating linkerscript $TARGET'))
    env.Depends(os.path.join('$BUILD_DIR', '${PROGNAME}.elf'), ram_ld)
    env['LINKFLAGS'] = [ram_ld if f == default_ld else f for f in env['LINKFLAGS']]
