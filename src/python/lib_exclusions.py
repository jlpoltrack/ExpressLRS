"""PlatformIO pre-script that adds target-aware `lib_ignore` entries.

PlatformIO's default LDF does not evaluate preprocessor branches, so it
discovers target-incompatible libraries behind inactive `#if`s. `chain+` or
`deep+` follows those branches but breaks external-library dependency
resolution. Keep the default mode and explicitly exclude incompatible
libraries here.

`lib_ignore` is read by PlatformIO's Library Dependency Finder, so mutate the
project config rather than `env["LIB_IGNORE"]`. Entries are exact library
names from PlatformIO's dependency graph and may identify project, global, or
`lib_deps` libraries.

Set "*" for every MCU of a TX/RX target type; its entries are combined with
that target type's MCU-specific entries. An ignored library is not selected or
built; its target must not include headers or symbols it provides.
"""

LIBRARY_EXCLUSIONS = {
    # target type -> "*" (all target MCUs) or MCU -> library names
    "TX": {
        "*": ("AnalogVbat","Baro","dronecan-dsdl","GYRO","libcanard","MSPVTX","PWM","rx-crsf","ServoOutput","VTXSPI"),
        "esp32c3": ("GFX Library for Arduino","U8g2","GSENSOR","SCREEN","THERMAL"),
        "esp8266": ("GSENSOR","SCREEN","THERMAL"),
    },
    "RX": {
        "*": ("ADC","Backpack","BLE","GSENSOR","Handset","POWER_DETECT","SCREEN","tx-crsf","VTX"),
        "esp8266": ("dronecan-dsdl","VTXSPI"),
        "rp2040": ("VTXSPI","THERMAL","SerialUpdate","WIFI"),
        "rp2350": ("VTXSPI","THERMAL","SerialUpdate","WIFI"),
    }
}


def get_target_type(target_name):
    target_parts = target_name.upper().split("_")
    return next((target for target in ("RX", "TX") if target in target_parts), None)


def get_excluded_libraries(target_type, mcu):
    target_exclusions = LIBRARY_EXCLUSIONS.get(target_type, {})
    return target_exclusions.get("*", ()) + target_exclusions.get(mcu, ())

Import("env")

target_type = get_target_type(env["PIOENV"])
excluded_libraries = ()
if target_type:
    mcu = env.BoardConfig().get("build.mcu", "").lower()
    excluded_libraries = get_excluded_libraries(target_type, mcu)

if excluded_libraries:
    config = env.GetProjectConfig()
    ignored_libraries = config.get("env:" + env["PIOENV"], "lib_ignore", [])
    config.set("env:" + env["PIOENV"], "lib_ignore", list(dict.fromkeys(ignored_libraries + list(excluded_libraries))))
    print("Ignoring libraries for %s/%s: %s" % (target_type, mcu, ", ".join(excluded_libraries)))

# libcanard is fetched as a plain git repo, only canard.c is needed (skip its drivers, tests and C++ wrappers)
def skip_libcanard_extras(node):
    path = node.get_abspath().replace("\\", "/")
    return None if "/libcanard/" in path and not path.endswith("/libcanard/canard.c") else node

env.AddBuildMiddleware(skip_libcanard_extras)

# can2040 relies on -O2 (its docs), the RP2 core builds everything else at -Os
def can2040_o2(lib_env, node):
    path = node.srcnode().get_abspath().replace("\\", "/")
    if not path.endswith("/can2040/src/can2040.c"):
        return node
    # Flags can be tuples, e.g. ("-include", file)
    flags = [f for f in lib_env["CCFLAGS"] if not (isinstance(f, str) and f.startswith("-O"))]
    return lib_env.Object(node, CCFLAGS=flags + ["-O2"])

env.AddBuildMiddleware(can2040_o2)
