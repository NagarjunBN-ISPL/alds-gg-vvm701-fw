# Sources are listed EXPLICITLY, not globbed. This is deliberate and matches
# ESP-IDF's own guidance.
#
# The previous `file(GLOB app_sources "${CMAKE_SOURCE_DIR}/src/*.c")` evaluated
# the glob once at configure time and cached it, so adding a new src/*.c left
# it silently out of the build — surfacing only as an "undefined reference" at
# link, which looks like a code bug rather than a stale-file-list bug. It bit
# twice on 2026-07-17 (gps.c, then tls_ca.c).
#
# CONFIGURE_DEPENDS is NOT a usable fix here: ESP-IDF include()s this file in
# CMake script mode to extract component requirements, and CONFIGURE_DEPENDS is
# invalid in that mode ("CONFIGURE_DEPENDS is invalid for script and find
# package modes") — it breaks the build outright.
#
# An explicit list fixes the staleness properly: adding a file means editing
# THIS file, and CMake tracks CMakeLists.txt as a build dependency, so the
# reconfigure happens automatically. A forgotten entry also fails loudly at
# link rather than silently.
#
# ==> Adding a new src/*.c? Add it to this list, keep it alphabetical. <==
idf_component_register(
    SRCS
        "${CMAKE_SOURCE_DIR}/src/di.c"
        "${CMAKE_SOURCE_DIR}/src/diagnostics.c"
        "${CMAKE_SOURCE_DIR}/src/dispenser.c"
        "${CMAKE_SOURCE_DIR}/src/du_task.c"
        "${CMAKE_SOURCE_DIR}/src/forensics.c"
        "${CMAKE_SOURCE_DIR}/src/fota.c"
        "${CMAKE_SOURCE_DIR}/src/genset.c"
        "${CMAKE_SOURCE_DIR}/src/gps.c"
        "${CMAKE_SOURCE_DIR}/src/main.c"
        "${CMAKE_SOURCE_DIR}/src/modem.c"
        "${CMAKE_SOURCE_DIR}/src/ms51_bus.c"
        "${CMAKE_SOURCE_DIR}/src/mqtt.c"
        "${CMAKE_SOURCE_DIR}/src/network.c"
        "${CMAKE_SOURCE_DIR}/src/nvs_config.c"
        "${CMAKE_SOURCE_DIR}/src/radar.c"
        "${CMAKE_SOURCE_DIR}/src/relay.c"
        "${CMAKE_SOURCE_DIR}/src/rs485_bus.c"
        "${CMAKE_SOURCE_DIR}/src/store.c"
        "${CMAKE_SOURCE_DIR}/src/tls_ca.c"
        "${CMAKE_SOURCE_DIR}/src/yenen.c"
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/src"
)
