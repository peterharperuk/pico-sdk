# Define the pico_btstack_* libraries from BTstack's CMake source catalog
# (tool/cmake/BTstackSources.cmake)
#
# Each library is described as a set of BTstack source groups. Group names refer
# to variables defined in # ${PICO_BTSTACK_PATH}/tool/cmake/BTstackSources.cmake,
# Group dependencies are resolved by BTstack (btstack_resolve_groups), so changes
# are picked up automatically.
#

include(${PICO_BTSTACK_PATH}/tool/cmake/BTstack.cmake)

# pico_btstack_add_groups(TARGET GROUP... [EXCLUDE GROUP...])
# Add the sources of the given BTstack source groups (and their dependencies)
# to an SDK interface library, and their include directories to TARGET_headers.
# Groups listed after EXCLUDE are left out even if they are dependencies, for
# groups the SDK provides or adds conditionally itself.
function(pico_btstack_add_groups TARGET)
    cmake_parse_arguments(ARG "" "" "EXCLUDE" ${ARGN})
    btstack_resolve_groups(groups GROUPS ${ARG_UNPARSED_ARGUMENTS})
    if (ARG_EXCLUDE)
        list(REMOVE_ITEM groups ${ARG_EXCLUDE})
    endif()
    btstack_collect_sources(sources GROUPS ${groups})
    btstack_collect_include_directories(include_dirs GROUPS ${groups})
    btstack_collect_consumer_include_directories(consumer_include_dirs GROUPS ${groups})
    target_sources(${TARGET} INTERFACE ${sources})
    target_include_directories(${TARGET}_headers SYSTEM INTERFACE ${include_dirs} ${consumer_include_dirs})
endfunction()

pico_add_library(pico_btstack_base NOFLAG)
pico_btstack_add_groups(pico_btstack_base
    core
    tlv-none
    third-party-micro-ecc
    third-party-rijndael
    third-party-md5
    third-party-yxml
    platform-embedded-tlv-flash-bank
    platform-embedded-hci-dump-stdout
    platform-embedded-hci-dump-segger-rtt
    # The SEGGER RTT core is added below, only when pico_stdio_rtt doesn't provide it
    EXCLUDE third-party-segger-rtt-core
)
btstack_collect_public_include_directories(BTSTACK_PUBLIC_INCLUDE_DIRS)
target_include_directories(pico_btstack_base_headers SYSTEM INTERFACE
    ${BTSTACK_PUBLIC_INCLUDE_DIRS}
    ${PICO_BTSTACK_PATH}/
    ${PICO_BTSTACK_PATH}/platform/embedded
)
target_sources(pico_btstack_base INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/btstack_stdin_pico.c
)

# pico-sdk now supports RTT using pico_enable_stdio_rtt in your cmake file or -DPICO_STDIO_RTT=1 on the command line
# Then the output of printf goes to RTT. But if you define ENABLE_SEGGER_RTT=1 it will use the btstack functionality to use RTT
# and we'll have to add the source file it requires.
btstack_collect_sources(BTSTACK_SEGGER_RTT_SOURCES GROUPS third-party-segger-rtt-core)
target_sources(pico_btstack_base INTERFACE
    "$<IF:$<BOOL:$<IF:$<STREQUAL:$<TARGET_PROPERTY:PICO_TARGET_STDIO_RTT>,>,${PICO_STDIO_RTT},$<TARGET_PROPERTY:PICO_TARGET_STDIO_RTT>>>,,${BTSTACK_SEGGER_RTT_SOURCES}>"
)

pico_add_library(pico_btstack_ble)
pico_btstack_add_groups(pico_btstack_ble
    ble
    gatt
    gatt-services
    le-device-db-memory
    le-device-db-tlv
)
pico_mirrored_target_link_libraries(pico_btstack_ble INTERFACE pico_btstack_base)
target_compile_definitions(pico_btstack_ble_headers INTERFACE ENABLE_BLE=1)

pico_add_library(pico_btstack_classic)
pico_btstack_add_groups(pico_btstack_classic
    classic
    link-key-db-tlv
)
pico_mirrored_target_link_libraries(pico_btstack_classic INTERFACE pico_btstack_base)
target_compile_definitions(pico_btstack_classic_headers INTERFACE ENABLE_CLASSIC=1)

pico_add_library(pico_btstack_mesh)
pico_btstack_add_groups(pico_btstack_mesh
    mesh
)
pico_mirrored_target_link_libraries(pico_btstack_mesh INTERFACE pico_btstack_ble)
target_compile_definitions(pico_btstack_mesh_headers INTERFACE ENABLE_MESH=1)

pico_add_library(pico_btstack_sbc_common NOFLAG)
pico_btstack_add_groups(pico_btstack_sbc_common
    classic-btstack-sbc-bluedroid
)

pico_add_library(pico_btstack_sbc_encoder NOFLAG)
pico_btstack_add_groups(pico_btstack_sbc_encoder
    classic-btstack-sbc-encoder-bluedroid
    third-party-bluedroid-sbc-encoder
)
pico_mirrored_target_link_libraries(pico_btstack_sbc_encoder INTERFACE pico_btstack_sbc_common)

pico_add_library(pico_btstack_sbc_decoder NOFLAG)
pico_btstack_add_groups(pico_btstack_sbc_decoder
    classic-btstack-sbc-decoder-bluedroid
    third-party-bluedroid-sbc-decoder
)
pico_mirrored_target_link_libraries(pico_btstack_sbc_decoder INTERFACE pico_btstack_sbc_common)

pico_add_library(pico_btstack_bnep_lwip NOFLAG)
pico_btstack_add_groups(pico_btstack_bnep_lwip
    bnep-lwip
)
