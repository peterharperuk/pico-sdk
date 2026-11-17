# PICO_CMAKE_CONFIG: PICO_BTSTACK_PATH, Path to BTstack. Can be passed to CMake or set in your environment if you do not wish to use the version included with the SDK, type=string, default=<PICO_SDK_PATH>/lib/btstack, group=pico_btstack
if (DEFINED ENV{PICO_BTSTACK_PATH} AND (NOT PICO_BTSTACK_PATH))
    set(PICO_BTSTACK_PATH $ENV{PICO_BTSTACK_PATH})
    message("Using PICO_BTSTACK_PATH from environment ('${PICO_BTSTACK_PATH}')")
endif ()

set(BTSTACK_TEST_PATH "src/bluetooth.h")
if (NOT PICO_BTSTACK_PATH)
    set(PICO_BTSTACK_PATH ${PICO_SDK_PATH}/lib/btstack)
    if (PICO_CYW43_SUPPORTED AND NOT EXISTS ${PICO_BTSTACK_PATH}/${BTSTACK_TEST_PATH})
        message(WARNING "btstack submodule has not been initialized; Pico W BLE support will be unavailable.
            hint: try 'git submodule update --init' from your SDK directory (${PICO_SDK_PATH}).")
    endif()
elseif (NOT EXISTS ${PICO_BTSTACK_PATH}/${BTSTACK_TEST_PATH})
    message(WARNING "PICO_BTSTACK_PATH specified but content not present.")
endif()

if (EXISTS ${PICO_BTSTACK_PATH}/${BTSTACK_TEST_PATH})
    message("BTstack available at ${PICO_BTSTACK_PATH}")

    pico_register_common_scope_var(PICO_BTSTACK_PATH)

    # Use BTstack's own CMake source catalog if it has one, otherwise fall back
    # to the legacy hand-maintained lists for older BTstack versions.
    if (EXISTS ${PICO_BTSTACK_PATH}/tool/cmake/BTstack.cmake)
        include(${CMAKE_CURRENT_LIST_DIR}/btstack_groups.cmake)
    else()
        message("BTstack at ${PICO_BTSTACK_PATH} has no CMake source catalog; using legacy source lists")
        include(${CMAKE_CURRENT_LIST_DIR}/btstack_legacy_sources.cmake)
    endif()

    pico_add_library(pico_btstack_flash_bank)
    target_sources(pico_btstack_flash_bank INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/btstack_flash_bank.c
    )
    target_include_directories(pico_btstack_flash_bank_headers SYSTEM INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    pico_mirrored_target_link_libraries(pico_btstack_flash_bank INTERFACE pico_btstack_base pico_flash)

    pico_add_library(pico_btstack_run_loop_async_context NOFLAG)
    target_sources(pico_btstack_run_loop_async_context INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/btstack_run_loop_async_context.c
    )
    target_include_directories(pico_btstack_run_loop_async_context_headers SYSTEM INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    pico_mirrored_target_link_libraries(pico_btstack_run_loop_async_context INTERFACE pico_btstack_base pico_async_context_base)

    pico_add_library(pico_btstack_bnep_lwip_sys_freertos NOFLAG)
    target_include_directories(pico_btstack_bnep_lwip_sys_freertos SYSTEM INTERFACE
            ${PICO_BTSTACK_PATH}/platform/freertos
            )
    pico_mirrored_target_link_libraries(pico_btstack_bnep_lwip_sys_freertos INTERFACE
            pico_btstack_bnep_lwip
    )
    target_compile_definitions(pico_btstack_bnep_lwip_sys_freertos INTERFACE
            LWIP_PROVIDE_ERRNO=1
            PICO_LWIP_CUSTOM_LOCK_TCPIP_CORE=1
            )

    pico_promote_common_scope_vars()

    # pico_btstack_make_gatt_header(TARGET_LIB TARGET_TYPE GATT_FILE)
    # \brief\ Make a GATT header file from a BTstack GATT file.
    #
    # Pass the target library name, library type, and path to the GATT input file.
    # To add additional directories to the gatt import path, add them to the end of the argument list.
    #
    # \param\ TARGET_LIB The target library name
    # \param\ TARGET_TYPE The target library type
    # \param\ GATT_FILE The path to the GATT input file
    function(pico_btstack_make_gatt_header TARGET_LIB TARGET_TYPE GATT_FILE)
            find_package (Python3 REQUIRED COMPONENTS Interpreter)
            get_filename_component(GATT_NAME "${GATT_FILE}" NAME_WE)
            get_filename_component(GATT_PATH "${GATT_FILE}" PATH)
            if (NOT GATT_PATH)
                set(GATT_PATH "${CMAKE_CURRENT_LIST_DIR}")
            endif()
            set(TARGET_GATT "${TARGET_LIB}_${GATT_NAME}_gatt_header")
            set(GATT_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/${TARGET_GATT}")
            set(GATT_HEADER "${GATT_BINARY_DIR}/${GATT_NAME}.h")
            list(TRANSFORM ARGN PREPEND "-I")
            add_custom_target(${TARGET_GATT} DEPENDS ${GATT_HEADER})
            add_custom_command(
                    OUTPUT ${GATT_HEADER}
                    DEPENDS ${GATT_FILE}
                    WORKING_DIRECTORY ${GATT_PATH}
                    COMMAND ${CMAKE_COMMAND} -E make_directory ${GATT_BINARY_DIR} &&
                            ${Python3_EXECUTABLE} ${PICO_BTSTACK_PATH}/tool/compile_gatt.py ${GATT_FILE} ${GATT_HEADER} ${ARGN}
                    VERBATIM)
            add_dependencies(${TARGET_LIB}
                    ${TARGET_GATT}
                    )
            target_include_directories(${TARGET_LIB} ${TARGET_TYPE}
                    ${GATT_BINARY_DIR}
                    )
    endfunction()

    function(suppress_btstack_warnings)
        set_source_files_properties(
            ${PICO_BTSTACK_PATH}/src/ble/att_server.c
            ${PICO_BTSTACK_PATH}/src/ble/gatt-service/device_information_service_server.c
            ${PICO_BTSTACK_PATH}/src/ble/gatt-service/hids_client.c
            ${PICO_BTSTACK_PATH}/src/btstack_util.c
            ${PICO_BTSTACK_PATH}/src/btstack_crypto.c
            ${PICO_BTSTACK_PATH}/src/classic/a2dp.c
            ${PICO_BTSTACK_PATH}/src/classic/a2dp_sink.c
            ${PICO_BTSTACK_PATH}/src/classic/a2dp_source.c
            ${PICO_BTSTACK_PATH}/src/classic/avdtp.c
            ${PICO_BTSTACK_PATH}/src/classic/avdtp_source.c
            ${PICO_BTSTACK_PATH}/src/classic/avrcp.c
            ${PICO_BTSTACK_PATH}/src/classic/avrcp_controller.c
            ${PICO_BTSTACK_PATH}/src/classic/btstack_sbc_decoder_bluedroid.c
            ${PICO_BTSTACK_PATH}/src/classic/avrcp_target.c
            ${PICO_BTSTACK_PATH}/src/classic/hid_device.c
            ${PICO_BTSTACK_PATH}/src/classic/hsp_ag.c
            ${PICO_BTSTACK_PATH}/src/classic/hsp_hs.c
            ${PICO_BTSTACK_PATH}/src/classic/pan.c
            ${PICO_BTSTACK_PATH}/src/classic/pbap_client.c
            ${PICO_BTSTACK_PATH}/src/classic/rfcomm.c
            ${PICO_BTSTACK_PATH}/src/classic/sdp_client_rfcomm.c
            ${PICO_BTSTACK_PATH}/src/classic/sdp_server.c
            ${PICO_BTSTACK_PATH}/src/classic/spp_server.c
            PROPERTIES
            COMPILE_OPTIONS "-Wno-cast-qual"
            )
        set_source_files_properties(
            ${PICO_BTSTACK_PATH}/src/ble/sm.c
            ${PICO_BTSTACK_PATH}/src/l2cap.c
            PROPERTIES
            COMPILE_OPTIONS "-Wno-cast-qual;-Wno-unused-parameter"
            )
        set_source_files_properties(
            ${PICO_BTSTACK_PATH}/src/btstack_hid_parser.c
            PROPERTIES
            COMPILE_OPTIONS "-Wno-maybe-uninitialized"
            )
        set_source_files_properties(
            ${PICO_BTSTACK_PATH}/src/btstack_tlv_none.c
            ${PICO_BTSTACK_PATH}/src/classic/avdtp_util.c
            PROPERTIES
            COMPILE_OPTIONS "-Wno-unused-parameter"
            )
        set_source_files_properties(
            ${PICO_BTSTACK_PATH}/platform/embedded/hci_dump_embedded_stdout.c
            PROPERTIES
            COMPILE_OPTIONS "-Wno-suggest-attribute=format"
            )
        set_source_files_properties(
            ${PICO_BTSTACK_PATH}/src/hci.c
            ${PICO_BTSTACK_PATH}/src/classic/rfcomm.c
            PROPERTIES
            COMPILE_OPTIONS "-Wno-cast-qual;-Wno-format"
            )
        set_source_files_properties(
            ${PICO_BTSTACK_PATH}/platform/embedded/hal_flash_bank_memory.c
            PROPERTIES
            COMPILE_OPTIONS "-Wno-sign-compare;-Wno-format"
            )
        set_source_files_properties(
            ${PICO_BTSTACK_PATH}/platform/embedded/btstack_tlv_flash_bank.c
            PROPERTIES
            COMPILE_OPTIONS "-Wno-unused-parameter;-Wno-sign-compare"
            )
        set_source_files_properties(
            ${PICO_BTSTACK_PATH}/src/ble/gatt-service/hids_client.c
            PROPERTIES
            COMPILE_OPTIONS "-Wno-cast-qual;-Wno-null-dereference"
            )
        set_source_files_properties(
            ${PICO_BTSTACK_PATH}/src/classic/hfp.c
            PROPERTIES
            COMPILE_OPTIONS "-Wno-cast-qual;-Wno-null-dereference;-Wno-unused-parameter"
            )
        set_source_files_properties(
            ${PICO_BTSTACK_PATH}/src/classic/goep_server.c
            PROPERTIES
            COMPILE_OPTIONS "-Wno-unused-parameter;-Wno-null-dereference"
            )
        set_source_files_properties(
            ${PICO_BTSTACK_PATH}/src/ble/gatt-service/battery_service_client.c
            ${PICO_BTSTACK_PATH}/src/ble/gatt-service/device_information_service_client.c
            PROPERTIES
            COMPILE_OPTIONS "-Wno-null-dereference"
            )
        set_source_files_properties(
            ${PICO_BTSTACK_PATH}/src/classic/hfp_hf.c
            PROPERTIES
            COMPILE_OPTIONS "-Wno-type-limits;-Wno-stringop-overflow"
            )
        set_source_files_properties(
            ${PICO_BTSTACK_PATH}/src/btstack_crypto.c
            PROPERTIES
            COMPILE_OPTIONS "-Wno-cast-qual;-Wno-sign-compare"
            )
    endfunction()
endif()
