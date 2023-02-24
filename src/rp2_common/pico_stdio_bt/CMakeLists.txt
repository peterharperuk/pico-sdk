pico_add_library(pico_stdio_bt_config)
target_include_directories(pico_stdio_bt_config INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/config
)

pico_add_library(pico_stdio_bt)
target_sources(pico_stdio_bt INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/stdio_bt.c
)
target_include_directories(pico_stdio_bt_headers INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
pico_mirrored_target_link_libraries(pico_stdio_bt INTERFACE
    pico_stdio
    pico_btstack_classic
    pico_btstack_cyw43
)