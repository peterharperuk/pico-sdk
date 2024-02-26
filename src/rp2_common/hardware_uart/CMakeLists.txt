pico_simple_hardware_target(uart)

pico_mirrored_target_link_libraries(hardware_uart INTERFACE hardware_resets hardware_clocks)

if (PICO_DEFAULT_UART_BAUD_RATE)
target_compile_definitions(hardware_uart INTERFACE
        PICO_DEFAULT_UART_BAUD_RATE=${PICO_DEFAULT_UART_BAUD_RATE})
endif()
