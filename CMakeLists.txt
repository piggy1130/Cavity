cmake_minimum_required(VERSION 3.13)

# Needs the Pico SDK: export PICO_SDK_PATH=~/pico/pico-sdk
include(pico_sdk_import.cmake)

project(sawtooth_daq C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(sawtooth_daq sawtooth_daq.c)

target_link_libraries(sawtooth_daq
    pico_stdlib
    pico_multicore
    hardware_spi
    hardware_adc
)

pico_enable_stdio_usb(sawtooth_daq 1)     # data goes to the Raspberry Pi over USB
pico_enable_stdio_uart(sawtooth_daq 0)

# Run entirely from SRAM so flash-cache misses can never stall the 16 us timing loop.
pico_set_binary_type(sawtooth_daq copy_to_ram)

pico_add_extra_outputs(sawtooth_daq)      # produces sawtooth_daq.uf2
