# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Pull in Raspberry Pi Pico SDK (must be before project)
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

pico_sdk_init()

project(icm20689pico C CXX ASM)

include_directories(./include)

add_library(icm20689pico src/icm20689pico.c)

# Add the standard library to the build
target_link_libraries(icm20689pico PRIVATE pico_stdlib)

# Add any user requested libraries
target_link_libraries(icm20689pico PRIVATE hardware_i2c)

add_executable(icm20689pico_test src/icm20689pico.c test/test.c)

# Add the standard library to the build
target_link_libraries(icm20689pico_test PRIVATE pico_stdlib)

# Add any user requested libraries
target_link_libraries(icm20689pico_test PRIVATE hardware_i2c)

pico_enable_stdio_usb(icm20689pico_test 1)
pico_enable_stdio_uart(icm20689pico_test 0)

pico_add_extra_outputs(icm20689pico_test)