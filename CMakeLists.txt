cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project())
include(pico_sdk_import.cmake)

# Pull in FreeRTOS
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
# Enable Debugging
set(CMAKE_BUILD_TYPE Debug)

# Set the Pico board
# set(PICO_BOARD pico_w)
# set(PICO_BOARD pico)

# Set FreeRTOS port
# set(FREERTOS_PORT GCC_ARM_CM0 CACHE STRING "")

project(little_roboto C CXX ASM)

# Initialize Pico SDK
pico_sdk_init()

# Add source files
add_executable(little_roboto 
    src/mrroboto_lowlevel.c 
    src/tasks.c 
    src/pwm.c 
    src/bno055.c
)

target_include_directories(little_roboto PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/../FreeRTOS-Kernel/include
    ${CMAKE_CURRENT_LIST_DIR}/../FreeRTOS-Kernel/portable/GCC/RP2040
)


# Link FreeRTOS, Pico SDK libraries, and any other required libraries
target_link_libraries(little_roboto PRIVATE
    pico_stdlib             # Link Pico SDK standard library
    hardware_i2c            # Link hardware_i2c
    hardware_pwm            # Link hardware_pwm

    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4

    # pico_cyw43_arch_none    # WiFi chip
    # -nostartfiles           # Optional: to avoid the startup code from libc
    # -specs=nosys.specs      # Prevent standard library from expecting system-level routines
)


pico_enable_stdio_usb(little_roboto 1) # Send standard I/O messages over USB
pico_enable_stdio_uart(little_roboto 1) # Send standard I/O messages over UART (to Debugger)

pico_add_extra_outputs(little_roboto) # Enable .uf2 output

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()


add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

