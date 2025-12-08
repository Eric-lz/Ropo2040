# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Set any variables required for importing libraries
SET(ProjectName Ropo2040)
SET(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/FreeRTOS-Kernel)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Set fastest clock speed
SET(PICO_USE_FASTEST_SUPPORTED_CLOCK 1)

project(Ropo2040 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add INA219 driver cloned into lib/
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/INA219)

# Add executable.
add_executable(${ProjectName} src/Ropo2040.cpp)

# Add sources
target_sources(${ProjectName} PRIVATE
    src/debug.cpp
    src/motor.cpp
    src/encoder.cpp
    src/current.cpp
    src/pid.cpp
    )

pico_set_program_name(${ProjectName} "Ropo2040")
pico_set_program_version(${ProjectName} "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(${ProjectName} 0)
pico_enable_stdio_usb(${ProjectName} 1)

# Add the standard library to the build
target_link_libraries(${ProjectName}
    pico_stdlib
    pico_multicore
    hardware_pwm
    FreeRTOS-Kernel-Heap4
    ina219
    )

# Add the standard include files to the build
target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ./inc
)

pico_add_extra_outputs(${ProjectName})

