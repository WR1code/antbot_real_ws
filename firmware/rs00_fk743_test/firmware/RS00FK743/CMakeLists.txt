cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME RS00FK743)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../App/Src/rs00_protocol.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../App/Src/rs00_stm32_fdcan.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../App/Src/steering_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../App/Src/chassis_direction.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../App/Src/chassis_translation_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../App/Src/mini_drive_protocol.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../App/Src/drive_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../App/Src/host_cmd_vel_protocol.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../App/Src/host_cmd_vel_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../App/Src/chassis_debug.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../App/Src/chassis_uart_ack.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../App/Src/system_watchdog.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../App/Src/system_health.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../App/Src/board_reliability_io.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../App/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    CHASSIS_DEBUG_ENABLE=$<IF:$<CONFIG:Release>,0,1>
    CHASSIS_UART_ACK_ENABLE=1
    CHASSIS_DEBUG_TEXT_ENABLE=0
    FIRMWARE_VERSION_PACKED=0x00010000UL
)

execute_process(
    COMMAND git -C ${CMAKE_CURRENT_SOURCE_DIR}/../.. rev-parse --short=8 HEAD
    OUTPUT_VARIABLE FIRMWARE_GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
string(LENGTH "${FIRMWARE_GIT_HASH}" FIRMWARE_GIT_HASH_LENGTH)
if((NOT FIRMWARE_GIT_HASH MATCHES "^[0-9a-fA-F]+$") OR
   (NOT FIRMWARE_GIT_HASH_LENGTH EQUAL 8))
    set(FIRMWARE_GIT_HASH "00000000")
endif()
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/../App/Inc/steering_config.h
     FIRMWARE_STEERING_CONFIG)
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/../App/Inc/drive_config.h
     FIRMWARE_DRIVE_CONFIG)
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/../App/Inc/board_reliability_config.h
     FIRMWARE_RELIABILITY_CONFIG)
string(SHA256 FIRMWARE_CONFIG_SHA256
       "${FIRMWARE_STEERING_CONFIG}${FIRMWARE_DRIVE_CONFIG}${FIRMWARE_RELIABILITY_CONFIG}")
string(SUBSTRING ${FIRMWARE_CONFIG_SHA256} 0 8 FIRMWARE_CONFIG_HASH)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    FIRMWARE_GIT_HASH32=0x${FIRMWARE_GIT_HASH}UL
    FIRMWARE_CONFIG_HASH32=0x${FIRMWARE_CONFIG_HASH}UL
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Keep programmer images synchronized with the ELF on every successful link.
get_filename_component(ARM_TOOLCHAIN_BIN_DIR "${CMAKE_C_COMPILER}" DIRECTORY)
set(PROJECT_OBJCOPY "${ARM_TOOLCHAIN_BIN_DIR}/arm-none-eabi-objcopy")
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${PROJECT_OBJCOPY} -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${PROJECT_OBJCOPY} -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating STM32 HEX and BIN images"
)
