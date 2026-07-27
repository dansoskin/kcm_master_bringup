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
set(CMAKE_PROJECT_NAME kcm_master_bringup)

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
    # Add user sources here
    ${CMAKE_SOURCE_DIR}/Core/Src/globals.c
    ${CMAKE_SOURCE_DIR}/Core/Src/functions.c
    ${CMAKE_SOURCE_DIR}/Core/Src/decode_packet.c
    ${CMAKE_SOURCE_DIR}/Core/Src/plate_kinematics.c
    ${CMAKE_SOURCE_DIR}/Core/Src/roulette_sm.c

    ${CMAKE_SOURCE_DIR}/Drivers/stm32_uart_dma_idleInt/uart.c
    ${CMAKE_SOURCE_DIR}/Drivers/stm32_uart_dma_idleInt/lwrb.c

    ${CMAKE_SOURCE_DIR}/Drivers/cFlexyStepper/cFlexyStepper.c
    ${CMAKE_SOURCE_DIR}/Drivers/cFlexyStepper/cFlexyStepper_Basic.c
    ${CMAKE_SOURCE_DIR}/Drivers/cFlexyStepper/mcu_specific/cFlexyStepper_stm.c

    ${CMAKE_SOURCE_DIR}/Drivers/canbus_wrapper/can.c
    ${CMAKE_SOURCE_DIR}/Drivers/synchronized_movement/synchronized_movement.c

    ${CMAKE_SOURCE_DIR}/Drivers/odrive_lib/src/odrive_comm.c
    ${CMAKE_SOURCE_DIR}/Drivers/odrive_lib/src/odrive_control.c
    ${CMAKE_SOURCE_DIR}/Drivers/odrive_lib/src/odrive_endpoints_0_6.c
    ${CMAKE_SOURCE_DIR}/Drivers/odrive_lib/src/odrive_feedback.c
    ${CMAKE_SOURCE_DIR}/Drivers/odrive_lib/src/odrive_periodic.c
    ${CMAKE_SOURCE_DIR}/Drivers/odrive_lib/src/odrive_setpoints.c
    ${CMAKE_SOURCE_DIR}/Drivers/odrive_lib/src/odrive_strings.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_SOURCE_DIR}/Drivers/stm32_uart_dma_idleInt
    ${CMAKE_SOURCE_DIR}/Drivers/cFlexyStepper
    ${CMAKE_SOURCE_DIR}/Drivers/canbus_wrapper
    ${CMAKE_SOURCE_DIR}/Drivers/synchronized_movement
    ${CMAKE_SOURCE_DIR}/Drivers/odrive_lib/include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Enable %f (float) formatting in printf/sprintf/vsnprintf.
# The toolchain links --specs=nano.specs, whose newlib-nano strips float support
# from the printf family; this pulls it back in (needed for FlexyStepper_log).
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
