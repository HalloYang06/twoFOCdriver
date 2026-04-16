cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME h7FOC)

include("cmake/gcc-arm-none-eabi.cmake")
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C ASM)

add_executable(${CMAKE_PROJECT_NAME}
        MC/mc_foc.c
        MC/axis.c

        BSP/bsp_pwm.c
        BSP/bsp_current_sense.c
        BSP/bsp_encoder_abz.c
        BSP/bsp_tamagawa_uart.c
        BSP/bsp_uart.c
        BSP/bsp_can.c
        BSP/bsp_lan9252.c

        libraries/control/pid.c
        libraries/math/mc_math.c
        libraries/math/mc_sin_table.c
        libraries/foc/foc_sincos.c
        libraries/foc/svpwm.c
        libraries/filter/filter_fo.c
        libraries/filter/filter_so.c
        libraries/filter/filter_notch.c
        libraries/filter/filter_offset.c

        comm/debug/comm_vofa.c
        comm/link/comm_can.c
        comm/link/comm_eth.c
        comm/protocol/comm_modbus.c
        comm/protocol/comm_od.c
        comm/protocol/comm_od_core.c
        comm/protocol/comm_can_ctrl.c
        comm/protocol/comm_ecat_if.c
        comm/Eth_stack/SPIDriver.c
        comm/Eth_stack/SPIDriver_port.c
        comm/Eth_stack/lan9252_port_stm32h7.c
        comm/Eth_stack/9252_HW.c
        comm/Eth_stack/ecatslv.c
        comm/Eth_stack/ecatappl.c
        comm/Eth_stack/coeappl.c
        comm/Eth_stack/ecatcoe.c
        comm/Eth_stack/mailbox.c
        comm/Eth_stack/sdoserv.c
        comm/Eth_stack/objdef.c
        comm/Eth_stack/cia402appl.c

        app/app_main.c
        app/app_axis.c
        app/app_comm.c

        Core/Src/spi.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c
)

set_source_files_properties(
        comm/Eth_stack/SPIDriver.c
        comm/Eth_stack/SPIDriver_port.c
        comm/Eth_stack/lan9252_port_stm32h7.c
        comm/Eth_stack/9252_HW.c
        comm/Eth_stack/ecatslv.c
        comm/Eth_stack/ecatappl.c
        comm/Eth_stack/coeappl.c
        comm/Eth_stack/ecatcoe.c
        comm/Eth_stack/mailbox.c
        comm/Eth_stack/sdoserv.c
        comm/Eth_stack/objdef.c
        comm/Eth_stack/cia402appl.c
        PROPERTIES COMPILE_OPTIONS "-Os"
)

add_subdirectory(cmake/stm32cubemx)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

set(CMSISCORE ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Core)
add_subdirectory(Drivers/CMSIS-DSP)

target_link_libraries(${CMAKE_PROJECT_NAME}
    CMSISDSP
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Core/Include
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS-DSP/Include
        ${CMAKE_SOURCE_DIR}/BSP
        ${CMAKE_SOURCE_DIR}/libraries/control
        ${CMAKE_SOURCE_DIR}/libraries/math
        ${CMAKE_SOURCE_DIR}/libraries/foc
        ${CMAKE_SOURCE_DIR}/libraries/filter
        ${CMAKE_SOURCE_DIR}/MC
        ${CMAKE_SOURCE_DIR}/comm
        ${CMAKE_SOURCE_DIR}/comm/debug
        ${CMAKE_SOURCE_DIR}/comm/link
        ${CMAKE_SOURCE_DIR}/comm/protocol
        ${CMAKE_SOURCE_DIR}/comm/Eth_stack
        ${CMAKE_SOURCE_DIR}/app
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Os)
endif()

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
