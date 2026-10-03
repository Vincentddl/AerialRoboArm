cmake_minimum_required(VERSION 3.22)

project(VL53L3CX_F103 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MCU_FLAGS -mcpu=cortex-m3 -mthumb -mthumb-interwork)

set(PROJECT_SOURCES
    Core/Startup/startup_stm32f103c8tx.s
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/i2c.c
    Core/Src/usart.c
    Core/Src/stm32f1xx_it.c
    Core/Src/stm32f1xx_hal_msp.c
    Core/Src/system_stm32f1xx.c
    Core/Src/syscalls.c
    App/Src/app.cpp
    Compat/arduino_compat.cpp
    Vendor/ST/vl53lx_class.cpp
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".elf")

target_include_directories(${PROJECT_NAME} PRIVATE
    Core/Inc App/Inc Compat Vendor/ST
    Drivers/STM32F1xx_HAL_Driver/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F1xx/Include
    Drivers/CMSIS/Include)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32F103xB
    ARDUINO_ARCH_STM32)

target_compile_options(${PROJECT_NAME} PRIVATE
    ${MCU_FLAGS}
    -ffunction-sections -fdata-sections -fno-common
    -Wall -Wextra
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti;-fno-threadsafe-statics>
    $<$<CONFIG:Debug>:-Og;-g3>
    $<$<CONFIG:Release>:-Os>)

target_link_options(${PROJECT_NAME} PRIVATE
    ${MCU_FLAGS}
    --specs=nano.specs --specs=nosys.specs
    -T${CMAKE_SOURCE_DIR}/STM32F103C8TX_FLASH.ld
    -Wl,--gc-sections -Wl,--no-warn-rwx-segments
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)

target_link_libraries(${PROJECT_NAME} PRIVATE c m)

set(HEX_FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex")
set(BIN_FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin")
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${HEX_FILE}
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${BIN_FILE}
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Generating HEX/BIN and printing image size")

find_program(OPENOCD_EXECUTABLE NAMES openocd
    HINTS "D:/DevEnv/openocd-v0.12.0-i686-w64-mingw32/bin")
if(OPENOCD_EXECUTABLE)
    add_custom_target(flash
        COMMAND ${OPENOCD_EXECUTABLE} -f ${CMAKE_SOURCE_DIR}/VL53L3CX_F103_STLinkV2.cfg
                -c "program $<TARGET_FILE:${PROJECT_NAME}> verify reset exit"
        DEPENDS ${PROJECT_NAME} USES_TERMINAL)
    add_custom_target(reset
        COMMAND ${OPENOCD_EXECUTABLE} -f ${CMAKE_SOURCE_DIR}/VL53L3CX_F103_STLinkV2.cfg
                -c "init; reset run; shutdown" USES_TERMINAL)
else()
    message(WARNING "OpenOCD not found: flash/reset targets are unavailable")
endif()
