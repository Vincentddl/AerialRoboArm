cmake_minimum_required(VERSION 3.20)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
project(HC13_RTT_ONLY C ASM)
get_filename_component(REPO "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
add_executable(hc13_rtt.elf main.c
    "${REPO}/Core/Src/system_stm32f1xx.c"
    "${REPO}/Core/Startup/startup_stm32f103c8tx.s"
    "${REPO}/User/bsp/Src/SEGGER_RTT.c")
target_include_directories(hc13_rtt.elf PRIVATE
    "${REPO}/Drivers/CMSIS/Include"
    "${REPO}/Drivers/CMSIS/Device/ST/STM32F1xx/Include"
    "${REPO}/User/bsp/Inc")
target_compile_definitions(hc13_rtt.elf PRIVATE STM32F103xB)
target_compile_options(hc13_rtt.elf PRIVATE -mcpu=cortex-m3 -mthumb -Os -g
    -ffunction-sections -fdata-sections --specs=nano.specs -Wall -Wextra)
target_link_options(hc13_rtt.elf PRIVATE -mcpu=cortex-m3 -mthumb
    --specs=nano.specs --specs=nosys.specs
    "-T${REPO}/STM32F103C8TX_FLASH.ld" -Wl,--gc-sections,--print-memory-usage
    "-Wl,-Map=${CMAKE_BINARY_DIR}/hc13_rtt.map")
add_custom_command(TARGET hc13_rtt.elf POST_BUILD
    COMMAND arm-none-eabi-objcopy -Obinary $<TARGET_FILE:hc13_rtt.elf> "${CMAKE_BINARY_DIR}/hc13_rtt.bin")
