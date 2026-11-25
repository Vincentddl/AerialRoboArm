cmake_minimum_required(VERSION 3.20)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

project(HSE_TEST C ASM)

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

add_executable(hse_test.elf
    hse_test.c
    startup_stm32f103_hse_test.s
)

target_include_directories(hse_test.elf PRIVATE
    "${REPO_ROOT}/Drivers/CMSIS/Device/ST/STM32F1xx/Include"
    "${REPO_ROOT}/Drivers/CMSIS/Include"
)

target_compile_definitions(hse_test.elf PRIVATE STM32F103xB)

target_compile_options(hse_test.elf PRIVATE
    -mcpu=cortex-m3
    -mthumb
    -Os
    -ffreestanding
    -fno-builtin
    -ffunction-sections
    -fdata-sections
    -Wall
    -Wextra
)

target_link_options(hse_test.elf PRIVATE
    -mcpu=cortex-m3
    -mthumb
    -nostdlib
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/hse_test.map
    -T${CMAKE_CURRENT_LIST_DIR}/hse_test.ld
)

add_custom_command(TARGET hse_test.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:hse_test.elf>
            ${CMAKE_CURRENT_BINARY_DIR}/hse_test.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:hse_test.elf>
            ${CMAKE_CURRENT_BINARY_DIR}/hse_test.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:hse_test.elf>
    COMMENT "Generating hse_test.hex and hse_test.bin"
)
