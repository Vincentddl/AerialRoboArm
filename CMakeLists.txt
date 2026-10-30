#此文件从模板自动生成! 请勿更改!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 4.0)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(FOC_DEMO2 C CXX ASM)
option(SERVO1_ONLY_TEST "Build isolated PA0 PWM servo test without HX8 control" OFF)
option(JY60_UART_DEMO "Read JY60 on former ELRS USART1, all actuator tasks disabled" OFF)
option(HX8_JY60_CAPTURE "Timestamped VOFA HX8 motion and JY60 capture experiment" OFF)
option(JY61P_CAPTURE "Use JY61P 200 Hz on USART1 in the joint capture experiment" ON)
option(JY61P_DIAGNOSTIC "Isolate JY61P UART diagnosis; no actuator commands" OFF)
if(JY61P_DIAGNOSTIC)
    if(NOT HX8_JY60_CAPTURE OR NOT JY61P_CAPTURE)
        message(FATAL_ERROR "JY61P_DIAGNOSTIC requires HX8_JY60_CAPTURE and JY61P_CAPTURE")
    endif()
    add_compile_definitions(JY61P_DIAGNOSTIC=1)
endif()
if(HX8_JY60_CAPTURE)
    if(JY60_UART_DEMO OR SERVO1_ONLY_TEST)
        message(FATAL_ERROR "Select only one standalone experiment")
    endif()
    add_compile_definitions(HX8_JY60_CAPTURE=1)
    if(JY61P_CAPTURE)
        add_compile_definitions(JY61P_CAPTURE=1)
    endif()
endif()
if(JY60_UART_DEMO)
    if(SERVO1_ONLY_TEST)
        message(FATAL_ERROR "JY60_UART_DEMO and SERVO1_ONLY_TEST are mutually exclusive")
    endif()
    add_compile_definitions(JY60_UART_DEMO=1)
endif()
option(AUTO_GRIP_THREE_FRAME "AUTO opens gripper then closes after three stable observations" OFF)
if(AUTO_GRIP_THREE_FRAME)
    add_compile_definitions(TASK_MANIP_AUTO_GRIP_THREE_FRAME=1)
endif()
if(SERVO1_ONLY_TEST)
    add_compile_definitions(APP_CONTROL_SERVO1_ONLY=1 BSP_PWM_ENABLE_SERVO_2=0)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
#add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
#add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
#add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m3 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
# newlib-nano: drop ~7-8 KB of float / long-long printf code that nothing
# in User/ actually uses. Must be passed at compile *and* link time.
add_compile_options(--specs=nano.specs)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

include_directories(Core/Inc Drivers/STM32F1xx_HAL_Driver/Inc/Legacy Drivers/STM32F1xx_HAL_Driver/Inc Middlewares/Third_Party/FreeRTOS/Source/include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 Drivers/CMSIS/Device/ST/STM32F1xx/Include Drivers/CMSIS/Include
        # ===== User module include roots =====
        User/global/Inc User/bsp/Inc User/drv/Inc User/mod/Inc User/task/Inc User/app/Inc)

add_definitions(-DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB)

file(GLOB_RECURSE SOURCES "Core/*.*" "Middlewares/*.*" "Drivers/*.*" "User/*.*")

# ===== demo_v7: exclude legacy code from build =====
list(FILTER SOURCES EXCLUDE REGEX ".*User/archive/.*$")
list(FILTER SOURCES EXCLUDE REGEX ".*test_console_drvtest\\.c$")
list(FILTER SOURCES EXCLUDE REGEX ".*app_threads\\.(c|h)$")
list(FILTER SOURCES EXCLUDE REGEX ".*app_testbench\\.(c|h)$")
list(FILTER SOURCES EXCLUDE REGEX ".*drv_vl53l0x\\.(c|h)$")
list(FILTER SOURCES EXCLUDE REGEX ".*tests/.*$")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F103C8TX_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m3 -mthumb -mthumb-interwork)
add_link_options(--specs=nano.specs)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
