set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(ARM_GCC_ROOT "D:/DevEnv/GNU-tools-for-STM32" CACHE PATH
    "GNU Arm Embedded Toolchain installation directory")

if(EXISTS "${ARM_GCC_ROOT}/bin/arm-none-eabi-gcc.exe")
    set(TOOLCHAIN_BIN "${ARM_GCC_ROOT}/bin")
    set(EXE_SUFFIX ".exe")
else()
    set(TOOLCHAIN_BIN "")
    set(EXE_SUFFIX "")
endif()

set(CMAKE_C_COMPILER   "${TOOLCHAIN_BIN}/arm-none-eabi-gcc${EXE_SUFFIX}")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_BIN}/arm-none-eabi-g++${EXE_SUFFIX}")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_BIN}/arm-none-eabi-gcc${EXE_SUFFIX}")
set(CMAKE_AR           "${TOOLCHAIN_BIN}/arm-none-eabi-ar${EXE_SUFFIX}")
set(CMAKE_OBJCOPY      "${TOOLCHAIN_BIN}/arm-none-eabi-objcopy${EXE_SUFFIX}" CACHE FILEPATH "objcopy")
set(CMAKE_SIZE         "${TOOLCHAIN_BIN}/arm-none-eabi-size${EXE_SUFFIX}" CACHE FILEPATH "size")
