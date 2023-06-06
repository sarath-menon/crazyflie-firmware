set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

if(MINGW
   OR CYGWIN
   OR WIN32)
  set(UTIL_SEARCH_CMD where)
elseif(UNIX OR APPLE)
  set(UTIL_SEARCH_CMD which)
endif()

set(COMPILER_PREFIX /usr/local/bin/arm-none-eabi-)

execute_process(
  COMMAND ${UTIL_SEARCH_CMD} ${TOOLCHAIN_PREFIX}gcc
  OUTPUT_VARIABLE BINUTILS_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE)
get_filename_component(ARM_TOOLCHAIN_DIR ${BINUTILS_PATH} DIRECTORY)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_COMPILER ${COMPILER_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${COMPILER_PREFIX}g++)
set(CMAKE_AR ${COMPILER_PREFIX}ar)
set(CMAKE_RANLIB ${COMPILER_PREFIX}ranlib)
set(CMAKE_LINKER ${COMPILER_PREFIX}ld)
set(CMAKE_ASM_COMPILER ${COMPILER_PREFIX}gcc)
set(CMAKE_OBJCOPY ${COMPILER_PREFIX}objcopy)
set(CMAKE_OBJDUMP ${COMPILER_PREFIX}objdump)
set(CMAKE_SIZE ${COMPILER_PREFIX}size)

# Compiler flags --------------------------------------------------------

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2  -std=c11")
set(CMAKE_C_FLAGS
    "${CMAKE_C_FLAGS} -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16"
)
set(CMAKE_C_FLAGS
    "${CMAKE_C_FLAGS} -DARM_MATH_CM4 -D__FPU_PRESENT=1 -ffunction-sections -fdata-sections"
)

# set(CPU_PARAMETERS -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -mfpu=fpv4-sp-d16)

# Linker flags
set(LINKER_FLAGS
    -specs=nano.specs
    -specs=nosys.specs
    ${CPU_PARAMETERS}
    -nostdlib
    -Wl,-Map=${PROJECT_NAME}.map,--cref,--gc-sections,--undefined=uxTopUsedPriority
    # -T${CMAKE_SOURCE_DIR}/tools/make/F405/linker -lc -lm -lnosys -lstdc++
    # -Wl,--end-group -Wl,--print-memory-usage
    # -Wl,-Map=${PROJECT_NAME}.map,--cref -Wl,--gc-sections
)

set(AC_LINKER_FLAGS "--entry=__start -nostdlib -T${MCU_LINKER_SCRIPT}")
