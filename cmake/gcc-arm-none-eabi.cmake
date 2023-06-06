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
set(CMAKE_SIZE ${COMPILER_PREFIX}strip)

set(CMAKE_BUILD_TYPE "MinSizeRel")

# Compiler flags --------------------------------------------------------

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fdata-sections -ffunction-sections")
set(CMAKE_C_FLAGS
    "${CMAKE_C_FLAGS} -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3"
)
set(CMAKE_C_FLAGS
    "${CMAKE_C_FLAGS} -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee"
)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-array-bounds -Wno-stringop-overread")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-stringop-overflow ")
set(CMAKE_C_FLAGS
    "${CMAKE_C_FLAGS} -Wmissing-braces -fno-strict-aliasing ${C_PROFILE}  -Wdouble-promotion "
)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_C_FLAGS}") set(CMAKE_CXX_FLAGS
# "${CMAKE_CXX_FLAGS} -fno-exceptions -fcheck-new -fno-rtti -pedantic ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -g3") set(CMAKE_CXX_FLAGS
# "${CMAKE_CXX_FLAGS} -fno-exceptions -fno-rtti -Wsuggest-override")

# set(CPU_PARAMETERS -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -mfpu=fpv4-sp-d16)

# Linker flags
set(LINKER_FLAGS
    -specs=nano.specs
    -specs=nosys.specs
    ${CPU_PARAMETERS}
    -nostdlib
    -Wl,-Map=cf2_cmake.map,--cref,--gc-sections,--undefined=uxTopUsedPriority
    -lc
    -lm
    -lnosys
    -Wl,--print-memory-usage
    # -lstdc++
    $<$<COMPILE_LANGUAGE:CXX>:-nostdinc++
    >
    # -Wl,--end-group -Wl,--print-memory-usage
    # -Wl,-Map=${PROJECT_NAME}.map,--cref -Wl,--gc-sections
)
