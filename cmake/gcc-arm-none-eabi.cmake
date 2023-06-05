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

# Compiler flags --------------------------------------------------------

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fsanitize=alignment")
set(CMAKE_C_FLAGS
    "${CMAKE_C_FLAGS} -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3"
)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-array-bounds -Wno-stringop-overread")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-stringop-overflow")

set(CPP_FLAGS "${CPP_FLAGS} ${CMAKE_C_FLAGS}")
set(CPP_FLAGS "${CPP_FLAGS} -fno-exceptions -fcheck-new -fno-rtti -pedantic ")
set(CPP_FLAGS "${CPP_FLAGS}  -Os -g3")
set(CMAKE_CXX_FLAGS "${CPP_FLAGS} ")

# set(CPU_PARAMETERS -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -mfpu=fpv4-sp-d16)

# Linker flags
set(LINKER_FLAGS
    -specs=nano.specs
    -specs=nosys.specs
    ${CPU_PARAMETERS}
    -nostdlib
    -Wl,-Map=${PROJECT_NAME}.map,--cref,--gc-sections,--undefined=uxTopUsedPriority
    -L${CMAKE_SOURCE_DIR}/tools/make/F405/linker
    # -lc -lm -lnosys -lstdc++ -Wl,--end-group -Wl,--print-memory-usage
    # -Wl,-Map=${PROJECT_NAME}.map,--cref -Wl,--gc-sections
)
