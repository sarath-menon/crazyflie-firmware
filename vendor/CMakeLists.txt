include(cmake/gcc-arm-none-eabi.cmake)
include(cmake/cmsis_dsp.cmake)
# include(cmake/board.cmake)

add_library(arm_math STATIC ${CMSIS_DSP_SRCS})
target_include_directories(arm_math PUBLIC ${CMSIS_DSP_INCLUDE_DIRS})
