set(FREERTOS_SRC_DIR ${CMAKE_SOURCE_DIR}/vendor/FreeRTOS)
set(CF2_SRCS_DIR ${CMAKE_SOURCE_DIR})
set(LIB_DIR ${CMAKE_SOURCE_DIR}/src/lib)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/src/helper.cmake)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/src/utils.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/src/stm32.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/src/cmsis_dsp.cmake)
