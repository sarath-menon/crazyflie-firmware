set(FREERTOS_SRC_DIR ${CMAKE_SOURCE_DIR}/vendor/FreeRTOS)
set(CF2_SRCS_DIR ${CMAKE_SOURCE_DIR})
set(LIB_DIR ${CMAKE_SOURCE_DIR}/src/lib)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/src/helper.cmake)

# libraries
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/src/deck.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/src/drivers.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/src/hal.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/src/lib.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/src/modules.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/src/freertos.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/src/stm32.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/src/cmsis_dsp.cmake)
