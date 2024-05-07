# KBUILD settings
set(KBUILD_OUTPUT_DIR ${CMAKE_SOURCE_DIR}/build)
# set(KCONFIG_ALLCONFIG "configs/all.config")

set(FREERTOS_INCLUDE_DIRS ${FREERTOS_SRC_DIR}/include
                          ${FREERTOS_SRC_DIR}/portable/GCC/ARM_CM4F)

set(CF2_INCLUDE_DIRS
    ${CF2_SRCS_DIR}/vendor/CMSIS/CMSIS/DSP/Include
    ${CF2_SRCS_DIR}/vendor/CMSIS/CMSIS/Core/Include
    ${CF2_SRCS_DIR}/vendor/CMSIS/CMSIS/DSP/Include
    ${CF2_SRCS_DIR}/vendor/libdw1000/inc
    ${CF2_SRCS_DIR}/src/config
    ${CF2_SRCS_DIR}/src/platform/interface
    ${CF2_SRCS_DIR}/src/deck/interface
    ${CF2_SRCS_DIR}/src/deck/drivers/interface
    ${CF2_SRCS_DIR}/src/drivers/interface
    ${CF2_SRCS_DIR}/src/drivers/bosch/interface
    ${CF2_SRCS_DIR}/src/drivers/esp32/interface
    ${CF2_SRCS_DIR}/src/hal/interface
    ${CF2_SRCS_DIR}/src/modules/interface
    ${CF2_SRCS_DIR}/src/modules/interface/kalman_core
    ${CF2_SRCS_DIR}/src/modules/interface/lighthouse
    ${CF2_SRCS_DIR}/src/modules/interface/outlierfilter
    ${CF2_SRCS_DIR}/src/modules/interface/cpx
    ${CF2_SRCS_DIR}/src/modules/interface/p2pDTR
    ${CF2_SRCS_DIR}/src/modules/interface/controller
    ${CF2_SRCS_DIR}/src/modules/interface/estimator
    ${CF2_SRCS_DIR}/src/utils/interface
    ${CF2_SRCS_DIR}/src/utils/interface/kve
    ${CF2_SRCS_DIR}/src/utils/interface/lighthouse
    ${CF2_SRCS_DIR}/src/utils/interface/tdoa
    # stm32 drivers
    ${CF2_SRCS_DIR}/src/lib/STM32F4xx_StdPeriph_Driver/inc
    ${CF2_SRCS_DIR}/src/lib/STM32_USB_OTG_Driver/inc)

set(LIB_INCLUDE_DIRS
    ${LIB_DIR}/FatFS
    ${LIB_DIR}/CMSIS/STM32F4xx/Include
    ${LIB_DIR}/STM32_USB_Device_Library/Core/inc
    ${LIB_DIR}/STM32_USB_OTG_Driver/inc
    ${LIB_DIR}/STM32F4xx_StdPeriph_Driver/inc
    ${LIB_DIR}/vl53l1
    ${LIB_DIR}/vl53l1/core/inc)

set(KBUILD_INCLUDE_DIRS ${KBUILD_OUTPUT_DIR}/include/generated)

set(INCLUDE_DIRS ${CF2_INCLUDE_DIRS} ${FREERTOS_INCLUDE_DIRS}
                 ${LIB_INCLUDE_DIRS} ${KBUILD_INCLUDE_DIRS})
