set(LIB_NAME src_lib)

list(
  APPEND
  LIB_LIB_SRCS
  # ${CF2_SRCS_DIR}/src/lib/Segger_RTT/RTT
  ${CF2_SRCS_DIR}/src/lib/CMSIS/STM32F4xx/Source/system_stm32f4xx.c
  # FatFS
  ${CF2_SRCS_DIR}/src/lib/FatFS/ff.c
  ${CF2_SRCS_DIR}/src/lib/FatFS/ffunicode.c
  # STM32F4xx_StdPeriph_Driver
  ${CF2_SRCS_DIR}/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c
  ${CF2_SRCS_DIR}/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dbgmcu.c
  ${CF2_SRCS_DIR}/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c
  ${CF2_SRCS_DIR}/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c
  ${CF2_SRCS_DIR}/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c
  ${CF2_SRCS_DIR}/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c
  ${CF2_SRCS_DIR}/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c
  ${CF2_SRCS_DIR}/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_iwdg.c
  ${CF2_SRCS_DIR}/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_misc.c
  ${CF2_SRCS_DIR}/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c
  ${CF2_SRCS_DIR}/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c
  ${CF2_SRCS_DIR}/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c
  ${CF2_SRCS_DIR}/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c
  ${CF2_SRCS_DIR}/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c
  # STM32_USB_Device_Library
  ${CF2_SRCS_DIR}/src/lib/STM32_USB_Device_Library/Core/src/usbd_core.c
  ${CF2_SRCS_DIR}/src/lib/STM32_USB_Device_Library/Core/src/usbd_ioreq.c
  ${CF2_SRCS_DIR}/src/lib/STM32_USB_Device_Library/Core/src/usbd_req.c
  # STM32_USB_OTG_Driver
  ${CF2_SRCS_DIR}/src/lib/STM32_USB_OTG_Driver/src/usb_core.c
  ${CF2_SRCS_DIR}/src/lib/STM32_USB_OTG_Driver/src/usb_dcd_int.c
  ${CF2_SRCS_DIR}/src/lib/STM32_USB_OTG_Driver/src/usb_dcd.c
  # vl53l1
  ${CF2_SRCS_DIR}/src/lib/vl53l1/core/src/vl53l1_api_calibration.c
  ${CF2_SRCS_DIR}/src/lib/vl53l1/core/src/vl53l1_api_core.c
  ${CF2_SRCS_DIR}/src/lib/vl53l1/core/src/vl53l1_api_debug.c
  ${CF2_SRCS_DIR}/src/lib/vl53l1/core/src/vl53l1_api.c
  ${CF2_SRCS_DIR}/src/lib/vl53l1/core/src/vl53l1_api_preset_modes.c
  ${CF2_SRCS_DIR}/src/lib/vl53l1/core/src/vl53l1_api_strings.c
  ${CF2_SRCS_DIR}/src/lib/vl53l1/core/src/vl53l1_core.c
  ${CF2_SRCS_DIR}/src/lib/vl53l1/core/src/vl53l1_core_support.c
  ${CF2_SRCS_DIR}/src/lib/vl53l1/core/src/vl53l1_error_strings.c
  ${CF2_SRCS_DIR}/src/lib/vl53l1/core/src/vl53l1_register_funcs.c
  ${CF2_SRCS_DIR}/src/lib/vl53l1/core/src/vl53l1_silicon_core.c
  ${CF2_SRCS_DIR}/src/lib/vl53l1/core/src/vl53l1_wait.c)

add_lib(${LIB_NAME} "${LIB_LIB_SRCS}")
