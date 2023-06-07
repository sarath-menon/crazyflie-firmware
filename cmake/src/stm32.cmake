set(LIB_NAME src_stm32)

list(
  APPEND
  LIB_SRCS
  # std peripheral drivers
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
  # USB driver
  ${CF2_SRCS_DIR}/src/lib/STM32_USB_OTG_Driver/src/usb_core.c
  ${CF2_SRCS_DIR}/src/lib/STM32_USB_OTG_Driver/src/usb_dcd_int.c
  ${CF2_SRCS_DIR}/src/lib/STM32_USB_OTG_Driver/src/usb_dcd.c)

add_lib(${LIB_NAME} "${LIB_SRCS}")
