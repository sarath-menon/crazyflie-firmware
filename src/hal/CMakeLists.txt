set(LIB_NAME src_hal)

list(
  APPEND
  LIB_SRCS
  ${CF2_SRCS_DIR}/src/hal/src/amg8833.c
  ${CF2_SRCS_DIR}/src/hal/src/buzzer.c
  ${CF2_SRCS_DIR}/src/hal/src/freeRTOSdebug.c
  ${CF2_SRCS_DIR}/src/hal/src/ledseq.c
  ${CF2_SRCS_DIR}/src/hal/src/ow_common.c
  ${CF2_SRCS_DIR}/src/hal/src/ow_syslink.c
  ${CF2_SRCS_DIR}/src/hal/src/pca9555.c
  ${CF2_SRCS_DIR}/src/hal/src/pca95x4.c
  ${CF2_SRCS_DIR}/src/hal/src/pm_stm32f4.c
  ${CF2_SRCS_DIR}/src/hal/src/proximity.c
  ${CF2_SRCS_DIR}/src/hal/src/radiolink.c
  ${CF2_SRCS_DIR}/src/hal/src/sensors_bmi088_bmp388.c
  ${CF2_SRCS_DIR}/src/hal/src/sensors_bmi088_i2c.c
  ${CF2_SRCS_DIR}/src/hal/src/sensors_bmi088_spi.c
  ${CF2_SRCS_DIR}/src/hal/src/sensors_bosch.c
  ${CF2_SRCS_DIR}/src/hal/src/sensors_mpu9250_lps25h.c
  ${CF2_SRCS_DIR}/src/hal/src/sensors.c
  ${CF2_SRCS_DIR}/src/hal/src/storage.c
  ${CF2_SRCS_DIR}/src/hal/src/syslink.c
  ${CF2_SRCS_DIR}/src/hal/src/usb_bsp.c
  ${CF2_SRCS_DIR}/src/hal/src/usbd_desc.c
  ${CF2_SRCS_DIR}/src/hal/src/usblink.c
  ${CF2_SRCS_DIR}/src/hal/src/usb.c
  ${CF2_SRCS_DIR}/src/hal/src/usec_time.c
  ${CF2_SRCS_DIR}/src/hal/src/amg8833.c
  ${CF2_SRCS_DIR}/src/hal/src/buzzer.c
  ${CF2_SRCS_DIR}/src/hal/src/freeRTOSdebug.c
  ${CF2_SRCS_DIR}/src/hal/src/ledseq.c
  ${CF2_SRCS_DIR}/src/hal/src/ow_common.c
  ${CF2_SRCS_DIR}/src/hal/src/ow_syslink.c
  ${CF2_SRCS_DIR}/src/hal/src/pca9555.c
  ${CF2_SRCS_DIR}/src/hal/src/pca95x4.c
  ${CF2_SRCS_DIR}/src/hal/src/pm_stm32f4.c
  ${CF2_SRCS_DIR}/src/hal/src/proximity.c
  ${CF2_SRCS_DIR}/src/hal/src/radiolink.c
  ${CF2_SRCS_DIR}/src/hal/src/sensors_bmi088_bmp388.c
  ${CF2_SRCS_DIR}/src/hal/src/sensors_bmi088_i2c.c
  ${CF2_SRCS_DIR}/src/hal/src/sensors_bmi088_spi.c
  ${CF2_SRCS_DIR}/src/hal/src/sensors_bosch.c
  ${CF2_SRCS_DIR}/src/hal/src/sensors_mpu9250_lps25h.c
  ${CF2_SRCS_DIR}/src/hal/src/sensors.c
  ${CF2_SRCS_DIR}/src/hal/src/storage.c
  ${CF2_SRCS_DIR}/src/hal/src/syslink.c
  ${CF2_SRCS_DIR}/src/hal/src/usb_bsp.c
  ${CF2_SRCS_DIR}/src/hal/src/usbd_desc.c
  ${CF2_SRCS_DIR}/src/hal/src/usblink.c
  ${CF2_SRCS_DIR}/src/hal/src/usb.c
  ${CF2_SRCS_DIR}/src/hal/src/usec_time.c)

add_lib(${LIB_NAME} "${LIB_SRCS}")

target_link_libraries(${LIB_NAME} PRIVATE src_deck)
