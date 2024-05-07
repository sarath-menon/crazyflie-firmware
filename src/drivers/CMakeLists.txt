set(LIB_NAME src_drivers)

list(
  APPEND
  LIB_DRIVERS_SRCS
  # bosch
  ${CF2_SRCS_DIR}/src/drivers/bosch/src/bmi055_accel.c
  ${CF2_SRCS_DIR}/src/drivers/bosch/src/bmi055_gyro.c
  ${CF2_SRCS_DIR}/src/drivers/bosch/src/bmi088_accel.c
  ${CF2_SRCS_DIR}/src/drivers/bosch/src/bmi088_fifo.c
  ${CF2_SRCS_DIR}/src/drivers/bosch/src/bmi088_gyro.c
  ${CF2_SRCS_DIR}/src/drivers/bosch/src/bmi160.c
  ${CF2_SRCS_DIR}/src/drivers/bosch/src/bmm150.c
  ${CF2_SRCS_DIR}/src/drivers/bosch/src/bmp280.c
  ${CF2_SRCS_DIR}/src/drivers/bosch/src/bmp3.c
  ${CF2_SRCS_DIR}/src/drivers/bosch/src/bstdr_comm_support.c
  # esp
  ${CF2_SRCS_DIR}/src/drivers/esp32/src/esp_rom_bootloader.c
  ${CF2_SRCS_DIR}/src/drivers/esp32/src/esp_slip.c
  # the rest
  ${CF2_SRCS_DIR}/src/drivers/src/ak8963.c
  ${CF2_SRCS_DIR}/src/drivers/src/cppm.c
  ${CF2_SRCS_DIR}/src/drivers/src/eeprom.c
  ${CF2_SRCS_DIR}/src/drivers/src/exti.c
  ${CF2_SRCS_DIR}/src/drivers/src/fatfs_sd.c
  ${CF2_SRCS_DIR}/src/drivers/src/i2cdev.c
  ${CF2_SRCS_DIR}/src/drivers/src/i2c_drv.c
  ${CF2_SRCS_DIR}/src/drivers/src/led.c
  ${CF2_SRCS_DIR}/src/drivers/src/lh_bootloader.c
  ${CF2_SRCS_DIR}/src/drivers/src/lps25h.c
  ${CF2_SRCS_DIR}/src/drivers/src/maxsonar.c
  ${CF2_SRCS_DIR}/src/drivers/src/motors.c
  ${CF2_SRCS_DIR}/src/drivers/src/mpu6050.c
  ${CF2_SRCS_DIR}/src/drivers/src/mpu6500.c
  ${CF2_SRCS_DIR}/src/drivers/src/ms5611.c
  ${CF2_SRCS_DIR}/src/drivers/src/nvic.c
  ${CF2_SRCS_DIR}/src/drivers/src/pca9685.c
  ${CF2_SRCS_DIR}/src/drivers/src/piezo.c
  ${CF2_SRCS_DIR}/src/drivers/src/pmw3901.c
  ${CF2_SRCS_DIR}/src/drivers/src/swd.c
  ${CF2_SRCS_DIR}/src/drivers/src/uart1.c
  ${CF2_SRCS_DIR}/src/drivers/src/uart2.c
  ${CF2_SRCS_DIR}/src/drivers/src/uart_syslink.c
  ${CF2_SRCS_DIR}/src/drivers/src/vl53l0x.c
  ${CF2_SRCS_DIR}/src/drivers/src/vl53l1x.c
  ${CF2_SRCS_DIR}/src/drivers/src/watchdog.c
  ${CF2_SRCS_DIR}/src/drivers/src/ws2812_cf2.c)

add_lib(${LIB_NAME} "${LIB_DRIVERS_SRCS}")

target_link_libraries(${LIB_NAME} PRIVATE src_freertos src_utils)
