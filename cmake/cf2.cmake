set(FREERTOS_SRC_DIR ${CMAKE_SOURCE_DIR}/vendor/FreeRTOS)
set(CF2_SRCS_DIR ${CMAKE_SOURCE_DIR})
set(LIB_DIR ${CMAKE_SOURCE_DIR}/src/lib)

# KBUILD settings
set(KBUILD_OUTPUT_DIR ${CMAKE_SOURCE_DIR}/build)
# set(KCONFIG_ALLCONFIG "configs/all.config")

set(FREERTOS_INCLUDE_DIRS ${FREERTOS_SRC_DIR}/include
                          ${FREERTOS_SRC_DIR}/portable/GCC/ARM_CM4F)

set(DECK_SRCS
    # api
    ${CF2_SRCS_DIR}/src/deck/api/deck_analog.c
    ${CF2_SRCS_DIR}/src/deck/api/deck_constants.c
    ${CF2_SRCS_DIR}/src/deck/api/deck_digital.c
    ${CF2_SRCS_DIR}/src/deck/api/deck_spi3.c
    ${CF2_SRCS_DIR}/src/deck/api/deck_spi.c
    # core
    ${CF2_SRCS_DIR}/src/deck/core/deck_drivers.c
    ${CF2_SRCS_DIR}/src/deck/core/deck_info.c
    ${CF2_SRCS_DIR}/src/deck/core/deck_memory.c
    ${CF2_SRCS_DIR}/src/deck/core/deck.c
    ${CF2_SRCS_DIR}/src/deck/core/deck_test.c
    # drivers
    ${CF2_SRCS_DIR}/src/deck/drivers/src/activeMarkerDeck.c
    ${CF2_SRCS_DIR}/src/deck/drivers/src/aideck.c
    # ${CF2_SRCS_DIR}/src/deck/drivers/src/bigquad.c
    ${CF2_SRCS_DIR}/src/deck/drivers/src/buzzdeck.c
    # ${CF2_SRCS_DIR}/src/deck/drivers/src/cppmdeck.c
    # ${CF2_SRCS_DIR}/src/deck/drivers/src/flapperdeck.c
    ${CF2_SRCS_DIR}/src/deck/drivers/src/flowdeck_v1v2.c
    # ${CF2_SRCS_DIR}/src/deck/drivers/src/gtgps.c
    ${CF2_SRCS_DIR}/src/deck/drivers/src/ledring12.c
    ${CF2_SRCS_DIR}/src/deck/drivers/src/lighthouse.c
    ${CF2_SRCS_DIR}/src/deck/drivers/src/locodeck.c
    ${CF2_SRCS_DIR}/src/deck/drivers/src/lpsTdoa2Tag.c
    ${CF2_SRCS_DIR}/src/deck/drivers/src/lpsTdoa3Tag.c
    ${CF2_SRCS_DIR}/src/deck/drivers/src/lpsTwrTag.c
    ${CF2_SRCS_DIR}/src/deck/drivers/src/multiranger.c
    ${CF2_SRCS_DIR}/src/deck/drivers/src/oa.c
    ${CF2_SRCS_DIR}/src/deck/drivers/src/usddeck.c
    # ${CF2_SRCS_DIR}/src/deck/drivers/src/zranger.c
    ${CF2_SRCS_DIR}/src/deck/drivers/src/zranger2.c
    # ${CF2_SRCS_DIR}/src/deck/drivers/src/cpx-host-on-uart2.c
)

set(DRIVERS_SRCS
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

set(HAL_SRCS
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

set(LIB_SRCS
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

set(MODULES_SRCS
    ${CF2_SRCS_DIR}/src/modules/src/app_channel.c
    # ${CF2_SRCS_DIR}/src/modules/src/app_handler.c
    ${CF2_SRCS_DIR}/src/modules/src/bootloader.c
    ${CF2_SRCS_DIR}/src/modules/src/collision_avoidance.c
    ${CF2_SRCS_DIR}/src/modules/src/collision_avoidance.c
    ${CF2_SRCS_DIR}/src/modules/src/commander.c
    ${CF2_SRCS_DIR}/src/modules/src/comm.c
    ${CF2_SRCS_DIR}/src/modules/src/console.c
    ${CF2_SRCS_DIR}/src/modules/src/crtp_commander_generic.c
    ${CF2_SRCS_DIR}/src/modules/src/crtp_commander_high_level.c
    ${CF2_SRCS_DIR}/src/modules/src/crtp_commander.c
    ${CF2_SRCS_DIR}/src/modules/src/crtp_commander_rpyt.c
    ${CF2_SRCS_DIR}/src/modules/src/crtp_localization_service.c
    ${CF2_SRCS_DIR}/src/modules/src/crtp.c
    ${CF2_SRCS_DIR}/src/modules/src/crtpservice.c
    ${CF2_SRCS_DIR}/src/modules/src/esp_deck_flasher.c
    ${CF2_SRCS_DIR}/src/modules/src/eventtrigger.c
    ${CF2_SRCS_DIR}/src/modules/src/extrx.c
    ${CF2_SRCS_DIR}/src/modules/src/health.c
    ${CF2_SRCS_DIR}/src/modules/src/kalman_supervisor.c
    ${CF2_SRCS_DIR}/src/modules/src/axis3fSubSampler.c
    ${CF2_SRCS_DIR}/src/modules/src/log.c
    ${CF2_SRCS_DIR}/src/modules/src/mem.c
    ${CF2_SRCS_DIR}/src/modules/src/crtp_mem.c
    ${CF2_SRCS_DIR}/src/modules/src/msp.c
    ${CF2_SRCS_DIR}/src/modules/src/param_logic.c
    ${CF2_SRCS_DIR}/src/modules/src/param_task.c
    ${CF2_SRCS_DIR}/src/modules/src/peer_localization.c
    ${CF2_SRCS_DIR}/src/modules/src/planner.c
    ${CF2_SRCS_DIR}/src/modules/src/platformservice.c
    ${CF2_SRCS_DIR}/src/modules/src/power_distribution_quadrotor.c
    # ${CF2_SRCS_DIR}/src/modules/src/power_distribution_flapper.c
    ${CF2_SRCS_DIR}/src/modules/src/pptraj_compressed.c
    ${CF2_SRCS_DIR}/src/modules/src/pptraj.c
    ${CF2_SRCS_DIR}/src/modules/src/queuemonitor.c
    ${CF2_SRCS_DIR}/src/modules/src/range.c
    ${CF2_SRCS_DIR}/src/modules/src/sensfusion6.c
    ${CF2_SRCS_DIR}/src/modules/src/serial_4way_avrootloader.c
    ${CF2_SRCS_DIR}/src/modules/src/serial_4way.c
    ${CF2_SRCS_DIR}/src/modules/src/sound_cf2.c
    ${CF2_SRCS_DIR}/src/modules/src/stabilizer.cpp
    ${CF2_SRCS_DIR}/src/modules/src/static_mem.c
    ${CF2_SRCS_DIR}/src/modules/src/supervisor.c
    ${CF2_SRCS_DIR}/src/modules/src/supervisor_state_machine.c
    ${CF2_SRCS_DIR}/src/modules/src/sysload.c
    ${CF2_SRCS_DIR}/src/modules/src/system.cpp
    ${CF2_SRCS_DIR}/src/modules/src/tdoaEngineInstance.c
    ${CF2_SRCS_DIR}/src/modules/src/vcp_esc_passthrough.c
    ${CF2_SRCS_DIR}/src/modules/src/worker.c
    # controller
    ${CF2_SRCS_DIR}/src/modules/src/controller/attitude_pid_controller.c
    ${CF2_SRCS_DIR}/src/modules/src/controller/controller_indi.c
    ${CF2_SRCS_DIR}/src/modules/src/controller/controller_mellinger.c
    ${CF2_SRCS_DIR}/src/modules/src/controller/controller.c
    ${CF2_SRCS_DIR}/src/modules/src/controller/controller_pid.c
    ${CF2_SRCS_DIR}/src/modules/src/controller/controller_brescianini.c
    ${CF2_SRCS_DIR}/src/modules/src/controller/position_controller_indi.c
    ${CF2_SRCS_DIR}/src/modules/src/controller/position_controller_pid.c
    # estimator
    ${CF2_SRCS_DIR}/src/modules/src/estimator/estimator_complementary.c
    ${CF2_SRCS_DIR}/src/modules/src/estimator/estimator_kalman.c
    ${CF2_SRCS_DIR}/src/modules/src/estimator/estimator_ukf.c
    ${CF2_SRCS_DIR}/src/modules/src/estimator/estimator.c
    ${CF2_SRCS_DIR}/src/modules/src/estimator/position_estimator_altitude.c
    # kalman core
    ${CF2_SRCS_DIR}/src/modules/src/kalman_core/kalman_core.c
    ${CF2_SRCS_DIR}/src/modules/src/kalman_core/mm_absolute_height.c
    ${CF2_SRCS_DIR}/src/modules/src/kalman_core/mm_distance.c
    ${CF2_SRCS_DIR}/src/modules/src/kalman_core/mm_distance_robust.c
    ${CF2_SRCS_DIR}/src/modules/src/kalman_core/mm_flow.c
    ${CF2_SRCS_DIR}/src/modules/src/kalman_core/mm_pose.c
    ${CF2_SRCS_DIR}/src/modules/src/kalman_core/mm_position.c
    ${CF2_SRCS_DIR}/src/modules/src/kalman_core/mm_sweep_angles.c
    ${CF2_SRCS_DIR}/src/modules/src/kalman_core/mm_tdoa.c
    ${CF2_SRCS_DIR}/src/modules/src/kalman_core/mm_tdoa_robust.c
    ${CF2_SRCS_DIR}/src/modules/src/kalman_core/mm_tof.c
    ${CF2_SRCS_DIR}/src/modules/src/kalman_core/mm_yaw_error.c
    # lighthouse
    ${CF2_SRCS_DIR}/src/modules/src/lighthouse/lighthouse_core.c
    ${CF2_SRCS_DIR}/src/modules/src/lighthouse/lighthouse_deck_flasher.c
    ${CF2_SRCS_DIR}/src/modules/src/lighthouse/lighthouse_position_est.c
    ${CF2_SRCS_DIR}/src/modules/src/lighthouse/lighthouse_storage.c
    ${CF2_SRCS_DIR}/src/modules/src/lighthouse/lighthouse_transmit.c
    ${CF2_SRCS_DIR}/src/modules/src/lighthouse/lighthouse_throttle.c
    # outlier filter
    ${CF2_SRCS_DIR}/src/modules/src/outlierfilter/outlierFilterTdoa.c
    ${CF2_SRCS_DIR}/src/modules/src/outlierfilter/outlierFilterTdoaSteps.c
    ${CF2_SRCS_DIR}/src/modules/src/outlierfilter/outlierFilterLighthouse.c
    # cpx
    ${CF2_SRCS_DIR}/src/modules/src/cpx/cpx_external_router.c
    ${CF2_SRCS_DIR}/src/modules/src/cpx/cpx_internal_router.c
    ${CF2_SRCS_DIR}/src/modules/src/cpx/cpx_uart_transport.c
    ${CF2_SRCS_DIR}/src/modules/src/cpx/cpxlink.c
    ${CF2_SRCS_DIR}/src/modules/src/cpx/cpx.c
    # p2p DTR
    ${CF2_SRCS_DIR}/src/modules/src/p2pDTR/DTR_handlers.c
    ${CF2_SRCS_DIR}/src/modules/src/p2pDTR/DTR_p2p_interface.c
    ${CF2_SRCS_DIR}/src/modules/src/p2pDTR/queueing.c
    ${CF2_SRCS_DIR}/src/modules/src/p2pDTR/token_ring.c
    # platform
    ${CF2_SRCS_DIR}/src/platform/src/platform.c
    ${CF2_SRCS_DIR}/src/platform/src/platform_cf2.c
    # ${CF2_SRCS_DIR}/src/platform/src/platform_bolt.c
    ${CF2_SRCS_DIR}/src/platform/src/platform_stm32f4.c
    ${CF2_SRCS_DIR}/src/platform/src/platform_utils.c
    # utils
    ${CF2_SRCS_DIR}/src/utils/src/cfassert.c
    ${CF2_SRCS_DIR}/src/utils/src/clockCorrectionEngine.c
    ${CF2_SRCS_DIR}/src/utils/src/configblockeeprom.c
    ${CF2_SRCS_DIR}/src/utils/src/cpuid.c
    ${CF2_SRCS_DIR}/src/utils/src/crc32.c
    ${CF2_SRCS_DIR}/src/utils/src/debug.c
    ${CF2_SRCS_DIR}/src/utils/src/eprintf.c
    ${CF2_SRCS_DIR}/src/utils/src/buf2buf.c
    ${CF2_SRCS_DIR}/src/utils/src/filter.c
    ${CF2_SRCS_DIR}/src/utils/src/FreeRTOS-openocd.c
    ${CF2_SRCS_DIR}/src/utils/src/num.c
    ${CF2_SRCS_DIR}/src/utils/src/rateSupervisor.c
    ${CF2_SRCS_DIR}/src/utils/src/sleepus.c
    ${CF2_SRCS_DIR}/src/utils/src/statsCnt.c
    ${CF2_SRCS_DIR}/src/utils/src/abort.c
    ${CF2_SRCS_DIR}/src/utils/src/malloc.c
    ${CF2_SRCS_DIR}/src/utils/src/pid.c
    # utils - kve
    ${CF2_SRCS_DIR}/src/utils/src/kve/kve.c
    ${CF2_SRCS_DIR}/src/utils/src/kve/kve_storage.c
    # utils -version
    ${CF2_SRCS_DIR}/src/utils/src/version_gen.c
    # lighthouse
    ${CF2_SRCS_DIR}/src/utils/src/lighthouse/lighthouse_calibration.c
    ${CF2_SRCS_DIR}/src/utils/src/lighthouse/lighthouse_geometry.c
    ${CF2_SRCS_DIR}/src/utils/src/lighthouse/ootx_decoder.c
    ${CF2_SRCS_DIR}/src/utils/src/lighthouse/pulse_processor.c
    ${CF2_SRCS_DIR}/src/utils/src/lighthouse/pulse_processor_v1.c
    ${CF2_SRCS_DIR}/src/utils/src/lighthouse/pulse_processor_v2.c
    # tdoa
    ${CF2_SRCS_DIR}/src/utils/src/tdoa/tdoaEngine.c
    ${CF2_SRCS_DIR}/src/utils/src/tdoa/tdoaStats.c
    ${CF2_SRCS_DIR}/src/utils/src/tdoa/tdoaStorage.c)

set(FREERTOS_SRCS
    ${FREERTOS_SRC_DIR}/event_groups.c
    ${FREERTOS_SRC_DIR}/list.c
    ${FREERTOS_SRC_DIR}/queue.c
    ${FREERTOS_SRC_DIR}/tasks.c
    ${FREERTOS_SRC_DIR}/timers.c
    ${FREERTOS_SRC_DIR}/stream_buffer.c
    ${FREERTOS_SRC_DIR}/portable/MemMang/heap_4.c
    ${FREERTOS_SRC_DIR}/portable/GCC/ARM_CM4F/port.c
    # libdw1000
    ${CF2_SRCS_DIR}/vendor/libdw1000/src/libdw1000.c
    ${CF2_SRCS_DIR}/vendor/libdw1000/src/libdw1000Spi.c
    # ${FREERTOS_SRC_DIR}/portable/MemMang/heap_3.c
    # ${FREERTOS_SRC_DIR}/portable/ThirdParty/GCC/Posix/port.c
    # ${FREERTOS_SRC_DIR}/portable/ThirdParty/GCC/Posix/utils/src/wait_for_event.c
)
set(STM32_SRCS
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

set(CF2_SRCS
    ${DECK_SRCS}
    ${DRIVERS_SRCS}
    ${HAL_SRCS}
    ${LIB_SRCS}
    ${MODULES_SRCS}
    ${STM32_SRCS}
    ${FREERTOS_SRCS})

# ---------------------------------------------------

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

# # Generate version.c dependencies add_custom_command( OUTPUT
# ${CMAKE_CURRENT_SOURCE_DIR}/../version.c WORKING_DIRECTORY
# ${CMAKE_CURRENT_SOURCE_DIR}/../ COMMAND python
# ${CMAKE_CURRENT_SOURCE_DIR}/../tools/make/versionTemplate.py
# ${CMAKE_CURRENT_SOURCE_DIR}/../src/utils/src/version.vtpl
# ${CMAKE_CURRENT_SOURCE_DIR}/../version.c) add_custom_target(version_cf ALL
# DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../version.c)
