set(LIB_NAME modules_lib)

list(
  APPEND
  LIB_SRCS
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

add_lib(${LIB_NAME} "${LIB_SRCS}")
