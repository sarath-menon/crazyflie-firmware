set(LIB_NAME src_utils)

list(
  APPEND
  LIB_SRCS
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
  # kve
  ${CF2_SRCS_DIR}/src/utils/src/kve/kve.c
  ${CF2_SRCS_DIR}/src/utils/src/kve/kve_storage.c
  # version
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
