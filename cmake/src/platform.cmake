set(LIB_NAME src_platform)

list(
  APPEND
  LIB_SRCS
  # platform
  ${CF2_SRCS_DIR}/src/platform/src/platform.c
  ${CF2_SRCS_DIR}/src/platform/src/platform_utils.c
  ${CF2_SRCS_DIR}/src/platform/src/platform_stm32f4.c
  ${CF2_SRCS_DIR}/src/platform/src/platform_cf2.c)

add_lib(${LIB_NAME} "${LIB_SRCS}")
