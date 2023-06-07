set(LIB_NAME src_freertos)

list(
  APPEND
  LIB_SRCS
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

add_lib(${LIB_NAME} "${LIB_SRCS}")
