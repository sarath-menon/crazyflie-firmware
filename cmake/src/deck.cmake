include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/src/include_dirs.cmake)

set(LIB_NAME src_deck)

add_library(
  ${LIB_NAME} STATIC
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

target_include_directories(${LIB_NAME} PUBLIC ${INCLUDE_DIRS})

target_compile_definitions(${LIB_NAME} PRIVATE ${MCU_DEFINITONS})

target_compile_options(
  ${LIB_NAME}
  PUBLIC ${CPU_PARAMETERS} ${LINKER_FLAGS} # $<$<CONFIG:Debug>:-Og -g3 -ggdb>
         # $<$<CONFIG:Release>:-Og -g0>
)

target_link_options(${LIB_NAME} PRIVATE ${CPU_PARAMETERS} ${LINKER_FLAGS})
set(CMAKE_EXE_LINKER_FLAGS "-T ${MCU_LINKER_SCRIPT}")
