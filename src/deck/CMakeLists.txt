set(LIB_NAME src_deck)

list(
  APPEND
  LIB_SRCS
  ${CF2_SRCS_DIR}/src/deck/api/deck_analog.c
  ${CF2_SRCS_DIR}/src/deck/api/deck_constants.c
  ${CF2_SRCS_DIR}/src/deck/api/deck_digital.c
  ${CF2_SRCS_DIR}/src/deck/api/deck_spi3.c
  ${CF2_SRCS_DIR}/src/deck/api/deck_spi.c # core
  ${CF2_SRCS_DIR}/src/deck/core/deck_drivers.c
  ${CF2_SRCS_DIR}/src/deck/core/deck_info.c
  ${CF2_SRCS_DIR}/src/deck/core/deck_memory.c
  ${CF2_SRCS_DIR}/src/deck/core/deck.c
  ${CF2_SRCS_DIR}/src/deck/core/deck_test.c
  # drivers ${CF2_SRCS_DIR}/src/deck/drivers/src/activeMarkerDeck.c
  ${CF2_SRCS_DIR}/src/deck/drivers/src/aideck.c #
  ${CF2_SRCS_DIR}/src/deck/drivers/src/bigquad.c
  ${CF2_SRCS_DIR}/src/deck/drivers/src/buzzdeck.c #
  ${CF2_SRCS_DIR}/src/deck/drivers/src/cppmdeck.c #
  ${CF2_SRCS_DIR}/src/deck/drivers/src/flapperdeck.c
  ${CF2_SRCS_DIR}/src/deck/drivers/src/flowdeck_v1v2.c #
  ${CF2_SRCS_DIR}/src/deck/drivers/src/gtgps.c
  ${CF2_SRCS_DIR}/src/deck/drivers/src/ledring12.c
  ${CF2_SRCS_DIR}/src/deck/drivers/src/lighthouse.c
  ${CF2_SRCS_DIR}/src/deck/drivers/src/locodeck.c
  ${CF2_SRCS_DIR}/src/deck/drivers/src/lpsTdoa2Tag.c
  ${CF2_SRCS_DIR}/src/deck/drivers/src/lpsTdoa3Tag.c
  ${CF2_SRCS_DIR}/src/deck/drivers/src/lpsTwrTag.c
  ${CF2_SRCS_DIR}/src/deck/drivers/src/multiranger.c
  ${CF2_SRCS_DIR}/src/deck/drivers/src/oa.c
  ${CF2_SRCS_DIR}/src/deck/drivers/src/usddeck.c #
  ${CF2_SRCS_DIR}/src/deck/drivers/src/zranger.c
  ${CF2_SRCS_DIR}/src/deck/drivers/src/zranger2.c #
  ${CF2_SRCS_DIR}/src/deck/drivers/src/cpx-host-on-uart2.c)

add_lib(${LIB_NAME} "${LIB_SRCS}")

target_link_libraries(${LIB_NAME} PRIVATE src_modules src_utils)
