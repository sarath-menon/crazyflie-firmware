set(LIB_NAME cmsis_dsp)

include(
  /Users/sarathmenon/Documents/eth_projects/freertos_sim/crazyflie-firmware/vendor/cmake/cmsis_dsp_.cmake
)

add_library(${LIB_NAME} STATIC ${CMSIS_DSP_SRCS})

message(${LIB_SRCS})

target_include_directories(${LIB_NAME} PUBLIC ${CMSIS_DSP_INCLUDE_DIRS})

target_compile_definitions(${LIB_NAME} PRIVATE ${MCU_DEFINITONS})

target_compile_options(
  ${LIB_NAME}
  PUBLIC ${CPU_PARAMETERS} ${LINKER_FLAGS} # $<$<CONFIG:Debug>:-Og -g3 -ggdb>
         # $<$<CONFIG:Release>:-Og -g0>
)

target_link_options(${LIB_NAME} PRIVATE ${CPU_PARAMETERS} ${LINKER_FLAGS} -T
                    ${MCU_LINKER_SCRIPT})
