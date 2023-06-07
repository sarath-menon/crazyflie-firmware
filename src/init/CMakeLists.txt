set(EXEC_NAME "cf2")

set(EXECUTABLE ${EXEC_NAME}.elf)

set(APP_SRC_FILES main.c)
# set(APP_INCLUDE_FOLDERS include)

# Executable files
add_executable(${EXECUTABLE} ${APP_SRC_FILES} ${LIB_SRCS} ${STARTUP_SCRIPT})

target_compile_definitions(${EXECUTABLE} PRIVATE ${MCU_DEFINITONS})

target_include_directories(${EXECUTABLE} PUBLIC ${INCLUDE_DIRS})

target_compile_options(
  ${EXECUTABLE}
  PUBLIC ${CPU_PARAMETERS} ${LINKER_FLAGS} # $<$<CONFIG:Debug>:-Og -g3 -ggdb>
         # $<$<CONFIG:Release>:-Og -g0>
)

target_link_options(${EXECUTABLE} PRIVATE ${CPU_PARAMETERS} ${LINKER_FLAGS} -T
                    ${MCU_LINKER_SCRIPT})

target_link_libraries(
  ${EXECUTABLE}
  PRIVATE src_deck
          src_drivers
          src_hal
          src_lib
          src_modules
          src_platform
          # src_utils
          src_freertos
          src_stm32
          src_freertos
          cmsis_dsp)

# Print executable areas
add_custom_command(
  TARGET ${EXECUTABLE}
  POST_BUILD
  COMMAND arm-none-eabi-size ${EXECUTABLE})

# Create hex file
add_custom_command(
  TARGET ${EXECUTABLE}
  POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex ${EXECUTABLE} ${EXEC_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary ${EXECUTABLE} ${EXEC_NAME}.bin)
