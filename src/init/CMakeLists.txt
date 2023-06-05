set(EXEC_NAME "cf2")

set(EXECUTABLE ${EXEC_NAME}.elf)

set(APP_SRC_FILES main.c)
# set(APP_INCLUDE_FOLDERS include)

set(MCU_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/tools/make/F405/linker/FLASH_CLOAD.ld)

# Executable files
add_executable(${EXECUTABLE} ${NRF_SDK_SRC_FILES} ${APP_SRC_FILES}
                             ${STARTUP_SCRIPT})

# MCU parameters
target_compile_definitions(${EXECUTABLE} PRIVATE ${MCU_DEFINITONS})

target_include_directories(${EXECUTABLE} PUBLIC ${INCLUDE_DIRS})

target_compile_options(
  ${EXECUTABLE}
  PRIVATE ${CPU_PARAMETERS}
          # $<$<CONFIG:Debug>:-Og -g3 -ggdb> $<$<CONFIG:Release>:-Og -g0>
)

target_link_options(${EXECUTABLE} PRIVATE -T${MCU_LINKER_SCRIPT}
                    ${CPU_PARAMETERS} ${LINKER_FLAGS})

# target_link_libraries(${EXECUTABLE} PRIVATE st_lib custom_drivers )

# Print executable areas
add_custom_command(
  TARGET ${EXECUTABLE}
  POST_BUILD
  COMMAND arm-none-eabi-size ${EXECUTABLE})

# Create hex file
add_custom_command(
  TARGET ${EXECUTABLE}
  POST_BUILD
  COMMAND arm-none-eabi-objcopy -O ihex ${EXECUTABLE} ${EXEC_NAME}.hex
  COMMAND arm-none-eabi-objcopy -O binary ${EXECUTABLE} ${EXEC_NAME}.bin)
