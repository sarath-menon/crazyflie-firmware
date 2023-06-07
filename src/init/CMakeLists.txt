set(EXEC_NAME "cf2")

set(EXECUTABLE ${EXEC_NAME}.elf)

set(APP_SRC_FILES main.c)
# set(APP_INCLUDE_FOLDERS include)

# Executable files
add_executable(${EXECUTABLE} ${APP_SRC_FILES} ${CF2_SRCS} ${CMSIS_DSP_SRCS}
                             ${STARTUP_SCRIPT})

target_compile_definitions(${EXECUTABLE} PRIVATE ${MCU_DEFINITONS})

target_include_directories(${EXECUTABLE} PUBLIC ${INCLUDE_DIRS}
                                                ${CMSIS_DSP_INCLUDE_DIRS})

target_compile_options(
  ${EXECUTABLE}
  PUBLIC ${CPU_PARAMETERS} ${LINKER_FLAGS} # $<$<CONFIG:Debug>:-Og -g3 -ggdb>
         # $<$<CONFIG:Release>:-Og -g0>
)

target_link_options(${EXECUTABLE} PRIVATE ${CPU_PARAMETERS} ${LINKER_FLAGS})
set(CMAKE_EXE_LINKER_FLAGS "-T ${MCU_LINKER_SCRIPT}")

target_link_libraries(${EXECUTABLE} PRIVATE src_deck)

# target_link_options( ${EXECUTABLE} PRIVATE "SHELL:-T
# ${CMAKE_SOURCE_DIR}/tools/make/F405/linker/FLASH.ld")

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
