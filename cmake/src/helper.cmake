include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/src/include_dirs.cmake)

function(add_module LIB_NAME LIB_SRCS)

  add_library(${LIB_NAME} STATIC ${LIB_SRCS})

  message(${LIB_SRCS})

  target_include_directories(${LIB_NAME} PUBLIC ${INCLUDE_DIRS})

  target_compile_definitions(${LIB_NAME} PRIVATE ${MCU_DEFINITONS})

  target_compile_options(
    ${LIB_NAME}
    PUBLIC ${CPU_PARAMETERS} ${LINKER_FLAGS} # $<$<CONFIG:Debug>:-Og -g3 -ggdb>
           # $<$<CONFIG:Release>:-Og -g0>
  )

  target_link_options(${LIB_NAME} PRIVATE ${CPU_PARAMETERS} ${LINKER_FLAGS} -T
                      ${MCU_LINKER_SCRIPT})

endfunction()
