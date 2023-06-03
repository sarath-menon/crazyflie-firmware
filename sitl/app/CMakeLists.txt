cmake_minimum_required(VERSION 3.10)

project(cf_sitl VERSION 1.0.0)

set(EXEC_NAME cf_sitl)

set(APP_INCLUDE_DIRS include)

set(APP_SRCS src/main.c)

add_executable(${EXEC_NAME} ${APP_SRCS} ${FREERTOS_SRCS} ${CF2_SRCS})

target_include_directories(
  ${EXEC_NAME} PUBLIC ${APP_INCLUDE_DIRS} ${FREERTOS_INCLUDE_DIRS}
                      ${CF2_INCLUDE_DIRS})

target_link_libraries(${EXEC_NAME} PRIVATE Threads::Threads)

target_compile_options(${EXEC_NAME} PUBLIC -DSITL_CF2 -DCRAZYFLIE_FW)
