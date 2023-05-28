cmake_minimum_required(VERSION 3.10)

project(cf_sitl VERSION 1.0.0)

set(EXEC_NAME cf_sitl)

# find thread package
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

set(FREERTOS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../vendor/FreeRTOS)

set(FREERTOS_INCLUDE_DIRS ${FREERTOS_SRC_DIR}/include
                          ${FREERTOS_SRC_DIR}/portable/ThirdParty/GCC/Posix)

set(PROJECT_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(PROJECT_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c)

set(FREERTOS_SRCS
    ${FREERTOS_SRC_DIR}/event_groups.c
    ${FREERTOS_SRC_DIR}/list.c
    ${FREERTOS_SRC_DIR}/queue.c
    ${FREERTOS_SRC_DIR}/tasks.c
    ${FREERTOS_SRC_DIR}/timers.c
    ${FREERTOS_SRC_DIR}/portable/MemMang/heap_3.c
    ${FREERTOS_SRC_DIR}/portable/ThirdParty/GCC/Posix/port.c
    ${FREERTOS_SRC_DIR}/portable/ThirdParty/GCC/Posix/utils/wait_for_event.c)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")

add_executable(${EXEC_NAME} ${PROJECT_SRCS} ${FREERTOS_SRCS})

target_include_directories(${EXEC_NAME} PUBLIC ${PROJECT_INCLUDE_DIRS}
                                               ${FREERTOS_INCLUDE_DIRS})

target_link_libraries(${EXEC_NAME} PRIVATE Threads::Threads)
