cmake_minimum_required(VERSION 3.10)

project(cf_sitl VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

# find thread package
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

set(FREERTOS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../vendor/FreeRTOS)
set(CF2_SRCS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../)

set(FREERTOS_INCLUDE_DIRS ${FREERTOS_SRC_DIR}/include
                          ${FREERTOS_SRC_DIR}/portable/ThirdParty/GCC/Posix)

set(FREERTOS_SRCS
    ${FREERTOS_SRC_DIR}/event_groups.c
    ${FREERTOS_SRC_DIR}/list.c
    ${FREERTOS_SRC_DIR}/queue.c
    ${FREERTOS_SRC_DIR}/tasks.c
    ${FREERTOS_SRC_DIR}/timers.c
    ${FREERTOS_SRC_DIR}/portable/MemMang/heap_3.c
    ${FREERTOS_SRC_DIR}/portable/ThirdParty/GCC/Posix/port.c
    ${FREERTOS_SRC_DIR}/portable/ThirdParty/GCC/Posix/utils/wait_for_event.c)

set(CF2_INCLUDE_DIRS
    # ${CF2_SRCS_DIR}/src/hal/interface
    # ${CF2_SRCS_DIR}/platform
    # ${CF2_SRCS_DIR}/src/utils/interface
    ${CF2_SRCS_DIR}/src/utils/interface
    # ${CF2_SRCS_DIR}/src/deck/interface #
    # ${CF2_SRCS_DIR}/src/deck/drivers/interface
    ${CF2_SRCS_DIR}/src/config
    ${CF2_SRCS_DIR}/src/modules/interface
    # ${CF2_SRCS_DIR}/src/modules/interface/controller
    # ${CF2_SRCS_DIR}/src/modules/interface/estimator
    ${CF2_SRCS_DIR}/src/hal/interface
    ${CF2_SRCS_DIR}/src/modules/interface/estimator
    ${CF2_SRCS_DIR}/src/utils/interface/lighthouse
    # ${CF2_SRCS_DIR}/src/utils/interface ${CF2_SRCS_DIR}/src/config
    # ${CF2_SRCS_DIR}/src/drivers/interface
    # ${CF2_SRCS_DIR}/src/platform/interface
    # ${CF2_SRCS_DIR}/vendor/CMSIS/CMSIS/DSP/Include
    # ${CF2_SRCS_DIR}/vendor/CMSIS/CMSIS/Core/Include
    ${CF2_SRCS_DIR}/build/include/generated)

set(CF2_SRCS
    # Mine
    # ${CF2_SRCS_DIR}/src/hal/src/usec_time_sim.c
    # Original
    # ${CF2_SRCS_DIR}/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_add_f32
    # ${CF2_SRCS_DIR}/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_dot_prod_f32
    # ${CF2_SRCS_DIR}/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_scale_f32
    # ${CF2_SRCS_DIR}/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_sub_f32
    # ${CF2_SRCS_DIR}/vendor/CMSIS/CMSIS/DSP/Source/CommonTables/arm_common_tables
    # ${CF2_SRCS_DIR}/vendor/CMSIS/CMSIS/DSP/Source/FastMathFunctions/arm_cos_f32
    # ${CF2_SRCS_DIR}/vendor/CMSIS/CMSIS/DSP/Source/FastMathFunctions/arm_sin_f32
    # ${CF2_SRCS_DIR}/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_power_f32
    # ${CF2_SRCS_DIR}/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32
    # ${CF2_SRCS_DIR}/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f32
    # ${CF2_SRCS_DIR}/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32
    # ${CF2_SRCS_DIR}/src/modules/src/pptraj.c
    # ${CF2_SRCS_DIR}/src/modules/src/pptraj_compressed.c
    # ${CF2_SRCS_DIR}/src/modules/src/planner.c
    # ${CF2_SRCS_DIR}/src/modules/src/collision_avoidance.c
    ${CF2_SRCS_DIR}/src/modules/src/stabilizer_sitl.cpp
    ${CF2_SRCS_DIR}/src/modules/src/system_sitl.cpp
    ${CF2_SRCS_DIR}/src/utils/src/rateSupervisor.c
    ${CF2_SRCS_DIR}/src/modules/src/static_mem_sitl.c
    # ${CF2_SRCS_DIR}/src/modules/src/controller/controller.c
    # ${CF2_SRCS_DIR}/src/modules/src/controller/controller_pid.c
    # ${CF2_SRCS_DIR}/src/modules/src/controller/position_controller_pid.c
    # ${CF2_SRCS_DIR}/src/modules/src/controller/position_controller_indi.c
    # ${CF2_SRCS_DIR}/src/modules/src/controller/attitude_pid_controller.c
    # ${CF2_SRCS_DIR}/src/modules/src/controller/controller_mellinger.c
    # ${CF2_SRCS_DIR}/src/modules/src/controller/controller_indi.c
    # ${CF2_SRCS_DIR}/src/modules/src/controller/controller_brescianini.c
    # ${CF2_SRCS_DIR}/src/utils/src/pid.c ${CF2_SRCS_DIR}/src/utils/src/num.c
    # ${CF2_SRCS_DIR}/src/utils/src/filter.c ${CF2_SRCS_DIR}/src/utils/src/num.c
    # ${CF2_SRCS_DIR}/src/modules/src/power_distribution_quadrotor.c I/O
    # ${CF2_SRCS_DIR}/src/utils/src/eprintf.c
    # ${CF2_SRCS_DIR}/src/modules/src/console.c
    # ${CF2_SRCS_DIR}/src/modules/src/crtp.c
)

# # compiler flags set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-math-errno -Os
# -Wconversion -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow
# -Wno-unknown-warning-option " )
set(CMAKE_C_FLAGS
    "${CMAKE_C_FLAGS} -std=gnu11 -fsanitize=address -fsanitize=alignment")

set(CPP_FLAGS "${CPP_FLAGS} ${CMAKE_C_FLAGS}")
set(CPP_FLAGS "${CPP_FLAGS} -fno-exceptions -fcheck-new -fno-rtti -pedantic ")
set(CPP_FLAGS "${CPP_FLAGS} -Os -g3")
set(CMAKE_CXX_FLAGS "${CPP_FLAGS} ")

add_subdirectory(app)
