include(cmake/gcc-arm-none-eabi.cmake)
include(cmake/cf2.cmake)
include(cmake/board.cmake)

cmake_minimum_required(VERSION 3.15.3)
project(cf2)

enable_language(C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find thread package
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

# Custom optionss
set(PLATFORM "cf2")

add_subdirectory(src/init)
