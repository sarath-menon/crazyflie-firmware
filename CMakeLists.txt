# include(cmake/gcc-arm-none-eabi.cmake) include(cmake/cf2.cmake)
# include(cmake/board.cmake)
include(cmake/top.cmake)
include(vendor/cmake/cmsis_dsp_.cmake)

cmake_minimum_required(VERSION 3.15.3)
project(cf2)

enable_language(C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Custom optionss
set(PLATFORM "cf2")

add_subdirectory(src/init)
add_subdirectory(src/deck)
add_subdirectory(src/drivers)
add_subdirectory(src/hal)
add_subdirectory(src/lib)
add_subdirectory(src/platform)
add_subdirectory(src/modules)

add_subdirectory(vendor)
