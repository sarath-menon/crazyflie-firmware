# Board  Defs --------------------------------------------------------

set(MCU_DEFINITONS -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000
                   -DUSE_STDPERIPH_DRIVER)

# CPU Parameters --------------------------------------------------------

set(CPU_PARAMETERS -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -mfpu=fpv4-sp-d16)

# Scripts --------------------------------------------------------

# set(STARTUP_SCRIPT
# ${CMAKE_SOURCE_DIR}/external/nrf51sdk1000dc26b5e/components/toolchain/gcc/gcc_startup_nrf51.s
# )
