# Board  Defs --------------------------------------------------------

set(MCU_DEFINITONS -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000
                   -DUSE_STDPERIPH_DRIVER -DCRAZYFLIE_FW)

# CPU Parameters --------------------------------------------------------

set(CPU_PARAMETERS -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16)

# Scripts --------------------------------------------------------

set(STARTUP_SCRIPT ${CMAKE_SOURCE_DIR}/src/init/startup_stm32f405xx.s)
set(MCU_LINKER_SCRIPT
    ${CMAKE_SOURCE_DIR}/tools/make/F405/linker/STM32F405RGTx_FLASH.ld)
