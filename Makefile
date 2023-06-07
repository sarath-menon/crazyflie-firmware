
OPENOCD           ?= openocd
OPENOCD_INTERFACE ?= interface/stlink.cfg
OPENOCD_TARGET    ?= target/stm32f4x.cfg
OPENOCD_CMDS      ?=

CPU                        = stm32f4
LOAD_ADDRESS_stm32f4       = 0x8000000
LOAD_ADDRESS_CLOAD_stm32f4 = 0x8004000

PYTHON            ?= python3

# Cload is handled in a special way on windows in WSL to use the Windows python interpreter
ifdef WSL_DISTRO_NAME
CLOAD_SCRIPT      ?= python.exe -m cfloader
else
CLOAD_SCRIPT      ?= $(PYTHON) -m cfloader
endif

DFU_UTIL          ?= dfu-util

PROG ?= $(srctree)/build/src/init/cf2

# Radio bootloader
CLOAD ?= 1
cload:
ifeq ($(CLOAD), 1)
	$(CLOAD_SCRIPT) $(CLOAD_CMDS) flash $(CLOAD_ARGS) $(PROG).bin stm32-fw
else
	@echo "Only cload build can be bootloaded. Launch build and cload with CLOAD=1"
endif

# Flags required by the ST library
ifeq ($(CLOAD), 1)
  LOAD_ADDRESS = $(LOAD_ADDRESS_CLOAD_$(CPU))
else
  LOAD_ADDRESS = $(LOAD_ADDRESS_$(CPU))
endif

unit:
# The flag "-DUNITY_INCLUDE_DOUBLE" allows comparison of double values in Unity. See: https://stackoverflow.com/a/37790196
	rake unit "DEFINES=$(ARCH_CFLAGS) -DUNITY_INCLUDE_DOUBLE" "FILES=$(FILES)" "UNIT_TEST_STYLE=$(UNIT_TEST_STYLE)"

#Flash the stm.
flash:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "flash write_image erase $(PROG).bin $(LOAD_ADDRESS) bin" \
                 -c "verify_image $(PROG).bin $(LOAD_ADDRESS) bin" -c "reset run" -c shutdown
#verify only
flash_verify:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "verify_image $(PROG).bin $(LOAD_ADDRESS) bin" -c "reset run" -c shutdown

#sends a usb message to the CF to place it in DFU mode, then updates firmware over usb
flash_dfu:
	$(PYTHON) $(srctree)/tools/make/usb-bootloader.py
	$(DFU_UTIL) -d 0483:df11 -a 0 -s 0x08004000:leave -D $(PROG).bin

#uses the dfu utility to flash the firmware at 0x08004000, just after the bootloader
#call this target directly if CF cannont be flashed automatically through flash_dfu
flash_dfu_manual:
	$(DFU_UTIL) -d 0483:df11 -a 0 -s 0x08004000:leave -D $(PROG).bin

#STM utility targets
halt:
	$(OPENOCD) -d0 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "halt" -c shutdown

reset:
	$(OPENOCD) -d0 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset" -c shutdown

openocd:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "\$$_TARGETNAME configure -rtos auto"

trace:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -f tools/trace/enable_trace.cfg

rtt:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets \
	           -c "rtt setup 0x20000000 262144 \"SEGGER RTT\"" -c "rtt start" -c "rtt server start 2000 0"

gdb: $(PROG).elf
	$(GDB) -ex "target remote localhost:3333" -ex "monitor reset halt" $^

erase:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "halt" -c "stm32f4x mass_erase 0" -c shutdown

#Print preprocessor #defines
prep:
	@$(CC) $(CFLAGS) -dM -E - < /dev/null

check_submodules:
	@cd $(srctree); $(PYTHON) tools/make/check-for-submodules.py

# Give control over to Kbuild
-include tools/kbuild/Makefile.kbuild


ifeq ($(KBUILD_SRC),)
# Python bindings
MOD_INC = src/modules/interface
MOD_SRC = src/modules/src

bindings_python build/cffirmware.py: bindings/setup.py $(MOD_SRC)/*.c
	swig -python -I$(MOD_INC) -Isrc/hal/interface -Isrc/utils/interface -Isrc/modules/interface/controller -o build/cffirmware_wrap.c bindings/cffirmware.i
	$(PYTHON) bindings/setup.py build_ext --inplace

test_python: build/cffirmware.py
	PYTHONPATH=build $(PYTHON) -m pytest test_python

python_wheel: build/cffirmware.py
	$(PYTHON) bindings/setup.py bdist_wheel
endif

.PHONY: all clean build compile unit prep erase flash check_submodules trace openocd gdb halt reset flash_dfu flash_dfu_manual flash_verify cload size print_version clean_version bindings_python test_python python_wheel
