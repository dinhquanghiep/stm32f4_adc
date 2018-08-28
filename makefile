# Note: Bổ sung thêm phần thay đổi của .h
# 			Trường hợp .h lồng trong .h thì thế nào?

# ---------------------------------------------------------------------
# Toolchain Configuration
# ---------------------------------------------------------------------
# TOOLCHAIN_DIR				:= D:/Work/6._IDE/gcc-arm-none-eabi/bin/
TOOLCHAIN_DIR				:= 
TOOLCHAIN_PREFIX 		:= arm-none-eabi-
CC									:= $(TOOLCHAIN_DIR)$(TOOLCHAIN_PREFIX)gcc
AR									:= $(TOOLCHAIN_DIR)$(TOOLCHAIN_PREFIX)ar
RANDLIB							:= $(TOOLCHAIN_DIR)$(TOOLCHAIN_PREFIX)ranlib
OBJCOPY							:= $(TOOLCHAIN_DIR)$(TOOLCHAIN_PREFIX)objcopy
SIZE								:= $(TOOLCHAIN_DIR)$(TOOLCHAIN_PREFIX)size

CC_FLAG := -std=c99 -c -Wall
CC_FLAG += -ffunction-sections -fdata-sections
CC_FLAG_DEBUG		:= -g
CC_FLAG_RELEASE := -Os
CC_FLAG_LIBRARY := -I
# ---------------------------------------------------------------------
# Project Configuration
# ---------------------------------------------------------------------
PROJ_DIR	:= .
PROJ_NAME	:= stm32f4_discovery

MCU_DEFS 			:= -DSTM32F4 -DSTM32F407 -DSTM32F407xx -DSTM32F40_41xxx -DUSE_STDPERIPH_DRIVER
MCU_SETTINGS 	:= -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 

INC_DIRS	:= $(CC_FLAG_LIBRARY)D:/Work/6._IDE/vcs/arm/lib/cmsis/cores
INC_DIRS	+= $(CC_FLAG_LIBRARY)D:/Work/6._IDE/vcs/arm/lib/cmsis/variant
INC_DIRS	+= $(CC_FLAG_LIBRARY)D:/Work/6._IDE/vcs/arm/stm32f4_discovery/spl/inc
INC_DIRS	+= $(CC_FLAG_LIBRARY)D:/Work/6._IDE/vcs/arm/stm32f4_discovery/spl/src
INC_DIRS	+= $(CC_FLAG_LIBRARY)$(PROJ_DIR)/inc

STARTUP_DIR					:= D:/Work/6._IDE/vcs/arm/lib/st/startup
SYSTEM_CONFIG_DIR		:= D:/Work/6._IDE/vcs/arm/lib/cmsis/variant
LINKER_SCRIPT_FILE	:= D:/Work/6._IDE/vcs/arm/lib/st/ldscripts/STM32F40_41xxx/STM32F417IGHx_FLASH.ld

PROJ_OUTPUT_DIR	:= $(PROJ_DIR)/output

PROJ_SRC_DIR		:= $(PROJ_DIR)/src
PROJ_SRC_FILES	:= $(wildcard $(PROJ_SRC_DIR)/*.c)
PROJ_OBJ_DIR 		:= $(PROJ_OUTPUT_DIR)/src
PROJ_OBJ_FILES	:= $(patsubst $(PROJ_SRC_DIR)/%.c, $(PROJ_OBJ_DIR)/%.o, $(PROJ_SRC_FILES))
# DEPS						:= $(PROJ_OBJ_FILES:$(PROJ_SRC_DIR)/%.c=$(PROJ_SRC_DIR)/%.o)

SPL_SRC_DIR			:= D:/Work/6._IDE/vcs/arm/stm32f4_discovery/spl/src
SPL_SRC_FILES		:= $(wildcard $(SPL_SRC_DIR)/*.c)
SPL_OBJ_DIR 		:= $(PROJ_OUTPUT_DIR)/lib
SPL_OBJ_FILES		:= $(patsubst $(SPL_SRC_DIR)/%.c, $(SPL_OBJ_DIR)/%.o, $(SPL_SRC_FILES))
# ---------------------------------------------------------------------
# Configuration Flags for compiler, assembler and linker
# ---------------------------------------------------------------------
CC_FLAGS 				:= $(CC_FLAG) $(CC_FLAG_DEBUG) $(MCU_SETTINGS) $(MCU_DEFS) $(INC_DIRS)
AS_FLAGS 				:= $(CC_FLAGS) -x assembler-with-cpp
LD_FLAGS 				:= $(MCU_SETTINGS) -Xlinker --gc-sections -lc -lm -lnosys
AR_FLAGS 				:= rc
SIZE_FLAGS 			:= --format=berkeley --radix=10
OBJCOPY_FLAGS		:= -O ihex
# ---------------------------------------------------------------------
# Command line interface
# ---------------------------------------------------------------------
.PHONY: run makelibrary buildsource link clean

run: makelibrary buildsource link hex size

clean: 
	@rm -rf $(PROJ_OUTPUT_DIR)/*.* $(SPL_OBJ_DIR)/*.* $(PROJ_OBJ_DIR)/*.*

makelibrary: $(PROJ_OUTPUT_DIR)/lib$(PROJ_NAME).a
$(PROJ_OUTPUT_DIR)/lib$(PROJ_NAME).a: $(SPL_OBJ_DIR)/system_stm32f4xx.o $(SPL_OBJ_FILES)
	$(AR) $(AR_FLAGS) $@ $+
	$(RANDLIB) $@
	
$(SPL_OBJ_DIR)/system_stm32f4xx.o: $(SYSTEM_CONFIG_DIR)/system_stm32f4xx.c
	$(CC) $(CC_FLAGS) -o $@ $<

$(SPL_OBJ_DIR)/%.o: $(SPL_SRC_DIR)/%.c
	$(CC) $(CC_FLAGS) -o $@ $<

buildsource: $(PROJ_OBJ_DIR)/startup_stm32f40_41xxx.o $(PROJ_OBJ_FILES)
$(PROJ_OBJ_DIR)/startup_stm32f40_41xxx.o: $(STARTUP_DIR)/startup_stm32f40_41xxx.s
	$(CC) $(AS_FLAGS) -o $@ $+

$(PROJ_OBJ_DIR)/%.o: $(PROJ_SRC_DIR)/%.c $(wildcard $(PROJ_SRC_DIR)/%.h)
	$(CC) $(CC_FLAGS) -o $@ $+

link: $(PROJ_OUTPUT_DIR)/$(PROJ_NAME).elf
$(PROJ_OUTPUT_DIR)/%.elf: $(PROJ_OBJ_FILES) $(PROJ_OBJ_DIR)/startup_stm32f40_41xxx.o $(PROJ_OUTPUT_DIR)/lib$(PROJ_NAME).a $(LINKER_SCRIPT_FILE)
	$(CC) $(LD_FLAGS) -o $@ -T$(LINKER_SCRIPT_FILE) -Wl,-Map=$(PROJ_OUTPUT_DIR)/$(PROJ_NAME).map $(PROJ_OBJ_FILES) $(PROJ_OBJ_DIR)/startup_stm32f40_41xxx.o $(PROJ_OUTPUT_DIR)/lib$(PROJ_NAME).a 

size:
	$(SIZE) $(SIZE_FLAGS) $(PROJ_OUTPUT_DIR)/*.elf

hex: $(PROJ_OUTPUT_DIR)/$(PROJ_NAME).hex
%.hex: %.elf
	$(OBJCOPY) $(OBJCOPY_FLAGS) $< $@
