##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.13.0-B3] date: [Thu Apr 29 21:47:12 MDT 2021]
##########################################################################################################################

# ------------------------------------------------
# Generic makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = main

DEVICE_DIRNAME = STM32F042K4Tx

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
# USER_INCLUDES := -I ./inc
USER_SOURCES := $(shell find ./$(DEVICE_DIRNAME)/Core/Src -name "*.c")

HAL_SOURCES := $(shell find ./$(DEVICE_DIRNAME)/Drivers/STM32F0xx_HAL_Driver/Src -name "*.c")

C_SOURCES += $(HAL_SOURCES)
C_SOURCES += $(USER_SOURCES)

# ASM sources
ASM_SOURCES = $(DEVICE_DIRNAME)/Core/Startup/startup_stm32f042k4tx.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m0

# fpu
#FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=soft

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
# C_DEFS = \
# -D USE_HAL_DRIVER \
# -D STM32F401xE

C_DEFS = \
-D USE_HAL_DRIVER \
-D STM32F042x6 \
-D DEBUG \
-D RING_ENCODER


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-I ./$(DEVICE_DIRNAME)/Core/Inc \
-I ./$(DEVICE_DIRNAME)/Drivers/STM32F0xx_HAL_Driver/Inc \
-I ./$(DEVICE_DIRNAME)/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy \
-I ./$(DEVICE_DIRNAME)/Drivers/CMSIS/Device/ST/STM32F0xx/Include \
-I ./$(DEVICE_DIRNAME)/Drivers/CMSIS/Include \
-I ./WLoopCAN/include

C_INCLUDES += $(USER_INCLUDES)

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = $(DEVICE_DIRNAME)/STM32F042K4TX_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: libs $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@
	@echo ""

$(BUILD_DIR)/%.o: %.s makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@
	@echo ""

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) makefile
	$(CC) $(OBJECTS) ./WLoopCAN/bin/wloop_can.a $(LDFLAGS) -o $@
	@echo ""
	$(SZ) $@
	@echo ""

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	rm -rf $(BUILD_DIR)
	rm -rf ./WLoopCAN/bin

analyze:
	$(PREFIX)objdump -t $(BUILD_DIR)/$(TARGET).elf

flash:
	st-flash write $(BUILD_DIR)/main.bin 0x08000000 
	st-flash reset

libs: 
	cd WLoopCAN && make ring_encoder

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
