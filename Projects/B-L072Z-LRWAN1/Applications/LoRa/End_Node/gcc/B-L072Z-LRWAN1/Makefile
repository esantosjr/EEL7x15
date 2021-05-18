# STM32 Makefile for GNU toolchain and openocd
#
# This Makefile fetches the Cube firmware package from ST's' website.
# This includes: CMSIS, STM32 HAL, BSPs, USB drivers and examples.
#
# Usage:
#	make     		Compile the application
#	make program	Compile and Flash the board

# A name common to all output files (elf, map, hex, bin, lst)
TARGET     = end_node

# Take a look into $(CUBE_DIR)/Drivers/BSP for available BSPs
# name needed in upper case and lower case
BOARD      = B-L072Z-LRWAN1
BOARD_UC   = B-L072Z-LRWAN1
BOARD_MC   = B-L072Z-LRWAN1
BOARD_LC   = b-l072z-lrwan1
RADIO      = sx1276
COMPONENT  = mlm32l0xx
COMP_MC    = MLM32L07X01
COMP_LC    = mlm32l07x01

OCDFLAGS   = -f board/st_nucleo_l073rz.cfg
GDBFLAGS   =

APP_ROOT   = ../../LoRaWAN/App

# MCU family and type in various capitalizations
MCU_FAMILY = stm32l0xx
MCU_FAM_MC = STM32L0xx
MCU_LC     = stm32l072xx
MCU_MC     = STM32L072xx
MCU_UC     = STM32L072CZ

# path of the ld-file inside the example directories
LDFILE     = ../../SW4STM32/$(COMP_LC)/$(MCU_UC)Yx_FLASH.ld
LKFILE     = ../../SW4STM32

# C files from the /src directory
SRCS       = main.c
SRCS      += bsp.c
SRCS      += debug.c
SRCS      += hw_gpio.c
SRCS      += hw_rtc.c
SRCS      += hw_spi.c
SRCS      += vcom.c

# C files from the /Core directory
SRCS      += $(COMPONENT)_hal_msp.c
SRCS      += $(COMPONENT)_it.c
SRCS      += $(COMPONENT)_hw.c

# Basic HAL libraries
SRCS      += $(MCU_FAMILY)_hal_spi.c
SRCS      += $(MCU_FAMILY)_hal_uart.c
SRCS      += $(MCU_FAMILY)_hal_uart_ex.c
SRCS      += $(MCU_FAMILY)_hal_adc.c
SRCS      += $(MCU_FAMILY)_hal_adc_ex.c
SRCS      += $(MCU_FAMILY)_hal_rcc.c
SRCS      += $(MCU_FAMILY)_hal_rcc_ex.c
SRCS      += $(MCU_FAMILY)_hal.c
SRCS      += $(MCU_FAMILY)_hal_cortex.c
SRCS      += $(MCU_FAMILY)_hal_gpio.c
SRCS      += $(MCU_FAMILY)_hal_dma.c
SRCS      += $(MCU_FAMILY)_hal_pwr.c
SRCS      += $(MCU_FAMILY)_hal_pwr_ex.c
SRCS      += $(MCU_FAMILY)_hal_rtc.c
SRCS      += $(MCU_FAMILY)_hal_rtc_ex.c
SRCS      += $(MCU_FAMILY)_hal_tim.c
SRCS      += $(MCU_FAMILY)_hal_tim_ex.c
SRCS      += $(MCU_FAMILY)_hal_i2c.c
SRCS      += $(MCU_FAMILY)_hal_i2c_ex.c
SRCS      += $(MCU_FAMILY)_hal_comp.c
SRCS      += $(MCU_FAMILY)_hal_comp_ex.c
SRCS      += $(MCU_FAMILY)_hal_crc.c
SRCS      += $(MCU_FAMILY)_hal_crc_ex.c
SRCS      += $(MCU_FAMILY)_hal_cryp.c
SRCS      += $(MCU_FAMILY)_hal_cryp_ex.c
SRCS      += $(MCU_FAMILY)_hal_dac.c
SRCS      += $(MCU_FAMILY)_hal_dac_ex.c
SRCS      += $(MCU_FAMILY)_hal_firewall.c
SRCS      += $(MCU_FAMILY)_hal_flash.c
SRCS      += $(MCU_FAMILY)_hal_flash_ex.c
SRCS      += $(MCU_FAMILY)_hal_flash_ramfunc.c
SRCS      += $(MCU_FAMILY)_hal_i2s.c
SRCS      += $(MCU_FAMILY)_hal_irda.c
SRCS      += $(MCU_FAMILY)_hal_iwdg.c
SRCS      += $(MCU_FAMILY)_hal_lptim.c
SRCS      += $(MCU_FAMILY)_hal_rng.c
SRCS      += $(MCU_FAMILY)_hal_usart.c

# LoRaWAN
# -- Patterns
SRCS      += lora.c
SRCS      += lora-test.c
SRCS      += NvmCtxMgmt.c
SRCS      += LmHandler.c
SRCS      += FragDecoder.c
SRCS      += LmhpClockSync.c
SRCS      += LmhpCompliance.c
SRCS      += LmhpFragmentation.c
SRCS      += LmhpRemoteMcastSetup.c

# -- MAC
SRCS      += LoRaMac.c
SRCS      += LoRaMacCrypto.c
SRCS      += LoRaMacAdr.c
SRCS      += LoRaMacClassB.c
SRCS      += LoRaMacCommands.c
SRCS      += LoRaMacConfirmQueue.c
SRCS      += LoRaMacParser.c
SRCS      += LoRaMacSerializer.c

# -- MAC regions
SRCS      += Region.c
SRCS      += RegionCommon.c
SRCS      += RegionAU915.c
#SRCS      += RegionUS915.c
#SRCS      += RegionLA915.c
#SRCS      += RegionEU868.c

# -- Conf
SRCS      += hw_gpio_template.c
SRCS      += hw_rtc_template.c
SRCS      += hw_spi_template.c

# -- Crypto
SRCS      += aes.c
SRCS      += cmac.c
SRCS      += soft-se.c

# -- Utilities
SRCS      += low_power_manager.c
SRCS      += queue.c
SRCS      += systime.c
SRCS      += timeServer.c
SRCS      += trace.c
SRCS      += utilities.c

# Drivers/BSP/Components
SRCS      += HTS221_Driver.c
SRCS      += HTS221_Driver_HL.c
SRCS      += LIS3MDL_MAG_driver.c
SRCS      += LIS3MDL_MAG_driver_HL.c
SRCS      += LPS22HB_Driver.c
SRCS      += LPS22HB_Driver_HL.c
SRCS      += LPS25HB_Driver.c
SRCS      += LPS25HB_Driver_HL.c
SRCS      += LSM6DS0_ACC_GYRO_driver.c
SRCS      += LSM6DS0_ACC_GYRO_driver_HL.c
SRCS      += LSM6DS3_ACC_GYRO_driver.c
SRCS      += LSM6DS3_ACC_GYRO_driver_HL.c
SRCS      += LSM6DSL_ACC_GYRO_driver.c
SRCS      += LSM6DSL_ACC_GYRO_driver_HL.c
SRCS      += LSM303AGR_ACC_driver.c
SRCS      += LSM303AGR_ACC_driver_HL.c
SRCS      += LSM303AGR_MAG_driver.c
SRCS      += LSM303AGR_MAG_driver_HL.c
SRCS      += $(RADIO).c

# Drivers/BSP/X_NUCLEO_IKS01A1
# SRCS      += x_nucleo_iks01a1.c
# SRCS      += x_nucleo_iks01a1_accelero.c
# SRCS      += x_nucleo_iks01a1_gyro.c
# SRCS      += x_nucleo_iks01a1_humidity.c
# SRCS      += x_nucleo_iks01a1_magneto.c
# SRCS      += x_nucleo_iks01a1_pressure.c
# SRCS      += x_nucleo_iks01a1_temperature.c

# Drivers/BSP/X_NUCLEO_IKS01A2
SRCS      += x_nucleo_iks01a2.c
SRCS      += x_nucleo_iks01a2_accelero.c
SRCS      += x_nucleo_iks01a2_gyro.c
SRCS      += x_nucleo_iks01a2_humidity.c
SRCS      += x_nucleo_iks01a2_magneto.c
SRCS      += x_nucleo_iks01a2_pressure.c
SRCS      += x_nucleo_iks01a2_temperature.c

# Drivers/BSP/...
SRCS      += $(BOARD_LC).c
SRCS      += $(COMP_LC).c

# CMSIS
SRCS      += system_$(MCU_FAMILY).c

# Directories
CUBE_DIR   = ../../../../../../..

BSP_DIR    = $(CUBE_DIR)/Drivers/BSP
CORE_DIR   = $(CUBE_DIR)/Projects/$(BOARD)/Applications/LoRa/End_Node/Core
HAL_DIR    = $(CUBE_DIR)/Drivers/$(MCU_FAM_MC)_HAL_Driver
CMSIS_DIR  = $(CUBE_DIR)/Drivers/CMSIS

DEV_DIR    = $(CMSIS_DIR)/Device/ST/$(MCU_FAM_MC)

COMP_DIR   = $(CUBE_DIR)/Drivers/BSP/Components
MWARE_DIR  = $(CUBE_DIR)/Middlewares/Third_Party

# that's it, no need to change anything below this line!

###############################################################################
# Toolchain

PREFIX     = arm-none-eabi
CC         = $(PREFIX)-gcc
AR         = $(PREFIX)-ar
OBJCOPY    = $(PREFIX)-objcopy
OBJDUMP    = $(PREFIX)-objdump
SIZE       = $(PREFIX)-size
GDB        = $(PREFIX)-gdb

OCD        = openocd

###############################################################################
# Options

# Defines
DEFS       = -D$(MCU_MC) -DUSE_HAL_DRIVER

# LoRa
DEFS       += -DUSE_B_L072Z_LRWAN1
DEFS       += -DREGION_AU915
DEFS       += -DSENSOR_ENABLED
DEFS       += -DX_NUCLEO_IKS01A2
# DEFS       += -DX_NUCLEO_IKS01A1

# Debug specific definitions for semihosting
DEFS       += -DUSE_DBPRINTF

# Include search paths (-I)
INCS       = -I$(APP_ROOT)/inc
INCS      += -I$(CORE_DIR)/inc
INCS      += -I$(BSP_DIR)/$(BOARD_MC)
INCS      += -I$(BSP_DIR)/CMWX1ZZABZ-0xx
INCS      += -I$(BSP_DIR)/X_NUCLEO_IKS01A2
# INCS      += -I$(BSP_DIR)/X_NUCLEO_IKS01A1

# Sensors
INCS      += -I$(COMP_DIR)/Common
INCS      += -I$(COMP_DIR)/hts221
INCS      += -I$(COMP_DIR)/lis3mdl
INCS      += -I$(COMP_DIR)/lps22hb
INCS      += -I$(COMP_DIR)/lps25hb
INCS      += -I$(COMP_DIR)/lsm6ds0
INCS      += -I$(COMP_DIR)/lsm6ds3
INCS      += -I$(COMP_DIR)/lsm6dsl
INCS      += -I$(COMP_DIR)/lsm303agr
INCS      += -I$(COMP_DIR)/$(RADIO)

INCS      += -I$(CMSIS_DIR)/Include
INCS      += -I$(CMSIS_DIR)/Device/ST/STM32L0xx/Include
INCS      += -I$(DEV_DIR)/Include
INCS      += -I$(HAL_DIR)/Inc
INCS      += -I$(HAL_DIR)/Inc/Legacy

# LoRaWAN
INCS      += -I$(MWARE_DIR)/LoRaWAN/Crypto
INCS      += -I$(MWARE_DIR)/LoRaWAN/Conf
INCS      += -I$(MWARE_DIR)/LoRaWAN/Conf/Inc
INCS      += -I$(MWARE_DIR)/LoRaWAN/Mac
INCS      += -I$(MWARE_DIR)/LoRaWAN/Mac/region
INCS      += -I$(MWARE_DIR)/LoRaWAN/Phy
INCS      += -I$(MWARE_DIR)/LoRaWAN/Utilities
INCS      += -I$(MWARE_DIR)/LoRaWAN/Patterns/Basic
INCS      += -I$(MWARE_DIR)/LoRaWAN/Patterns/Advanced
INCS      += -I$(MWARE_DIR)/LoRaWAN/Patterns/Advanced/LmHandler
INCS      += -I$(MWARE_DIR)/LoRaWAN/Patterns/Advanced/LmHandler/packages

# Source search paths
VPATH      = $(APP_ROOT)/src
VPATH     += $(CORE_DIR)/src
VPATH     += $(BSP_DIR)/$(BOARD_MC)
VPATH     += $(BSP_DIR)/CMWX1ZZABZ-0xx
VPATH     += $(BSP_DIR)/X_NUCLEO_IKS01A2
# VPATH     += $(BSP_DIR)/X_NUCLEO_IKS01A1
VPATH     += $(HAL_DIR)/Src
VPATH     += $(DEV_DIR)/Source
VPATH     += $(DEV_DIR)/Source/Templates
VPATH     += $(CMSIS_DIR)/Device/ST/STM32L0xx/Source/Templates

# Components
VPATH     += $(COMP_DIR)/hts221
VPATH     += $(COMP_DIR)/lis3mdl
VPATH     += $(COMP_DIR)/lps22hb
VPATH     += $(COMP_DIR)/lps25hb
VPATH     += $(COMP_DIR)/lsm6ds0
VPATH     += $(COMP_DIR)/lsm6ds3
VPATH     += $(COMP_DIR)/lsm6dsl
VPATH     += $(COMP_DIR)/lsm303agr
VPATH     += $(COMP_DIR)/$(RADIO)

# LoRaWAN
VPATH     += $(MWARE_DIR)/LoRaWAN/Conf/Src
VPATH     += $(MWARE_DIR)/LoRaWAN/Crypto
VPATH     += $(MWARE_DIR)/LoRaWAN/Mac
VPATH     += $(MWARE_DIR)/LoRaWAN/Mac/region
VPATH     += $(MWARE_DIR)/LoRaWAN/Phy
VPATH     += $(MWARE_DIR)/LoRaWAN/Utilities
VPATH     += $(MWARE_DIR)/LoRaWAN/Patterns
VPATH     += $(MWARE_DIR)/LoRaWAN/Patterns/Basic
VPATH     += $(MWARE_DIR)/LoRaWAN/Patterns/Advanced
VPATH     += $(MWARE_DIR)/LoRaWAN/Patterns/Advanced/LmHandler
VPATH     += $(MWARE_DIR)/LoRaWAN/Patterns/Advanced/LmHandler/packages

# Library search paths
LIBS       = -L$(CMSIS_DIR)/Lib

# Compiler flags
CFLAGS     = -Wall -g -std=c99 -Os
CFLAGS    += -mcpu=cortex-m0plus -mthumb
#CFLAGS    += -Wextra
CFLAGS    += -Wno-unused-parameter -Wno-missing-field-initializers
CFLAGS    += -fmessage-length=0 -fno-exceptions -fno-builtin -funsigned-char
CFLAGS    += -MMD -fno-delete-null-pointer-checks -fomit-frame-pointer
#CFLAGS    += -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS    += -ffunction-sections -fdata-sections
CFLAGS    += $(INCS) $(DEFS)

# Linker flags
LDFLAGS    = -Wl,--gc-sections -Wl,-Map=$(TARGET).map $(LIBS) -T$(LDFILE)

# Enable Semihosting
LDFLAGS   += --specs=rdimon.specs -lc -lrdimon

OBJS       = $(addprefix obj/,$(SRCS:.c=.o))
DEPS       = $(addprefix dep/,$(SRCS:.c=.d))

# Prettify output
V = 1
ifeq ($V, 0)
	Q = @
	P = > /dev/null
endif

###################################################

.PHONY: all dirs program debug template clean

all: $(TARGET).bin

-include $(DEPS)

dirs: dep obj
dep obj src:
	@echo "[MKDIR]   $@"
	$Qmkdir -p $@

obj/%.o : %.c | dirs
	@echo "[CC]      $(notdir $<)"
	$Q$(CC) $(CFLAGS) -c -o $@ $< -MMD -MF dep/$(*F).d

$(TARGET).elf: $(OBJS)
	@echo "[LD]      $(TARGET).elf"
	$Q$(CC) $(CFLAGS) $(LDFLAGS) $(LKFILE)/startup_$(MCU_LC).s $^ -o $@
	@echo "[OBJDUMP] $(TARGET).lst"
	$Q$(OBJDUMP) -St $(TARGET).elf >$(TARGET).lst
	@echo "[SIZE]    $(TARGET).elf"
	$(SIZE) $(TARGET).elf

$(TARGET).bin: $(TARGET).elf
	@echo "[OBJCOPY] $(TARGET).bin"
	$Q$(OBJCOPY) -O binary $< $@

clean:
	@echo "[RM]      $(TARGET).bin"; rm -f $(TARGET).bin
	@echo "[RM]      $(TARGET).elf"; rm -f $(TARGET).elf
	@echo "[RM]      $(TARGET).map"; rm -f $(TARGET).map
	@echo "[RM]      $(TARGET).lst"; rm -f $(TARGET).lst
	@echo "[RMDIR]   dep"          ; rm -fr dep
	@echo "[RMDIR]   obj"          ; rm -fr obj
