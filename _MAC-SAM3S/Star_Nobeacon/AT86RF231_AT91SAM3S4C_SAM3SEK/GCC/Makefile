############################################################################################
# Makefile for the project MAC_Example Star_Nobeacon Release Using single source files
############################################################################################
# $Id: Makefile 24841 2011-01-12 07:45:25Z mahendran.p $

# Build specific properties
_TAL_TYPE = AT86RF231
_PAL_TYPE = AT91SAM3S4C
_PAL_GENERIC_TYPE = SAM3
_BOARD_TYPE = RZ600_231_SAM3SEK
_HIGHEST_STACK_LAYER = MAC

# Path variables
## Path to main project directory
PATH_ROOT = ../../../../..
PATH_APP = ../..
PATH_TAL = $(PATH_ROOT)/TAL
PATH_TINY_TAL = $(PATH_ROOT)/TINY_TAL
PATH_TAL_CB = $(PATH_ROOT)/TAL/Src
PATH_PAL = $(PATH_ROOT)/PAL
PATH_RES = $(PATH_ROOT)/Resources
PATH_MAC = $(PATH_ROOT)/MAC
PATH_SAM3S_LIB = $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/Libraries
PATH_LDSCRIPTS = ./linker_scripts
PATH_RES = $(PATH_ROOT)/Resources
PATH_SIO_SUPPORT = $(PATH_ROOT)/Applications/Helper_Files/SIO_Support

## General Flags
PROJECT = Star
MCU = cortex-m3
CC = arm-none-eabi-gcc
SIZE = arm-none-eabi-size
STRIP = arm-none-eabi-strip
OBJCOPY = arm-none-eabi-objcopy


## Options common to compile, link and assembly rules
COMMON = -mcpu=$(MCU) -mthumb

## Stack related flags
CFLAGS = $(COMMON) $(INCLUDES)
CFLAGS += -Wall -Werror -g -Wundef -std=c99 -Os -mlong-calls -ffunction-sections
CFLAGS += -DDEBUG=0
CFLAGS += -DFFD
CFLAGS += -DREDUCED_PARAM_CHECK
CFLAGS += -DTAL_TYPE=$(_TAL_TYPE)
CFLAGS += -DPAL_GENERIC_TYPE=$(_PAL_GENERIC_TYPE)
CFLAGS += -DPAL_TYPE=$(_PAL_TYPE)
CFLAGS += -DBOARD_TYPE=$(_BOARD_TYPE)
CFLAGS += -DHIGHEST_STACK_LAYER=$(_HIGHEST_STACK_LAYER)
CFLAGS += -DIEEE_ADDR=0x123456789ABCDEF0
CFLAGS += -DNODMA_SPI

## Assembly specific flags
ASMFLAGS = $(COMMON) $(INCLUDES)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-g

## Linker flags
LDFLAGS = $(COMMON) --gc-sections -nostartfiles -Wl,-Map=$(PROJECT).map

# Defines which are the available memory targets
MEMORIES = flash

# Output directories
OBJ = obj

# Includes
## Main and application stuff
INCLUDES  = -I$(PATH_APP)/Inc
INCLUDES += -I$(PATH_ROOT)/Include

## PAL related
INCLUDES += -I$(PATH_PAL)/Inc
INCLUDES += -I$(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Boards
INCLUDES += -I$(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Boards/$(_BOARD_TYPE)
INCLUDES += -I$(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Inc
INCLUDES += -I$(PATH_PAL)/$(_PAL_GENERIC_TYPE)/Generic/Inc

## LIB/PAL related
INCLUDES += -I$(PATH_SAM3S_LIB)/cmsis
INCLUDES += -I$(PATH_SAM3S_LIB)

## TAL related
INCLUDES += -I$(PATH_TAL)/$(_TAL_TYPE)/Inc/
INCLUDES += -I$(PATH_TAL)/$(_TAL_TYPE)/Inc
INCLUDES += -I$(PATH_TAL)/Inc

# Include Directories for MAC
INCLUDES += -I $(PATH_MAC)/Inc/

## Include directories for resources
INCLUDES += -I $(PATH_RES)/Buffer_Management/Inc/
INCLUDES += -I $(PATH_RES)/Queue_Management/Inc/

## Include directories for SIO support
INCLUDES += -I $(PATH_SIO_SUPPORT)/Inc

## USB descriptor files
INCLUDES += -I$(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Boards/$(_BOARD_TYPE)/descriptors

#-------------------------------------------------------------------------------
#  Files
#-------------------------------------------------------------------------------

# Directories where source files can be found

#main.c
VPATH += $(PATH_APP)/Src
VPATH += $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Boards/$(_BOARD_TYPE)
VPATH += $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Boards/$(_BOARD_TYPE)/descriptors
VPATH += $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Src
VPATH += $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/Generic/Src
VPATH += $(PATH_SAM3S_LIB)/
VPATH += $(PATH_SAM3S_LIB)/cmsis
VPATH += $(PATH_SAM3S_LIB)/utility
VPATH += $(PATH_SAM3S_LIB)/usb/common/core
VPATH += $(PATH_SAM3S_LIB)/usb/device/core
VPATH += $(PATH_SAM3S_LIB)/usb/common/cdc
VPATH += $(PATH_SAM3S_LIB)/usb/device/cdc-serial
VPATH += $(PATH_SAM3S_LIB)/usb/usbd
VPATH += $(PATH_TAL)/$(_TAL_TYPE)/Src
VPATH += $(PATH_RES)/Buffer_Management/Src
VPATH += $(PATH_RES)/Queue_Management/Src
VPATH += $(PATH_SIO_SUPPORT)/Src
VPATH += $(PATH_MAC)/Src


# Objects built from C source files
## Main
C_OBJECTS += main.o

## platform dependent
C_OBJECTS += board_cstartup_gnu.o
C_OBJECTS += exceptions.o
C_OBJECTS += pal_board.o
C_OBJECTS += pal_irq.o

##SIO HUB
C_OBJECTS += pal_sio_hub.o

## SAM3S Generic PAL related
C_OBJECTS += pal_timer.o
C_OBJECTS += pal_trx_access.o
C_OBJECTS += pal_utils.o
C_OBJECTS += pal.o
C_OBJECTS += pal_uart.o
C_OBJECTS += pal_usb.o

## SAM3S LIB objects
C_OBJECTS += core_cm3.o
#C_OBJECTS += stdio.o
#C_OBJECTS += syscalls.o

## SAM3S USB LIB objects
C_OBJECTS += CDCDSerialDriverDescriptors.o
C_OBJECTS += USBD.o
C_OBJECTS += USBD_HAL.o
C_OBJECTS += CDCDSerialDriver.o
C_OBJECTS += CDCSetControlLineStateRequest.o
C_OBJECTS += CDCLineCoding.o
C_OBJECTS += USBDDriver.o
C_OBJECTS += USBDCallbacks_Initialized.o
C_OBJECTS += USBDCallbacks_Resumed.o
C_OBJECTS += USBDCallbacks_Suspended.o
C_OBJECTS += USBDCallbacks_Reset.o
C_OBJECTS += USBDDriverCb_CfgChanged.o
C_OBJECTS += USBDDriverCb_IfSettingChanged.o
C_OBJECTS += USBSetAddressRequest.o
C_OBJECTS += USBGenericDescriptor.o
C_OBJECTS += USBInterfaceRequest.o
C_OBJECTS += USBGenericRequest.o
C_OBJECTS += USBGetDescriptorRequest.o
C_OBJECTS += USBSetConfigurationRequest.o
C_OBJECTS += USBFeatureRequest.o
C_OBJECTS += USBEndpointDescriptor.o
C_OBJECTS += USBConfigurationDescriptor.o

## TAL/TINY_TAL objects
C_OBJECTS += tal_init.o
C_OBJECTS += tal_irq_handler.o
C_OBJECTS += tal_pib.o
C_OBJECTS += tal_pwr_mgmt.o
C_OBJECTS += tal_rx_enable.o
C_OBJECTS += tal_slotted_csma.o
C_OBJECTS += tal_rx.o
C_OBJECTS += tal_tx.o
C_OBJECTS += tal_ed.o
C_OBJECTS += tal.o

## Resources objects
C_OBJECTS += bmm.o
C_OBJECTS += qmm.o

## SIO Handler objects
C_OBJECTS += sio_handler.o

## MAC objects
C_OBJECTS += mac.o
C_OBJECTS += mac_api.o
C_OBJECTS += mac_associate.o
C_OBJECTS += mac_beacon.o
C_OBJECTS += mac_callback_wrapper.o
C_OBJECTS += mac_data_ind.o
C_OBJECTS += mac_data_req.o
C_OBJECTS += mac_disassociate.o
C_OBJECTS += mac_dispatcher.o
C_OBJECTS += mac_mcps_data.o
C_OBJECTS += mac_misc.o
C_OBJECTS += mac_orphan.o
C_OBJECTS += mac_pib.o
C_OBJECTS += mac_poll.o
C_OBJECTS += mac_process_beacon_frame.o
C_OBJECTS += mac_process_tal_tx_frame_status.o
C_OBJECTS += mac_rx_enable.o
C_OBJECTS += mac_scan.o
C_OBJECTS += mac_security.o
C_OBJECTS += mac_start.o
C_OBJECTS += mac_sync.o
C_OBJECTS += mac_tx_coord_realignment_command.o

## MAC API objects
C_OBJECTS += usr_mcps_purge_conf.o
C_OBJECTS += usr_mlme_get_conf.o
C_OBJECTS += usr_mlme_beacon_notify_ind.o
C_OBJECTS += usr_mlme_disassociate_conf.o
C_OBJECTS += usr_mlme_disassociate_ind.o
C_OBJECTS += usr_mlme_orphan_ind.o
C_OBJECTS += usr_mlme_poll_conf.o
C_OBJECTS += usr_mlme_rx_enable_conf.o
C_OBJECTS += usr_mlme_sync_loss_ind.o

# Append  project directory to output filename
OUTPUT := $(PROJECT)

#-------------------------------------------------------------------------------
#       Build
#-------------------------------------------------------------------------------

all: $(OBJ) $(MEMORIES)

$(OBJ):
	mkdir $@


define RULES
C_OBJECTS_$(1) = $(addprefix $(OBJ)/$(1)_, $(C_OBJECTS))
ASM_OBJECTS_$(1) = $(addprefix $(OBJ)/$(1)_, $(ASM_OBJECTS))

$(1): $$(ASM_OBJECTS_$(1)) $$(C_OBJECTS_$(1))
	$(CC) $(LDFLAGS) -T"$(PATH_LDSCRIPTS)/$$@.lds" -o $(OUTPUT).elf $$^
	$(OBJCOPY) -O binary $(OUTPUT).elf $(OUTPUT).bin
	$(SIZE) $$^ $(OUTPUT).elf


$$(C_OBJECTS_$(1)): $(OBJ)/$(1)_%.o: %.c Makefile $(OBJ)
ifeq ($(SILENT),true)
	@echo compiling $$@
	@-$(CC) $(CFLAGS) -D$(1) -c -o $$@ $$<
else
	$(CC) $(CFLAGS) -D$(1) -c -o $$@ $$<
endif

$$(ASM_OBJECTS_$(1)): $(OBJ)/$(1)_%.o: %.S Makefile $(OBJ)
	$(CC) $(ASFLAGS) -D$(1) -c -o $$@ $$<

ifeq ($(SILENT),true)
	# compiling $$@
	@-$(CC) $(ASMFLAGS) -D$(1) -c -o $$@ $$<
else
	$(CC) $(ASMFLAGS) -D$(1) -c -o $$@ $$<
endif

endef

$(foreach MEMORY, $(MEMORIES), $(eval $(call RULES,$(MEMORY))))

clean:
	-rm -f $(OBJ)/*.o $(OBJ)/*.lst $(OBJ)/.d -r $(OBJ)/.dep
	-rm -f $(PROJECT).bin $(PROJECT).elf $(PROJECT).map
