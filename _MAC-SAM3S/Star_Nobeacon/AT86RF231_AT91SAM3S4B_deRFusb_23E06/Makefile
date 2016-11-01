############################################################################################
# Makefile for the project Star Release Using single source files
############################################################################################

# Build specific properties
_TAL_TYPE = AT86RF231
_PAL_TYPE = AT91SAM3S4B
_PAL_GENERIC_TYPE = SAM3
_BOARD_TYPE = deRFusb_23E06
_HIGHEST_STACK_LAYER = MAC

_NODE_TYPE = SABRE_COORD
#_NODE_TYPE = SABRE_AM
#_NODE_TYPE = SABRE_LH
#_NODE_TYPE = SABRE_RH

_DEBUG = 1

_SIO_TYPE = USB0
#_BAUD_RATE = 115200
_BAUD_RATE = 230400

#_DATA_RATE = 500
#_DATA_RATE = 1000
_DATA_RATE = 2000

# available memories on target 
MEMORIES = flash

## Trace level used
# (can be overriden by adding TRACE_LEVEL=#number to the command-line)
# TRACE_LEVEL_DEBUG      5
# TRACE_LEVEL_INFO       4
# TRACE_LEVEL_WARNING    3
# TRACE_LEVEL_ERROR      2
# TRACE_LEVEL_FATAL      1
# TRACE_LEVEL_NO_TRACE   0
TRACE_LEVEL = 0

# verbosity level of compilation, uncomment for verbose make console output
SILENT = true

# Path variables
PATH_ROOT = ../../../../../Atmel_lib/MAC_v_2_6_1
PATH_SABRE_LIB = ./../../../../../SABRe_lib/SABRe_v_3_0
## !! Application located in the _MAC-megaRF folder to keep files synchronized !!
PATH_APP = ./../../../_MAC-megaRF/Star_Nobeacon
## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
PATH_TAL = $(PATH_ROOT)/TAL
PATH_MAC = $(PATH_ROOT)/MAC
PATH_TAL_CB = $(PATH_ROOT)/TAL/Src
PATH_PAL = $(PATH_ROOT)/PAL
PATH_RES = $(PATH_ROOT)/Resources
PATH_SIO = $(PATH_ROOT)/Applications/Helper_Files/SIO_Support
PATH_AT91LIB = $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/libraries
PATH_STARTUP = $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Startup
PATH_LDSCRIPTS = $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Linker_scripts

# Project name and output files destinations
PROJECT = Star
BIN_DIR = _bin
OBJ_DIR = _obj
OUTPUT := $(BIN_DIR)/$(PROJECT)


#------------------------------------------------------------------------------
# Toolchain
#------------------------------------------------------------------------------

# Tool suffix when cross-compiling, either 'arm-elf-' or 'arm-none-eabi-'
TOOLCHAIN = arm-none-eabi-
# Compilation tools
CC     = $(TOOLCHAIN)gcc
CP     = $(TOOLCHAIN)objcopy
SIZE   = $(TOOLCHAIN)size
STRIP  = $(TOOLCHAIN)strip

## Options common to compile, link and assembly rules
# General Flags
MCU    = cortex-m3
COMMON = -mcpu=$(MCU) -mthumb

# comment out, if debugging
OPTFLAGS = -O0 -ffunction-sections -mlong-calls

## Compile options common for all C compilation units.
CFLAGS += $(COMMON) 
CFLAGS += $(INCLUDES)
CFLAGS += -Wall -Werror -g -Wundef -std=c99 $(OPTFLAGS)
CFLAGS += -Wno-parentheses
CFLAGS += -DDEBUG=$(_DEBUG) -DNOASSERT
CFLAGS += -DREDUCED_PARAM_CHECK
CFLAGS += -DFFD
#CFLAGS += -DSIO_HUB -D$(_SIO_TYPE) -DBAUD_RATE=$(_BAUD_RATE)
CFLAGS += -DNODE_TYPE=$(_NODE_TYPE)
CFLAGS += -DHIGH_DATA_RATE_SUPPORT -DDATA_RATE=$(_DATA_RATE)
CFLAGS += -DENABLE_TSTAMP
CFLAGS += -DTAL_TYPE=$(_TAL_TYPE)
CFLAGS += -DPAL_GENERIC_TYPE=$(_PAL_GENERIC_TYPE)
CFLAGS += -DPAL_TYPE=$(_PAL_TYPE)
CFLAGS += -DVENDOR_BOARDTYPES=1
CFLAGS += -DBOARD_TYPE=$(_BOARD_TYPE)
CFLAGS += -DHIGHEST_STACK_LAYER=$(_HIGHEST_STACK_LAYER)
CFLAGS += -MD -MP -MT $(*F).o -MF $(OBJ_DIR)/$(@F).d
# AT91-specific
CFLAGS += -mthumb-interwork  
CFLAGS += -fomit-frame-pointer
CFLAGS += -DTRACE_LEVEL=$(TRACE_LEVEL)

## Assembly specific flags
ASFLAGS =  $(COMMON) -Wall -g $(OPTFLAGS) $(INCLUDES) -D__ASSEMBLY__

## Linker flags
LDFLAGS =  $(COMMON) -g $(OPTFLAGS) -nostartfiles -Wl,--gc-sections

#------------------------------------------------------------------------------
# Include paths, files to build
#------------------------------------------------------------------------------

# Include directories for application
INCLUDES = -I $(PATH_APP)/Inc
# Includes SABRe directories
INCLUDES += -I$(PATH_SABRE_LIB)/Inc
# Include directories for general includes
INCLUDES += -I $(PATH_ROOT)/Include/
# Include directories for resources
INCLUDES += -I$(PATH_ROOT)/Resources/Buffer_Management/Inc/
INCLUDES += -I$(PATH_ROOT)/Resources/Queue_Management/Inc/
# Include directories for MAC
INCLUDES += -I$(PATH_ROOT)/MAC/Inc/
# Include directories for TAL
INCLUDES += -I$(PATH_ROOT)/TAL/Inc/
INCLUDES += -I$(PATH_ROOT)/TAL/$(_TAL_TYPE)/Inc/
# Include directories for PAL
INCLUDES += -I$(PATH_ROOT)/PAL/Inc/
INCLUDES += -I$(PATH_ROOT)/PAL/$(_PAL_GENERIC_TYPE)/Generic/Inc
INCLUDES += -I$(PATH_ROOT)/PAL/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Boards/
# Include directories for specific boards types
INCLUDES += -I$(PATH_ROOT)/PAL/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Boards/$(_BOARD_TYPE)
INCLUDES += -I$(PATH_ROOT)/PAL/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Inc
# Include directories for SIO support
INCLUDES += -I$(PATH_SIO)/Inc
# AT91Lib 
INCLUDES += -I$(PATH_AT91LIB)/cmsis
INCLUDES += -I$(PATH_AT91LIB)/drivers
INCLUDES += -I$(PATH_AT91LIB)
# USB descriptors 
INCLUDES += -I ./../descriptors

## paths to search for source files
VPATH += $(PATH_APP)/Src
VPATH += $(PATH_SABRE_LIB)/Src
VPATH += $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/Generic/Src
VPATH += $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Src
VPATH += $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Boards/$(_BOARD_TYPE)
VPATH += $(PATH_MAC)/Src
VPATH += $(PATH_TAL)/$(_TAL_TYPE)/Src
VPATH += $(PATH_RES)/Buffer_Management/Src
VPATH += $(PATH_RES)/Queue_Management/Src
VPATH += $(PATH_SIO)/Src
VPATH += $(PATH_AT91LIB)/
VPATH += $(PATH_AT91LIB)/cmsis
VPATH += $(PATH_AT91LIB)/drivers/pio
VPATH += $(PATH_AT91LIB)/drivers/pmc
VPATH += $(PATH_AT91LIB)/drivers/usart
VPATH += $(PATH_AT91LIB)/drivers/utility
VPATH += $(PATH_STARTUP)

## Objects built from C source files
# main application
C_OBJECTS += main.o
C_OBJECTS += sabre_tasks.o
C_OBJECTS += fifo.o
# stack sources
C_OBJECTS += pal.o
C_OBJECTS += pal_timer.o
C_OBJECTS += pal_trx_access.o
C_OBJECTS += pal_utils.o
C_OBJECTS += bmm.o
C_OBJECTS += qmm.o
C_OBJECTS += tal.o
C_OBJECTS += tal_rx.o
C_OBJECTS += tal_tx.o
C_OBJECTS += tal_ed.o
C_OBJECTS += tal_slotted_csma.o
C_OBJECTS += tal_pib.o
C_OBJECTS += tal_init.o
C_OBJECTS += tal_irq_handler.o
C_OBJECTS += tal_pwr_mgmt.o
C_OBJECTS += tal_rx_enable.o
C_OBJECTS += mac_associate.o
C_OBJECTS += mac_beacon.o
C_OBJECTS += mac_callback_wrapper.o
C_OBJECTS += mac_data_ind.o
C_OBJECTS += mac_data_req.o
C_OBJECTS += mac_disassociate.o
C_OBJECTS += mac_dispatcher.o
C_OBJECTS += mac.o
C_OBJECTS += mac_mcps_data.o
C_OBJECTS += mac_misc.o
C_OBJECTS += mac_orphan.o
C_OBJECTS += mac_pib.o
C_OBJECTS += mac_poll.o
C_OBJECTS += mac_process_beacon_frame.o
C_OBJECTS += mac_process_tal_tx_frame_status.o
C_OBJECTS += mac_rx_enable.o
C_OBJECTS += mac_scan.o
C_OBJECTS += mac_start.o
C_OBJECTS += mac_sync.o
C_OBJECTS += mac_tx_coord_realignment_command.o
C_OBJECTS += mac_api.o
C_OBJECTS += usr_mcps_purge_conf.o
C_OBJECTS += usr_mlme_beacon_notify_ind.o
C_OBJECTS += usr_mlme_disassociate_conf.o
C_OBJECTS += usr_mlme_disassociate_ind.o
C_OBJECTS += usr_mlme_get_conf.o
C_OBJECTS += usr_mlme_orphan_ind.o
C_OBJECTS += usr_mlme_poll_conf.o
C_OBJECTS += usr_mlme_rx_enable_conf.o
C_OBJECTS += usr_mlme_sync_loss_ind.o
# platform dependent
C_OBJECTS += pal_irq.o
C_OBJECTS += pal_board.o
C_OBJECTS += board_cstartup_gnu.o
C_OBJECTS += exceptions.o
# SIO support
C_OBJECTS += sio_handler.o
# AT91Lib sources
C_OBJECTS += core_cm3.o
C_OBJECTS += led.o
C_OBJECTS += pio.o
C_OBJECTS += pmc.o
C_OBJECTS += uart_console.o
C_OBJECTS += stdio.o

## Objects built from Assembly source files
ASM_OBJECTS = 


# -----------------------------------------------------------------------------
# make targets and -rules
# -----------------------------------------------------------------------------

# include file with flash rules
include $(PATH_ROOT)/Flash_SAM3.in

all: $(BIN_DIR) $(OBJ_DIR) $(MEMORIES)

$(BIN_DIR) $(OBJ_DIR):
	mkdir $@

define RULES

C_OBJECTS_$(1) = $(addprefix $(OBJ_DIR)/$(1)_, $(C_OBJECTS))
ASM_OBJECTS_$(1) = $(addprefix $(OBJ_DIR)/$(1)_, $(ASM_OBJECTS))

$(1): $$(ASM_OBJECTS_$(1)) $$(C_OBJECTS_$(1))
	@echo --------------------------------------------------------------------
	@echo linking
	@echo --------------------------------------------------------------------
	$(CC) $(LDFLAGS) -T$(PATH_LDSCRIPTS)/$$@.lds -Wl,-Map=$(OUTPUT)-$$@.map \
					-o $(OUTPUT)-$$@.elf $$^
	@echo --------------------------------------------------------------------
	@echo creating BIN file
	@echo --------------------------------------------------------------------
	$(CP) -O binary $(OUTPUT)-$$@.elf $(OUTPUT)-$$@.bin
	@echo --------------------------------------------------------------------
	@echo determine file size
	@echo --------------------------------------------------------------------
	@-$(SIZE) $$^ $(OUTPUT)-$$@.elf

$$(C_OBJECTS_$(1)): $(OBJ_DIR)/$(1)_%.o: %.c Makefile $(OBJ_DIR) $(BIN_DIR)
ifeq ($(SILENT),true)
	@echo compiling $$@
	@-$(CC) $(CFLAGS) -D$(1) -c -o $$@ $$<
else
	$(CC) $(CFLAGS) -D$(1) -c -o $$@ $$<	
endif

$$(ASM_OBJECTS_$(1)): $(OBJ_DIR)/$(1)_%.o: %.S Makefile $(OBJ_DIR) $(BIN_DIR)
ifeq ($(SILENT),true)
	@echo compiling $$@
	@-$(CC) $(ASMFLAGS) -D$(1) -c -o $$@ $$<
else
	$(CC) $(ASMFLAGS) -D$(1) -c -o $$@ $$<
endif	

endef

$(foreach MEMORY, $(MEMORIES), $(eval $(call RULES,$(MEMORY))))

clean:
	-rm -f $(OBJ_DIR)/*.o $(OBJ_DIR)/*.lst $(OBJ_DIR)/.d -r $(OBJ_DIR)/.dep
	-rm -f $(BIN_DIR)/*.bin $(BIN_DIR)/*.elf $(BIN_DIR)/*.map
	
