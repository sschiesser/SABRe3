############################################################################################
# Makefile for the project Star Release Using single source files
############################################################################################

# Build specific properties
_TAL_TYPE = ATMEGARFA1
_PAL_TYPE = ATMEGA128RFA1
_PAL_GENERIC_TYPE = MEGA_RF
_BOARD_TYPE = S2_USBSTICK
_HIGHEST_STACK_LAYER = MAC

_NODE_TYPE = SABRE_COORD
#_NODE_TYPE = SABRE_AM
#_NODE_TYPE = SABRE_LH
#_NODE_TYPE = SABRE_RH

#_PAN_ID = 0x6666
_PAN_ID = 0x5ABE

_DUMMY_DATA = 1
_DEBUG = 0
_VENDOR_BOARDTYPE = 1

_SIO_TYPE = UART0
#_BAUD_RATE = 115200
_BAUD_RATE = 230400

#_DATA_RATE = 500
#_DATA_RATE = 1000
_DATA_RATE = 2000

# Path variables
## Path to main project directory
PATH_ROOT = ./../../../../../Atmel_lib/MAC_v_2_8_0b_ICST
PATH_SABRE_LIB = ./../../../../../SABRe_lib/SABRe_v_3_0
PATH_APP = ./..
PATH_TAL = $(PATH_ROOT)/TAL
PATH_MAC = $(PATH_ROOT)/MAC
PATH_PAL = $(PATH_ROOT)/PAL
PATH_TFA = $(PATH_ROOT)/TFA
PATH_RES = $(PATH_ROOT)/Resources
PATH_GLOB_INC = $(PATH_ROOT)/Include
PATH_SIO = $(PATH_ROOT)/Applications/Helper_Files/SIO_Support

## General Flags
PROJECT = ATMEGA128RFA1_S2_USBSTICK
TARGET_NAME = $(PROJECT)-$(_NODE_TYPE)-D$(_DEBUG)
MCU = atmega128rfa1
TARGET_DIR = ./_bin
COMPILE_DIR = ./GCC
TARGET = $(TARGET_DIR)/$(TARGET_NAME).elf
CC = avr-gcc

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -Werror -g -Wundef -std=c99 -Os
CFLAGS += -DNODE_TYPE=$(_NODE_TYPE)
CFLAGS += -DDEBUG=$(_DEBUG) -DDUMMY_DATA=$(_DUMMY_DATA)
CFLAGS += -DPAN_ID=$(_PAN_ID)
CFLAGS += -DSIO_HUB -D$(_SIO_TYPE) -DBAUD_RATE=$(_BAUD_RATE)
#CFLAGS += -DHIGH_DATA_RATE_SUPPORT -DDATA_RATE=$(_DATA_RATE)
CFLAGS += -DBOARD_TYPE=$(_BOARD_TYPE)
CFLAGS += -DTFA_BAT_MON -DENABLE_TFA
CFLAGS += -DENABLE_TSTAMP
CFLAGS += -DFFD
CFLAGS += -DREDUCED_PARAM_CHECK
CFLAGS += -DTAL_TYPE=$(_TAL_TYPE)
CFLAGS += -DPAL_GENERIC_TYPE=$(_PAL_GENERIC_TYPE)
CFLAGS += -DPAL_TYPE=$(_PAL_TYPE)
CFLAGS += -DVENDOR_BOARDTYPE=$(_VENDOR_BOARDTYPE)
CFLAGS += -DHIGHEST_STACK_LAYER=$(_HIGHEST_STACK_LAYER)
CFLAGS += -MD -MP -MT $(*F).o -MF $(COMPILE_DIR)/dep/$(@F).d

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-g

## Linker flags
LDFLAGS = $(COMMON) -Wl,-Map=$(TARGET_DIR)/$(TARGET_NAME).map -Wl,--section-start=.data=0x800200

## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings

## Include directories for application
INCLUDES = -I$(PATH_APP)/Inc
## Includes SABRe directories
INCLUDES += -I$(PATH_SABRE_LIB)/Inc
## Include directories for general includes
INCLUDES += -I$(PATH_GLOB_INC)
## Include directories for resources
INCLUDES += -I$(PATH_RES)/Buffer_Management/Inc/
INCLUDES += -I$(PATH_RES)/Queue_Management/Inc/
## Include directories for MAC
INCLUDES += -I$(PATH_MAC)/Inc/
## Include directories for TAL
INCLUDES += -I$(PATH_TAL)/Inc/
INCLUDES += -I$(PATH_TAL)/$(_TAL_TYPE)/Inc/
## Include directories for PAL
INCLUDES += -I$(PATH_PAL)/Inc/
INCLUDES += -I$(PATH_PAL)/$(_PAL_GENERIC_TYPE)/Generic/Inc
INCLUDES += -I$(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Inc/
## Include directories for specific boards type
INCLUDES += -I$(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Boards/
INCLUDES += -I$(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Boards/$(_BOARD_TYPE)
## Include directories for SIO support
INCLUDES += -I$(PATH_SIO)/Inc
## Include directories for TFA
INCLUDES += -I$(PATH_TFA)/Inc

## Objects that must be built in order to link
OBJECTS = $(COMPILE_DIR)/main.o\
	$(COMPILE_DIR)/sabre_tasks.o\
	$(COMPILE_DIR)/sabre_ms58.o\
	$(COMPILE_DIR)/pal_uart.o\
	$(COMPILE_DIR)/pal_usb_ftdi.o\
	$(COMPILE_DIR)/pal_sio_hub.o\
	$(COMPILE_DIR)/pal_irq.o\
	$(COMPILE_DIR)/pal_twi.o\
	$(COMPILE_DIR)/pal.o\
	$(COMPILE_DIR)/pal_mcu_generic.o\
	$(COMPILE_DIR)/pal_timer.o\
	$(COMPILE_DIR)/pal_board.o\
	$(COMPILE_DIR)/pal_utils.o\
	$(COMPILE_DIR)/bmm.o\
	$(COMPILE_DIR)/qmm.o\
	$(COMPILE_DIR)/tal.o\
	$(COMPILE_DIR)/tal_rx.o\
	$(COMPILE_DIR)/tal_tx.o\
	$(COMPILE_DIR)/tal_ed.o\
	$(COMPILE_DIR)/tal_slotted_csma.o\
	$(COMPILE_DIR)/tal_pib.o\
	$(COMPILE_DIR)/tal_init.o\
	$(COMPILE_DIR)/tal_irq_handler.o\
	$(COMPILE_DIR)/tal_pwr_mgmt.o\
	$(COMPILE_DIR)/tal_rx_enable.o \
	$(COMPILE_DIR)/mac_associate.o \
	$(COMPILE_DIR)/mac_data_extract_mhr.o \
	$(COMPILE_DIR)/mac_beacon.o \
	$(COMPILE_DIR)/mac_callback_wrapper.o \
	$(COMPILE_DIR)/mac_data_ind.o \
	$(COMPILE_DIR)/mac_data_req.o \
	$(COMPILE_DIR)/mac_disassociate.o \
	$(COMPILE_DIR)/mac_dispatcher.o \
	$(COMPILE_DIR)/mac.o \
	$(COMPILE_DIR)/mac_mcps_data.o \
	$(COMPILE_DIR)/mac_misc.o \
	$(COMPILE_DIR)/mac_orphan.o \
	$(COMPILE_DIR)/mac_pib.o \
	$(COMPILE_DIR)/mac_poll.o \
	$(COMPILE_DIR)/mac_process_beacon_frame.o \
	$(COMPILE_DIR)/mac_process_tal_tx_frame_status.o \
	$(COMPILE_DIR)/mac_rx_enable.o \
	$(COMPILE_DIR)/mac_scan.o \
	$(COMPILE_DIR)/mac_start.o \
	$(COMPILE_DIR)/mac_sync.o \
	$(COMPILE_DIR)/mac_tx_coord_realignment_command.o \
	$(COMPILE_DIR)/mac_api.o \
	$(COMPILE_DIR)/usr_mcps_purge_conf.o \
	$(COMPILE_DIR)/usr_mlme_beacon_notify_ind.o \
	$(COMPILE_DIR)/usr_mlme_disassociate_conf.o \
	$(COMPILE_DIR)/usr_mlme_disassociate_ind.o \
	$(COMPILE_DIR)/usr_mlme_get_conf.o \
	$(COMPILE_DIR)/usr_mlme_orphan_ind.o \
	$(COMPILE_DIR)/usr_mlme_poll_conf.o \
	$(COMPILE_DIR)/usr_mlme_rx_enable_conf.o \
	$(COMPILE_DIR)/usr_mlme_sync_loss_ind.o \
	$(COMPILE_DIR)/sio_handler.o\
	$(COMPILE_DIR)/tfa.o

## Objects explicitly added by the user
LINKONLYOBJECTS =

## Build
all: $(TARGET) $(TARGET_DIR)/$(TARGET_NAME).hex $(TARGET_DIR)/$(TARGET_NAME).eep $(TARGET_DIR)/$(TARGET_NAME).lss size

## Compile
$(COMPILE_DIR)/main.o: $(PATH_APP)/Src/main.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  -o $@ $<
$(COMPILE_DIR)/sabre_tasks.o: $(PATH_SABRE_LIB)/Src/sabre_tasks.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/sabre_ms58.o: $(PATH_SABRE_LIB)/Src/sabre_ms58.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/pal_uart.o: $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/Generic/Src/pal_uart.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/pal_usb_ftdi.o: $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Src/pal_usb_ftdi.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/pal_sio_hub.o: $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Src/pal_sio_hub.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/pal_irq.o: $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Src/pal_irq.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/pal_twi.o: $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/Generic/Src/pal_twi.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/pal.o: $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/Generic/Src/pal.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/pal_mcu_generic.o: $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Src/pal_mcu_generic.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/pal_timer.o: $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Src/pal_timer.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/pal_board.o: $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/$(_PAL_TYPE)/Boards/$(_BOARD_TYPE)/pal_board.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/pal_utils.o: $(PATH_PAL)/$(_PAL_GENERIC_TYPE)/Generic/Src/pal_utils.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/bmm.o: $(PATH_RES)/Buffer_Management/Src/bmm.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/qmm.o: $(PATH_RES)/Queue_Management/Src/qmm.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/tal.o: $(PATH_TAL)/$(_TAL_TYPE)/Src/tal.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/tal_rx.o: $(PATH_TAL)/$(_TAL_TYPE)/Src/tal_rx.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/tal_tx.o: $(PATH_TAL)/$(_TAL_TYPE)/Src/tal_tx.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/tal_ed.o: $(PATH_TAL)/$(_TAL_TYPE)/Src/tal_ed.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/tal_slotted_csma.o: $(PATH_TAL)/$(_TAL_TYPE)/Src/tal_slotted_csma.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/tal_pib.o: $(PATH_TAL)/$(_TAL_TYPE)/Src/tal_pib.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/tal_init.o: $(PATH_TAL)/$(_TAL_TYPE)/Src/tal_init.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/tal_irq_handler.o: $(PATH_TAL)/$(_TAL_TYPE)/Src/tal_irq_handler.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/tal_pwr_mgmt.o: $(PATH_TAL)/$(_TAL_TYPE)/Src/tal_pwr_mgmt.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/tal_rx_enable.o: $(PATH_TAL)/$(_TAL_TYPE)/Src/tal_rx_enable.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_associate.o: $(PATH_MAC)/Src/mac_associate.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_data_extract_mhr.o: $(PATH_MAC)/Src/mac_data_extract_mhr.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_beacon.o: $(PATH_MAC)/Src/mac_beacon.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_callback_wrapper.o: $(PATH_MAC)/Src/mac_callback_wrapper.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_data_ind.o: $(PATH_MAC)/Src/mac_data_ind.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_data_req.o: $(PATH_MAC)/Src/mac_data_req.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_disassociate.o: $(PATH_MAC)/Src/mac_disassociate.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_dispatcher.o: $(PATH_MAC)/Src/mac_dispatcher.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac.o: $(PATH_MAC)/Src/mac.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_mcps_data.o: $(PATH_MAC)/Src/mac_mcps_data.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_misc.o: $(PATH_MAC)/Src/mac_misc.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_orphan.o: $(PATH_MAC)/Src/mac_orphan.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_pib.o: $(PATH_MAC)/Src/mac_pib.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_poll.o: $(PATH_MAC)/Src/mac_poll.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_process_beacon_frame.o: $(PATH_MAC)/Src/mac_process_beacon_frame.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_process_tal_tx_frame_status.o: $(PATH_MAC)/Src/mac_process_tal_tx_frame_status.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_rx_enable.o: $(PATH_MAC)/Src/mac_rx_enable.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_scan.o: $(PATH_MAC)/Src/mac_scan.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_start.o: $(PATH_MAC)/Src/mac_start.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_sync.o: $(PATH_MAC)/Src/mac_sync.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_tx_coord_realignment_command.o: $(PATH_MAC)/Src/mac_tx_coord_realignment_command.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/usr_mcps_data_conf.o: $(PATH_MAC)/Src/usr_mcps_data_conf.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/usr_mcps_data_ind.o: $(PATH_MAC)/Src/usr_mcps_data_ind.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/usr_mcps_purge_conf.o: $(PATH_MAC)/Src/usr_mcps_purge_conf.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/usr_mlme_associate_conf.o: $(PATH_MAC)/Src/usr_mlme_associate_conf.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/usr_mlme_associate_ind.o: $(PATH_MAC)/Src/usr_mlme_associate_ind.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/usr_mlme_beacon_notify_ind.o: $(PATH_MAC)/Src/usr_mlme_beacon_notify_ind.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/usr_mlme_comm_status_ind.o: $(PATH_MAC)/Src/usr_mlme_comm_status_ind.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/usr_mlme_disassociate_conf.o: $(PATH_MAC)/Src/usr_mlme_disassociate_conf.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/usr_mlme_disassociate_ind.o: $(PATH_MAC)/Src/usr_mlme_disassociate_ind.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/usr_mlme_get_conf.o: $(PATH_MAC)/Src/usr_mlme_get_conf.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/usr_mlme_orphan_ind.o: $(PATH_MAC)/Src/usr_mlme_orphan_ind.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/usr_mlme_poll_conf.o: $(PATH_MAC)/Src/usr_mlme_poll_conf.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/usr_mlme_reset_conf.o: $(PATH_MAC)/Src/usr_mlme_reset_conf.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/usr_mlme_rx_enable_conf.o: $(PATH_MAC)/Src/usr_mlme_rx_enable_conf.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/usr_mlme_scan_conf.o: $(PATH_MAC)/Src/usr_mlme_scan_conf.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/usr_mlme_set_conf.o: $(PATH_MAC)/Src/usr_mlme_set_conf.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/usr_mlme_start_conf.o: $(PATH_MAC)/Src/usr_mlme_start_conf.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/usr_mlme_sync_loss_ind.o: $(PATH_MAC)/Src/usr_mlme_sync_loss_ind.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/mac_api.o: $(PATH_MAC)/Src/mac_api.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/sio_handler.o: $(PATH_SIO)/Src/sio_handler.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
$(COMPILE_DIR)/tfa.o: $(PATH_TFA)/$(_TAL_TYPE)/Src/tfa.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<



##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

## avr-size options
ifdef windir
SIZEFLAGS = -C --mcu=${MCU}
else
ifdef WINDIR
SIZEFLAGS = -C --mcu=${MCU}
else
SIZEFLAGS = -B
endif
endif

size: ${TARGET}
	@echo
	@avr-size $(SIZEFLAGS) ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(COMPILE_DIR)/*.o $(TARGET_DIR)/$(TARGET_NAME).elf $(COMPILE_DIR)/dep/* $(TARGET_DIR)/$(TARGET_NAME).hex $(TARGET_DIR)/$(TARGET_NAME).eep $(TARGET_DIR)/$(TARGET_NAME).lss $(TARGET_DIR)/$(TARGET_NAME).map

##Options for null device
ifdef windir
NULLDEV = NUL:
else
ifdef WINDIR
NULLDEV = NUL:
else
NULLDEV = /dev/null
endif
endif
## Other dependencies
-include $(shell mkdir $(COMPILE_DIR)/dep 2>$(NULLDEV)) $(wildcard $(COMPILE_DIR)/dep/*)

