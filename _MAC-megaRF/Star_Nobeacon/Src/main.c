/*
 * @file main.c
 *
 * @brief  MAC Example - Star Network
 *
 * This is the source code of a simple MAC example. It implements the
 * firmware for all nodes of a network with star topology.
 *
 * $Id: main.c 24937 2011-01-18 04:02:51Z yogesh.bellan $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2009, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* === INCLUDES ============================================================= */

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "pal.h"
#include "pal_twi.h"
#include "pal_spi.h"
#include "tal.h"
#include "mac_api.h"
#include "sio_handler.h"
#include "app_config.h"
#include "sabre_config.h"
#include "sabre_ms58.h"
#include "sabre_imu.h"
#include "ieee_const.h"
/** Only during development phase */
#include <util/delay.h>

/* === TYPES ================================================================ */

/* === MACROS =============================================================== */

/* === GLOBALS ============================================================== */

/** Debug mode flag */
bool debug_mode;
/** Application state enumerator */
app_state_t app_state;
/** Saved timestamp values for time difference calculation */
uint32_t OldTimestamp; /* Timestamp of the SABRe frame */
uint32_t OldSiotime;   /* SIO time (coordinator) for data sending to host
                          computer */
uint32_t OldTrxtime;   /* TRX time (node) for data sending to coordinator */

/** Short and extended address of the coordinator */
associated_device_t coord_addr;
/** Number of done network scans */
uint8_t number_of_scans;
/** Informations related to all associated devices */
associated_device_t device_list[MAX_NUMBER_OF_DEVICES];

/* === EXTERNALS ============================================================ */
/*****************************
  * ...from sabre_tasks.c... *
  ****************************/
/** SABRe node identificator */
extern sabre_node_t sabre_node;
/** Data buffers and respective counters */
extern uint8_t **lh_trx_buffer;
extern uint8_t **rh_trx_buffer;
extern uint8_t **am_trx_buffer;
extern uint8_t *sio_buffer;
extern uint8_t lh_buffer_cnt;
extern uint8_t rh_buffer_cnt;
extern uint8_t am_buffer_cnt;
/** Interrupt flags */
//extern volatile uint8_t sw0_on;
//extern volatile uint8_t sw1_on;
//extern volatile uint8_t sw2_on;
extern volatile bool accel_ready;
extern volatile bool gyro_ready;
extern volatile bool comp_ready;
/** IMU version information */
extern imu_version_t imu_version;

extern bool imu_available;  /* Doesn't come from an interruption vector, but
                               is activated when the IMU initialization routine
							   ended successful. */
extern bool ack_received;
/****************************
  * ...from sabre_ms58.c... *
  ***************************/
/** PROM values of the ms5803-01ba pressure sensor */
extern uint16_t ms58_prom_values[MS58_PROM_VALUES_MAX];
//extern uint32_t ms58_adc_values[MS58_ADC_VALUES_MAX];


/* === PROTOTYPES ========================================================== */
/** Application initialization routine */
static void app_init(sabre_node_t ntype);
/** SIO sending function */
static void sio_task(void);
static void network_scan_indication_cb(void *parameter);
//static void data_led_off_cb(void *parameter);
static bool assign_new_short_addr(uint64_t addr64, uint16_t *addr16);
/** FAKE FUNCTION FOR DEVELOPEMENT */
extern bool sabre_imu_FAKE_init(void);
//extern bool sabre_am_FAKE_init(void);
extern void sabre_keys_FAKE_init(void);
extern void sabre_battery_init(void);
extern bool sabre_imu_start(imu_version_t version);

/* === IMPLEMENTATION ======================================================= */
/**
 * @brief Main function of the device application
 */
int main(void)
{
	app_state = SABRE_APP_STARTING;
	
	/* Initialize the node, depending on node type and compiler flags */
	app_init(sabre_node);
	
	//pal_alert();
	
    /* Main loop */
    while (1) {
	    wpan_task();
		sabre_task(sabre_node);
		if((COORDINATOR == sabre_node)) {
			sio_task();
		}
    }
}


/**
 * @brief Application initialization function
 *
 * @param ntype type of node in the network
 */
void app_init(sabre_node_t ntype) {	
	/************************************************************************/
	/* Verify if the node type has been correctly defined                   */
	/************************************************************************/
	if(_NDEF == sabre_node) {
		pal_alert();
	}
	
	/************************************************************************/
	/* Initialize the MAC layer and its underlying layers,                  */
	/* like PAL, TAL, BMM.                                                  */
	/************************************************************************/
    if (wpan_init() != MAC_SUCCESS) {
        /*
         * Stay here; we need a valid IEEE address.
         * Check kit documentation how to create an IEEE address
         * and to store it into the EEPROM.
         */
        pal_alert();
    }

	/************************************************************************/
	/* Check compiler debug mode and set flag                               */
	/************************************************************************/
#if (DEBUG > 0)
	debug_mode = true;
#else
	debug_mode = false;
#endif

#if (DUMMY_DATA > 0)
#define SABRE_GENERATE_DUMMY
#endif


	/************************************************************************/
	/* Initialize LEDs.                                                     */
	/************************************************************************/
    pal_led_init();
    pal_led(LED_START, LED_ON);         // indicating application is started
    pal_led(LED_NWK_SETUP, LED_OFF);    // indicating node is associated
    pal_led(LED_DATA, LED_OFF);         // indicating successful data transmission

	/************************************************************************/
	/* Coordinator waits an some time before starting network               */
	/************************************************************************/
	if(COORDINATOR == sabre_node) {
		uint8_t i;
		// 100 * 10000 us should give about 1 s
		for(i = 0; i < 50; i++) {
			pal_timer_delay(10000);
		}
	}
	
	/************************************************************************/
	/* The stack is initialized above, hence the global interrupts are      */
	/* enabled here.                                                        */
	/************************************************************************/
	pal_global_irq_enable();

	/************************************************************************/
	/* Initialize SIO                                                       */
	/************************************************************************/
#ifdef SIO_HUB
	/* Initialize SIO @ BAUD_RATE */
	if (pal_sio_init(SIO_CHANNEL) != MAC_SUCCESS) {
		/* Something went wrong during initialization. */
		pal_alert();
	}

#if ((!defined __ICCAVR__) && (!defined __ICCARM__) && (!defined __GNUARM__) && \
		(!defined __ICCAVR32__) && (!defined __GNUAVR32__))
		fdevopen(_sio_putchar, _sio_getchar);
#endif

	if(debug_mode) {
		/* To make sure the hyper terminal gets connected to the system */
		//sio_getchar();

		printf("****************************\n");
		printf("* SABRe v3.0               *\n");
		printf("****************************\n");

		switch (ntype) {
		case COORDINATOR:
			printf("* - coordinator            *\n");
			break;
		case LEFTHAND:
			printf("* - left hand              *\n");
			break;
		case RIGHTHAND:
			printf("* - right hand             *\n");
			break;
		case AIRMEMS:
			printf("* - airmems                *\n");
			break;
		default:
			pal_alert();
		}

#ifdef BEACON_SUPPORT
		printf("* - beacon                 *\n");
#else
		printf("* - no beacon              *\n");
#endif /* #ifdef BEACON_SUPPORT */

#ifdef HIGH_DATA_RATE_SUPPORT
#if (DEFAULT_CHANNEL_PAGE == 17)
		printf("* - data rate: 2Mbps       *\n");
#elif (DEFAULT_CHANNEL_PAGE == 16)
		printf("* - data rate: 1Mbps       *\n");
#elif (DEFAULT_CHANNEL_PAGE == 2)
		printf("* - data rate: 500kbps     *\n");
#elif (DEFAULT_CHANNEL_PAGE == 0)
		printf("* - data rate: 250kbps     *\n");
#endif /* #if (DEFAULT_CHANNEL_PAGE ==...) */
#else
		printf("* - data rate: 250kbps     *\n");
#endif /* #ifdef HIGH_DATA_RATE_SUPPORT */

		printf("****************************\n\n");
	}

#endif /* #ifdef SIO_HUB */

	/************************************************************************/
	/* Counters for incoming buffers (LH, RH, AM),                          */
	/* to know first which have been updated and have to be sent,           */
	/* second how many have been dropped                                    */
	/************************************************************************/
	imu_version = SABRE_IMU_MPU9150;
	lh_buffer_cnt = 0;
	rh_buffer_cnt = 0;
	am_buffer_cnt = 0;
	
	OldTimestamp = 0;
	OldSiotime = 0;
	OldTrxtime = 0;
	
	accel_ready = false;
	gyro_ready = false;
	comp_ready = false;
		
	/************************************************************************/
	/* Allocate memory to the node buffers                                  */
	/************************************************************************/
	if((LEFTHAND == sabre_node) || (COORDINATOR == sabre_node)) {
		uint8_t i, j;
		lh_trx_buffer = malloc(SABRE_TRX_BUF_DEPTH * sizeof(uint8_t*));
		for(i = 0; i < SABRE_TRX_BUF_DEPTH; i++) {
			lh_trx_buffer[i] = malloc(SABRE_LH_MESSLEN * sizeof(uint8_t));
		}
		for(i = 0; i < SABRE_TRX_BUF_DEPTH; i++) {
			for(j = 0; j < SABRE_LH_MESSLEN; j++) {
				lh_trx_buffer[i][j] = 0;
			}
		}
		if(debug_mode) {
			printf("Memory allocated to lh_trx_buffer: %d x %d (x 8) = %d bits\n",
			        SABRE_TRX_BUF_DEPTH, SABRE_LH_MESSLEN,
					SABRE_TRX_BUF_DEPTH*SABRE_LH_MESSLEN*8);
		}
	}
	if((RIGHTHAND == sabre_node) || (COORDINATOR == sabre_node)) {
		uint8_t i, j;
		rh_trx_buffer = malloc(SABRE_TRX_BUF_DEPTH * sizeof(uint8_t*));
		for(i = 0; i < SABRE_TRX_BUF_DEPTH; i++) {
			rh_trx_buffer[i] = malloc(SABRE_RH_MESSLEN * sizeof(uint8_t));
		}
		for(i = 0; i < SABRE_TRX_BUF_DEPTH; i++) {
			for(j = 0; j < SABRE_RH_MESSLEN; j++) {
				rh_trx_buffer[i][j] = 0;
			}
		}
		if(debug_mode) {
			printf("Memory allocated to rh_trx_buffer: %d x %d (x 8) = %d bits\n",
			        SABRE_TRX_BUF_DEPTH, SABRE_RH_MESSLEN,
					SABRE_TRX_BUF_DEPTH*SABRE_RH_MESSLEN*8);
		}
	}
	if((AIRMEMS == sabre_node) || (COORDINATOR == sabre_node)) {
		uint8_t i, j;
		am_trx_buffer = malloc(SABRE_TRX_BUF_DEPTH * sizeof(uint8_t*));
		for(i = 0; i < SABRE_TRX_BUF_DEPTH; i++) {
			am_trx_buffer[i] = malloc(SABRE_AM_MESSLEN * sizeof(uint8_t));
		}
		for(i = 0; i < SABRE_TRX_BUF_DEPTH; i++) {
			for(j = 0; j < SABRE_AM_MESSLEN; j++) {
				am_trx_buffer[i][j] = 0;
			}
		}
		if(debug_mode) {
			printf("Memory allocated to am_trx_buffer: %d x %d (x 8) = %d bits\n",
			        SABRE_TRX_BUF_DEPTH, SABRE_AM_MESSLEN,
					SABRE_TRX_BUF_DEPTH*SABRE_AM_MESSLEN*8);
		}
	}
	if(COORDINATOR == sabre_node) {
		uint8_t i;
		sio_buffer = malloc(SIO_BUFFER_SIZE * sizeof(uint8_t));
		for(i = 0; i < SIO_BUFFER_SIZE; i++) {
			sio_buffer[i] = 0;
		}
		if(debug_mode) {
			printf("Memory allocated to sio_buffer: %d (x 8) = %d bits\n",
			        SIO_BUFFER_SIZE, SIO_BUFFER_SIZE*8);
		}

		#if defined(SABRE_USB_HIGHSPEED)
			pal_spi_init(SPI_SPEED_FCPU_DIV_4 | SPI_MODE_MASTER);
		#endif
	}
	
	/************************************************************************/
	/* Initialize SABRe nodes                                               */
	/************************************************************************/
	if(LEFTHAND == sabre_node) {
		sabre_switches_init();
	}
	if((LEFTHAND == sabre_node) || (RIGHTHAND == sabre_node)) {
		//sabre_keys_FAKE_init();
		sabre_keys_init();
	}
	if(RIGHTHAND == sabre_node) {
#if ((PAL_GENERIC_TYPE == AVR) || (PAL_GENERIC_TYPE == MEGA_RF))
	sabre_battery_init();
#endif
	#if !defined(SABRE_GENERATE_DUMMY)
		if(!sabre_imu_init(imu_version)) {
			if(debug_mode) {
				printf("IMU not connected or bus error!\n");
			}
			imu_available = false;
		}
		else {
			if(debug_mode) {
				printf("done!\n");
			}
			imu_available = true;
		}
	#endif
	}
	if(AIRMEMS == sabre_node) {
#if ((PAL_GENERIC_TYPE == AVR) || (PAL_GENERIC_TYPE == MEGA_RF))
	sabre_battery_init();
#endif
		if(!sabre_airmems_init()) {
		//if(!sabre_am_FAKE_init()) {
			if(debug_mode) {
				printf("airMEMS not connected or initialization failed!"\
				       "Exiting program!\n");
			}
			pal_alert();
		}
		else {
			if(debug_mode) {
				printf("done!\n");
			}
		}
	}

    /************************************************************************/
    /* Reset the MAC layer to the default values                            */
    /* This request will cause a mlme reset confirm message ->              */
    /* usr_mlme_reset_conf                                                  */
    /************************************************************************/
	app_state = SABRE_WPAN_STARTING;

	if(debug_mode) {
		printf("Requesting network reset... ");
	}

    wpan_mlme_reset_req(true);
}


/**
 * @brief SIO queue management function
 *
 * @param none
 */
void sio_task(void) {
	if( (SABRE_APP_STARTING != app_state) && (SABRE_WPAN_STARTING != app_state) ) {
		bool sio_buf_overflow;
		uint8_t i, offset, sio_buf_length, trx_buf_depth;
		uint32_t Currenttime, Elapsedtime, Lt1, Lt2, Dt;
	
		/* SIO transmission can be different than TRX reception.
		 * The Elapsedtime value is fetched as often than possible,
		 * but SIO transmission is activated only when the
		 * SABRE_SIO_TX_PERIOD threshold has been reached
		 */
		pal_get_current_time(&Currenttime);
		Elapsedtime = Currenttime - OldSiotime;
		
		//if(debug_mode) {
			//printf("Current time: %lu us\nElapsed time: %lu us\n", Currenttime, Elapsedtime);
		//}
	
		if(Elapsedtime > SABRE_SIO_PERIOD) {
			OldSiotime = Currenttime; // Save the new OldSiotime for next loop.
			if(debug_mode) {
				pal_get_current_time(&Lt1);
				printf("\nSIO period: %lu.%03d ms\nBuffer counters: LH %i, RH %i, AM %i\n",
					(uint32_t)(Elapsedtime / 1000), (uint16_t)(Elapsedtime % 1000),
					lh_buffer_cnt, rh_buffer_cnt, am_buffer_cnt);
			}
		
			/* If one of the buffer counter is positive, some data have been received.
			 * Put all the available data into the sio_buffer and send them.
			 * If too many data have been buffered and
			 */
			offset = 0;
			sio_buf_overflow = false;
			trx_buf_depth = lh_buffer_cnt;
			if((lh_buffer_cnt > 0) && (!sio_buf_overflow)) {
				while(lh_buffer_cnt > 0) {
					for(i = 0; i < SABRE_LH_MESSLEN; i++) {
						sio_buffer[offset + i] = lh_trx_buffer[(trx_buf_depth-lh_buffer_cnt)][i];
					}
					lh_buffer_cnt--;
					if(offset > (0xff - SABRE_LH_MESSLEN)) {
						sio_buf_overflow = true;
						break;
					}
					else {
						offset += SABRE_LH_MESSLEN;
					}
				}
			}
			
			trx_buf_depth = rh_buffer_cnt;
			if((rh_buffer_cnt > 0) && (!sio_buf_overflow)) {
				while(rh_buffer_cnt > 0) {
					for (i = 0; i < SABRE_RH_MESSLEN; i++) {
						sio_buffer[offset + i] = rh_trx_buffer[(trx_buf_depth-rh_buffer_cnt)][i];
					}
					rh_buffer_cnt--;
					if(offset > (0xff - SABRE_RH_MESSLEN)) {
						sio_buf_overflow = true;
						break;
					}
					else {
						offset += SABRE_RH_MESSLEN;
					}
				}
			}
			
			trx_buf_depth = am_buffer_cnt;
			if((am_buffer_cnt > 0) & (!sio_buf_overflow)) {
				while(am_buffer_cnt > 0) {
					for (i = 0; i < SABRE_AM_MESSLEN; i++) {
						sio_buffer[offset + i] = am_trx_buffer[(trx_buf_depth-am_buffer_cnt)][i];
					}
					am_buffer_cnt--;
					if(offset > (0xff - SABRE_AM_MESSLEN)) {
						sio_buf_overflow = true;
						break;
					}
					else {
						offset += SABRE_AM_MESSLEN;
					}
				}
			}
			
			sio_buf_length = offset;
			if(debug_mode) {
				if(sio_buf_overflow) {
					printf("SIO buffer overflow!!!\n");
				}
				for(i = 0; i < sio_buf_length; i++) {
					printf("0x%02x ", sio_buffer[i]);
				}
				pal_get_current_time(&Lt2);
				Dt = Lt2- Lt1;
				printf("\nTime used by the SIO: %lu.%03d ms\n\n", (uint32_t)(Dt / 1000), (uint16_t)(Dt % 1000));
			}
			else {
				#if defined(SABRE_USB_HIGHSPEED)
					if(sio_buf_length > 0) {
 						SPI_PORT &= ~(1 << SEL);
						for(i = 0; i < sio_buf_length; i++) {
							pal_spi_send_byte(sio_buffer[i]);
						}
						SPI_PORT |= (1 << SEL);
						//sio_binarywrite(&sio_buffer[0], sio_buf_length);
					}
				#else
					sio_binarywrite(&sio_buffer[0], sio_buf_length);
					//for (i = 0; i < sio_buf_length; i++) {
						//sio_putchar(sio_buffer[i]);
					//}
				#endif
			}
			sio_buf_overflow = false;
		}
	}
}


/* === GENERAL WPAN FUNCTIONS === */

/**
 * @brief Callback function usr_mlme_reset_conf
 *
 * @param status Result of the reset procedure
 */
void usr_mlme_reset_conf(uint8_t status)
{
    if (MAC_SUCCESS == status)
    {
		if(debug_mode) {
			printf("reset done!\n");
		}
		if (COORDINATOR == sabre_node) {
			// Start a new network.
			/*
			* Set the short address of this node.
			* Use: bool wpan_mlme_set_req(uint8_t PIBAttribute,
			*                             void *PIBAttributeValue);
			*
			* This request leads to a set confirm message -> usr_mlme_set_conf
			*/
			uint8_t short_addr[2];

			if(debug_mode) {
				printf("Coordinator requesting short address setting... ");
			}
			short_addr[0] = (uint8_t)COORD_SHORT_ADDR;          // low byte
			short_addr[1] = (uint8_t)(COORD_SHORT_ADDR >> 8);   // high byte
			wpan_mlme_set_req(macShortAddress, short_addr);
		}
		else {
			if(debug_mode) {
				printf("Node requesting network scan... ");
			}
			// Search for a coordinator
			/*
			 * Initiate an active scan over all channels to determine
			 * which channel is used by the coordinator.
			 * Use: bool wpan_mlme_scan_req(uint8_t ScanType,
			 *                              uint32_t ScanChannels,
			 *                              uint8_t ScanDuration,
			 *                              uint8_t ChannelPage);
			 *
			 * This request leads to a scan confirm message -> usr_mlme_scan_conf
			 * Scan for about 50 ms on each channel -> ScanDuration = 1
			 * Scan for about 1/2 second on each channel -> ScanDuration = 5
			 * Scan for about 1 second on each channel -> ScanDuration = 6
			 */
			wpan_mlme_scan_req(MLME_SCAN_TYPE_ACTIVE,
							   SCAN_ALL_CHANNELS,
							   SCAN_DURATION,
							   DEFAULT_CHANNEL_PAGE);

			// Indicate network scanning by a LED flashing
			pal_timer_start(TIMER_LED_OFF,
							LED_NWK_PERIOD,
							TIMEOUT_RELATIVE,
							(FUNC_PTR)network_scan_indication_cb,
							NULL);
		}
    }
    else
    {
		if(debug_mode) {
			printf("\nRequesting network reset... ");
		}
        // something went wrong; restart
        wpan_mlme_reset_req(true);
    }
}


/* === DEVICE FUNCTIONS === */

/**
 * @brief Callback function usr_mlme_scan_conf
 *
 * @param status            Result of requested scan operation
 * @param ScanType          Type of scan performed
 * @param ChannelPage       Channel page on which the scan was performed
 * @param UnscannedChannels Bitmap of unscanned channels
 * @param ResultListSize    Number of elements in ResultList
 * @param ResultList        Pointer to array of scan results
 */
void usr_mlme_scan_conf(uint8_t status,
                        uint8_t ScanType,
                        uint8_t ChannelPage,
                        uint32_t UnscannedChannels,
                        uint8_t ResultListSize,
                        void *ResultList)
{
    number_of_scans++;

    if (MAC_SUCCESS == status)
    {
		if(debug_mode) {
			printf("scan sucessful!\n");
		}
        wpan_pandescriptor_t *coordinator;
        uint8_t i;

        /*
         * Analyze the ResultList.
         * Assume that the first entry of the result list is our coodinator.
         */
        coordinator = (wpan_pandescriptor_t *)ResultList;
        for (i = 0; i < ResultListSize; i++) {
            /*
             * Check if the PAN descriptor belongs to our coordinator.
             * Check if coordinator allows association.
             */
            if ((coordinator->LogicalChannel == DEFAULT_CHANNEL) &&
                (coordinator->ChannelPage == DEFAULT_CHANNEL_PAGE) &&
                (coordinator->CoordAddrSpec.PANId == DEFAULT_PAN_ID) &&
                ((coordinator->SuperframeSpec & ((uint16_t)1 <<\
					ASSOC_PERMIT_BIT_POS)) == ((uint16_t)1 << ASSOC_PERMIT_BIT_POS))
               ) {
			if(debug_mode) {
				printf("PAN ID: 0x%04x\nChannel: %i\nChannel page: %i\n",
					coordinator->CoordAddrSpec.PANId, coordinator->LogicalChannel,
					coordinator->ChannelPage);
			}

                // Store the coordinator's address
                if (coordinator->CoordAddrSpec.AddrMode == WPAN_ADDRMODE_SHORT) {
					if(debug_mode) {
						printf("Coordinator's address: 0x%04x\n\n", coord_addr.short_addr);
					}
                    ADDR_COPY_DST_SRC_16(coord_addr.short_addr,
						coordinator->CoordAddrSpec.Addr.short_address);
                }
                else if (coordinator->CoordAddrSpec.AddrMode == WPAN_ADDRMODE_LONG) {
					if(debug_mode) {
						printf("Addres: 0x%llx\n\n", coord_addr.ieee_addr);
					}
                    ADDR_COPY_DST_SRC_64(coord_addr.ieee_addr,
						coordinator->CoordAddrSpec.Addr.long_address);
                }
                else {
					if(debug_mode) {
						printf("\nRequesting network reset... ");
					}
                    // Something went wrong; restart
                    wpan_mlme_reset_req(true);
                    return;
                }

				if(debug_mode) {
					printf("Requesting association... ");
				}
                /*
                 * Associate to our coordinator
                 * Use: bool wpan_mlme_associate_req(uint8_t LogicalChannel,
                 *                                   uint8_t ChannelPage,
                 *                                   wpan_addr_spec_t *CoordAddrSpec,
                 *                                   uint8_t CapabilityInformation);
                 * This request will cause a mlme associate confirm message ->
                 * usr_mlme_associate_conf
                 */
                wpan_mlme_associate_req(coordinator->LogicalChannel,
                                        coordinator->ChannelPage,
                                        &(coordinator->CoordAddrSpec),
                                        WPAN_CAP_ALLOCADDRESS);
                return;
            }

            // Get the next PAN descriptor
            coordinator++;
        }

        /*
         * If here, the result list does not contain our expected coordinator.
         * Let's scan again.
         */
        if (number_of_scans < MAX_NUMBER_OF_SCANS)
        {
			if(debug_mode) {
				printf("Coordinator doesn't match!\nRequesting new network scan... ");
			}
            wpan_mlme_scan_req(MLME_SCAN_TYPE_ACTIVE,
                               SCAN_ALL_CHANNELS,
                               SCAN_DURATION,
                               DEFAULT_CHANNEL_PAGE);
        }
		/* No network could be found after maximum number of scan.
		 * Restart.
		 */
        else
        {
			if(debug_mode) {
				printf("\nRequesting network reset... ");
			}
			wpan_mlme_reset_req(true);
        }
    }
    else if (status == MAC_NO_BEACON)
    {
		if(debug_mode) {
			printf("No coordinator found!\nRequesting new longer network scan... ");
		}
        /*
         * No beacon is received; no coordiantor is located.
         * Scan again, but used longer scan duration.
         */
        if (number_of_scans < MAX_NUMBER_OF_SCANS)
        {
            wpan_mlme_scan_req(MLME_SCAN_TYPE_ACTIVE,
                               SCAN_ALL_CHANNELS,
                               SCAN_DURATION,
                               DEFAULT_CHANNEL_PAGE);
        }
		/* No network could be found after maximum number of scan.
		 * Restart.
		 */
        else
        {
			if(debug_mode) {
				printf("\nRequesting network reset... ");
			}
			wpan_mlme_reset_req(true);
        }
    }
	/* Something went wrong.
	 * Restart.
	 */
    else
    {
		if(debug_mode) {
			printf("\nRequesting network reset... ");
		}
        wpan_mlme_reset_req(true);
    }

    /* Keep compiler happy. */
    ScanType = ScanType;
    UnscannedChannels = UnscannedChannels;
    ChannelPage = ChannelPage;
}


/**
 * @brief Callback function indicating network search
 */
static void network_scan_indication_cb(void *parameter)
{
    pal_led(LED_NWK_SETUP, LED_TOGGLE);

    // Re-start led timer again
    pal_timer_start(TIMER_LED_OFF,
                    LED_NWK_PERIOD,
                    TIMEOUT_RELATIVE,
                    (FUNC_PTR)network_scan_indication_cb,
                    NULL);

    parameter = parameter; /* Keep compiler happy. */
}


/**
 * @brief Callback function for switching data LED off
 */
//static void data_led_off_cb(void *parameter)
//{
	//pal_timer_stop(TIMER_LED_OFF);
    ////pal_led(LED_DATA, LED_OFF);
//
    //parameter = parameter; /* Keep compiler happy. */
//}


/**
 * @brief Callback function usr_mlme_associate_conf
 *
 * @param AssocShortAddress    Short address allocated by the coordinator
 * @param status               Result of requested association operation
 */
void usr_mlme_associate_conf(uint16_t AssocShortAddress, uint8_t status)
{
    if (status == MAC_SUCCESS)
    {
		if(debug_mode) {
			printf("association confirmed! Node starting...\n\n");
		}
        /* Stop timer used for search indication (same as used for data transmission)
		   and keep NWK LED on */
        pal_timer_stop(TIMER_LED_OFF);
        pal_led(LED_NWK_SETUP, LED_ON);

		/* Send a first message to coordinator */
		//uint8_t src_addr_mode = WPAN_ADDRMODE_SHORT;
		//wpan_addr_spec_t dst_addr;
		//uint8_t payload[32];
		////static uint8_t msduHandle = 0;
		//dst_addr.AddrMode = WPAN_ADDRMODE_SHORT;
		//dst_addr.PANId = DEFAULT_PAN_ID;
		//ADDR_COPY_DST_SRC_16(dst_addr.Addr.short_address, coord_addr.short_addr);
//
		//if(debug_mode) {
			//switch(sabre_node) {
				//case LEFTHAND:
					//strcpy((char*)payload, "HELLO, LH entering network!");
					//break;
				//case RIGHTHAND:
					//strcpy((char*)payload, "HELLO, RH entering network!");
					//break;
				//case AIRMEMS:
					//strcpy((char*)payload, "HELLO, AM entering network!");
					//break;
				//default:
					//strcpy((char*)payload, "No allowed node here!");
					//break;
			//}
		//}
		//else {
			//uint8_t i;
			//for (i = 0; i < sabre_message_len; i++) {
				//if(0 == i) {
					//payload[i] = SABRE_START_BYTE;
				//}
				//else if(1 == i) {
					//payload[i] = sabre_node_address;
				//}
				//else if((sabre_message_len - 1) == i) {
					//payload[i] = SABRE_STOP_BYTE;
				//}
				//else {
					//payload[i] = 0xff;
				//}
			//}
		//}

#if (NODE_TYPE==SABRE_RH)
		/* start IMU */
		if(!sabre_imu_start(imu_version)) {
			pal_alert();
		}
#endif
		app_state = SABRE_RUNNING;
		
		//pal_led(LED_DATA, LED_ON);
		
		/*
		wpan_mcps_data_req(src_addr_mode,
		                   &dst_addr,
						   sizeof(payload),
						   payload,
						   msduHandle,
						   WPAN_TXOPT_ACK);
		*/
        // Start a timer that sends some data to the coordinator every 2 seconds.
/*        pal_timer_start(TIMER_TX_DATA,
                        LED_DATA_PERIOD,
                        TIMEOUT_RELATIVE,
                        (FUNC_PTR)app_timer_cb,
                        NULL);*/
    }
    else
    {
		if(debug_mode) {
			printf("\nRequesting network reset... ");
		}
        // Something went wrong; restart
        wpan_mlme_reset_req(true);
    }

    /* Keep compiler happy. */
    AssocShortAddress = AssocShortAddress;
}



/**
 * @brief Callback function usr_mlme_comm_status_ind
 *
 * @param SrcAddrSpec      Pointer to source address specification
 * @param DstAddrSpec      Pointer to destination address specification
 * @param status           Result for related response operation
 */
void usr_mlme_comm_status_ind(wpan_addr_spec_t *SrcAddrSpec,
                              wpan_addr_spec_t *DstAddrSpec,
                              uint8_t status)
{
    if (MAC_SUCCESS == status)
    {
        /*
         * Now the association of the device has been successful and its
         * information, like address, could  be stored.
         * But for the sake of simple handling it has been done
         * during assignment of the short address within the function
         * assign_new_short_addr()
         */
    }

    /* Keep compiler happy. */
    SrcAddrSpec = SrcAddrSpec;
    DstAddrSpec = DstAddrSpec;
}


/**
 * Callback function usr_mcps_data_conf
 *
 * @param msduHandle  Handle of MSDU handed over to MAC earlier
 * @param status      Result for requested data transmission request
 * @param Timestamp   The time, in symbols, at which the data were transmitted
 *                    (only if timestamping is enabled).
 *
 */
#ifdef ENABLE_TSTAMP
void usr_mcps_data_conf(uint8_t msduHandle, uint8_t status, uint32_t Timestamp)
#else
void usr_mcps_data_conf(uint8_t msduHandle, uint8_t status)
#endif  /* ENABLE_TSTAMP */
{
    if (status == MAC_SUCCESS)
    {
		if(debug_mode) {
			printf("Sent data acknowledged!\n");
		}
		ack_received = true;
        pal_led(LED_DATA, LED_OFF);
		//sabre_battery_monitor();
	}
	else {
		//pal_led(LED_DATA, LED_OFF);
	}

    /* Keep compiler happy. */
    msduHandle = msduHandle;
#ifdef ENABLE_TSTAMP
    Timestamp = Timestamp;
#endif  /* ENABLE_TSTAMP */
}




/* === COORDINATOR FUNCTIONS === */


/**
 * @brief Callback function usr_mlme_set_conf
 *
 * @param status        Result of requested PIB attribute set operation
 * @param PIBAttribute  Updated PIB attribute
 */
void usr_mlme_set_conf(uint8_t status, uint8_t PIBAttribute)
{
    if ((MAC_SUCCESS == status) && (macShortAddress == PIBAttribute))
    {
		if(debug_mode) {
			printf("set short address confirmed!\nRequesting association permission... ");
		}
        /*
         * Allow other devices to associate to this coordinator.
         * Use: bool wpan_mlme_set_req(uint8_t PIBAttribute,
         *                             void *PIBAttributeValue);
         *
         * This request leads to a set confirm message -> usr_mlme_set_conf
         */
         uint8_t association_permit = true;

         wpan_mlme_set_req(macAssociationPermit, &association_permit);
    }
    else if ((MAC_SUCCESS == status) && (macAssociationPermit == PIBAttribute))
    {
		if(debug_mode) {
			printf("permitting association confirmed!\nRequesting RX on when idle... ");
		}
        /*
         * Set RX on when idle to enable the receiver as default.
         * Use: bool wpan_mlme_set_req(uint8_t PIBAttribute,
         *                             void *PIBAttributeValue);
         *
         * This request leads to a set confirm message -> usr_mlme_set_conf
         */
         bool rx_on_when_idle = true;

         wpan_mlme_set_req(macRxOnWhenIdle, &rx_on_when_idle);
    }
    else if ((status == MAC_SUCCESS) && (PIBAttribute == macRxOnWhenIdle))
    {
		if(debug_mode) {
			printf("RX on when idle confirmed!\nRequesting network start... ");
		}
        /*
         * Start a nonbeacon-enabled network
         * Use: bool wpan_mlme_start_req(uint16_t PANId,
         *                               uint8_t LogicalChannel,
         *                               uint8_t ChannelPage,
         *                               uint8_t BeaconOrder,
         *                               uint8_t SuperframeOrder,
         *                               bool PANCoordinator,
         *                               bool BatteryLifeExtension,
         *                               bool CoordRealignment)
         *
         * This request leads to a start confirm message -> usr_mlme_start_conf
         */
         wpan_mlme_start_req(DEFAULT_PAN_ID,
                             DEFAULT_CHANNEL,
                             DEFAULT_CHANNEL_PAGE,
                             15, 15,
                             true, false, false);
    }
    else
    {
		if(debug_mode) {
			printf("\nRequesting network reset... ");
		}
        // something went wrong; restart
        wpan_mlme_reset_req(true);
    }
}


/**
 * @brief Callback function usr_mlme_start_conf
 *
 * @param status        Result of requested start operation
 */
void usr_mlme_start_conf(uint8_t status) {
    if (MAC_SUCCESS == status) {
		if(debug_mode) {
			printf("network start confirmed!\nPAN ID: 0x%04x\nChannel: %i\n"\
				"Channel page: %i\nOwn short address: 0x%04x\n\n",
				DEFAULT_PAN_ID, DEFAULT_CHANNEL,
				DEFAULT_CHANNEL_PAGE, COORD_SHORT_ADDR);
		}
        /*
         * Network is established.
         * Waiting for association indication from a device.
         * -> usr_mlme_associate_ind
         */
         // Stop timer used for search indication
         pal_timer_stop(TIMER_LED_OFF);
         pal_led(LED_NWK_SETUP, LED_ON);
		 
		 app_state = SABRE_COORDINATOR_WAITING;
    }
    else
    {
		if(debug_mode) {
			printf("\nRequesting network reset... ");
		}
        // something went wrong; restart
        wpan_mlme_reset_req(true);
    }
}


/**
 * @brief Callback function usr_mlme_associate_ind
 *
 * @param DeviceAddress         Extended address of device requesting association
 * @param CapabilityInformation Capabilities of device requesting association
 */
void usr_mlme_associate_ind(uint64_t DeviceAddress,
                            uint8_t CapabilityInformation)
{
	if(debug_mode) {
		printf("Association request received from device 0x%04x'%04x'%04x'%04x...",
		       (uint16_t)(DeviceAddress >> 48), (uint16_t)(DeviceAddress >> 32),
			   (uint16_t)(DeviceAddress >> 16), (uint16_t)(DeviceAddress));
	}
    /*
     * Any device is allowed to join the network
     * Use: bool wpan_mlme_associate_resp(uint64_t DeviceAddress,
     *                                    uint16_t AssocShortAddress,
     *                                    uint8_t status);
     *
     * This response leads to comm status indication -> usr_mlme_comm_status_ind
     * Get the next available short address for this device
     */
    uint16_t associate_short_addr = macShortAddress_def;

    if (assign_new_short_addr(DeviceAddress, &associate_short_addr) == true)
    {
		if(debug_mode) {
			printf("assigning short address: 0x%04x\n", associate_short_addr);
		}
        wpan_mlme_associate_resp(DeviceAddress,
                                 associate_short_addr,
                                 ASSOCIATION_SUCCESSFUL);
    }
    else
    {
		if(debug_mode) {
			printf("max capacity reached!\n");
		}
        wpan_mlme_associate_resp(DeviceAddress,
                                 associate_short_addr,
                                 PAN_AT_CAPACITY);
    }

    /* Keep compiler happy. */
    CapabilityInformation = CapabilityInformation;
}



/**
 * @brief Callback function usr_mcps_data_ind
 *
 * @param SrcAddrSpec      Pointer to source address specification
 * @param DstAddrSpec      Pointer to destination address specification
 * @param msduLength       Number of octets contained in MSDU
 * @param msdu             Pointer to MSDU
 * @param mpduLinkQuality  LQI measured during reception of the MPDU
 * @param DSN              DSN of the received data frame.
 * @param Timestamp        The time, in symbols, at which the data were received
 *                         (only if timestamping is enabled).
 */
void usr_mcps_data_ind(wpan_addr_spec_t *SrcAddrSpec,
                       wpan_addr_spec_t *DstAddrSpec,
                       uint8_t msduLength,
                       uint8_t *msdu,
                       uint8_t mpduLinkQuality,
#ifdef ENABLE_TSTAMP
                       uint8_t DSN,
                       uint32_t Timestamp)
#else
                       uint8_t DSN)
#endif  /* ENABLE_TSTAMP */
{
	//sabre_node_t source;
	char sourceChar[11];
	uint8_t i;
	uint32_t Deltatime;
	
    pal_led(LED_DATA, LED_ON);

	pal_trx_read_timestamp(&Timestamp);
	Deltatime = Timestamp - OldTimestamp;
	OldTimestamp = Timestamp;
	if(Deltatime >= ((uint32_t)1 << 16)) {
		Deltatime = 0xffff;
	}


	switch (msdu[1]) {
	//source = LEFTHAND;
	case SABRE_LH_ADDRESS:
		if(debug_mode) {
			strcpy(sourceChar, "Left hand");
		}
		for(i = 0; i < msduLength; i++) {
			if(i == (msduLength - 4)) {
				lh_trx_buffer[lh_buffer_cnt][i] = (uint8_t)(Deltatime & 0xff);
			}
			else if(i == (msduLength - 3)) {
				lh_trx_buffer[lh_buffer_cnt][i] = (uint8_t)((Deltatime >> 8) & 0xff);
			}
			else if(i == (msduLength - 2)) {
				lh_trx_buffer[lh_buffer_cnt][i] = mpduLinkQuality;
			}
			else {
				lh_trx_buffer[lh_buffer_cnt][i] = msdu[i];
			}
		}
		lh_buffer_cnt++;
		break;
		
	//source = RIGHTHAND;
	case SABRE_RH_ADDRESS:
		if(debug_mode) {
			strcpy(sourceChar, "Right hand");
		}
		for(i = 0; i < msduLength; i++) {
			if(i == (msduLength - 4)) {
				rh_trx_buffer[rh_buffer_cnt][i] = (uint8_t)(Deltatime & 0xff);
			}
			else if(i == (msduLength - 3)) {
				rh_trx_buffer[rh_buffer_cnt][i] = (uint8_t)((Deltatime >> 8) & 0xff);
			}
			else if(i == (msduLength - 2)) {
				rh_trx_buffer[rh_buffer_cnt][i] = mpduLinkQuality;
			}
			else {
				rh_trx_buffer[rh_buffer_cnt][i] = msdu[i];
			}
		}
		rh_buffer_cnt++;
		break;
		
	//source = AIRMEMS;
	case SABRE_AM_ADDRESS:
		if(debug_mode) {
			strcpy(sourceChar, "airMEMS");
		}
		for(i = 0; i < msduLength; i++) {
			if(i == (msduLength - 4)) {
				am_trx_buffer[am_buffer_cnt][i] = (uint8_t)(Deltatime & 0xff);
			}
			else if(i == (msduLength - 3)) {
				am_trx_buffer[am_buffer_cnt][i] = (uint8_t)(Deltatime >> 8);
			}
			else if(i == (msduLength -2)) {
				am_trx_buffer[am_buffer_cnt][i] = mpduLinkQuality;
			}
			else {
				am_trx_buffer[am_buffer_cnt][i] = msdu[i];
			}
		}
		am_buffer_cnt++;
		break;

	default:
		if(debug_mode) {
			strcpy(sourceChar, "unknown");
		}
		//source = _NDEF;
		break;
	}
	
	if(debug_mode) {
		printf("Data received...\nAddress: %d\nSource: %s\nLength: %i\n"\
			"Link quality: %i\nDelta time: %lu us\nValues:\n", 
			msdu[1],sourceChar, msduLength, mpduLinkQuality, Deltatime);
		for(i = 0; i < msduLength; i++) {
			printf("0x%02x", msdu[i]);
			(i % 8 == 7) ? (printf("\n")) : (printf(" "));
		}
		printf("\nBuffer counters: LH %i, RH %i, AM %i\n\n",
			lh_buffer_cnt, rh_buffer_cnt, am_buffer_cnt);
	}

	//pal_timer_start(TIMER_LED_OFF,
	                //LED_DATA_PERIOD,
					//TIMEOUT_RELATIVE,
					//(FUNC_PTR)data_led_off_cb,
					//NULL);
	pal_led(LED_DATA, LED_OFF);

    /* Keep compiler happy. */
    SrcAddrSpec = SrcAddrSpec;
    DstAddrSpec = DstAddrSpec;
    msduLength = msduLength;
    msdu = msdu;
    mpduLinkQuality = mpduLinkQuality;
    DSN = DSN;
#ifdef ENABLE_TSTAMP
    Timestamp = Timestamp;
#endif  /* ENABLE_TSTAMP */
}


/**
 * @brief Application specific function to assign a short address
 *
 */
static bool assign_new_short_addr(uint64_t addr64, uint16_t *addr16)
{
    uint8_t i;
	
	if(debug_mode) {
		printf("Current device list:\n");
		for(i = 0; i < MAX_NUMBER_OF_DEVICES; i++) {
			printf("device %02d -> 0x%04x    0x%04x'%04x'%04x'%04x\n",
			       i, device_list[i].short_addr,
				   (uint16_t)(device_list[i].ieee_addr >> 48),
				   (uint16_t)(device_list[i].ieee_addr >> 32),
				   (uint16_t)(device_list[i].ieee_addr >> 16),
				   (uint16_t)(device_list[i].ieee_addr));
		}
	}

    // Check if device has been associated before
    for (i = 0; i < MAX_NUMBER_OF_DEVICES; i++)
    {
		
        if (device_list[i].short_addr == 0x0000)
        {
            // If the short address is 0x0000, it has not been used before
            continue;
        }
        if (device_list[i].ieee_addr == addr64)
        {
			if(debug_mode) {
				printf("Old device!\n");
			}
			
            // Assign the previously assigned short address again
            *addr16 = device_list[i].short_addr;
            return true;
        }
    }

    for (i = 0; i < MAX_NUMBER_OF_DEVICES; i++)
    {
		if(debug_mode) {
			printf("New device!\n");
		}
		
        if (device_list[i].short_addr == 0x0000)
        {
            *addr16 = CPU_ENDIAN_TO_LE16(i + 0x0001);
             // get next short address
			device_list[i].short_addr = CPU_ENDIAN_TO_LE16(i + 0x0001);
			// store extended address
			device_list[i].ieee_addr = addr64;
            return true;
        }
    }

    // If we are here, no short address could be assigned.
    return false;
}











/**
 * @brief Callback function for the application timer
 *
 * @param AssocShortAddress    Short address allocated by the coordinator
 * @param status               Result of requested association operation
 */

//static void app_timer_cb(void *parameter)
//{
	//if(debug_mode) {
		//printf("Timer exited!\n");
	//}
    ///*
     //* Send some data and restart timer.
     //* Use: bool wpan_mcps_data_req(uint8_t SrcAddrMode,
     //*                              wpan_addr_spec_t *DstAddrSpec,
     //*                              uint8_t msduLength,
     //*                              uint8_t *msdu,
     //*                              uint8_t msduHandle,
     //*                              uint8_t TxOptions);
     //*
     //* This request will cause a mcps data confirm message ->
     //* usr_mcps_data_conf
     //*/
    //uint8_t src_addr_mode;
    //wpan_addr_spec_t dst_addr;
    //uint8_t payload[sabre_message_len];
	//uint8_t i;
    //static uint8_t msduHandle = 0;
//
    //src_addr_mode = WPAN_ADDRMODE_SHORT;
//
    //dst_addr.AddrMode = WPAN_ADDRMODE_SHORT;
    //dst_addr.PANId = DEFAULT_PAN_ID;
    //ADDR_COPY_DST_SRC_16(dst_addr.Addr.short_address, coord_addr.short_addr);
//
	//for(i = 0; i < sabre_message_len; i++) {
		//if(0 == i) {
			//payload[i] = SABRE_START_BYTE;
		//}
		//else if(1 == i) {
			//payload[i] = sabre_node_address;
		//}
		//else if( ((sabre_message_len - 4) == i) || ((sabre_message_len - 3) == i) || ((sabre_message_len - 2) == i) ) {
			//payload[i] = 0x00;
		//}
		//else if((sabre_message_len - 1) == i) {
			//payload[i] = SABRE_STOP_BYTE;
		//}
		//else {
			//payload[i] = (uint8_t)rand();  // any dummy data
		//}
	//}
    ////msduHandle++;               // increment handle
	//if(debug_mode) {
		//printf("Requesting data transmission... ");
	//}
    //wpan_mcps_data_req(src_addr_mode,
                       //&dst_addr,
                       //sabre_message_len,
                       //payload,
                       //msduHandle,
                       //WPAN_TXOPT_ACK);
//
    //pal_timer_start(TIMER_TX_DATA,
                    //DATA_TX_PERIOD,
                    //TIMEOUT_RELATIVE,
                    //(FUNC_PTR)app_timer_cb,
                    //NULL);
//
    //parameter = parameter;  /* Keep compiler happy. */
//}



/**
 * @brief Callback function switching off the LED
 */
//static void data_exchange_led_off_cb(void *parameter)
//{
    //pal_led(LED_DATA, LED_OFF);
//
    //parameter = parameter;  /* Keep compiler happy. */
//}


/* EOF */
