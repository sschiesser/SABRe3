/**
 * @file SABRe_coordinatorMAC.c
 *
 * @brief  MAC Example - coordinator nonbeacon mode
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

/* === INCLUDES ============================================================ */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "pal.h"
#include "tal.h"
#include "mac_api.h"
#include "sio_handler.h"
#include "app_config.h"
#include "ieee_const.h"
#include <util/delay.h>


/* === TYPES =============================================================== */

typedef struct associated_device_tag
{
    uint16_t short_addr;
    uint64_t ieee_addr;
}
/** This type definition of a structure can store the short address and the
 *  extended address of a device.
 */
associated_device_t;

/**
 * This enum store the current state of the coordinator.
 */
typedef enum coord_state_tag
{
    COORD_STARTING = 0,
    COORD_RUNNING
}coord_state_t;


/* === MACROS ============================================================== */
#ifdef CHANNEL
#define DEFAULT_CHANNEL                 (CHANNEL)
#ifdef HIGH_DATA_RATE_SUPPORT
#define DEFAULT_CHANNEL_PAGE            (17)
#else
#define DEFAULT_CHANNEL_PAGE			(0)
#endif
#else
/** Defines the default channel. */
#if (TAL_TYPE == AT86RF212)
    #ifdef CHINESE_BAND
        #define DEFAULT_CHANNEL                 (0)
        #define DEFAULT_CHANNEL_PAGE            (5)
    #else
        #define DEFAULT_CHANNEL                 (1)
        #define DEFAULT_CHANNEL_PAGE            (0)
    #endif  /* #ifdef CHINESE_BAND */
#else
#define DEFAULT_CHANNEL                 (20)
#ifdef HIGH_DATA_RATE_SUPPORT
#define DEFAULT_CHANNEL_PAGE            (17)
#else
#define DEFAULT_CHANNEL_PAGE			(0)
#endif
#endif  /* #if (TAL_TYPE == AT86RF212) */
#endif  /* #ifdef CHANNEL */
/** Defines the PAN ID of the network. */
#ifdef PAN_ID
#define DEFAULT_PAN_ID                  (PAN_ID)
#else
#define DEFAULT_PAN_ID                  CCPU_ENDIAN_TO_LE16(0xBABE)
#endif
/** Defines the short address of the coordinator. */
#define COORD_SHORT_ADDR                (0x0000)
/** Defines the maximum number of devices the coordinator will handle. */
#define MAX_NUMBER_OF_DEVICES           (100)
/** This is the time period in micro seconds for data transmissions. */
#define DATA_TX_PERIOD                  (2000000)
/** Defines the bit mask of channels that should be scanned. */
#if (TAL_TYPE == AT86RF212)
    #ifdef CHINESE_BAND
        #define SCAN_ALL_CHANNELS           (0x0000000F)
    #else
        #if (DEFAULT_CHANNEL == 0)
            #define SCAN_ALL_CHANNELS       (0x00000001)
        #else
            #define SCAN_ALL_CHANNELS       (0x000007FE)
        #endif
    #endif  /* #ifdef CHINESE_BAND */
#else
#define SCAN_ALL_CHANNELS               (0x07FFF800)
#endif

/** Defines the scan duration time. */
#define SCAN_DURATION                   (4)
/** Defines the maximum number of scans before starting own network. */
#define MAX_NUMBER_OF_SCANS             (3)

/* === BEACON-SPECIFIC CONSTANTS ========================================= */

/** Defines the scan duration time. */
#define SCAN_DURATION_COORDINATOR       (1)

/** Defines the default Beacon Order. */
#define DEFAULT_BO                      (4)
/** Defines the default Superframe Order. */
#define DEFAULT_SO                      (4)

/**
 * Defines the length of the beacon payload delivered to the devices.
 * This is the text "Atmel beacon demo" + one space + one uin8t_t variable.
 */
#define BEACON_PAYLOAD_LEN              (1)

/**
 * Defines the time in ms to initiate an update of the beacon payload.
 */
#define APP_BCN_PAYLOAD_DURATION_MS     (3000)

/**
 * Defines the time in ms to initiate a broadcast data transmission
 * to all devices.
 */
#define APP_BC_DATA_DURATION_MS         (9000)

/* ==================================================================== */

/** Defines the time to iniate a indirect data transmission to the device. */
#define APP_INDIRECT_DATA_DURATION_MS   (6000)
/** Defines the number of flashes to indicate device association. */
#define ASSOCIATE_DEVICE_FLASH			(5)

/** Defines the LED on duration time in us. */
#define LED_ON_DURATION_LONG			(500000)
#define LED_ON_DURATION_MEDIUM			(100000)
#define LED_ON_DURATION_SHORT			(50000)

#if (NO_OF_LEDS >= 3)
#define LED_START                       (LED_0)
#define LED_NWK_SETUP                   (LED_1)
#define LED_DATA                        (LED_2)
#elif (NO_OF_LEDS == 2)
#define LED_START                       (LED_0)
#define LED_NWK_SETUP                   (LED_0)
#define LED_DATA                        (LED_1)
#else
#define LED_START                       (LED_0)
#define LED_NWK_SETUP                   (LED_0)
#define LED_DATA                        (LED_0)
#endif

/* === GLOBALS ============================================================= */

/** This structure stores the short and extended address of the coordinator. */
//static associated_device_t coord_addr;
/** Number of done network scans */
//static uint8_t number_of_scans;
/** This array stores all device related information. */
static associated_device_t device_list[MAX_NUMBER_OF_DEVICES];
/** This array stores the current beacon payload. */
static uint8_t beacon_payload[BEACON_PAYLOAD_LEN] = {"S"};
/** This variable stores the current state of the node. */
static coord_state_t coord_state = COORD_STARTING;
/** This variable counts the number of transmitted data frames. */
//static uint32_t tx_cnt;
static uint8_t dummy_message[42];
static uint8_t dummy_3bit_values[1]; // switches
static uint8_t dummy_4bit_values[1]; // battery level
static uint8_t dummy_8bit_values[3]; // rf link quality (LQI)
static uint16_t dummy_10bit_values[13]; // ADC results
//static int16_t dummy_11bit_values[3]; // 
//static int16_t dummy_12bit_values[3]; // 
static int16_t dummy_16bit_values[13]; // gyroscope (x, y, z, t), compass (x, y, z), accelerometer (x, y, z) values & timestamp (3)
static int32_t dummy_32bit_values[2]; // airMEMS pressure & temperature values

static bool dummy_3bit_inc = true;
//static bool dummy_4bit_inc = true;
static bool dummy_8bit_inc = true;
static bool dummy_10bit_inc = true;
//static bool dummy_11bit_inc = true;
//static bool dummy_12bit_inc = true;
static bool dummy_16bit_inc = true;
static bool dummy_32bit_inc = true;

#define U3_MIN						0x00
#define U3_MAX						0x07
#define U4_MIN						0x00
#define U4_MAX						0x0F
#define U8_MIN						0x00
#define U8_MAX						0xFF
#define U10_MIN						0x0000
#define U10_MAX						0x03FF
#define S11_MIN						0x83FF
#define S11_MAX						0x03FF
#define S12_MIN						0x87FF
#define S12_MAX						0x07FF
#define S16_MIN						0xFFFF
#define S16_MAX						0x7FFF
#define S32_MIN						0x7FFFFFFFL
#define S32_MAX						0xFFFFFFFFL


/* === PROTOTYPES ========================================================== */

//static void app_timer_cb(void *parameter);
static void associate_device_cb(void *parameter);
//static void network_scan_indication_cb(void *parameter);
//static void data_exchange_led_off_cb(void *parameter);
static bool assign_new_short_addr(uint64_t addr64, uint16_t *addr16);
//static void bc_data_cb(void *parameter);
//static void bcn_payload_update_cb(void *parameter);

/* === IMPLEMENTATION ====================================================== */

void dummy_init_values(void) {
	uint8_t i;
	dummy_3bit_values[0] = U3_MIN;
	dummy_4bit_values[0] = U4_MIN;
	for(i = 0; i < 3; i++) {
		dummy_8bit_values[i] = U8_MIN;
	}
	for(i = 0; i < 13; i++) {
		dummy_10bit_values[i] = U10_MIN;
	}
	//for(i = 0; i < 3; i++) {
		//dummy_11bit_values[i] = S11_MIN;
		//dummy_12bit_values[i] = S12_MIN;
	//}
	for(i = 0; i < 13; i++) {
		dummy_16bit_values[i] = S16_MIN;
	}
	for(i = 0; i < 2; i++) {
		dummy_32bit_values[i] = S32_MIN;
	}
}

void dummy_generate_values(void) {
#if (DEBUG > 0)
#ifdef SIO_HUB
	printf("Generating new dummy values...\n");
#endif
#endif
	static uint8_t i;
	/* 3bit -- switches */
	if(dummy_3bit_values[0] == U3_MAX) {
		dummy_3bit_values[0]--;
		dummy_3bit_inc = false;
	}
	else if(dummy_3bit_values[0] == U3_MIN) {
		dummy_3bit_values[0]++;
		dummy_3bit_inc = true;
	}
	else {
		dummy_3bit_values[0] = (dummy_3bit_inc) ? (dummy_3bit_values[0]+1) : (dummy_3bit_values[0]-1);
	}
	///* 4bit -- battery */
	//if(dummy_4bit_values[0] == U4_MAX) {
		//dummy_4bit_values[0]--;
		//dummy_4bit_inc = false;
	//}
	//else if(dummy_4bit_values[0] == U4_MIN) {
		//dummy_4bit_values[0]++;
		//dummy_4bit_inc = true;
	//}
	//else {
		//dummy_4bit_values[0] = (dummy_4bit_inc) ? (dummy_4bit_values[0]+1) : (dummy_4bit_values[0]-1);
	//}
	/* 8bit -- LQI & battery(2) */
	for(i = 0; i < 3; i++) {
		if(dummy_8bit_values[i] == U8_MAX) {
			dummy_8bit_values[i]--;
			dummy_8bit_inc = false;
		}
		else if(dummy_8bit_values[i] == U8_MIN) {
			dummy_8bit_values[i]++;
			dummy_8bit_inc = true;
		}
		else {
			dummy_8bit_values[i] = (dummy_8bit_inc) ? (dummy_8bit_values[i]+1) : (dummy_8bit_values[i]-1);
		}
	}
	/* 10bit -- keys(12/13) */
	for(i = 0; i < 13; i++) {
		if(dummy_10bit_values[i] == U10_MAX) {
			dummy_10bit_values[i]--;
			dummy_10bit_inc = false;
		}
		else if(dummy_10bit_values[i] == U10_MIN) {
			dummy_10bit_values[i]++;
			dummy_10bit_inc = true;
		}
		else {
			dummy_10bit_values[i] = (dummy_10bit_inc) ? (dummy_10bit_values[i]+1) : (dummy_10bit_values[i]-1);
		}
	}
	///* 11bit -- accelerometer(3) */
	//for(i = 0; i < 3; i++) {
		//if(dummy_11bit_values[i] == S11_MAX) {
			//dummy_11bit_values[i]--;
			//dummy_11bit_inc = false;
		//}
		//else if(dummy_11bit_values[i] == S11_MIN) {
			//dummy_11bit_values[i]++;
			//dummy_11bit_inc = true;
		//}
		//else {
			//dummy_11bit_values[i] = (dummy_11bit_inc) ? (dummy_11bit_values[i]+1) : (dummy_11bit_values[i]-1);
		//}
	//}
	///* 12bit -- compass(3) */
	//for(i = 0; i < 3; i++) {
		//if(dummy_12bit_values[i] == S12_MAX) {
			//dummy_12bit_values[i]--;
			//dummy_12bit_inc = false;
		//}
		//else if(dummy_12bit_values[i] == S12_MIN) {
			//dummy_12bit_values[i]++;
			//dummy_12bit_inc = true;
		//}
		//else {
			//dummy_12bit_values[i] = (dummy_12bit_inc) ? (dummy_12bit_values[i]+1) : (dummy_12bit_values[i]-1);
		//}
	//}
	/* 16bit -- accelerometer (3), compass (3), gyro(4) & timestamp (3) */
	for(i = 0; i < 13; i++) {
		if(dummy_16bit_values[i] == S16_MAX) {
			dummy_16bit_values[i]--;
			dummy_16bit_inc = false;
		}
		else if(dummy_16bit_values[i] == S16_MIN) {
			dummy_16bit_values[i]++;
			dummy_16bit_inc = true;
		}
		else {
			dummy_16bit_values[i] = (dummy_16bit_inc) ? (dummy_16bit_values[i]+1) : (dummy_16bit_values[i]-1);
		}
	}
	/* 32bit -- pressure & temperature */
	for(i = 0; i < 2; i++) {
		if(dummy_32bit_values[i] == S32_MAX) {
			dummy_32bit_values[i]--;
			dummy_32bit_inc = false;
		}
		else if(dummy_32bit_values[i] == S32_MIN) {
			dummy_32bit_values[i]++;
			dummy_32bit_inc = true;
		}
		else {
			dummy_32bit_values[i] = (dummy_32bit_inc) ? (dummy_32bit_values[i]+1) : (dummy_32bit_values[i]-1);
		}
	}

//#if (DEBUG > 0)
//#ifdef SIO_HUB
	//printf("Generated message for node %x\nkeys: ", node);
	//for(i = 0; i < 13; i++) {
		//printf("0x%04x ", dummy_10bit_values[i]);
	//}
	/*printf("\nswitches: 0x%x\ntimestamp: %i (0x%04x)\nlqi: %i (0x%02x)\n",
			dummy_3bit_values[0], dummy_16bit_values[0], dummy_16bit_values[0],
			dummy_8bit_values[0], dummy_8bit_values[0]);*/
//#endif
//#endif

//#if (DEBUG > 0)
//#ifdef SIO_HUB
	//printf("generated message for node %i\nkeys: ", node);
	//for(i = 0; i < 12; i++) {
		//printf("0x%04x ", dummy_10bit_values[i]);
	//}
	//printf("\naccelerometer: ");
	//for(i = 0; i < 3; i++) {
		//printf("0x%d - ", dummy_11bit_values[i]);
	//}
	//printf("\ngyroscope: ");
	//for(i = 0; i < 4; i++) {
		//printf("0x%d - ", dummy_16bit_values[i]);
	//}
	//printf("\ncompass: ");
	//for(i = 0; i < 3; i++) {
		//printf("0x%d - ", dummy_12bit_values[i]);
	//}
	/*printf("\nbattery: 0x%02x\ntimestamp: %i (0x%04x)\nlqi: %i (%02x)\n",
			dummy_4bit_values[0], dummy_16bit_values[4], dummy_16bit_values[4],
			dummy_8bit_values[0], dummy_8bit_values[0]);*/
//#endif
//#endif
//
//#if (DEBUG > 0)
//#ifdef SIO_HUB
	/*printf("generated message for node %i\npressure: %ld (0x%08lx)\ntemperature: %ld (0x%08lx)"\
			"\nbattery: 0x%02x\ntimestamp: %i (0x%04x)\nlqi: %i (0x%02x)\n",
			node, dummy_32bit_values[0], dummy_32bit_values[0], dummy_32bit_values[1],
			dummy_32bit_values[1], dummy_4bit_values[0], dummy_16bit_values[0],
			dummy_16bit_values[0], dummy_8bit_values[0], dummy_8bit_values[0]); */
//#endif
//#endif
}

void dummy_pack_values(uint8_t node) {
#if (DEBUG > 0)
#ifdef SIO_HUB
	printf("Packing values...\n");
#endif
#endif
	uint8_t i;
	for(i = 0; i < 39; i++) {
		dummy_message[i] = 0;
	}
	dummy_message[0] = 0x41;
	switch (node) {
	/************************************************************************/
	/* LEFT HAND PACKAGING                                                  */
	/************************************************************************/
	case 0:
		dummy_message[1] = 240;
		for(i = 0; i < 13; i++) {
			dummy_message[i+2] = (uint8_t)(dummy_10bit_values[i] & 0xff);
		}
		dummy_message[15] = (uint8_t)( (((dummy_10bit_values[0] >> 8) & 0x03) << 6) |\
									   (((dummy_10bit_values[1] >> 8) & 0x03) << 4) |\
									   (((dummy_10bit_values[2] >> 8) & 0x03) << 2) |\
									   (((dummy_10bit_values[3] >> 8) & 0x03)) );
		dummy_message[16] = (uint8_t)( (((dummy_10bit_values[4] >> 8) & 0x03) << 6) |\
									   (((dummy_10bit_values[5] >> 8) & 0x03) << 4) |\
									   (((dummy_10bit_values[6] >> 8) & 0x03) << 2) |\
									   (((dummy_10bit_values[7] >> 8) & 0x03)) );
		dummy_message[17] = (uint8_t)( (((dummy_10bit_values[8] >> 8) & 0x03) << 6) |
									   (((dummy_10bit_values[9] >> 8) & 0x03) << 4) |\
									   (((dummy_10bit_values[10] >> 8) & 0x03) << 2) |\
									   (((dummy_10bit_values[11] >> 8) & 0x03)) );
		dummy_message[18] = (uint8_t)( (((dummy_10bit_values[12] >> 8) & 0x03) << 6) |\
									   (dummy_3bit_values[0] << 3) );
		dummy_message[19] = (uint8_t)(dummy_16bit_values[10] & 0xff);
		dummy_message[20] = (uint8_t)((dummy_16bit_values[10] >> 8) & 0xff);
		dummy_message[21] = (uint8_t)(dummy_8bit_values[0]);
		dummy_message[22] = 0x5A;
		break;
	/************************************************************************/
	/* RIGHT HAND PACKAGING                                                 */
	/************************************************************************/
	case 1:
		dummy_message[1] = 241;
		// Keys low bytes
		for(i = 0; i < 12; i++) {
			dummy_message[i+2] = (uint8_t)(dummy_10bit_values[i] & 0xff);
		}
		// Keys high bytes
		dummy_message[14] = (uint8_t)( (((dummy_10bit_values[0] >> 8) & 0x03) << 6) |\
									   (((dummy_10bit_values[1] >> 8) & 0x03) << 4) |\
									   (((dummy_10bit_values[2] >> 8) & 0x03) << 2) |\
									   (((dummy_10bit_values[3] >> 8) & 0x03)) );
		dummy_message[15] = (uint8_t)( (((dummy_10bit_values[4] >> 8) & 0x03) << 6) |\
									   (((dummy_10bit_values[5] >> 8) & 0x03) << 4) |\
									   (((dummy_10bit_values[6] >> 8) & 0x03) << 2) |\
									   (((dummy_10bit_values[7] >> 8) & 0x03)) );
		dummy_message[16] = (uint8_t)( (((dummy_10bit_values[8] >> 8) & 0x03) << 6) |\
									   (((dummy_10bit_values[9] >> 8) & 0x03) << 4) |\
									   (((dummy_10bit_values[10] >> 8) & 0x03) << 2) |\
									   (((dummy_10bit_values[11] >> 8) & 0x03)) );
		// Gyroscope low & high bytes
		for(i = 0; i < 4; i++) {
			dummy_message[i+17] = (uint8_t)(dummy_16bit_values[i] & 0xff);
			dummy_message[i+21] = (uint8_t)((dummy_16bit_values[i] >> 8) & 0xff);
		}
		// Compass low & high bytes
		for(i = 0; i < 3; i++) {
			dummy_message[i+25] = (uint8_t)(dummy_16bit_values[i+4] & 0xff);
			dummy_message[i+28] = (uint8_t)((dummy_16bit_values[i+4] >> 8) & 0xff);
		}
		// Accelerometer low & high bytes
		for(i = 0; i < 3; i++) {
			dummy_message[i+31] = (uint8_t)(dummy_16bit_values[i+7] & 0xff);
			dummy_message[i+34] = (uint8_t)((dummy_16bit_values[i+7] >> 8) & 0xff);
		}
		// Battery level
		dummy_message[37] = (uint8_t)(dummy_8bit_values[0]);
		// Timestamp & LQI
		dummy_message[38] = (uint8_t)(dummy_16bit_values[11] & 0xff);
		dummy_message[39] = (uint8_t)((dummy_16bit_values[11] >> 8) & 0xff);
		dummy_message[40] = (uint8_t)(dummy_8bit_values[0]);
		dummy_message[41] = 0x5A;
		break;
	/************************************************************************/
	/* airMEMS PACKAGING                                                    */
	/************************************************************************/
	case 2:
		dummy_message[1] = 242;
		for(i = 0; i < 4; i++) {
			dummy_message[i+2] = (uint8_t)(((dummy_32bit_values[0] >> (8*i)) & 0xff));
			dummy_message[i+6] = (uint8_t)(((dummy_32bit_values[1] >> (8*i)) & 0xff));
		}
		dummy_message[10] = (uint8_t)(dummy_4bit_values[0] & 0xf);
		dummy_message[11] = (uint8_t)(dummy_16bit_values[12] & 0xff);
		dummy_message[12] = (uint8_t)((dummy_16bit_values[12] >> 8) & 0xff);
		dummy_message[13] = (uint8_t)(dummy_8bit_values[1]);
		dummy_message[14] = 90;
		break;
	default:
		break;
	}
#if (DEBUG > 0)
#ifdef SIO_HUB
	uint8_t cntmax = 0;
	switch(node) {
	case 0:
		cntmax = 23;
		break;
	case 1:
		cntmax = 39;
		break;
	case 2:
		cntmax = 15;
		break;
	default:
		break;
	}
	printf("Message %i packed...\n", node);
	for(i = 0; i < cntmax; i++) {
		printf("%02x ", dummy_message[i]);
	}
	printf("\n");
#endif
#endif
}

void dummy_send_data(uint8_t node) {
#if (DEBUG > 0)
#ifdef SIO_HUB
	printf("Sending values...\n");
#endif
#endif

	static uint8_t i;
	static uint8_t cntmax = 0;
	switch(node) {
	case 0:
		cntmax = 23;
		break;
	case 1:
		cntmax = 42;
		break;
	case 2:
		cntmax = 15;
		break;
	default:
		cntmax = 0xff;
		break;
	}
#if (DEBUG > 0)
#ifdef SIO_HUB
	if(cntmax != 0xff) {
		for(i = 0; i < cntmax; i++) {
			printf("%x ", dummy_message[i]);
			//putchar(dummy_message[i]);
		}
		printf("\n\n\n");
	}
#endif
#else
	if(cntmax != 0xff) {
		for(i = 0; i < cntmax; i++) {
			putchar(dummy_message[i]);
		}
	}
#endif
}


/**
 * @brief Main function of the device application
 */
int main(void)
{
    /* Initialize the MAC layer and its underlying layers, like PAL, TAL, BMM. */
    if (wpan_init() != MAC_SUCCESS)
    {
        /*
         * Stay here; we need a valid IEEE address.
         * Check kit documentation how to create an IEEE address
         * and to store it into the EEPROM.
         */
        pal_alert();
    }

    /* Initialize LEDs. */
    pal_led_init();
    pal_led(LED_START, LED_OFF);         // indicating application is started
    pal_led(LED_NWK_SETUP, LED_OFF);    // indicating node is associated
    pal_led(LED_DATA, LED_OFF);         // indicating successful data transmission

    /*
     * The stack is initialized above, hence the global interrupts are enabled
     * here.
     */
    pal_global_irq_enable();

#ifdef SIO_HUB
    /* Initialize the serial interface used for communication with terminal program. */
    if (pal_sio_init(SIO_CHANNEL) != MAC_SUCCESS)
    {
        /* Something went wrong during initialization. */
        pal_alert();
    }

#if ((!defined __ICCAVR__) && (!defined __ICCARM__) && (!defined __GNUARM__) && \
     (!defined __ICCAVR32__) && (!defined __GNUAVR32__))
    fdevopen(_sio_putchar, _sio_getchar);
#endif
	
#if (DEBUG > 0)
    /* To make sure the hyper terminal gets connected to the system */
    //sio_getchar();

	printf("****************************\n");
    printf("* SABRe application        *\n");
	printf("****************************\n");
	printf("* - coordinator            *\n");
#ifdef BEACON_SUPPORT
	printf("* - beacon                 *\n");
#else
	printf("* - no beacon              *\n");
#endif /* #ifdef BEACON_SUPPORT */
#ifdef HIGH_DATA_RATE_SUPPORT
	printf("* - high data rate support *\n");
#else
	printf("* - low data rate          *\n");
#endif
	printf("****************************\n\n");
#endif /* #if (DEBUG > 0) */

#else
	/* If SIO_HUB is not defined, the coordinator cannot work */
	pal_alert();
#endif /* SIO_HUB */

	pal_led(LED_NWK_SETUP, LED_ON);
    /*
     * Reset the MAC layer to the default values
     * This request will cause a mlme reset confirm message ->
     * usr_mlme_reset_conf
     */
    //wpan_mlme_reset_req(true);

	uint8_t cnt = 0;
	
	dummy_init_values();
	
    /* Main loop */
    while (1)
    {
		if(0 == cnt) dummy_generate_values();
		dummy_pack_values(cnt);
		dummy_send_data(cnt);
	
		(cnt < 2) ? cnt++ : (cnt = 0);
		
#if (DEBUG > 0)
	_delay_ms(1000);
#else
	static uint8_t delay_val;
	delay_val = (rand() % 3) + 1;
	//delay_val = 3;
	while(delay_val--) {
		_delay_ms(1);
	}
#endif
		
    }
}



/* === RESET MAC ========================================================== */

/**
 * @brief Callback function usr_mlme_reset_conf
 *
 * @param status Result of the reset procedure
 */
void usr_mlme_reset_conf(uint8_t status)
{
#if (DEBUG > 0)
#ifdef SIO_HUB
        printf("Setting newtork on page %d.\n", DEFAULT_CHANNEL_PAGE);
#endif
#endif
    if (status == MAC_SUCCESS)
    {
		/*
		 * Start coordinator immediately
         * Use: bool wpan_mlme_set_req(uint8_t PIBAttribute,
         *                             void *PIBAttributeValue);
         *
         * This request leads to a set confirm message -> usr_mlme_set_conf
		 */
		 uint8_t short_addr[2];

        short_addr[0] = (uint8_t)COORD_SHORT_ADDR;          // low byte
        short_addr[1] = (uint8_t)(COORD_SHORT_ADDR >> 8);   // high byte
	    wpan_mlme_set_req(macShortAddress, short_addr);
    }
    else
    {
        // something went wrong; restart
        wpan_mlme_reset_req(true);
    }
}



/* === START NEW NETWORK ========================================== */

/**
 * @brief Callback function usr_mlme_set_conf
 *
 * @param status        Result of requested PIB attribute set operation
 * @param PIBAttribute  Updated PIB attribute
 */
void usr_mlme_set_conf(uint8_t status, uint8_t PIBAttribute)
{
    if ((status == MAC_SUCCESS) && (PIBAttribute == macShortAddress))
    {
#if (DEBUG > 0)
#ifdef SIO_HUB
		printf("Coordinator set to short address mode\n");
#endif
#endif
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
    else if ((status == MAC_SUCCESS) && (PIBAttribute == macAssociationPermit))
    {
#if (DEBUG > 0)
#ifdef SIO_HUB
		printf("Device association allowed.\n");
#endif
#endif
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
#if (DEBUG > 0)
#ifdef SIO_HUB
		printf("RX on in idle mode.\n");
#endif
#endif
        /* Set the beacon payload length. */
        uint8_t beacon_payload_len = BEACON_PAYLOAD_LEN;
        wpan_mlme_set_req(macBeaconPayloadLength, &beacon_payload_len);
    }
    else if ((status == MAC_SUCCESS) && (PIBAttribute == macBeaconPayloadLength))
    {
#if (DEBUG > 0)
#ifdef SIO_HUB
		printf("Beacon payload length set.\n");
#endif
#endif
        /*
         * Once the length of the beacon payload has been defined,
         * set the actual beacon payload.
         */
         wpan_mlme_set_req(macBeaconPayload, &beacon_payload);
    }
    else if ((status == MAC_SUCCESS) && (PIBAttribute == macBeaconPayload))
    {
#if (DEBUG > 0)
#ifdef SIO_HUB
		printf("Beacon payload set.\n");
#endif
#endif

        if (COORD_STARTING == coord_state)
        {
#if (DEBUG > 0)
#ifdef SIO_HUB
			printf("Coordinator starting...\n- scanning channels.\n");
#endif
#endif
            /*
             * Initiate an active scan over all channels to determine
             * which channel to use.
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
                               SCAN_DURATION_COORDINATOR,
                               DEFAULT_CHANNEL_PAGE);
        }
        else
        {
#if (DEBUG > 0)
#ifdef SIO_HUB
			printf("Coordinator running...\n");
#endif
#endif
            /* Do nothing once the node is properly running. */
        }
	}		
    else
    {
        // something went wrong; restart
        wpan_mlme_reset_req(true);
    }
}


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
#if (DEBUG > 0)
#ifdef SIO_HUB
	printf("Channel scan done:\n- starting own beacon-network\n- PAN ID = 0x%04x\n", DEFAULT_PAN_ID);
#ifdef HIGH_DATA_RATE_SUPPORT
	printf("- High data rate support\n");
#endif /* #ifdef SIO_HUB */
#endif /* #ifdef HIGH_DATA_RATE SUPPORT */
#endif /* #if (DEBUG > 0) */

    /*
     * We are not interested in the actual scan result,
     * because we start our network on the pre-defined channel anyway.
     * Start a beacon-enabled network
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
                         DEFAULT_BO,
                         DEFAULT_SO,
                         true, false, false);

    /* Keep compiler happy. */
    status = status;
    ScanType = ScanType;
    ChannelPage = ChannelPage;
    UnscannedChannels = UnscannedChannels;
    ResultListSize = ResultListSize;
    ResultList = ResultList;
}


/**
 * @brief Callback function usr_mlme_start_conf
 *
 * @param status        Result of requested start operation
 */
void usr_mlme_start_conf(uint8_t status)
{
    if (status == MAC_SUCCESS)
    {
        coord_state = COORD_RUNNING;
#if (DEBUG > 0)
#ifdef SIO_HUB
        printf("\n!!NETWORK STARTED!!\n\n");
#endif /* #ifdef SIO_HUB */
#endif /* #if (DEBUG > 0) */
        /*
         * Network is established.
         * Waiting for association indication from a device.
         * -> usr_mlme_associate_ind
         */
         // Stop timer used for search indication
         pal_timer_stop(APP_TIMER_LED_OFF);
         pal_led(LED_NWK_SETUP, LED_ON);
		 
        /*
         * Now that the network has been started successfully,
         * the timer for broadcast data transmission is started.
         * This is independent from the actual number of associated nodes.
         */

        /* Start timer to initiate broadcast data transmission. */
        //pal_timer_start(APP_TIMER_BC_DATA,
                        //((uint32_t)APP_BC_DATA_DURATION_MS * 1000),
                        //TIMEOUT_RELATIVE,
                        //(FUNC_PTR)bc_data_cb,
                        //NULL);

        ///*
         //* Now that the network has been started successfully,
         //* the timer for updating the beacon payload is started.
         //*/
        //pal_timer_start(APP_TIMER_BCN_PAYLOAD_UPDATE,
                        //((uint32_t)APP_BCN_PAYLOAD_DURATION_MS * 1000),
                        //TIMEOUT_RELATIVE,
                        //(FUNC_PTR)bcn_payload_update_cb,
                        //NULL);

	}
    else
    {
        // something went wrong; restart
        wpan_mlme_reset_req(true);
    }
}

/**
 * @brief Callback function for initiation of broadcast data transmission
 *
 * @param parameter Pointer to callback parameter
 *                  (not used in this application, but could be used
 *                  to indicated LED to be switched off)
 */
//static void bc_data_cb(void *parameter)
//{
//#if (DEBUG > 0)
//#ifdef SIO_HUB
	//printf("Broadcast data to all devices.\n");
//#endif
//#endif
    ///* Store the current MSDU handle to be used for a broadcast data frame. */
    //static uint8_t curr_msdu_handle;
    //uint8_t src_addr_mode;
    //wpan_addr_spec_t dst_addr;
    //uint8_t payload;
//#if (DEBUG > 0)
//#ifdef SIO_HUB
    //char sio_array[255];
//#endif
//#endif
//
//
    ///*
     //* Request transmission of broadcast data to all devices.
     //*
     //* Since this is a beacon-enabled network,
     //* this request will just queue this frame into the broadcast data queue.
     //*
     //* Once this the next beacon frame is about to be transmitted,
     //* the broadcast data frame will be announced by setting
     //* the frame pending bit of the frame control field of this particular
     //* beacon frame.
     //*
     //* Immediately after the successful transmission of the beacon frame,
     //* the pending broadcast frame will be transmitted.
     //*/
    //src_addr_mode = WPAN_ADDRMODE_SHORT;
    //dst_addr.AddrMode = WPAN_ADDRMODE_SHORT;
    //dst_addr.PANId = DEFAULT_PAN_ID;
    ///* Broadcast destination address is used. */
    //dst_addr.Addr.short_address = BROADCAST;
//
    //payload = (uint8_t)rand();  // Any dummy data
    //curr_msdu_handle++;         // Increment handle
    //tx_cnt++;
//
//#if (DEBUG > 0)
//#ifdef SIO_HUB
    //sprintf(sio_array, "Broadcast frame Tx count:  %" PRIu32 "\n", tx_cnt);
    //printf(sio_array);
//#endif
//#endif
//
    ///* The transmission is direct, but without acknowledgement. */
    //if (wpan_mcps_data_req(src_addr_mode,
                            //&dst_addr,
                            //1,  // One octet
                            //&payload,
                            //curr_msdu_handle,
                            //WPAN_TXOPT_OFF)
       //)
    //{
        //pal_led(LED_DATA, LED_ON);
    //}
    //else
    //{
        ///*
         //* Data could not be queued into the broadcast queue.
         //* Add error handling if required.
         //*/
    //}
//
    ///* Start timer to initiate next broadcast data transmission. */
    //pal_timer_start(APP_TIMER_BC_DATA,
                    //((uint32_t)APP_BC_DATA_DURATION_MS * 1000),
                    //TIMEOUT_RELATIVE,
                    //(FUNC_PTR)bc_data_cb,
                    //NULL);
//
    //parameter = parameter;  /* Keep compiler happy. */
//}

/**
 * @brief Callback function for updating the beacon payload
 *
 * @param parameter Pointer to callback parameter
 *                  (not used in this application, but could be used
 *                  to indicated LED to be switched off)
 */
//static void bcn_payload_update_cb(void *parameter)
//{
//#if (DEBUG > 0)
//#ifdef SIO_HUB
	//printf("Update payload value.\n");
//#endif
//#endif
    ///*
     //* Counter holding the variable portion of the beacon payload.
     //*
     //* Note: If this is changed, also the define BEACON_PAYLOAD_LEN needs
     //* to be updated accordingly.
     //* If this happens, the PIB attribute macBeaconPayloadLength needs to be
     //* adjusted again as well. Since in this application the length of the
     //* beacon payload never changes, this can be skipped.
     //*/
    //static uint8_t bcn_payload_cnt;
//
    ///* The counter transmitted in the beacon payload is updated and
     //* the new beacon payload is set.
     //*/
    //bcn_payload_cnt++;
    //bcn_payload_cnt %= 10;
    ///* Create printable character. */
    //beacon_payload[BEACON_PAYLOAD_LEN - 1] = bcn_payload_cnt + 0x30;
    //wpan_mlme_set_req(macBeaconPayload, &beacon_payload);
//
//
    ///* Restart timer for updating beacon payload. */
    //pal_timer_start(APP_TIMER_BCN_PAYLOAD_UPDATE,
                    //((uint32_t)APP_BCN_PAYLOAD_DURATION_MS * 1000),
                    //TIMEOUT_RELATIVE,
                    //(FUNC_PTR)bcn_payload_update_cb,
                    //NULL);
//
    //parameter = parameter;  /* Keep compiler happy. */
//}


/* === ASSOCIATE DEVICE TO NETWORK ============================================ */

/**
 * @brief Callback function usr_mlme_associate_ind
 *
 * @param DeviceAddress         Extended address of device requesting association
 * @param CapabilityInformation Capabilities of device requesting association
 */
void usr_mlme_associate_ind(uint64_t DeviceAddress,
                            uint8_t CapabilityInformation)
{
#if (DEBUG > 0)
#ifdef SIO_HUB
	printf("Received association request.\n");
#endif
#endif
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
#if (DEBUG > 0)
#ifdef SIO_HUB
		printf("Association successful.\n- device ID = 0x%04x\n", associate_short_addr);
#endif
#endif
        wpan_mlme_associate_resp(DeviceAddress,
                                 associate_short_addr,
                                 ASSOCIATION_SUCCESSFUL);
		
		pal_led(LED_NWK_SETUP, LED_ON);
		pal_timer_start(APP_TIMER_LED_OFF,
						LED_ON_DURATION_SHORT,
						TIMEOUT_RELATIVE,
						(FUNC_PTR)associate_device_cb,
						(void *)0);
    }
    else
    {
        wpan_mlme_associate_resp(DeviceAddress,
                                 associate_short_addr,
                                 PAN_AT_CAPACITY);
    }

    /* Keep compiler happy. */
    CapabilityInformation = CapabilityInformation;
}


/**
 * @brief Small callback function indicating a node association
 *
 * @param parameter		Counter for LED flashing         
 */
void associate_device_cb(void *param)
{
	if((int)param < ASSOCIATE_DEVICE_FLASH)
	{
		param++;
		pal_led(LED_NWK_SETUP, LED_TOGGLE);
		pal_timer_start(APP_TIMER_LED_OFF,
						LED_ON_DURATION_SHORT,
						TIMEOUT_RELATIVE,
						(FUNC_PTR)associate_device_cb,
						(void *)&param);
	}
	else
	{
		pal_led(LED_NWK_SETUP, LED_ON);
		pal_timer_stop(APP_TIMER_LED_OFF);
	}						
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
    if (status == MAC_SUCCESS)
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
 * @brief Application specific function to assign a short address
 *
 */
static bool assign_new_short_addr(uint64_t addr64, uint16_t *addr16)
{
    uint8_t i;

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
            // Assign the previously assigned short address again
            *addr16 = device_list[i].short_addr;
            return true;
        }
    }

    for (i = 0; i < MAX_NUMBER_OF_DEVICES; i++)
    {
        if (device_list[i].short_addr == 0x0000)
        {
            *addr16 = CPU_ENDIAN_TO_LE16(i + 0x0001);
            device_list[i].short_addr = CPU_ENDIAN_TO_LE16(i + 0x0001); // get next short address
            device_list[i].ieee_addr = addr64;      // store extended address
            return true;
        }
    }

    // If we are here, no short address could be assigned.
    return false;
}



/* === INDICATE DATA RECEPTION: NOT USED! =================================== */

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
    pal_led(LED_DATA, LED_TOGGLE);
	static uint8_t i;
	
#if (DEBUG > 0)
	printf("Data received...\nLink quality: %d\n", mpduLinkQuality);
	for(i = 0; i < msduLength; i++)
	{
		printf("msdu[%d]: 0x%02x\n", i, msdu[i]);
	}
#else
	for(i = 0; i < msduLength; i++)
	{
		putchar(msdu[i]);
	}
#endif
	pal_led(LED_DATA, LED_TOGGLE);


   /*
    * Dummy data has been received successfully.
    * Application code could be added here ...
    */
    // Start a timer switching off the LED
    //pal_timer_start(APP_TIMER_LED_OFF,
                    //LED_ON_DURATION_SHORT,
                    //TIMEOUT_RELATIVE,
                    //(FUNC_PTR)data_exchange_led_off_cb,
                    //NULL);
//
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
 * @brief Callback function usr_mlme_associate_conf
 *
 * @param AssocShortAddress    Short address allocated by the coordinator
 * @param status               Result of requested association operation
 */
void usr_mlme_associate_conf(uint16_t AssocShortAddress, uint8_t status)
{
    //if (status == MAC_SUCCESS)
    //{
        //// Stop timer used for search indication (same as used for data transmission)
        //pal_timer_stop(APP_TIMER_LED_OFF);
        //pal_led(LED_NWK_SETUP, LED_ON);
//
        //// Start a timer that sends some data to the coordinator every 2 seconds.
        ////pal_timer_start(TIMER_TX_DATA,
                        ////DATA_TX_PERIOD,
                        ////TIMEOUT_RELATIVE,
                        ////(FUNC_PTR)app_timer_cb,
                        ////NULL);
    //}
    //else
    //{
        //// Something went wrong; restart
        //wpan_mlme_reset_req(true);
    //}

    /* Keep compiler happy. */
    AssocShortAddress = AssocShortAddress;
}


/**
 * @brief Callback function for the application timer
 *
 * @param AssocShortAddress    Short address allocated by the coordinator
 * @param status               Result of requested association operation
 */
//static void app_timer_cb(void *parameter)
//{
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
//
    //uint8_t src_addr_mode;
    //wpan_addr_spec_t dst_addr;
    //uint8_t payload;
    //static uint8_t msduHandle = 0;
//
    //src_addr_mode = WPAN_ADDRMODE_SHORT;
//
    //dst_addr.AddrMode = WPAN_ADDRMODE_SHORT;
    //dst_addr.PANId = DEFAULT_PAN_ID;
    //ADDR_COPY_DST_SRC_16(dst_addr.Addr.short_address, coord_addr.short_addr);
//
    //payload = (uint8_t)rand();  // any dummy data
    //msduHandle++;               // increment handle
    ////wpan_mcps_data_req(src_addr_mode,
                       ////&dst_addr,
                       ////1,
                       ////&payload,
                       ////msduHandle,
                       ////WPAN_TXOPT_ACK);
//
    //pal_timer_start(TIMER_TX_DATA,
                    //DATA_TX_PERIOD,
                    //TIMEOUT_RELATIVE,
                    //(FUNC_PTR)app_timer_cb,
                    //NULL);
//
    //parameter = parameter;  /* Keep compiler happy. */
//}



/* === CONFIRM DATA TRANMISSION: NOT USED YET!!! =============================== */

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
    //if (status == MAC_SUCCESS)
    //{
        ///*
         //* Dummy data has been transmitted successfully.
         //* Application code could be added here ...
         //*/
        //pal_led(LED_DATA, LED_TOGGLE);
        //// Start a timer switching off the LED
        //pal_timer_start(APP_TIMER_LED_OFF,
                        //500000,
                        //TIMEOUT_RELATIVE,
                        //(FUNC_PTR)data_exchange_led_off_cb,
                        //NULL);
    //}

    /* Keep compiler happy. */
    msduHandle = msduHandle;
#ifdef ENABLE_TSTAMP
    Timestamp = Timestamp;
#endif  /* ENABLE_TSTAMP */
}


/**
 * @brief Callback function switching off the LED
 */
//static void data_exchange_led_off_cb(void *parameter)
//{
    //pal_led(LED_DATA, LED_OFF);
	//pal_timer_stop(APP_TIMER_LED_OFF);
//
    //parameter = parameter;  /* Keep compiler happy. */
//}




/* EOF */