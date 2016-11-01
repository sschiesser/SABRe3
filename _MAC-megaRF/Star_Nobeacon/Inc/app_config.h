/**
 * @file app_config.h
 *
 * @brief These are application-specific resources which are used
 *        in the example application of the device in addition to the
 *        underlaying stack.
 *
 * $Id: app_config.h 12345 2011-12-07 16:43:56Z uwalter $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2009, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* Prevent double inclusion */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* === Includes ============================================================= */

#include "stack_config.h"

/* === Macros =============================================================== */

/** Check the compiler flags to determine which channel and data rate are used,
  * in order to apply the corresponding channel page */
#ifdef CHANNEL
	#define DEFAULT_CHANNEL                 (CHANNEL)
	#ifdef HIGH_DATA_RATE_SUPPORT
		#if (DATA_RATE == 2000)
			#define DEFAULT_CHANNEL_PAGE    (17)
		#elif (DATA_RATE == 1000)
			#define DEFAULT_CHANNEL_PAGE	(16)
		#elif (DATA_RATE == 500)
			#define DEFAULT_CHANNEL_PAGE	(2)
		#endif /* #if (DATA_RATE == ) */
	#else
		#define DEFAULT_CHANNEL_PAGE		(0)
	#endif /* #ifdef HIGH_DATA_RATE_SUPPORT */
#else
	#define DEFAULT_CHANNEL                 (20)
	#ifdef HIGH_DATA_RATE_SUPPORT
		#if (DATA_RATE == 2000)
			#define DEFAULT_CHANNEL_PAGE	(17)
		#elif (DATA_RATE == 1000)
			#define DEFAULT_CHANNEL_PAGE	(16)
		#elif (DATA_RATE == 500)
			#define DEFAULT_CHANNEL_PAGE	(2)
		#endif /* if (DATA_RATE == ) */
	#else
		#define DEFAULT_CHANNEL_PAGE        (0)
	#endif  /* #ifdef HIGH_DATA_RATE_SUPPORT */
#endif  /* #ifdef CHANNEL */

/** Defines the PAN ID of the network. */
#ifdef PAN_ID
	#define DEFAULT_PAN_ID                  CCPU_ENDIAN_TO_LE16(PAN_ID)
#else
	#define DEFAULT_PAN_ID                  CCPU_ENDIAN_TO_LE16(0x5ABE)
#endif /* #ifdef PAN_ID */

/** Defines the short address of the coordinator. */
#ifdef COORD_ADDR
	#define COORD_SHORT_ADDR				(COORD_ADDR)
#else
	#define COORD_SHORT_ADDR                (0x0000)
#endif /* #ifdef COORD_ADDR */

/** Maximum number of devices the coordinator will handle. */
#define MAX_NUMBER_OF_DEVICES           (100)

/** Time period in microseconds for toggling NWK LED during network scan */
#define LED_NWK_PERIOD                  (500000)

/** Time period in microseconds for switching off the data reception LED */
#define LED_DATA_PERIOD                 (100)

//#define SABRE_GENERATE_DUMMY
//#define SABRE_USB_HIGHSPEED
/** Time intervals in microseconds for sending data to SIO (coordinator),
  * depending on Debug or Release mode */
#define SABRE_SIO_PERIOD_MIN            (100)
#define SABRE_SIO_PERIOD_FAST_US        (100)
#define SABRE_SIO_PERIOD_SLOW_US        (200000)
#if (DEBUG > 0)
	#if (SABRE_SIO_PERIOD_SLOW_US < SABRE_SIO_PERIOD_MIN)
		#define SABRE_SIO_PERIOD        (SABRE_SIO_PERIOD_MIN)
	#else
		#define SABRE_SIO_PERIOD        (SABRE_SIO_PERIOD_SLOW_US)
	#endif
#else
	#if (SABRE_SIO_PERIOD_FAST_US < SABRE_SIO_PERIOD_MIN)
		#define SABRE_SIO_PERIOD       (SABRE_SIO_PERIOD_MIN)
	#else
		#define SABRE_SIO_PERIOD       (SABRE_SIO_PERIOD_FAST_US)
	#endif
#endif

/** Time intervals in microseconds for sending data to TRX (node),
  * depending on Debug or Release mode */
#define SABRE_TRX_PERIOD_MIN           (100)
#define SABRE_TRX_PERIOD_FAST_US       (5000)
#define SABRE_TRX_PERIOD_SLOW_US       (1000000)
#if (DEBUG > 0)
	#if (SABRE_TRX_PERIOD_SLOW_US < SABRE_TRX_PERIOD_MIN)
		#define SABRE_TRX_PERIOD       (SABRE_TRX_PERIOD_MIN)
	#else
		#define SABRE_TRX_PERIOD       (SABRE_TRX_PERIOD_SLOW_US)
	#endif
#else
	#if (SABRE_TRX_PERIOD_FAST_US < SABRE_TRX_PERIOD_MIN)
		#define SABRE_TRX_PERIOD      (SABRE_TRX_PERIOD_MIN)
	#else
		#define SABRE_TRX_PERIOD      (SABRE_TRX_PERIOD_FAST_US)
	#endif
#endif

/** Bit mask of channels that should be scanned. */
#define SCAN_ALL_CHANNELS              (0x07FFF800)

/** Network scan duration time. */
#define SCAN_DURATION                  (4)
/** Maximum number of scans before restarting. */
#define MAX_NUMBER_OF_SCANS            (3)

/** @brief This is the first timer identifier of the application.
 *
 *  The value of this identifier is an increment of the largest identifier
 *  value used by the MAC.
 */
#if (NUMBER_OF_TOTAL_STACK_TIMERS == 0)
	#define APP_FIRST_TIMER_ID         (0)
#else
	#define APP_FIRST_TIMER_ID         (LAST_STACK_TIMER_ID + 1)
#endif

/** Assign functions to LEDs depending on their number */
#if (NO_OF_LEDS >= 3)
	#define LED_START                  (LED_0)
	#define LED_NWK_SETUP              (LED_1)
	#define LED_DATA                   (LED_2)
#elif (NO_OF_LEDS == 2)
	#define LED_START                  (LED_0)
	#define LED_NWK_SETUP              (LED_0)
	#define LED_DATA                   (LED_1)
#else
	#define LED_START                  (LED_0)
	#define LED_NWK_SETUP              (LED_0)
	#define LED_DATA                   (LED_0)
#endif

/* === Types ================================================================ */

/** Application state enumerator */
typedef enum app_state_tag {
	SABRE_APP_STARTING=0,
	SABRE_WPAN_STARTING,				// 1
	SABRE_COORDINATOR_WAITING,			// 2
	SABRE_RUNNING,						// 3
	SABRE_KEYS_POLLING,					// 4
	SABRE_KEYS_POLLING_DONE,			// 5
	SABRE_KEYS_SORTING,					// 6
	SABRE_KEYS_SORTING_DONE,			// 7
	SABRE_IMU_POLLING,					// 8
	SABRE_IMU_POLLING_DONE,				// 9
	SABRE_AIRMEMS_POLLING,				// 10
	SABRE_AIRMEMS_POLLING_DONE,			// 11
	SABRE_AIRMEMS_CALCULATING,			// 12
	SABRE_AIRMEMS_READY,				// 13
	SABRE_DATA_PACKING,					// 14
	SABRE_RUNNING_ERROR,				// 15
	SABRE_SYNC_LOST						// 16
} app_state_t;

/** Structure storing the short and the extended address of a device */
typedef struct associated_device_tag {
    uint16_t short_addr;
    uint64_t ieee_addr;
} associated_device_t;

/** Timer ID's used by the Application */
typedef enum {
    /* App Timers start from APP_FIRST_TIMER_ID */

    /** Application timer id used by the device to send data */
    TIMER_TX_DATA = (APP_FIRST_TIMER_ID),

    /** Application timer id used to switch off LED */
    TIMER_LED_OFF = (APP_FIRST_TIMER_ID + 1),
	
	/** Application timer id used to wait for MS5803 pressure sensor */
	TIMER_MS58 = (APP_FIRST_TIMER_ID + 2),
	
	/** Application timer ids used to wait for any non responding IMU part */
	TIMER_IMU_GYRO = (APP_FIRST_TIMER_ID + 3),
	TIMER_IMU_COMP = (APP_FIRST_TIMER_ID + 4),
	TIMER_IMU_ACCEL = (APP_FIRST_TIMER_ID + 5)
} SHORTENUM app_timer_t;


/** Defines the number of timers used by the application. */
#define NUMBER_OF_APP_TIMERS        (6)

/** Defines the total number of timers used by the application and the layers below. */
#define TOTAL_NUMBER_OF_TIMERS      (NUMBER_OF_APP_TIMERS + NUMBER_OF_TOTAL_STACK_TIMERS)

/** Defines the number of additional large buffers used by the application */
#define NUMBER_OF_LARGE_APP_BUFS    (1)

/** Defines the number of additional small buffers used by the application */
#define NUMBER_OF_SMALL_APP_BUFS    (0)


/** Defines the total number of large buffers used by the application and the
  * layers below. */
#define TOTAL_NUMBER_OF_LARGE_BUFS  (NUMBER_OF_LARGE_APP_BUFS + NUMBER_OF_LARGE_STACK_BUFS)

/** Defines the total number of small buffers used by the application and the
  * layers below. */
#define TOTAL_NUMBER_OF_SMALL_BUFS  (NUMBER_OF_SMALL_APP_BUFS + NUMBER_OF_SMALL_STACK_BUFS)

/** Defines the total number of small and large buffers used by the application and the
  * layers below. */
#define TOTAL_NUMBER_OF_BUFS        (TOTAL_NUMBER_OF_LARGE_BUFS + TOTAL_NUMBER_OF_SMALL_BUFS)


/** Defines the transceiver buffer size */
#define TRX_BUFFER_SIZE				(127)

/** Defines the SIO buffer size*/
#define SIO_BUFFER_SIZE				(255)

/** Defines the USB transmit buffer size */
#define USB_TX_BUF_SIZE             (10)

/** Defines the USB receive buffer size */
#define USB_RX_BUF_SIZE             (10)



/**
  * USB-specific definitions
  */

/** USB Vendor ID (16-bit number) */
#define USB_VID                 0x03EB /* Atmel's USB vendor ID */

/** USB Product ID (16-bit number) */
#define USB_PID                 0x2018 /* RZ USB stick product ID */

/** USB Release number (BCD format, two bytes) */
#define USB_RELEASE             { 0x00, 0x01 } /* 01.00 */

/** Maximal number of UTF-16 characters used in any of the strings
  * below.  This is only used for compilers that cannot handle the
  * initialization of flexible array members within structs. */
#define USB_STRING_SIZE         10

/** String representation for the USB vendor name. */
#define USB_VENDOR_NAME L"ATMEL"

/** String representation for the USB product name. */
#define USB_PRODUCT_NAME L"STK600"

/** Defines the UART transmit buffer size */
#define UART_MAX_TX_BUF_LENGTH      (10)

/** Defines the UART receive buffer size */
#define UART_MAX_RX_BUF_LENGTH      (10)

/** Offset of IEEE address storage location within EEPROM */
#define EE_IEEE_ADDR                (0)


/* === Externals ============================================================ */

/* === Prototypes =========================================================== */

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* APP_CONFIG_H */
/* EOF */
