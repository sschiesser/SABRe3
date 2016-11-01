/* Teensy RawHID example
 * http://www.pjrc.com/teensy/rawhid.html
 * Copyright (c) 2009 PJRC.COM, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above description, website URL and copyright notice and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "usb_rawhid.h"

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

/* SABRe messages properties */
#define SABRE_ADDRESS_01 0xF0	// SABRe LH
#define SABRE_MESSLEN_01 23
#define SABRE_ADDRESS_02 0xF1	// SABRe RH
#define SABRE_MESSLEN_02 42
#define SABRE_ADDRESS_03 0xF2	// SABRe airMEMS
#define SABRE_MESSLEN_03 15

#define SABRE_MESSLEN_MAX SABRE_MESSLEN_02

/* Global variables */
volatile uint8_t do_output;		// Flag to enable USB transmission
volatile uint8_t buffer_counter;	// Buffer counter
volatile uint8_t sync_flag;		// Synchronization flag between UART input and buffer start
uint8_t buffer[64];				// Storage buffer between UART and USB

//void init_buffer(void)
//{
	//static uint8_t i;
	//for(i = 0; i < 64; i++)
	//{
		//buffer[i] = 0;
	//}
//}


int main(void)
{
	static uint8_t retVal;
	static uint8_t timeoutCnt = 0;
	static uint8_t newValues = 1;
	
	// set for 16 MHz clock
	CPU_PRESCALE(0);

	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();
	while (!usb_configured()) /* wait */ ;

	// Initialize the USART1 for serial data reception @ 230400 bps from receiver
	/* Must turn off USART before reconfiguring it, otherwise incorrect operation may occur */
	//UCSR1B = 0;
	//UCSR1A = 0;
	//UCSR1C = 0;
	/* Set the new baud rate before configuring the USART */
	//UBRR1  = 8;
	/* Reconfigure the USART in double speed mode for a wider baud rate range at the expense of accuracy */
	//UCSR1C = ((1 << UCSZ10) | (1 << UCSZ11));
	//UCSR1A = (1 << U2X1);
	//UCSR1B = ((1 << RXCIE1) | (1 << RXEN1));
	
	/* Initialize the pin PC6 to monitor USART reception and PC7 to show lost synchronization */
	DDRC |= ((1 << DDC6) | (1 << DDC7));
	
	do_output = 0;
	buffer_counter = 0;
	sync_flag = 0;	
	
	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

	while (1) {
		// if time to send output, transmit something interesting
		if (do_output) {
			do_output = 0;
			if((retVal = usb_rawhid_send(buffer, 0)) == 0) {
				timeoutCnt += 1;
			}
			else {
				//timeoutCnt = 0;
				newValues = 1;
			}
			////init_buffer();
		}

		if(newValues) {
			static uint8_t i;
			for(i = 0; i < 42; i++) {
				buffer[i] = (uint8_t)(rand() % 255);
				//buffer[i] = (uint8_t)(2*i);
			}
			buffer[0] = 0x41;
			buffer[1] = 0xF1;
			buffer[41] = 0x5A;
			//buffer[63] = timeoutCnt;
			timeoutCnt = 0;
			do_output = 1;
			newValues = 0;
		}
	}
}


/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
 *  for later transmission to the host.
 */
ISR(USART1_RX_vect, ISR_BLOCK)
{
	cli();
	buffer[buffer_counter] = UDR1;
	
	/* If sync between buffer and receiver message lost, wait for start frame and re-sync */
	if(sync_flag == 0) {
		if((buffer_counter == 0) && (buffer[buffer_counter] == 0x41)) {
			PORTC |= (1 << PORTC7);
			sync_flag = 1;
			PORTC &= ~(1 << PORTC7);
		}
		else {
			//PORTC |= (1 << PORTC7);
			buffer_counter = 0;
			sync_flag = 0;
			//PORTC &= ~(1 << PORTC7);
		}
	}
	
	if(sync_flag) {
		if(buffer[buffer_counter] == 0x5A) {
			if(buffer[0] == 0x41) {
				if((buffer[1] == SABRE_ADDRESS_01) && (buffer_counter == (SABRE_MESSLEN_01-1))) {
					PORTC |= (1 << PORTC6);
					do_output = 1;
					buffer_counter = 0;
					PORTC &= ~(1 << PORTC6);
				}
				else if((buffer[1] == SABRE_ADDRESS_02) && (buffer_counter == (SABRE_MESSLEN_02-1))) {
					PORTC |= (1 << PORTC6);
					do_output = 1;
					buffer_counter = 0;
					PORTC &= ~(1 << PORTC6);
				}
				else if((buffer[1] == SABRE_ADDRESS_03) && (buffer_counter == (SABRE_MESSLEN_03-1))) {
					PORTC |= (1 << PORTC6);
					do_output = 1;
					buffer_counter = 0;
					PORTC &= ~(1 << PORTC6);
				}
				else {
					buffer_counter += 1;
				}
			}
			else {
				buffer_counter += 1;
			}
		}
		else {
			if(buffer_counter >= SABRE_MESSLEN_MAX) {
				PORTC |= ((1 << PORTC6) || (1 << PORTC7));
				sync_flag = 0;
				buffer_counter = 0;
				PORTC &= ((1 << PORTC6) || (1 << PORTC7));
			}
			else {
				buffer_counter += 1;
			}
		}
	}
	sei();
}



