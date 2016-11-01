/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

#define F_CPU 16000000UL
/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#include <util/delay.h>

int main (void)
{
	// Insert system clock initialization code here (sysclk_init()).

	board_init();
	
	PORTB = 0xFF;
	DDRB = 0xFF;
	
	for(;;) {
		PORTB ^= 0xFF;
		_delay_ms(400);
		PORTB ^= 0xFF;
		_delay_ms(400);
	}

	// Insert application code here, after the board has been initialized.
}
