/*
* timers.c --> Code for reading the timing selection switches
*
* Created: 11/14/2012 8:57 AM
* Modified: 8/20/2014 9:17 AM
*  Author: DMH
*
*/

#include <util/atomic.h>
#include "timers.h"
#include "interrupts.h"
#include "door.h"

// Array holding the 4-bit timing selection values (in seconds), Stored in PGM Memory
const ui8 PROGMEM timing_values_PGM[] = {
	1,	// 0
	2,	// 1
	3,	// 2
	5,	// 3
	7,	// 4
	10, // 5
	15,	// 6
	20,	// 7
	25,	// 8
	30, // 9
	35, // A
	40,	// B
	45,	// C
	50, // D
	55,	// E
	60, // F
};

// Reads the time selection switch port
unsigned long getTime(uint8_t pin)
{
	unsigned long t;
	uint8_t ds_port_value;

	// Set the DS Enable signal
	if(pin==DR1_OUT)
	{
		sbi(DS_PORT, DS1_PIN);
		cbi(DS_PORT, DS2_PIN);
	} else {
		sbi(DS_PORT, DS2_PIN);
		cbi(DS_PORT, DS1_PIN);
	}
	ds_port_value = (DS_PIN_PORT & 0x3c)>>2;
	t = getTimingValue(ds_port_value);
	(pin==DR1_OUT) ? cbi(DS_PORT, DS1_PIN) : cbi(DS_PORT, DS2_PIN);
	return t;
}

//  Resets the timer or returns the time (ms) since the last reset
unsigned long door_timer(bool reset)
{
	if(reset) 	// Reset the Door Timer and return
	{
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			timer0_millis = 0;
			timer0_fract = 0;
		}
		return 0;
	}

	// Else, Return the time since last reset
	unsigned long t;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		t = timer0_millis;
	}
	return t;
}

