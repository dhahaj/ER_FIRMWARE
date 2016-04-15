/*
*  interrupts.c -> Interrupt service routines definitions
*
*  Created: 1/23/2013 11:55:43 AM
*  Modified: 8/20/2014 9:00 AM
*  Author: DMH
*/

#include "interrupts.h"
#include "timers.h"
#include "main.h"

volatile unsigned long timer0_millis = 0;
volatile static ui8 timer0_fract = 0;
volatile uint8_t counter_dr1=0, counter_dr2=0;

// Timer1A Compare Interrupt - Handles the independent operation
ISR ( TIMER1_COMPA_vect )
{
	#if (!EExER) // Disabled for EExER
	unsigned long t;
	// Check Door 1
	if(dr1.isActive)
	{
		// Door1 Button is still pressed, reset the counter
		if(bit_is_clear(INPUT_PIN_PORT, DR1_BUTTON)) counter_dr1 = 0;
		else if( ++counter_dr1 > ((t=getTime(DR1_OUT))*10) ) // When the counter is >= time1, turn off door 1
		{
			activateDoor1(false);
			dr1.isActive = false;
			counter_dr1 = 0; // reset the counter for the next go round
		}
	}

	// Check Door 2
	if(dr2.isActive)
	{
		// Door2 Button is still pressed, reset the counter
		if(bit_is_clear(INPUT_PIN_PORT, DR2_BUTTON)) counter_dr2=0;
		else if( ++counter_dr2 > ((t=getTime(DR2_OUT))*10) ) // When the counter is >= time2, turn off door 2
		{
			activateDoor2(false);
			dr2.isActive = false;
			counter_dr2 = 0; // reset the counter for the next go round
		}
	}
	#endif
}

// Timer0 overflow interrupt - Increments the millis variable
ISR(TIMER0_OVF_vect)
{
	unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;

	m += MILLIS_INC;
	f += FRACT_INC;

	if (f >= FRACT_MAX)
	{
		f -= FRACT_MAX;
		m += 1;
	}
	timer0_fract = f;
	timer0_millis = m;
}

/*
Interrupt Handler For PCINT23:16 (PIND)
This interrupt signal routine is called when any of the DIP
settings input pins are changed. It must read the new settings
and apply them accordingly. */
ISR(PCINT2_vect)
{
	#if (!EExER) // the EExER build disables this setting
	uint8_t reg = (INPUT_PIN_PORT^dip.inputReg); // The set bits in 'reg' correspond to a pin change on the DIP switch

	if(bit_get(reg, MODE_PIN)) // Operating Mode Changed
	{
		// Set the new operating mode
		dip.mode = (bit_get(INPUT_PIN_PORT, MODE_PIN)>0) ? DEPENDENT : INDEPENDENT;
		if(dip.mode==INDEPENDENT) // Went from dependent -> independent
		{
			if(doorIsActive(dr1) || doorIsActive(dr2)) // Just inactivate all the doors
			{
				doorPinWrite(dr1, false);
				doorPinWrite(dr2, false);
				cbi(RELAY_PORT,RELAY1_PIN);
				cbi(RELAY_PORT, RELAY2_PIN);
				door_timer(true); // Reset the timer
			}
		} else /*Went from independent -> dependent, check and stop any active doors*/ {
			if(doorIsActive(dr1))
			{
				doorPinWrite(dr1, false);
				cbi(RELAY_PORT,RELAY1_PIN);
			}

			if(doorIsActive(dr2))
			{
				doorPinWrite(dr2, false);
				cbi(RELAY_PORT,RELAY2_PIN);
			}
		}
		bit_write(dip.mode, dip.inputReg, 1);
	}

	if(bit_get(reg, ACTIVE_MODE1_PIN)) // Door 1 Active Mode Changed
	{
		dr1.outputMode = (bit_get(INPUT_PIN_PORT, ACTIVE_MODE1_PIN)) ? ACTIVE_HIGH:ACTIVE_LOW; // Set the Door 1 active mode
		(bit_get(INPUT_PIN_PORT, ACTIVE_MODE1_PIN)>0) ? sbi(dip.inputReg, ACTIVE_MODE1_PIN) : cbi(dip.inputReg, ACTIVE_MODE1_PIN); // Clear or set the input register
		doorPinWrite(dr1, ((dr1.isActive||dr1.isToggled||dip.retracting) ? true:false) ); // Change door 1 output to correspond with the changes
	}

	if(bit_get(reg, ACTIVE_MODE2_PIN)) // Door 2 Active Mode Changed
	{
		dr2.outputMode = (bit_get(INPUT_PIN_PORT, ACTIVE_MODE2_PIN)) ? ACTIVE_HIGH:ACTIVE_LOW; // Set the Door 2 active mode
		(bit_get(INPUT_PIN_PORT, ACTIVE_MODE2_PIN)) ? sbi(dip.inputReg, ACTIVE_MODE2_PIN) : cbi(dip.inputReg, ACTIVE_MODE2_PIN); // Clear or set the input register
		doorPinWrite(dr2, ((dr2.isActive || dr2.isToggled||dip.retracting) ? true:false) ); // Change door 2 output to correspond with the changes
	}

	if(bit_get(reg, TOGGLE1_PIN)) // Door 1 Toggle Mode Changed
	{
		dr1.toggleMode = bit_get(INPUT_PIN_PORT, TOGGLE1_PIN) ? false:true; // Set the door 1 toggle value
		dr1.toggleMode ? cbi(dip.inputReg, TOGGLE1_PIN) : sbi(dip.inputReg, TOGGLE1_PIN); // Clear or set the input register
		if( (dr1.isToggled || dr1.isActive) && dr1.toggleMode==false ) // Toggle mode changed to off and output is toggled! Revert outputs now...
		{
			doorPinWrite(dr1,false);
			cbi(RELAY_PORT,RELAY1_PIN);
			dr1.isToggled = false;
			dr1.isActive = false;
			if(dip.mode==DEPENDENT)
			{
				doorPinWrite(dr2,false);
				cbi(RELAY_PORT,RELAY2_PIN);
				dr2.isToggled = false;
			}
		}
	}

	if(bit_get(reg, TOGGLE2_PIN)) // Door 2 Toggle Mode Changed
	{
		dr2.toggleMode = bit_get(INPUT_PIN_PORT, TOGGLE2_PIN) ? false:true; // Set the door 2 toggle value
		dr2.toggleMode ? cbi(dip.inputReg,TOGGLE1_PIN) : sbi(dip.inputReg,TOGGLE1_PIN); // Clear or set the input register
		if( (dr2.isToggled || dr2.isActive) && dr2.toggleMode==false && dip.mode == INDEPENDENT) // Toggle mode changed to off and output is toggled! Revert outputs now...
		{
			doorPinWrite(dr2,false);
			cbi(RELAY_PORT, RELAY2_PIN);
			dr2.isToggled = false;
			dr2.isActive = false;
		}
		//bit_write( !bit_get(dr2.toggleMode,0), dip.inputReg, BIT(7)); // Change the input register value
	}
	#endif /*  EExER */
}