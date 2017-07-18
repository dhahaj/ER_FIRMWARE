/*
*  interrupts.c -> Interrupt service routines definitions
*
*  Created: 1/23/2013 11:55:43 AM
*  Modified: 7/30/2015 12:29 PM
*  Author: DMH
*/

#include "interrupts.h"
#include "timers.h"
#include "main.h"

volatile unsigned long timer0_millis = 0;
volatile uint8_t timer0_fract = 0;
volatile uint16_t counter_dr1=0, counter_dr2=0;

ISR ( TIMER1_COMPA_vect ) /*	Timer1A Compare Interrupt - Handles the independent operation */
{
#if (!EExER) // Disabled for EExER

	unsigned long t;
	if(dr1.is_active && !dr1.toggle_mode) // Check Door 1
	{
		if(bit_is_clear(INPUT_PIN_PORT, DR1_BUTTON)){
			 counter_dr1 = 0; // Door1 Button is still pressed, reset the counter
		}
		else if( ++counter_dr1 > ((t=getTime(DR1_OUT))*10) ) // When the counter is >= time1, turn off door 1
		{
			activateDoor1(false);
			dr1.is_active = false;
			//dr1.toggled = false;
			counter_dr1 = 0; // reset the counter for the next go round
		}
		nop;
	}

	// Check Door 2
	if(dr2.is_active && !dr2.toggle_mode)
	{
		if(bit_is_clear(INPUT_PIN_PORT, DR2_BUTTON)) {
			counter_dr2=0; // Door2 Button is still pressed, reset the counter
		}
		else if( ++counter_dr2 > ((t=getTime(DR2_OUT))*10) ) // When the counter is >= time2, turn off door 2
		{
			activateDoor2(false);
			dr2.is_active = false;
			//dr2.toggled = false;
			counter_dr2 = 0; // reset the counter for the next go round
		}
		nop;
	}
#endif
}

// Timer0 overflow interrupt - Increments the millis variable
ISR(TIMER0_OVF_vect)
{
	unsigned long m = timer0_millis;
	ui8 f = timer0_fract;

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
// the EExER build disables this setting
#if (!EExER)
	uint8_t reg = (INPUT_PIN_PORT^dip_switch.prev_values); // The set bits in 'reg' correspond to a pin change on the DIP switch

	if(bit_get(reg, MODE_PIN)) /* Operating Mode Changed */
	{
		dip_switch.mode = (bit_get(INPUT_PIN_PORT, MODE_PIN)>0) ? DEPENDENT : INDEPENDENT; // Set the new operating mode
		if(dip_switch.mode==INDEPENDENT) /* Went from dependent -> independent */
		{
			if(doorActive(&dr1) || doorActive(&dr2)) // Just inactivate all the doors
			{
				doorPinWrite(&dr1, false);
				doorPinWrite(&dr2, false);
				cbi(RELAY_PORT, RELAY1_PIN);
				cbi(RELAY_PORT, RELAY2_PIN);
				door_timer(true); // Reset the timer
			}
		} else /* Went from independent -> dependent, check and stop any active doors */
			{
				if(doorActive(&dr1))
				{
					doorPinWrite(&dr1, false);
					cbi(RELAY_PORT,RELAY1_PIN);
					dr1.toggled = false;
				}

				if(doorActive(&dr2))
				{
					doorPinWrite(&dr2, false);
					cbi(RELAY_PORT,RELAY2_PIN);
					dr2.toggled = false;
				}
			}
			bit_write(dip_switch.mode, dip_switch.prev_values, 1);
		}

		if(bit_get(reg, ACTIVE_MODE1_PIN)) 	/* Door 1 Active Mode Changed */
		{
			dr1.active_high = (bit_get(INPUT_PIN_PORT, ACTIVE_MODE1_PIN)) ? ACTIVE_HIGH:ACTIVE_LOW; // Set the Door 1 active mode
			(bit_get(INPUT_PIN_PORT, ACTIVE_MODE1_PIN)>0) ? sbi(dip_switch.prev_values, ACTIVE_MODE1_PIN) : cbi(dip_switch.prev_values, ACTIVE_MODE1_PIN); // Clear or set the input register
			doorPinWrite(&dr1, ((dr1.is_active||dr1.toggled||dip_switch.retracting) ? true : false) ); // Change door 1 output to correspond with the changes
		}

		if(bit_get(reg, ACTIVE_MODE2_PIN)) /* Door 2 Active Mode Changed */
		{
			dr2.active_high = (bit_get(INPUT_PIN_PORT, ACTIVE_MODE2_PIN)) ? ACTIVE_HIGH:ACTIVE_LOW; // Set the Door 2 active mode
			(bit_get(INPUT_PIN_PORT, ACTIVE_MODE2_PIN)) ? sbi(dip_switch.prev_values, ACTIVE_MODE2_PIN) : cbi(dip_switch.prev_values, ACTIVE_MODE2_PIN); // Clear or set the input register
			doorPinWrite(&dr2, ((dr2.is_active || dr2.toggled||dip_switch.retracting) ? true : false) ); // Change door 2 output to correspond with the changes
		}

		if(bit_get(reg, TOGGLE1_PIN)) /* Door 1 Toggle Mode Changed */
		{
			dr1.toggle_mode = bit_get(INPUT_PIN_PORT, TOGGLE1_PIN) ? false:true; // Set the door 1 toggle value
			dr1.toggle_mode ? cbi(dip_switch.prev_values, TOGGLE1_PIN) : sbi(dip_switch.prev_values, TOGGLE1_PIN); // Clear or set the input register
			if( (dr1.toggled || dr1.is_active) && dr1.toggle_mode==false ) // Toggle mode changed to off and output is toggled! Revert outputs now...
			{
				doorPinWrite(&dr1, false);
				cbi(RELAY_PORT, RELAY1_PIN);
				dr1.toggled = false;
				dr1.is_active = false;
				if(dip_switch.mode==DEPENDENT)
				{
					doorPinWrite(&dr2,false);
					cbi(RELAY_PORT,RELAY2_PIN);
					dr2.toggled = false;
					dr2.is_active = false;
				}
			} else /* Toggle mode changed to ON */
			{
				if(dr1.is_active)
				{
					doorPinWrite(&dr1, false);
					cbi(RELAY_PORT, RELAY1_PIN);
					dr1.toggled = false;
					dr1.is_active = false;
				}
			}
		}

		if(bit_get(reg, TOGGLE2_PIN)) /* Door 2 Toggle Mode Changed */
		{
			dr2.toggle_mode = bit_get(INPUT_PIN_PORT, TOGGLE2_PIN) ? false : true; // Set the door 2 toggle value
			dr2.toggle_mode ? cbi(dip_switch.prev_values, TOGGLE2_PIN) : sbi(dip_switch.prev_values, TOGGLE2_PIN); // Clear or set the input register

			if( (dr2.toggled || dr2.is_active) && dip_switch.mode == INDEPENDENT) /* Check for any active outputs and turn them off */
			{
				doorPinWrite(&dr2, false);
				cbi(RELAY_PORT, RELAY2_PIN);
				dr2.toggled = false;
				dr2.is_active = false;
			}
		}
#endif /*  EExER */
}
