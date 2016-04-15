/*
*  Revision 5.0 - For Release, UL Testing
*  main.c -> Main code for the ER Controller, with EExER function definitions.
*
*  Created: 1/23/2013 11:51 AM
*  Modified: 8/20/2014 9:00 AM
*  Author: DMH
*
*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <avr/interrupt.h>
#include <avr/portpins.h>
#include <avr/sfr_defs.h>
#include "main.h"
#include "timers.h"
#include "interrupts.h"

/*
*	Check that EExER is defined for build selection by the preprocessor
*/
#ifdef EExER
	#if (!EExER)
		#warning "Building the standard ER firmware."
		//#undef EExER
	#else
		#warning "Building the EExER firmware."
	#endif
#else
	#error "EExER is not defined! Add it to the compiler symbols."
#endif

static void avr_init(void);
static void init_door(const Door *dr);

Door dr1 = {false, false, false, ACTIVE_HIGH, DR1_OUT};
Door dr2 = {false, false, false, ACTIVE_HIGH, DR2_OUT};
Inputs dip = {DEPENDENT, 0x00, false};

int main(void)
{
	avr_init();
	#if (!EExER)
	bool dr1BtnReleased=true, dr2BtnReleased=true; // Not needed for std ER builds.
	#endif

	/**
	**	MAIN LOOP
	**/
	for(;;)
	{

/**---------------------  EExER MAIN LOOP -------------------------------------------------------------------------------------------------------------------------------------- **/

	#if (EExER)
		if( button_is_pressed(&INPUT_PIN_PORT, DR2_BUTTON) || button_is_pressed(&INPUT_PIN_PORT, DR1_BUTTON) )
			dependentRetract();
	#else


/**--------------------- STANDARD ER MAIN LOOP --------------------------------------------------------------------------------------------------------------------------------------**/

		/**	Dependent Mode	**/
		if(dip.mode==DEPENDENT)
		{
			if( button_is_pressed(&INPUT_PIN_PORT, DR2_BUTTON) || button_is_pressed(&INPUT_PIN_PORT, DR1_BUTTON) ) dependentRetract();
		}

		/**	Independent Mode	**/
		else if(dip.mode==INDEPENDENT)
		{
			//	Check the Door 1 button
			if( !dr1.isActive && button_is_pressed(&INPUT_PIN_PORT, DR1_BUTTON) )
			{
				// Toggle if in toggle mode && button has been released
				if( dr1.toggleMode && dr1BtnReleased )
				{
					dr1BtnReleased=false; // Change the previous button state off
					dr1.isToggled = !dr1.isToggled;
					activateDoor1(dr1.isToggled);
				}

				// Activate door if not in toggle mode
				else if(!dr1.toggleMode)
				{
					dr1.isActive = true;
					activateDoor1(true);
				}
			} else dr1BtnReleased=true;


			//	Poll the Door 2 button
			if( button_is_pressed(&INPUT_PIN_PORT, DR2_BUTTON) && !dr2.isActive )
			{
				// Toggle if in toggle mode && button has been released
				if(dr2.toggleMode && dr2BtnReleased)
				{
					dr2BtnReleased=false; // Change the previous button state off
					dr2.isToggled = !dr2.isToggled;
					activateDoor2(dr2.isToggled);
				}

				// Activate door if not in toggle mode
				else if(!dr2.toggleMode)
				{
					dr2.isActive = true;
					activateDoor2(true);
				}
			} else dr2BtnReleased=true;
		}

		//	Check for any changed DIP settings
		if( dip.inputReg != (INPUT_PIN_PORT & INPUT_MASK) )
			readDip();

	#endif /* EExER */

		wdt_reset(); // Reset the watchdog timer
	}
	return 0;
}

/**
 * Initializes the I/O Ports, Configures the Interrupts and Timers
 */
static void avr_init(void)
{
 	OUTPUT_DDR = (bit(DR1_OUT)|bit(DR2_OUT)); // Initialize output port
 	RELAY_DDR |= bit(RELAY1_PIN) | bit(RELAY2_PIN);
 	DS_DDR = ( BIT(DS1_PIN) | BIT(DS2_PIN) );	// PC0 & PC1 set as outputs for DS switches

 	wdt_reset();
 	wdt_enable(WDTO_2S); // Enable Watchdog Timer @ 2 second time-out

 	sbi(PCICR, PCIE2); // Enable Pin Change Interrupt 2
 	PCMSK2 = BIT(PCINT16)|BIT(PCINT17)|BIT(PCINT20)|BIT(PCINT23)|BIT(PCINT22);

 	TCCR0A |= BIT(CS01)|BIT(CS00); // Initialize timer0 with a prescaler of 64
 	sbi(TIMSK0, TOIE0); 	// enable timer 0 overflow interrupt

 	TCCR1B |= (1 << WGM12 ); // Configure timer 1 for CTC mode
 	TIMSK1 |= BIT(OCIE1A); // Enable Output Compare Interrupt Channel A

 	sei(); // Turn on interrupts

 	OCR1A = 1562; // Set CTC compare value to 0.2Hz at 1 MHz AVR clock , with a prescaler of 64
 	TCCR1B |= ((1 << CS10 ) | (1 << CS11 )); // Start timer at Fcpu /64

	#if (!EExER)
 	readDip();
	#endif // EExER
	init_door(&dr1);
	init_door(&dr2);
}

/**
 * \ Initializes the door outputs
 *
 * \param Door Provides a reference to the door
 *
 * \return void
 */
static void init_door(const Door *dr)
{
	bit_write(!(*dr).outputMode, OUTPUT_PORT, bit((*dr).pin));
}

/**
 * \Sets or clears a doors' output
 *
 * \param door The Door to act upon
 * \param activate The value which should be written to the port output
 *
 * \return void
 */
void doorPinWrite(Door door, bool activate)
{
	byte mode, outputMode;
	outputMode = door.outputMode; // Fetch the configured output mode of the door (Active High/Low)
	mode = (activate) ? bit_get(outputMode, 0) : !bit_get(outputMode, 0); // Compare the configured value and passed parameter to obtain a masked prt value
	bit_write(mode, OUTPUT_PORT, bit(door.pin)); // write the masked port value to the outputs
}

/**
 * \Read the DIP switches from the Input Port. Note EExER builds has no need for this method currently.
 *
 * \param void
 *
 * \return void
 */
void readDip(void)
{
	dip.inputReg = INPUT_PIN_PORT & INPUT_MASK; // Read the port
	dip.mode = bit_is_set(dip.inputReg, MODE_PIN) ? DEPENDENT : INDEPENDENT;
	dr1.toggleMode = bit_is_set(dip.inputReg, TOGGLE1_PIN) ? false : true;
	dr2.toggleMode = bit_is_set(dip.inputReg,TOGGLE2_PIN) ? false : true;
	dr1.outputMode = (bit_get(INPUT_PIN_PORT, ACTIVE_MODE1_PIN)) ? ACTIVE_HIGH:ACTIVE_LOW;
	dr2.outputMode = (bit_get(INPUT_PIN_PORT, ACTIVE_MODE2_PIN)) ? ACTIVE_HIGH:ACTIVE_LOW;
}

/**
 * \Method for door 1 independent operation
 *
 * \param activate Should Door 1 be activated? If so, we activate the door output for the amount of time set by the timing selection switch, and also monitor
 *	the other door input in the case that it was pressed while we were inside this timing loop.
 *
 * \return void
 */
void activateDoor1(bool activate)
{
	if(activate==false)
	{
		doorPinWrite(dr1, false);
		cbi(RELAY_PORT, RELAY1_PIN);
	} else {
		doorPinWrite(dr1, true);
		bool dr2BtnPressed = false;

		// reset the timer
		door_timer(true);

		// Loop and Monitor the second input while waiting for the signal delay time to expire
		while( (door_timer(false)) <= SIG_DELAY)
		{
			if(bit_is_clear(INPUT_PIN_PORT, DR2_BUTTON)) dr2BtnPressed = true;
			wdt_reset();
		}

		// Activate relay 1
		sbi(RELAY_PORT, RELAY1_PIN);

		// Activate Door 2 if button was pressed and isn't active
		if(dr2BtnPressed)
		{
			if(dr2.toggleMode)
			{
				bool b = !doorIsActive(dr2);
				activateDoor2(b);
				dr2.isToggled = b;
			} else activateDoor2(dr2.isActive = !dr2.isActive);
		}
	}
	wdt_reset(); // Reset the watchdog timer
}

/**
 * \Method for door 2 independent operation
 *
 * \param activate Should Door 2 be activated? If so, we activate the door output for the amount of time set by the timing selection switch, and also monitor
 *	the other door input in the case that it was pressed while we were inside this timing loop.
 *
 * \return void
 */
void activateDoor2(bool activate)
{
	if(activate==false)
	{
		doorPinWrite(dr2, false);
		cbi(RELAY_PORT,RELAY2_PIN);
	} else {
		// Activate door 2
		doorPinWrite(dr2, true);

		bool btn_pressed = false;

		// reset the timer
		door_timer(true);

		// Monitor input 1 while waiting for the signal delay to expire
		while((door_timer(false)) <= SIG_DELAY)
		{
			if(bit_is_clear(INPUT_PIN_PORT, DR1_BUTTON)) btn_pressed = true;
		}

		// Activate relay 2
		sbi(RELAY_PORT,RELAY2_PIN);

		// Activate door 2 if the button was pressed and isn't active
		if(btn_pressed)
		{
			if(dr1.toggleMode)
			{
				bool b = !doorIsActive(dr1);
				activateDoor1(b);
				dr1.isToggled = b;
			} else activateDoor1(dr1.isActive = !dr1.isActive);
		}
	}
	wdt_reset(); // Reset the watchdog timer
}

/**
 * \Method for handling dependent operation
 *
 * \param void
 *
 * \return void
 */
void dependentRetract(void)
{
	/**	In Toggle Mode **/
	if(dr1.toggleMode)
	{
		if(dr1.isToggled) // Outputs currently toggled, so toggle them off
		{
			doorPinWrite(dr1,false); // Door 1 off
			doorPinWrite(dr2,false); // Door 2 off
			cbi(RELAY_PORT, RELAY1_PIN); // Turn off door signal outputs
			cbi(RELAY_PORT, RELAY2_PIN);
			dr1.isToggled = false;
			dr2.isToggled = false;
		} else /* Doors not toggled, toggle them now */ {
			doorPinWrite(dr1, true); // Door 1 Active
			dr1.isToggled = true;
			_delay_ms(DIFF_DELAY); // Differential Delay
			//wdt_reset(); // Reset the watchdog timer
			doorPinWrite(dr2, true); // Door 2 Active
			dr2.isToggled = true;
			_delay_ms(SIG_DELAY);	// Door Signal Delay
			sbi(RELAY_PORT, RELAY1_PIN);	// Turn on relays now
			sbi(RELAY_PORT, RELAY2_PIN);
		}

		// Maintain while buttons are held
		while(bit_is_clear(INPUT_PIN_PORT, DR1_BUTTON) || bit_is_clear(INPUT_PIN_PORT, DR2_BUTTON))
		{
			wdt_reset();
		}
	} else /** Not in Toggle Mode **/ {
		dip.retracting = true;

		#if (!EExER)
		doorPinWrite(dr1, true); // Door 1 Active
		_delay_ms(DIFF_DELAY); // Differential Delay
		doorPinWrite(dr2, true); // Door 2 Active
		_delay_ms(SIG_DELAY);	// Door Signal Delay
		sbi(RELAY_PORT, RELAY1_PIN);	// Activate door signal outputs
		sbi(RELAY_PORT, RELAY2_PIN);
		#else
		sbi(RELAY_PORT, RELAY1_PIN);	// Activate door signal outputs
		_delay_ms(EExER_DELAY);
		doorPinWrite(dr1, true); // Door 1 Active
		_delay_ms(DIFF_DELAY); // Differential Delay
		doorPinWrite(dr2, true); // Door 2 Active
		_delay_ms(SIG_DELAY);	// Door Signal Delay
		sbi(RELAY_PORT, RELAY2_PIN);
		#endif
		door_timer(true); // Reset the Door timer
		unsigned long t;
		while( door_timer(false) <= ((t=getTime(DR1_OUT))*1000) ) // Hold Time Delay, value set from DS1
		{
			wdt_reset(); // Reset the watchdog timer
			if( dr1.toggleMode==true )
				break;
			if( bit_is_clear(INPUT_PIN_PORT, DR1_BUTTON) || bit_is_clear(INPUT_PIN_PORT, DR2_BUTTON) )	// Maintain active outputs when button is held
				door_timer(true); // reset the door timer
			if(dip.mode==INDEPENDENT || dr1.toggleMode==true) /* If output or toggle mode changed, break loop */
				break;
		}
		doorPinWrite(dr1, false); // Door outputs off
		doorPinWrite(dr2, false);
		cbi(RELAY_PORT, RELAY1_PIN);	// Deactivate door signal outputs
		cbi(RELAY_PORT, RELAY2_PIN);
		dip.retracting = false;
	}
}

/**
 * \Checks and debounces the door input buttons
 *
 * \param PIN The pin to check. Input pins are pullup enabled.
 * \param BUTTON_BIT A port mask to isolate the pin to be checked.
 *
 * \return bool True if button pressed, false otherwise.
 */
bool button_is_pressed(volatile ui8 *PIN, ui8 BUTTON_BIT)
{
	if ( bit_is_clear(*PIN, BUTTON_BIT) )
	{
		_delay_ms(DEBOUNCE_TIME);
		if (bit_is_clear(*PIN, BUTTON_BIT) ) return true;
	}
	return false;
}

/**
 * \ Reads a door port value and compares it with the door output mode
 *
 * \param door The door to reference
 *
 * \return boolean Returns the boolean comparison of the doors' configured settings and the actual port value.
 */
bool doorIsActive(Door door)
{
	//Door _door = *door;
	// Read the port
	uint8_t portValue = 0x01 & (DOOR_PIN_PORT>>door.pin);
	return (portValue == door.outputMode);
}
