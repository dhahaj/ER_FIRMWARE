/*
*  Revision 5.2
*  main.c -> Main code for the ER Controller, with optional EExER builds.
*
*  Created: 1/23/2013 11:51 AM
*  Modified: 4/1/2016 8:27 AM
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
#include "door.h"

// Door Structs 
dr1 = Door(DR1_OUT, RELAY1_PIN);
dr2 = Door(DR2_OUT, RELAY2_PIN);

/*
*	Determine if EExER is defined to enable it by the preprocessor and display a confirmation message.
*/
#ifdef EExER
#if (!EExER)
  #pragma message("Building the standard ER firmware.")
#else
  #pragma message("Building the EExER firmware.")
#endif
#else
  #error "EExER is not defined! Add it to the compiler symbols."
#endif

// Function Declarations
static void avr_init(void);
void init_door(Door *dr);
dr1 = {false, false, false, ACTIVE_HIGH, DR1_OUT};
dr2 = {false, false, false, ACTIVE_HIGH, DR2_OUT};
Inputs dip_switch = {DEPENDENT, 0x00, false};

int main(void)
{

	avr_init();

#if (!EExER)
	volatile bool dr1BtnReleased=true, dr2BtnReleased=true; // Not needed for std ER builds.
#endif

	/**
	**	MAIN LOOP
	**/
	while(1)
	{

  #if (EExER) /* EExER  */
		if( buttonPressed(&INPUT_PIN_PORT, DR2_BUTTON) || buttonPressed(&INPUT_PIN_PORT, DR1_BUTTON) ) dependentRetract();
  #else /* ER */

		/**********************/
		/**	DEPENDENT MODE	**/
		/**********************/
		if(dip_switch.mode==DEPENDENT) {
			if( buttonPressed(&INPUT_PIN_PORT, DR2_BUTTON) || buttonPressed(&INPUT_PIN_PORT, DR1_BUTTON))
			dependentRetract();
		}

		/**********************/
		/**	INDEPENDENT MODE **/
		/**********************/
		else
		{
			/**** DOOR 1 ****/
			if( buttonPressed(&INPUT_PIN_PORT, DR1_BUTTON) && !dr1.is_active ) /*	Monitor the Door 1 input */
			{
				if( dr1.toggle_mode && dr1BtnReleased) /* Toggle if in toggle mode && button has been released */
				{
					dr1BtnReleased = false; /* Change the previous button state off */
					dr1.toggled = !dr1.toggled;
					activateDoor1(dr1.toggled);
				}

				else if(!dr1.is_active && !dr1.toggle_mode) /* Activate door if not in toggle mode */
				{
					dr1.is_active = true;
					activateDoor1(true);
				}
			} else dr1BtnReleased=true;

			/**** DOOR 2 ****/
			if( buttonPressed(&INPUT_PIN_PORT, DR2_BUTTON) && !dr2.is_active) /* Monitor the Door 2 input */
			{
				if(dr2.toggle_mode && dr2BtnReleased) /* Toggle if in toggle mode && button has been released */
				{
					dr2BtnReleased = false; /* Change the previous button state off */
					dr2.toggled = !dr2.toggled;
					activateDoor2(dr2.toggled);
				}
				/* Activate door if not in toggle mode */
				else if(dr2BtnReleased)
				{
					dr2.is_active = true;
					activateDoor2(true);
				}
			} else dr2BtnReleased=true;
		}

		/* Check for any changed DIP settings */
		if( dip_switch.prev_values != (INPUT_PIN_PORT & INPUT_MASK) )
		readDip();
		
#endif /* EExER */
		wdt_reset();
	}

	return 0;

} // END MAIN

/**
 *  \brief avr_init
 *  
 *  \details Initializes the I/O Ports, Configures the Interrupts and Timers
 */
static void avr_init(void) 
{
	//	OUTPUT_DDR = (BIT(DR1_OUT)|BIT(DR2_OUT)); // Initialize output port
	//	RELAY_DDR |= BIT(RELAY1_PIN) | BIT(RELAY2_PIN);
	DS_DDR = ( BIT(DS1_PIN) | BIT(DS2_PIN) );	// PC0 & PC1 set as outputs for DS switches

	wdt_reset(); // Reset watchdog for safety.
	wdt_enable(WDTO_2S); // Enable Watchdog Timer with a 2 second time-out

	sbi(PCICR, PCIE2); // Enable Pin Change Interrupt 2
	PCMSK2 = BIT(PCINT16)|BIT(PCINT17)|BIT(PCINT20)|BIT(PCINT23)|BIT(PCINT22);

	TCCR0A |= BIT(CS01)|BIT(CS00); // Initialize timer0 with a prescaler of 64
	sbi(TIMSK0, TOIE0); 	// enable timer 0 overflow interrupt

	TCCR1B |= (1 << WGM12 ); // Configure timer 1 for CTC mode
	TIMSK1 |= BIT(OCIE1A); // Enable Output Compare Interrupt Channel A

	sei(); // Enable interrupts

	OCR1A = 1562; // Set CTC compare value to 0.2Hz at 1 MHz AVR clock , with a prescaler of 64
	TCCR1B |= ((1 << CS10 ) | (1 << CS11 )); // Start timer at Fcpu /64

#if (!EExER)
	readDip();
#endif // EExER
	
  // init_door(&dr1);
	// init_door(&dr2);
}

/**
* \ Initializes the door outputs
*
* \param Door: Pointer to the door object
*
* \return void
*//*
void init_door(const Door *dr)
{
bit_write(!(*dr).active_high, OUTPUT_PORT, BIT((*dr).pin));
}
*/

/**
 *  \brief doorPinWrite
 *  
 *  \param [in] door     A pointer variable to the door object
 *  \param [in] activate The value which should be written to the port output.
 *  
 *  \details Sets or clears a doors' output
 */
void doorPinWrite(const Door *door, bool activate)
{
	uint8_t mode, outputMode;
	outputMode = (*door).isActive(); // Fetch the configured output mode of the door (Active High/Low)
	mode = (activate) ? bit_get(outputMode, 0) : !bit_get(outputMode, 0); // Compare the configured value and passed parameter to obtain a masked port value
	bit_write(mode, OUTPUT_PORT, BIT((*door).pin)); // write the masked port value to the outputs
}

/**
 *  \brief readDip
 *  
 *  \details Read the DIP switches from the Input Port. Note EExER builds has no need for this method currently.
 */
void readDip(void)
{
  // Read the value of the port masked with the input pins.
	dip_switch.prev_values = INPUT_PIN_PORT & INPUT_MASK;
  
  // Run through each of the bits to apply the DIP Switch configuration.
	dip_switch.mode = bit_is_set(dip_switch.prev_values, MODE_PIN) ? DEPENDENT : INDEPENDENT;
	dr1.setToggled(bit_is_set(dip_switch.prev_values, TOGGLE1_PIN) ? false : true);
	dr2.setToggled( bit_is_set(dip_switch.prev_values,TOGGLE2_PIN) ? false : true );
	dr1->active_mode = (bit_get(INPUT_PIN_PORT, ACTIVE_MODE1_PIN)) ? ACTIVE_HIGH : ACTIVE_LOW;
	dr2.active_mode = (bit_get(INPUT_PIN_PORT, ACTIVE_MODE2_PIN)) ? ACTIVE_HIGH : ACTIVE_LOW;
}

/**
* \brief activateDoor1: Function for activating DR1 independently of DR2.
*
* \param activate: Flag to activate the output. If true, we activate the door output for the amount 
*                  of time set by the rotary switch. The other door input must also be monitored to 
*                  avoid the possibility of missing an input trigger while inside the timed loop.
*
*/
void activateDoor1(bool activate)
{
  // Turn door 1 output OFF
	if(!activate)
	{
		doorPinWrite(&dr1, false);
		cbi(RELAY_PORT, RELAY1_PIN);
	}

  // Turn door 1 output OFF
  else
  {
    doorPinWrite(&dr1, true);
    bool dr2BtnPressed = false;
    bool btn_toggled = bit_is_clear(INPUT_PIN_PORT, DR2_BUTTON); /* This variable is used to avoid toggling the door2 output if the button is being held */
    
    // Reseet the timer.
    door_timer(true);
    
    while( (door_timer(false) ) <= SIG_DELAY)
    {
      // Monitor DR2 input while inside this loop to avoid not catching an input event 
      // on the second door.
      if( !dr2->isActive() && !dr2BtnPressed && bit_is_clear(INPUT_PIN_PORT, DR2_BUTTON) )
        dr2BtnPressed = true;
      wdt_reset();
    }
    
    // Activate RLY1
    sbi(RELAY_PORT, RELAY1_PIN);
    
    // See if the DR2 button was pressed while inside the loop above.
    if(dr2BtnPressed && !btn_toggled) 
    {
      // Determine if DR2 is running in toggle mode 
      // and if it's currently active.
      if(dr2->isToggled() && !dr2.isToggled())
      {
        // Read the pin state
        bool b = !doorActive(&dr2);
        activateDoor2(b);
        dr2.setToggled(b);
      } 
      
      // Otherwise see if the door is not active.
      else if(!dr2.isToggled()) 
      {
        bool active = dr2.is_active;
        if(!active)
          activateDoor2(dr2.is_active);
      }
    }

  } // End Else
  wdt_reset();
} // End activateDoor1

/**
* \Method activateDoor2: Function for activating DR2 independently of DR1.
*
* \param activate: Flag to activate the output. If true, we activate the door output for the amount 
*                  of time set by the rotary switch. The other door input must also be monitored to 
*                  avoid the possibility of missing an input trigger while inside the timed loop.
*
*/
void activateDoor2(bool activate)
{
  // Turn OFF DR2 output
  if(activate==false)
  {
    doorPinWrite(&dr2, false);
    cbi(RELAY_PORT, RELAY2_PIN);
  }
  
  // Turn ON DR2 output.
  else
  {
    doorPinWrite(&dr2, true);
    bool btn_pressed = false;

    // This variable is used to avoid toggling DR1 output if the button is being held.
    bool btn_toggled = bit_is_clear(INPUT_PIN_PORT, DR1_BUTTON);

    // reset the timer.
    door_timer(true);

    // Monitor input 1 while waiting for the signal delay to expire.
    while((door_timer(false)) <= SIG_DELAY) {
      if(bit_is_clear(INPUT_PIN_PORT, DR1_BUTTON) && !dr1.isActive() && !btn_pressed) {
        btn_pressed = true;
      }
      wdt_reset();
    }

    // Activate RLY2.
    sbi(RELAY_PORT,RELAY2_PIN);

    // Activate DR2 if the button was pressed and the outputs are not already active.
    if(btn_pressed && !btn_toggled)
    {
      if(dr1.toggle_mode && !dr1.MyMethod()())
      {
        bool active = !doorActive(&dr1);
        activateDoor1(active);
        dr1.toggled = active;
      }

      // Not in toggle mode.
      else if(!dr1.toggle_mode)
      {
        bool active = !dr1.is_active;
        dr1.is_active = active;
        activateDoor1(active);
      }
    }
    
  }// END else

  wdt_reset();
} // END activateDoor2

/**
* \brief dependentRetract 
*
* \details Hhandles dependent operation.
*
*/
void dependentRetract(void)
{
  /**	In Toggle Mode **/
  if(dr1.isToggled())
  {
    if(dr1.isToggled()) // Outputs currently toggled, so toggle them off
    {
      doorPinWrite(&dr1,false); // Door 1 off
      doorPinWrite(&dr2,false); // Door 2 off
      cbi(RELAY_PORT, RELAY1_PIN); // Turn off door signal outputs
      cbi(RELAY_PORT, RELAY2_PIN);
      dr1.toggled = false;
      dr2.toggled = false;
      } else /* Doors not toggled, toggle them now */ {
      doorPinWrite(&dr1, true); // Door 1 Active
      dr1.setToggled(true);
      _delay_ms(DIFF_DELAY); // Differential Delay
      //wdt_reset(); // Reset the watchdog timer
      doorPinWrite(&dr2, true); // Door 2 Active
      dr2.toggled = true;
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
    dip_switch.retracting = true;

#if (!EExER)
    doorPinWrite(&dr1, true); // Door 1 Active
    _delay_ms(DIFF_DELAY); // Differential Delay
    doorPinWrite(&dr2, true); // Door 2 Active
    _delay_ms(SIG_DELAY);	// Door Signal Delay
    sbi(RELAY_PORT, RELAY1_PIN);	// Activate door signal outputs
    sbi(RELAY_PORT, RELAY2_PIN);
#else
    sbi(RELAY_PORT, RELAY1_PIN);	// Activate door signal outputs
    _delay_ms(EExER_DELAY);
    doorPinWrite(&dr1, true); // Door 1 Active
    _delay_ms(DIFF_DELAY); // Differential Delay
    doorPinWrite(&dr2, true); // Door 2 Active
    _delay_ms(SIG_DELAY);	// Door Signal Delay
    sbi(RELAY_PORT, RELAY2_PIN);
#endif
    door_timer(true); // Reset the Door timer
    unsigned long t;
    while( door_timer(false) <= ((t=getTime(DR1_OUT))*1000) ) // Hold Time Delay, value set from DS1
    {
      wdt_reset(); // Reset the watchdog timer
      if( dr1.toggle_mode==true )
        break;
      if( bit_is_clear(INPUT_PIN_PORT, DR1_BUTTON) || bit_is_clear(INPUT_PIN_PORT, DR2_BUTTON) )	// Maintain active outputs when button is held
        door_timer(true); // reset the door timer
      if(dip_switch.mode==INDEPENDENT || dr1.setToggled(true)) /* If output or toggle mode changed, break loop */
        break;
    }
    doorPinWrite(&dr1, false); // Door outputs off
    doorPinWrite(&dr2, false);
    cbi(RELAY_PORT, RELAY1_PIN);	// Deactivate door signal outputs
    cbi(RELAY_PORT, RELAY2_PIN);
    dip_switch.retracting = false;
  }
}

/**
* \brief buttonPressed
*
* \param PIN: A pointer variable for the PINPORT
* \param BUTTON_BIT: The bit on the PINPORT to debouce.
*
* \return bool True if button pressed, false otherwise.
*
* \details Debounces a button input
*/
bool buttonPressed(volatile ui8 *PIN, ui8 BUTTON_BIT)
{
  if ( bit_is_clear(*PIN, BUTTON_BIT) )
  {
    _delay_ms(DEBOUNCE_TIME);
    if (bit_is_clear(*PIN, BUTTON_BIT) )
      return true;
  }
  return false;
}

/**
* \brief doorActive
*
* \param door: A pointer to the door object.
*
* \return boolean Returns the boolean comparison of the doors' configured settings and the actual port value.
*
* \details Reads a door port value and compares it with the door output mode.
*/
bool doorActive(const Door *door)
{
  // Read the port
  uint8_t portValue = 0x01 & (DOOR_PIN_PORT>>(*door).pin);
  // Compare the port value to the door state.
  bool result = (bool)(portValue == (*door).isActive());
  return result;
}
