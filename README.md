# PS8x FIRMWARE

<<<<<<< HEAD
=======
1/27/2017: TODO - modify the preprocessor code to keeep the interrupt code and instean use the preprocessor to turn off the pin change interrupts on certain pins. 

>>>>>>> origin/master
REVISION HISTORY:
	8/21/2014: Release candidate version 5.1
	9/13/2015: Rev 5.2 to fix a bug in the timing code found from DVPR1 testing

Compilation Info: 
	Define a symbol named EExER with a value of 1 to compile the code for the EExER build version. Set the value to 0 to build the standard ER build version.

Build Versions:
	EExER - This build essentially removes most of the features found in the ER code as it is not required. More specifically, no DIP setting features and trigger Relay1 before triggering the rest of the outputs.
	ER - This version is compiled to be the standard ER firmware with the ability to use the entire feature set (e.g. the DIP switch settings). It allows for Dependent/Independent operation on dual doors, Active High/Low outputs, and Toggling features. The DIP switches are interrupt driven so changes are implemented immediately and the system auto adjusts depending on its current state. The order of events upon a trigger event is as follows: 

Mode Definitions:
	Dependent Mode: (INPUT1 | INPUT2) => (DOOR1 ACTIVE)->(DELAY 0.5s)->(DOOR2 ACTIVE)->(DELAY 0.5s)->(RELAY1 & RELAY2)->(DELAY DS1)->(RETURN)
	Independent Mode: (INPUTn) => (DOORn ACTIVE)->(DELAY 0.5s)->(RELAYn)->(DELAY DSn)->(RETURN)
	Operation in independent mode also requires the monitoring of the opposing door input during the delay time so that a triggering event is not missed. This is accomplished by controlling the timing and outputs outputs within the Timer1 Compare Interrupt Service Routine Vector (see interrupts.c).

EExER - This version is compiled for compatibility with the current delayed egress used in the 85-800. The pre-compiler bypasses all the DIP switch features as the delayed egress board controls the outputs. This version does not populate the DIP switch (DIP1) or Door 2 rotary encoder (DS2) as neither of these features will be used in this configuration. The order of events when a trigger event occurs is as follows: (INPUT1 | INPUT2) => (RELAY1)->(DELAY 0.5s)->(DOOR1 ACTIVE)->(DELAY 0.5s)->(DOOR2 ACTIVE)->(DELAY 0.5s)->(RELAY2)->(DELAY DS1)->(RETURN).

Device: 
	Attiny88-AU MCU, TQFP 32A Package. 
Datasheets: 
	Complete http://www.atmel.com/Images/doc8008.pdf
	Summary http://www.atmel.com/Images/8008S.pdf

AVR Toolchain Source Download: 
	http://distribute.atmel.no/tools/opensource/Atmel-AVR-Toolchain-3.4.2/

Fuse Settings: 
	SPI Enabled, WD Timer Enabled, System Clock Divided by 8, Brownout Detection @ 2.7V, Internal Oscillator set to 8MHz with 64ms Start Delay.
	- Extended = 0xFF
	- High = 0xCD
	- Low = 0x6E

TODO: 
	Implement self testing features or write separate code for system testing for quality assurance. 

*This software was developed using Atmel Studio 7.0 IDE and compiled with the bundled AVR toolchain.
