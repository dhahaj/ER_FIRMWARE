/*
*  timers.h
*
*  Created: 7/25/2012 2:07:38 PM
*  Modified: 7/30/2015 12:29 PM*  Author: DMH
*  Author: DMH
*/

#ifndef TIMERS_H_
#define TIMERS_H_

#include <avr/interrupt.h>

// Fetches a timing value from the timing values array
#define getTimingValue(P) ( pgm_read_byte( timing_values_PGM + (P) ) )

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#define nop __asm__("nop\n\t");
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)
#define bit_set(p,m1) ((p) |= (m1))
#define bit_clear(p,m1) ((p) &= ~(m1))
#define bit_write(c,p,m1) (c ? bit_set(p,m1) : bit_clear(p,m1))
#define BIT(x) (0x01 << (x))
//#define bit(x) (BIT(x))
#define bit_get(p,m1) ((p) & (BIT(m1)))

volatile unsigned long timer0_millis;
volatile uint8_t timer0_fract;

unsigned long getTime(uint8_t pin);

#endif /* TIMERS_H_ */
