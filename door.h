/*
 * door.h -> Door type definitions
 *
 * Created: 7/2/2014
 * Modified: 8/20/2014
 * Author: DMH
 *
 */

#ifndef DOOR_H_
#define DOOR_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	volatile bool toggleMode;
	volatile bool isToggled, isActive;
	volatile uint8_t outputMode;
	const uint8_t pin;
} Door;

#endif /* DOOR_H_ */