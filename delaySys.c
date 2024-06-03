/*
 * delaySys.c
 */

#include <stdint.h>
#include "delaySys.h"


volatile uint32_t millisec; //Tick Timer

void delaySys(uint32_t delay_ms) // assumes 1 ms tick
{
    uint32_t start = millisec;

    while (millisec - start < delay_ms) ; // wait
}

// Blinking LED without blocking
// return HIGH or LOW for time millisececonds
int led_blink(unsigned long time)
{
    return (millisec % (2*time)) > time;
}
