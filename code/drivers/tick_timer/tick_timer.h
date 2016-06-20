/*
 * tick_timer.h
 *
 * Created: 3/30/2016 1:40:30 PM
 *  Author: psammand
 */ 


#ifndef TICK_TIMER_H_
#define TICK_TIMER_H_

#include <avr/io.h>

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

void timer_init();
void timer_delay_us(uint16_t delay);
void timer_delay_ms(uint32_t delay);
uint32_t timer_micros();
uint32_t timer_millis();

#endif /* TICK_TIMER_H_ */