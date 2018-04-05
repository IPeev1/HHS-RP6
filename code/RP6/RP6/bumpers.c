/*
 * bumpers.c
 *
 * Created: 3/28/2018 3:10:43 PM
 *  Author: mc_he
 */ 
#include <avr/io.h>
#include <util/delay.h>
#include "bumpers.h"

#define PORT_L PORTB
#define PIN_L PINB
#define DDR_L DDRB
#define IO_L PINB0

#define PORT_R PORTC
#define PIN_R PINC
#define DDR_R DDRC
#define IO_R PINC6

#define DISTANCE 0

int getBumperLeft() {
	
	PORT_L &= ~(1 << IO_L);
	DDR_L &= ~(1 << IO_L);
	_delay_us(2);
	
	uint8_t pushed = PIN_L & (1 << IO_L);
	
	DDR_L |= (1 << IO_L);
	PORT_L |= (1 <<  IO_L);
	
	return pushed;
}

int getBumperRight() {
	
	PORT_R &= ~(1 << IO_R);
	DDR_R &= ~(1 << IO_R);
	_delay_us(2);
	
	uint8_t pushed = PIN_R & (1 << IO_R);
	
	DDR_R |= (1 << IO_R);
	PORT_R |= (1 <<  IO_R);
	
	return pushed;
}

int getBumpers() {
	
	return getBumperLeft() || getBumperRight();
}