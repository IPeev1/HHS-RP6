/*
 * ultrasonicSensor.c
 *
 * Created: 3/21/2018 11:28:36 PM
 *  Author: mc_he
 */ 
#define F_CPU 16000000UL

#include "ultrasonicSensor.h"
#include "matthijs_testFunctions.h"
#include <avr/io.h>
#include <util/delay.h>

#define speedOfSound 1715
#define TRIGGER PINB1
#define ECHO PINB4
#define MAX_SENSOR_VALUE 30000

void initTimer(){
	
	TCCR1B |= (1 << CS11); //prescaler 8
	TCNT1 = 0;
}

int ultrasonicSensor() {
	
	TCNT1 = 0;
	
	unsigned long pulseStartTime = 0;
	unsigned long numLoops = 0;
	unsigned long maxLoops = 16000;
	
	DDRB |= (1 << TRIGGER);
	
	
	PORTB &= ~(1 << TRIGGER);
	_delay_us(2);
	PORTB |= (1 << TRIGGER);
	_delay_us(5);
	PORTB &= ~(1 << TRIGGER);
	
	DDRB &= ~(1 << ECHO);

	while (PINB & (1 << ECHO)) {
		if(numLoops++ == maxLoops) {
			return MAX_SENSOR_VALUE;
		}
	}
	
	while (~PINB & (1 << ECHO)) {
		if(numLoops++ == maxLoops) {
			return MAX_SENSOR_VALUE;
		}
	}
	
	pulseStartTime = TCNT1;
	
	while (PINB & (1 << ECHO)) {
		if(numLoops++ == maxLoops) {
			return MAX_SENSOR_VALUE;
		}
	}
	return cyclesToMm(TCNT1 - pulseStartTime);
}

int cyclesToMm(unsigned long cycles) {
	
	return (cycles * speedOfSound) / 2000;
}

void printUltrasonicSensorDistance() {
	int distance = 0;
	int distanceCm = 0;
	int distance100thCm = 0;
	
	distance = ultrasonicSensor();
	distanceCm = distance / 100;
	distance100thCm = distance - (distanceCm * 100);
	writeString("Distance: ");
	writeInt(distanceCm);
	writeChar('.');
	writeInt(distance100thCm);
	writeString("\r\n");
}