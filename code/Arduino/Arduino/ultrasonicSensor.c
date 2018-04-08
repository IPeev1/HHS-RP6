/*
 * ultrasonicSensor.c
 *
 * Created: 3/21/2018 11:28:36 PM
 *  Author: mc_he
 */ 
#define F_CPU 16000000UL

#include "ultrasonicSensor.h"
#include "USART.h"
#include <avr/io.h>
#include <util/delay.h>

#define SPEED_OF_SOUND 1715
#define CORRECTION 5;					//Compensates for the offset of the sensors

#define TRIGGER PINL1					//The pin that is connected to the trigger pin of the sensor
#define ECHO PINL1						//The pin that is connected to the echo pin of the sensor
#define DDR_REGISTER DDRL				//The data direction register of the TRIGGER and ECHO pins
#define PORT_REGISTER PORTL				//PORT register of the TRIGGER and ECHO pins
#define PIN_REGISTER PINL				//PIN register of the TRIGGER and ECHO pins

void initTimer(){
	
	TCCR1B |= (1 << CS11);				//Prescaler 8
	TCNT1 = 0;
}

int ultrasonicSensor() {
	
	TCNT1 = 0;							//Reset value of TCNT1
	
	unsigned long pulseStartTime = 0;	//Used to record value of TCNT1 when the pulse starts
	
	//Values to prevent infinite loops:
	unsigned long numLoops = 0;
	unsigned long maxLoops = 40000;
	
	DDR_REGISTER |= (1 << TRIGGER);			//Set TRIGGER pin as output
	
	PORT_REGISTER &= ~(1 << TRIGGER);		//Set TRIGGER pin to low for 2 us to ensure a clean pulse
	_delay_us(2);
	PORT_REGISTER |= (1 << TRIGGER);		//Send pulse for 5 us
	_delay_us(5);
	PORT_REGISTER &= ~(1 << TRIGGER);		//Set TRIGGER pin to low
	
	DDR_REGISTER &= ~(1 << ECHO);			//Set ECHO pin as input

	while (PIN_REGISTER & (1 << ECHO)) {	//Wait for any old pulse to end
		if(numLoops++ == maxLoops) {
			return 0;
		}
	}
	
	while (~PIN_REGISTER & (1 << ECHO)) {		//Wait until PING))) returns a pulse
		if(numLoops++ == maxLoops) {
			return 0;
		}
	}
	
	pulseStartTime = TCNT1;						//Set pulseStartTime to current TCNT1 value
	
	while (PIN_REGISTER & (1 << ECHO)) {		//Wait until the pulse from PING))) ends
		if(numLoops++ == maxLoops) {
			return 0;
		}
	}
	return cyclesToMm(TCNT1 - pulseStartTime);	//Calculate and return distance in mm
}

int cyclesToMm(unsigned long cycles) {
	
	return ((cycles * SPEED_OF_SOUND) / 20000) - CORRECTION;
}

void printUltrasonicSensorDistance() { //Prints the measured distance in centimeters
	int distance = 0;
	int distanceCm = 0;
	int distance10thCm = 0;
	
	distance = ultrasonicSensor();
	distanceCm = distance / 10;
	distance10thCm = distance - (distanceCm * 10);
	writeString("Distance: ");
	writeInt(distanceCm);
	writeChar('.');
	writeInt(distance10thCm);
	writeString("\r\n");
}