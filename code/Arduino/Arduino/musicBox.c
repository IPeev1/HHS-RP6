/*
 * musicBox.c
 *
 * Created: 3/27/2018 1:04:26 AM
 * Author: Matthijs Heidema
 * Studentnr: 17117100
 */ 
#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include "musicBox.h"

#define PRESCALER 256

void initBackBeep() {
	
	TCCR4A |= (1 << COM4A0);				//Toggle OC1A on match
	TCCR4B |= (1 << CS42) | (1 << WGM42);	//Prescaler 256, CTC mode
	OCR4A = freqToOCR(BEEP_FREQ);
}

int freqToOCR(float frequency) { //Converts a frequency in Hz to a OCR-compatible value
	
	return F_CPU / (2 * PRESCALER * frequency);
}