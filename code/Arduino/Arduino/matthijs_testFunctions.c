/*
 * matthijs_USART.c
 *
 * Created: 3/14/2018 10:11:11 PM
 *  Author: mc_he
 */ 
#include "matthijs_testFunctions.h"
#include <avr/io.h>
#include <stdlib.h>

void writeChar(char x) { //Sends char x over serial communication
	while(~UCSR0A & (1 << UDRE0)); //Wait until UDRE0 is set
	UDR0 = x;
}

void writeString(char st[]) { //Sends char array over serial communication. Dependent on writeChar()
	for(uint8_t i = 0 ; st[i] != 0 ; i++) {
		writeChar( st[i] );
	}
}

void writeInt(int i) { //Sends integer i over serial communication. Dependent on writeString()
	char buffer[8];
	itoa(i,buffer,10); //Converts i to a string
	writeString(buffer);
}

void testTransmitUSART(char charToSend, int intToSend) {	//Transmits charToSend and intToSend over USART, separated by a space and followed by CR LF
	
	writeChar(charToSend);
	writeChar(' ');
	writeInt(intToSend);
	writeString("\r\n");
}