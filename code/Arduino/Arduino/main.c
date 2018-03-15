/*
 * Arduino1280 XBEE Communication
 *
 * Created: 3/13/2018 2:02:25 PM
 * Author : mc_he
 */ 
#define F_CPU 16000000UL														//Clock speed

//Libraries
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include "matthijs_testFunctions.h" /*Contains functions for transmitting over USART for testing purposes*/

//Definitions
#define BUFFER_SIZE 255
/*#define USART_INTERRUPT_VECTOR USART_RX_vect*/								//Edit this line in when using Atmega168PB
#ifndef USART_INTERRUPT_VECTOR
#define USART_INTERRUPT_VECTOR USART0_RX_vect
#endif

// Global variables
int8_t globalDriveDirection;		// Value -1, 0 or 1
int8_t globalTurnDirection;			// Value -1, 0 or 1
int8_t globalDriveThrottle;			//value between 0 - 100

ISR(USART_INTERRUPT_VECTOR) {
	static char buffer[BUFFER_SIZE];											//Character buffer to store numerals
	static int bufferPos = -1;													//Represents which buffer positions are currently in use to store numerals
	static char received = 0;													//Stores the last character received through USART
	static char command = 0;													//Stores a character that represents a command. Default value is null
	
	received = UDR0;
	
	if ('0' <= received && received <= '9') {									//If received contains a a numeral
		
		if (command == 't') {													//If command 't' is currently set
			if (bufferPos < BUFFER_SIZE)										//Check to prevent overflow of the buffer
				buffer[++bufferPos] = received;									//Add numeral to buffer
		}
			
	} else if ('a' <= received && received <= 'z') {							//If received contains a (lower case) letter
		
		switch(received) {														//Each valid command is represented by a case
			
			case 'w':
			case 'a':
			case 's':
			case 'd':
			case 't':
			command = received;
		}
	} else if (received == '\r') {												//If received contains a carriage return
		
		uint16_t intValue = 0;													//Value to be passed over I2C with the command. Default value is 0.
		
		if (command == 't') {													//If the command is 't', the buffer is converted to an integer and stored in intValue
			uint8_t charToInt;
		
			for (uint8_t i = 0; i <= bufferPos; i++) {
				charToInt = (int) (buffer[i] - '0');
				intValue += charToInt * ((int)(pow(10, bufferPos - i) + 0.5));	//The 0.5 is necessary to properly convert the return value of pow() into an integer
			}
			
		}
		if (command) {															//Only if a command is set is data transmitted
			
			switch (command) {
				
				case 'w':
				if (globalDriveDirection == 1) {
					globalDriveDirection = 0;
				} else {
					globalDriveDirection = 1;
				}
				break;
				
				case 'a':
				if (globalTurnDirection == -1) {
					globalTurnDirection = 0;
					} else {
					globalTurnDirection = -1;
				}
				break;
				
				case 's':
				if (globalDriveDirection == -1) {
					globalDriveDirection = 0;
				} else {
					globalDriveDirection = -1;
				}
				break;
				
				case 'd':
				if (globalTurnDirection == 1) {
					globalTurnDirection = 0;
					} else {
					globalTurnDirection = 1;
				}
				break;
				
				case 't':
				if (intValue <= 100) {
					globalDriveThrottle = intValue;
				}
				break;
			}
		
			command = 0;														//Reset command
			bufferPos = -1;													//Reset buffer position
			
			globalVariablesTransmitUSART(globalDriveDirection, globalTurnDirection, globalDriveThrottle);
		}
	}
}

int main(void)
{	
	//USART initialization
	UCSR0A = 0x00;								
	UCSR0B |= (1 << RXCIE0 | 1 << RXEN0);		//Enable USART receiver, receiver interrupt
	UCSR0B |= 1 << TXEN0;	/*Transmitter enabled for testing*/
	UCSR0C |= (1 << UCSZ01 | 1 << UCSZ00);		//Asynchronous USART, Parity none, 1 Stop bit, 8-bit character size
	UBRR0H = 00;
	UBRR0L = 103;								//Baudrate 9600
	
	sei();										//Enable interrupt routines
	
    while (1) 
    {
    }
}