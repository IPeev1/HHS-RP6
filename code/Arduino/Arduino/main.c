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
#include "i2c_mst.c"
#include "matthijs_testFunctions.h" /*Contains functions for transmitting over USART for testing purposes*/

//Definitions
#define BUFFER_SIZE 255
/*#define USART_INTERRUPT_VECTOR USART_RX_vect*/								//Edit this line in when using Atmega168PB
#ifndef USART_INTERRUPT_VECTOR
#define USART_INTERRUPT_VECTOR USART0_RX_vect
#endif

//I2C Functions -----------------------------------
void I2C_sendConstructor(uint8_t I2Cdata[]);
void init_rp6Data();
void rp6DataConstructor();
//-------------------------------------------------

// Global variables
struct rp6DataBP {
	int8_t		driveSpeed;				//value between 0 - 100
	int8_t		driveDirection;			//Value 0 or 1
	int8_t		turnDirection;			//Value -1, 0 or 1
	uint8_t		accelerationRate;		//Percentage to accelerate with					(Default: 30)
	uint16_t	turnRate;				//Intensity to turn with						(Default: 3000)
	uint16_t	driveSpeedThreshold;	//Minimal power needed to actually start moving	(Default: 5000)
	uint32_t	updateSpeed;			//Interval time between updates					(Default: 200000)
	uint8_t		enableBeeper;			//Set to 1 to enable reverse driving beeper		(Default: 1	(On))
	uint16_t	motorEncoderLVal;		//Segment count of the left motor encoder		(Updated by interrupt)
	uint16_t	motorEncoderRVal;		//Same for right encoder ---^
} rp6Data;


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
				if (rp6Data.driveDirection == 1) {
					rp6Data.driveDirection = 0;
				} else {
					rp6Data.driveDirection = 1;
				}
				break;
				
				case 'a':
				if (rp6Data.turnDirection == -1) {
					rp6Data.turnDirection = 0;
					} else {
					rp6Data.turnDirection = -1;
				}
				break;
				
				case 's':
				if (rp6Data.driveDirection == -1) {
					rp6Data.driveDirection = 0;
				} else {
					rp6Data.driveDirection = -1;
				}
				break;
				
				case 'd':
				if (rp6Data.turnDirection == 1) {
					rp6Data.turnDirection = 0;
					} else {
					rp6Data.turnDirection = 1;
				}
				break;
				
				case 't':
				if (intValue <= 100) {
					rp6Data.driveSpeed = intValue;
				}
				break;
			}
		
			command = 0;														//Reset command
			bufferPos = -1;													//Reset buffer position
			
			globalVariablesTransmitUSART(rp6Data.driveDirection, rp6Data.turnDirection, rp6Data.driveSpeed);
		}
	}
}

int main(void)
{	
	//USART initialization
	UCSR0A = 0x00;								
	UCSR0B |= (1 << RXCIE0 | 1 << RXEN0);		//Enable USART receiver, receiver interrupt
	UCSR0B |= 1 << TXEN0;						/*Transmitter enabled for testing*/
	UCSR0C |= (1 << UCSZ01 | 1 << UCSZ00);		//Asynchronous USART, Parity none, 1 Stop bit, 8-bit character size
	UBRR0H = 00;
	UBRR0L = 103;								//Baudrate 9600
	
	sei();										//Enable interrupt routines
	init_master();
	PORTD |= 0b00000011; //Pullup SDA and SCL
	
	
	
	while (1)
	{
	}
}

//I2C functions ----------------------------
void init_rp6Data(){
	rp6Data.driveSpeed = 0;
	rp6Data.driveDirection = 0;
	rp6Data.turnDirection = 0;
	rp6Data.accelerationRate = 30;
	rp6Data.turnRate = 3000;
	rp6Data.driveSpeedThreshold = 5000;
	rp6Data.updateSpeed = 200000;
	rp6Data.enableBeeper = 1;
	rp6Data.motorEncoderLVal = 0;
	rp6Data.motorEncoderRVal = 0;
}


void rp6DataConstructor(){
	uint8_t I2Cdata[20];
	
	I2Cdata[0] = 1;
	I2Cdata[1] = rp6Data.driveSpeed;
	I2Cdata[2] = rp6Data.driveDirection + 1;
	I2Cdata[3] = rp6Data.turnDirection + 1;
	I2Cdata[4] = rp6Data.accelerationRate;
	I2Cdata[5] = rp6Data.turnRate * 255 / 8000;
	I2Cdata[6] = rp6Data.driveSpeedThreshold * 255 / 6000;
	I2Cdata[7] = rp6Data.updateSpeed / 2000;
	I2Cdata[8] = rp6Data.enableBeeper;
	
	for(int i = 9; i <= 19; i++){
		I2Cdata[i] = 0;
	}
	
	I2C_sendConstructor(I2Cdata);
}


void I2C_sendConstructor(uint8_t I2Cdata[]){
	for(int i = 0; i <= 19; i++){
		verzenden(8, I2Cdata[i]);
	}
}

//------------------------------------------