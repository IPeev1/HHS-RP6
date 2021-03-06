/*

De Haagse Hogeschool RP6 robot project (NSE)

Jaar 1 (2017 - 2018)

Created: 13-03-2018
Finished: 07-04-2018

Authors:
- Ivan Peev
- Matthijs Heidema
- Rens van den Heuvel
- Willem van der Gaag
- Sander Klein Breteler

Code voor de Arduino Mega ATmega2560

*/

//Defines ----------------------------------------
//General
#define F_CPU 16000000									//Clock speed

//I2C
#define SCL 100000										//Define the TWI clock frequency
#define DATASIZE 20										//Define how much data is transferred per transmit (Array length)
#define RP6_ADDRESS 3									//Define the slave address of the RP6 chip
#define TWISendStart()		(TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN)|(1<<TWIE))
#define TWISendStop()		(TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN)|(1<<TWIE))
#define TWISendTransmit()	(TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE))
#define TWISendACK()		(TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWEA))
#define TWISendNACK()		(TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE))

//USART
#define USART_INTERRUPT_VECTOR USART0_RX_vect
//-------------------------------------------------

//Libraries ---------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include "matthijs_testFunctions.h" /*Contains functions for transmitting over USART for testing purposes*/
#include "ultrasonicSensor.h"
#include "musicBox.h"
//-------------------------------------------------

//Global variables --------------------------------
//Timers (used with micros function)
uint32_t writeTerminalTimer = 0;
uint32_t writeTerminalTimerSpeed = 250000;
uint32_t ultrasonicTimer = 0;
uint32_t ultrasonicTimerSpeed = 100000;
uint32_t backBeepTimer = 0;
uint32_t backBeepSpeed = 500000;

//Micros
volatile uint64_t t3TotalOverflow;				//Track the total overflows

//Compass
int compassFlag = 0;

//I2C
uint8_t sendDataTWI[DATASIZE];							//Create an array for holding the data that needs to be send
uint8_t receiveDataTWI[DATASIZE];						//Same but for the receive data --^

//Structs (shared data between Arduino and RP6)
struct rp6DataBP {
	uint16_t	driveSpeed;				//value between 0 - 100
	int8_t		driveDirection;			//Value 0 or 1
	int8_t		turnDirection;			//Value -1, 0 or 1
	uint16_t	accelerationRate;		//Percentage to accelerate with					(Default: 30)
	uint16_t	turnRate;				//Intensity to turn with						(Default: 3000)
	uint16_t	driveSpeedThreshold;	//Minimal power needed to actually start moving	(Default: 5000)
	uint32_t	updateSpeed;			//Interval time between updates					(Default: 200000)
	uint8_t		enableBeeper;			//Set to 1 to enable reverse driving beeper		(Default: 1	(On))
	int64_t		compassAngle;			//Degrees from north given by compass
} rp6Data;
struct arduinoDataBP {
	uint16_t	bumperFlag;		//Segment count of the left motor encoder		(Updated by interrupt)
	uint16_t	actualDriveSpeed;		//Same for right encoder ---^
	uint16_t	actualLeftMotorSpeed;		//Distance driven by left motor
	uint16_t	actualRightMotorSpeed;		//right motor ---^
	uint16_t	totalDistance;			//Total distance driven by the robot
} arduinoData;

//USART
char USARTcommand = ' ';
char USARTreceived = ' ';
char USARTinput[255];
int USARTinputPos = -1;
//-------------------------------------------------

//Function declarations ---------------------------
//General
void init_interrupt();

//Micros
uint64_t micros();								//Keep track of the amount of microseconds passed since boot
ISR(TIMER3_OVF_vect);							//Interrupt for Timer3, for micros()
void init_micros();								//Configure Timer3

//USART
void init_USART();
ISR(USART_INTERRUPT_VECTOR);
void writeToTerminal();

//I2C
void init_TWI();
void init_TWI_Timer2();
void init_arduinoData();
void init_rp6Data();
ISR(TWI_vect);
ISR(TIMER2_OVF_vect);
void I2C_receiveInterpreter();
void arduinoDataInterpreter();
void rp6DataConstructor();
void clearSendData();
void clearReceiveData();
void TWIWrite(uint8_t u8data);
uint8_t TWIGetStatus();
void TWIwaitUntilReady();
void checkCode(uint8_t code);
void writeToSlave(uint8_t address, uint8_t dataByte[]);
void readFromSlave(uint8_t address);
void readFromCompass();

//Blinkers
void turnSignal();

//Ultrasonic
void checkUltrasonic();

//Beeper
void beeper();
//------------------------------------------------

////////////////// MAIN PROGRAM //////////////////
int main(void){
	//Initialize all functions
	init_interrupt();
	init_micros();
	init_USART();
	init_TWI();
	init_TWI_Timer2();
	initBackBeep();
	init_rp6Data();
	init_arduinoData();
	initTimer();
	init_USART();
	//-----------------------
	
	while (1){
		
		if(arduinoData.bumperFlag) {
			rp6Data.driveSpeed = 0;
		}
		
		writeToTerminal();
		
		checkUltrasonic();
		
		turnSignal();
		
		beeper();
		
	}
}
//////////////////////////////////////////////////

//Function definitions ---------------------------
//General
void init_interrupt(){
	sei();									//Enable global interrupts
}

//Micros
void init_micros(){
	TCCR3B |= (1 << CS00);			//Set a timer prescaler of 'none'
	TIMSK3 |= (1 << TOIE3);			//Enable overflow interrupts
	TCNT3 = 0;						//Initialize the timer by setting it to 0
	t3TotalOverflow = 0;			//Initialize the overflow counter by setting it to 0
}

ISR(TIMER3_OVF_vect){						//When the internal timer 3 overflows and loops back to 0, this interrupt triggers
	t3TotalOverflow++;							//Increase the total overflow counter
}

uint64_t micros(){
	uint8_t currentTimer3Value = TCNT3;																				//Get the current value of the Timer 0 register
	uint64_t microsReturnValue = ((4096 * t3TotalOverflow) + (currentTimer3Value * 4096 / 65536));					//Calculate the passed microseconds based on the total amount of overflows and the current timer value.
	return microsReturnValue;																						//Return the calculated value
}

//USART
void init_USART(){
	//USART initialization
	UCSR0A = 0x00;
	UCSR0B |= (1 << RXCIE0 | 1 << RXEN0);		//Enable USART receiver, receiver interrupt
	UCSR0B |= 1 << TXEN0;						/*Transmitter enabled for testing*/
	UCSR0C |= (1 << UCSZ01 | 1 << UCSZ00);		//Asynchronous USART, Parity none, 1 Stop bit, 8-bit character size
	UBRR0 = 16;								//Baudrate 9600
}

ISR(USART_INTERRUPT_VECTOR){
	static uint16_t number[3] = {0,0,0};
	static int numberSize[3] = {0,0,0};
	static int numberStart[3] = {0,0,0};
	
	USARTreceived = UDR0;
	
	if(('0' <= USARTreceived && USARTreceived <= '9') || USARTreceived == ' '){
		
		if(USARTinputPos < 255)
		USARTinput[++USARTinputPos] = USARTreceived;
		
		}else if('a' <= USARTreceived && USARTreceived <= 'z' && USARTreceived != 'b'){
		
		USARTcommand = USARTreceived;
		
		}else if(USARTreceived == 'b'){
		
		USARTinputPos--;
		
		}else if(USARTreceived == '\r'){
		
		if(USARTinputPos >= 0){
			number[0] = 0;
			number[1] = 0;
			number[2] = 0;
			
			numberSize[0] = -1;
			numberSize[1] = -1;
			numberSize[2] = -1;
			
			numberStart[0] = 0;
			
			int numberPos = 0;
			
			for(uint8_t i = 0; i <= USARTinputPos; i++){
				if(USARTinput[i] == ' '){
					numberPos++;
					numberStart[numberPos] = i + 1;
					if(numberPos > 2){
						break;
						}else{
						continue;
					}
					}else{
					numberSize[numberPos]++;
				}
			}
			
			uint8_t charToInt;
			numberPos = 0;
			
			for(uint8_t i = 0; i <= USARTinputPos; i++){
				
				if(USARTinput[i] == ' '){
					numberPos++;
					if(numberPos > 2){
						break;
						}else{
						continue;
					}
				}
				
				charToInt = (int) (USARTinput[i] - '0');
				number[numberPos] += charToInt * ( (int)(pow(10, numberSize[numberPos] + numberStart[numberPos] - i) + 0.5) );
				
			}
		}
		
		if(USARTcommand){
			switch(USARTcommand){
				case 'w':
				if (rp6Data.driveDirection == 1) {
					rp6Data.driveDirection = 0;
					} else {
					rp6Data.driveDirection = 1;
				}
				break;
				
				case 's':
				if (rp6Data.driveDirection == -1) {
					rp6Data.driveDirection = 0;
					} else {
					rp6Data.driveDirection = -1;
				}
				break;
				
				case 'a':
				if (rp6Data.turnDirection == -1) {
					rp6Data.turnDirection = 0;
					} else {
					rp6Data.turnDirection = -1;
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
				if (number[0] <= 100) {
					rp6Data.driveSpeed = number[0];
				}
				break;
				
				case 'r':
				rp6Data.turnRate = number[0];
				break;
				
				case 'q':
				rp6Data.accelerationRate = number[0];
				break;
				
				case 'z':
				rp6Data.driveSpeed = 0;
				rp6Data.turnDirection = 0;
				rp6Data.driveDirection = 0;
				break;
			}
		}
		
		USARTcommand = 0;
		USARTinputPos = -1;
		
	}
	
	
	
	
}

void writeToTerminal(){
	if (writeTerminalTimer < micros()) {
		writeString("\f\r");
		
		writeString("Distance to object: ");
		writeInt(ultrasonicSensor());
		
		writeString("mm\n\rCompass Angle: ");
		writeInt(rp6Data.compassAngle);
		writeString(" degrees");
		
		writeString("\n\n\rSet speed: ");
		writeInt(rp6Data.driveSpeed);
		
		writeString("%\n\n\rDirection: ");
		if(rp6Data.driveDirection == 1) writeString("Forward, ");
		else if(rp6Data.driveDirection == 0) writeString("Stationary, ");
		else if(rp6Data.driveDirection == -1) writeString("Backwards, ");
		if(rp6Data.turnDirection == -1) writeString("turning left");
		else if(rp6Data.turnDirection == 0) writeString("going straight");
		else if(rp6Data.turnDirection == 1) writeString("turning right");
		
		writeString("\n\rAcceleration rate: ");
		writeInt(rp6Data.accelerationRate);
		
		writeString("\n\rTurn rate: ");
		writeInt(rp6Data.turnRate);
		
		writeString("\n\n\rCommand: ");
		writeChar(USARTcommand);
		
		writeString("\n\rValue: ");
		if(USARTinputPos >= 0){
			for(int i = 0; i <= USARTinputPos; i++){
				writeChar(USARTinput[i]);
			}
		}
		
		writeTerminalTimer = micros() + writeTerminalTimerSpeed;
	}
}

//I2C
void init_TWI()
{
	TWSR = 0;									//No prescaling
	TWBR = ((F_CPU / SCL) - 16) / 2;			//Set SCL to 100kHz
	TWCR = (1 << TWEN) | (1 << TWIE);			//Enable TWI and Enable Interrupt
}

void init_TWI_Timer2(){
	TCCR2B |= (1 << CS20) | (1 << CS21) | (1 << CS22);
	TIMSK2 |= (1 << TOIE2);
	TCNT2 = 0;
}

void init_arduinoData(){
	arduinoData.bumperFlag = 0;
	arduinoData.actualDriveSpeed = 0;
	arduinoData.actualLeftMotorSpeed = 0;
	arduinoData.actualRightMotorSpeed = 0;
	arduinoData.totalDistance = 0;
}

void init_rp6Data(){
	rp6Data.driveSpeed = 0;
	rp6Data.driveDirection = 0;
	rp6Data.turnDirection = 0;
	rp6Data.accelerationRate = 4900;
	rp6Data.turnRate = 9000;
	rp6Data.driveSpeedThreshold = 5000;
	rp6Data.updateSpeed = 200;
	rp6Data.enableBeeper = 1;
}

ISR(TWI_vect){
	static int bytecounter = 0;
	
	switch(TWSR){
		case 0x40:
		if(compassFlag){
			TWISendNACK();
		}else{
			clearReceiveData();
			bytecounter = 0;
			TWISendACK();
		}
		break;
		
		case 0x50:
		receiveDataTWI[bytecounter] = TWDR;
		if(bytecounter < DATASIZE - 2){
			bytecounter++;
			TWISendACK();
			}else{
			bytecounter++;
			TWISendNACK();
		}
		break;
		
		case 0x58:
		if(!compassFlag){
			receiveDataTWI[bytecounter] = TWDR;
			TWISendStop();
			I2C_receiveInterpreter();
		}else{
			uint64_t temp = TWDR;
			rp6Data.compassAngle = ((temp * 360) / 255);
			TWISendStop();
			compassFlag = 0;
		}
		break;
	}
}

ISR(TIMER2_OVF_vect){
	static int counter = 0;
	
	if(counter == 3){
		rp6DataConstructor();
	}else if(counter == 6){
		readFromCompass();
	}else if(counter == 9){
		readFromSlave(RP6_ADDRESS);
	}else if(counter >= 12){
		readFromCompass();
		counter = 0;
	}
	
	counter++;
}

void I2C_receiveInterpreter(){
	int dataSet = receiveDataTWI[0];
	switch(dataSet){
		case(1): arduinoDataInterpreter(); break;
	}
}

void arduinoDataInterpreter(){
	arduinoData.bumperFlag = (receiveDataTWI[1] << 8) + receiveDataTWI[2];
	arduinoData.actualDriveSpeed = (receiveDataTWI[3] << 8) + receiveDataTWI[4];
	arduinoData.actualLeftMotorSpeed = (receiveDataTWI[5] << 8) + receiveDataTWI[6];
	arduinoData.actualRightMotorSpeed = (receiveDataTWI[7] << 8) + receiveDataTWI[8];
	arduinoData.totalDistance = (receiveDataTWI[9] << 8) + receiveDataTWI[10];
}

void rp6DataConstructor(){
	clearSendData();
	
	sendDataTWI[0] = 1;
	if(rp6Data.driveSpeed > 100){rp6Data.driveSpeed = 100;}
	sendDataTWI[1] = rp6Data.driveSpeed;
	sendDataTWI[2] = rp6Data.driveDirection + 1;
	sendDataTWI[3] = rp6Data.turnDirection + 1;
	
	sendDataTWI[4] = (rp6Data.accelerationRate >> 8);
	sendDataTWI[5] = rp6Data.accelerationRate;
	
	sendDataTWI[6] = (rp6Data.turnRate >> 8);
	sendDataTWI[7] = rp6Data.turnRate;
	
	sendDataTWI[8] = (rp6Data.driveSpeedThreshold >> 8);
	sendDataTWI[9] = rp6Data.driveSpeedThreshold;
	
	sendDataTWI[10] = (rp6Data.updateSpeed >> 8);
	sendDataTWI[11] = rp6Data.updateSpeed;
	
	sendDataTWI[12] = rp6Data.enableBeeper;
	
	sendDataTWI[13] = (rp6Data.compassAngle >> 8);
	sendDataTWI[14] = rp6Data.compassAngle;
	
	for(int i = 15; i < DATASIZE; i++){
		sendDataTWI[i] = 0;
	}
	
	writeToSlave(RP6_ADDRESS, sendDataTWI);
}

void clearSendData(){
	for(int i = 0; i < DATASIZE; i++){
		sendDataTWI[i] = 0;
	}
}

void clearReceiveData(){
	for(int i = 0; i < DATASIZE; i++){
		receiveDataTWI[i] = 0;
	}
}

void TWIWrite(uint8_t u8data)
{
	TWDR = u8data;
	TWISendTransmit();
}

uint8_t TWIGetStatus(){
	return (TWSR & 0xF8);
}

void TWIwaitUntilReady(){
	while (!(TWCR & (1 << TWINT)));
}

void checkCode(uint8_t code){
	if (TWIGetStatus() != code){
		char buffer[255];
		writeString("\n\n\rERROR: Wrong status! Code retrieved: 0x");
		writeString( itoa( TWIGetStatus(), buffer, 16) );
		writeString("\n\n\r");
	}
}

void writeToSlave(uint8_t address, uint8_t dataByte[]){
	
	TWISendStart();
	TWIwaitUntilReady();
	checkCode(0x08);
	
	TWIWrite((address << 1));
	TWIwaitUntilReady();
	checkCode(0x18);
	
	for(int i = 0; i < DATASIZE; i++){
		TWIWrite(dataByte[i]);
		TWIwaitUntilReady();
		checkCode(0x28);
	}
	
	TWISendStop();
	
}

void readFromSlave(uint8_t address){
	
	TWISendStart();
	TWIwaitUntilReady();
	checkCode(0x08);
	
	TWIWrite( (address << 1) + 1 );
	TWIwaitUntilReady();
	
}

void readFromCompass(){
	compassFlag = 1;
	TWISendStart();
	TWIwaitUntilReady();
	checkCode(0x08);
	
	TWIWrite(0xC0);
	TWIwaitUntilReady();
	checkCode(0x18);
	
	TWIWrite(1);
	TWIwaitUntilReady();
	checkCode(0x28);
	
	TWISendStart();
	TWIwaitUntilReady();
	checkCode(0x10);
	
	TWIWrite(0xC1);
	TWIwaitUntilReady();
}

//Blinkers
void turnSignal(){
	static uint32_t turnSignalDelay = 500000;
	static uint32_t turnSignalStart = 0;
	
	DDRC |= (1 << PINC1);
	DDRD |= (1 << PIND7);
	
	if(rp6Data.turnDirection == -1){
		if(turnSignalStart < micros()){
			PORTC ^= (1 << PINC1);	
			PORTD &= ~(1 << PIND7);
			turnSignalStart = micros() + turnSignalDelay;
		}
	}else if(rp6Data.turnDirection == 1){
		if(turnSignalStart < micros()){
			PORTC &= ~(1 << PINC1);
			PORTD ^= (1 << PIND7);
			turnSignalStart = micros() + turnSignalDelay;
		}
	}else{
		PORTC &= ~(1 << PINC1);
		PORTD &= ~(1 << PIND7);
	}
}

//Ultrasonic
void checkUltrasonic(){
	if(ultrasonicTimer < micros()){
		
		uint16_t distance = ultrasonicSensor();
		static int stopState = 0;
		static uint16_t tempAcceleration;
		
		if(distance > 400 && stopState == 1){
			rp6Data.accelerationRate = tempAcceleration;
			}else if(distance > 300 && stopState == 2){
			stopState = 0;
		}
		
		if(distance < 400 && distance > 300 && rp6Data.driveSpeed > 40 && rp6Data.driveDirection == 1){
			rp6Data.driveSpeed = 40;
			}else if(distance < 300 && distance > 85 && rp6Data.driveSpeed > 25 && rp6Data.driveDirection == 1){
			rp6Data.driveSpeed = 25;
			}else if(distance < 85 && rp6Data.driveDirection == 1){
			if(stopState == 0){
				tempAcceleration = rp6Data.accelerationRate;
				rp6Data.accelerationRate = 5000;
				rp6Data.driveSpeed = 0;
				stopState = 1;
				}else if(stopState == 1){
				rp6Data.accelerationRate = tempAcceleration;
				stopState = 2;
			}
		}
		
		ultrasonicTimer = micros() + ultrasonicTimerSpeed;
	}
}

//Beeper
void beeper(){
	if (backBeepTimer < micros()) {
		if ((rp6Data.driveDirection == -1 && rp6Data.driveSpeed > 20)  || arduinoData.bumperFlag) {
			DDRH ^= (1 << BEEPER);
			backBeepTimer = micros() + backBeepSpeed;
			} else {
			DDRH &= ~(1 << BEEPER);
		}
	}
	
	if (rp6Data.driveDirection != -1 && !arduinoData.bumperFlag) {
		DDRH &= ~(1 << BEEPER);
	}
}
//------------------------------------------------