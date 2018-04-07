/*

De Haagse Hogeschool RP6 robot project (NSE)

Jaar 1 (2017 - 2018)

Created: 13-03-2018
Finished: 08-04-2018

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
#define F_CPU 16000000																//Clock speed

//I2C
#define SCL 100000																	//Define the TWI clock frequency
#define DATASIZE 15																	//Define how much data is transferred per transmit (Array length)
#define RP6_ADDRESS 3																//Define the slave address of the RP6 chip
#define TWISendStart()		(TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN)|(1<<TWIE))		//Defines for a quick and easy way of setting a couple of register settings
#define TWISendStop()		(TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN)|(1<<TWIE))
#define TWISendTransmit()	(TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE))
#define TWISendACK()		(TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWEA))
#define TWISendNACK()		(TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE))					//--^

//USART
#define USART_INTERRUPT_VECTOR USART0_RX_vect
//-------------------------------------------------

//Libraries ---------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include "matthijs_testFunctions.h"							//Contains functions for transmitting over USART
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
volatile uint64_t t3TotalOverflow;							//Track the total overflows to calculate the time

//Compass
int compassFlag = 0;										//Used to differentiate reading from the compass and reading from the RP6 with I2C

//I2C
uint8_t sendDataTWI[DATASIZE];								//Create an array for holding the data that needs to be send
uint8_t receiveDataTWI[DATASIZE];							//Same but for the receive data --^

//Structs (shared data between Arduino and RP6)
struct rp6DataBP {
	uint16_t	driveSpeed;									//value between 0 - 100
	int8_t		driveDirection;								//Value -1, 0 or 1				(this is converted to 0 and 1 when arriving in the RP6)
	int8_t		turnDirection;								//Value -1, 0 or 1
	uint16_t	accelerationRate;							//Percentage to accelerate with
	uint16_t	turnRate;									//Intensity to turn with
	uint16_t	driveSpeedThreshold;						//Minimal power needed to actually start moving
	uint32_t	updateSpeed;								//Interval time between updates
	int64_t		compassAngle;								//Degrees from north given by compass
} rp6Data;
struct arduinoDataBP {
	uint16_t	bumperFlag;									//Flag for when the bumper is pressed to set the speed to 0
	uint16_t	actualDriveSpeed;							//Actual speed of the robot, not the requested speed
	uint16_t	actualLeftMotorSpeed;						//Actual speed of the left motor
	uint16_t	actualRightMotorSpeed;						//right motor ---^
} arduinoData;

//USART
char USARTcommand = ' ';									//Store the command received from the terminal
char USARTreceived = ' ';									//Store the character that is send through the serial line
char USARTinput[255];										//Store the values given with the command from the terminal
int USARTinputPos = -1;										//Keeps track of how many elements are filled in the input array
//-------------------------------------------------

//Function declarations ---------------------------
//General
void init_interrupt();										//Globally enable the use of interrupts

//Micros
uint64_t micros();											//Keep track of the amount of microseconds passed since boot
ISR(TIMER3_OVF_vect);										//Interrupt for Timer3, for micros()
void init_micros();											//Configure Timer3

//USART
void init_USART();
ISR(USART_INTERRUPT_VECTOR);
void writeToTerminal();

//I2C
void init_TWI();											//Initialize the TWI registers
void init_TWI_Timer2();										//Initialize the timer with which the TWI synchronizes the structs
void init_arduinoData();									//Initialize the default values of the Arduino struct
void init_rp6Data();										//Initialize the default values of the RP6 struct
ISR(TWI_vect);												//ISR used for responding to certain TWI status codes
ISR(TIMER2_OVF_vect);										//ISR for the overflow of the synchronize timer
void arduinoDataInterpreter();								//Interprets the array of data retrieved from the RP6
void rp6DataConstructor();									//Constructs an array of data from the struct, to send to the RP6
void clearSendData();										//Clear the array used for sending data
void clearReceiveData();									//Clear the array used for incoming data
void TWIWrite(uint8_t u8data);								//Write data to the TWI data line
uint8_t TWIGetStatus();										//Read the current TWI status code from the register and return it
void TWIwaitUntilReady();									//Wait until the TWI hardware has finished its current job
void checkCode(uint8_t code);								//Check the TWI status register for a certain code, if that code is not present, throw and error
void writeToSlave(uint8_t address, uint8_t dataByte[]);		//Write an array of data to a TWI slave
void readFromSlave(uint8_t address);						//Read data from a TWI slave
void readFromCompass();										//Read data from the compass

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
		
		if(arduinoData.bumperFlag) {				//If the bumper flag is set, aka the bumpers have been hit
			rp6Data.driveSpeed = 0;						//Set the drive speed to 0, this avoids running into the same object again
		}
		
		writeToTerminal();							//Write all the relevant information to the terminal
		
		checkUltrasonic();							//Check the distance to objects in front of the robot with the ultrasonic sensor, if objects get to close this function wil stop the robot
		
		turnSignal();								//If the robot is turning, this function will turn on the blinkers
		
		beeper();									//If the robot is driving backwards, this function turns on the beeper
		
	}
}
//////////////////////////////////////////////////

//Function definitions ---------------------------
//General
void init_interrupt(){												//Globally enable the use of interrupts
	sei();																//Enable global interrupts
}

//Micros
void init_micros(){													//Configure Timer3
	TCCR3B |= (1 << CS00);												//Set a timer prescaler of 'none'
	TIMSK3 |= (1 << TOIE3);												//Enable overflow interrupts
	TCNT3 = 0;															//Initialize the timer by setting it to 0
	t3TotalOverflow = 0;												//Initialize the overflow counter by setting it to 0
}

ISR(TIMER3_OVF_vect){								//When the internal timer 3 overflows and loops back to 0, this interrupt triggers
	t3TotalOverflow++;									//Increase the total overflow counter
}

uint64_t micros(){
	uint8_t currentTimer3Value = TCNT3;																				//Get the current value of the Timer 0 register
	uint64_t microsReturnValue = ((4096 * t3TotalOverflow) + (currentTimer3Value * 4096 / 65536));					//Calculate the passed microseconds based on the total amount of overflows and the current timer value.
	return microsReturnValue;																						//Return the calculated value
}

//USART
void init_USART(){
	UCSR0A = 0x00;
	UCSR0B |= (1 << RXCIE0 | 1 << RXEN0);			//Enable USART receiver, receiver interrupt
	UCSR0B |= 1 << TXEN0;							//Enable transmitting of data
	UCSR0C |= (1 << UCSZ01 | 1 << UCSZ00);			//Asynchronous USART, Parity none, 1 Stop bit, 8-bit character size
	UBRR0 = 16;										//Baud rate 57600
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
void init_TWI(){													//Initialize the TWI registers
	TWSR = 0;															//No prescaling and reset the status codes
	TWBR = ((F_CPU / SCL) - 16) / 2;									//Set SCL to 100kHz
	TWCR = (1 << TWEN) | (1 << TWIE);									//Enable TWI and Enable Interrupt
}

void init_TWI_Timer2(){												//Initialize the timer with which the TWI synchronizes the structs
	TCCR2B |= (1 << CS20) | (1 << CS21) | (1 << CS22);					//Set a prescaler of 1024
	TIMSK2 |= (1 << TOIE2);												//Enable overflow interrupts
	TCNT2 = 0;															//Initialize the timer to 0
}

void init_arduinoData(){											//Initialize the default values of the Arduino struct
	arduinoData.bumperFlag = 0;
	arduinoData.actualDriveSpeed = 0;
	arduinoData.actualLeftMotorSpeed = 0;
	arduinoData.actualRightMotorSpeed = 0;
}

void init_rp6Data(){												//Initialize the default values of the RP6 struct
	rp6Data.driveSpeed = 0;
	rp6Data.driveDirection = 0;
	rp6Data.turnDirection = 0;
	rp6Data.accelerationRate = 4900;
	rp6Data.turnRate = 9000;
	rp6Data.driveSpeedThreshold = 5000;
	rp6Data.updateSpeed = 200;
}

ISR(TWI_vect){														//ISR used for responding to certain TWI status codes
	static int bytecounter = 0;											//Keep track of how many bytes have been received, initialize at 0
	
	switch(TWSR){														//Switch on the different status codes the TWI register can have
		case 0x40:															//0x40 SLA+R has been transmitted; ACK has been received
		if(compassFlag){														//If the compass flag is set, meaning we want to read compass data
			TWISendNACK();															//Send NACK to tell the slave to only send one byte
		}else{																	//If the compass flag is not set
			clearReceiveData();														//Clear the array for receiving data
			bytecounter = 0;														//Reset the byte counter
			TWISendACK();															//Send ACK to tell the slave to start transmitting multiple bytes
		}
		break;
		
		case 0x50:															//0x50 Data byte has been received; ACK has been returned
		receiveDataTWI[bytecounter] = TWDR;										//Read the data from the register and add it to the receive array
		if(bytecounter < DATASIZE - 2){												//If received a byte and it is not the next to last
			bytecounter++;																//Set the counter to the next byte
			TWISendACK();																//Send ACK to tell the slave to send more
			}else{																	//If the byte is the next to last
			bytecounter++;																//Set the counter to the next byte
			TWISendNACK();																//Send NACK to tell the slave that the next byte is the last and he should not send more
		}
		break;
		
		case 0x58:															//0x58 Data byte has been received; NOT ACK has been returned
		if(compassFlag){														//If we are reading compass data
			uint64_t temp = TWDR;													//Read the byte from the TWI data register
			rp6Data.compassAngle = ((temp * 360) / 255);							//Rescale the 8bit value to a 360 degrees scale
			TWISendStop();															//Send a stop signal to end the TWI transmission
			compassFlag = 0;														//Reset the compass flag
		}else{																	//If we are reading data from the RP6, this code would mean it is the last byte
			receiveDataTWI[bytecounter] = TWDR;										//Set the retrieved byte in the array
			TWISendStop();															//Set a stop signal to end the transmission
			arduinoDataInterpreter();												//Run the data interpreter to extract the data from the array and place it in the struct
		}
		break;
	}
}

ISR(TIMER2_OVF_vect){														//ISR for the overflow of the synchronize timer
	static int counter = 0;														//Keep track of how many time the overflow interrupt has happened
	
	if(counter == 3){															//If the counter is 3
		rp6DataConstructor();														//Construct the RP6 data in the array and send it
	}else if(counter == 6){														//If the counter is 6
		readFromCompass();															//Read the current compass angle from the compass
	}else if(counter == 9){														//If the counter is 9
		readFromSlave(RP6_ADDRESS);													//Read the Arduino data from the RP6
	}else if(counter >= 12){													//If the counter is 12 or more
		readFromCompass();															//Read from the compass again
		counter = 0;																//Reset the counter to start over again
	}
	
	counter++;																	//Add 1 to the counter
}

void arduinoDataInterpreter(){															//Interprets the array of data retrieved from the RP6
	arduinoData.bumperFlag = (receiveDataTWI[1] << 8) + receiveDataTWI[2];
	arduinoData.actualDriveSpeed = (receiveDataTWI[3] << 8) + receiveDataTWI[4];
	arduinoData.actualLeftMotorSpeed = (receiveDataTWI[5] << 8) + receiveDataTWI[6];
	arduinoData.actualRightMotorSpeed = (receiveDataTWI[7] << 8) + receiveDataTWI[8];
}

void rp6DataConstructor(){													//Constructs an array of data from the struct, to send to the RP6
	clearSendData();															//Clear the array before beginning
	
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
	
	sendDataTWI[12] = (rp6Data.compassAngle >> 8);
	sendDataTWI[13] = rp6Data.compassAngle;
	
	for(int i = 14; i < DATASIZE; i++){											//Fill the left over bytes in the array with 0
		sendDataTWI[i] = 0;
	}
	
	writeToSlave(RP6_ADDRESS, sendDataTWI);
}

void clearSendData(){														//Clear the array used for sending data
	for(int i = 0; i < DATASIZE; i++){
		sendDataTWI[i] = 0;
	}
}

void clearReceiveData(){													//Clear the array used for incoming data
	for(int i = 0; i < DATASIZE; i++){
		receiveDataTWI[i] = 0;
	}
}

void TWIWrite(uint8_t u8data){												//Write data to the TWI data line
	TWDR = u8data;																//Put the given byte in the register for sending
	TWISendTransmit();															//Transmit the data in the register
}

uint8_t TWIGetStatus(){														//Read the current TWI status code from the register and return it
	return (TWSR & 0xF8);														//Read the status register and mask the prescaler bits. Return the left over variable
}

void TWIwaitUntilReady(){													//Wait until the TWI hardware has finished its current job
	while (!(TWCR & (1 << TWINT)));
}

void checkCode(uint8_t code){												//Check the TWI status register for a certain code, if that code is not present, throw and error
	if (TWIGetStatus() != code){
		char buffer[255];
		writeString("\n\n\rERROR: Wrong status! Code retrieved: 0x");
		writeString( itoa( TWIGetStatus(), buffer, 16) );
		writeString("\n\n\r");
	}
}

void writeToSlave(uint8_t address, uint8_t dataByte[]){						//Write an array of data to a TWI slave
	
	TWISendStart();																//First send a start bit, this lets slaves know a transmission is going to start
	TWIwaitUntilReady();														//Wait until that action is done
	checkCode(0x08);															//Check the status register for code 0x08 A START condition has been transmitted
	
	TWIWrite((address << 1));													//Write the address over the line to address a certain slave
	TWIwaitUntilReady();														//wait
	checkCode(0x18);															//0x18 SLA+W has been transmitted; ACK has been received
	
	for(int i = 0; i < DATASIZE; i++){											//Send all the bytes
		TWIWrite(dataByte[i]);														//Write the byte over the line
		TWIwaitUntilReady();														//wait
		checkCode(0x28);															//0x28 Data byte has been transmitted; ACK has been received
	}
	
	TWISendStop();																//Send a stop bit to let the slave know all data has been send
	
}

void readFromSlave(uint8_t address){										//Read data from a TWI slave
	
	TWISendStart();																//Send a start bit
	TWIwaitUntilReady();														//Wait
	checkCode(0x08);															//0x08 A START condition has been transmitted
	
	TWIWrite( (address << 1) + 1 );												//Send the address and a 1 to let the slave know he needs to send data
	TWIwaitUntilReady();														//Wait
																				//The rest of the function is handled by the ISR
}

void readFromCompass(){														//Read data from the compass
	
	compassFlag = 1;															//Set the compass flag to 1 so the ISR handles the data correctly
	
	TWISendStart();																//Start transmission
	TWIwaitUntilReady();
	checkCode(0x08);															//0x08 A START condition has been transmitted
	
	TWIWrite(0xC0);																//Write the address with write command
	TWIwaitUntilReady();
	checkCode(0x18);															//0x18 SLA+W has been transmitted; ACK has been received
	
	TWIWrite(1);																//Write a 1 to the compass setting it to return only 1 byte
	TWIwaitUntilReady();
	checkCode(0x28);															//0x28 Data byte has been transmitted; ACK has been received
	
	TWISendStart();																//Send repeated start bit
	TWIwaitUntilReady();
	checkCode(0x10);															//0x10 A repeated START condition has been transmitted
	
	TWIWrite(0xC1);																//Write the address with a read command, triggering the compass to send its data
	TWIwaitUntilReady();
																				//The rest of the function is handled by the ISR
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