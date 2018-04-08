/*

De Haagse Hogeschool RP6 robot project (NSE)

Jaar 1 (2017 - 2018)

Created: 12-03-2018
Finished: 08-04-2018

Authors:
- Ivan Peev
- Matthijs Heidema
- Rens van den Heuvel
- Willem van der Gaag
- Sander Klein Breteler

Code voor de RP6 ATmega32

*/

//Defines ----------------------------------------
//General
#define F_CPU 8000000									//Clock speed of the CPU

//I2C
#define SCL 100000										//Set the TWI clock speed
#define DATASIZE 15										//Set the array data size that is send through the TWI
#define RP6_ADDRESS 3									//Set the address of this slave

//Bumpers
#define BUMPED_STOP_TIME 70000							//Determines for how long the RP6 stands still after the bumpers are pushed
#define BUMPED_BACK_TIME 1000000						//Determines for how long the RP6 drives backwards after the bumpers are pushed. Includes BUMPED_STOP_TIME
//------------------------------------------------

//Libraries --------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include "bumpers.h"
//------------------------------------------------

//Global variables -------------------------------
//Micros
volatile uint64_t t0TotalOverflow;						//Track the total overflows

//I2C
uint8_t sendData[DATASIZE];								//Create an array for sending data over the TWI
uint8_t receiveData[DATASIZE];							//Create an array for retrieving data over the TWI

//Structs (shared data between Arduino and RP6)
struct rp6DataBP {
	uint32_t	driveSpeed;								//value between 0 - 100
	int8_t		driveDirection;							//Value 0 or 1
	int8_t		turnDirection;							//Value -1, 0 or 1
	uint16_t	accelerationRate;						//Percentage to accelerate with
	uint16_t	turnRate;								//Intensity to turn with
	uint16_t	driveSpeedThreshold;					//Minimal power needed to actually start moving
	uint32_t	updateSpeed;							//Interval time between updates
	int64_t		compassAngle;							//Degrees from north given by compass
} rp6Data, bumpedData;
struct arduinoDataBP {
	uint16_t	bumperFlag;								//Flag for when the bumper is pressed to set the speed to 0
	uint16_t	actualDriveSpeed;						//Actual speed of the robot, not the requested speed
	uint16_t	actualLeftMotorSpeed;					//Actual speed of the left motor
	uint16_t	actualRightMotorSpeed;					//right motor ---^
} arduinoData;
//------------------------------------------------

//Function declarations --------------------------
//General
void init_interrupt();									//Initialize global interrupts

//Micros
uint64_t micros();										//Keep track of the amount of microseconds passed since boot
ISR(TIMER0_OVF_vect);									//Interrupt for Timer0, for micros()
void init_micros();										//Configure Timer0

//I2C
void init_TWI();										//Initialize the TWI registers
void init_rp6Data();									//Initialize the default values of the RP6 struct
void init_arduinoData();								//Initialize the default values of the Arduino struct
ISR(TWI_vect);											//ISR used for responding to certain TWI status codes
void clearSendData();									//Clear the array used for sending data
void clearReceiveData();								//Clear the array used for incoming data
void rp6DataInterpreter();								//Interprets the array of data retrieved from the Arduino
void arduinoDataConstructor();							//Constructs an array of data from the struct, to send to the Arduino

//Motor
void init_motor_io();									//Initialize all inputs and outputs needed for the motors
void init_motor_timer();								//Initialize the timer needed to create a PWM signal for the motors
int motorDriver(struct rp6DataBP rp6Data);				//Calculate the PWM signal for the motors based on the struct data

//Bumpers
void init_bumpedData();
uint8_t bumperCheck();
//------------------------------------------------

////////////////// MAIN PROGRAM //////////////////
int main(void) {
	//Initialize all functions
	init_interrupt();
	
	init_micros();
	
	init_TWI();
	init_rp6Data();
	init_arduinoData();
	clearSendData();
	clearReceiveData();
	
	init_motor_io();
	init_motor_timer();
	
	init_bumpedData();
	//------------------------
		
	while(1){										
		if (bumperCheck()) {							//If the bumpers are hit, if so
			motorDriver(bumpedData);						//Run the motor driver with special input
		} else {										//Otherwise
			motorDriver(rp6Data);							//Run the motor driver with the global struct data
		}
	}
}
//////////////////////////////////////////////////

//Function definitions ---------------------------
//General
void init_interrupt(){												//Globally enable the use of interrupts
	sei();																//Enable global interrupts
}

//Micros
void init_micros(){													//Configure timer 0
	TCCR0 |= (1 << CS00);												//Set a timer prescaler of '64'
	TCCR0 |= (1 << CS01);												//---^
	TIMSK |= (1 << TOIE0);												//Enable overflow interrupts
	TCNT0 = 0;															//Initialize the timer by setting it to 0
	t0TotalOverflow = 0;												//Initialize the overflow counter by setting it to 0
}

ISR(TIMER0_OVF_vect){												//Interrupt for Timer0, for micros()
	t0TotalOverflow++;													//Increase the total overflow counter
}

uint64_t micros(){
	uint8_t currentTimer0Value = TCNT0;																				//Get the current value of the Timer 0 register
	uint64_t microsReturnValue = ((2048 * t0TotalOverflow) + (currentTimer0Value * 2048 / 256));					//Calculate the passed microseconds based on the total amount of overflows and the current timer value.
	return microsReturnValue;																						//Return the calculated value
}

//I2C
void init_TWI(){													//Initialize the TWI registers
	TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE);						//Enable TWI; Enable Acknowledge; Enable Interrupt
	TWSR = 0;															//No prescaling
	TWAR = (RP6_ADDRESS << 1);											//Set slave address
	TWBR = ((F_CPU / SCL) - 16) / 2;									//set TWI clock speed
	DDRC |= 0b00000011;													//Set SDA and SCL on output
	PORTC |= 0b00000011;												//Set pull up resistor
}

void init_rp6Data(){												//Initialize the default values of the RP6 struct
	rp6Data.driveSpeed = 0;
	rp6Data.driveDirection = 1;
	rp6Data.turnDirection = 0;
	rp6Data.accelerationRate = 2000;
	rp6Data.turnRate = 2500;
	rp6Data.driveSpeedThreshold = 7000;
	rp6Data.updateSpeed = 200;
}

void init_arduinoData(){											//Initialize the default values of the Arduino struct
	arduinoData.bumperFlag = 0;
	arduinoData.actualDriveSpeed = 0;
	arduinoData.actualLeftMotorSpeed = 0;
	arduinoData.actualRightMotorSpeed = 0;
}

ISR(TWI_vect){														//ISR used for responding to certain TWI status codes
	static int byteCounter = 0;											//Define a counter to track how many bytes have been send or received
	switch(TWSR){														//Switch on the status code in the status register
		case 0x60:															//0x60 Own SLA+W has been received; ACK has been returned
			clearReceiveData();													//Clear the array for receiving data
			byteCounter = 0;													//Reset the byte counter
		break;
		
		case 0x80:															//0x80 Previously addressed with own SLA+W; data has been received; ACK has been returned
			if(byteCounter <= DATASIZE){										//If the counter is less or equal to the max value
				receiveData[byteCounter] = TWDR;									//Add the received byte to the array
				byteCounter++;														//Up the counter by 1
			}
		break;
		
		case 0xA0:															//0xA0 A STOP condition or repeated START condition has been received while still addressed as Slave
			rp6DataInterpreter();												//Interpret the received data and place it in the struct
		break;
		
		case 0xA8:															//0xA8 Own SLA+R has been received; ACK has been returned
			arduinoDataConstructor();											//Construct the send array with the data from the struct
			byteCounter = 0;													//Reset the byte counter
			TWDR = sendData[byteCounter];										//Set the first byte in the TWI data register
		break;
		
		case 0xB8:															//0xB8 Data byte in TWDRn has been transmitted; ACK has been received
			byteCounter++;														//Up the counter
			TWDR = sendData[byteCounter];										//Send the data
		break;
	}
	
	TWCR |= (1 << TWINT);												//Execute the current register settings
}

void clearSendData(){												//Clear the array used for sending data
	for(int i = 0; i < DATASIZE; i++){
		sendData[i] = 0;
	}
}

void clearReceiveData(){											//Clear the array used for incoming data
	for(int i = 0; i < DATASIZE; i++){
		receiveData[i] = 0;
	}
}

void rp6DataInterpreter(){											//Interprets the array of data retrieved from the Arduino
	if(receiveData[2]-1 == 0){
		rp6Data.driveSpeed = 0;
	}else{
		rp6Data.driveSpeed = receiveData[1];
	}
	
	if(receiveData[2] < 1){
		rp6Data.driveDirection = 0;
	}else{
		rp6Data.driveDirection = 1;
	}
	
	rp6Data.turnDirection = receiveData[3]-1;
	rp6Data.accelerationRate = (receiveData[4] << 8) + receiveData[5];
	rp6Data.turnRate = (receiveData[6] << 8) + receiveData[7];
	rp6Data.driveSpeedThreshold = (receiveData[8] << 8) + receiveData[9];
	rp6Data.updateSpeed = (receiveData[10] << 8) + receiveData[11];
	rp6Data.compassAngle = (receiveData[12] << 8) + receiveData[13];
}

void arduinoDataConstructor(){										//Constructs an array of data from the struct, to send to the Arduino
	clearSendData();
	
	sendData[0] = 1;
	
	sendData[1] = (arduinoData.bumperFlag >> 8);
	sendData[2] = arduinoData.bumperFlag;
	
	sendData[3] = (arduinoData.actualDriveSpeed >> 8);
	sendData[4] = arduinoData.actualDriveSpeed;
	
	sendData[5] = (arduinoData.actualLeftMotorSpeed >> 8);
	sendData[6] = arduinoData.actualLeftMotorSpeed;
	
	sendData[7] = (arduinoData.actualRightMotorSpeed >> 8);
	sendData[8] = arduinoData.actualRightMotorSpeed;
	
	for(int i = 9; i < DATASIZE; i++){
		sendData[i] = 0;
	}
}

//Motor
void init_motor_io(){												//Initialize all inputs and outputs needed for the motors
	DDRD |= 0b00110000;													//Set D5 and D4 on output, these are the motors
	DDRC |= 0b00001100;													//Set C2 and C3 on output, these are the motor directions
}

void init_motor_timer(){											//Initialize the timer needed to create a PWM signal for the motors
	TCCR1A |= (1 << COM1A1);											//Clear PWM on compare match while counting up, and set when counting down
	TCCR1A |= (1 << COM1B1);											//Same for B
	TCCR1A |= (1 << WGM11);												//Set a Phase correct ICR1 topped PWM signal
	TCCR1B |= (1 << WGM13);												//---^
	TCCR1B |= (1 << CS10);												//Enable without prescaler
	ICR1 = 63999;														//Set the top at 63999, this is 64000 steps aka 125Hz (250x full counter)
	OCR1A = 0;															//Start the compare registers at 0, no signal
	OCR1B = 0;															//---^
}

int motorDriver(struct rp6DataBP rp6Data){							//Calculate the PWM signal for the motors based on the struct data
	static uint64_t updateTimer = 0;									//Declare a timer to check update interval
	
	//Current dual motor values
	static int currentDriveDirection = 1;
	static int currentTurnDirection = 0;
	static uint32_t speedDifference = 0;
	static int64_t currentDriveSpeed = 0;
	//-------------------------
	
	//Current individual motor values
	static int leftMotorDirection = 1;
	static int rightMotorDirection = 1;
	
	static uint32_t leftMotorSpeed = 0;
	static uint32_t rightMotorSpeed = 0;
	//-------------------------
	
	//Update timer
	rp6Data.updateSpeed = rp6Data.updateSpeed * 1000;											//Multiply the update speed with 1000 to go from milliseconds to microseconds
	
	if(updateTimer > micros()){																	//Only execute motor update code if the timer has passed
		return 0;
	}else{
		updateTimer = micros() + rp6Data.updateSpeed;												//If the timer has passed, set new timer and execute the code
	}
	
	//Remap drive speed percentage
	rp6Data.driveSpeed = (rp6Data.driveSpeed * 25600) / 100;									//The given drive speed is a percentage, remap it to a PWM compare value (Max 25600)
	if(rp6Data.driveSpeed < rp6Data.driveSpeedThreshold){rp6Data.driveSpeed = 0;}				//If the speed is less than the threshold, the speed is set to 0 because the power is to low to drive
	
	//Check and change drive direction
	if(rp6Data.driveDirection != currentDriveDirection && currentDriveSpeed != 0){				//If the drive direction differs from what we are currently driving and we are not standing still
		rp6Data.driveSpeed = 0;																		//Set the requested speed to 0
	}else if(rp6Data.driveDirection != currentDriveDirection && currentDriveSpeed == 0){		//If the direction is wrong but we are standing still
		currentDriveDirection = rp6Data.driveDirection;												//Reverse the driving direction
		rp6Data.driveSpeed = 0;																		//And remain stationary for this update cycle
	}
	
	//Smoothly adjust current drive speed to requested drive speed
	if(rp6Data.accelerationRate >= 5000){														//If the acceleration rate has been set to 5000 or more, smooth acceleration is disabled and the current drive speed is set to the requested speed straight away
		currentDriveSpeed = rp6Data.driveSpeed;														//Set the current drive speed
	}else{																						//If a regular acceleration rate is set
		speedDifference = sqrt(pow((rp6Data.driveSpeed - currentDriveSpeed), 2));					//Calculate the speed difference (always positive)
	
		if(speedDifference < 2000){																	//If the difference is less than 2000
			currentDriveSpeed = rp6Data.driveSpeed;														//Set the current speed to the requested value
		}else{																						//If the difference is more than 2000
			if(rp6Data.driveSpeed - currentDriveSpeed < 0){												//Check if we need to accelerate or decelerate, if we need to decelerate
				if(currentDriveSpeed < rp6Data.driveSpeedThreshold){										//If the speed is less than the threshold
					currentDriveSpeed = rp6Data.driveSpeed;														//Set the speed to the requested value (Probably 0)
				}else{																						//If the current speed is higher than the threshold
					currentDriveSpeed -= rp6Data.accelerationRate;												//Decelerate with the set acceleration rate
				}
			}else{																						//If we need to accelerate
				if(currentDriveSpeed < rp6Data.driveSpeedThreshold){										//And we are still at a speed lower than the threshold
					currentDriveSpeed += rp6Data.driveSpeedThreshold;											//Speed up with the minimum threshold
				}else{																						//If we are at a speed higher than the threshold
					if(currentDriveSpeed < 7000){currentDriveSpeed += rp6Data.accelerationRate/4;}				//Accelerate slower when the speed is low
					else{currentDriveSpeed += rp6Data.accelerationRate;}										//Accelerate with the set acceleration rate
					if(currentDriveSpeed > rp6Data.driveSpeed){currentDriveSpeed = rp6Data.driveSpeed;}			//If we overshot the requested speed, set the current speed to the requested value (Can't be much of a difference)
				}
			}
		}
	}
	
	//Middle safety check
	if(currentDriveSpeed < 0){					//If the current drive speed is somehow lower than 0
		currentDriveSpeed = 0;						//Set it back to 0
	}else if(currentDriveSpeed > 25600){		//If it is higher than the max allowed value
		currentDriveSpeed = 25600;					//Set it back to the max allowed value
	}
	
	
	//Split motor drive direction and speed
	leftMotorDirection = currentDriveDirection;
	rightMotorDirection = currentDriveDirection;
	leftMotorSpeed = currentDriveSpeed;
	rightMotorSpeed = currentDriveSpeed;
	
	
	//Check turn direction
	if(rp6Data.turnDirection != currentTurnDirection){												//If the turn direction is changed
		
		leftMotorSpeed = currentDriveSpeed;																//Start by driving straight
		rightMotorSpeed = currentDriveSpeed;															//---^
		currentTurnDirection = rp6Data.turnDirection;													//Set the new turn direction
		updateTimer += 100000;																			//Add an extra delay to decrease wear and tear on the gears
		
	}else if(currentTurnDirection == -1){															//If the turn direction is -1, we go left
		
		if(currentDriveSpeed == 0){																		//If the speed is 0, we need to turn around our axle
			leftMotorDirection = 0;																			//Turn the left motor backwards
			rightMotorDirection = 1;																		//Turn the right motor forwards
			leftMotorSpeed = rp6Data.driveSpeedThreshold + 2000 + (rp6Data.turnRate / 2);					//Set the speed to minimal + a little extra for compensation + half the turn rate
			rightMotorSpeed = rp6Data.driveSpeedThreshold + 2000 + (rp6Data.turnRate / 2);					//---^
		}else{																							//If we are driving (Forward or backwards does not matter)
			if((currentDriveSpeed - rp6Data.turnRate) < (rp6Data.driveSpeedThreshold + 2000)){				//If the current drive speed minus the turn rate is lower than the threshold (with compensation)
				leftMotorSpeed = rp6Data.driveSpeedThreshold + 2000;											//Then the left motor is set on the threshold plus compensation
				if((leftMotorSpeed + (rp6Data.turnRate * 2)) > 25600){											//If the left motor speed plus twice the turn rate is more than the max allowed
					rightMotorSpeed = 25600;																		//Set the right to the max allowed
				}else{																							//If not
					rightMotorSpeed = (leftMotorSpeed + (rp6Data.turnRate * 2));									//Set the right motor speed on the left plus twice the turn rate
				}
			}else if((currentDriveSpeed + rp6Data.turnRate) > 25600){										//If the current speed plus the turn rate is more than allowed
				rightMotorSpeed = 25600;																		//Set the right motor to max speed
				if((rightMotorSpeed - (rp6Data.turnRate * 2)) < (rp6Data.driveSpeedThreshold + 2000)){			//If the right motor speed minus twice the turn rate is lower than the threshold plus compensation
					leftMotorSpeed = (rp6Data.driveSpeedThreshold + 2000);											//Set the left motor on the threshold plus compensation
				}else{																							//If not
					leftMotorSpeed = (rightMotorSpeed - (rp6Data.turnRate * 2));									//Set the left motor speed on the right speed minus twice the turn rate
				}
			}else{																							//If turning remains within the margins
				leftMotorSpeed = (currentDriveSpeed - rp6Data.turnRate);										//Set the left motor on the current speed minus the turn rate
				rightMotorSpeed = (currentDriveSpeed + rp6Data.turnRate);										//Set the right motor on the current speed plus the turn rate
			}
		}
		
	}else if(currentTurnDirection == 1){															//If the turn direction is 1, we go to the right
		
		if(currentDriveSpeed == 0){																		//If we stand still, we turn around our axle
			leftMotorDirection = 1;																			//Left motor forward
			rightMotorDirection = 0;																		//Right motor backward
			leftMotorSpeed = rp6Data.driveSpeedThreshold + 2000 + (rp6Data.turnRate / 2);					//Set the speed to minimal + a little extra for compensation + half the turn rate
			rightMotorSpeed = rp6Data.driveSpeedThreshold + 2000 + (rp6Data.turnRate / 2);					//---^
		}else{																							//If we are driving (Forward or backwards does not matter)
			if((currentDriveSpeed - rp6Data.turnRate) < (rp6Data.driveSpeedThreshold + 2000)){				//If the current drive speed minus the turn rate is lower than the threshold (with compensation)
				rightMotorSpeed = rp6Data.driveSpeedThreshold + 2000;											//Then the right motor is set on the threshold plus compensation
				if((rightMotorSpeed + (rp6Data.turnRate * 2)) > 25600){											//If the right motor speed plus twice the turn rate is more than the max allowed
					leftMotorSpeed = 25600;																			//Set the left to the max allowed
				}else{																							//If not
					leftMotorSpeed = (rightMotorSpeed + (rp6Data.turnRate * 2));									//Set the left motor speed on the left plus twice the turn rate
				}
			}else if((currentDriveSpeed + rp6Data.turnRate) > 25600){										//If the current speed plus the turn rate is more than allowed
				leftMotorSpeed = 25600;																			//Set the left motor to max speed
				if((leftMotorSpeed - (rp6Data.turnRate * 2)) < (rp6Data.driveSpeedThreshold + 2000)){			//If the left motor speed minus twice the turn rate is lower than the threshold plus compensation
					rightMotorSpeed = (rp6Data.driveSpeedThreshold + 2000);											//Set the right motor on the threshold plus compensation
				}else{																							//If not
					rightMotorSpeed = (leftMotorSpeed - (rp6Data.turnRate * 2));									//Set the right motor speed on the left speed minus twice the turn rate
				}
			}else{																							//If turning remains within the margins
				rightMotorSpeed = (currentDriveSpeed - rp6Data.turnRate);										//Set the right motor on the current speed minus the turn rate
				leftMotorSpeed = (currentDriveSpeed + rp6Data.turnRate);										//Set the left motor on the current speed plus the turn rate
			}
		}
		
	}
	
	//Slower left wheel correction
	if(currentDriveSpeed != 0){
		leftMotorSpeed += 750;
	}
	
	//Final safety check
	if(leftMotorSpeed > 27000){						//Check if we did not accidentally set a speed higher than allowed on the left motor
		leftMotorSpeed = 27000;							//If we did, set it back to its max.
	}
	if(rightMotorSpeed > 25600){					//Since the right motor is used to stabilize the left, it is allowed to go a little faster. But if it is even more than that
		rightMotorSpeed = 25600;						//Set it back to its max
	}
	
	
	//Set motor direction
	if(leftMotorDirection){
		PORTC &= 0b11111011;
	}else{
		PORTC |= 0b00000100;
	}
	if(rightMotorDirection){
		PORTC &= 0b11110111;
	}else{
		PORTC |= 0b00001000;
	}
	
	//Set the struct values so the Arduino can see what the actual values are
	arduinoData.actualDriveSpeed = currentDriveSpeed;
	arduinoData.actualLeftMotorSpeed = leftMotorSpeed;
	arduinoData.actualRightMotorSpeed = rightMotorSpeed;
	
	//Engage the motors
	OCR1A = rightMotorSpeed;						//Set the calculated value to the PWM compare to engage the right motor
	OCR1B = leftMotorSpeed;							//And do the same for the left one
	
	return 0;
}

//Bumpers
void init_bumpedData() {														//When the bumpers are pushed, motordriver uses bumpedData instead of rp6Data
	bumpedData.driveSpeed = 0;
	bumpedData.driveDirection = 1;
	bumpedData.turnDirection = 0;
	bumpedData.accelerationRate = 2000;
	bumpedData.turnRate = 2500;
	bumpedData.driveSpeedThreshold = 7000;
	bumpedData.updateSpeed = 200;
}

uint8_t bumperCheck() {
	
	static uint32_t bumperTimer = 0;											//Used to determine for how long the RP6 should drive backwards
	
	if (getBumpers() && !arduinoData.bumperFlag) {								//If one or both bumpers are pushed
		arduinoData.bumperFlag = 1;												//Set bumperFlag
		bumperTimer = micros();													//Update bumperTimer to current time
	}
		
	if (arduinoData.bumperFlag) {												//If bumperFlag is set
		
		if (micros() < bumperTimer + BUMPED_STOP_TIME) {						//Make an emergency stop for the time defined in BUMPED_STOP_TIME
			bumpedData.driveSpeed = 0;
			bumpedData.accelerationRate = 5000;
		} else if (micros() < bumperTimer + BUMPED_BACK_TIME) {					//Drive backwards for the time defined in BUMPED_BACK_TIME (includes BUMPED_STOP_TIME)
			bumpedData.driveSpeed = 30;											//The speed is always 30 when reacting to bumpers
			bumpedData.driveDirection = 0;
			bumpedData.accelerationRate = rp6Data.accelerationRate;				//accelerationRate is as set in rp6Data
		} else {
			arduinoData.bumperFlag = 0;											//When BUMPED_BACK_TIME has passed, reset bumperFlag
		}
	}
	
	return arduinoData.bumperFlag;												//Return whether bumperFlag is set or not
}
//------------------------------------------------