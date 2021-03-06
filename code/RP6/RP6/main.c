/*

De Haagse Hogeschool RP6 robot project (NSE)

Jaar 1 (2017 - 2018)

Created: 12-03-2018
Finished: 07-04-2018

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
#define F_CPU 8000000

//I2C
#define SCL 100000
#define DATASIZE 20
#define RP6_ADDRESS 3

//Bumpers
#define BUMPED_STOP_TIME 70000
#define BUMPED_BACK_TIME 1000000
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
volatile uint64_t t0TotalOverflow;				//Track the total overflows

//I2C
uint8_t sendData[DATASIZE];
uint8_t receiveData[DATASIZE];

//Structs (shared data between Arduino and RP6)
struct rp6DataBP {
	uint32_t	driveSpeed;				//value between 0 - 100
	int8_t		driveDirection;			//Value 0 or 1
	int8_t		turnDirection;			//Value -1, 0 or 1
	uint16_t	accelerationRate;		//Percentage to accelerate with					(Default: 30)
	uint16_t	turnRate;				//Intensity to turn with						(Default: 3000)
	uint16_t	driveSpeedThreshold;	//Minimal power needed to actually start moving	(Default: 5000)
	uint32_t	updateSpeed;			//Interval time between updates					(Default: 200000)
	uint8_t		enableBeeper;			//Set to 1 to enable reverse driving beeper		(Default: 1	(On))
	int64_t		compassAngle;			//Degrees from north given by compass
} rp6Data, bumpedData;
struct arduinoDataBP {
	uint16_t	bumperFlag;		//Segment count of the left motor encoder		(Updated by interrupt)
	uint16_t	actualDriveSpeed;		//Same for right encoder ---^
	uint16_t	actualLeftMotorSpeed;		//Distance driven by left motor
	uint16_t	actualRightMotorSpeed;		//right motor ---^
	uint16_t	totalDistance;			//Total distance driven by the robot
} arduinoData;
//------------------------------------------------

//Function declarations --------------------------
//General
void init_interrupt();							//Initialize global interrupts

//Micros
uint64_t micros();								//Keep track of the amount of microseconds passed since boot
ISR(TIMER0_OVF_vect);							//Interrupt for Timer0, for micros()
void init_micros();								//Configure Timer0

//I2C
void init_TWI();
void init_rp6Data();
void init_arduinoData();
ISR(TWI_vect);
void clearSendData();
void clearReceiveData();
void I2C_receiveInterpreter();
void rp6DataInterpreter();
void arduinoDataConstructor();

//Motor
void init_motor_io();
void init_motor_timer();
int motorDriver(struct rp6DataBP rp6Data);

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
		if (bumperCheck()) {
			motorDriver(bumpedData);
		} else {
			motorDriver(rp6Data);
		}
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
	TCCR0 |= (1 << CS00);			//Set a timer prescaler of '64'
	TCCR0 |= (1 << CS01);			//---^
	TIMSK |= (1 << TOIE0);			//Enable overflow interrupts
	TCNT0 = 0;						//Initialize the timer by setting it to 0
	t0TotalOverflow = 0;			//Initialize the overflow counter by setting it to 0
}

ISR(TIMER0_OVF_vect){						//When the internal timer 1 overflows and loops back to 0, this interrupt triggers
	t0TotalOverflow++;							//Increase the total overflow counter
}

uint64_t micros(){
	uint8_t currentTimer0Value = TCNT0;																				//Get the current value of the Timer 0 register
	uint64_t microsReturnValue = ((2048 * t0TotalOverflow) + (currentTimer0Value * 2048 / 256));					//Calculate the passed microseconds based on the total amount of overflows and the current timer value.
	return microsReturnValue;																						//Return the calculated value
}

//I2C
void init_TWI(){
	TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE);		//Enable TWI; Enable Acknowledge; Enable Interrupt
	TWSR = 0;											//No prescaling
	TWAR = (RP6_ADDRESS << 1);									//Set slave address
	TWBR = ((F_CPU / SCL) - 16) / 2;					//set SCL to 100kHz
	DDRC |= 0b00000011;
	PORTC |= 0b00000011;
}

void init_rp6Data(){
	rp6Data.driveSpeed = 0;
	rp6Data.driveDirection = 1;
	rp6Data.turnDirection = 0;
	rp6Data.accelerationRate = 2000;
	rp6Data.turnRate = 2500;
	rp6Data.driveSpeedThreshold = 7000;
	rp6Data.updateSpeed = 200;
	rp6Data.enableBeeper = 1;
}

void init_arduinoData(){
	arduinoData.bumperFlag = 0;
	arduinoData.actualDriveSpeed = 0;
	arduinoData.actualLeftMotorSpeed = 0;
	arduinoData.actualRightMotorSpeed = 0;
	arduinoData.totalDistance = 0;
}

ISR(TWI_vect){
	static int byteCounter = 0;
	switch(TWSR){
		case 0x60:
			clearReceiveData();
			byteCounter = 0;
		break;
		
		case 0x80:
			if(byteCounter < 20){
				receiveData[byteCounter] = TWDR;
				byteCounter++;
			}
		break;
		
		case 0xA0:
			I2C_receiveInterpreter();
		break;
		
		case 0xA8:
			arduinoDataConstructor();
			byteCounter = 0;
			TWDR = sendData[byteCounter];
		break;
		
		case 0xB8:
			byteCounter++;
			TWDR = sendData[byteCounter];
		break;
	}
	
	TWCR |= (1 << TWINT);
}

void clearSendData(){
	for(int i = 0; i < DATASIZE; i++){
		sendData[i] = 0;
	}
}

void clearReceiveData(){
	for(int i = 0; i < DATASIZE; i++){
		receiveData[i] = 0;
	}
}

void I2C_receiveInterpreter(){
	int dataSet = receiveData[0];
	switch(dataSet){
		case(1): rp6DataInterpreter(); break;
	}
}

void rp6DataInterpreter(){
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
	rp6Data.enableBeeper = receiveData[12];
	rp6Data.compassAngle = (receiveData[13] << 8) + receiveData[14];
}

void arduinoDataConstructor(){
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
	
	sendData[9] = (arduinoData.totalDistance >> 8);
	sendData[10] = arduinoData.totalDistance;
	
	for(int i = 11; i < DATASIZE; i++){
		sendData[i] = 0;
	}
}

//Motor
void init_motor_io(){
	DDRD |= 0b00110000;		//Set D5 and D4 on output, these are the motors
	DDRD &= 0b11110011;		//Set D3 and D2 on input, these are the encoders
	DDRC |= 0b00001100;		//Set C2 and C3 on output, these are the motor directions
}

void init_motor_timer(){
	TCCR1A |= (1 << COM1A1);		//Clear PWM on compare match while counting up, and set when counting down
	TCCR1A |= (1 << COM1B1);		//Same for B
	TCCR1A |= (1 << WGM11);			//Set a Phase correct ICR1 topped PWM signal
	TCCR1B |= (1 << WGM13);			//---^
	TCCR1B |= (1 << CS10);			//Enable without prescaler
	ICR1 = 63999;					//Set the top at 63999, this is 64000 steps aka 125Hz (250x full counter)
	OCR1A = 0;						//Start the compare registers at 0, no signal
	OCR1B = 0;						//---^
}

int motorDriver(struct rp6DataBP rp6Data){	
	static uint64_t updateTimer = 0;				//Declare a timer to check update interval
	
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
	rp6Data.updateSpeed = rp6Data.updateSpeed * 1000;
	if(updateTimer > micros()){														//Only execute motor update code if the timer has passed
		return 0;
	}else{
		updateTimer = micros() + rp6Data.updateSpeed;											//If the timer has passed, set new timer and execute the code
	}
	
	//Remap drive speed percentage
	rp6Data.driveSpeed = (rp6Data.driveSpeed * 25600) / 100;										//The given drive speed is a percentage, remap it to a PWM compare value (Max 25600)
	if(rp6Data.driveSpeed < rp6Data.driveSpeedThreshold){rp6Data.driveSpeed = 0;}								//If the speed is less than the threshold, the speed is set to 0 because the power is to low to drive --------- EDIT SUGGESTION: Rescale so that 1% gives the minimal amount of actual movement and 100% the max. Threshold and less doesn't participate ~Sander
	
	//Check and change drive direction
	if(rp6Data.driveDirection != currentDriveDirection && currentDriveSpeed != 0){			//If the drive direction differs from what we are currently driving and we are not standing still
		rp6Data.driveSpeed = 0;																	//Set the requested speed to 0
	}else if(rp6Data.driveDirection != currentDriveDirection && currentDriveSpeed == 0){	//If the direction is wrong but we are standing still
		currentDriveDirection = rp6Data.driveDirection;											//Reverse the driving direction
		rp6Data.driveSpeed = 0;																	//And remain stationary for this update cycle
	}
	
	if(rp6Data.accelerationRate >= 5000){
		currentDriveSpeed = rp6Data.driveSpeed;
	}else{
		//Smoothly adjust current drive speed to requested drive speed
		speedDifference = sqrt(pow((rp6Data.driveSpeed - currentDriveSpeed), 2));				//Calculate the speed difference (always positive)
	
		if(speedDifference < 2000){														//If the difference is less than 2000
			currentDriveSpeed = rp6Data.driveSpeed;													//Set the current speed to the requested value
		}else{																			//If the difference is more than 2000
			if(rp6Data.driveSpeed - currentDriveSpeed < 0){											//Check if we need to accelerate or decelerate, if we need to decelerate
				if(currentDriveSpeed < rp6Data.driveSpeedThreshold){									//If the speed is less than the threshold
					currentDriveSpeed = rp6Data.driveSpeed;													//Set the speed to the requested value (Probably 0)
				}else{																			//If the current speed is higher than 5000
					currentDriveSpeed -= rp6Data.accelerationRate;							//Decelerate with a given percentage of the current speed, determined by accelerationRate
				}
			}else{																			//If we need to accelerate
				if(currentDriveSpeed < rp6Data.driveSpeedThreshold){									//And we are still at a speed lower than the threshold
					currentDriveSpeed += rp6Data.driveSpeedThreshold;										//Speed up with the minimum threshold
				}else{																			//If we are at a speed higher than the threshold
					if(currentDriveSpeed < 7000){currentDriveSpeed += rp6Data.accelerationRate/4;}
					else{currentDriveSpeed += rp6Data.accelerationRate;}							//Accelerate with a percentage of the current speed, determined by accelerationRate
					if(currentDriveSpeed > rp6Data.driveSpeed){currentDriveSpeed = rp6Data.driveSpeed;}				//If we overshot the requested speed, set the current speed to the requested value (Can't be much of a difference)
				}
			}
		}
	}
	
	
	if(currentDriveSpeed < 0){
		currentDriveSpeed = 0;
	}else if(currentDriveSpeed > 25600){
		currentDriveSpeed = 25600;
	}
	
	
	//Split motor drive direction and speed
	leftMotorDirection = currentDriveDirection;
	rightMotorDirection = currentDriveDirection;
	leftMotorSpeed = currentDriveSpeed;
	rightMotorSpeed = currentDriveSpeed;
	
	
	//Check turn direction
	if(rp6Data.turnDirection != currentTurnDirection){						//If the turn direction is changed
		leftMotorSpeed = currentDriveSpeed;								//Start by driving straight
		rightMotorSpeed = currentDriveSpeed;							//---^
		currentTurnDirection = rp6Data.turnDirection;							//Set the new turn direction
		updateTimer += 100000;											//Add an extra delay to decrease wear and tear on the gears
	}else if(currentTurnDirection == -1){							//If the turn direction is -1, we go left
		if(currentDriveSpeed == 0){										//If the speed is 0, we need to turn around our axle
			leftMotorDirection = 0;											//Turn the left motor backwards
			rightMotorDirection = 1;										//Turn the right motor forwards
			leftMotorSpeed = rp6Data.driveSpeedThreshold + 2000 + (rp6Data.turnRate / 2);				//Set the speed to minimal + twice the turn rate
			rightMotorSpeed = rp6Data.driveSpeedThreshold + 2000 + (rp6Data.turnRate / 2);				//---^
		}else{															//If we are driving (Forward or backwards does not matter)
			
			if((currentDriveSpeed - rp6Data.turnRate) < (rp6Data.driveSpeedThreshold + 2000)){
				leftMotorSpeed = rp6Data.driveSpeedThreshold + 2000;
				if((leftMotorSpeed + (rp6Data.turnRate * 2)) > 25600){
					rightMotorSpeed = 25600;
				}else{
					rightMotorSpeed = (leftMotorSpeed + (rp6Data.turnRate * 2));
				}
			}else if((currentDriveSpeed + rp6Data.turnRate) > 25600){
				rightMotorSpeed = 25600;
				if((rightMotorSpeed - (rp6Data.turnRate * 2)) < (rp6Data.driveSpeedThreshold + 2000)){
					leftMotorSpeed = (rp6Data.driveSpeedThreshold + 2000);
				}else{
					leftMotorSpeed = (rightMotorSpeed - (rp6Data.turnRate * 2));
				}
			}else{
				leftMotorSpeed = (currentDriveSpeed - rp6Data.turnRate);
				rightMotorSpeed = (currentDriveSpeed + rp6Data.turnRate);
			}
		}
	}else if(currentTurnDirection == 1){							//If the turn direction is 1, we go to the right
		if(currentDriveSpeed == 0){										//If we stand still, we turn around our axle
			leftMotorDirection = 1;											//Left motor forward
			rightMotorDirection = 0;										//Right motor backward
			leftMotorSpeed = rp6Data.driveSpeedThreshold + 2000 + (rp6Data.turnRate / 2);				//set motor speed to minimal + twice the turn rate
			rightMotorSpeed = rp6Data.driveSpeedThreshold + 2000 + (rp6Data.turnRate / 2);				//---^
		}else{														//If we are driving (Forward or backwards does not matter)
			
			if((currentDriveSpeed - rp6Data.turnRate) < (rp6Data.driveSpeedThreshold + 2000)){
				rightMotorSpeed = rp6Data.driveSpeedThreshold + 2000;
				if((rightMotorSpeed + (rp6Data.turnRate * 2)) > 25600){
					leftMotorSpeed = 25600;
				}else{
					leftMotorSpeed = (rightMotorSpeed + (rp6Data.turnRate * 2));
				}
			}else if((currentDriveSpeed + rp6Data.turnRate) > 25600){
				leftMotorSpeed = 25600;
				if((leftMotorSpeed - (rp6Data.turnRate * 2)) < (rp6Data.driveSpeedThreshold + 2000)){
					rightMotorSpeed = (rp6Data.driveSpeedThreshold + 2000);
				}else{
					rightMotorSpeed = (leftMotorSpeed - (rp6Data.turnRate * 2));
				}
			}else{
				rightMotorSpeed = (currentDriveSpeed - rp6Data.turnRate);
				leftMotorSpeed = (currentDriveSpeed + rp6Data.turnRate);
			}
		}
	}
	
	//Slower left wheel correction
	if(currentDriveSpeed != 0){
		leftMotorSpeed += 750;
	}
	
	//Final safety check
	if(leftMotorSpeed > 27000){						//Check if we did not accidentally set a speed higher than allowed on the left motor
		leftMotorSpeed = 27000;							//If we did, set it back to its max
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
	
	arduinoData.actualDriveSpeed = currentDriveSpeed;
	arduinoData.actualLeftMotorSpeed = leftMotorSpeed;
	arduinoData.actualRightMotorSpeed = rightMotorSpeed;
	
	//Engage the motors
	OCR1A = rightMotorSpeed;						//Set the calculated value to the PWM compare to engage the right motor
	OCR1B = leftMotorSpeed;							//And do the same for the left one
	
	return 0;
}

//Bumpers
void init_bumpedData() {
	bumpedData.driveSpeed = 0;
	bumpedData.driveDirection = 1;
	bumpedData.turnDirection = 0;
	bumpedData.accelerationRate = 2000;
	bumpedData.turnRate = 2500;
	bumpedData.driveSpeedThreshold = 7000;
	bumpedData.updateSpeed = 200;
	bumpedData.enableBeeper = 1;
}

uint8_t bumperCheck() {
	
	static uint32_t bumperTimer = 0; //Used to determine how long the RP6 drives backwards
	//static uint8_t enable = 0; //if 1, RP6 drives backwards
	
	if (getBumpers() && !arduinoData.bumperFlag) { //If one or both bumpers are pushed
		arduinoData.bumperFlag = 1;
		bumperTimer = micros();
	}
		
	if (arduinoData.bumperFlag) {
		
		if (micros() < bumperTimer + BUMPED_STOP_TIME) {
			bumpedData.driveSpeed = 0;
			bumpedData.accelerationRate = 5000;
		} else if (micros() < bumperTimer + BUMPED_BACK_TIME) {
			bumpedData.driveSpeed = 30;
			bumpedData.driveDirection = 0;
			bumpedData.accelerationRate = rp6Data.accelerationRate;
		} else {
			arduinoData.bumperFlag = 0;
		}
	}
	
	return arduinoData.bumperFlag;
}
//------------------------------------------------