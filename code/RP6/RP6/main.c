/*
 * RP6.c
 *
 * Created: 12-Mar-18 11:07:57
 * Author : Rens
 *			Sander
 *			Ivan
 *			Willem
 *			Matthijs
 */ 
//Defines
#define F_CPU 8000000

//Includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <util/delay.h>

//Global variables
uint8_t statLED[2] = {64, 1};
uint16_t motorEncoderLVal = 0;					//Segment count of the left motor encoder (Updated by interrupt)
uint16_t motorEncoderRVal = 0;					//Same for right encoder ---^

//Global motor control variables
int8_t globalDriveDirection;		// Value -1, 0 or 1
int8_t globalTurnDirection;		// Value -1, 0 or 1
int8_t globalDriveThrottle;			//value between 0 - 100


//Functions
void init_interrupt();							//Initialize global interrupts
void init_LED();								//Initialize the status LEDs

//Motor control functions
int driveSpeed();
int driveDirection();
int turnDirection();

//Micros function ------------------------------------
uint64_t micros();								//Keep track of the amount of microseconds passed since boot
ISR(TIMER0_OVF_vect);							//Interrupt for Timer0, for micros()
void init_micros();								//Configure Timer0
volatile uint64_t t0TotalOverflow;				//Track the total overflows
//----------------------------------------------------

//Motor functions ------------------------------------
void init_motor_io();
void init_motor_timer();
void init_motor_encoder();
void init_motor();
void enableMotorEncoder(int enable);
int motorDriver(int64_t driveSpeed, int driveDirection, int turnDirection);		//driveSpeed: 0 - 100 | driveDirection: 0 - 1 | turnDirection: -1, 0, 1
//----------------------------------------------------

//Main function
int main(void) {
	//Initialize all functions
	init_interrupt();
	init_micros();
	
	init_motor();
	init_LED();
	//-----------------------
	
	while(1){
		
		motorDriver(driveSpeed(), driveDirection(), globalTurnDirection);
	}
}

//Motor control functions
int driveSpeed() {
	
	if (globalDriveDirection) {	//If going forward or backwards
		return globalDriveThrottle;
	}
	return 0;						//If standing still
}


int driveDirection() {
	
	if (globalDriveDirection == 1) {	//If going forward
		return 1;
	}
	return 0;							//If going backwards or standing still
}

//Other functions
void init_interrupt(){
	sei();									//Enable global interrupts
}


void init_LED(){
	DDRB |= 0b10000011;
	DDRC |= 0b01110000;
}

//Micros function --------------------------------------
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
//------------------------------------------------------
//Motor functions --------------------------------------
void init_motor(){
	init_motor_io();		//Initialize the necessary ports
	init_motor_timer();		//Initialize the Phase correct PWM timer for the engines
	init_motor_encoder();	//Initialize the external interrupts for the motor encoders
}


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


void init_motor_encoder(){
	motorEncoderLVal = 0;						//Reset the motor encoder variable
	motorEncoderRVal = 0;						//---^
	MCUCR |= (1 << ISC00);						//Set interrupt to trigger on any logical change
	MCUCR |= (1 << ISC10);						//---^
	GICR |= (1 << INT0);						//Enable interrupt 0
	GICR |= (1 << INT1);						//Enable interrupt 1
}


ISR(INT0_vect){
	motorEncoderLVal++;							//Increase the encoder variable
}


ISR(INT1_vect){
	motorEncoderRVal++;							//Increase the encoder variable
}


void enableMotorEncoder(int enable){
	if(enable){									//If enable is set
		GICR |= (1 << INT0);						//Enable the external interrupt
		GICR |= (1 << INT1);						//---^
	}else{										//If not set
		GICR &= ~(1 << INT0);						//Disable the interrupt
		GICR &= ~(1 << INT1);						//---^
	}
}


int motorDriver(int64_t driveSpeed, int driveDirection, int turnDirection){
	//Motor settings
	static int		deviationCorrection		= 15;				//Drift correction
	static uint8_t	accelerationRate		= 30;				//Percentage to accelerate with
	static uint16_t	turnRate				= 3000;				//Intensity to turn with
	static uint16_t	driveSpeedThreshold		= 5000;				//Minimal power needed to actually start moving
	static uint32_t updateSpeed				= 200000;			//Interval time between updates
	//-------------
	
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
	if(updateTimer > micros()){														//Only execute motor update code if the timer has passed
		return 0;
	}else{
		updateTimer = micros() + updateSpeed;											//If the timer has passed, set new timer and execute the code
	}
	
	
	//Remap drive speed percentage
	driveSpeed = (driveSpeed * 25600) / 100;										//The given drive speed is a percentage, remap it to a PWM compare value (Max 25600)
	if(driveSpeed < driveSpeedThreshold){driveSpeed = 0;}								//If the speed is less than the threshold, the speed is set to 0 because the power is to low to drive --------- EDIT SUGGESTION: Rescale so that 1% gives the minimal amount of actual movement and 100% the max. Threshold and less doesn't participate ~Sander
	
	
	//Check and change drive direction
	if(driveDirection != currentDriveDirection && currentDriveSpeed != 0){			//If the drive direction differs from what we are currently driving and we are not standing still
		driveSpeed = 0;																	//Set the requested speed to 0
	}else if(driveDirection != currentDriveDirection && currentDriveSpeed == 0){	//If the direction is wrong but we are standing still
		currentDriveDirection = driveDirection;											//Reverse the driving direction
		driveSpeed = 0;																	//And remain stationary for this update cycle
	}
	
	
	//Smoothly adjust current drive speed to requested drive speed
	speedDifference = sqrt(pow((driveSpeed - currentDriveSpeed), 2));				//Calculate the speed difference (always positive)
	
	if(speedDifference < 2000){														//If the difference is less than 2000
		currentDriveSpeed = driveSpeed;													//Set the current speed to the requested value
	}else{																			//If the difference is more than 2000
		if(driveSpeed - currentDriveSpeed < 0){											//Check if we need to accelerate or decelerate, if we need to decelerate
			if(currentDriveSpeed < driveSpeedThreshold){									//If the speed is less than the threshold
				currentDriveSpeed = driveSpeed;													//Set the speed to the requested value (Probably 0)
			}else{																			//If the current speed is higher than 5000
				currentDriveSpeed -= ((currentDriveSpeed * accelerationRate)/100);				//Decelerate with a given percentage of the current speed, determined by accelerationRate
			}
		}else{																			//If we need to accelerate
			if(currentDriveSpeed < driveSpeedThreshold){									//And we are still at a speed lower than the threshold
				currentDriveSpeed += driveSpeedThreshold;										//Speed up with the minimum threshold
			}else{																			//If we are at a speed higher than the threshold
				currentDriveSpeed += ((currentDriveSpeed * accelerationRate)/100);				//Accelerate with a percentage of the current speed, determined by accelerationRate
				if(currentDriveSpeed > driveSpeed){currentDriveSpeed = driveSpeed;}				//If we overshot the requested speed, set the current speed to the requested value (Can't be much of a difference)
			}
		}
	}
	
	
	//Split motor drive direction and speed
	leftMotorDirection = currentDriveDirection;
	rightMotorDirection = currentDriveDirection;
	leftMotorSpeed = currentDriveSpeed;
	rightMotorSpeed = currentDriveSpeed;
	
	
	//Check turn direction
	if(turnDirection != currentTurnDirection){						//If the turn direction is changed
		leftMotorSpeed = currentDriveSpeed;								//Start by driving straight
		rightMotorSpeed = currentDriveSpeed;							//---^
		currentTurnDirection = turnDirection;							//Set the new turn direction
		if(currentTurnDirection == 0){									//If the new direction is 0(straight)
			enableMotorEncoder(1);											//Enable the encoders
		}else{															//If the new direction is not straight
			enableMotorEncoder(0);											//Disable the encoders
		}
		updateTimer += 100000;											//Add an extra delay to decrease wear and tear on the gears
	}else if(currentTurnDirection == -1){							//If the turn direction is -1, we go left
		if(currentDriveSpeed == 0){										//If the speed is 0, we need to turn around our axle
			leftMotorDirection = 0;											//Turn the left motor backwards
			rightMotorDirection = 1;										//Turn the right motor forwards
			leftMotorSpeed = driveSpeedThreshold + turnRate*2;				//Set the speed to minimal + twice the turn rate
			rightMotorSpeed = driveSpeedThreshold + turnRate*2;				//---^
		}else{															//If we are driving (Forward or backwards does not matter)
			leftMotorSpeed = driveSpeedThreshold;							//Set the left motor to minimal
			rightMotorSpeed += turnRate;									//Increase the right motor with the turn rate
		}
	}else if(currentTurnDirection == 0){							//Encoder crap
		if(motorEncoderLVal != motorEncoderRVal){
			if(motorEncoderLVal - motorEncoderRVal > 0){
				rightMotorSpeed += ( rightMotorSpeed * sqrt(pow((motorEncoderLVal - motorEncoderRVal), 2)) ) / 100;
			}else{
				rightMotorSpeed -= ( rightMotorSpeed * sqrt(pow((motorEncoderLVal - motorEncoderRVal), 2)) ) / 100;
			}
		}															//-------------
	}else if(currentTurnDirection == 1){							//If the turn direction is 1, we go to the right
		if(currentDriveSpeed == 0){										//If we stand still, we turn around our axle
			leftMotorDirection = 1;											//Left motor forward
			rightMotorDirection = 0;										//Right motor backward
			leftMotorSpeed = driveSpeedThreshold + turnRate*2;				//set motor speed to minimal + twice the turn rate
			rightMotorSpeed = driveSpeedThreshold + turnRate*2;				//---^
			}else{														//If we are driving (Forward or backwards does not matter)
			leftMotorSpeed += turnRate;										//Increase the left motor with the set turn rate
			rightMotorSpeed = driveSpeedThreshold;							//Set right motor to minimal
		}
	}
	
	
	//Deviation correction
	rightMotorSpeed += (rightMotorSpeed * deviationCorrection) / 100;		//Set a deviation correction on the right motor
	
	
	//Final safety check
	if(leftMotorSpeed > 25600){						//Check if we did not accidentally set a speed higher than allowed on the left motor
		leftMotorSpeed = 25600;							//If we did, set it back to its max
	}
	if(rightMotorSpeed > 30000){					//Since the right motor is used to stabilize the left, it is allowed to go a little faster. But if it is even more than that
		rightMotorSpeed = 30000;						//Set it back to its max
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
	
	
	//Engage the motors
	OCR1A = rightMotorSpeed;						//Set the calculated value to the PWM compare to engage the right motor
	OCR1B = leftMotorSpeed;							//And do the same for the left one
	
	return 0;
}
//------------------------------------------------------
