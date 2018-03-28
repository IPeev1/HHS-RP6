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
#define TRUE 0xFF;
#define FALSE 0;

//Includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <util/delay.h>
#include "i2c.c"

//Global variables
uint8_t statLED[2] = {64, 1};

uint64_t I2CsyncTimer = 0;
uint32_t syncSpeed = 5000000;

//Functions
void init_interrupt();							//Initialize global interrupts
void init_LED();								//Initialize the status LEDs

//I2C functions ------------------
ISR(TWI_vect){
	slaaftwi();
}
uint8_t data_ont[20];
volatile uint8_t data_flag = FALSE;
volatile uint8_t databyte = 0x33;

void ontvangData(uint8_t [],uint8_t);
uint8_t verzendByte();
//I2C receive
void init_rp6Data();
void I2C_receiveInterpreter(uint8_t I2Cdata[]);
void rp6DataInterpreter(uint8_t I2Cdata[]);
//I2C send
void init_arduinoData();
void arduinoDataConstructor();
void I2C_sendArray(uint8_t I2Cdata[]);
//----------------------------------------------------
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
int motorDriver();
//----------------------------------------------------

//Global structs
struct rp6DataBP {
	int8_t		driveSpeed;				//value between 0 - 100
	int8_t		driveDirection;			//Value 0 or 1
	int8_t		turnDirection;			//Value -1, 0 or 1
	uint8_t		accelerationRate;		//Percentage to accelerate with					(Default: 30)
	uint16_t	turnRate;				//Intensity to turn with						(Default: 3000)
	uint16_t	driveSpeedThreshold;	//Minimal power needed to actually start moving	(Default: 5000)
	uint32_t	updateSpeed;			//Interval time between updates					(Default: 200000)
	uint8_t		enableBeeper;			//Set to 1 to enable reverse driving beeper		(Default: 1	(On))
} rp6Data;


struct arduinoDataBP {
	uint16_t	motorEncoderLVal;		//Segment count of the left motor encoder		(Updated by interrupt)
	uint16_t	motorEncoderRVal;		//Same for right encoder ---^
} arduinoData;


////temp usart shit
#define BAUDRATE		38400
#define UBRR_BAUD	(((long)F_CPU/((long)16 * BAUDRATE))-1)
#define resetData()  for(uint8_t i=0;i<20;++i) data[i]=0
void initUSART() {

	UBRRH = UBRR_BAUD >> 8;
	UBRRL = (uint8_t) UBRR_BAUD;
	UCSRA = 0x00;
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
	UCSRB = (1 << TXEN) | (1 << RXEN);
}


void writeChar(char ch)
{
	while (!(UCSRA & (1<<UDRE)));
	UDR = (uint8_t)ch;
}


void writeString(char *string)
{
	while(*string)
	writeChar(*string++);
}


void writeInteger(int16_t number, uint8_t base)
{
	char buffer[17];
	itoa(number, &buffer[0], base);
	writeString(&buffer[0]);
}
///------

//Main function
int main(void) {
	//Initialize all functions
	init_interrupt();
	init_micros();
	
	init_motor();
	init_LED();
	
	init_i2c_slave(8);
	
	init_rp6Data();
	init_arduinoData();
	
	initUSART();
	//-----------------------
	writeString("Start!");

	while(1){
		writeString("Start 2!");
		_delay_ms(250);
		writeString("Start 3!");
		writeInteger(rp6Data.driveSpeed, 10);
		writeString("Start 4!");
		writeString("\n");
		writeInteger(rp6Data.driveDirection, 10);
		writeString("\n");
		writeInteger(rp6Data.turnDirection, 10);
		writeString("\n");
		writeInteger(rp6Data.accelerationRate, 10);
		writeString("\n");
		writeInteger(rp6Data.turnRate, 10);
		writeString("\n");
		writeInteger(rp6Data.driveSpeedThreshold, 10);
		writeString("\n");
		writeInteger(rp6Data.updateSpeed, 10);
		writeString("\n");
		writeInteger(rp6Data.enableBeeper, 10);
		writeString("\n-------------------------\n");
	}
	
	I2CsyncTimer = micros();
	int temp = 0;
	
	while(1){
		_delay_ms(1000);
		writeInteger(rp6Data.driveSpeed, 10);
		writeString("\n");
		
		rp6Data.accelerationRate = 30;
		rp6Data.turnRate = 3000;
		rp6Data.driveSpeedThreshold = 5000;
		rp6Data.updateSpeed = 200000;
		rp6Data.enableBeeper = 1;
		
		if(I2CsyncTimer < micros()){
			//arduinoDataConstructor();
			I2CsyncTimer = micros() + syncSpeed;
			
			temp++;
			if(temp > 1){temp = 0;}
				
			switch(temp){
				case(0):
				rp6Data.driveDirection = 1;
				rp6Data.driveSpeed = 50;
				rp6Data.turnDirection = 0;
				writeInteger(temp, 10);
				writeString("\n");
				writeInteger(rp6Data.driveSpeed, 10);
				writeString("\n");
				break;
				case(1):
				rp6Data.driveDirection = 0;
				rp6Data.driveSpeed = 100;
				rp6Data.turnDirection = 0;
				writeInteger(temp, 10);
				writeString("\n");
				writeInteger(rp6Data.driveSpeed, 10);
				writeString("\n");
				break;
			}
		}
		
		motorDriver();
	}
}

//I2C control functions
void ontvangData(uint8_t data[],uint8_t tel){
	for(int i=0;i<tel;++i)
	data_ont[i]=data[i];
	I2C_receiveInterpreter(data_ont);
}


uint8_t verzendByte() {
	return databyte++;
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
//I2C functions receive --------------------------------
void init_rp6Data(){
	rp6Data.driveSpeed = 0;
	rp6Data.driveDirection = 1;
	rp6Data.turnDirection = 0;
	rp6Data.accelerationRate = 30;
	rp6Data.turnRate = 3000;
	rp6Data.driveSpeedThreshold = 5000;
	rp6Data.updateSpeed = 200000;
	rp6Data.enableBeeper = 1;
}


void I2C_receiveInterpreter(uint8_t I2Cdata[]){
	//int dataSet = I2Cdata[0];
	//switch(dataSet){
	//	case(1): rp6DataInterpreter(I2Cdata); break;
	//}
}


void rp6DataInterpreter(uint8_t I2Cdata[]){
	if(I2Cdata[2]-1 == 0){
		rp6Data.driveSpeed = 0;
	}else{
		rp6Data.driveSpeed = I2Cdata[1];
	}
	
	if(I2Cdata[2] < 1){
		rp6Data.driveDirection = 0;
	}else{
		rp6Data.driveDirection = 1;
	}
	
	rp6Data.turnDirection = I2Cdata[3]-1;
	rp6Data.accelerationRate = I2Cdata[4];
	rp6Data.turnRate = I2Cdata[5] * 8000 / 255;
	rp6Data.driveSpeedThreshold = I2Cdata[6] * 6000 / 255;
	rp6Data.updateSpeed = I2Cdata[7] * 2000;
	rp6Data.enableBeeper = I2Cdata[8];
}

//I2C functions send -----------------------------------
void init_arduinoData(){
	arduinoData.motorEncoderLVal = 0;
	arduinoData.motorEncoderRVal = 0;
}


void arduinoDataConstructor(){
	uint8_t I2Cdata[20];
	
	I2Cdata[0] = 1;
	I2Cdata[1] = arduinoData.motorEncoderLVal * 255 / 30000;
	I2Cdata[2] = arduinoData.motorEncoderRVal * 255 / 30000;
	
	for(int i = 3; i <= 19; i++){
		I2Cdata[i] = 0;
	}
	
	I2C_sendArray(I2Cdata);
}


void I2C_sendArray(uint8_t I2Cdata[]){
	for(int i = 0; i <= 19; i++){
		//verzenden(8, I2Cdata[i]);
	}
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
	arduinoData.motorEncoderLVal = 0;				//Reset the motor encoder variable
	arduinoData.motorEncoderRVal = 0;				//---^
	MCUCR |= (1 << ISC00);						//Set interrupt to trigger on any logical change
	MCUCR |= (1 << ISC10);						//---^
	GICR |= (1 << INT0);						//Enable interrupt 0
	GICR |= (1 << INT1);						//Enable interrupt 1
}


ISR(INT0_vect){
	arduinoData.motorEncoderLVal++;							//Increase the encoder variable
}


ISR(INT1_vect){
	arduinoData.motorEncoderRVal++;							//Increase the encoder variable
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


int motorDriver(){
	/*
	localDriveSpeed = rp6Data.driveSpeed;
	rp6Data.driveDirection;
	rp6Data.turnDirection;
	rp6Data.accelerationRate;
	rp6Data.turnRate;
	rp6Data.driveSpeedThreshold;
	rp6Data.updateSpeed;
	rp6Data.enableBeeper;
	*/
	//Motor settings
	static int		deviationCorrection		= 15;				//Drift correction
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
	
	
	//Smoothly adjust current drive speed to requested drive speed
	speedDifference = sqrt(pow((rp6Data.driveSpeed - currentDriveSpeed), 2));				//Calculate the speed difference (always positive)
	
	if(speedDifference < 2000){														//If the difference is less than 2000
		currentDriveSpeed = rp6Data.driveSpeed;													//Set the current speed to the requested value
	}else{																			//If the difference is more than 2000
		if(rp6Data.driveSpeed - currentDriveSpeed < 0){											//Check if we need to accelerate or decelerate, if we need to decelerate
			if(currentDriveSpeed < rp6Data.driveSpeedThreshold){									//If the speed is less than the threshold
				currentDriveSpeed = rp6Data.driveSpeed;													//Set the speed to the requested value (Probably 0)
			}else{																			//If the current speed is higher than 5000
				currentDriveSpeed -= ((currentDriveSpeed * rp6Data.accelerationRate)/100);				//Decelerate with a given percentage of the current speed, determined by accelerationRate
			}
		}else{																			//If we need to accelerate
			if(currentDriveSpeed < rp6Data.driveSpeedThreshold){									//And we are still at a speed lower than the threshold
				currentDriveSpeed += rp6Data.driveSpeedThreshold;										//Speed up with the minimum threshold
			}else{																			//If we are at a speed higher than the threshold
				currentDriveSpeed += ((currentDriveSpeed * rp6Data.accelerationRate)/100);				//Accelerate with a percentage of the current speed, determined by accelerationRate
				if(currentDriveSpeed > rp6Data.driveSpeed){currentDriveSpeed = rp6Data.driveSpeed;}				//If we overshot the requested speed, set the current speed to the requested value (Can't be much of a difference)
			}
		}
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
			leftMotorSpeed = rp6Data.driveSpeedThreshold + rp6Data.turnRate*2;				//Set the speed to minimal + twice the turn rate
			rightMotorSpeed = rp6Data.driveSpeedThreshold + rp6Data.turnRate*2;				//---^
		}else{															//If we are driving (Forward or backwards does not matter)
			leftMotorSpeed = rp6Data.driveSpeedThreshold;							//Set the left motor to minimal
			rightMotorSpeed += rp6Data.turnRate;									//Increase the right motor with the turn rate
		}
	}else if(currentTurnDirection == 0){							//Encoder crap
		if(arduinoData.motorEncoderLVal != arduinoData.motorEncoderRVal){
			if(arduinoData.motorEncoderLVal - arduinoData.motorEncoderRVal > 0){
				rightMotorSpeed += ( rightMotorSpeed * sqrt(pow((arduinoData.motorEncoderLVal - arduinoData.motorEncoderRVal), 2)) ) / 100;
			}else{
				rightMotorSpeed -= ( rightMotorSpeed * sqrt(pow((arduinoData.motorEncoderLVal - arduinoData.motorEncoderRVal), 2)) ) / 100;
			}
		}															//-------------
	}else if(currentTurnDirection == 1){							//If the turn direction is 1, we go to the right
		if(currentDriveSpeed == 0){										//If we stand still, we turn around our axle
			leftMotorDirection = 1;											//Left motor forward
			rightMotorDirection = 0;										//Right motor backward
			leftMotorSpeed = rp6Data.driveSpeedThreshold + rp6Data.turnRate*2;				//set motor speed to minimal + twice the turn rate
			rightMotorSpeed = rp6Data.driveSpeedThreshold + rp6Data.turnRate*2;				//---^
			}else{														//If we are driving (Forward or backwards does not matter)
			leftMotorSpeed += rp6Data.turnRate;										//Increase the left motor with the set turn rate
			rightMotorSpeed = rp6Data.driveSpeedThreshold;							//Set right motor to minimal
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
