/*
 * ultrasonicSensor.h
 *
 * Created: 3/22/2018 10:57:07 AM
 *  Author: mc_he
 */ 


#ifndef ULTRASONICSENSOR_H_
#define ULTRASONICSENSOR_H_


void initTimer();
int ultrasonicSensor();
int cyclesToMm(unsigned long cycles);
void printUltrasonicSensorDistance();

#endif /* ULTRASONICSENSOR_H_ */