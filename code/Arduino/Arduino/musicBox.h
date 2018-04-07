/*
 * musicBox.h
 *
 * Created: 3/27/2018 12:55:48 AM
 *  Author: mc_he
 */ 


#ifndef MUSICBOX_H_
#define MUSICBOX_H_

#define BEEPER PINH3
#define BEEP_FREQ 400

void initBackBeep();									//Initializes Timer/Counter 1 in CTC mode, prescaler 256
int freqToOCR(float frequency);						//Converts a frequency in Hz to an OCR-compatible value


#endif /* MUSICBOX_H_ */