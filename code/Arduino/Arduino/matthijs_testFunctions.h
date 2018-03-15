/*
 * matthijs_testFunctions.h
 *
 * Created: 3/14/2018 10:12:38 PM
 *  Author: mc_he
 */ 


#ifndef MATTHIJS_TESTFUNCTIONS_H_
#define MATTHIJS_TESTFUNCTIONS_H_


void writeChar(char x);
void writeString(char st[]);
void writeInt(int i);

void testTransmitUSART(char charToSend, int intToSend);
void globalVariablesTransmitUSART(int direction, int turn, int throttle);


#endif /* MATTHIJS_TESTFUNCTIONS_H_ */