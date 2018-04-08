/*
 * USART.h
 *
 * Created: 3/14/2018 10:12:38 PM
 *  Author: mc_he
 */ 


#ifndef USART_H_
#define USART_H_


void writeChar(char x);
void writeString(char st[]);
void writeInt(int i);

void testTransmitUSART(char charToSend, int intToSend);
void globalVariablesTransmitUSART(int direction, int turn, int throttle);


#endif /* USART_H_ */