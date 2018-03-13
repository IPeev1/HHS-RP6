/*
 * Arduino1280_XBEE_LED_TOGGLE.c
 *
 * Created: 3/13/2018 2:02:25 PM
 * Author : mc_he
 */ 
#define F_CPU 16000000UL		//Clockspeed

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define BUILTIN_LED 7
#define DELAY_TIME 500			//Delay time in milliseconds

volatile char letter = 'H';		//Used to toggle LED, H = high, L = low

ISR(USART0_RX_vect) {
	
	letter = UDR0;				//letter gets value received
}

int main(void)
{
    void writeChar(char x);
	
	DDRB |= (1 << BUILTIN_LED);
	
	//USART initialization
	UCSR0A = 0x00;								
	UCSR0B |= (1 << RXCIE0 | 1 << RXEN0);		//Enable USART receiver, receiver interrupt
	UCSR0C |= (1 << UCSZ01 | 1 << UCSZ00);		//Asynchrounous USART, Parity none, 1 Stop bit, 8-bit character size
	UBRR0H = 00;
	UBRR0L = 103;								//Baudrate 9600
	
	sei();										//Enable interrupt routines
	
    while (1) 
    {
		writeChar('B');
		_delay_ms(DELAY_TIME);
		
		if (letter == 'H') {
			PORTB |= (1 << BUILTIN_LED);		//Turn on BUILTIN_LED
			
		} else if (letter == 'L') {
			PORTB &= ~(1 << BUILTIN_LED);		//Turn off BUILTIN_LED
		}
    }
}

void writeChar(char x) { //Sends char x over serial communication
	while(~UCSR0A & (1 << UDRE0)); //Wait until UDRE0 is set
	UDR0 = x;
}