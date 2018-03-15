#include "i2c.h"

//volatile TWIMode toestand =  Ready;


#define SCL_frequentie 100000

#define BAUDRATE		38400  
#define UBRR_BAUD	(((long)F_CPU/((long)16 * BAUDRATE))-1)
#define resetData()  for(uint8_t i=0;i<20;++i) data[i]=0



void (*ontfunc) (uint8_t[],uint8_t);
uint8_t (*verfunc) ();

void init_i2c_slave(uint8_t ad) {
	
		TWSR = 0;
		TWBR = ((F_CPU / SCL_frequentie) - 16) / 2;
		TWCR = (1 << TWIE) | (1 << TWEN) | (1 << TWEA);
		TWAR = ad<<1;
}

void slaaftwi() {	
		static uint8_t data[40];
		static uint8_t teller=0;
	switch(TWSR) {
		case 0x10:
		case 0x08:	
			break;
		
		case 0x60:

		  teller=0;

		  break;
		case 0x68:

		  break;
		case 0x80:
		  data[teller++] = TWDR;

		  break;
		case 0xA0:
		  ontfunc(data,teller);
		  resetData();
		  break;
		case 0xA8:
		  teller=0;
		  TWDR=verfunc();
		  break;
		case 0xB8: 
    	  TWDR=verfunc();
		  break;
		case 0xC0:   //NACK
		   break;
		case 0xC8:
		break;		   	
	}	
	 TWCR |= (1<<TWINT);    // Clear TWINT Flag	 
}


void init_i2c_ontvang( void (*ontvanger) (uint8_t [],uint8_t)) {
	ontfunc=ontvanger;
}

void init_i2c_verzend( uint8_t (*verzender) ()) {
	verfunc=verzender;
}
