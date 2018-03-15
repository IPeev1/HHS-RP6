#include "i2c_mst.h"
#include <avr/io.h>
#include <stdlib.h>


void init_master() {
		TWSR = 0;
		// Set bit rate
		TWBR = ( ( F_CPU / SCL_frequentie ) - 16) / 2;
		TWCR = (1<<TWEN);
}

void ontvangen(uint8_t ad,uint8_t b[],uint8_t max) {
	uint8_t op[15];
	
	TWCR |= (1<<TWSTA);
	while(!(TWCR & (1<<TWINT)));
	op[0] = TWSR;

	TWDR=(ad<<1)+1;
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

	op[1] = TWSR;
	b[0]=TWDR;
	
	uint8_t tel=0;
	do{
		if(tel == max-1)
	    	TWCR=(1<<TWINT)|(1<<TWEN);
		else
	    	TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWEA);
			while(!(TWCR & (1<<TWINT)));
	op[tel] = TWSR;
	b[tel]=TWDR;
	}while(op[tel++] == 0x50);

	TWCR=(1<<TWINT)|(1<<TWSTO)|(1<<TWEN);

//   for(uint8_t i=0;i<tel;++i) {
//	 writeString("\n\r");writeInteger(op[i],16);
//	 writeString(" data ");writeInteger(b[i],10);
//   }

}

void verzenden(uint8_t ad,uint8_t b) {
//  uint8_t op[5];

	TWCR |= (1<<TWSTA);
	while(!(TWCR & (1<<TWINT)));
 //   op[0] = TWSR;
	TWDR=(ad<<1);
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
//    op[1] = TWSR;

	TWDR=b;
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
  //  op[2] = TWSR;

	TWCR=(1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
//	while(!(TWCR & (1<<TWINT)));
  //  for(uint8_t i=0;i<3;++i) {
		// writeString("\n\r");writeInteger(op[0],16);
		// writeString(" ");writeInteger(op[1],16);
		// writeString(" ");writeInteger(op[2],16);
}
