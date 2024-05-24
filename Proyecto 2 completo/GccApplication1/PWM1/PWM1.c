/*
 * PWM1.c
 *
 * Created: 16/04/2024 10:15:04 a. m.
 *  Author: Eber Alexander
 */ 

#include "PWM1.h"

uint16_t topVal = 0;
uint16_t i, j;		//transition variables

//(channel <- channelA/channelB, inverted <- yes/nop)
void channel(uint8_t setChannel, uint8_t inverted){
	switch(setChannel){
		case 1:
			if(inverted == 1){
				TCCR1A |= (1 << COM1A0) | (1 << COM1A1);	//channel A inverted
			}else{
				TCCR1A |= (1 << COM1A1);					//channel A no inverted
			}
		break;
		
		case 2:
			if(inverted == 1){
				TCCR1A |= (1 << COM1B0) | (1 << COM1B1);	//channelB inverted
			}else{
				TCCR1A |= (1 << COM1B1);					//channelB no inverted
			}
		break;
	}
}

/*Settings for Fast PWM 1, 16 bits, use it just one time
(inverted <- yes/nop, modePWM <- normal/settedUp, prescaler <- 1,8,64,256,1024)
*/
void initFastPWM1(uint8_t modePWM, uint16_t prescaler){
	//initialize register timer1
	TCCR1A = 0;
	TCCR1B = 0;
	
	switch (prescaler)
	{
		case 8:
			TCCR1B |= (1 << CS11);					//prescaler 8
		break;
		
		case 1024:
			TCCR1B |= (1 << CS10) | (1 << CS12);	//prescaler 1024
		break;
	}
	
	//selecting mode of operation
	switch (modePWM)
	{
		case normal:
			//fast PWM 10-bit resolution
			TCCR1A |= (1 << WGM11) | (1 << WGM10);
			TCCR1B |= (1 << WGM12);
		break;
		
		case settedUp:
			//fast PWM with ICR1 as TOP
			TCCR1A |= (1 << WGM11);
			TCCR1B |= (1 << WGM12) | (1 << WGM13);
			topVal = 1;		//set if it's required a top value
		break;
	}
	
}

//topValue, used if mode is settedUp
void topValue(uint16_t top){
	if (topVal == 1)	{
		ICR1 = top;		//set top value
	}else{
		topVal = 0;
	}
}

//conversion for servos, mapping values
void convertServo(uint16_t analogIn, uint8_t selChannel){
	switch(selChannel){
		case 1:
			i = analogIn;
			j = (200/12)*i+1000;
			OCR1A = maping(analogIn,0,255,1000,2000);
		break;
		
		case 2:
			i = analogIn;
			j = (200/12)*i+1000;
			OCR1B = maping(analogIn,0,255,1000,2000);
		break;
	}
}

float maping(float x, float in_min, float in_max, float out_min, float out_max){
	return ((x - in_min)*(out_max - out_min)/(in_max - in_min)) + out_min;
}
