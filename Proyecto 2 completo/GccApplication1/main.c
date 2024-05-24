/*
 * P2, proob.c
 *
 * Created: 21/05/2024 16:57:09
 * Author : Alex Cuxe
 */ 

//Importar librerías
#define F_CPU 16000000
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include "PWM1/PWM1.h"
#include "PWM2/PWM2.h"

//definición de uso para pines
#define adelanteDC	PINB0
#define servo1		PINB1
#define servo2		PINB2
#define velDC		PINB3
#define atrasDC		PINB4
#define potServo1	PINC0
#define potServo2	PINC1
#define potServo3	PINC2
#define potVelDC	PINC3
#define bridge1		PIND2
#define servo3		PIND3
#define bridge2		PIND4
#define led1		PIND5
#define led2		PIND6
#define savePos		PIND7

//definición de constantes
#define manualMode	1
#define eepromMode	2
#define usartMode	3
#define address1	0x06
#define address2	0x05
#define address3	0x02
#define address4	0x03

//definición de variables
volatile char bufferRX;		//it is volatile 'cuase it could change anytime
uint8_t count = 0;
uint8_t dir = 0;
uint8_t counterPos = 1;
uint8_t currentMode = manualMode;
uint8_t answer2 = 0;
uint8_t setServo1 = 0;
uint8_t setServo2 = 0;
uint8_t setServo3 = 0;
uint8_t setVel = 0;
uint8_t newAction = 0;
//uint8_t EEMEM eeprom_data;
//uint8_t x eeprom_data1;
uint8_t pos1 = 0;
uint8_t pos2 = 0;
uint8_t pos3 = 0;
uint8_t velMotor = 0;
uint8_t readData = 0;

//definicion de funciones
void initADC(void);
void initPCINT(void);
void initUART9600(void);
void writeText(char* text);
void menu(void);
void sentChar(void);



int main(void){
	cli();	//clear interruptions
	//Settings for every pin, inputs and outputs
	
	//Settings for PORTB
	PORTB |= (1 << adelanteDC) | (1 << atrasDC);	//inputs with pull-up
	DDRB &= ~( (1 << adelanteDC) | (1 << atrasDC) );
	DDRB |= (1 << servo1) | (1 << servo2);
	
	
	//settings for PORTC, entire port as input
	DDRC = 0;
	
	//settings for PORTD
	PORTD |= (1 << savePos);	//input with pull-up
	DDRD &= ~(1 << savePos);
	DDRD |= (1 << bridge1) | (1 << bridge2) | (1 <<  led1) | (1 << led2);
	
	//init libraries and other settings
	initADC();
	initPCINT();
	
	initUART9600();
	initFastPWM1(settedUp, 8);	//init PWM1
	channel(channelA, nop);
	channel(channelB, nop);
	topValue(39999);
	
	initPWM2A(no_invertido, 1024);
	initPWM2B(no_invertido, 1024);
	
	sei();	//Enable interruptions
	
	//confirmation message
	writeText("Hello world");
	menu();
	
	//Initializing conversion for ADC module
	ADCSRA |= (1 << ADSC);
	
	//It shows the number of the current mode, MANUAL MODE
	PORTD |= (1 << led1);
	PORTD &= ~(1 <<  led2);
	
    while (1) 
    {
    }
}











//*****************************************************************************
//                            Modules and Functions
//*****************************************************************************

void initADC(void){
	ADMUX = 0;
	//Vref = AVcc = 5Vs
	ADMUX |= (1 << REFS0);
	ADMUX &= ~(1 << REFS1);
	
	ADMUX |= (1 << ADLAR);	//left adjust, for using ADCH
	
	ADCSRA = 0;
	ADCSRA |= (1 << ADEN);	//turn on ADC
	ADCSRA |= (1 << ADIE);	//interruption
	
	//prescaler 128 > 125kHz
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	
	DIDR0 |= (1 << ADC0D) | (1 << ADC1D) | (1 << ADC2D) | (1 << ADC3D);	//disable PC0 - PC3 digital input
}

ISR (ADC_vect){
	ADCSRA |= (1 << ADIF);	//turn off flag
	if(currentMode == manualMode){
		if (count == 0){						//select current channel for reading
			count = 1;							//select next channel for reading
			ADMUX = (ADMUX & 0xF0);
			pos1 = ADCH;
			convertServo(ADCH, channelB);
		}else if(count == 1){					//select current channel for reading
			count = 2;							//select next channel for reading
			ADMUX = (ADMUX & 0xF0) | 1;
			pos2 = ADCH;
			convertServo(ADCH, channelA);
		}else if(count == 2){					//select current channel for reading
			count = 3;							//select next channel for reading
			ADMUX = (ADMUX & 0xF0) | 2;
			pos3 = ADCH;
			updateDutyCB2(ADCH);
		}else if(count == 3){					//select current channel for reading
			count = 0;							//select next channel for reading
			ADMUX = (ADMUX & 0xF0) | 3;
			velMotor = ADCH;
			OCR2A = ADCH;
		}
	}
	
	ADCSRA |= (1 << ADSC);		//starts conversion
}


void initPCINT(void){
	PCICR |= (1 << PCIE0) | (1 << PCIE2);		//PCINT0 and PCINT2
	PCMSK0 |= (1 << PCINT0) | (1 << PCINT4);	//PB0 and PB4
	PCMSK2 |= (1 << PCINT23);					//PD7
}

ISR (PCINT2_vect){
	//If current mode is manual, saving push button works
	if(currentMode == manualMode){
		
		if (!(PIND & (1 << PIND7))) {					//if there's been a change on PD7
			
			if(counterPos == 1){
				counterPos = 2;							//select next position for saving
				writeText("\nGuardada posición 1\n\n");	//confirmation message
				eeprom_update_byte((uint8_t*)address1, pos1);
			}else if(counterPos == 2){
				counterPos = 3;							//select next position for saving
				writeText("\nGuardada posición 2\n\n");	//confirmation message
				eeprom_update_byte((uint8_t*)address2, pos2);
			}else if(counterPos == 3){
				counterPos = 4;							//select next position for saving
				writeText("\nGuardada posición 3\n\n");	//confirmation message
				eeprom_update_byte((uint8_t*)address3, pos3);
			}else if(counterPos == 4){
				counterPos = 1;							//select next position for saving
				writeText("\nGuardada posición 4\n\n");	//confirmation message
				eeprom_update_byte((uint8_t*)address4, velMotor);
			}
			
		}
		
	}else{
		counterPos = 1;
	}
	
}


ISR (PCINT0_vect){
	//If current mode is manual, direction push buttons works
	//if(currentMode == manualMode){
		if (!(PINB & (1 << PINB0))) {
			PORTD |= (1 << PORTD2);
			PORTD &= ~(1 << PORTD4);
			dir = 1;
		}else if (!(PINB & (1 << PINB4))) {
			PORTD |= (1 << PORTD4);
			PORTD &= ~(1 << PORTD2);
			dir = 1;
		}else if (PINB & (1 << PINB0)) {
			PORTD &= ~((1 << PORTD2) | (1 << PORTD4));
			dir = 0;
		}else if (PINB & (1 << PINB4)) {
			PORTD &= ~((1 << PORTD2) | (1 << PORTD4));
			dir = 0;
		}
	//}
}


void initUART9600(void){
	//settigns for RX and TX
	DDRD &= ~(1 << DDD0);		//Rx as input
	DDRD |= (1 << DDD1);		//TX as output
	
	//Fast mode, U2X0
	UCSR0A = 0;
	UCSR0A |= (1 << U2X0);
	
	//Settigns for register B
	UCSR0B = 0;
	UCSR0B |= (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0); //ISR, enable for RX and TX
	
	// settigns for register C
	UCSR0C = 0;
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);		// character size: 8 bits, no parity, 1 stop bit
	
	//Baudrate
	UBRR0 = 207;		// 9600
}


void writeText(char* text){
	//uint8_t i;
	for (uint8_t i = 0; text[i] != '\0'; i++)
	{
		while(!(UCSR0A & (1 << UDRE0)));
		UDR0 = text[i];
	}
}


//ISR, recieve
ISR(USART_RX_vect){
	bufferRX = UDR0;
	
	//if buffer is emptym, if it is not, it waits
	while(!(UCSR0A & (1 << UDRE0)));
	char copy = bufferRX;
	uint8_t num = atoi(&copy);
	
	if (newAction == 0){
	
		if(answer2 == 0){
		
			if(num == manualMode){		//It checks if the mun received is related to manual mode
				PORTD |= (1 << led1);	//It shows the number of the current mode
				PORTD &= ~(1 <<  led2);
				currentMode = manualMode;
				answer2 = 0;
				sentChar();
				writeText("\nMANUAL MODE SELECTED\n");
				menu();
			}else if(num == eepromMode){
				PORTD |= (1 << led2);	//It shows the number of the current mode
				PORTD &= ~(1 <<  led1);
				currentMode = eepromMode;
				answer2 = 0;
				sentChar();
				writeText("\nEEPROM MODE SELECTED\n");
				menu();
				readData = eeprom_read_byte((uint8_t*)address1);
				convertServo(readData, channelB);
				readData = eeprom_read_byte((uint8_t*)address2);
				convertServo(readData, channelA);
				readData = eeprom_read_byte((uint8_t*)address3);
				updateDutyCB2(readData);
				readData = eeprom_read_byte((uint8_t*)address4);
				OCR2A = readData;
				
			}else if (num == usartMode){
				PORTD |= (1 << led2) | (1 <<  led1);	//It shows the number of the current mode
				currentMode = usartMode;
				answer2  = 1;
				sentChar();
				writeText("\nUSART MODE SELECTED\n");
				writeText("\n\n******** Choose an action ********\n");
				writeText("\t 1. Go to manual mode\n");
				writeText("\t 2. Go to EEPROM mode\n");
				writeText("\t 3. Control door1's servo\n");
				writeText("\t 4. Control door2's servo\n");
				writeText("\t 5. Control direction's servo\n");
				writeText("\t 6. Contorl velocity's DC\n\n");
			}else{
				answer2 = 0;
				writeText("\n\nInvalid option\n\n");
				menu();
			}
		
		}else{
			sentChar();
			switch (num){
				case 1:
					PORTD |= (1 << led1);	//It shows the number of the current mode
					PORTD &= ~(1 <<  led2);
					currentMode = manualMode;
					answer2 = 0;
					//sentChar();
					writeText("\nMANUAL MODE SELECTED\n");
					menu();
					break;
				
				case 2:
					PORTD |= (1 << led2);	//It shows the number of the current mode
					PORTD &= ~(1 <<  led1);
					currentMode = eepromMode;
					answer2 = 0;
					//sentChar();
					writeText("\nEEPROM MODE SELECTED\n");
					menu();
					break;
			
				case 3:
					newAction = 1;
					setServo1 = 1;
					//sentChar();
					writeText("\nDoor 1 selected\n");
					writeText("\n\n******** Choose an action ********\n");
					writeText("\t 1. Open door\n");
					writeText("\t 2. Close door\n");
					break;
			
				case 4:
					newAction = 1;
					setServo2 = 1;
					//sentChar();
					writeText("\nDoor 2 selected\n");
					writeText("\n\n******** Choose an action ********\n");
					writeText("\t 1. Open door\n");
					writeText("\t 2. Close door\n");
					break;
				
				case 5:
					newAction = 1;
					setServo3 = 1;
					//sentChar();
					writeText("\nDirection selected\n");
					writeText("\n\n******** Choose an action ********\n");
					writeText("\t 1. Left\n");
					writeText("\t 2. Right\n");
					writeText("\t 3. Straight\n");
					break;
				
				case 6:
					newAction = 1;
					setVel = 1;
					//sentChar();
					writeText("\nVelocity selected\n");
					writeText("\n\n******** Choose an action ********\n");
					writeText("\t 1. Fast\n");
					writeText("\t 2. Slow\n");
					break;
				
				default: 
					newAction = 0;
					answer2 = 0;
					writeText("\n\n\t Invalid option\n");
					break;
			}
		
		}
	}else{
		newAction = 0;
		answer2 = 0;
		sentChar();
		
		//control servo 1 selected
		if(setServo1 == 1){
			setServo1 = 0;
			
			switch (num){
				case 1:
					writeText("\n Open door\n");
					convertServo(255, channelB);
					break;
				
				case 2:
					writeText("\n Close door\n");
					convertServo(0, channelB);
					break;
				
				default:
					writeText("\n Invalid option\n");
					break;
			}
		}
		
		//Control servo2 selected
		if(setServo2 == 1){
			setServo2 = 0;
			
			switch (num){
				case 1:
					writeText("\n Open door\n");
					convertServo(255, channelA);
					break;
				
				case 2:
					writeText("\n Close door\n");
					convertServo(0, channelA);
					break;
				
				default:
					writeText("\n Invalid option\n");
					break;
			}
		}
		
		//control servo3 selected
		if(setServo3 == 1){
			setServo3 = 0;
			
			switch (num){
				case 1:
				writeText("\n Left\n");
				updateDutyCB2(255);
				break;
				
				case 2:
				writeText("\n Right\n");
				updateDutyCB2(0);
				break;
				
				case 3:
				writeText("\n Straight");
				updateDutyCB2(125);
				break;
				
				default:
				writeText("\n Invalid option\n");
				break;
			}
		}
		
		
		if(setVel == 1){
			setVel = 0;
			
			switch (num){
				case 1:
				writeText("\n Fast\n");
				OCR2A = 255;
				break;
				
				case 2:
				writeText("\n Slow\n");
				OCR2A = 75;
				break;
				
				default:
				writeText("\n Invalid option\n");
				break;
			}
		}
		
		menu();		//It shows menu every time
	}
}


void menu(void){
	writeText("\n\n******** Choose operation mode ********\n");
	writeText("\t 1. Manual mode\n");
	writeText("\t 2. EEPROM mode\n");
	writeText("\t 3. USART mode\n\n");
}

void sentChar(void){
	UDR0 = bufferRX;
	writeText(" -> Sent character");
}
//The end :D