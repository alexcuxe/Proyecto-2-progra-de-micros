/*
 * PWM1.h
 *
 * Created: 15/04/2024 01:16:14 a. m.
 *  Author: Eber Alexander
 */ 


#ifndef PWM1_H_
#define PWM1_H_

#include <avr/io.h>
#include <stdint.h>

#define yes 1
#define nop 0
#define normal 1
#define settedUp 2
#define channelA 1
#define channelB 2

/* Use it before setChannel();
Settings for Fast PWM 1, 16 bits
(modePWM <- normal/settedUp, prescaler <- 1,8,64,256,1024)
*/
void initFastPWM1(uint8_t modePWM, uint16_t prescaler);

//(channel <- channelA/channelB, inverted <- yes/nop)
void channel(uint8_t setChannel, uint8_t inverted);

//topValue, used if mode is settedUp
void topValue(uint16_t top);

/*conversion for servos, mapping values
(analogIN = ADCH, channelA/channelB)*/
void convertServo(uint16_t analogIn, uint8_t selChannel);

float maping(float, float, float, float, float);

#endif /* PWM1_H_ */