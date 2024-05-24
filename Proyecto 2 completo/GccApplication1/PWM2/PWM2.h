#ifndef PWM2_H_
#define PWM2_H_

#include <avr/io.h>
#include <stdint.h>

#define invertido 1
#define no_invertido 0

float map1(float, float, float, float, float);

void initPWM2A(uint8_t inverted, uint16_t prescaler);

void initPWM2B(uint8_t inverted, uint16_t prescaler);

void updateDutyCA2(uint8_t duty1);

void updateDutyCB2(uint8_t duty2);
void setup_PWM(void);

#endif /* PWM2_H_ */