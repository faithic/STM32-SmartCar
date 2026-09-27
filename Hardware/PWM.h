#ifndef _PWM_H
#define _PWM_H
#include "stm32f10x.h"
void PWM_Init(void);
void PWM_SetCompare1(uint16_t Compare);
void PWM_SetCompare2(uint16_t Compare);
#endif
