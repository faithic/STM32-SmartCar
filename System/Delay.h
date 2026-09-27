#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);

// Call once per TIM4 update interrupt, i.e. every 20 ms.
void Delay_TickUpdate(void);

// Milliseconds since power on. Resolution is 20 ms on this car, because TIM4
// is the only free periodic source (SysTick is claimed by Delay_us/Delay_ms).
uint32_t GetTick(void);

#endif
