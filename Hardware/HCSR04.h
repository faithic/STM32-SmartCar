#ifndef _HCSR04_H
#define _HCSR04_H
void HCSR04_Init(void);
void HCSR04_Trigger(void);
void HCSR04_SysTickStart(void);
void HCSR04_SysTickStop(void);
uint8_t HCSR04_WaitEchoHigh(void);
uint8_t Ultrasonic_WaitEchoLow(void);
float Ultrasonic_GetDistance(void);
#endif
