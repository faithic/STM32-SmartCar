#ifndef __BT_H
#define __BT_H
#include <stdio.h>
#include <stdarg.h>
#include "OLED.h"
#include "Car.h"
void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);
void BT_control(void);
#endif
