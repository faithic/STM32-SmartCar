#ifndef _ENCODER_H
#define _ENCODER_H
#include "stm32f10x.h" 
extern volatile int16_t LeftCount;
extern volatile int16_t RightCount;
//左轮编码器初始化
void Encoder1_Init(void);
//读取CNT的值，并清零
int16_t Encoder1_GetCount(void);
//右轮编码器初始化
void Encoder2_Init(void);
//读取CNT的值，并清零
int16_t Encoder2_GetCount(void);
//直接初始化左右轮编码器
void Encoder_Init(void);
//返回左轮10ms采样周期内的编码器计数
int16_t Encoder_GetLeftCount(void);
//返回右轮10ms采样周期内的编码器计数
int16_t Encoder_GetRightCount(void);
#endif
