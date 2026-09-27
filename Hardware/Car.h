#ifndef _CAR_H
#define _CAR_H
#include "PWM.h"
#include "Motor.h"
//Car初始化函数
void Car_Init(void);
//运动指令
void Car_Move(int16_t LeftTarget,int16_t RightTarget);
//慢速向前
void Go_Forward_Slowly(void);
//正常向前
void Go_Forward_Normally(void);
//快速向前
void Go_Forward_Quickly(void);
//后退
void Back_Off(void);
//左转
void Turn_Left(void);
//右转
void Turn_Right(void);
//停止
void Car_Stop(void);
//原地自转
void Car_SpinLeft(void);
void Car_SpinRight(void);
#endif
