#ifndef _CAR_H
#define _CAR_H

#include "stm32f10x.h"
#include "PWM.h"
#include "Motor.h"

// Car初始化函数
void Car_Init(void);
// 停止
void Car_Stop(void);
// 基础开环/设定速度运动指令
void Car_Move(int16_t LeftTarget, int16_t RightTarget);

// 慢速向前
void Go_Forward_Slowly(void);
// 正常向前
void Go_Forward_Normally(void);
// 快速向前
void Go_Forward_Quickly(void);
// 后退
void Back_Off(void);
// 开环差速左转
void Turn_Left(void);
// 开环差速右转
void Turn_Right(void);
// 原地自转
void Car_SpinLeft(void);
void Car_SpinRight(void);

// ================= 航向角双闭环高级控制接口 =================
void Turn_To_Angle(float target_angle);
void Turn_Left_90(void);
void Turn_Right_90(void);
void Car_DriveStraight(float base_speed, float target_yaw);

#endif
