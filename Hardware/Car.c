#include "stm32f10x.h"                  // Device header
#include "Encoder.h"
#include "Motor.h"
#include "Control.h"
#include "PID.h"
//Car初始化函数
void Car_Init(void)
{
    Motor_Init();//包含PWM初始化
    Encoder_Init();
    PID_LeftInit(&LeftPID);
    PID_RightInit(&RightPID);
}
//停止
void Car_Stop(void)
{
    Control_Stop();
}
//运动指令
void Car_Move(int16_t LeftTarget,int16_t RightTarget)
{
	Control_Start();//避免之前调用Stop()时控制器置0，再次调用该函数时小车动不了
	Control_SetTarget(LeftTarget,RightTarget);
}
//慢速向前
void Go_Forward_Slowly(void)
{
	Car_Move(25,25);
}
//正常向前
void Go_Forward_Normally(void)		
{
	Car_Move(50,50);
}
//快速向前
void Go_Forward_Quickly(void)		
{
	Car_Move(70,70);
}
//后退
void Back_Off(void)					
{
	Car_Move(-50,-50);
}
//左转
void Turn_Left(void)					
{
	Car_Move(20,70);
}
//右转
void Turn_Right(void)				
{
	Car_Move(70,20);
}
//原地自转
void Car_SpinLeft(void)
{
    Car_Move(-40,40);
}
void Car_SpinRight(void)
{
    Car_Move(40,-40);
}