#ifndef _PID_H
#define _PID_H
#include "stm32f10x.h"   
#include <math.h>
//声明PID结构体类型，和.c完全一致
typedef struct
{
    //PID参数
    float Kp;
    float Ki;
    float Kd;
    //目标值 
    float Target;
    // 实际值
    float Actual;
    //当前误差 
    float Error;
    //上一次误差
    float LastError;
    //上上次误差（增量式PID需要） 
    float PrevError;
    //积分项 
    float Integral;
    //PID输出 
    float Output;
    //输出限幅 
    float MaxOutput;
    //积分限幅
    float MaxIntegral;
	//积分分离阈值
	float IntegralThreshold;
	// P项输出
	float P_Out;    
	// I项输出
	float I_Out;    
	// D项输出
	float D_Out;  
	//微分
	float Derivative;
}PID_TypeDef;
//外部声明左右轮全局PID变量，方便其他文件调用LeftPID、RightPID
extern PID_TypeDef LeftPID;
extern PID_TypeDef RightPID;
//PID初始化
void PID_Init(PID_TypeDef *pid);
void PID_LeftInit(PID_TypeDef *pid);
void PID_RightInit(PID_TypeDef *pid);
//PID清零函数
void PID_Clear(PID_TypeDef *pid);
//设置目标值
void PID_SetTarget(PID_TypeDef *pid,float target);
//积分限幅
void PID_IntegralLimit(PID_TypeDef *pid);
//输出限幅
void PID_OutputLimit(PID_TypeDef *pid);
//积分分离函数
void PID_IntegralUpdate(PID_TypeDef *pid);
//修改PID参数
void PID_SetParameter(PID_TypeDef *pid,float Kp,float Ki,float Kd,float MaxOutput,float MaxIntegral);
// 清积分
void PID_ResetIntegral(PID_TypeDef *pid);
//位置式PID
float PID_Position(PID_TypeDef *pid, float Speed);
#endif
