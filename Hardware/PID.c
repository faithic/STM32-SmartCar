#include "stm32f10x.h"                  // Device header
#include <math.h>
//定义结构体
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
//左右轮各定义一个结构体
PID_TypeDef LeftPID;
PID_TypeDef RightPID;
//1.PID初始化函数
void PID_Init(PID_TypeDef *pid)
{
	//PID参数
	pid->Kp = 2.0f;
    pid->Ki = 0.8f;
    pid->Kd = 0.0f;
	//限幅
	pid->MaxIntegral = 300.0f;
	pid->MaxOutput = 160.0f;
	//目标值
    pid->Target = 0;
	//反馈值
    pid->Actual = 0;
	//误差
    pid->Error = 0;
    pid->LastError = 0;
	pid->PrevError = 0;
	pid->Derivative = 0;
	//积分
    pid->Integral = 0;
	//输出
    pid->Output = 0;
	//积分分离阈值
	pid->IntegralThreshold = 20.0f;
	//各输出项的值
	pid->D_Out = 0;
	pid->I_Out = 0;
	pid->P_Out = 0;
}
void PID_LeftInit(PID_TypeDef *pid)
{
    pid->Kp = 2.0f;
    pid->Ki = 0.1f;
    pid->Kd = 0.0f;

    pid->MaxIntegral = 300.0f;
    pid->MaxOutput   = 160.0f;

    pid->Target = 0;
    pid->Actual = 0;
    pid->Error = 0;
    pid->LastError = 0;
    pid->PrevError = 0;
    pid->Derivative = 0;
    pid->Integral = 0;
    pid->Output = 0;

    pid->IntegralThreshold = 20.0f;

    pid->P_Out = 0;
    pid->I_Out = 0;
    pid->D_Out = 0;
}
void PID_RightInit(PID_TypeDef *pid)
{
    pid->Kp = 2.0f;
    pid->Ki = 0.1f;
    pid->Kd = 0.0f;

    pid->MaxIntegral = 300.0f;
    pid->MaxOutput   = 160.0f;

    pid->Target = 0;
    pid->Actual = 0;
    pid->Error = 0;
    pid->LastError = 0;
    pid->PrevError = 0;
    pid->Derivative = 0;
    pid->Integral = 0;
    pid->Output = 0;

    pid->IntegralThreshold = 20.0f;

    pid->P_Out = 0;
    pid->I_Out = 0;
    pid->D_Out = 0;
}
//2.PID清零函数->停车时->恢复PID运行状态，但是PID参数保持不变
void PID_Clear(PID_TypeDef *pid)
{
	pid->Target = 0;//因为对应的情况是停车，目标值自然要设为0
	pid->Actual = 0;
    pid->Error = 0;//将所有误差清零，避免下一次启动时误差积累过大
	pid->PrevError = 0;
	pid->LastError = 0;
	pid->Derivative = 0;
    pid->Integral = 0;//积分项清零
    pid->Output = 0;//输出也要清零 原因:我们是根据这里的Output传入Motor_SetSpeed()函数，要实现停车自然要清零
	pid->P_Out = 0;
	pid->I_Out = 0;
	pid->D_Out = 0;
	//注意：pid->Kp pid->Ki pid->Kd MaxOutput MaxIntegral IntegralThreshold这些都不是状态变量，属于PID参数
}
//3.修改target的值
void PID_SetTarget(PID_TypeDef *pid,float target)
{
	pid->Target = target;
}
//4.积分限幅函数
void PID_IntegralLimit(PID_TypeDef *pid)
{
	if(pid->Integral>pid->MaxIntegral)
	{
		pid->Integral=pid->MaxIntegral;
	}
	else if(pid->Integral<-(pid->MaxIntegral))
	{
		pid->Integral=-(pid->MaxIntegral);
	}
}
//5.输出限幅函数
void PID_OutputLimit(PID_TypeDef *pid)
{
	if(pid->Output>pid->MaxOutput)
	{
		pid->Output=pid->MaxOutput;
	}
	else if(pid->Output<-(pid->MaxOutput))
	{
		pid->Output=-(pid->MaxOutput);
	}
}
//积分分离解决：启动阶段积分没有意义  而积分限幅解决：积分无限增大 积分分离永远在前，积分限幅永远在后
//6.积分分离函数
void PID_IntegralUpdate(PID_TypeDef *pid)
{
	//判断是否在阈值范围内
	if(fabsf(pid->Error)< pid->IntegralThreshold)
	{
		//更新积分
		pid->Integral += pid->Error;
		//积分限幅
		PID_IntegralLimit(pid);
	}
}
//7.单独修改PID参数
void PID_SetParameter(PID_TypeDef *pid,float Kp,float Ki,float Kd,float MaxOutput,float MaxIntegral,float IntegralThreshold)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->MaxIntegral = MaxIntegral;
	pid->MaxOutput = MaxOutput;
	pid->IntegralThreshold = IntegralThreshold;
}
//8.清积分函数
void PID_ResetIntegral(PID_TypeDef *pid)
{
	pid->Integral = 0;
	pid->I_Out = 0;//由于Integral为0， 积分项也应该为0
}
//9.位置式PID函数  Speed是传入的实际速度 可利用Encoder_GetLeftSpeed(),Encoder_GetRightSpeed()传入
float PID_Position(PID_TypeDef *pid, float Speed)
{
	//更新实际速度
	pid->Actual = Speed;
	//计算误差
	pid->Error = pid->Target - pid->Actual;
	//积分分离函数 (更新积分+积分限幅)
	PID_IntegralUpdate(pid);
	//计算微分(误差变化率)
	pid->Derivative = pid->Error - pid->LastError;
	//计算比例项
	pid->P_Out = pid->Kp * pid->Error;
	//计算积分项
	pid->I_Out = pid->Ki * pid->Integral;
	//计算微分项
	pid->D_Out = pid->Kd * pid->Derivative;
	//计算PID总输出
	pid->Output = pid->P_Out + pid->I_Out + pid->D_Out;
	//输出限幅
	PID_OutputLimit(pid);
	//更新误差
	pid->LastError = pid->Error;
	//返回PWM
	return pid->Output;
}
