#include "stm32f10x.h"                  // Device header
#include "Encoder.h"
#include "PID.h"
#include "Motor.h"
#define CONTROL_OUTPUT_MAX     100.0f
#define TARGET_SPEED_MAX     160.0f
#define RIGHT_SPEED_COMPENSATION   1.0f
static uint8_t Control_Enable = 0;// 用于使能判断 其他模块无法直接修改
//1.管理控制器状态
void Control_Start(void)
{
	if(Control_Enable)
	{
        return;
	}
	//初始化控制状态->避免之前存在一些状态变量没有清干净
	PID_Clear(&LeftPID);
    PID_Clear(&RightPID);
	//开启控制器
    Control_Enable = 1;
}
void Control_Stop(void)
{
	//电机速度置0
	Motor_SetSpeed(0,0);
	//PID清零
	PID_Clear(&LeftPID);
	PID_Clear(&RightPID);
	//关闭控制器
	Control_Enable = 0;
}
//2.限幅函数
static float Control_Limit(float Value,float Max)
{
    if(Value > Max)
        Value = Max;
    else if(Value < -Max)
        Value = -Max;
    return Value;
}
//3.管理整个控制器的目标速度
void Control_SetTarget(float Left, float Right )
{
	//对传入的目标速度限幅
    Left = Control_Limit(Left, TARGET_SPEED_MAX);
    Right = Control_Limit(Right , TARGET_SPEED_MAX);
	//调用PID模块中的PID_SetTarget()函数对目标速度进行修改(不直接对结构体操作)
    PID_SetTarget(&LeftPID, Left);
    PID_SetTarget(&RightPID, Right+RIGHT_SPEED_COMPENSATION);
}
//4.完成一次控制周期  利用编码器反馈，计算新的PWM输出，使实际速度不断逼近目标速度
void Control_Update(void)
{
	//设置为局部变量即可
	float LeftOutput = 0.0f;
	float RightOutput = 0.0f;
	float Left_Actual = 0.0f;
	float Right_Actual = 0.0f;
	if(Control_Enable == 0)
    {
        return;
    }
	//读取编码器反馈
	Left_Actual = (float)(Encoder_GetLeftCount());
	Right_Actual =(float)(Encoder_GetRightCount());
	//PID计算
	LeftOutput = PID_Position(&LeftPID,Left_Actual);
	RightOutput = PID_Position(&RightPID,Right_Actual);
	//双保险(已有输出限幅)
	LeftOutput = Control_Limit(LeftOutput,CONTROL_OUTPUT_MAX);
	RightOutput = Control_Limit(RightOutput,CONTROL_OUTPUT_MAX);
	//PWM输出
	Motor_SetLeftSpeed((int16_t)LeftOutput);
	Motor_SetRightSpeed((int16_t)RightOutput);
}
