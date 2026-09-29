/**
  ******************************************************************************
  * @file    AnglePID.h
  * @brief   航向角外环PID控制模块（配合MPU6050与底层左右轮速度PID构成双闭环）
  ******************************************************************************
  */

#ifndef _ANGLE_PID_H_
#define _ANGLE_PID_H_

#include "stm32f10x.h"

/**
  * @brief 角度PID控制器结构体
  */
typedef struct
{
    /* PID 增益参数 */
    float Kp;                  // 比例增益（主要转向响应力度）
    float Ki;                  // 积分增益（消除微小静摩擦稳态误差）
    float Kd;                  // 微分增益（阻尼角速度，抑制超调与震荡）

    /* 状态与误差变量 */
    float Target;              // 目标航向角（°，通常在 -180 ~ +180 之间）
    float Actual;              // 当前航向角（°，来自 App_MPU6050_GetAngle()）
    float Error;               // 当前归一化最短角度误差（°，-180 ~ +180）
    float LastError;           // 上一次误差（°）
    float Integral;            // 积分累加值
    float Derivative;          // 误差微分项

    /* 各项输出分量（方便调试打串口看波形） */
    float P_Out;
    float I_Out;
    float D_Out;

    /* 限幅与保护参数 */
    float MaxOutput;           // 最大转速差修正量（turn_correction 限幅，如 40.0f）
    float MaxIntegral;         // 积分项限幅（防止积分饱和，如 20.0f）
    float IntegralThreshold;   // 积分分离阈值（如 15.0f，误差大时不积分）
    float DeadZone;            // 角度死区（如 0.5f，误差小于此值不抖动）

    float Output;              // 最终计算输出（turn_correction 差速修正量）
} AnglePID_TypeDef;

/* 声明全局角度PID实例 */
extern AnglePID_TypeDef AnglePID;

/* 核心函数 */
void AnglePID_Init(void);
float AnglePID_Calculate(float target_angle, float current_angle);
void AnglePID_Reset(void);

/* 参数调整接口 */
void AnglePID_SetPID(float Kp, float Ki, float Kd);
void AnglePID_SetLimits(float MaxOutput, float MaxIntegral, float IntegralThreshold);

/* 角度归一化辅助函数（处理 ±180° 环绕） */
float AnglePID_NormalizeAngle(float angle);

#endif
