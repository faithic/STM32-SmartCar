/**
  ******************************************************************************
  * @file    AnglePID.c
  * @brief   航向角外环PID控制模块实现
  *          输入：目标角度、当前角度 (来自 MPU6050)
  *          输出：左右轮差速修正量 turn_correction（正值向左偏/逆时针，负值向右偏/顺时针）
  ******************************************************************************
  */

#include "AnglePID.h"
#include <math.h>

AnglePID_TypeDef AnglePID;

/**
  * @brief  将角度折返归一化到 [-180.0, +180.0] 范围内
  * @param  angle: 输入任意角度
  * @retval 折返后的角度（-180.0 ~ +180.0）
  * @note   用于解决例如：目标 179°，当前 -179° 时，实际误差为 -2°
  */
float AnglePID_NormalizeAngle(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

/**
  * @brief  角度PID初始化函数
  * @note   默认参数针对本小车底盘速度闭环调谐（以 20ms 控制周期为基准）：
  *         Kp=1.6f: 误差 10° 产生 16 的差速修正，平稳不突兀
  *         Ki=0.02f: 配合积分分离，用于克服静摩擦死区
  *         Kd=1.2f: 阻尼快速旋转时的角速度，抑制过冲
  *         MaxOutput=40.0f: 限制最大差速，防止电机瞬间反转或超出内环最大速度
  */
void AnglePID_Init(void)
{
    AnglePID.Kp = 1.6f;
    AnglePID.Ki = 0.02f;
    AnglePID.Kd = 1.2f;

    AnglePID.Target = 0.0f;
    AnglePID.Actual = 0.0f;
    AnglePID.Error = 0.0f;
    AnglePID.LastError = 0.0f;
    AnglePID.Integral = 0.0f;
    AnglePID.Derivative = 0.0f;

    AnglePID.P_Out = 0.0f;
    AnglePID.I_Out = 0.0f;
    AnglePID.D_Out = 0.0f;

    AnglePID.MaxOutput = 40.0f;           // 输出限幅（±40 编码器脉冲/20ms）
    AnglePID.MaxIntegral = 20.0f;         // 积分限幅
    AnglePID.IntegralThreshold = 15.0f;   // 积分分离阈值（误差大于15°关闭积分）
    AnglePID.DeadZone = 0.5f;             // 角度死区（±0.5°内不反复抖动微调）

    AnglePID.Output = 0.0f;
}

/**
  * @brief  复位角度PID内部状态（清空积分和上一拍历史误差）
  */
void AnglePID_Reset(void)
{
    AnglePID.Error = 0.0f;
    AnglePID.LastError = 0.0f;
    AnglePID.Integral = 0.0f;
    AnglePID.Derivative = 0.0f;
    AnglePID.P_Out = 0.0f;
    AnglePID.I_Out = 0.0f;
    AnglePID.D_Out = 0.0f;
    AnglePID.Output = 0.0f;
}

/**
  * @brief  单独配置PID参数
  */
void AnglePID_SetPID(float Kp, float Ki, float Kd)
{
    AnglePID.Kp = Kp;
    AnglePID.Ki = Ki;
    AnglePID.Kd = Kd;
}

/**
  * @brief  单独配置限幅与积分分离阈值
  */
void AnglePID_SetLimits(float MaxOutput, float MaxIntegral, float IntegralThreshold)
{
    AnglePID.MaxOutput = MaxOutput;
    AnglePID.MaxIntegral = MaxIntegral;
    AnglePID.IntegralThreshold = IntegralThreshold;
}

/**
  * @brief  角度外环PID核心计算函数
  * @param  target_angle: 目标航向角（单位：度）
  * @param  current_angle: 当前航向角（来自 App_MPU6050_GetAngle()，单位：度）
  * @retval 左右轮速度修正量 turn_correction
  *         正值：需要左转（逆时针） -> left = base - turn, right = base + turn
  *         负值：需要右转（顺时针） -> left = base - turn, right = base + turn
  */
float AnglePID_Calculate(float target_angle, float current_angle)
{
    AnglePID.Target = target_angle;
    AnglePID.Actual = current_angle;

    // 1. 计算原始角度偏差
    float raw_error = target_angle - current_angle;

    // 2. 角度环绕归一化到 [-180.0, +180.0] 范围内（选择最短转向路径）
    AnglePID.Error = AnglePID_NormalizeAngle(raw_error);

    // 3. 角度死区滤波：小车在极小误差内不产生电机微颤
    if (fabsf(AnglePID.Error) < AnglePID.DeadZone)
    {
        AnglePID.Error = 0.0f;
    }

    // 4. 积分抗饱和与积分分离
    if (fabsf(AnglePID.Error) < AnglePID.IntegralThreshold && AnglePID.Error != 0.0f)
    {
        AnglePID.Integral += AnglePID.Error;
        // 积分限幅
        if (AnglePID.Integral > AnglePID.MaxIntegral)
        {
            AnglePID.Integral = AnglePID.MaxIntegral;
        }
        else if (AnglePID.Integral < -AnglePID.MaxIntegral)
        {
            AnglePID.Integral = -AnglePID.MaxIntegral;
        }
    }
    else
    {
        // 误差过大（如大角度转向开始时）或进入死区时，清零积分项避免积分滞后超调
        AnglePID.Integral = 0.0f;
    }

    // 5. 计算误差微分（D项：抑制旋转惯性引起的过冲）
    AnglePID.Derivative = AnglePID.Error - AnglePID.LastError;
    AnglePID.LastError = AnglePID.Error;

    // 6. 各项分量计算
    AnglePID.P_Out = AnglePID.Kp * AnglePID.Error;
    AnglePID.I_Out = AnglePID.Ki * AnglePID.Integral;
    AnglePID.D_Out = AnglePID.Kd * AnglePID.Derivative;

    // 7. 总输出累加
    AnglePID.Output = AnglePID.P_Out + AnglePID.I_Out + AnglePID.D_Out;

    // 8. 输出限幅 [-MaxOutput, +MaxOutput]
    if (AnglePID.Output > AnglePID.MaxOutput)
    {
        AnglePID.Output = AnglePID.MaxOutput;
    }
    else if (AnglePID.Output < -AnglePID.MaxOutput)
    {
        AnglePID.Output = -AnglePID.MaxOutput;
    }

    return AnglePID.Output;
}

