#include "stm32f10x.h"                  // Device header
#include "Encoder.h"
#include "Motor.h"
#include "Control.h"
#include "PID.h"
#include "AnglePID.h"
#include "app_mpu6050.h"
#include "Delay.h"
#include <math.h>

// Car初始化函数
void Car_Init(void)
{
    Motor_Init();       // 包含PWM初始化
    Encoder_Init();
    PID_LeftInit(&LeftPID);
    PID_RightInit(&RightPID);
    AnglePID_Init();    // 航向角外环PID初始化
}

// 停止
void Car_Stop(void)
{
    Control_Stop();
}

// 运动指令
void Car_Move(int16_t LeftTarget, int16_t RightTarget)
{
	Control_Start();    // 避免之前调用Stop()时控制器置0，再次调用该函数时小车动不了
	Control_SetTarget(LeftTarget, RightTarget);
}

// 慢速向前
void Go_Forward_Slowly(void)
{
	Car_Move(25, 25);
}

// 正常向前
void Go_Forward_Normally(void)		
{
	Car_Move(50, 50);
}

// 快速向前
void Go_Forward_Quickly(void)		
{
	Car_Move(70, 70);
}

// 后退
void Back_Off(void)					
{
	Car_Move(-50, -50);
}

// 左转
void Turn_Left(void)					
{
	Car_Move(20, 70);
}

// 右转
void Turn_Right(void)				
{
	Car_Move(70, 20);
}

// 原地自转
void Car_SpinLeft(void)
{
    Car_Move(-40, 40);
}

void Car_SpinRight(void)
{
    Car_Move(40, -40);
}

// ======================== 航向角双闭环高级控制 ========================

/**
  * @brief  航向角闭环转弯函数（原地旋转到指定相对角度）
  * @param  target_angle: 目标相对角度（例如 +90.0f 为左转90°，-90.0f 为右转90°，180.0f 为掉头）
  * @note   双闭环机制：外环 AnglePID 计算差速修正量 turn，内环 Control_SetTarget(-turn, turn)
  */
void Turn_To_Angle(float target_angle)
{
    uint8_t stable_count = 0;
    uint16_t timeout = 0; // 超时计数器（防止机械卡死无限死循环）

    // 1. 复位当前 MPU6050 航向角为 0° 参考基准
    App_MPU6050_ResetAngle();
    AnglePID_Reset();

    // 2. 等待 25ms：因为 App_MPU6050_ResetAngle() 只是置标志位，
    //    真正的 AngleZ=0 是在 20ms 的 TIM4 中断中执行清零的
    Delay_ms(25);

    // 3. 启动内环控制器
    Control_Start();

    // 4. 双闭环迭代调节（每 20ms 与 TIM4 控制周期同步）
    while (1)
    {
        float current_angle = App_MPU6050_GetAngle();
        float error = AnglePID_NormalizeAngle(target_angle - current_angle);
        float current_gz = App_MPU6050_GetGz(); // 获取当前角速度

        // 5. 稳定到达判定：角度误差 < 1.5° 且角速度 < 3.0°/s，连续 5 个控制周期（100ms）满足
        if (fabsf(error) <= 1.5f && fabsf(current_gz) <= 3.0f)
        {
            stable_count++;
            if (stable_count >= 5)
            {
                break; // 已精确稳定在目标角度，退出转向
            }
        }
        else
        {
            stable_count = 0;
        }

        // 6. 角度外环计算：输入目标角度与当前角度，输出左右轮速度修正量
        float turn = AnglePID_Calculate(target_angle, current_angle);

        // 7. 原地自转：base_speed = 0
        //    左轮 = base_speed - turn = -turn
        //    右轮 = base_speed + turn = +turn
        Control_SetTarget(-turn, turn);

        // 8. 配合 20ms 控制心跳周期延时
        Delay_ms(20);

        // 9. 超时安全退出（150 * 20ms = 3000ms，防止受阻打滑卡死）
        timeout++;
        if (timeout >= 150)
        {
            break;
        }
    }

    // 10. 到位后立即停车
    Car_Stop();
}

/**
  * @brief  原地左转 90 度
  */
void Turn_Left_90(void)
{
    Turn_To_Angle(90.0f);
}

/**
  * @brief  原地右转 90 度
  */
void Turn_Right_90(void)
{
    Turn_To_Angle(-90.0f);
}

/**
  * @brief  定角度直线巡航控制（单次周期调用）
  * @param  base_speed: 基础前进速度（例如 30.0f ~ 60.0f）
  * @param  target_yaw: 期望保持的航向角（例如 0.0f 为初始方向）
  */
void Car_DriveStraight(float base_speed, float target_yaw)
{
    Control_Start();

    float current_angle = App_MPU6050_GetAngle();
    float turn = AnglePID_Calculate(target_yaw, current_angle);

    // 外环输出给内环左右轮速度目标
    float left_target = base_speed - turn;
    float right_target = base_speed + turn;

    Control_SetTarget(left_target, right_target);
}
