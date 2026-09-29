#ifndef APP_MPU6050_H
#define APP_MPU6050_H

#include "stm32f10x.h"

// 【小车工程的两条使用约束】
// 1) App_MPU6050_Update() 现在由 TIM4 的 20ms 中断调用，它是不可重入的：
//    主循环里绝对不能再调一次，否则两条 I2C 事务会在同一条总线上交错。
//    主循环只许调下面那些 Get* 读数值。
// 2) 那些值（ax/ay/az/gx/gy/gz/temperature）在 .c 里没有加 volatile。当前
//    优化等级是 -O0 所以没事；谁要是去 Project.uvprojx 里把 <Optim> 调高，
//    必须先把这 7 个变量加上 volatile，否则主循环会一直读到被缓存的旧值。

// 零偏校准的默认采样组数，500组约需1.2秒
#define MPU_CALIB_SAMPLES 500

void App_MPU6050_Init(void);

// @返回值：1 - 初始化成功且传感器可用，0 - 初始化/通信失败
uint8_t App_MPU6050_IsReady(void);

// @返回值：0 - 读取成功，-1 - I2C通信失败（此时数据不可信，不要拿去积分）
int App_MPU6050_Update(void);

float App_MPU6050_GetAx(void);
float App_MPU6050_GetAy(void);
float App_MPU6050_GetAz(void);

float App_MPU6050_GetTemperature(void);

float App_MPU6050_GetGx(void);
float App_MPU6050_GetGy(void);
float App_MPU6050_GetGz(void);

// ---- 角度解算（Yaw）----

// 陀螺仪零偏校准，必须在完全静止时调用。返回0成功，-1失败
int App_MPU6050_Calibrate(uint16_t samples);

// 定时器中断用：显式传入时间间隔dt，单位s（TIM4的20ms中断传0.02f）
void App_MPU6050_UpdateAngle(float dt);

// 主循环用：内部自动测量时间间隔，不要在中断里调用
void App_MPU6050_UpdateAngleAuto(void);

// 获取Z轴累计角度，单位°，范围±180
float App_MPU6050_GetAngle(void);

// 把当前角度清零
void App_MPU6050_ResetAngle(void);

// 获取Z轴零偏，单位°/s
float App_MPU6050_GetGyroBiasZ(void);

#endif
