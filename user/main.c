#include "stm32f10x.h"                
#include "Delay.h"                  
#include "OLED.h"                      
#include "PWM.h"                     
#include "Motor.h"                    
#include "Car.h"                       
#include "Encoder.h"                  
#include "Timer.h"                     
#include "PID.h"                       
#include "Control.h"                 
#include "app_mpu6050.h"               
#include "SoftI2C.h"                   
#include "Serial_Printf.h"             // 包含串口打印函数 (用于上位机调试)

int main(void)
{
	// ======================== 1. 硬件外设初始化 ========================
	OLED_Init();         
	Car_Init();           
	Serial_Init();        // 初始化串口3 (PB10, PB11)，波特率9600，用于发送PID调试数据

	App_MPU6050_Init();   

	// ======================== 2. 陀螺仪零偏校准 ========================
	// 提示: 上电时必须保持小车静止放置在平地上，等待校准完成
	if (App_MPU6050_IsReady())
	{
		Serial_Printf("calibrating, keep still...\r\n");
		// 采样计算零偏，若检测到晃动会返回错误
		if (App_MPU6050_Calibrate(MPU_CALIB_SAMPLES) != 0)
		{
			Serial_Printf("calibrate FAILED (moved?)\r\n"); // 校准失败(受到晃动)
		}
		else
		{
			Serial_Printf("calibrate done\r\n");            // 校准成功
		}
	}
	else
	{
		Serial_Printf("MPU6050 not ready, skip calibration\r\n"); // 硬件未响应
	}

	// 打印校准后Z轴陀螺仪的零点偏移量 (x1000 deg/s)
	Serial_Printf("bias_z=%d (x1000 deg/s)\r\n", (int)(App_MPU6050_GetGyroBiasZ() * 1000.0f));

	// ======================== 3. 启动系统心脏 ========================
	// 初始化TIM4，开启20ms(50Hz)周期中断。小车的闭环控制全部在这个中断里后台运行！
	Timer_Init();
	
	// ======================== 4. 主循环 (应用层逻辑) ========================
	while (1)
	{
		// 每500ms(25个20ms周期)向屏幕和串口刷新一次当前状态数据
		static uint32_t MpuLastTick = 0;
		uint32_t now = GetTick();
		if (now - MpuLastTick >= 25) 
		{
			// 提取传感器数据并放大为整数显示，避免浮点数打印问题
			int ang = (int)(App_MPU6050_GetAngle() * 10.0f);        // 当前航向角 (单位: 0.1度)
			int gz  = (int)(App_MPU6050_GetGz() * 100.0f);          // Z轴角速度 (单位: 0.01度/秒)
			int az  = (int)(App_MPU6050_GetAz() * 1000.0f);         // Z轴加速度 (单位: 0.001g)
			int tp  = (int)(App_MPU6050_GetTemperature() * 10.0f);  // 芯片温度 (单位: 0.1摄氏度)

			MpuLastTick = now;

			// 通过串口输出到电脑，可用串口助手查看
			Serial_Printf("t=%u ang=%d gz=%d az=%d tp=%d\r\n",
				(unsigned)now, ang, gz, az, tp);

			// 在OLED屏幕上显示航向角和角速度
			OLED_ShowSignedNum(1, 6, ang, 5);   // 第1行: 航向角
			OLED_ShowSignedNum(2, 6, gz, 6);    // 第2行: 旋转角速度
		}
		
		// ------------------------------------------------------------------
		// 【运动控制测试区】

	}
}

// ======================== 5. 核心控制中断 ========================
// 定时器4中断服务函数，周期为20ms (50Hz)
void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4,TIM_IT_Update) == SET)
    {
        // 1. 数据采集
        LeftCount = Encoder1_GetCount();
        RightCount = Encoder2_GetCount();
        
		// 2. 内环控制
		Control_Update();
		
		// 3. 姿态更新: 
		Delay_TickUpdate();                // 系统时间滴答(GetTick)累加20ms
		App_MPU6050_Update();              
		App_MPU6050_UpdateAngle(0.02f);    
		
		// 4. 清除中断标志位，等待下一次20ms到来
        TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
    }
}