#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "PWM.h"
#include "Motor.h"
#include "Car.h"
#include "BT.h"
#include "Encoder.h"
#include "Timer.h"
#include "PID.h"
#include "Control.h"
//#include "Track.h"
#include "Gray.h"
#include "ADC_DMA.h"
#include "Servo.h"
#include "HCSR04.h"
#include "app_mpu6050.h"
#include "SoftI2C.h"
#include "Serial_Printf.h"
int main(void)
{
	OLED_Init();
	Car_Init();
	Serial_Init();//蓝牙模块初始化
//	Servo_Init();
//	Gray_Init();

	// ============== MPU6050（软件I2C：PA4=SDA, PA5=SCL） ==============
	// 上面那句 Gray_Init() 是故意注释掉的，别顺手恢复：
	//   ADC_DMA_Init() 会把 PA2~PA7 全配成 GPIO_Mode_AIN，而 AIN 下施密特
	//   触发器关闭、IDR 恒读 0。SoftI2C 判断应答的约定是"读到0=从机应答"，
	//   所以 PA4/PA5 一旦变成 AIN，I2C 不会报错，而是一路返回"成功"却读回
	//   全 0x00——最难查的那种故障。将来要恢复灰度，必须让 Gray_Init() 排在
	//   App_MPU6050_Init() 的前面。
	// 以后 VL53L1X 挂在 PA4/PA5 同一条传感器总线上（地址不同就能共存），XSHUT 另找空闲脚；PA6/PA7 留给 HCSR04。
	App_MPU6050_Init();

	// 原始14字节自检。用来抓"WHO_AM_I 正常、所有写都报成功、但角度冻在0"
	// 这个静默故障：判据是 buf[1..13] 不能全是 FF（全 FF 说明主机没送出第9个
	// 应答时钟）。这段绕过模块直接调 SoftI2C，所以上面要 include SoftI2C.h。
	{
		uint8_t raw[14];
		uint8_t k;
		if (IIC_ReadRegs(0x68, 0x3B, raw, 14) == 0)
		{
			Serial_Printf("raw14:");
			for (k = 0; k < 14; k++)
			{
				Serial_Printf(" %02X", raw[k]);
			}
			Serial_Printf("\r\n");
		}
		else
		{
			Serial_Printf("raw14 read FAILED\r\n");
		}
	}

	if (App_MPU6050_IsReady())
	{
		// 接下来 2~5 秒车必须完全静止、放平。模块内部校准时一句都不打印，
		// 所以必须在这里先提示，否则第一次上电的人会以为死机了。
		Serial_Printf("calibrating, keep still...\r\n");
		if (App_MPU6050_Calibrate(MPU_CALIB_SAMPLES) != 0)
		{
			// 校准期间被碰了。零偏仍是"本次能拿到的最好估计"，不会被清零，
			// 所以打印一行继续跑就行，不要卡死、更不要去清零零偏。
			Serial_Printf("calibrate FAILED (moved?)\r\n");
		}
		else
		{
			Serial_Printf("calibrate done\r\n");
		}
	}
	else
	{
		// 传感器没通就别浪费3秒空转
		Serial_Printf("MPU6050 not ready, skip calibration\r\n");
	}
	//Track_Init();
	// 零偏诊断：静止时这个值应该很小（|bias_z| < 20 就是正常，单位 0.001度/秒）。
	// 角度如果长期漂移，先看这里——零偏变大就重新上电校准，而不是去调死区。
	Serial_Printf("bias_z=%d (x1000 deg/s)\r\n", (int)(App_MPU6050_GetGyroBiasZ() * 1000.0f));
	//控制器启动
//	Control_Start();
//	Control_SetTarget(15,15);
	//开启循迹
//	Track_Start();
//	Track_ClearErrorStatistics();//相关数据复原
	Timer_Init();
	while (1)
	{
		BT_control();//需要蓝牙控制时直接调用
		// ---- 调试用：每500ms打一行（25 tick x 20ms）。验收通过后删掉 ----
		// 这里只读数值，绝不调 Update()/UpdateAngle()（会双重积分）。
		{
			static uint32_t MpuLastTick = 0;
			uint32_t now = GetTick();
			if (now - MpuLastTick >= 25)
			{
				int ang = (int)(App_MPU6050_GetAngle() * 10.0f);        // 0.1度
				int gz  = (int)(App_MPU6050_GetGz() * 100.0f);          // 0.01度/秒
				int az  = (int)(App_MPU6050_GetAz() * 1000.0f);         // 0.001g，水平应约 +1000
				int tp  = (int)(App_MPU6050_GetTemperature() * 10.0f);  // 0.1摄氏度

				MpuLastTick = now;

				Serial_Printf("t=%u ang=%d gz=%d az=%d tp=%d\r\n",
					(unsigned)now, ang, gz, az, tp);

				OLED_ShowSignedNum(1, 6, ang, 5);   // +00900 就是 +90.0度
				OLED_ShowSignedNum(2, 6, gz, 6);
			}
		}
//		Gray_Update();
//		Track_CalculateError();
		//显示灰度返回值
//		OLED_ShowNum(1,1,Gray_GetValue(0),4);
//		OLED_ShowNum(1,6,Gray_GetValue(1),4);
//		OLED_ShowNum(2,1,Gray_GetValue(2),4);
//		OLED_ShowNum(2,6,Gray_GetValue(3),4);
//		OLED_ShowNum(3,1,Gray_GetValue(4),4);
//		OLED_ShowNum(3,6,Gray_GetValue(5),4);
		//显示归一化结果
//		OLED_ShowSignedNum(1,1,Strength[0]*100,3);
//		OLED_ShowSignedNum(1,6,Strength[1]*100,3);
//		OLED_ShowSignedNum(1,11,Strength[2]*100,3);
//		OLED_ShowSignedNum(2,1,Strength[3]*100,3);
//		OLED_ShowSignedNum(2,6,Strength[4]*100,3);
//		OLED_ShowSignedNum(2,11,Strength[5]*100,3);
		//显示目标速度
//		OLED_ShowSignedNum(1,1,LeftPID.Target,3);
//		OLED_ShowSignedNum(2,1,RightPID.Target,3);
//		OLED_ShowSignedNum(1,1,Track_GetErrorMax()*100,4);
//		OLED_ShowSignedNum(2,1,Track_GetErrorMin()*100,4);
//		OLED_ShowSignedNum(3,1,Track_GetErrorAverage()*100,4);
		//显示误差
//		OLED_ShowSignedNum(4,1,Track_GetError()*100,3);
		//显示是否丟线
//		OLED_ShowNum(4,7,Track_IsLineLost(),2);
//		Delay_ms(100);
	}
}
// 定时中断函数  0.02s->20ms
void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4,TIM_IT_Update) == SET)
    {
		//灰度滤波
//		Gray_Update();   
		//循迹计算->更新目标速度
		//Track_Update();
        //读取左右轮实际速度
        LeftCount = Encoder1_GetCount();
        RightCount = Encoder2_GetCount();
		//利用编码器反馈，计算新的PWM输出  关联PID Motor Encoder模块
		Control_Update();
		// MPU6050：读14字节(约1ms) + 固定20ms步长的角度积分。
		// 放在 Control_Update() 后面，保持原有控制环时序不变。
		// 铁律：Update() 不可重入——这里调了，主循环里就绝对不能再调一次，
		//       否则两条 I2C 事务会在同一条总线上交错。
		Delay_TickUpdate();                // 给 GetTick() 累加 20ms
		App_MPU6050_Update();              // 读 I2C
		App_MPU6050_UpdateAngle(0.02f);    // 积分角度
		//清除中断标志
        TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
    }
}
//中断函数每隔20ms就会进入一次，所以在车已经停了的时候还会不断进入中断函数
//为使Control_Update()函数不会被反复调用，加入Control_Enable = 1的状态使能
