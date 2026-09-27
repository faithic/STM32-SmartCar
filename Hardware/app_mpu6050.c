#include "app_mpu6050.h"
#include "SoftI2C.h"   // 软件 I2C：PA4 = SDA，PA5 = SCL
// #include "i2c.h"    // 硬件 I2C，2026-09-20 停用（PB8/PB9 要腾给别的用途），保留备查
#include "Delay.h"
#include "Serial_Printf.h"

// 从机地址
#define MPU6050_ADDR      0x68

#define MPU_ACCEL_SENS    16384.0f  // 加速度计±2g量程，16384 LSB/g
#define MPU_GYRO_SENS     16.4f     // 陀螺仪±2000°/s量程，16.4 LSB/(°/s)

// 角度解算参数
#define MPU_YAW_SIGN      (+1.0f)   // Yaw符号：从上往下看逆时针为正。板子倒贴时改成 -1.0f
#define MPU_YAW_DEADZONE  0.0f      // 死区，单位°/s。默认0：死区会按比例吃掉慢速转动的角度，是系统性误差
                                    // 而长期漂移由Update()里的温漂零偏跟踪兜着，实测发现静止仍漂再考虑调大
#define MPU_YAW_DT_MAX    0.12f     // 单步最大积分时长，单位s：超过就丢弃这一步并重新播种

// 静止判定的参数。注意：静止只用来决定"要不要学零偏"，不再用来冻结积分

#define MPU_STILL_GYRO_ENTER 2.0f   // 进入静止：三轴校正后角速度都要小于它，单位°/s
#define MPU_STILL_GYRO_EXIT  3.0f   // 退出静止：任一轴超过它立刻退出，单位°/s
#define MPU_STILL_CONFIRM    15     // 连续这么多个样本都在门限内才认定静止
#define MPU_STILL_ACC_MIN 0.9025f   // 0.95g^2
#define MPU_STILL_ACC_MAX 1.1025f   // 1.05g^2

// 温漂零偏慢跟踪
#define MPU_BIAS_TAU       30.0f    // 跟踪时间常数，单位s。写时间常数而不是写每次乘的固定
                                    // 系数，是为了让跟踪速度不随调用频率变化（主循环17Hz
                                    // 和定时器50Hz下，固定系数会让跟踪快慢差3倍）
#define MPU_BIAS_DT_MAX    0.2f     // 算跟踪系数时对dt取的封顶，单位s：主循环卡顿后，别让零偏一步跳到当前读数
#define MPU_CALIB_SPAN_MAX 2.0f     // 校准期间允许的单轴极差，单位°/s：超了说明板子被碰了

static float ax, ay, az; // 加速度计的结果，单位g
static float temperature; // 温度计的结果，单位摄氏度
static float gx, gy, gz; // 单位°/s
static uint8_t ChipID = 0xFF; // WHO_AM_I读到的模块ID：0x68=真MPU6050；0x70=MPU6500
static uint8_t MpuReady = 0;   // 1=初始化通过，0=初始化/通信失败

// 角度解算的状态。这些变量同时被"主循环读"和"定时器中断写"访问，必须加volatile：
// 否则ARMCC在-O2下会把GetAngle()内联进主循环，把变量缓存到寄存器，永远不再从内存重读
static volatile float GyroBiasX = 0.0f;   // X轴零偏，单位°/s，用于静止判定
static volatile float GyroBiasY = 0.0f;   // Y轴零偏，单位°/s，用于静止判定
static volatile float GyroBiasZ = 0.0f;   // Z轴零偏，单位°/s（存°/s而不是原始值，以后改量程依然有效）
static volatile float AngleZ    = 0.0f;   // 累计角度，单位°，范围±180
static volatile float LastGz    = 0.0f;   // 上一次的角速度，梯形积分用
static volatile uint8_t LastGzValid = 0;  // 0=还没播种，第一次调用只记录初值不做积分
static volatile uint8_t ResetReq    = 0;  // 复位请求：主循环置位，由积分侧执行清零
static volatile uint8_t SampleFresh = 0;  // 本次I2C读取是否成功，0=数据不可信
static volatile uint8_t MotionStill = 0;  // 1=判定为静止。只影响"要不要学零偏"，不影响积分
static volatile uint16_t StillCount = 0;  // 连续满足静止门限的样本数，用于连续确认
static uint32_t LastTick = 0;             // UpdateAngleAuto用：上一次积分的时间戳

static int reg_write(uint8_t reg, uint8_t value);
static int reg_write_retry(uint8_t reg, uint8_t value, uint8_t times);
static uint8_t reg_read(uint8_t reg);
static void YawIntegrate(float gyro_z, float dt);
static void StillCheckAndTrackBias(float dt);

//
// @简介：对MPU6050进行初始化
//
void App_MPU6050_Init(void)
{
	uint8_t init_ok = 1;

	// #1. 初始化软件 I2C（PA4 = SDA，PA5 = SCL）
	//     原来用的是硬件 I2C1 重映射到 PB8/PB9
	//     旧的硬件 I2C 初始化整段注释在下面，需要时换回来即可
	IIC_Init();

	// #2. 总线恢复 + 自检
	//     IIC_BusRecover 敲完时钟后会回读两条线的空闲电平，谁被拽住就报出来。
	//     这一步顶替了硬件 I2C 时代的 My_I2C_BusRecover：软 I2C 不会卡死，但会静默失败，
	//     没有这个自检，"初始化成功"和"两根线一根都没通"在串口上长得一模一样
	switch (IIC_BusRecover())
	{
		case -1:
			Serial_Printf("SDA held low! check wiring/pull-ups\r\n");
			init_ok = 0;
			break;
		case -2:
			Serial_Printf("SCL held low! check wiring/pull-ups\r\n");
			init_ok = 0;
			break;
		default: break;
	}

	/* ============ 硬件 I2C 初始化，2026-09-20 停用，保留备查 ============
	   换回来时记得把上面的 IIC_Init()/IIC_BusRecover() 一并注释掉，
	   并把文件开头的 #include "SoftI2C.h" 换回 #include "i2c.h"

	// #1. 初始化I2C总线 PB8 PB9 - I2C1
	// 将I2C1的引脚重映射到PB8和PB9
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//打开AFIO时钟
	GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);//重映射

	// 初始化PB8和PB9 - AF_OD
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//打开GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);//打开I2C1时钟

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;  //复用开漏输出
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	// #2. 总线恢复
	// 上一次运行的固件如果在I2C传输中途被复位（典型场景：只按了MCU的复位键，没给
	// MPU6050断电），MPU6050可能还拽着SDA不放。这时I2C的BUSY位永远是1，之后每一次
	// 通信都会卡在"等总线空闲"上——现象就是"上电以后串口一个字都没有"。
	// 这里先敲9个时钟把它顶出来，再做正常的I2C初始化。这一步只在启动时做一次
	if (My_I2C_BusRecover(I2C1, GPIOB, GPIO_Pin_8, GPIO_Pin_9) != 0)
		Serial_Printf("I2C bus stuck! check pull-ups/wiring\r\n");

	// 恢复函数把引脚改成了普通开漏输出，这里必须改回复用功能，I2C才能接管这两个脚
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	// #3. 初始化I2C1
	// 设置I2C的参数
	I2C_InitTypeDef I2C_InitStruct = {0};

	I2C_InitStruct.I2C_ClockSpeed = 400000;// I2C通信速度
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; // 占空比
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;

	I2C_Init(I2C1, &I2C_InitStruct);
	==================================================================== */

	// #3. 设置MPU6050的参数 初始化MPU6050
	// 带重试：上电头几次访问偶发失败是正常的（MPU6050内部振荡器还没稳）。
	// 软 I2C 是主控自己翻转时钟的，不存在"等从机应答"的死等，所以不会卡死；
	// 重试主要是兜住上电瞬间从机还没准备好的那几次 NAK
	if (reg_write_retry(0x6b, 0x80, 3) != 0) // 复位
	{
		Serial_Printf("MPU6050 reset write failed\r\n");
		init_ok = 0;
	}
	Delay_ms(100);

	if (reg_write_retry(0x6b, 0x00, 3) != 0) // 将MPU6050从休眠模式唤醒
	{
		Serial_Printf("MPU6050 wake write failed\r\n");
		init_ok = 0;
	}

	// #4. 判断模块型号：读WHO_AM_I(0x75)
	//     0x68        = 真MPU6050
	//     0x70        = MPU6500
	//     0xFF        = I2C没响应，先查接线和供电
	//     只判断不拦截：型号不认识也继续往下跑。否则一旦读不到就卡死，串口上什么都看不到
	ChipID = reg_read(0x75);

	if (ChipID == 0x68)
		Serial_Printf("WHO_AM_I=0x%02X (MPU6050)\r\n", ChipID);
	else if (ChipID == 0xFF)
	{
		Serial_Printf("WHO_AM_I=0xFF - I2C no response!\r\n");
		init_ok = 0;
	}
	else
		Serial_Printf("WHO_AM_I=0x%02X (MPU6500/clone)\r\n", ChipID);

	if (reg_write_retry(0x1a, 0x03, 3) != 0) // CONFIG：内置低通滤波，抑制电机振动干扰
	{
		Serial_Printf("MPU6050 config write failed\r\n");
		init_ok = 0;
	}

	// 陀螺仪量程±2000°/s（灵敏度16.4 LSB/(°/s)）
	// 量程必须给够：原地转弯角速度可达300~400°/s，±250会饱和切峰
	if (reg_write_retry(0x1b, 0x18, 3) != 0)
	{
		Serial_Printf("MPU6050 gyro config failed\r\n");
		init_ok = 0;
	}

	if (reg_write_retry(0x1c, 0x00, 3) != 0) // 将加速度计的量程设置为+-2g
	{
		Serial_Printf("MPU6050 accel config failed\r\n");
		init_ok = 0;
	}

	MpuReady = init_ok;
	if (!MpuReady)
		Serial_Printf("MPU6050 init FAILED, data output paused\r\n");
}

uint8_t App_MPU6050_IsReady(void)
{
	return MpuReady;
}

//
// @简介：更新mpu6050的值
// @返回值：0 - 读取成功，-1 - I2C通信失败（此时数据不可信，不要拿去积分）
//
int App_MPU6050_Update(void)
{
	if (!MpuReady)
	{
		SampleFresh = 0;
		return -1;
	}

	// 从0x3B开始一次性连读14字节（ax/ay/az/温度/gx/gy/gz 各2字节）
	uint8_t buf[14];

	// 返回值必须检查：读失败时buf里是残留值，换算出来的角速度可能高达±2000°/s，
	// 拿去积分会凭空造出几十度的假旋转。小车跑起来电机电流会干扰I2C，偶发失败是常态
	if (IIC_ReadRegs(MPU6050_ADDR, 0x3b, buf, 14) != 0)
	// if (My_I2C_RegReadBytes(I2C1, 0xd0, 0x3b, buf, 14) != 0)  // 硬件 I2C 版本，已停用
	{
		SampleFresh = 0;
		return -1;
	}

	int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]); // ax的原始数据
	int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]); // ay的原始数据
	int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]); // az的原始数据

	ax = ax_raw / MPU_ACCEL_SENS;
	ay = ay_raw / MPU_ACCEL_SENS;
	az = az_raw / MPU_ACCEL_SENS;

	int16_t temperature_raw = (int16_t)((buf[6] << 8) | buf[7]); // 温度的原始数据


	if (ChipID == 0x68)
		temperature = temperature_raw / 340.0f + 36.53f; // MPU6050
	else
		temperature = temperature_raw / 333.87f + 21.0f; // MPU6500及山寨片

	int16_t gx_raw = (int16_t)((buf[8] << 8) | buf[9]); // gx的原始数据
	int16_t gy_raw = (int16_t)((buf[10] << 8) | buf[11]); // gy的原始数据
	int16_t gz_raw = (int16_t)((buf[12] << 8) | buf[13]); // gz的原始数据

	gx = gx_raw / MPU_GYRO_SENS; // ±2000°/s量程，灵敏度16.4 LSB/(°/s)
	gy = gy_raw / MPU_GYRO_SENS;
	gz = gz_raw / MPU_GYRO_SENS;

	// 静止判定和温漂零偏跟踪不在这里做了，挪到StillCheckAndTrackBias()里（2026-09-24）。
	// 原因：这里是主循环调用、频率随负载浮动（实测17Hz），而YawIntegrate在小车工程里
	// 由TIM4的20ms中断调用。判定和积分必须在同一个时基上，否则中断会拿着一个陈旧好几拍
	// 的判定位反复积分。挪过去之后零偏跟踪的时间常数也能按真实dt算，不再依赖调用频率

	SampleFresh = 1;
	return 0;
}

// ============================================================================
// 角度解算（Yaw）
//
// 现在改成：角速度永远照常积分，长期漂移交给零偏慢跟踪兜。静止时零偏收敛到原始
// 均值，残差趋零，积分自然不漂——冻结本来就是冗余的。被它挡掉的那点噪声，按
// σ_θ=σ_g·√(dt·T)算每分钟只有约0.1°
// ============================================================================

//
// @简介：陀螺仪零偏校准（必须在传感器完全静止时调用）
// @参数 samples - 采样组数，MPU_CALIB_SAMPLES(500)约需1.2秒
// @返回值：0 - 校准成功，-1 - 校准结果不可信（期间被碰了，或I2C通信故障）
// @备注：无论返回值是几，零偏都会被写成"本次能拿到的最好的估计"，不会被清零。
//        清零是危险的：零偏为0而真实零偏大于静止门限时，StillCheckAndTrackBias()
//        里的零偏跟踪会永久死锁（永远不静止→永远不学→永远不静止）。
//        另外调用本函数前定时器中断必须还没开始跑YawIntegrate，否则会和它抢零偏
//
int App_MPU6050_Calibrate(uint16_t samples)
{
	uint8_t attempt;
	uint8_t have_result = 0;
	float best_span = 0.0f;
	float best_x = 0.0f, best_y = 0.0f, best_z = 0.0f;

	// #1. 预热：刚唤醒时芯片内部还没稳定，先等一会儿再采
	Delay_ms(100);

	// #2. 丢弃前10个采样：唤醒后最初的几个数据不可靠
	for (uint8_t i = 0; i < 10; i++)
	{
		App_MPU6050_Update();
		Delay_ms(2);
	}

	// #3. 最多采3轮，取"最稳"的那一轮。校准期间被碰一下是常事（挪板子、接线），
	//     一轮不合格就整段判失败的话，上电时旧零偏是0，正好踩中上面说的死锁
	for (attempt = 0; attempt < 3; attempt++)
	{
		float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
		float min_x = 1e9f, max_x = -1e9f;
		float min_y = 1e9f, max_y = -1e9f;
		float min_z = 1e9f, max_z = -1e9f;
		uint16_t valid = 0; // 真正读成功的样本数，和samples可能不等
		float span;

		for (uint16_t i = 0; i < samples; i++)
		{
			if (App_MPU6050_Update() == 0) // 读失败的样本不参与平均
			{
				sum_x += gx;
				sum_y += gy;
				sum_z += gz;

				if (gx < min_x) min_x = gx;
				if (gx > max_x) max_x = gx;
				if (gy < min_y) min_y = gy;
				if (gy > max_y) max_y = gy;
				if (gz < min_z) min_z = gz;
				if (gz > max_z) max_z = gz;

				valid++;
			}
			Delay_ms(2);
		}

		if (valid < samples / 2u) // 读成功率太低，这一轮的平均值不可信
		{
			Delay_ms(100);
			continue;
		}

		// #4. 判"动没动"看极差，不看零偏的绝对值。未校准的MPU6050零偏按手册
		//     可以到±20°/s，绝对值大只说明片子零偏大，不代表校准失败；而极差大
		//     一定说明这段时间里角速度在变，也就是板子被碰了
		span = max_x - min_x;
		if (max_y - min_y > span) span = max_y - min_y;
		if (max_z - min_z > span) span = max_z - min_z;

		if (!have_result || span < best_span)
		{
			have_result = 1;
			best_span = span;
			best_x = sum_x / (float)valid;
			best_y = sum_y / (float)valid;
			best_z = sum_z / (float)valid;
		}

		if (span <= MPU_CALIB_SPAN_MAX) break; // 这一轮够稳，不用再采了

		Delay_ms(100);
	}

	if (!have_result) return -1; // 一次都没读成功，零偏保持原样

	// 零偏存成°/s而不是原始值：这样以后改量程，零偏依然有效（原始值会差8倍）
	GyroBiasX = best_x;
	GyroBiasY = best_y;
	GyroBiasZ = best_z;

	// #5. 校准完成后角度从0开始，静止判定也重新起算
	AngleZ = 0.0f;
	LastGz = 0.0f;
	LastGzValid = 0;
	LastTick = GetTick();
	MotionStill = 0;
	StillCount = 0;

	return (best_span <= MPU_CALIB_SPAN_MAX) ? 0 : -1; // 三轮都被碰了
}

//
// @简介：静止判定 + 温漂零偏慢跟踪（static，只在本文件内部使用）
// @参数 dt - 距离上一次调用的时长，单位s
// @备注：必须在App_MPU6050_Update()之后调用，用的是本次刷新出来的gx/gy/gz/ax/ay/az。
//        判定结果只决定"要不要学零偏"，不再干预积分——见文件里"角度解算"那段的说明
//
static void StillCheckAndTrackBias(float dt)
{
	float acc_norm_sq;
	float gx_corr, gy_corr, gz_corr;
	float a;

	// #1. 判定看的是"减掉当前零偏之后"的角速度，它是对真实转速最好的估计。
	//     代价是自指：零偏错→判定错→零偏更错。兜底靠两点：
	//     a) 进入门限放到2.0°/s（原来0.8），给Calibrate()的残差留够余量
	//     b) Calibrate()失败时不再把零偏清零（见该函数）。零偏=0而真实零偏大于
	//        门限时这套逻辑会永久死锁：永远不静止→永远不学→永远不静止
	acc_norm_sq = ax * ax + ay * ay + az * az;
	gx_corr = gx - GyroBiasX;
	gy_corr = gy - GyroBiasY;
	gz_corr = gz - GyroBiasZ;

	// #2. 迟滞：进入静止要求三轴都小于ENTER，退出只要任一轴超过EXIT。
	//     单门限会让角速度在门限附近抖动时判定逐样本翻转，而每翻转一次就把零偏
	//     朝当时的读数拽一下，累积成单向误差
	if (MotionStill)
	{
		if (acc_norm_sq < MPU_STILL_ACC_MIN || acc_norm_sq > MPU_STILL_ACC_MAX ||
			gx_corr > MPU_STILL_GYRO_EXIT || gx_corr < -MPU_STILL_GYRO_EXIT ||
			gy_corr > MPU_STILL_GYRO_EXIT || gy_corr < -MPU_STILL_GYRO_EXIT ||
			gz_corr > MPU_STILL_GYRO_EXIT || gz_corr < -MPU_STILL_GYRO_EXIT)
		{
			MotionStill = 0;
			StillCount = 0;
		}
	}
	else if (acc_norm_sq > MPU_STILL_ACC_MIN && acc_norm_sq < MPU_STILL_ACC_MAX &&
			gx_corr > -MPU_STILL_GYRO_ENTER && gx_corr < MPU_STILL_GYRO_ENTER &&
			gy_corr > -MPU_STILL_GYRO_ENTER && gy_corr < MPU_STILL_GYRO_ENTER &&
			gz_corr > -MPU_STILL_GYRO_ENTER && gz_corr < MPU_STILL_GYRO_ENTER)
	{
		// #3. 连续确认：中途被打断就重新计数。避免转弯减速时穿过门限的那一两拍
		//     被当成静止，把还没转完的角度学进零偏
		if (StillCount < MPU_STILL_CONFIRM) StillCount++;

		if (StillCount >= MPU_STILL_CONFIRM) MotionStill = 1;
	}
	else
	{
		StillCount = 0;
	}

	if (!MotionStill) return;

	// #4. 一阶低通，时间常数与采样率无关：α = dt/τ（dt远小于τ时，这个线性形式和
	//     1-exp(-dt/τ)数值上没有差别，省掉一次expf）。写成固定系数（比如原来的
	//     0.001）会把跟踪速度绑死在调用频率上：主循环17Hz和定时器50Hz下差3倍
	if (dt > MPU_BIAS_DT_MAX) dt = MPU_BIAS_DT_MAX; // 卡顿后别让零偏一步跳到当前读数
	a = dt / MPU_BIAS_TAU;

	GyroBiasX += a * (gx - GyroBiasX);
	GyroBiasY += a * (gy - GyroBiasY);
	GyroBiasZ += a * (gz - GyroBiasZ);
}

//
// @简介：Yaw角度积分的核心（static，只在本文件内部使用）
// @参数 gyro_z - 已减去零偏的Z轴角速度，单位°/s
// @参数 dt - 距离上一次积分的时长，单位s
//
static void YawIntegrate(float gyro_z, float dt)
{
	// #1. 处理复位请求。AngleZ的写权限只给积分侧：如果在主循环里直接写AngleZ=0，
	//     可能正好被中断里"读-改-写"的最后一步覆盖掉，复位会静默失败
	if (ResetReq)
	{
		ResetReq = 0;
		AngleZ = 0.0f;
		LastGzValid = 0;
		return;
	}

	// #2. 本次I2C读取失败：不能拿上一次的陈旧数据继续积分
	if (!SampleFresh) return;

	if (dt <= 0.0f) return;

	// #3. 静止判定 + 零偏跟踪。和积分共用同一个dt，所以判定、跟踪、积分三者同拍。
	//     放在dt有效性检查之前：主循环卡顿后这一步不做积分，但零偏该更新还是要更新
	StillCheckAndTrackBias(dt);

	// #4. 步长不可信（比如中间停顿了很久）：丢弃这一步并重新播种，
	//     否则会用一个跨越了长时间空档的梯形去积分
	if (dt > MPU_YAW_DT_MAX)
	{
		LastGzValid = 0;
		return;
	}

	// #5. 统一符号方向，再做死区滤波
	gyro_z *= MPU_YAW_SIGN;

	if (gyro_z > -MPU_YAW_DEADZONE && gyro_z < MPU_YAW_DEADZONE)
		gyro_z = 0.0f;

	// #6. 第一次调用只记录初值，不做积分（否则会凭空多算半个采样）
	if (!LastGzValid)
	{
		LastGz = gyro_z;
		LastGzValid = 1;
		return;
	}

	// #7. 梯形积分：取前后两点的平均值，比矩形积分更贴合真实转动
	AngleZ += 0.5f * (LastGz + gyro_z) * dt;
	LastGz = gyro_z;

	// #8. 折返到±180°，防止长时间同向转动后数值无限增长
	while (AngleZ > 180.0f) AngleZ -= 360.0f;
	while (AngleZ < -180.0f) AngleZ += 360.0f;
}

//
// @简介：更新角度积分（定时器中断专用，显式传入时间间隔）
// @参数 dt - 距离上一次调用的时长，单位s
// @备注：小车工程挂在TIM4的20ms中断里，直接传0.02f即可。
//        不要在中断里调用App_MPU6050_Update()读I2C，只在这里做纯数学运算。
//        调用前必须先调用App_MPU6050_Update()刷新gz
//
void App_MPU6050_UpdateAngle(float dt)
{
	YawIntegrate(gz - GyroBiasZ, dt);
}

//
// @简介：更新角度积分（主循环专用，自动测量时间间隔）
// @备注：内部用GetTick()测dt（1ms分辨率），只适合主循环调用。
//        如果在定时器中断里调用，请改用App_MPU6050_UpdateAngle()传固定dt
//
void App_MPU6050_UpdateAngleAuto(void)
{
	uint32_t now = GetTick();
	uint32_t elapsed = now - LastTick; // 无符号减法，计数器回绕时结果依然正确

	LastTick = now;

	YawIntegrate(gz - GyroBiasZ, (float)elapsed * 0.001f);
}

//
// @简介：获取Z轴累计角度，单位°，范围±180
//
float App_MPU6050_GetAngle(void)
{
	return AngleZ;
}

//
// @简介：把当前角度清零（比如每次转弯前重置参考方向）
// @备注：这里只置请求标志，真正的清零由积分侧执行，避免和中断抢着写AngleZ
//
void App_MPU6050_ResetAngle(void)
{
	ResetReq = 1;
}

//
// @简介：获取Z轴零偏，单位°/s（校准后打印出来，零偏变大时才能诊断）
//
float App_MPU6050_GetGyroBiasZ(void)
{
	return GyroBiasZ;
}

// ============================================================================
// 以下为读取原始数据的接口
// ============================================================================

//
// @简介：获取x轴向加速度，单位g
//
float App_MPU6050_GetAx(void)
{
	return ax;
}

//
// @简介：获取y轴向加速度，单位g
//
float App_MPU6050_GetAy(void)
{
	return ay;
}

//
// @简介：获取z轴向加速度，单位g
//
float App_MPU6050_GetAz(void)
{
	return az;
}

//
// @简介：获取温度计的值，单位摄氏度
//
float App_MPU6050_GetTemperature(void)
{
	return temperature;
}

//
// @简介：获取绕x轴的角速度，单位°/s
//
float App_MPU6050_GetGx(void)
{
	return gx;
}

//
// @简介：获取绕y轴的角速度，单位°/s
//
float App_MPU6050_GetGy(void)
{
	return gy;
}

//
// @简介：获取绕z轴的角速度，单位°/s
//
float App_MPU6050_GetGz(void)
{
	return gz;
}

//
// @简介：向寄存器写值
// @参数 reg - 要写入的寄存器的地址
// @参数 value - 要写入的值
// @返回值：0 - 成功，其它 - 失败（返回值约定见SoftI2C.h）
//
static int reg_write(uint8_t reg, uint8_t value)
{
	return IIC_WriteRegs(MPU6050_ADDR, reg, &value, 1);

	// 硬件 I2C 版本，已停用：
	// uint8_t bytesToSend[] = {reg, value};
	// return My_I2C_SendBytes(I2C1, 0xd0, bytesToSend, 2);
}

//
// @简介：向寄存器写值，失败自动重试
// @参数 reg - 要写入的寄存器的地址
// @参数 value - 要写入的值
// @参数 times - 最多尝试几次
// @返回值：0 - 成功，其它 - 最后一次失败的返回值
//
static int reg_write_retry(uint8_t reg, uint8_t value, uint8_t times)
{
	int r = -1;
	uint8_t i;

	for(i = 0; i < times; i++)
	{
		r = reg_write(reg, value);
		if(r == 0) return 0;

		Delay_ms(2); // 失败后缓一下再重试，给从机时间恢复
	}

	return r;
}

//
// @简介：读取寄存器的值
// @参数 reg - 要读取的寄存器的地址
// @返回值：读到的值；通信失败时返回0xFF
// @备注：一次"写寄存器地址 -> 重复起始 -> 读"完成，中间不产生STOP，符合MPU6050手册的
//        单字节读时序。硬件I2C那版是分成两次调用、中间插了一个STOP，而且两次返回值
//        都被丢弃，读没读到根本看不出来
//
static uint8_t reg_read(uint8_t reg)
{
	uint8_t regValue = 0xFF; // 初始化为0xFF：通信失败时返回0xFF而不是内存随机值

	if (IIC_ReadRegs(MPU6050_ADDR, reg, &regValue, 1) != 0)
		regValue = 0xFF; // 读失败：保持0xFF，让上层能看出"没读到"而不是拿到残留数据

	// 硬件 I2C 版本，已停用：
	// My_I2C_SendBytes(I2C1, 0xd0, &reg, 1); // 发送寄存器的地址
	// My_I2C_ReceiveBytes(I2C1, 0xd0, &regValue, 1); // 读取一个字节

	return regValue;
}
