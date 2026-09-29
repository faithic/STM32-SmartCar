#ifndef __SERIAL_PRINTF_H
#define __SERIAL_PRINTF_H

// 给蓝牙串�?USART3)用的精简 printf�?
//
// 支持�?d %u %x %X %c %s %%  以及可选的 '0' 标志和宽度，例如 %02X  %3d  %5u
// 不支持浮点。要打浮点请先定点化再当整数打：
//     Serial_Printf("ang=%d", (int)(App_MPU6050_GetAngle() * 10.0f));   // 0.1° 分辨�?
//
// 为什么不�?C 库的 sprintf：这个函数一个库符号都不引用，不存在链接期意�?
// （本工程 useUlib=0，不�?MicroLIB，printf 家族的行为和 MicroLIB 不一样）�?
// 而且它是边格式化边发字节，没�?codex 那边 char buf[128] + 无边�?vsprintf 的溢出隐患�?
//
// 【只能在主循环里调�?600 波特下一个字符约 1ms，一�?40 字符就是 42ms�?
// 超过 TIM4 �?20ms 中断周期。在中断里调它会周期性自锁�?
void Serial_Init(void);
void Serial_Printf(const char *Fmt, ...);

#endif

