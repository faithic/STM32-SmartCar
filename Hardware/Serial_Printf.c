#include "stm32f10x.h"
#include "Serial_Printf.h"
#include <stdarg.h>

// ============================ 底层输出 ============================

#include "stm32f10x.h"
#include "Serial_Printf.h"
#include <stdarg.h>

void Serial_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					
	
	USART_InitTypeDef USART_InitStructure;					
	USART_InitStructure.USART_BaudRate = 9600;			
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;		
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		
	USART_Init(USART3, &USART_InitStructure);				
	
	USART_Cmd(USART3, ENABLE);								
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART3, Byte);		
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);	
}

static void SP_PutChar(char c)
{
	Serial_SendByte((uint8_t)c);
}

static void SP_PutString(const char *s)
{
	while (*s != '\0')
	{
		SP_PutChar(*s++);
	}
}

// �?Value �?Base(10�?6) 进制打印，右对齐�?Width �?
// Upper   != 0 -> 用大�?A-F，否则小�?
// ZeroPad != 0 -> �?'0'，否则补空格
static void SP_PutNumber(unsigned long Value, unsigned char Base,
                         unsigned char Upper, unsigned char Width,
                         unsigned char ZeroPad)
{
	char Buf[12];               // 32位十进制最�?0位，十六进制8位，够用
	unsigned char n = 0;
	unsigned char i;

	if (Value == 0)
	{
		Buf[n++] = '0';
	}
	else
	{
		while (Value != 0)
		{
			unsigned char d = (unsigned char)(Value % Base);
			Value /= Base;
			if (d < 10)
			{
				Buf[n++] = (char)('0' + d);
			}
			else
			{
				Buf[n++] = (char)((Upper ? 'A' : 'a') + (d - 10));
			}
		}
	}

	// 位数不够就在左边补齐
	for (i = n; i < Width; i++)
	{
		SP_PutChar(ZeroPad ? '0' : ' ');
	}

	// 上面是低位先出的，倒着打回�?
	while (n > 0)
	{
		SP_PutChar(Buf[--n]);
	}
}

// ============================ 对外接口 ============================

void Serial_Printf(const char *Fmt, ...)
{
	va_list ap;
	const char *p = Fmt;

	va_start(ap, Fmt);

	while (*p != '\0')
	{
		unsigned char ZeroPad;
		unsigned char Width;

		if (*p != '%')
		{
			SP_PutChar(*p);
			p++;
			continue;
		}

		p++;                                    // 跳过 '%'

		ZeroPad = 0;
		Width   = 0;

		if (*p == '0')
		{
			ZeroPad = 1;
			p++;
		}
		while (*p >= '0' && *p <= '9')
		{
			Width = (unsigned char)(Width * 10 + (unsigned char)(*p - '0'));
			p++;
		}

		switch (*p)
		{
		case 'd':
			{
				// 注意：uint8_t 实参会按默认实参提升变成 int，所以这里必�?
				// �?int 取，不能�?va_arg(ap, uint8_t)
				int v = va_arg(ap, int);
				if (v < 0)
				{
					SP_PutChar('-');
					// �?0u - (unsigned)v 而不�?-(long)v�?
					// v �?INT_MIN 时前者依然正�?
					SP_PutNumber((unsigned long)(0u - (unsigned int)v),
					             10, 0, Width, ZeroPad);
				}
				else
				{
					SP_PutNumber((unsigned long)v, 10, 0, Width, ZeroPad);
				}
			}
			p++;
			break;

		case 'u':
			SP_PutNumber((unsigned long)va_arg(ap, unsigned int),
			             10, 0, Width, ZeroPad);
			p++;
			break;

		case 'x':
			SP_PutNumber((unsigned long)va_arg(ap, unsigned int),
			             16, 0, Width, ZeroPad);
			p++;
			break;

		case 'X':
			SP_PutNumber((unsigned long)va_arg(ap, unsigned int),
			             16, 1, Width, ZeroPad);
			p++;
			break;

		case 'c':
			SP_PutChar((char)va_arg(ap, int));
			p++;
			break;

		case 's':
			{
				const char *s = va_arg(ap, const char *);
				if (s == 0)
				{
					s = "(null)";
				}
				SP_PutString(s);
			}
			p++;
			break;

		case '%':
			SP_PutChar('%');
			p++;
			break;

		default:
			// 不认识的格式符（比如写错�?%D），原样打出来让人一眼看见，
			// 而不是静默吞掉。字符串末尾孤零零一�?'%' 也走这里�?
			if (*p == '\0')
			{
				break;
			}
			SP_PutChar('%');
			SP_PutChar(*p);
			p++;
			break;
		}
	}

	va_end(ap);
}

