/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : PC001V21.c
* Author             : WOW
* Version            : V2.0.1
* Date               : 06/26/2017
* Description        : PC001V21层控制板.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifdef PL009V31HC				//称重LCD板---双称重

#include "PL009V31HC.H"

#include "R61509V.h"
#include "CS5530.H"


#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_PWM.H"
#include "STM32_USART.H"


#include "SWITCHID.H"

#include "string.h"				//串和内存操作函数头文件
#include "stdlib.h"				//串和内存操作函数头文件
#include "stm32f10x_dma.h"

#define TestChN1		//单通道测试,屏蔽后为双通道
#define TestFloat		//未启用通道不配置，屏蔽后：未开启通道会主动关闭,双通道时无效


//#define	Master	1		//0--从机，1-主机
u16	DelayTime=0x1000;
u16	LCDTime=0;
u16	DSPTime=0;

u8 hour=23,min=15,second=40;
u16 millisecond=0;

//GT32L32Def 	GT32L32;
//char	Char_Buffer[256];		//记录format内码


RS485Def  RS485;
#define	Rs485Size	256
u8 RxdBuffe[Rs485Size]={0};
u8 RevBuffe[Rs485Size]={0};
u8 TxdBuffe[Rs485Size]={0};
u8 RxFlg=0;
u16 RxNum=0;
u32	Rs485_Time	=	0;
u8	SlaveID	=	0;
u32 Rev_ADC	=	0;
u8	Rev_ID	=	0;



SwitchDef	SWITCHID;
u8 SwitchID=0;	//拔码开关地址


LCDDef	sLCD;


//R61509VDef R61509V;
u8	DspFlg	=	0;

u16 BKlight	=	0;

u8 WriteFlag	=	0;
u16	row	=	1;



CS5530Def CS5530;
CS5530Def CS5530B;
u32	CS5530_Time	=	0;
u32	CS5530_ADC_CMP			=	0;
u32 CS5530_ADC_Value		=0xFFFFFFFF;
u32 CS5530_ADC_ValueB		=0xFFFFFFFF;
u32 CS5530_ADC_Valuebac	=0xFFFFFFFF;

u32	WeigthARR[100]	=	{0};
u8	DspFlg1	=	0;
u32	We1	=	0;
u8	NuW	=	0;
u32	BsW	=	0;	//空重
u32 SsW	=	0;	//单重
u8	St	=	0;	//步骤
u8	SN	=	0;	//数量
u8	SNb	=	0;	//备存数量

u16 weighVar[20]={0};
u8	VarN	=	0;

u32 Rand	=	0;
//============LCD驱动调试变量
u16 HX	=	0;
u16	HY	=	0;
unsigned short	Color	=	0;
u8 Rxdata	=0;
u32 SYSTIME	=	0;
/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void PL009V31HC_Configuration(void)
{
	SYS_Configuration();					//系统配置---打开系统时钟 STM32_SYS.H
	
	GPIO_DeInitAll();							//将所有的GPIO关闭----V20170605	
	
//	SwitchID_Configuration();
//	
//	CS5530_Configuration();
	
	RS485_Configuration();
	
	LCD_Configuration();	
	
	LCD_Printf(0,32,32,LCD565_CYAN,"LCD初始化完成");  //后边的省略号就是可变参数
	
	SysTick_Configuration(1000);	//系统嘀嗒时钟配置72MHz,单位为uS
	
//	IWDG_Configuration(2000);			//独立看门狗配置---参数单位ms	

	PWM_OUT(TIM2,PWM_OUTChannel1,500,900);	//PWM设定-20161127版本--运行指示灯
	USART_DMA_ConfigurationNR	(USART1,115200,64);	//USART_DMA配置--查询方式，不开中断
//	PWM_OUT(TIM2,PWM_OUTChannel4,500,200);		//PWM设定-20161127版本--背光
//	LCD_Clean(LCD565_RED);			//清除屏幕函数--
//	LCD_Printf(100,100,32,"时钟");					//后边的省略号就是可变参数
//	LCD_Printf(100,130,32,"%02d:",hour);		//后边的省略号就是可变参数
//	LCD_Printf(148,130,32,"%02d:",min);			//后边的省略号就是可变参数
//	LCD_Printf(196,130,32,"%02d",second);		//后边的省略号就是可变参数
//	R61509V_DrawCircle(200,120, 100, 1, R61509V_YELLOW );		//画一个圆形框
	
//	Display.Init	=	PL010V15_Configuration;
//	GPIO_Configuration_OPP50(GPIOB,GPIO_Pin_12);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
//	GPIO_SetBits(GPIOB,GPIO_Pin_12);
}
/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void PL009V31HC_Server(void)
{	
//	GPIO_Toggle	(GPIOB,	GPIO_Pin_3);		//将GPIO相应管脚输出翻转----V20170605
//	IWDG_Feed();				//独立看门狗喂狗
//	RS485_Server();			//通讯管理---负责信息的接收与发送
	LCD_Server();				//显示服务相关
//	CS5530_Server();	//称重服务，AD值处理，获取稳定值
//	SwitchID_Server();	//拔码开关处理--动态更新拨码地址
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void CS5530_Server(void)		//称重服务，AD值处理，获取稳定值
{
	CS5530_Process(&CS5530);
	
	if((CS5530_ADC_Value	!=	CS5530.Data.WeighLive)&&(0	!=	CS5530.Data.WeighLive)&&(0xFFFFFFFF	!=	CS5530.Data.WeighLive))
	{
		CS5530_ADC_Value	=	CS5530.Data.WeighLive;
		CS5530_ADC_Value	=	CS5530.Data.WeighLive;
		USART_DMAPrintf	(USART1,"CH1:%0.8X\r\n",CS5530_ADC_Value>>2);					//自定义printf串口DMA发送程序,后边的省略号就是可变参数
		return;
	}
#ifndef TestChN1	//双通道
//	return;
	CS5530_Process(&CS5530B);
	if((CS5530_ADC_Value	!=	CS5530B.Data.WeighLive)&&(0	!=	CS5530B.Data.WeighLive)&&(0xFFFFFFFF	!=	CS5530B.Data.WeighLive))
	{
		CS5530_ADC_ValueB	=	CS5530B.Data.WeighLive;
		CS5530_ADC_ValueB	=	CS5530B.Data.WeighLive;
		USART_DMAPrintf	(USART1,"CH2:%0.8X\r\n",CS5530_ADC_ValueB>>2);					//自定义printf串口DMA发送程序,后边的省略号就是可变参数
		return;
	}
#endif
//	if(CS5530_Time++>=50)		//1秒钟
//	{		
//		CS5530_Time=0;
//		CS5530_ADC_Valuebac=CS5530_ReadData(&CS5530);	//读取AD值，如果返回0xFFFFFFFF,则未读取到24位AD值
//		if(CS5530_ADC_Valuebac!=0xFFFFFFFF)
//		{
//			
////			CS5530_ADC_Valuebac>>=5;		//19位精度
//			
//			if(CS5530_ADC_Value==CS5530_ADC_Valuebac)
//			{
//				DspFlg	=	0;
//			}
//			if(CS5530_ADC_Value>=CS5530_ADC_Valuebac)
//			{
//				CS5530_ADC_CMP	=	CS5530_ADC_Value-CS5530_ADC_Valuebac;
//			}
//			else
//			{
//				CS5530_ADC_CMP	=	CS5530_ADC_Valuebac-CS5530_ADC_Value;				
//			}
//			DspFlg	=	1;
//			CS5530_ADC_Value	=	CS5530_ADC_Valuebac;
//			
//			if(CS5530_ADC_CMP>100)
//			{
//				NuW	=	0;
//				return;
//			}
//			WeigthARR[NuW]	=	CS5530_ADC_Value;
//			if(NuW++>=20-1)
//			{
//				long	temp	=	0;
//				//We1
//				
//				u8 i	=	0;
//				
//				temp	=	CS5530_GetWeightUseMedinaFilter(&CS5530);
//				
//				if(temp>200)
//				{
//					NuW	=	0;
//					return;
//				}
//				NuW	=	0;
//				We1	=	WeigthARR[i];
//				for(i=1;i<20;i++)
//				{
//					We1	=	(We1+WeigthARR[i])/2;
//				}
//				if(St	==	1)
//				{
//					St=0;
//					BsW	=	We1;
//				}
//				else if(St	==	2)
//				{
//					SsW	=	(We1	-	BsW)/10;
//					St	=	3;
//					SN	=	10;
//					SNb	=	10;
//				}
//				else if(St	==	3)
//				{
//					SN	=	(We1-BsW+500)/SsW;	//计数数量
//					if(SN!=SNb)
//					{
//						SNb	=	SN;
//						PL010V13_PrintfString(0		,128,32	,"数量%0.3d",SNb);				//待发药槽位，后边的省略号就是可变参数
//					}
//				}
//				DspFlg1	=	1;
//			}
//				
//		}
//		else
//		{
////			We1	=0;
////			NuW	=0;
//		}
//	}
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void LCD_Server(void)			//显示服务相关
{
	#if 1
//	if(ATTime++>1000)
//	{
//		ATTime=0;
//		if(ATNum++>4)
//			ATNum=0;
//		LCD_Clean(LCD565_BLACK); 					//清除屏幕函数
//	}
//	LCD_ShowAntenna(300,0,ATNum,200);   //显示12x12天线
	RxNum=RS485_ReadBufferIDLE(&RS485,RxdBuffe);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数，然后重新将接收缓冲区地址指向RxdBuffer
	if(RxNum)
	{
		u16 temp	=	0;
		u16 PenColor	=LCD565_RED;
		if(0	!=	WriteFlag)
		{
			WriteFlag	=	0;
			PenColor	=LCD565_WHITE;
		}
		else
		{
			WriteFlag	=	1;
			PenColor	=LCD565_GREEN;
		}
//		temp	=	8*3*RxNum;
//		if(0<temp%400)
//		{
//			temp	=	(temp/400+1)*16;
//		}
//		else
//		{
//			temp	=	(temp/400)*16;
//		}
		if(row>=240-32)
		{
			LCD_Clean(LCD565_BLACK); 					//清除屏幕函数
			row	=	0;
		}
		
		LCD_ShowHex(0,row,16,PenColor,RxNum,8,RxdBuffe);                //显示十六进制数据
		temp	=	8*3*RxNum;
		if(0<temp%400)
		{
			row	+=	(temp/400+1)*16;
		}
		else
		{
			row	+=	(temp/400)*16;
		}
		if(row>=240-32)
		{
//			row=0;
//			LCD_Clean(LCD565_BLACK); 					//清除屏幕函数
		}
//		LCD_Show(0,100,32,RxNum,RxdBuffe);
//		PWM_OUT(TIM3,PWM_OUTChannel3,500,800);		//PWM设定-20161127版本--背光
	}
	return;
#endif
#if 0
	u32 LCDTime=100;
//	u8 i	=	0;
	for(Color	=	0;Color<=0xFFFF;)
	{
		LCD_Fill(10,10,380,20,Color);			//清除屏幕函数--
//		LCD_Clean(Color);			//清除屏幕函数--
		SysTick_DeleymS(LCDTime);				//SysTick延时nmS
		Color+=50;
//		GPIO_Toggle	(GPIOB,	GPIO_Pin_3);		//将GPIO相应管脚输出翻转----V20170605
	}
#endif
#if 0
	millisecond++;
	if(millisecond>=999)
	{
//		SSD1963_DrawRectangle(100,100,110,110);
		millisecond=0;
		second++;
		if(second>59)
		{
			second=0;
			min++;			
			if(min>59)
			{
				min=0;
				hour++;
				if(hour>23)
				{
					hour=0;
				}
				LCD_Printf(100,130,32,"%02d:",hour);		//后边的省略号就是可变参数
				
			}
			LCD_Printf(148,130,32,"%02d:",min);		//后边的省略号就是可变参数
		}
		LCD_Printf(196,130,32,"%02d",second);		//后边的省略号就是可变参数
	}
#endif
#if 0		//画圆
	unsigned short i	=	0;
	for(i=1;i<100;i++)
	{
		LCD_DrawCircle(200,	120, i, 0, LCD565_WHITE );			//画一个圆形框	
	}
	for(i=100;i>0;i--)
	{
		LCD_DrawCircle(200,	120, i, 0, LCD565_BLACK );			//画一个圆形框	
	}
	LCD_DrawCircle(50,	60, 50, 0, LCD565_YELLOW );			//画一个圆形框	
	LCD_DrawCircle(340,	60,	50, 0, LCD565_YELLOW );			//画一个圆形框	
	LCD_DrawCircle(50,	180, 50, 0, LCD565_YELLOW );			//画一个圆形框	
	LCD_DrawCircle(340,	180, 50, 0, LCD565_YELLOW );			//画一个圆形框	
	LCD_DrawCircle(200,	120, 100, 0, LCD565_YELLOW );			//画一个圆形框	
	
	SysTick_DeleymS(500);					//SysTick延时nmS
	LCD_Clean(LCD565_BLACK);		//清除屏幕函数--
#endif
#if 0		//画圆
//	LCD_Fill(10,10,50,50,LCD565_YELLOW);	//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
	
//	LCD_DrawRectangle(10,130,60,180,LCD565_YELLOW);			//画一个矩形框
	
//	R61509V_ShowChar(10,120,32,5,"sdsdf");
	
	LCD_DrawCircle(50,	50, 50, 1, LCD565_YELLOW );			//画一个圆形框	
	LCD_DrawCircle(350,	50,	50, 1, LCD565_YELLOW );			//画一个圆形框	
	LCD_DrawCircle(50,	190, 50, 1, LCD565_YELLOW );			//画一个圆形框	
	LCD_DrawCircle(350,	190, 50, 1, LCD565_YELLOW );			//画一个圆形框	
	LCD_DrawCircle(200,	120, 100, 1, LCD565_YELLOW );			//画一个圆形框	
	
	SysTick_DeleymS(500);					//SysTick延时nmS
	LCD_Clean(LCD565_RED);		//清除屏幕函数--
#endif
#if 0		//画点测试
	u16 x=0,y=0;
	for(y=0;y<sLCD.Data.MaxH;y++)
	{
		for(x=0;x<sLCD.Data.MaxV;x++)
		{
			LCD_DrawDot(x,y,LCD565_YELLOW);
//			SysTick_DeleyuS(10);				//SysTick延时nmS
		}
	}
	SysTick_DeleymS(100);				//SysTick延时nmS
	LCD_Clean(LCD565_RED);			//清除屏幕函数--
#endif	
#if 0			//测试波形绘制
	u16 x=0,y=0;
	
	Rand	=	rand();
	x	=	HX;
	y	=	R61509V_H-Rand%200;
	LCD_DrawDot(x,y,LCD565_YELLOW);
//	R61509V_DrawLine(x,HY,x,y,R61509V_YELLOW);
	HY	=	y;
	if(HX++>=R61509V_V)
	{
		HX	=	0;
		LCD_Clean(LCD565_RED);			//清除屏幕函数--
//		SysTick_DeleymS(500);				//SysTick延时nmS
	}
//	SysTick_DeleymS(100);					//SysTick延时nmS
#endif
#if 0		//刷屏测试
	u32 LCDTime=500;
////	u8 i	=	0;
//	for(Color	=	0;Color<=0xFFFF;)
//	{
//		LCD_Clean(Color);			//清除屏幕函数--
//		SysTick_DeleymS(LCDTime);				//SysTick延时nmS
//		Color+=100;		
//	}
	
	
	LCD_Clean(LCD565_WHITE);		//清除屏幕函数--
	SysTick_DeleymS(LCDTime);				//SysTick延时nmS

	LCD_Clean(LCD565_BLACK);		//清除屏幕函数--
	SysTick_DeleymS(LCDTime);				//SysTick延时nmS
	
	LCD_Clean(LCD565_BLUE);		//清除屏幕函数--
	SysTick_DeleymS(LCDTime);				//SysTick延时nmS
	
	LCD_Clean(LCD565_BRED);			//清除屏幕函数--
	SysTick_DeleymS(LCDTime);				//SysTick延时nmS
	
	LCD_Clean(LCD565_GRED);			//清除屏幕函数--
	SysTick_DeleymS(LCDTime);				//SysTick延时nmS
	
	LCD_Clean(LCD565_GBLUE);			//清除屏幕函数--
	SysTick_DeleymS(LCDTime);				//SysTick延时nmS
	
	LCD_Clean(LCD565_RED);			//清除屏幕函数--
	SysTick_DeleymS(LCDTime);				//SysTick延时nmS
	
	LCD_Clean(LCD565_MAGENTA);			//清除屏幕函数--
	SysTick_DeleymS(LCDTime);				//SysTick延时nmS
	
	LCD_Clean(LCD565_GREEN);			//清除屏幕函数--
	SysTick_DeleymS(LCDTime);				//SysTick延时nmS
	
	LCD_Clean(LCD565_CYAN);			//清除屏幕函数--
	SysTick_DeleymS(LCDTime);				//SysTick延时nmS
	
	LCD_Clean(LCD565_YELLOW);			//清除屏幕函数--
	SysTick_DeleymS(LCDTime);				//SysTick延时nmS
#endif
#if 0	//画直线
	HX	=	0;HY	=	10;
//	for(HY	=	10;HY<=10;HY++)
//	{
//		for(HX	=	10;HX<=400;)
//		{
//			LCD_DrawDot(HX,HY,R61509V_YELLOW);
//			HX	+=	1;
//			SysTick_DeleymS(10);				//SysTick延时nmS
//		}
//	}
//	R61509V_SetWindowAddress(0,10,100,10);//设置窗地址
//	R61509V_SetWindowAddress(0,10,300,10);//设置窗地址
//	for(HX=0;HX<=400;HX++)
//	{
//		R61509V_WriteData16(R61509V_YELLOW); 	//笔画颜色	
//	}
	R61509V_DrawLine(0,100,500,100,R61509V_YELLOW);
	R61509V_DrawLine(100,0,100,230,R61509V_YELLOW);
	R61509V_DrawLine(350,0,350,230,R61509V_YELLOW);
	SysTick_DeleymS(1000);				//SysTick延时nmS
	R61509V_Clean(R61509V_RED);			//清除屏幕函数--蓝白
//	//清除直线
//	HX	=	10;HY	=	10;
//	for(HX	=	10;HX<230;HX++)
//	{
//		for(HY	=	10;HY<=380;)
//		{
//			LCD_DrawDot(HX,HY,R61509V_RED);
//			HY	+=	5;
//			SysTick_DeleymS(100);				//SysTick延时nmS
//		}
//	}

//	LCD_Clean(R61509V_RED);			//清除屏幕函数--蓝白
#endif
#if 0	//LCD驱动调试.----画线
	LCD_Clean(LCD565_RED);			//清除屏幕函数--蓝白
	HX	=	10;HY	=	10;
	for(HY	=	10;HY<=230;)
	{
		LCD_DrawLine(10,HY,390,HY,LCD565_YELLOW);
		HY	+=	5;
	}
	SysTick_DeleymS(1000);				//SysTick延时nmS
	HX	=	10;HY	=	10;
	for(HX	=	10;HX<=390;)
	{
		LCD_DrawLine(HX,10,HX,230,LCD565_YELLOW);
		HX	+=	5;
//		SysTick_DeleymS(100);				//SysTick延时nmS
	}
	SysTick_DeleymS(1000);				//SysTick延时nmS
	//清除
	HX	=	10;HY	=	10;
	for(HY	=	10;HY<=230;)
	{
		LCD_DrawLine(10,HY,390,HY,LCD565_RED);
		HY	+=	5;
	}
	SysTick_DeleymS(1000);				//SysTick延时nmS
	HX	=	10;HY	=	10;
	for(HX	=	10;HX<=390;)
	{
		LCD_DrawLine(HX,10,HX,230,LCD565_RED);
		HX	+=	5;
	}
	SysTick_DeleymS(1000);				//SysTick延时nmS
	LCD_Clean(LCD565_RED);			//清除屏幕函数--蓝白
#endif
#if 0
//	LCD_Clean(LCD565_RED);			//清除屏幕函数--蓝白
	
//	LCD_Printf(0		,0,32	,"萘");									//待发药槽位，后边的省略号就是可变参数
	
//	LCD_Printf(0		,0,16	,"显示测试……");									//待发药槽位，后边的省略号就是可变参数
//	LCD_Printf(0		,16,16	,"显示测试……");									//待发药槽位，后边的省略号就是可变参数
//	LCD_Printf(0		,32,32	,"显示测试……");									//待发药槽位，后边的省略号就是可变参数
//	LCD_Printf(0		,64,32	,"显示测试……");									//待发药槽位，后边的省略号就是可变参数
//	LCD_Printf(0		,96,32	,"显示测试……");									//待发药槽位，后边的省略号就是可变参数
	LCD_Printf(0		,128,32	,"测试显示……");									//待发药槽位，后边的省略号就是可变参数
	LCD_Printf(0		,160,32	,"怲试显示……");									//待发药槽位，后边的省略号就是可变参数

//	SysTick_DeleymS(1000);				//SysTick延时nmS
//	LCD_Clean(LCD565_RED);			//清除屏幕函数--蓝白	
#endif
	if(DspFlg)
	{
		DspFlg	=	0;
//		LCD_Printf(0		,32,32	,"%0.8d",CS5530_ADC_CMP);				//待发药槽位，后边的省略号就是可变参数
//		LCD_Printf(0		,64	,32	,"%0.8d",CS5530_ADC_Value);				//待发药槽位，后边的省略号就是可变参数
	}
	if(DspFlg1)
	{
		DspFlg1	=	0;
//		LCD_Printf(0		,96,32	,"%0.8d",We1);				//待发药槽位，后边的省略号就是可变参数
	}
}
/*******************************************************************************
* 函数名			:	RS485_Server
* 功能描述		:	数据收发--获取AD值命令:FA F5 ID 01 ,上报数据格式：FB F6 ID B1 B2 B3 高位在前
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void RS485_Server(void)			//通讯管理---负责信息的接收与发送
{
	
	RxNum=RS485_ReadBufferIDLE(&RS485,RevBuffe);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数，然后重新将接收缓冲区地址指向RxdBuffer
	if(RxNum)
	{
		Rxdata	=	RevBuffe[0];
		SysTick_DeleymS(10);				//SysTick延时nmS
		RS485_DMASend	(&RS485,&Rxdata,1);	//RS485-DMA发送程序
		LCD_Printf(0		,0	,32,LCD565_RED,"收到数据%d",Rxdata);				//待发药槽位，后边的省略号就是可变参数
		return;
	}
//	else
//	{
//		if(SYSTIME++>=500)
//		{
//			SYSTIME	=	0;
//			RS485_DMASend	(&RS485,(u32*)&Rxdata,1);	//RS485-DMA发送程序
//			SysTick_DeleymS(5);				//SysTick延时nmS
//		}
//	}
	return;
#if Master			//主机
	RxNum=RS485_ReadBufferIDLE(&RS485,(u32*)RevBuffe,(u32*)RxdBuffe);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数，然后重新将接收缓冲区地址指向RxdBuffer
	if(		RxNum								\
			&&	(RevBuffe[0]	==	0xFB)			\
			&&	(RevBuffe[1]	==	0xF6)			\
		)
	{		
		Rs485_Time	=	0;
		Rev_ADC	=	0;
		Rev_ID	=	RevBuffe[2];
		
		Rev_ADC	+=	RevBuffe[3];
		Rev_ADC	<<=	8;
		Rev_ADC	+=	RevBuffe[4];
		Rev_ADC<<=8;
		Rev_ADC	+=	RevBuffe[5];
		
		LCD_Printf(0		,128	,16	,"接收到通道%0.2d称重值：%0.8d",Rev_ID,Rev_ADC);				//待发药槽位，后边的省略号就是可变参数
		
	}
	if(Rs485_Time	++>5)
	{
		Rs485_Time	=	0;
		TxdBuffe[0]	=	0xFA;
		TxdBuffe[1]	=	0xF5;
		TxdBuffe[2]	=	SlaveID;
		TxdBuffe[3]	=	0x01;
		RS485_DMASend	(&RS485,(u32*)TxdBuffe,4);	//RS485-DMA发送程序
		if(SlaveID++>=250)
			SlaveID	=	1;
	}
#else
	
	if(		RxNum												\
			&&(RevBuffe[0]	==	0xFA)			\
			&&(RevBuffe[1]	==	0xF5)			\
			&&(RevBuffe[2]	==	SwitchID)	\
			&&(RevBuffe[3]	==	0x01)			\
		)
	{
		RxFlg	=	1;
		Rs485_Time	=	0;
	}
	if(RxFlg	==	1)
	{
		if(Rs485_Time++>=2)	//延时10mS
		{
			//CS5530_ADC_Value
			RxFlg	=	0;
			Rs485_Time	=	0;
			
			TxdBuffe[0]	=	0xFB;
			TxdBuffe[1]	=	0xF6;
			TxdBuffe[2]	=	SwitchID;
			TxdBuffe[3]	=	(CS5530_ADC_Value>>16)&0xFF;
			TxdBuffe[4]	=	(CS5530_ADC_Value>>8)&0xFF;
			TxdBuffe[5]	=	(CS5530_ADC_Value>>0)&0xFF;
			RS485_DMASend	(&RS485,TxdBuffe,6);	//RS485-DMA发送程序
		}
	}
#endif
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void SwitchID_Server(void)	//拔码开关处理--动态更新拨码地址
{
	u8 ID_Temp	=	0;
	ID_Temp	=	SWITCHID_Read(&SWITCHID);		//
	if(SwitchID	!=	ID_Temp)
	{
		SwitchID	=	ID_Temp;
		LCD_Printf(0		,0,32	,LCD565_RED,"地址：%0.2d",SwitchID);				//待发药槽位，后边的省略号就是可变参数
	}
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void SwitchID_Configuration(void)
{
	SWITCHID.NumOfSW	=	6;
	
	SWITCHID.SW1_PORT	=	GPIOC;
	SWITCHID.SW1_Pin	=	GPIO_Pin_6;
	
	SWITCHID.SW2_PORT	=	GPIOC;
	SWITCHID.SW2_Pin	=	GPIO_Pin_7;
	
	SWITCHID.SW3_PORT	=	GPIOC;
	SWITCHID.SW3_Pin	=	GPIO_Pin_8;
	
	SWITCHID.SW4_PORT	=	GPIOC;
	SWITCHID.SW4_Pin	=	GPIO_Pin_9;
	
	SWITCHID.SW5_PORT	=	GPIOC;
	SWITCHID.SW5_Pin	=	GPIO_Pin_12;
	
	SWITCHID.SW6_PORT	=	GPIOC;
	SWITCHID.SW6_Pin	=	GPIO_Pin_13;
	
	SwitchIdInitialize(&SWITCHID);							//
	
	SwitchID	=	SWITCHID_Read(&SWITCHID);		//
}


/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void LCD_Configuration(void)
{
	LCDPortDef		*Port;
	SPIDef				*SPIx;
	
	Port	=	&(sLCD.Port);
	SPIx		=	&(sLCD.GT32L32.SPI);
	
	Port->sCS_PORT		=	GPIOB;
	Port->sCS_Pin			=	GPIO_Pin_7;
	
	Port->sDC_PORT		=	GPIOB;
	Port->sDC_Pin			=	GPIO_Pin_6;
	
	Port->sWR_PORT		=	GPIOB;
	Port->sWR_Pin			=	GPIO_Pin_8;
	
	Port->sRD_PORT		=	GPIOB;
	Port->sRD_Pin			=	GPIO_Pin_5;
	
	Port->sREST_PORT	=	GPIOB;
	Port->sREST_Pin		=	GPIO_Pin_9;
	
	Port->sBL_PORT		=	GPIOB;
	Port->sBL_Pin			=	GPIO_Pin_3;
	
	Port->sTE_PORT		=	GPIOB;
	Port->sTE_Pin			=	GPIO_Pin_4;
	
	Port->sDATABUS_PORT	=	GPIOE;
	Port->sDATABUS_Pin	=	GPIO_Pin_All;
	
	sLCD.Data.BColor	=	LCD565_BLACK;
	sLCD.Data.PColor	=	LCD565_WHITE;
	sLCD.Flag.Rotate	=	Draw_Rotate_90D;	//使用旋转角度	
	
	SPIx->Port.SPIx=SPI1;
	
	SPIx->Port.CS_PORT=GPIOA;
	SPIx->Port.CS_Pin=GPIO_Pin_4;
	
	SPIx->Port.CLK_PORT	=	GPIOA;
	SPIx->Port.CLK_Pin=GPIO_Pin_5;
	
	SPIx->Port.MISO_PORT	=	GPIOA;
	SPIx->Port.MISO_Pin=GPIO_Pin_6;
	
	SPIx->Port.MOSI_PORT	=	GPIOA;
	SPIx->Port.MOSI_Pin=GPIO_Pin_7;
	
	SPIx->Port.SPI_BaudRatePrescaler_x=SPI_BaudRatePrescaler_2;
	
	R61509V_Initialize(&sLCD);
	
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE);				//I/O口重映射开启
	PWM_OUTRemap(TIM2,PWM_OUTChannel2,500,50);									//PWM设定-20161127版本--运行指示灯
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void CS5530_Configuration(void)
{
	CS5530.Port.CS_PORT=GPIOD;
	CS5530.Port.CS_Pin=GPIO_Pin_8;
	
	CS5530.Port.SDI_PORT=GPIOB;
	CS5530.Port.SDI_Pin=GPIO_Pin_15;
	
	CS5530.Port.SDO_PORT=GPIOB;
	CS5530.Port.SDO_Pin=GPIO_Pin_14;
	
	CS5530.Port.SCLK_PORT=GPIOB;
	CS5530.Port.SCLK_Pin=GPIO_Pin_13;
	
	CS5530_Initialize(&CS5530);

#ifndef TestChN1		//双通道
	CS5530B.Port.CS_PORT=GPIOB;
	CS5530B.Port.CS_Pin=GPIO_Pin_12;
	
	CS5530B.Port.SDI_PORT=GPIOB;
	CS5530B.Port.SDI_Pin=GPIO_Pin_15;
	
	CS5530B.Port.SDO_PORT=GPIOB;
	CS5530B.Port.SDO_Pin=GPIO_Pin_14;
	
	CS5530B.Port.SCLK_PORT=GPIOB;
	CS5530B.Port.SCLK_Pin=GPIO_Pin_13;
	
	CS5530_Initialize(&CS5530B);
#else
	#ifndef TestFloat		//未启用通道配置为非浮空
		GPIO_Configuration_OPP50(GPIOB,GPIO_Pin_12);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
		GPIO_SetBits(GPIOB,GPIO_Pin_12);
	#endif
#endif
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void RS485_Configuration(void)
{
	RS485.USARTx=USART2;
	RS485.RS485_CTL_PORT=GPIOA;
	RS485.RS485_CTL_Pin=GPIO_Pin_1;
	
	RS485_DMA_ConfigurationNR	(&RS485,19200,Rs485Size);	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
}




/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void PL010V13_DISPLAY(void)
{
//	R61509V_DrawPixelEx( 100, 100,LCD_FORE_COLOR);
//	R61509V_DrawHLine( 10, 100, 200, LCD_FORE_COLOR);
}
void PL010Delay(u32 time)
{	
	while(time--);
}


#endif
