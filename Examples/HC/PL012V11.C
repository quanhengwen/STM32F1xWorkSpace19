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

#ifdef PL012V11				//拆零柜LCD板

#include "PL012V11.H"

//#include "R61509V.h"
#include "ILI9326.h"

#include "CS5530.H"

#include "GT32L32M0180.H"
#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_PWM.H"
#include "STM32_USART.H"


#include "SWITCHID.H"

#include "string.h"				//串和内存操作函数头文件
#include "stm32f10x_dma.h"

LCDDef	sLCD;

u16	DelayTime=0x1000;
u16	LCDTime=0;
u16	DSPTime=0;
u32 testADC=0;
u32 bacADC=0;
u32 bacADC2=0;



u16 SumFed[8]={0};		//总共已发药数量
u16	SumFQ[8]={0};			//总共发药请求数量
u8	NumFW=0;		//待发药槽位
u8	Onlinede=0;		//待发药槽位






//t_Point point;
u8 zimo[720]="R61509V_DrawRectangle(11,11,229,389,0X07FF)";

RS485_TypeDef  RS485;
u8 RS485FLG	=	0;
u32 RS485Time	=	0;
u32 RSRLen	=	0;

u8 TxdBuffe[256]={0};
u8 RxdBuffe[256]={0};
u8 RevBuffe[256]={0};
u16 RxNum=0;
char	Char_Buffer[256]={0xFF};		//记录format内码
//t_LcdCfg **pLcdpara;


SWITCHID_CONF	SWITCHID;
u8 SwitchID=0;	//拔码开关地址







/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void PL012V11_Configuration(void)
{
	SYS_Configuration();					//系统配置---打开系统时钟 STM32_SYS.H
	
	GPIO_DeInitAll();							//将所有的GPIO关闭----V20170605
	
	
	
	
	

	
//	R61509V_Configuration();
	ILI9326_Configuration();
	

	
	RS485_Configuration();
	
	LCD_Printf(0		,0	,32	,0,"序号：");				//待发药槽位，后边的省略号就是可变参数
	
	SwitchID_Configuration();
	
	
	SysTick_Configuration(1000);	//系统嘀嗒时钟配置72MHz,单位为uS
	
//	IWDG_Configuration(1000);			//独立看门狗配置---参数单位ms	
	PWM_OUT(TIM2,PWM_OUTChannel1,1,900);	//PWM设定-20161127版本--指示灯
	PWM_OUT(TIM3,PWM_OUTChannel3,500,200);		//PWM设定-20161127版本--背光
	memset(TxdBuffe,0xA5,128);
}
/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void PL012V11_Server(void)
{	
	IWDG_Feed();								//独立看门狗喂狗
	LCD_Display();
	return;
	
//	RS485_Server();
	if(DelayTime++>=500)
	{
		DelayTime	=	0;
	}
	if(LCDTime++>=1000)
	{
		LCDTime	=	0;
//		R61509V_Clean(R61509V_BRED); 					//清除屏幕函数
		LCD_Display();
	}
//	SwitchID_Server();
	 
//	if(LCDTime==500)
//	{
//		PL010V13_PrintfString(0		,134+50+16	,16	,"发药层控制器通信异常！！！！");				//错误状态
//	}
//	else if(LCDTime==1000)
//	{
//		PL010V13_PrintfString(0		,134+50+16	,16	,"XXXXXXXXXXXXXXXXXXXXXXXXXX");				//错误状态
//	}	

	
#if 0
	if(DelayTime==0)
	{
		
		DelayTime=0;
		CS5530_ADC_Value=CS5530_ReadData(&CS5530);	//读取AD值，如果返回0xFFFFFFFF,则未读取到24位AD值
//		R61509V_Clean(R61509V_BLACK); 					//清除屏幕函数
//		R61509V_ShowEn(0,0,CS5530_ADC_Value);
//		PL010V13_PrintfString(0		,0	,16	,"%2d",SwitchID);				//待发药槽位，后边的省略号就是可变参数
		PL010V13_PrintfString(120		,100	,32	,"%8d",CS5530_ADC_Value);				//待发药槽位，后边的省略号就是可变参数
	}
#endif
	

}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void RS485_Server(void)
{
#if 0
	RxNum=RS485_ReadBufferIDLE(&RS485,RevBuffe);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数，然后重新将接收缓冲区地址指向RxdBuffer
	if(RxNum	==	100)
	{
		RS485Time	=	0;
		PWM_OUT(TIM3,PWM_OUTChannel3,500,800);		//PWM设定-20161127版本--背光
	}
	if(RS485Time++>=1000)
	{
		RS485Time	=	0;
		RS485_DMASend(&RS485,(u32*)TxdBuffe,100);	//RS485-DMA发送程序
		PWM_OUT(TIM3,PWM_OUTChannel3,500,1);		//PWM设定-20161127版本--背光
	}
#else
	RxNum=RS485_ReadBufferIDLE(&RS485,RevBuffe);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数，然后重新将接收缓冲区地址指向RxdBuffer
	if(RxNum)
	{
		RS485Time	=	0;
		RS485FLG	=	1;
		RSRLen	=	RxNum;
		RS485_DMASend(&RS485,RevBuffe,100);	//RS485-DMA发送程序
		PWM_OUT(TIM3,PWM_OUTChannel3,500,800);		//PWM设定-20161127版本--背光
	}
	if(RS485FLG	==	1)
	{
		if(RS485Time++>=500)
		{
			RS485Time	=	0;
			RS485_DMASend(&RS485,RevBuffe,RSRLen);	//RS485-DMA发送程序
			RSRLen	=	0;
			RS485FLG	=	0;
			PWM_OUT(TIM3,PWM_OUTChannel3,500,800);		//PWM设定-20161127版本--背光
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
void SwitchID_Configuration(void)
{
	SWITCHID.NumOfSW	=	8;
	
	SWITCHID.SW1_PORT	=	GPIOD;
	SWITCHID.SW1_Pin	=	GPIO_Pin_2;
	
	SWITCHID.SW2_PORT	=	GPIOA;
	SWITCHID.SW2_Pin	=	GPIO_Pin_15;
	
	SWITCHID.SW3_PORT	=	GPIOA;
	SWITCHID.SW3_Pin	=	GPIO_Pin_12;
	
	SWITCHID.SW4_PORT	=	GPIOA;
	SWITCHID.SW4_Pin	=	GPIO_Pin_11;
	
	SWITCHID.SW5_PORT	=	GPIOA;
	SWITCHID.SW5_Pin	=	GPIO_Pin_8;
	
	SWITCHID.SW6_PORT	=	GPIOB;
	SWITCHID.SW6_Pin	=	GPIO_Pin_15;
	
	SWITCHID.SW7_PORT	=	GPIOB;
	SWITCHID.SW7_Pin	=	GPIO_Pin_14;
	
	SWITCHID.SW8_PORT	=	GPIOB;
	SWITCHID.SW8_Pin	=	GPIO_Pin_13;
	
	SwitchIdInitialize(&SWITCHID);							//
	
//	SwitchID	=	SWITCHID_Read(&SWITCHID);		//
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void SwitchID_Server(void)
{
	u8 temp=0;	//拔码开关临时地址
	temp	=	SWITCHID_Read(&SWITCHID);		//读取地址
	if(SwitchID	!=	temp)
	{
		u8 AddrH	=	0,AddrL	=	0;
		u32	w=0,n=0,d=0;
//	w	=	1;n	=	0;
//	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//待发药槽位，后边的省略号就是可变参数
//	w	+=	1;n	+=	8;
//	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//待发药槽位，后边的省略号就是可变参数
//	w	+=	1;n	+=	8;
//	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//待发药槽位，后边的省略号就是可变参数
//	w	+=	1;n	+=	8;
//	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//待发药槽位，后边的省略号就是可变参数
//	w	+=	1;n	+=	8;
//	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//待发药槽位，后边的省略号就是可变参数
//	w	+=	1;n	+=	8;
//	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//待发药槽位，后边的省略号就是可变参数
//	w	+=	1;n	+=	8;
//	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//待发药槽位，后边的省略号就是可变参数
//	w	+=	1;n	+=	8;
//	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//待发药槽位，后边的省略号就是可变参数
		
		w	=	12345678;
		LCD_Printf(0		,0	,32	,0,"序号：");				//待发药槽位，后边的省略号就是可变参数
		LCD_Printf(0		,32	,32	,0,"拔码：");				//待发药槽位，后边的省略号就是可变参数
		LCD_Printf(96	,0	,32	,0,"%8d",w);				//待发药槽位，后边的省略号就是可变参数
		w	=	0;
		SwitchID	=	temp;
		for(n=0;n<8;n++)
		{
			if((temp&0x01)	==0x01)
			{
				d	=	1;
			}
			else
			{
				d	=	0;
			}
			temp>>=1;
			LCD_Printf(n*16+96		,32	,32	,0,"%d",d);				//待发药槽位，后边的省略号就是可变参数
		}
		AddrH	=	(SwitchID>>4)&0x0F;
		AddrL	=	(SwitchID>>0)&0x0F;
		LCD_Printf(0		,64	,32	,0,"层%0.2d，位%0.2d",AddrH,AddrL);				//待发药槽位，后边的省略号就是可变参数
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
void R61509V_Configuration(void)
{

	//=======================LCD端口
	LCDPortDef	*LcdPort	=	&sLCD.Port;
	
	LcdPort->sCS_PORT	=	GPIOB;
	LcdPort->sCS_Pin	=	GPIO_Pin_7;

	LcdPort->sDC_PORT	=	GPIOB;
	LcdPort->sDC_Pin	=	GPIO_Pin_6;
	
	LcdPort->sWR_PORT	=	GPIOB;
	LcdPort->sWR_Pin	=	GPIO_Pin_8;
	
	LcdPort->sRD_PORT	=	GPIOB;
	LcdPort->sRD_Pin	=	GPIO_Pin_5;
	
	LcdPort->sREST_PORT	=	GPIOB;
	LcdPort->sREST_Pin	=	GPIO_Pin_9;
	
	LcdPort->sBL_PORT		=	GPIOB;
	LcdPort->sBL_Pin		=	GPIO_Pin_0;
	
	LcdPort->sTE_PORT		=	GPIOB;
	LcdPort->sTE_Pin		=	GPIO_Pin_4;
	
	LcdPort->sDATABUS_PORT	=	GPIOC;
	LcdPort->sDATABUS_Pin		=	GPIO_Pin_All;
	
	sLCD.Data.BColor	=	LCD565_BLACK;
	sLCD.Data.PColor	=	LCD565_WHITE;
	sLCD.Flag.Rotate	=	Draw_Rotate_90D;
	
	//=======================字库端口
	sLCD.GT32L32.SPI.Port.SPIx	=	SPI1;
	sLCD.GT32L32.SPI.Port.CS_PORT	=	GPIOA;
	sLCD.GT32L32.SPI.Port.CS_Pin		=	GPIO_Pin_4;
	sLCD.GT32L32.SPI.Port.SPI_BaudRatePrescaler_x=SPI_BaudRatePrescaler_2;

	R61509V_Initialize(&sLCD);
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
void ILI9326_Configuration(void)
{

	//=======================LCD端口
	LCDPortDef	*LcdPort	=	&sLCD.Port;
	
	LcdPort->sCS_PORT		=	GPIOB;
	LcdPort->sCS_Pin		=	GPIO_Pin_7;

	LcdPort->sDC_PORT		=	GPIOB;
	LcdPort->sDC_Pin		=	GPIO_Pin_6;
	
	LcdPort->sWR_PORT		=	GPIOB;
	LcdPort->sWR_Pin		=	GPIO_Pin_8;
	
	LcdPort->sRD_PORT		=	GPIOB;
	LcdPort->sRD_Pin		=	GPIO_Pin_5;
	
	LcdPort->sREST_PORT	=	GPIOB;
	LcdPort->sREST_Pin	=	GPIO_Pin_9;
	
	LcdPort->sBL_PORT		=	GPIOB;
	LcdPort->sBL_Pin		=	GPIO_Pin_0;
	
	LcdPort->sTE_PORT		=	GPIOB;
	LcdPort->sTE_Pin		=	GPIO_Pin_4;
	
	LcdPort->sDATABUS_PORT	=	GPIOC;
	LcdPort->sDATABUS_Pin		=	GPIO_Pin_All;
	
	sLCD.Data.BColor	=	LCD565_BLACK;
	sLCD.Data.PColor	=	LCD565_RED;
	sLCD.Flag.Rotate	=	Draw_Rotate_90D;
	
	//=======================字库端口
	sLCD.GT32L32.SPI.Port.SPIx			=	SPI1;
	sLCD.GT32L32.SPI.Port.CS_PORT		=	GPIOA;
	sLCD.GT32L32.SPI.Port.CS_Pin		=	GPIO_Pin_4;
	sLCD.GT32L32.SPI.Port.SPI_BaudRatePrescaler_x=SPI_BaudRatePrescaler_64;

	ILI9326_Initialize(&sLCD);
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
	
	RS485_DMA_ConfigurationNR	(&RS485,19200,256);	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
}
/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void LCD_PowerUp(void)
{	
//	u16 i=0,j=0;
	
	
	
	
	
//	R61509V_DrawDot(50,50,0X5458);
//	R61509V_DrawDot_big(50,50,0X5458);
//	R61509V_DrawLine(0,0,240,400,0X5458);
//	R61509V_DrawLine(0,400,240,0,0X5458);
	
//	R61509V_DrawCircle(120,100, 50,0,0X5458);				//画一个圆形框
	
//	R61509V_DrawCircle(120,200, 120,0,0X01CF);				//画一个圆形框
	
//	R61509V_DrawCircle(120,400, 80,0,0X5458);				//画一个圆形框
//	
//	R61509V_DrawCircle(120,0, 80,0,0X5458);					//画一个圆形框
//	
//	R61509V_DrawCircle(120,200, 80,1,0X5458);				//画一个圆形框
//	
//	R61509V_DrawCircle(200,120, 50,0,0xF800);				//画一个圆形框
//	
//	R61509V_DrawCircle(120,200, 30,1,0X07FF);				//画一个圆形框
	
//	R61509V_DrawRectangle(3,3,237,397,0x07E0);			//画一个矩形框
//	R61509V_DrawRectangle(4,4,236,396,0x07E0);			//画一个矩形框
//	R61509V_DrawRectangle(5,5,235,395,0x07E0);			//画一个矩形框
//	R61509V_DrawRectangle(6,6,234,394,0x07E0);			//画一个矩形框
//	
//	R61509V_DrawRectangle(7,7,233,393,0X07FF);			//画一个矩形框
//	R61509V_DrawRectangle(8,8,232,392,0X07FF);			//画一个矩形框
//	R61509V_DrawRectangle(9,9,231,391,0X07FF);			//画一个矩形框
//	R61509V_DrawRectangle(10,10,230,390,0X07FF);		//画一个矩形框
//	R61509V_DrawRectangle(11,11,229,389,0X07FF);		//画一个矩形框
//	R61509V_DrawRectangle(12,12,228,388,0X07FF);			//画一个矩形框
//	R61509V_DrawLine(12,12,228,388,0X5458);						//AB 两个坐标画一条直线
//	R61509V_DrawLine(12,388,228,12,0X5458);						//AB 两个坐标画一条直线
	
//	R61509V_DrawLine(0,10,400,10,0X5458);						//AB 两个坐标画一条直线
//	
//	R61509V_DrawLine(0,20,400,20,0X5458);						//AB 两个坐标画一条直线
	
//	R61509V_DrawLine(0,100,160,100,0X5458);						//AB 两个坐标画一条直线
//	
//	R61509V_DrawLine(80,0,80,400,0X5458);						//AB 两个坐标画一条直线
	
//	R61509V_DrawRectangle(10,10,390,230,0X07FF);		//画一个矩形框
	
//	R61509V_DrawCircle(200,120, 10,1,0X5458);					//画一个圆形框
//	
//	R61509V_DrawCircle(200,120, 80,0,0X5458);					//画一个圆形框
	
//	R61509V_DrawLine(240,240,20,240,0X5458);						//AB 两个坐标画一条直线
	
//	R61509V_ShowChar(1,1,32,100,zimo);								//高通字库测试程序
//	
//	R61509V_ShowCharT(50,50,15,0);
//	R61509V_ShowEn(200,120,12);
	
//	PL010V13_PrintfString(0		,16	,16	,"待发药槽位：%3d",RevBuffe[0]);				//后边的省略号就是可变参数
//	PL010V13_PrintfString(0		,32	,16	,"待发药数量：%3d",RevBuffe[1]);				//后边的省略号就是可变参数

//	PL010V13_PrintfString(0		,0	,32	,"槽位-%2d数量-%2d",RevBuffe[0],RevBuffe[1]);				//后边的省略号就是可变参数
	
//	PL010V13_PrintfString(0		,0	,32	,"待发药数量：%2d",RevBuffe[1]);				//后边的省略号就是可变参数
//	
//	
//	PL010V13_PrintfString(0		,100	,32	,"待发药数量：%2d",RevBuffe[1]);				//后边的省略号就是可变参数
//	PL010V13_PrintfString(0		,100	,32	,"待发药数量：%2d",RevBuffe[1]);				//后边的省略号就是可变参数
//	
	#if 0
	PL010Delay(0xFFFF);
	PL010V13_PrintfString(1		,0	,16	,"待发药槽位：%2d",RevBuffe[0]);				//后边的省略号就是可变参数
	PL010V13_PrintfString(1		,0	,16	,"待发药槽位：%2d",RevBuffe[0]);				//后边的省略号就是可变参数
	PL010Delay(0xFFFF);
	PL010V13_PrintfString(1		,20	,16	,"待发药数量：%2d",RevBuffe[1]);				//后边的省略号就是可变参数
	PL010Delay(0xFFFF);
	PL010V13_PrintfString(1		,40	,16	,"已发药数量：%2d",RevBuffe[1]);				//后边的省略号就是可变参数
	PL010Delay(0xFFFF);
	
//	PL010V13_PrintfString(0		,60	,16	,"总共请求数量：%4d",SumFQ);				//总共发药请求数量
//	PL010V13_PrintfString(0		,80	,16	,"总共发药数量：%4d",SumFed);				//总共已发药数量
	PL010V13_PrintfString(1		,100	,16	,"错误状态：");				//错误状态
	PL010Delay(0xFFFF);
	
	PL010V13_PrintfString(1		,160	,16	,"提示：长按3秒读取发药头");				//错误状态
	PL010Delay(0xFFFF);
	

	//平行线
	R61509V_DrawLine(221,0,221,400,R61509V_WHITE);						//AB 两个坐标画一条直线
	R61509V_DrawLine(201,0,201,400,R61509V_WHITE);						//AB 两个坐标画一条直线
	R61509V_DrawLine(181,0,181,400,R61509V_WHITE);						//AB 两个坐标画一条直线
	R61509V_DrawLine(161,0,161,400,R61509V_WHITE);						//AB 两个坐标画一条直线
	//垂直线--中
	R61509V_DrawLine(161,200,221,200,R61509V_WHITE);					//AB 两个坐标画一条直线
	//垂直线--右
	R61509V_DrawLine(161,250-1,221,250-1,R61509V_WHITE);			//AB 两个坐标画一条直线
	R61509V_DrawLine(161,300-1,221,300-1,R61509V_WHITE);			//AB 两个坐标画一条直线
	R61509V_DrawLine(161,350-1,221,350-1,R61509V_WHITE);			//AB 两个坐标画一条直线
	R61509V_DrawLine(161,400-1,221,400-1,R61509V_WHITE);			//AB 两个坐标画一条直线
	
	//垂直线--左
	R61509V_DrawLine(161,1,221,1,					R61509V_WHITE);							//AB 两个坐标画一条直线
	R61509V_DrawLine(161,51,221,51,				R61509V_WHITE);						//AB 两个坐标画一条直线
	R61509V_DrawLine(161,101,221,101,			R61509V_WHITE);					//AB 两个坐标画一条直线
	R61509V_DrawLine(161,151,221,151,			R61509V_WHITE);					//AB 两个坐标画一条直线

	//填充
	R61509V_Fill(2,180,50,200-2,R61509V_RED);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
	
	
	
	
//	R61509V_DrawLine(10,200,10,280,R61509V_WHITE);						//AB 两个坐标画一条直线
	
	#else
//	R61509V_Clean(R61509V_WHITE);			//清除屏幕函数------
//	PL010Delay(0x8FFFFF);
	LCD_Clean(LCD565_BLUE);			//清除屏幕函数------
	PL010Delay(0x8FFFFF);
	LCD_Clean(LCD565_GRED);			//清除屏幕函数------
	PL010Delay(0x8FFFFF);
	LCD_Clean(LCD565_BLACK);			//清除屏幕函数------
	PL010Delay(0x8FFFFF);
	LCD_Clean(LCD565_WHITE);			//清除屏幕函数------

//	PL010Delay(0xFFFF);
//	PL010V13_PrintfString(0	,0,16	,"TEST");				//错误状态
	#endif
//	R61509V_Clean(R61509V_BLACK);			//清除屏幕函数------
	
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
void LCD_Display(void)
{
	LCD_Printf(0		,0	,32	,0,"显示测试!!@#$%^&*");				//错误状态
	LCD_Printf(0		,32	,32	,0,"显示测试!!@#$%^&*");				//错误状态
	LCD_Printf(0		,64	,32	,0,"显示测试!!@#$%^&*");				//错误状态
	LCD_Printf(0		,96	,32	,0,"显示测试!!@#$%^&*");				//错误状态
	LCD_Printf(0		,128,32	,0,"显示测试!!@#$%^&*");				//错误状态
	
#if 0
	DSPTime++;
	if(DSPTime>500)
		DSPTime=0;
	if(DSPTime%500==0)
	{
		LCD_Clean(LCD565_GRED);			//清除屏幕函数------
//		LCD_Printf(0		,120	,32	,0,"显示测试!!！！！！@#$%^&*");				//错误状态
	}
	if(DSPTime%500==100)
	{
		LCD_Clean(LCD565_BLUE);			//清除屏幕函数------
//		LCD_Printf(0		,120	,32	,0,"显示测试!!！！！！@#$%^&*");				//错误状态
	}
	if(DSPTime%500==200)
	{
		LCD_Clean(LCD565_BLACK);			//清除屏幕函数------
//		LCD_Printf(0		,120	,32	,0,"显示测试!!！！！！@#$%^&*");				//错误状态
	}
	if(DSPTime%500==300)
	{
		LCD_Clean(LCD565_LGRAY);			//清除屏幕函数------
//		LCD_Printf(0		,120	,32	,0,"显示测试!!！！！！@#$%^&*");				//错误状态	
	}
	if(DSPTime%500==400)
	{
		LCD_Clean(LCD565_WHITE);			//清除屏幕函数------
//		LCD_Printf(0		,120	,32	,0,"显示测试!!！！！！@#$%^&*");				//错误状态
	}

#endif
	
#if 0		
	DSPTime++;
	if(DSPTime>500)
	{
		DSPTime=0;
		LCD_WXS();		//位显示
		LCD_DDSP();		//显示总共请求数量和已发数量
	}	
	LCD_WS();		//位闪烁
	
	if(LCDTime>=1000)
	{			
		LCDTime=0;		
	}	
	
	RxNum=RS485_ReadBufferIDLE(&RS485,(u32*)RevBuffe,(u32*)RxdBuffe);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数，然后重新将接收缓冲区地址指向RxdBuffer
	if(RxNum==4&&RevBuffe[0]==0x00&&RevBuffe[1]==0xFF)		//RS485接收到数据
	{
		NumFW=RevBuffe[2];
		LCD_Printf(96		,0	,16	,0,"%2d",RevBuffe[2]);				//待发药槽位，后边的省略号就是可变参数
		LCD_Printf(96		,20	,16	,0,"%2d",RevBuffe[3]);				//待发药数量，后边的省略号就是可变参数
//		PL010V13_PrintfString(192		,68	,16	,"%2d",RevBuffe[1]);				//已发药数量，后边的省略号就是可变参数
	}
	else if(RxNum==3&&RevBuffe[0]==0x02)		//RS485接收到数据
	{
		SumFQ[RevBuffe[1]-1]+=RevBuffe[2];
		NumFW=RevBuffe[1];
		LCD_Printf(96		,0	,16	,0,"%2d",RevBuffe[1]);				//待发药槽位，后边的省略号就是可变参数
		LCD_Printf(96		,20	,16	,0,"%2d",RevBuffe[2]);				//待发药数量，后边的省略号就是可变参数
		LCD_Printf(96		,40	,16	,0,"%2d",0);									//已发药数量，后边的省略号就是可变参数
		
//		PL010V13_PrintfString(0		,60	,16	,0,"总共请求数量：%4d",SumFQ);				//总共发药请求数量
		LCD_Printf(0		,120	,16	,0,"正在发药！！！！！！！！！！");				//错误状态
	}
	else if(RxNum==4&&RevBuffe[0]==0x82)		//RS485接收到数据
	{
		SumFed[RevBuffe[1]-1]+=RevBuffe[2];
		LCD_Printf(96		,0	,16	,0,"%2d",RevBuffe[1]);				//待发药槽位，后边的省略号就是可变参数
		LCD_Printf(96		,20	,16	,0,"%2d",0);									//待发药数量，后边的省略号就是可变参数
		LCD_Printf(96		,40	,16	,0,"%2d",RevBuffe[2]);				//已发药数量，后边的省略号就是可变参数		
		
//		PL010V13_PrintfString(112		,80	,16	,"%4d",SumFed);					//总共已发药数量
		
	}
	else if(RxNum==6&&RevBuffe[0]==0x81)	//槽位信息0x01--获取,0x81--上报
	{
		Onlinede=RevBuffe[4];
	}
	else if(RxNum==1&&RevBuffe[0]==0x01)	//槽位信息0x01--获取,0x81--上报
	{
		LCD_Printf(0		,120	,16		,0,"获取槽位信息！！！");				//错误状态
		NumFW=0;
		Onlinede=0;
		memset(SumFed,0x00,8);
		memset(SumFQ,0x00,8);

		LCD_Printf(96		,0	,16	,0,"%2d",0);				//待发药槽位，后边的省略号就是可变参数
		LCD_Printf(96		,20	,16	,0,"%2d",0);				//待发药数量，后边的省略号就是可变参数
	}
	
	for(DelayTime=0;DelayTime<1000;DelayTime++)
	{
		
	}
	if(RxNum&&(RevBuffe[0]==0x82))		//错误状态显示
	{
		switch(RevBuffe[3])
		{
			case 0x00:	LCD_Printf(0		,120	,16	,0,"！！！！！！！！！！！！！！");				//错误状态
				break;
			case 0x80:	LCD_Printf(0		,120	,16	,0,"药品被卡住！！！！！！！！！");				//错误状态
				break;
			case 0x81:	LCD_Printf(0		,120	,16	,0,"缺药！！！！！！！！！！！！");				//错误状态
				break;
			case 0x82:	LCD_Printf(0		,120	,16	,0,"等待命令结果超时！！！！！！");				//错误状态
				break;
			case 0xC0:	LCD_Printf(0		,120	,16	,0,"单元柜控制器通信异常！！！！");				//错误状态
				break;
			case 0xC1:	LCD_Printf(0		,120	,16	,0,"发药层控制器通信异常！！！！");				//错误状态
				break;
		}
//		RevBuffe[0]=0;
//		RevBuffe[1]=0;
//		RevBuffe[2]=0;
//		RevBuffe[3]=0;
	}
#endif
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
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明 
* 输入			: void
* 返回值			: void
*******************************************************************************/
void LCD_WS(void)		//位闪烁
{
	if(NumFW)		//槽位闪烁	DSPTime
	{
		if(NumFW==1&&(Onlinede>>0&0x01))
		{
			if(DSPTime==500)
			{
				LCD_Fill(2,180,50,200-2,LCD565_GREEN);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				LCD_Fill(2,180,50,200-2,LCD565_MAGENTA);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
			}
		}
		else if(NumFW==2&&(Onlinede>>1&0x01))
		{
			if(DSPTime==500)
			{
				LCD_Fill(52,180,100,200-2,LCD565_GBLUE);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				LCD_Fill(52,180,100,200-2,LCD565_MAGENTA);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
			}			
		}
		else if(NumFW==3&&(Onlinede>>2&0x01))
		{
			if(DSPTime==500)
			{
				LCD_Fill(102,180,150,200-2,LCD565_GREEN);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				LCD_Fill(102,180,150,200-2,LCD565_MAGENTA);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
			}			
		}
		else if(NumFW==4&&(Onlinede>>3&0x01))
		{
			if(DSPTime==500)
			{
				LCD_Fill(152,180,200,200-2,LCD565_GREEN);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				LCD_Fill(152,180,200,200-2,LCD565_MAGENTA);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
			}			
		}
		else if(NumFW==5&&(Onlinede>>4&0x01))
		{
			if(DSPTime==500)
			{
				LCD_Fill(202,180,250,200-2,LCD565_GREEN);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				LCD_Fill(202,180,250,200-2,LCD565_MAGENTA);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
			}			
		}
		else if(NumFW==6&&(Onlinede>>5&0x01))
		{
			if(DSPTime==500)
			{
				LCD_Fill(252,180,300,200-2,LCD565_GREEN);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				LCD_Fill(252,180,300,200-2,LCD565_MAGENTA);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
			}			
		}
		else if(NumFW==7&&(Onlinede>>6&0x01))
		{
			if(DSPTime==500)
			{
				LCD_Fill(302,180,350,200-2,LCD565_GREEN);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				LCD_Fill(302,180,350,200-2,LCD565_MAGENTA);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
			}			
		}
		else if(NumFW==8&&(Onlinede>>7&0x01))
		{
			if(DSPTime==500)
			{
				LCD_Fill(352,180,400-2,200-2,LCD565_GREEN);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				LCD_Fill(352,180,400-2,200-2,LCD565_MAGENTA);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
			}			
		}
	}
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明 
* 输入			: void
* 返回值			: void
*******************************************************************************/
void LCD_WXS(void)		//位显示
{
		if((Onlinede>>0&0x01)==0x01)
		{
			LCD_Fill(2,180,50,200-2,LCD565_GREEN);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
		}
		else
		{
			LCD_Fill(2,180,50,200-2,LCD565_RED);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>1&0x01)==0x01)
		{
			LCD_Fill(52,180,100,200-2,LCD565_GREEN);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
		}
		else
		{
			LCD_Fill(52,180,100,200-2,LCD565_RED);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>2&0x01)==0x01)
		{
			LCD_Fill(102,180,150,200-2,LCD565_GREEN);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
		}
		else
		{
			LCD_Fill(102,180,150,200-2,LCD565_RED);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>3&0x01)==0x01)
		{
			LCD_Fill(152,180,200,200-2,LCD565_GREEN);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
		}
		else
		{
			LCD_Fill(152,180,200,200-2,LCD565_RED);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>4&0x01)==0x01)
		{
			LCD_Fill(202,180,250,200-2,LCD565_GREEN);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
		}
		else
		{
			LCD_Fill(202,180,250,200-2,LCD565_RED);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>5&0x01)==0x01)
		{
			LCD_Fill(252,180,300,200-2,LCD565_GREEN);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
		}
		else
		{
			LCD_Fill(252,180,300,200-2,LCD565_RED);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>6&0x01)==0x01)
		{
			LCD_Fill(302,180,350,200-2,LCD565_GREEN);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
		}
		else
		{
			LCD_Fill(302,180,350,200-2,LCD565_RED);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>7&0x01)==0x01)
		{
			LCD_Fill(352,180,400-2,200-2,LCD565_GREEN);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
		}
		else
		{
			LCD_Fill(352,180,400-2,200-2,LCD565_RED);				//在指定区域内填充指定颜色;区域大小:(xend-xsta)*(yend-ysta)
		}
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明 
* 输入			: void
* 返回值			: void
*******************************************************************************/
void LCD_DDSP(void)		//显示总共请求数量和已发数量
{
//	u8 i=0;
//	u8 w=2;
	//请求数量显示
//	for(i=0;i<8;i++)
//	{
//		PL010V13_PrintfString(w,200,16	,"%5d",SumFQ[i]);				//待发药数量，后边的省略号就是可变参数
//		w+=50;
//	}
	//请求数量显示
	LCD_Printf(2,200,16	,0,"%5d",SumFQ[0]);				//待发药数量，后边的省略号就是可变参数
	LCD_Printf(52,200,16	,0,"%5d",SumFQ[1]);				//待发药数量，后边的省略号就是可变参数
	LCD_Printf(102,200,16	,0,"%5d",SumFQ[2]);				//待发药数量，后边的省略号就是可变参数
	LCD_Printf(152,200,16	,0,"%5d",SumFQ[3]);				//待发药数量，后边的省略号就是可变参数
	LCD_Printf(201,200,16	,0,"%5d",SumFQ[4]);				//待发药数量，后边的省略号就是可变参数
	LCD_Printf(251,200,16	,0,"%5d",SumFQ[5]);				//待发药数量，后边的省略号就是可变参数
	LCD_Printf(301,200,16	,0,"%5d",SumFQ[6]);				//待发药数量，后边的省略号就是可变参数
	LCD_Printf(351,200,16	,0,"%5d",SumFQ[7]);				//待发药数量，后边的省略号就是可变参数

	//已发数量显示	
	LCD_Printf(2,220,16		,0,"%5d",SumFed[0]);				//待发药数量，后边的省略号就是可变参数
	LCD_Printf(52,220,16,0,"%5d",SumFed[1]);				//待发药数量，后边的省略号就是可变参数
	LCD_Printf(102,220,16	,0,"%5d",SumFed[2]);				//待发药数量，后边的省略号就是可变参数
	LCD_Printf(152,220,16	,0,"%5d",SumFed[3]);				//待发药数量，后边的省略号就是可变参数
	LCD_Printf(201,220,16	,0,"%5d",SumFed[4]);				//待发药数量，后边的省略号就是可变参数
	LCD_Printf(251,220,16	,0,"%5d",SumFed[5]);				//待发药数量，后边的省略号就是可变参数
	LCD_Printf(301,220,16	,0,"%5d",SumFed[6]);				//待发药数量，后边的省略号就是可变参数
	LCD_Printf(351,220,16	,0,"%5d",SumFed[7]);				//待发药数量，后边的省略号就是可变参数
}
#endif
