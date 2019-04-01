#ifdef USART_TEST

#include "USART_TEST.H"

#include "STM32_USART.H"
#include "STM32_DMA.H"
#include "STM32_TIM.H"
#include "STM32_WDG.H"
#include "STM32_PWM.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"


#include	"stdio.h"			//用于printf
#include	"string.h"		//用于printf
#include	"stdarg.h"		//用于获取不确定个数的参数
#include	"stdlib.h"		//malloc动态申请内存空间


#define	BufferSize 128		//DMA1缓冲大小

u32	num_temp=0;
u16	tema=0;

u32 DMASTAST=0;
ErrorStatus status = ERROR;

u8	txflg1=0;	//USART1发送标志
u16	tx1_tcont=0;	//USART1发送超时-计时

u8 rxBuffer1[BufferSize]={0};
u8 txBuffer1[BufferSize]={0};
u8 num=0;
int *pr=NULL;

unsigned char t1[4]={0x06,0x01,0x00,0xF8};

unsigned char ts1[4]={0x43,0x01,0x00,0xBB};
unsigned char td1[5]={0x05,0x02,0x00,0x00,0xF8};
unsigned char te1[4]={0x19,0x01,0x10,0xd5};

unsigned char ts2[4]={0x43,0x01,0x30,0x8b};
unsigned char td2[5]={0x05,0x02,0x00,0x00,0xf8};
unsigned char te2[4]={0x19,0x01,0x10,0xd5};

unsigned char ts3[4]={0x43,0x01,0x19,0xa2};
unsigned char td3[5]={0x05,0x02,0x00,0x00,0xf8};
unsigned char te3[4]={0x19,0x01,0x10,0xd5};

static void function(void);
//u8 itf=0;
/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void Usart_test_Configuration(void)
{
	SYS_Configuration();					//系统配置---打开系统时钟 STM32_SYS.H
	
	GPIO_DeInitAll();							//将所有的GPIO关闭----V20170605
	
	PWM_OUT(TIM2,PWM_OUTChannel1,1,900);						//PWM设定-20161127版本	
	
	USART_DMA_ConfigurationNR	(USART1,625000,BufferSize);	//USART_DMA配置--查询方式，不开中断
  
//  IWDG_Configuration(1000);			//独立看门狗配置---参数单位ms	
  
  SysTick_Configuration(100);	//系统嘀嗒时钟配置72MHz,单位为uS
  pr=(int*)0x20024300;
}
/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void Usart_test_Server(void)
{	
	u16 Length	=	0;
	
	IWDG_Feed();								//独立看门狗喂狗
	
	function();	
	return;
	
	Length	=	API_USART_ReadBufferIDLE(USART1,rxBuffer1);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数
	if(Length)
	{
    API_USART_DMA_Send(USART1,rxBuffer1,Length);	//串口DMA发送程序
		
	}
	if(USART_GetFlagStatus(USART1,USART_FLAG_TC))
	{		
		//USART_DMAPrintf(USART1,"自定义printf串口DMA发送程序,后边的省略号就是可变参数\r\n");					//自定义printf串口DMA发送程序,后边的省略号就是可变参数
	}
	else
	{
		//SysTick_DeleymS(5000);				//SysTick延时nmS
	}
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
static void function(void)
{
	static unsigned char i	=	0;
	if(i>=10)
	{
		SysTick_DeleyS(1);					//SysTick延时nS
		i=0;
	}
	
	if(0==i)
	{
		API_USART_DMA_Send(USART1,t1,4);	//串口DMA发送程序
		SysTick_DeleyuS(200);					//SysTick延时nuS
	}
	
	else if(1==i)
		API_USART_DMA_Send(USART1,ts1,4);	//串口DMA发送程序
	else if(2==i)
		API_USART_DMA_Send(USART1,td1,5);	//串口DMA发送程序
	else if(3==i)
	{
		API_USART_DMA_Send(USART1,te1,4);	//串口DMA发送程序
		SysTick_DeleyuS(800);					//SysTick延时nuS
	}
	
	else if(4==i)
		API_USART_DMA_Send(USART1,ts2,4);	//串口DMA发送程序
	else if(5==i)
		API_USART_DMA_Send(USART1,td2,5);	//串口DMA发送程序
	else if(6==i)
	{
		API_USART_DMA_Send(USART1,te2,4);	//串口DMA发送程序
		SysTick_DeleyuS(500);					//SysTick延时nuS
	}
	
	else if(7==i)
		API_USART_DMA_Send(USART1,ts3,4);	//串口DMA发送程序
	else if(8==i)
		API_USART_DMA_Send(USART1,td3,5);	//串口DMA发送程序
	else if(9==i)
		API_USART_DMA_Send(USART1,te3,4);	//串口DMA发送程序
	
	i+=1;
}
#endif

