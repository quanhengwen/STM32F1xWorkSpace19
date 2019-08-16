#ifdef USART_TEST

#include "USART_TEST.H"

#include "STM32_USART.H"
#include "STM32_WDG.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"



#include	"stdio.h"			//用于printf
#include	"string.h"		//用于printf
#include	"stdarg.h"		//用于获取不确定个数的参数
#include	"stdlib.h"		//malloc动态申请内存空间


#define	BufferSize 512		//DMA1缓冲大小

unsigned char rxd[BufferSize]={0};
unsigned char txd[BufferSize]={0};


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
	
	api_usart_configuration_NR(USART2,19200,BufferSize);	//USART_DMA配置--查询方式，不开中断
  
  SysTick_Configuration(1000);	//系统嘀嗒时钟配置72MHz,单位为uS
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
	unsigned short rxdlen=0;
	rxdlen	=	api_usart_receive(USART2,rxd);
	if(rxdlen)
	{
		api_usart_send(USART2,rxd,rxdlen);
	}
}

#endif

