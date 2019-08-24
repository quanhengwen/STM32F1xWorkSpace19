#ifdef NE01V0A1

#include "NE01V0A1.H"

#include	"AMP_Protocol.H"

#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_TIM.H"
#include "STM32_USART.H"
#include "STM32_SPI.H"

#include "SWITCHID.H"
#include "IOT5302W.H"     //读卡器

#include 	"CRC.H"

#include "string.h"				//串和内存操作函数头文件

//------------------系统LED指示灯
#define ampSYSLEDPort    						GPIOA
#define ampSYSLEDPin     						GPIO_Pin_0
//------------------通讯接口--与控制板连接
#define ampCommBusPort     					USART1
#define ampCommBusCtlPort 					GPIOA
#define ampCommBusCtlPin  					GPIO_Pin_8
#define ampCommBusBaudRate        	115200
//------------------通讯接口--与LCD连接
#define ampCommLayPort     					USART2
#define ampCommLayCtlPort 					GPIOA
#define ampCommLayCtlPin  					GPIO_Pin_1
#define ampCommLayBaudRate        	115200

/* Private variables ---------------------------------------------------------*/

static RS485Def ampRS485Bus;   //uart4,PA15   		//层板接口
static RS485Def ampRS485lay;   //usart1,PA8    	//副柜接口

#define ampCommBusBufferSize 512

unsigned char ampCommBusRxd[ampCommBusBufferSize]={0};
unsigned char ampCommBusTxd[ampCommBusBufferSize]={0};

unsigned char ampCommLayRxd[ampCommBusBufferSize]={0};
unsigned char ampCommLayTxd[ampCommBusBufferSize]={0};

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void API_NE01V0A1_Configuration(void)
{	
	SYS_Configuration();					//系统配置---打开系统时钟 STM32_SYS.H	

	Communication_Configuration();
  GPIO_Configuration_OPP50(ampSYSLEDPort,ampSYSLEDPin);
	
	IWDG_Configuration(3000);			//独立看门狗配置---参数单位ms
  
  SysTick_Configuration(1000);	//系统嘀嗒时钟配置72MHz,单位为uS

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
void API_NE01V0A1_Server(void)
{ 
	IWDG_Feed();														//独立看门狗喂狗
	
	SysLed_server();
	Communication_Server();
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void SysLed_server(void)
{
	static unsigned short time = 0;
	if(time++>1000)
	{
		time=0;
		api_gpio_toggle(ampSYSLEDPort,ampSYSLEDPin);		//将GPIO相应管脚输出翻转----V20170605
	}
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void Communication_Configuration(void)
{
	//-----------------------------副柜接口UART4
  ampRS485Bus.USARTx  				=	ampCommBusPort;
	ampRS485Bus.RS485_CTL_PORT	=	ampCommBusCtlPort;
	ampRS485Bus.RS485_CTL_Pin		=	ampCommBusCtlPin;
	api_rs485_configuration_NR(&ampRS485Bus,ampCommBusBaudRate,maxFramesize);
	
  //-----------------------------层板接口USART2
  ampRS485lay.USARTx  = ampCommLayPort;
  ampRS485lay.RS485_CTL_PORT  = ampCommLayCtlPort;
  ampRS485lay.RS485_CTL_Pin   = ampCommLayCtlPin;
  api_rs485_configuration_NR(&ampRS485lay,ampCommLayBaudRate,maxFramesize);
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void Communication_Server(void)
{
	unsigned short len=0;
  len	=	api_rs485_receive(&ampRS485Bus,ampCommLayTxd);
	if(len)
	{
		api_rs485_send_force(&ampRS485lay,ampCommLayTxd,len);
	}
	len	=	api_rs485_receive(&ampRS485lay,ampCommBusTxd);
	if(len)
	{
		api_rs485_send_force(&ampRS485Bus,ampCommBusTxd,len);
	}
}
//------------------------------------------------------------------------------
#endif
