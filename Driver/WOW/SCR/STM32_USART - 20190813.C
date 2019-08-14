/******************************** User_library *********************************
* 文件名 	: STM32_SDCard.H
* 作者   	: wegam@sina.com
* 版本   	: V
* 日期   	: 2016/01/01
* 说明   	: 
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/


#include "STM32_USART.H"

#include "STM32_GPIO.H"
//#include "STM32_SYSTICK.H"
//#include 	"LinkedList.H"

#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_nvic.h"
#include "stm32f10x_rcc.h"

#include	"stdio.h"			//用于printf
#include	"string.h"		//用于printf
#include	"stdarg.h"		//用于获取不确定个数的参数
#include	"stdlib.h"		//malloc动态申请内存空间


/*##############################################################################
################################################################################
设置波特率时的误差计算
--------------------------------------------------------------------------------
波特率				|fPCLK = 36MHz	误差%			|		fPCLK = 72MHz	误差%			|
--------------------------------------------------------------------------------
2400				|						0%						|						0%							
--------------------------------------------------------------------------------
9600				|						0%						|						0%								
--------------------------------------------------------------------------------
19200				|						0%						|						0%							
--------------------------------------------------------------------------------
57600				|						0%						|						0%							
--------------------------------------------------------------------------------
115200			|						0.15%					|						0%							
--------------------------------------------------------------------------------
230400			|						0.16%					|						0.16%							
--------------------------------------------------------------------------------
460800			|						0.16%					|						0.16%							
--------------------------------------------------------------------------------
921600			|						0.16%					|						0.16%							
--------------------------------------------------------------------------------
2250000			|						0%						|						0%							
--------------------------------------------------------------------------------

################################################################################
###############################################################################*/

//--------USART全局变量定义
#define	uart_dma_buffer_size	1024	//默认串口DMA接收缓冲大小,如果配置时未输入缓存大小时使用的默认值
#define	uBaudRate	115200	//默认串口波特率

struct
{
  unsigned short nUSART1;
  unsigned short nUSART2;
  unsigned short nUSART3;
  unsigned short nUART4;
//  unsigned short nUART5;	  //----无DMA
}DmaSize;      //配置时设置的DMA接收缓存大小
struct
{
  unsigned short nUSART1;
  unsigned short nUSART2;
  unsigned short nUSART3;
  unsigned short nUART4;
//  unsigned short nUART5;	  //----无DMA
}RemaDmaSize;     //DMA开启后剩余DMA接收缓存大小，通过与设置的DMA缓存大小来判断数据是否在接收
struct
{
  unsigned short nUSART1;
  unsigned short nUSART2;
  unsigned short nUSART3;
  unsigned short nUART4;
//  unsigned short nUART5;	  //----无DMA
}rx_retry_count;
struct
{
  unsigned short nUSART1;
  unsigned short nUSART2;
  unsigned short nUSART3;
  unsigned short nUART4;
//  unsigned short nUART5;	  //----无DMA
}tx_retry_count;

struct
{
  unsigned char nUSART1	:1;
  unsigned char nUSART2	:1;
  unsigned char nUSART3	:1;
  unsigned char nUART4	:1;
  unsigned char nUART5	:1;	  //----无DMA
}usart_type;		//串口类型0---USART,1-RS485
struct
{
  unsigned char nUSART1	:1;
  unsigned char nUSART2	:1;
  unsigned char nUSART3	:1;
  unsigned char nUART4	:1;
  unsigned char nUART5	:1;	  //----无DMA
}usart_tx_idle_flag;		//空闲标志0---忙,1-空闲
struct
{
  unsigned char nUSART1	:1;
  unsigned char nUSART2	:1;
  unsigned char nUSART3	:1;
  unsigned char nUART4	:1;
  unsigned char nUART5	:1;	  //----无DMA
}usart_rx_idle_flag;		//空闲标志0---忙,1-空闲
struct
{
  RS485Def *rs485_usart1;
  RS485Def *rs485_usart2;
	RS485Def *rs485_usart3;
  RS485Def *rs485_uart4;
  RS485Def *rs485_uart5;
}rs485_addr;		//RS485地址

/* Private variables ---------------------------------------------------------*/
unsigned char uRx1Addr[uart_dma_buffer_size]={0};
unsigned char uRx2Addr[uart_dma_buffer_size]={0};
unsigned char uRx3Addr[uart_dma_buffer_size]={0};
unsigned char uRx4Addr[uart_dma_buffer_size]={0};

unsigned char uTx1Addr[uart_dma_buffer_size]={0};
unsigned char uTx2Addr[uart_dma_buffer_size]={0};
unsigned char uTx3Addr[uart_dma_buffer_size]={0};
unsigned char uTx4Addr[uart_dma_buffer_size]={0};
/* Private function prototypes -----------------------------------------------*/
//*****************RS485收发控制


/* function-------------------------------------------------------------------*/
/*******************************************************************************/








//-----------------------------------------------------------------------------
//-----------------------------------API---------------------------------------
//-----------------------------------------------------------------------------
/*******************************************************************************
* 函数名			:	USART_ReadBuffer
* 功能描述		:	串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数，然后重新将接收缓冲区地址指向RxdBuffer， 
* 输入			: void
* 返回值			: void
*******************************************************************************/
unsigned short api_usart_dma_receive(USART_TypeDef* USARTx,u8 *RevBuffer)
{
	unsigned short len=0;
	if(get_usart_rx_idle(USARTx))
	{		
		len	= get_usart_rx_dma_buffer(USARTx,RevBuffer);
		del_usart_rx_idle(USARTx);
	}
	return len;
}
/*******************************************************************************
*函数名			:	RS485_ReadBufferIDLE
*功能描述		:	串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数，然后重新将接收缓冲区地址指向RxdBuffer
*输入				: 
*返回值			:	无
*******************************************************************************/
unsigned short api_rs485_dma_receive(RS485Def *pRS485,u8 *RevBuffer)	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数，然后重新将接收缓冲区地址指向RxdBuffer，
{
	USART_TypeDef* USARTx	=	pRS485->USARTx;
	if(get_usart_tx_idle(USARTx))
	{
		set_rs485_tx(pRS485,DISABLE);
		set_rs485_rx(pRS485,ENABLE);
		del_usart_tx_idle(USARTx);		
	}
	if(get_usart_rx_idle(USARTx))
	{
		unsigned short len=0;
		len	= get_usart_rx_dma_buffer(USARTx,RevBuffer);
		return len;
	}
	return 0;
}
//-----------------------------------------------------------------------------



/*******************************************************************************
*函数名			:	
*功能描述		:	串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
*输入				: 
*返回值			:	如果数据已经传入到DMA，返回Buffer大小，否则返回0（发送器忙）
*******************************************************************************/
unsigned short api_usart_dma_send(USART_TypeDef* USARTx,u8 *tx_buffer,u16 BufferSize)		//串口DMA发送程序
{
	if(0==get_usart_tx_idle(USARTx))		//串口状态检查
		return 0;
	return set_usart_tx_dma_buffer(USARTx,tx_buffer,BufferSize);		//串口DMA发送程序	
}
/*******************************************************************************
*函数名			:	
*功能描述		:	串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
*输入				: 
*返回值			:	如果数据已经传入到DMA，返回Buffer大小，否则返回0（发送器忙）
*******************************************************************************/
unsigned short api_rs485_dma_send(RS485Def *pRS485,u8 *tx_buffer,u16 BufferSize)		//RS485-DMA发送程序
{
	//-------------接收空闲和发送空闲
  unsigned short sendedlen  =0;
	if(get_usart_tx_idle(pRS485->USARTx)&&get_usart_rx_idle(pRS485->USARTx))
	{
		set_rs485_tx(pRS485,ENABLE);
		set_rs485_rx(pRS485,DISABLE);
		sendedlen = set_usart_tx_dma_buffer(pRS485->USARTx,(u8*)tx_buffer,BufferSize);		//串口DMA发送程序
	}
	return sendedlen;
}
/*******************************************************************************
*函数名		: function
*功能描述	:	串口接收服务函数
*输入			: 
*输出			:	无
*返回值		:	无
*例程			:	api_usart_dma_printf(USART2,"中文ENG=%d\n",num);
*特别说明	:	在DMA发送完成后需要释放动态空间，free(USART_BUFFER);
					:	USART_BUFFER定义在STM32_USART.H
*******************************************************************************/
unsigned short api_usart_dma_printf(USART_TypeDef* USARTx,const char *format,...)
{
	
//		va_list ap; 										//VA_LIST 是在C语言中解决变参问题的一组宏，所在头文件：#include <stdarg.h>,用于获取不确定个数的参数
//    static char string[ 256 ];			//定义数组，
//    va_start( ap, format );
//    vsprintf( string , format, ap );    
//    va_end( ap );	

	unsigned short	str_len=0;
	unsigned char		format_buffer[256]={0};
	
	//3)**********args为定义的一个指向可变参数的变量，va_list以及下边要用到的va_start,va_end都是是在定义，可变参数函数中必须要用到宏， 在stdarg.h头文件中定义
	va_list args;  
	//5)**********初始化args的函数，使其指向可变参数的第一个参数，format是可变参数的前一个参数
	va_start(args, format);
	//6)**********正常情况下返回生成字串的长度(除去\0),错误情况返回负值
	str_len = vsprintf((char*)format_buffer, format, args);
	//7)**********结束可变参数的获取
	va_end(args); 
	//8)**********将等发送缓冲区大小（数据个数）及缓冲区地址发给DMA开启发送	
	if(0==get_usart_tx_idle(USARTx))		//串口发送是否空闲
		return 0;
	return set_usart_tx_dma_buffer(USARTx,(u8*)format_buffer,str_len);		//串口DMA发送程序
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*******************************************************************************/
unsigned short api_rs485_dma_printf(RS485Def *pRS485,const char *format,...)
{
		
//		va_list ap; 										//VA_LIST 是在C语言中解决变参问题的一组宏，所在头文件：#include <stdarg.h>,用于获取不确定个数的参数
//    static char string[ 256 ];			//定义数组，
//    va_start( ap, format );
//    vsprintf( string , format, ap );    
//    va_end( ap );
	//2)**********定义缓冲区大小变量
	unsigned short	str_len;
	unsigned char		format_buffer[256]={0};
	//3)**********args为定义的一个指向可变参数的变量，va_list以及下边要用到的va_start,va_end都是是在定义，可变参数函数中必须要用到宏， 在stdarg.h头文件中定义
	va_list args;  
	//5)**********初始化args的函数，使其指向可变参数的第一个参数，format是可变参数的前一个参数
	va_start(args, format);
	//6)**********正常情况下返回生成字串的长度(除去\0),错误情况返回负值
	str_len = vsprintf((char*)format_buffer, format, args);
	//7)**********结束可变参数的获取
	va_end(args); 
	//8)**********将等发送缓冲区大小（数据个数）及缓冲区地址发给DMA开启发送
	//8)**********DMA发送完成后注意应该释放缓冲区：free(USART_BUFFER);
	if(get_usart_rx_idle(pRS485->USARTx))
	{
		set_rs485_tx(pRS485,ENABLE);
		set_rs485_rx(pRS485,DISABLE);
		return set_usart_tx_dma_buffer(pRS485->USARTx,(u8*)format_buffer,str_len);		//串口DMA发送程序
	}
	else
		return 0;
}
//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------API-configuration
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
void api_usart_dma_configurationST(USART_TypeDef* USARTx,USART_InitTypeDef* USART_InitStructure,unsigned short BufferSize)	//USART_DMA配置--结构体形式，不开中断
{
	usart_rcc_initialize(USARTx);
	usart_rcc_initialize(USARTx);
	usart_gpio_initialize(USARTx);										//串口GPIO配置
	usart_initialize_st(USARTx,USART_InitStructure);	//USART_DMA配置--查询方式，不开中断
	usart_dma_initialize(USARTx,BufferSize);					//USART_DMA配置--查询方式，不开中断
}
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
void api_usart_dma_configurationNR(USART_TypeDef* USARTx,u32 USART_BaudRate,unsigned short BufferSize)	//USART_DMA配置--查询方式，不开中断
{
	usart_rcc_initialize(USARTx);
	usart_gpio_initialize(USARTx);							//串口GPIO配置
	usart_initialize_nr(USARTx,USART_BaudRate);	//USART_DMA配置--查询方式，不开中断
	usart_dma_initialize(USARTx,BufferSize);		//USART_DMA配置--查询方式，不开中断
}
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
*输入				: 
*返回值			:	无
*******************************************************************************/
void api_rs485_dma_configurationNR(RS485Def *pRS485,u32 USART_BaudRate,unsigned short BufferSize)	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
{	
	set_usart_type(pRS485->USARTx,1);		//设置串口类型：0---USART,1-RS485
	set_rs485_addr(pRS485);
	
	if(pRS485->RS485_CTL_PORT)
		GPIO_Configuration_OPP50	(pRS485->RS485_CTL_PORT,pRS485->RS485_CTL_Pin);			//将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度50MHz----V20170605
	
	if(pRS485->RS485_TxEn_PORT)
		GPIO_Configuration_OPP50	(pRS485->RS485_TxEn_PORT,pRS485->RS485_TxEn_Pin);			//将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度50MHz----V20170605
	
	if(pRS485->RS485_RxEn_PORT)
		GPIO_Configuration_OPP50	(pRS485->RS485_RxEn_PORT,pRS485->RS485_RxEn_Pin);			//将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度50MHz----V20170605	
	
	set_rs485_tx(pRS485,DISABLE);
	set_rs485_rx(pRS485,ENABLE);
	usart_rcc_initialize(pRS485->USARTx);
	usart_gpio_initialize(pRS485->USARTx);							//串口GPIO配置	
	usart_initialize_nr(pRS485->USARTx,USART_BaudRate);	//USART_DMA配置--查询方式，不开中断
	usart_dma_initialize(pRS485->USARTx,BufferSize);		//USART_DMA配置--查询方式，不开中断
}
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
void api_usart_dma_configuration_even(USART_TypeDef* USARTx,u32 USART_BaudRate,unsigned short BufferSize)	//USART_DMA配置--查询方式，不开中断,偶校验
{
	usart_rcc_initialize(USARTx);
	usart_gpio_initialize(USARTx);							//串口GPIO配置
	usart_initialize_even(USARTx,USART_BaudRate);	//USART_DMA配置--查询方式，不开中断--偶校验
	usart_dma_initialize(USARTx,BufferSize);		//USART_DMA配置--查询方式，不开中断
}
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
void api_usart_dma_configuration_odd(USART_TypeDef* USARTx,u32 USART_BaudRate,unsigned short BufferSize)	//USART_DMA配置--查询方式，不开中断,偶校验
{
	usart_rcc_initialize(USARTx);
	usart_gpio_initialize(USARTx);							//串口GPIO配置
	usart_initialize_odd(USARTx,USART_BaudRate);	//USART_DMA配置--查询方式，不开中断--偶校验
	usart_dma_initialize(USARTx,BufferSize);		//USART_DMA配置--查询方式，不开中断
}
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--中断方式
*输入				: 
*返回值			:	无
*******************************************************************************/
void api_usart_dma_configuration_idleIT(USART_TypeDef* USARTx,u32 USART_BaudRate,unsigned short BufferSize)	//USART_DMA配置--中断方式
{
  //2)******************************GPIO配置
	usart_rcc_initialize(USARTx);
  usart_gpio_initialize(USARTx);                //串口GPIO配置
  //2)******************************中断配置
  usart_it_initialize(USARTx);	                //串口中断配置	
	
	USART_ITConfig(USARTx,USART_IT_IDLE, ENABLE);					//使用空闲中断，DMA自动接收，检测到总线空闲表示发送端已经发送完成，数据保存在DMA缓冲器中
	USART_ClearITPendingBit(USARTx,USART_IT_IDLE); 				//清除空闲串口标志位
	
	USART_Cmd(USARTx, ENABLE);
  
  //2)******************************DMA
  usart_dma_initialize(USARTx,BufferSize);	//USART_DMA配置--查询方式，不开中断
}
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
void api_usart_dma_configurationNR_Remap(USART_TypeDef* USARTx,u32 USART_BaudRate,unsigned short BufferSize)	//USART_DMA配置--完全映射，不开中断
{
	usart_gpio_initialize_Remap(USARTx);				//串口GPIO配置--完全映射
	usart_initialize_nr(USARTx,USART_BaudRate);	//USART_DMA配置--查询方式，不开中断
	usart_dma_initialize(USARTx,BufferSize);		//USART_DMA配置--查询方式，不开中断
}
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
*输入				: 
*返回值			:	无
*******************************************************************************/
void api_rs485_dma_configurationNR_Remap(RS485Def *pRS485,u32 USART_BaudRate,unsigned short BufferSize)	//USART_DMA配置--完全映射
{
	usart_gpio_initialize_Remap(pRS485->USARTx);				//串口GPIO配置--完全映射
	usart_initialize_nr(pRS485->USARTx,USART_BaudRate);	//USART_DMA配置--查询方式，不开中断
	usart_dma_initialize(pRS485->USARTx,BufferSize);		//USART_DMA配置--查询方式，不开中断
	set_rs485_addr(pRS485);
	if(pRS485->RS485_CTL_PORT)
		GPIO_Configuration_OPP50	(pRS485->RS485_CTL_PORT,pRS485->RS485_CTL_Pin);			//将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度50MHz----V20170605
	
	if(pRS485->RS485_TxEn_PORT)
		GPIO_Configuration_OPP50	(pRS485->RS485_CTL_PORT,pRS485->RS485_TxEn_Pin);			//将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度50MHz----V20170605
	
	if(pRS485->RS485_RxEn_PORT)
		GPIO_Configuration_OPP50	(pRS485->RS485_CTL_PORT,pRS485->RS485_RxEn_Pin);			//将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度50MHz----V20170605	
	
	set_rs485_tx(pRS485,DISABLE);
	set_rs485_rx(pRS485,ENABLE);
}
//-----------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
*输入				: 
*返回值			:	无
*******************************************************************************/
void api_rs485_dma_configurationIT(RS485Def *pRS485,u32 USART_BaudRate,unsigned short BufferSize)	//USART_DMA配置--完全映射
{
	USART_TypeDef* USARTx	=	pRS485->USARTx;
	usart_rcc_initialize(USARTx);
	usart_gpio_initialize(USARTx);				//串口GPIO配置--完全映射
	usart_initialize_nr(USARTx,USART_BaudRate);	//USART_DMA配置--查询方式，不开中断
	usart_it_initialize(USARTx);	            	//串口中断配置
	
	set_usart_type(pRS485->USARTx,1);		//设置串口类型：0---USART,1-RS485
	set_rs485_addr(pRS485);	
	
	USART_Cmd(USARTx, ENABLE);
	/* Enable USART1 Receive and Transmit interrupts */
//  USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
//  USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);
	USART_ITConfig(USARTx, USART_IT_TC, ENABLE);
	USART_ClearITPendingBit(USARTx,USART_IT_TC); 					//清除空闲串口标志位
	
	USART_ITConfig(USARTx,USART_IT_IDLE, ENABLE);					//使用空闲中断，DMA自动接收，检测到总线空闲表示发送端已经发送完成，数据保存在DMA缓冲器中
	USART_ClearITPendingBit(USARTx,USART_IT_IDLE); 				//清除空闲串口标志位
	
	if(pRS485->RS485_CTL_PORT)
		GPIO_Configuration_OPP50	(pRS485->RS485_CTL_PORT,pRS485->RS485_CTL_Pin);			//将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度50MHz----V20170605
	
	if(pRS485->RS485_TxEn_PORT)
		GPIO_Configuration_OPP50	(pRS485->RS485_CTL_PORT,pRS485->RS485_TxEn_Pin);			//将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度50MHz----V20170605
	
	if(pRS485->RS485_RxEn_PORT)
		GPIO_Configuration_OPP50	(pRS485->RS485_CTL_PORT,pRS485->RS485_RxEn_Pin);			//将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度50MHz----V20170605	
	
	set_rs485_tx(pRS485,DISABLE);
	set_rs485_rx(pRS485,ENABLE);
	
	usart_dma_initialize(USARTx,BufferSize);		//USART_DMA配置--查询方式，不开中断	
}
//-----------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	USART_ConfigurationIT
*功能描述		:	USART_配置---常规中断方式
*输入				: 
*返回值			:	无
*修改时间		:	2018/01/06
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_usart_configuration_it(USART_TypeDef* USARTx,u32 USART_BaudRate)	//USART_中断方式配置
{
	//1)**********定义变量	
	usart_rcc_initialize(USARTx);
	usart_gpio_initialize(USARTx);						//串口GPIO配置
	usart_it_initialize(USARTx);	           	//串口中断配置	
	
	USART_ITConfig(USARTx,USART_IT_IDLE, ENABLE);					//使用空闲中断，DMA自动接收，检测到总线空闲表示发送端已经发送完成，数据保存在DMA缓冲器中
	USART_ClearITPendingBit(USARTx,USART_IT_IDLE); 				//清除空闲串口标志位
	
	/* Enable USART1 Receive and Transmit interrupts */
  USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
//  USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);
	USART_ITConfig(USARTx, USART_IT_TC, ENABLE);
	
	USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
	USART_ClearITPendingBit(USARTx, USART_IT_TC);
	
	USART_Cmd(USARTx, ENABLE);
}
//-----------------------------------------------------------------------------






//-----------------------------------------------------------------------------
//---------------------------------static1-------------------------------------
//-----------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void usart_rcc_initialize(USART_TypeDef* USARTx)
{
	switch(*(u32*)&USARTx)
	{
		case 	USART1_BASE:
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//USART1时钟开启			
					break;
		case 	USART2_BASE:
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//USART2时钟开启		
					break;
		case 	USART3_BASE:
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//USART3时钟开启		
					break;
		case 	UART4_BASE:
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);	//UART4时钟开启	
					break;
		case 	UART5_BASE:   //UART5不支持DMA
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);	//UART4时钟开启	
					break;
		default :break;
	}
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
static void	usart_gpio_initialize(USART_TypeDef* USARTx)	//串口GPIO配置
{
	//1)**********定义变量	
	GPIO_InitTypeDef GPIO_InitStructure;					//GPIO结构体
	u16 TXD_Pin=0;																//串口发送脚
	u16 RXD_Pin=0;																//串口接收脚
	GPIO_TypeDef* GPIO_TX=0;
	GPIO_TypeDef* GPIO_RX=0;
	switch(*(u32*)&USARTx)
	{
		case 	USART1_BASE:
			
					GPIO_TX=GPIOA;
					GPIO_RX=GPIOA;
					TXD_Pin=GPIO_Pin_9;											//USART1-TX>PA9
					RXD_Pin=GPIO_Pin_10;										//USART1-RX>PA10
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
					break;
		case 	USART2_BASE:
			
					GPIO_TX=GPIOA;
					GPIO_RX=GPIOA;
					TXD_Pin=GPIO_Pin_2;		//USART2-TX>PA2
					RXD_Pin=GPIO_Pin_3;		//USART2-RX>PA3
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
					break;
		case 	USART3_BASE:
			
					GPIO_TX=GPIOB;
					GPIO_RX=GPIOB;
					TXD_Pin=GPIO_Pin_10;	//USART3-TX>PB10
					RXD_Pin=GPIO_Pin_11;	//USART3-RX>PB11
					
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);				//关闭AFIO时钟,为关闭JTAG功能
					GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  //关闭JTAG功能
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
					break;
		case 	UART4_BASE:

					GPIO_TX=GPIOC;
					GPIO_RX=GPIOC;
					TXD_Pin=GPIO_Pin_10;	//USART1-TX>PC10
					RXD_Pin=GPIO_Pin_11;	//USART1-RX>PC11
					
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE);
					break;
		case 	UART5_BASE:   //UART5不支持DMA
			
          GPIO_TX=GPIOC;
					GPIO_RX=GPIOD;
					TXD_Pin=GPIO_Pin_10;	//USART1-TX>PC12
					RXD_Pin=GPIO_Pin_2; 	//USART1-RX>PD2
          RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,ENABLE);
					break;
		default :break;
	}
	//3)**********初始化串口
	//3.1)**********初始化TXD引脚
	GPIO_InitStructure.GPIO_Pin 		= TXD_Pin;
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_Init(GPIO_TX,&GPIO_InitStructure);

	//3.2)**********初始化RXD引脚
	GPIO_InitStructure.GPIO_Pin 		= RXD_Pin;
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_IPU;							//上拉输入
	GPIO_Init(GPIO_RX,&GPIO_InitStructure);
}
/*******************************************************************************
*函数名			:	usart_gpio_initialize_Remap
*功能描述		:	USART_GPIO完全映射配置
*输入				: 
*返回值			:	无
*******************************************************************************/
static void	usart_gpio_initialize_Remap(USART_TypeDef* USARTx)	//USART_GPIO完全映射配置
{
	//1)**********定义变量	
	GPIO_InitTypeDef GPIO_InitStructure;					//GPIO结构体
	
	u16 TXD_Pin=0;																//串口发送脚
	u16 RXD_Pin=0;																//串口接收脚
	GPIO_TypeDef* GPIO_TX=0;
	GPIO_TypeDef* GPIO_RX=0;

	//2)******************************配置相关GPIO/串口时钟打开
	//2.1)**********USART1
	if(USARTx==USART1)
	{
		TXD_Pin=GPIO_Pin_6;									  //USART1-TX>PA9
		RXD_Pin=GPIO_Pin_7;										//USART1-RX>PA10
		
		GPIO_TX=GPIOB;
		GPIO_RX=GPIOB;
		
		GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);				//I/O口重映射开启
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//USART1时钟开启

	}
	//2.2)**********USART2
	else if(USARTx==USART2)
	{		
		TXD_Pin=GPIO_Pin_5;		//USART2-TX>PA2
		RXD_Pin=GPIO_Pin_6;		//USART2-RX>PA3
		
		GPIO_TX=GPIOD;
		GPIO_RX=GPIOD;
		
		GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);				//I/O口重映射开启
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//USART1时钟开启

	}
	//2.3)**********USART3
	else if(USARTx==USART3)
	{		
		TXD_Pin=GPIO_Pin_8;	//USART3-TX>PB10
		RXD_Pin=GPIO_Pin_9;	//USART3-RX>PB11
		
		GPIO_TX=GPIOD;
		GPIO_RX=GPIOD;
		
		GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE);				//I/O口重映射开启
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//USART1时钟开启
	}
	//2.4)**********USART4
	else if(USARTx==UART4)	//无映射
	{
		return;
	}
	//2.5)**********USART5
	else if(USARTx==UART5)	//无映射
	{		
		return;
	}
	//3)**********初始化串口
	//3.1)**********初始化TXD引脚
	GPIO_InitStructure.GPIO_Pin = TXD_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_TX,&GPIO_InitStructure);

	//3.2)**********初始化RXD引脚
	GPIO_InitStructure.GPIO_Pin = RXD_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;							//上拉输入
	GPIO_Init(GPIO_RX,&GPIO_InitStructure);	
}
//-----------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
static void usart_initialize_st(USART_TypeDef* USARTx,USART_InitTypeDef* USART_InitStructure)	//USART标准配置
{
	//3.3)**********初始化串口参数
	USART_DeInit(USARTx);
	
	USART_InitStructure->USART_Mode        = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure->USART_HardwareFlowControl = USART_HardwareFlowControl_None;//流控
	
	USART_Init(USARTx, USART_InitStructure);								//初始化串口
	
  USART_ITConfig(USARTx,USART_IT_IDLE, DISABLE);					//使用空闲中断，DMA自动接收，检测到总线空闲表示发送端已经发送完成，数据保存在DMA缓冲器中
	USART_ClearITPendingBit(USARTx,USART_IT_IDLE); 					//清除空闲串口标志位
	USART_Cmd(USARTx, ENABLE);
  //2)******************************DMA
}
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
static void usart_initialize_nr(USART_TypeDef* USARTx,u32 USART_BaudRate)	//USART_DMA配置--查询方式，不开中断
{
	USART_InitTypeDef USART_InitStructure;				//USART结构体	
	//3.3)**********初始化串口参数
	USART_DeInit(USARTx);
	USART_InitStructure.USART_BaudRate    = USART_BaudRate; 					  //波特率
	USART_InitStructure.USART_WordLength  = USART_WordLength_8b;		    //数据位
	USART_InitStructure.USART_StopBits    = USART_StopBits_1;				    //停止位
	USART_InitStructure.USART_Parity      = USART_Parity_No ; 					//奇偶校验
	USART_InitStructure.USART_Mode        = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//流控
	USART_Init(USARTx, &USART_InitStructure);											//初始化串口
	
  USART_ITConfig(USARTx,USART_IT_IDLE, DISABLE);					//使用空闲中断，DMA自动接收，检测到总线空闲表示发送端已经发送完成，数据保存在DMA缓冲器中
	USART_ClearITPendingBit(USARTx,USART_IT_IDLE); 					//清除空闲串口标志位
	USART_Cmd(USARTx, ENABLE);
  //2)******************************DMA
}
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
static void usart_initialize_even(USART_TypeDef* USARTx,u32 USART_BaudRate)	//USART_DMA配置--查询方式，不开中断--偶校验
{
	//1)**********定义变量	
	
	USART_InitTypeDef USART_InitStructure;				//USART结构体
	//3.3)**********初始化串口参数
	USART_DeInit(USARTx);
	USART_InitStructure.USART_BaudRate 		= USART_BaudRate; 			//波特率
	USART_InitStructure.USART_WordLength 	= USART_WordLength_9b;	//数据位
	USART_InitStructure.USART_StopBits 		= USART_StopBits_1;			//停止位
	USART_InitStructure.USART_Parity 			= USART_Parity_Even ; 	//偶校验
	USART_InitStructure.USART_Mode				= USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//流控
	USART_Init(USARTx, &USART_InitStructure);											//初始化串口
	

	USART_ITConfig(USARTx,USART_IT_IDLE, DISABLE);					//使用空闲中断，DMA自动接收，检测到总线空闲表示发送端已经发送完成，数据保存在DMA缓冲器中
	USART_ClearITPendingBit(USARTx,USART_IT_IDLE); 					//清除空闲串口标志位
	
	USART_Cmd(USARTx, ENABLE);
}
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
static void usart_initialize_odd(USART_TypeDef* USARTx,u32 USART_BaudRate)	//USART_DMA配置--查询方式，不开中断--奇校验
{
	//1)**********定义变量	
	USART_InitTypeDef USART_InitStructure;				//USART结构体
	//3.3)**********初始化串口参数
	USART_DeInit(USARTx);
	USART_InitStructure.USART_BaudRate = USART_BaudRate; 					//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;		//数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;				//停止位
	USART_InitStructure.USART_Parity = USART_Parity_Odd ; 				//奇偶校验-奇校验
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//流控
	USART_Init(USARTx, &USART_InitStructure);											//初始化串口
	
	USART_ITConfig(USARTx,USART_IT_IDLE, DISABLE);					//使用空闲中断，DMA自动接收，检测到总线空闲表示发送端已经发送完成，数据保存在DMA缓冲器中
	USART_ClearITPendingBit(USARTx,USART_IT_IDLE); 					//清除空闲串口标志位
	
	USART_Cmd(USARTx, ENABLE);
}
/*******************************************************************************/
static void	usart_dma_initialize(USART_TypeDef* USARTx,u16 BufferSize)	//USART_DMA配置--查询方式，不开中断
{
	//1)**********定义变量	
	DMA_InitTypeDef DMA_Initstructure;
	
	unsigned char*	uart_rxd	=	get_usart_rx_data_addr(USARTx);
	unsigned short 	dma_size	=	set_usart_dma_buffer_size(USARTx,BufferSize);
	
	DMA_Channel_TypeDef* DMAx_Channeltx=get_usart_tx_dma_channel(USARTx);			//DMA发送通道请求信号---当DMA串口发送数据完成时，会发起DMA中断
	DMA_Channel_TypeDef* DMAx_Channelrx=get_usart_rx_dma_channel(USARTx);			//DMA接收通道请求信号---DMA串口接收由串口发起中断，因此此处接收通道中断不使用	
	if(0==uart_rxd)
		return;
	if(0==DMAx_Channeltx)
		return;
	if(0==DMAx_Channelrx)
		return;
	//4)**********根据串口索引相关DMA通道及其它参数
	switch(*(u32*)&USARTx)
	{
		case 	USART1_BASE:
					RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);	
					break;
		case 	USART2_BASE:
					RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
					break;
		case 	USART3_BASE:
					RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
					break;
		case 	UART4_BASE:
					RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2,ENABLE);
					break;
		case 	UART5_BASE:
					//UART5不支持DMA
					return;
		default :break;
	}

	//5)**********DMA发送初始化，外设作为DMA的目的端
	DMA_Initstructure.DMA_PeripheralBaseAddr  = (u32)(&USARTx->DR);						//DMA外设源地址
	DMA_Initstructure.DMA_MemoryBaseAddr      = (u32)uart_rxd;								//DMA数据内存地址
	DMA_Initstructure.DMA_DIR                 = DMA_DIR_PeripheralDST;			  //DMA_DIR_PeripheralDST（外设作为DMA的目的端），DMA_DIR_PeripheralSRC（外设作为数据传输的来源）
	DMA_Initstructure.DMA_BufferSize          = 0; 								  					//指定DMA通道的DMA缓存的大小
	DMA_Initstructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;	  //DMA_PeripheralInc_Enable（外设地址寄存器递增），DMA_PeripheralInc_Disable（外设地址寄存器不变），
	DMA_Initstructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;				  //DMA_MemoryInc_Enable（内存地址寄存器递增），DMA_MemoryInc_Disable（内存地址寄存器不变）
	DMA_Initstructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte;	//外设数据宽度--DMA_PeripheralDataSize_Byte（数据宽度为8位），DMA_PeripheralDataSize_HalfWord（数据宽度为16位），DMA_PeripheralDataSize_Word（数据宽度为32位）
	DMA_Initstructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;		  //内存数据宽度--DMA_MemoryDataSize_Byte（数据宽度为8位），DMA_MemoryDataSize_HalfWord（数据宽度为16位），DMA_MemoryDataSize_Word（数据宽度为32位）
	DMA_Initstructure.DMA_Mode                = DMA_Mode_Normal;						  //DMA工作模式--DMA_Mode_Normal（只传送一次）, DMA_Mode_Circular（不停地传送）
	DMA_Initstructure.DMA_Priority            = DMA_Priority_High; 					  //DMA通道的转输优先级--DMA_Priority_VeryHigh（非常高）DMA_Priority_High（高)，DMA_Priority_Medium（中），DMA_Priority_Low（低）
	DMA_Initstructure.DMA_M2M                 = DMA_M2M_Disable;						  //DMA通道的内存到内存传输--DMA_M2M_Enable(设置为内存到内存传输)，DMA_M2M_Disable（非内存到内存传输）
	DMA_Init(DMAx_Channeltx,&DMA_Initstructure);														  //初始化DMA

	//6)**********DMA接收初始化，外设作为DMA的源端
	DMA_Initstructure.DMA_PeripheralBaseAddr  = (u32)(&USARTx->DR);						//DMA外设源地址
	DMA_Initstructure.DMA_MemoryBaseAddr      = (u32)uart_rxd;						  	//DMA数据内存地址
	DMA_Initstructure.DMA_DIR                 = DMA_DIR_PeripheralSRC;			  //DMA_DIR_PeripheralDST（外设作为DMA的目的端），DMA_DIR_PeripheralSRC（外设作为数据传输的来源）
	DMA_Initstructure.DMA_BufferSize          = dma_size; 								  	//指定DMA通道的DMA缓存的大小
	DMA_Initstructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;	  //DMA_PeripheralInc_Enable（外设地址寄存器递增），DMA_PeripheralInc_Disable（外设地址寄存器不变），
	DMA_Initstructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;				  //DMA_MemoryInc_Enable（内存地址寄存器递增），DMA_MemoryInc_Disable（内存地址寄存器不变）
	DMA_Initstructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte; 	//外设数据宽度--DMA_PeripheralDataSize_Byte（数据宽度为8位），DMA_PeripheralDataSize_HalfWord（数据宽度为16位），DMA_PeripheralDataSize_Word（数据宽度为32位）
	DMA_Initstructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;		  //内存数据宽度--DMA_MemoryDataSize_Byte（数据宽度为8位），DMA_MemoryDataSize_HalfWord（数据宽度为16位），DMA_MemoryDataSize_Word（数据宽度为32位）
	DMA_Initstructure.DMA_Mode                = DMA_Mode_Normal;						  //DMA工作模式--DMA_Mode_Normal（只传送一次）, DMA_Mode_Circular（不停地传送）
	DMA_Initstructure.DMA_Priority            = DMA_Priority_High; 					  //DMA通道的转输优先级--DMA_Priority_VeryHigh（非常高）DMA_Priority_High（高)，DMA_Priority_Medium（中），DMA_Priority_Low（低）
	DMA_Initstructure.DMA_M2M                 = DMA_M2M_Disable;						  //DMA通道的内存到内存传输--DMA_M2M_Enable(设置为内存到内存传输)，DMA_M2M_Disable（非内存到内存传输）
	DMA_Init(DMAx_Channelrx,&DMA_Initstructure);														  //初始化DMA	
	
	//8)**********配置相关中断
	//8.1)**********串口接收中断配置
	//--将串口接收中断关闭，然后开启串口空闲中断，利用DMA自动接收串口数据
	//--若DMA接收未开启，则使用串口接收中断
	//--若DMA接收开启，串口接收中断应该关闭，在DMA配置中会将串口接收中断关闭

	/* 启动DMA1通道5*/
	DMA_Cmd(DMAx_Channeltx,DISABLE);				//关闭DMA发送----需要发送时再打开
	//10.2)**********使能串口
	DMA_Cmd(DMAx_Channelrx,ENABLE);					//打开DMA接收----自动接收串口数据	
	//9.1)**********关闭DMA发送	
	
	//8.2)**********使能串口DMA方式接收
	USART_DMACmd(USARTx,USART_DMAReq_Rx,ENABLE);
	//8.3)**********使能串口DMA方式发送
	USART_DMACmd(USARTx,USART_DMAReq_Tx,ENABLE);
	//9)**********清除相关中断标志位	
	//	DMA_Cmd(DMAx_Channeltx,ENABLE);
	//9.2)**********使能相关DMA通道传输完成中断
	DMA_ITConfig(DMAx_Channeltx,DMA_IT_TC, DISABLE);
	//9.3)-------------------------------------清除DMA标志                                 					
	DMA_Clear_Tx_Flag(USARTx);	// 清除串口DMA方式发送中断全局标志
	DMA_Clear_Rx_Flag(USARTx);	// 清除串口DMA方式接收中断全局标志	
}
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
static void usart_it_initialize(USART_TypeDef* USARTx)	//串口GPIO配置
{
	//1)**********定义变量
  NVIC_InitTypeDef 	NVIC_InitStructure; 					//NVIC结构体
  u8 USARTx_IRQChannel=0;

	switch(*(u32*)&USARTx)
	{
		case 	USART1_BASE:
					USARTx_IRQChannel=USART1_IRQChannel;		//中断			
					break;
		case 	USART2_BASE:
          USARTx_IRQChannel=USART2_IRQChannel;		//中断	
					break;
		case 	USART3_BASE:
          USARTx_IRQChannel=USART3_IRQChannel;		//中断					break;
		case 	UART4_BASE:
          USARTx_IRQChannel=UART4_IRQChannel;		//中断
					break;
		case 	UART5_BASE:   //UART5不支持DMA
          USARTx_IRQChannel=UART5_IRQChannel;		//中断	
					break;
		default :break;
	}
	//4)**********串口全局中断初始化
	NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     //默认1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;              //默认1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------
//---------------------------------static2-------------------------------------
//-----------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	
*功能描述		:	串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
*输入				: 
*返回值			:	如果数据已经传入到DMA，返回Buffer大小，否则返回0（发送器忙）
*******************************************************************************/
static unsigned char set_usart_type(USART_TypeDef* USARTx,unsigned char type)
{
	//串口类型0---USART,1-RS485
	if(USART1==USARTx)
		usart_type.nUSART1	=	type;
	if(USART2==USARTx)
		usart_type.nUSART2	=	type;
	if(USART3==USARTx)
		usart_type.nUSART3	=	type;
	if(UART4==USARTx)
		usart_type.nUART4	=	type;
	if(UART5==USARTx)
		usart_type.nUART5	=	type;
	return 0;
}
/*******************************************************************************
*函数名			:	
*功能描述		:	串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
*输入				: 
*返回值			:	如果数据已经传入到DMA，返回Buffer大小，否则返回0（发送器忙）
*******************************************************************************/
static unsigned char get_usart_type(USART_TypeDef* USARTx)
{
	//串口类型0---USART,1-RS485
	if(USART1==USARTx)
		return usart_type.nUSART1;
	if(USART2==USARTx)
		return usart_type.nUSART2;
	if(USART3==USARTx)
		return usart_type.nUSART3;
	if(UART4==USARTx)
		return usart_type.nUART4;
	if(UART5==USARTx)
		return usart_type.nUART5;
	return 0;
}
/*******************************************************************************
*函数名			:	
*功能描述		:	串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
*输入				: 
*返回值			:	如果数据已经传入到DMA，返回Buffer大小，否则返回0（发送器忙）
*******************************************************************************/
static void set_rs485_addr(RS485Def *pRS485)		//串口DMA发送程序
{
	USART_TypeDef* USARTx	=	pRS485->USARTx;
	if(USART1==USARTx)
		rs485_addr.rs485_usart1	=	pRS485;
	if(USART2==USARTx)
		rs485_addr.rs485_usart2	=	pRS485;
	if(USART3==USARTx)
		rs485_addr.rs485_usart3	=	pRS485;
	if(UART4==USARTx)
		rs485_addr.rs485_uart4	=	pRS485;
	if(UART5==USARTx)
		rs485_addr.rs485_uart5	=	pRS485;
}
/*******************************************************************************
*函数名			:	
*功能描述		:	串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
*输入				: 
*返回值			:	如果数据已经传入到DMA，返回Buffer大小，否则返回0（发送器忙）
*******************************************************************************/
static RS485Def* get_rs485_addr(USART_TypeDef* USARTx)		//串口DMA发送程序
{
	if(0==get_usart_type(USARTx))
		return 0;
	if(USART1==USARTx)
		return rs485_addr.rs485_usart1;
	if(USART2==USARTx)
		return rs485_addr.rs485_usart2;
	if(USART3==USARTx)
		return rs485_addr.rs485_usart3;
	if(UART4==USARTx)
		return rs485_addr.rs485_uart4;
	if(UART5==USARTx)
		return rs485_addr.rs485_uart5;
	return 0;
}
/*******************************************************************************
*函数名			:	
*功能描述		:	串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
*输入				: 
*返回值			:	如果数据已经传入到DMA，返回Buffer大小，否则返回0（发送器忙）
*******************************************************************************/
static unsigned short set_usart_tx_dma_buffer(USART_TypeDef* USARTx,u8 *tx_buffer,u16 BufferSize)		//串口DMA发送程序
{
	unsigned short conf_buffer_size	=	get_usart_dma_buffer_size(USARTx);
	unsigned char* tx_addr	=	get_usart_tx_data_addr(USARTx);
	
	DMA_Channel_TypeDef*	DMAy_Channelx	=	get_usart_tx_dma_channel(USARTx);
	
	if(BufferSize>conf_buffer_size)
		return 0;		
	memcpy(tx_addr,tx_buffer,BufferSize);
	
	DMAy_Channelx->CCR    &= (u32)0xFFFFFFFE;				//DMA_Cmd(DMA1_Channel4,DISABLE);//DMA发送关闭，只能在DMA关闭情况下才可以写入CNDTR					
	DMAy_Channelx->CNDTR 	=   BufferSize;					  //设定待发送缓冲区大小
	DMAy_Channelx->CMAR 	=   (u32)tx_addr;			  	//发送缓冲区
	DMAy_Channelx->CCR    |=  (u32)0x00000001;			//DMA_Cmd(DMA1_Channel4,ENABLE);//DMA发送开启3
	return BufferSize;
}
/*******************************************************************************
*函数名			:	
*功能描述		:	串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
*输入				: 
*返回值			:	如果数据已经传入到DMA，返回Buffer大小，否则返回0（发送器忙）
*******************************************************************************/
static unsigned short get_usart_rx_dma_buffer(USART_TypeDef* USARTx,u8 *RevBuffer)		//串口DMA接收程序
{
	unsigned short length=0;
	unsigned char*	uart_rxd;
	unsigned short 	dma_size;
	DMA_Channel_TypeDef* DMAy_Channelrx;			//DMA接收通道请求信号---DMA串口接收由串口发起中断，因此此处接收通道中断不使用
	
	if(UART5==USARTx)		//UART5不支持DMA
		return 0;
	if(0==uart_rxd)
		return 0;
//	if(0==get_usart_rx_idle(USARTx))
//		return 0;
//	del_usart_rx_idle(USARTx);
//	if(USART_GetFlagStatus(USARTx,USART_FLAG_IDLE))
//	{	
		uart_rxd	=	get_usart_rx_data_addr(USARTx);
		dma_size	=	get_usart_dma_buffer_size(USARTx);

		DMAy_Channelrx=get_usart_rx_dma_channel(USARTx);
		length 	= DMAy_Channelrx->CNDTR;										//DMA_GetCurrDataCounter(DMA1_Channel5);	//得到真正接收数据个数(DMA_GetCurrDataCounter返回当前DMA通道x剩余的待传输数据数目)
		if(length>dma_size)	//超出缓存，避免内存溢出
			return 0;
		length	=	dma_size-length;												 	//设定缓冲区大小减剩余缓冲区大小得到实际接收到的数据个数
		if(0==length)
			return 0;
		DMAy_Channelrx->CCR&= (u32)0xFFFFFFFE;							//DMA_Cmd(DMAx_Channelx,DISABLE);//DMA接收关闭，只能在DMA关闭情况下才可以写入CNDTR
		memcpy(RevBuffer,uart_rxd,length);									//复制指定大小的数据
		//------重新指向接收缓冲区地址并使能DMA接收			
		DMAy_Channelrx->CMAR=(u32)uart_rxd;									//重新设置DMA接收地址
		DMAy_Channelrx->CNDTR=dma_size;			  							//重新设置接收数据个数	
		DMAy_Channelrx->CCR |= ((u32)0x00000001);						//DMA_Cmd(DMAx_Channelrx,ENABLE);//开启接收DMA
		
		USARTx->SR = (u16)~USART_FLAG_IDLE;									//USART_ClearFlag(USARTx,USART_FLAG_IDLE);
		
		set_rx_retry_count(USARTx,0);
		set_usart_rx_capacity_backup(USARTx,dma_size);			//缓存容量
		return length;			//返回接收到的数据个数
//	}

}
//-----------------------------------------------------------------------------



/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char* get_usart_rx_data_addr(USART_TypeDef* USARTx)
{
	if(USART1	==	USARTx)
	{
		return uRx1Addr;
	}
	else if(USART2	==	USARTx)
	{
		return uRx2Addr;
	}
	else if(USART3	==	USARTx)
	{
		return uRx3Addr;
	}
	else if(UART4	==	USARTx)
	{
		return uRx4Addr;		
	}
	return 0;
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
static unsigned char* get_usart_tx_data_addr(USART_TypeDef* USARTx)
{
	if(USART1	==	USARTx)
	{
		return uTx1Addr;
	}
	else if(USART2	==	USARTx)
	{
		return uTx2Addr;
	}
	else if(USART3	==	USARTx)
	{
		return uTx3Addr;
	}
	else if(UART4	==	USARTx)
	{
		return uTx4Addr;		
	}
	return 0;
}
//-----------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned short set_usart_dma_buffer_size(USART_TypeDef* USARTx,unsigned short size)
{
	if(0==size)													//如果未设定缓存大小，使用默认值
		size	=	uart_dma_buffer_size;
	else if(size>uart_dma_buffer_size)	//大于默认缓存
		size	=	uart_dma_buffer_size;
	if(USART1	==	USARTx)
	{
		DmaSize.nUSART1	=	size;
	}
	else if(USART2	==	USARTx)
	{
		DmaSize.nUSART2	=	size;
	}
	else if(USART3	==	USARTx)
	{
		DmaSize.nUSART3	=	size;
	}
	else if(UART4	==	USARTx)
	{
		DmaSize.nUART4	=	size;
	}
	else
		return 0;
	return size;
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
static unsigned short get_usart_dma_buffer_size(USART_TypeDef* USARTx)
{
	if(USART1	==	USARTx)
	{
		return DmaSize.nUSART1;
	}
	else if(USART2	==	USARTx)
	{
		return DmaSize.nUSART2;
	}
	else if(USART3	==	USARTx)
	{
		return DmaSize.nUSART3;
	}
	else if(UART4	==	USARTx)
	{
		return DmaSize.nUART4;
	}
	return 0;
}
//-----------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned short set_usart_rx_capacity_backup(USART_TypeDef* USARTx,unsigned short size)
{
	if(size>uart_dma_buffer_size)	//大于配置容量
		size	=	uart_dma_buffer_size;
	if(USART1	==	USARTx)
	{
		RemaDmaSize.nUSART1	=	size;
	}
	else if(USART2	==	USARTx)
	{
		RemaDmaSize.nUSART2	=	size;

	}
	else if(USART3	==	USARTx)
	{
		RemaDmaSize.nUSART3	=	size;
	}
	else if(UART4	==	USARTx)
	{
		RemaDmaSize.nUART4	=	size;
	}
	else
		return 0;
	return size;
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
static unsigned short get_usart_rx_capacity_backup(USART_TypeDef* USARTx)
{
	if(USART1	==	USARTx)
	{
		return RemaDmaSize.nUSART1;
	}
	else if(USART2	==	USARTx)
	{
		return RemaDmaSize.nUSART2;
	}
	else if(USART3	==	USARTx)
	{
		return RemaDmaSize.nUSART3;
	}
	else if(UART4	==	USARTx)
	{
		return RemaDmaSize.nUART4;
	}
	return 0;
}
//-----------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned short set_tx_retry_count(USART_TypeDef* USARTx,unsigned short count)
{
	if(USART1	==	USARTx)
	{
		tx_retry_count.nUSART1	=	count;
	}
	else if(USART2	==	USARTx)
	{
		tx_retry_count.nUSART2	=	count;
	}
	else if(USART3	==	USARTx)
	{
		tx_retry_count.nUSART3	=	count;
	}
	else if(UART4	==	USARTx)
	{
		tx_retry_count.nUART4	=	count;
	}
	return count;
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
static unsigned short get_tx_retry_count(USART_TypeDef* USARTx)
{
	if(USART1	==	USARTx)
	{
		return tx_retry_count.nUSART1;
	}
	else if(USART2	==	USARTx)
	{
		return tx_retry_count.nUSART2;
	}
	else if(USART3	==	USARTx)
	{
		return tx_retry_count.nUSART3;
	}
	else if(UART4	==	USARTx)
	{
		return tx_retry_count.nUART4;
	}
	return 0;
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
static unsigned short set_rx_retry_count(USART_TypeDef* USARTx,unsigned short count)
{
	if(USART1	==	USARTx)
	{
		rx_retry_count.nUSART1	=	count;
	}
	else if(USART2	==	USARTx)
	{
		rx_retry_count.nUSART2	=	count;
	}
	else if(USART3	==	USARTx)
	{
		rx_retry_count.nUSART3	=	count;
	}
	else if(UART4	==	USARTx)
	{
		rx_retry_count.nUART4	=	count;
	}
	return count;
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
static unsigned short get_rx_retry_count(USART_TypeDef* USARTx)
{
	if(USART1	==	USARTx)
	{
		return rx_retry_count.nUSART1;
	}
	else if(USART2	==	USARTx)
	{
		return rx_retry_count.nUSART2;
	}
	else if(USART3	==	USARTx)
	{
		return rx_retry_count.nUSART3;
	}
	else if(UART4	==	USARTx)
	{
		return rx_retry_count.nUART4;
	}
	return 0;
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
static DMA_Channel_TypeDef* get_usart_tx_dma_channel(USART_TypeDef* USARTx)
{
	if(USART1	==	USARTx)
	{
		return DMA1_Channel4;
	}
	else if(USART2	==	USARTx)
	{
		return DMA1_Channel7;
	}
	else if(USART3	==	USARTx)
	{
		return DMA1_Channel2;
	}
	else if(UART4	==	USARTx)
	{
		return DMA2_Channel5;
	}
	return 0;	
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
static DMA_Channel_TypeDef* get_usart_rx_dma_channel(USART_TypeDef* USARTx)
{
	if(USART1	==	USARTx)
	{
		return DMA1_Channel5;
	}
	else if(USART2	==	USARTx)
	{
		return DMA1_Channel6;
	}
	else if(USART3	==	USARTx)
	{
		return DMA1_Channel3;
	}
	else if(UART4	==	USARTx)
	{
		return DMA2_Channel3;
	}
	return 0;	
}
/*******************************************************************************
*函数名			:	DMA_Clear_Tx_Flag
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void DMA_Clear_Tx_Flag(USART_TypeDef* USARTx)
{
	if(USART1	==	USARTx)
	{
		DMA_ClearFlag(DMA1_FLAG_GL4);                                 					// 清除DMA所有标志
	}
	else if(USART2	==	USARTx)
	{
		DMA_ClearFlag(DMA1_FLAG_GL7);                                 					// 清除DMA所有标志
	}
	else if(USART3	==	USARTx)
	{
		DMA_ClearFlag(DMA1_FLAG_GL2);                                 					// 清除DMA所有标志
	}
	else if(UART4	==	USARTx)
	{
		DMA_ClearFlag(DMA2_FLAG_GL5);                                 					// 清除DMA所有标志
	}
}
/*******************************************************************************
*函数名			:	DMA_Clear_Rx_Flag
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void DMA_Clear_Rx_Flag(USART_TypeDef* USARTx)
{
	if(USART1	==	USARTx)
	{
		DMA_ClearFlag(DMA1_FLAG_GL5);                                 					// 清除DMA所有标志
	}
	else if(USART2	==	USARTx)
	{
		DMA_ClearFlag(DMA1_FLAG_GL6);                                 					// 清除DMA所有标志
	}
	else if(USART3	==	USARTx)
	{
		DMA_ClearFlag(DMA1_FLAG_GL3);                                 					// 清除DMA所有标志
	}
	else if(UART4	==	USARTx)
	{
		DMA_ClearFlag(DMA2_FLAG_GL3);                                 					// 清除DMA所有标志
	}
}
//------------------------------------------------------------------------------



/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	0--非空闲，1--空闲
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char set_usart_rx_idle(USART_TypeDef* USARTx)
{	
	//空闲标志0---忙,1-空闲
	if(USART1	==	USARTx)
		usart_rx_idle_flag.nUSART1	=	1;
	if(USART2	==	USARTx)
		usart_rx_idle_flag.nUSART2	=	1;
	if(USART3	==	USARTx)
		usart_rx_idle_flag.nUSART3	=	1;
	if(UART4	==	USARTx)
		usart_rx_idle_flag.nUART4	=	1;
	if(UART5	==	USARTx)
		usart_rx_idle_flag.nUART5	=	1;
	return 0;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	0--非空闲，1--空闲
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char set_usart_tx_idle(USART_TypeDef* USARTx)
{
	//空闲标志0---忙,1-空闲
	if(USART1	==	USARTx)
		usart_tx_idle_flag.nUSART1	=	1;
	if(USART2	==	USARTx)
		usart_tx_idle_flag.nUSART2	=	1;
	if(USART3	==	USARTx)
		usart_tx_idle_flag.nUSART3	=	1;
	if(UART4	==	USARTx)
		usart_tx_idle_flag.nUART4	=	1;
	if(UART5	==	USARTx)
		usart_tx_idle_flag.nUART5	=	1;
	return 0;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	0--非空闲，1--空闲
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char get_usart_rx_idlebac(USART_TypeDef* USARTx)
{	
	//空闲标志0---忙,1-空闲
	if(USART1	==	USARTx)
	{
		if(USART_GetFlagStatus(USART1,USART_FLAG_IDLE))
			set_usart_rx_idle(USART1);
		return usart_rx_idle_flag.nUSART1;
	}
	if(USART2	==	USARTx)
	{
		if(USART_GetFlagStatus(USART2,USART_FLAG_IDLE))
			set_usart_rx_idle(USART2);
		return usart_rx_idle_flag.nUSART2;
	}
	if(USART3	==	USARTx)
	{
		if(USART_GetFlagStatus(USART3,USART_FLAG_IDLE))
			set_usart_rx_idle(USART3);
		return usart_rx_idle_flag.nUSART3;
	}
	if(UART4	==	USARTx)
	{
		if(USART_GetFlagStatus(UART4,USART_FLAG_IDLE))
			set_usart_rx_idle(UART4);
		return usart_rx_idle_flag.nUART4;
	}
	if(UART5	==	USARTx)
	{
		if(USART_GetFlagStatus(UART5,USART_FLAG_IDLE))
			set_usart_rx_idle(UART5);
		return usart_rx_idle_flag.nUART5;
	}
	return 0;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	0--非空闲，1--空闲
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char get_usart_tx_idlebac(USART_TypeDef* USARTx)
{
	//空闲标志0---忙,1-空闲
	if(USART1	==	USARTx)
	{
		if(USART_GetFlagStatus(USART1,USART_FLAG_TC))
		{
			set_usart_tx_idle(USART1);
			USART_ClearFlag(USART1,USART_FLAG_TC);
		}
		return usart_tx_idle_flag.nUSART1;	
	}		
	if(USART2	==	USARTx)	
	{
		if(USART_GetFlagStatus(USART2,USART_FLAG_TC))
		{
			set_usart_tx_idle(USART2);
			USART_ClearFlag(USART2,USART_FLAG_TC);
		}			
		return usart_tx_idle_flag.nUSART2;
	}
	if(USART3	==	USARTx)	
	{
		if(USART_GetFlagStatus(USART3,USART_FLAG_TC))
		{
			set_usart_tx_idle(USART3);
			USART_ClearFlag(USART3,USART_FLAG_TC);
		}
		return usart_tx_idle_flag.nUSART3;
	}
	if(UART4	==	USARTx)
	{
		if(USART_GetFlagStatus(UART4,USART_FLAG_TC))
		{
			set_usart_tx_idle(UART4);
			USART_ClearFlag(UART4,USART_FLAG_TC);
		}
		return usart_tx_idle_flag.nUART4;
	}
	if(UART5	==	USARTx)
	{
		if(USART_GetFlagStatus(UART5,USART_FLAG_TC))
		{
			set_usart_tx_idle(UART5);
			USART_ClearFlag(UART5,USART_FLAG_TC);
		}
		return usart_tx_idle_flag.nUART5;
	}
	return 0;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	0--非空闲，1--空闲
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char del_usart_rx_idle(USART_TypeDef* USARTx)
{	
	//空闲标志0---忙,1-空闲
	if(USART1	==	USARTx)
		usart_rx_idle_flag.nUSART1	=	0;
	if(USART2	==	USARTx)
		usart_rx_idle_flag.nUSART2	=	0;
	if(USART3	==	USARTx)
		usart_rx_idle_flag.nUSART3	=	0;
	if(UART4	==	USARTx)
		usart_rx_idle_flag.nUART4		=	0;
	if(UART5	==	USARTx)
		usart_rx_idle_flag.nUART5		=	0;
	return 0;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	0--非空闲，1--空闲
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char del_usart_tx_idle(USART_TypeDef* USARTx)
{
	//空闲标志0---忙,1-空闲
	if(USART1	==	USARTx)
		usart_tx_idle_flag.nUSART1	=	0;
	if(USART2	==	USARTx)
		usart_tx_idle_flag.nUSART2	=	0;
	if(USART3	==	USARTx)
		usart_tx_idle_flag.nUSART3	=	0;
	if(UART4	==	USARTx)
		usart_tx_idle_flag.nUART4	=	0;
	if(UART5	==	USARTx)
		usart_tx_idle_flag.nUART5	=	0;
	return 0;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	0--非空闲，1--空闲
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char get_usart_rx_idle(USART_TypeDef* USARTx)
{	
	//-----------------接收空闲检测：IDLE空闲标志,DMA缓存满，DMA缓存未占用，DMA缓存占用后没变化，DMA未开启
	unsigned short	capacity	=	0;			//剩余缓存大小
	
	
	unsigned short	dma_conf_size							=	get_usart_dma_buffer_size(USARTx);	
	DMA_Channel_TypeDef*	DMAy_Channelx				=	get_usart_rx_dma_channel(USARTx);	
	
	capacity	=	DMAy_Channelx->CNDTR;		      //获取DMA接收缓存剩余空间
	//-----------------空闲标志
	if(USART_GetFlagStatus(USARTx,USART_FLAG_IDLE))		//检测发送数据寄存器是否为空	RESET-非空，SET-空，
	{
		USART_ClearFlag(USARTx,USART_FLAG_IDLE);
		return 1;			//空闲
	}
	//-----------------DMA缓存满
	if(capacity==0)
	{
		return 1;
	}
	//-----------------DMA缓存未占用
	if(capacity==dma_conf_size)
	{
		return 1;
	}
	//-----------------DMA缓存占用后没变化
	if(capacity<dma_conf_size)	//缓存已使用：检查是否还在使用
	{	
		unsigned short	dma_conf_capacity_backup	=	get_usart_rx_capacity_backup(USARTx);			//剩余缓存大小
		unsigned short	rxdelay		=	get_rx_retry_count(USARTx);
		//---------------缓存无变化
		if(capacity	==	dma_conf_capacity_backup)
		{
			if(rxdelay<=2)
			{
				rxdelay++;
				set_rx_retry_count(USARTx,rxdelay);
			}
			else
			{
				return 1;
			}
		}
		else	//数量有变化
		{
			set_rx_retry_count(USARTx,0);
			set_usart_rx_capacity_backup(USARTx,capacity);
			return 0;
		}
	}
	return 0;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	0--非空闲，1--空闲
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned char get_usart_tx_idle(USART_TypeDef* USARTx)
{
	//-----------------发送空闲检测：TC发送完成标志,TXE缓存空,，DMA缓存空，DMA未开启
	unsigned short	txdelay		=	get_tx_retry_count(USARTx);

	DMA_Channel_TypeDef*	DMAy_Channelx	=	get_usart_tx_dma_channel(USARTx);	
	//-----------------TC发送完成标志
	if(USART_GetFlagStatus(USARTx,USART_FLAG_TC))		//检测发送数据寄存器是否为空	RESET-非空，SET-空
	{
		set_tx_retry_count(USARTx,0);
		DMA_Cmd(DMAy_Channelx,DISABLE);
		USART_ClearFlag(USARTx,USART_FLAG_TC);
		USART_ClearFlag(USARTx,USART_FLAG_TXE);
		return 1;				//发送空闲
	}
	//------------------DMA未开启
	if(0	==	(DMAy_Channelx->CCR&0x00000001))	//dma未开
	{
		return 1;				//发送空闲
	}
	//------------------DMA缓存空，TXE缓存空
	if((0==DMAy_Channelx->CNDTR)&&(USART_GetFlagStatus(USARTx,USART_FLAG_TXE)))
	{	
		if(txdelay<2)
		{
			txdelay+=1;
			set_tx_retry_count(USARTx,txdelay);
			return 0;				//发送空闲
		}
		else
		{
			DMA_Cmd(DMAy_Channelx,DISABLE);
			USART_ClearFlag(USARTx,USART_FLAG_TXE);
			set_tx_retry_count(USARTx,0);
			return 1;				//发送空闲
		}
	}
	return 0;
}
//----------------------------------------------------------------



/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void set_rs485_tx(RS485Def *pRS485,FunctionalState NewState)
{
	//----------------txen脚高为发送
	//----------------rxen脚低为接收
	if(pRS485->RS485_TxEn_PORT)
	{
		if (NewState != DISABLE)
		{
			pRS485->RS485_TxEn_PORT->BSRR		= pRS485->RS485_TxEn_Pin;
		}
		else
		{
			pRS485->RS485_TxEn_PORT->BRR 		= pRS485->RS485_TxEn_Pin;
		}
	}
	if(pRS485->RS485_CTL_PORT)
	{
		if (NewState != DISABLE)
		{
			pRS485->RS485_CTL_PORT->BSRR		= pRS485->RS485_CTL_Pin;
		}
		else
		{
			pRS485->RS485_CTL_PORT->BRR 		= pRS485->RS485_CTL_Pin;
		}
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
static void set_rs485_rx(RS485Def *pRS485,FunctionalState NewState)
{
	//----------------txen脚高为发送
	//----------------rxen脚低为接收
	if(pRS485->RS485_RxEn_PORT)
	{
		if (NewState != DISABLE)
		{
			pRS485->RS485_RxEn_PORT->BRR 		= pRS485->RS485_RxEn_Pin;			
		}
		else
		{
			pRS485->RS485_RxEn_PORT->BSRR		= pRS485->RS485_RxEn_Pin;
		}
	}
	if(pRS485->RS485_CTL_PORT)
	{
		if (NewState != DISABLE)
		{
			pRS485->RS485_CTL_PORT->BRR 		= pRS485->RS485_CTL_Pin;			
		}
		else
		{
			pRS485->RS485_CTL_PORT->BSRR		= pRS485->RS485_CTL_Pin;
		}
	}   
}
//----------------------------------------------------------------






/*******************************************************************************
*函数名			:	USART_ConfigurationIT
*功能描述		:	USART_配置---常规中断方式
*输入				: 
*返回值			:	无
*修改时间		:	2018/01/06
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void	USART_Send(USART_TypeDef* USARTx,u8* TxdBuffer,u16 Lengh)
{
	u8 Temp	=	0;
	u16	Len	=	0;
	while(Len<Lengh)
	{
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE)!=SET)
		{			
		}
		Temp=*TxdBuffer;
		USART_SendData(USARTx, Temp);
		TxdBuffer++;
		Len++;
	}
}
/*******************************************************************************
*函数名			:	USART_ConfigurationIT
*功能描述		:	USART_配置---常规中断方式
*输入				: 
*返回值			:	无
*修改时间		:	2018/01/06
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void	USART_TxServer(USART_TypeDef* USARTx)
{
}
/*******************************************************************************
*函数名		:USART_RX_Server
*功能描述	:串口接收服务函数
*输入			: 
*输出			:无
*返回值		:无
*例程			：
*******************************************************************************/

void USART_RxServer(USART_TypeDef* USARTx)		//串口接收服务程序	
{

}
/*******************************************************************************
*函数名		:USART_RX_Server
*功能描述	:串口接收服务函数
*输入			: 
*输出			:无
*返回值		:无
*例程			：
*******************************************************************************/

void USART_Process(void)		//串口服务程序	
{
  USART_TxServer(USART1);
  USART_TxServer(USART2);
  USART_TxServer(USART3);
  USART_TxServer(UART4);
  USART_TxServer(UART5);
}


/*******************************************************************************
* 函数名		:	
* 功能描述	:	printf重定义 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
int fputc(int ch, FILE *f)	//printf重定义
{

	USART_SendData(USART1, (unsigned char) ch);// USART1 可以换成 USART2 等

	while(!(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == SET));

	return (ch);

}

  
/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
//int fgetc(FILE *f)
//{
// 
//	while(!(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET))
//	{
//	}
//	return (USART_ReceiveData(USART1));
//}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	
*******************************************************************************/
unsigned char UART5ReceiveData(unsigned char* RecByte)			//串口5读数据,返回0-无中断，返回1有中断无数据（发送中断），返回2-有中断有接收到数据
{
	//==============================Port5
	if(USART_GetITStatus(UART5, USART_IT_RXNE))
  {
		*RecByte	=	USART_ReceiveData(UART5);
	
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
		return 2;
	}
	else if(USART_GetITStatus(UART5, USART_IT_TC))
  {   
    USART_ClearITPendingBit(UART5, USART_IT_TC);
		return 1;
  }
	return 0;
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------





//-----------------------------------------------------------------------------串口收发
/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_IRQHandler(void)
{
	//--------------------------------------------------------发送中断
	if(USART_GetITStatus(USART1,USART_IT_TC))
	{
		USART_ClearITPendingBit(USART1,USART_IT_TC);
		if(get_usart_type(USART1))		//串口类型：0---USART,1-RS485
		{
			RS485Def*	pRS485	=	get_rs485_addr(USART1);
			if(0==pRS485)
				return;
			set_rs485_tx(pRS485,DISABLE);
			set_rs485_rx(pRS485,ENABLE);
			set_usart_tx_idle(USART1);
		}
	}
	//--------------------------------------------------------接收空闲中断
	if(USART_GetITStatus(USART1,USART_IT_IDLE))
	{
		USART_ClearITPendingBit(USART1,USART_IT_IDLE);
		if(get_usart_type(USART1))		//串口类型：0---USART,1-RS485
		{
			RS485Def*	pRS485	=	get_rs485_addr(USART1);
			if(0==pRS485)
				return;
			set_usart_rx_idle(USART1);
		}
	}	
}
/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : This function handles USART2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART2_IRQHandler(void)
{
//--------------------------------------------------------发送中断
	if(USART_GetITStatus(USART2,USART_IT_TC))
	{
		USART_ClearITPendingBit(USART2,USART_IT_TC);
		if(get_usart_type(USART2))		//串口类型：0---USART,1-RS485
		{
			RS485Def*	pRS485	=	get_rs485_addr(USART2);
			if(0==pRS485)
				return;
			set_rs485_tx(pRS485,DISABLE);
			set_rs485_rx(pRS485,ENABLE);			
		}
		set_usart_tx_idle(USART2);
	}
	//--------------------------------------------------------接收空闲中断
	if(USART_GetITStatus(USART2,USART_IT_IDLE))
	{
		USART_ClearITPendingBit(USART2,USART_IT_IDLE);
		if(get_usart_type(USART2))		//串口类型：0---USART,1-RS485
		{
			RS485Def*	pRS485	=	get_rs485_addr(USART2);
			if(0==pRS485)
				return;
		}
		set_usart_rx_idle(USART2);
	}
}







