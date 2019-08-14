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


#ifndef __STM32_USART_H
#define __STM32_USART_H

//#include	"stdio.h"			//用于printf
//#include	"string.h"			//用于printf
//#include	"stdarg.h"			//用于获取不确定个数的参数
//#include	"stdlib.h"			//malloc动态申请内存空间

#include "stm32f10x_type.h"
#include "stm32f10x_usart.h"


#define	uart_dma_buffer_size	256	//默认串口DMA接收缓冲大小,如果配置时未输入缓存大小时使用的默认值
#define	uBaudRate	115200	//默认串口波特率

typedef struct
{
	//USART:485用到的串口;串口号--USART1,USART2,USART3,UART4;//UART5不支持DMA
	USART_TypeDef* 	USARTx;								//USART1,USART2,USART3,UART4;//UART5
	//RS485_CTL
	GPIO_TypeDef* 	RS485_CTL_PORT;				//GPIOX
	unsigned short 	RS485_CTL_Pin;				//GPIO_Pin_x
	
	GPIO_TypeDef* 	RS485_TxEn_PORT;			//GPIOX
	unsigned short 	RS485_TxEn_Pin;				//GPIO_Pin_x
	
	GPIO_TypeDef* 	RS485_RxEn_PORT;			//GPIOX
	unsigned short 	RS485_RxEn_Pin;				//GPIO_Pin_x
}RS485Def;
//------------------------------------------------------
typedef struct
{
	unsigned char tx_idle	:1;		//发送空闲标志	:0---空闲,1---忙，在发送中
	unsigned char rx_idle	:2;		//接收空闲标志	:0---空闲,1---接收完成，2---正在接收
	unsigned char type		:1;		//串口类型			:0---USART,1-RS485	
}usart_flag_def;
//------------------------------------------------------
typedef struct
{
	unsigned short	dma_size_set;		//配置DMA容量
	unsigned short	dma_size_free;	//剩余DMA容量
	unsigned char		Rxd[uart_dma_buffer_size];
	unsigned char		Txd[uart_dma_buffer_size];
	unsigned char		tx_retry;				//发送重试计数
	unsigned char		rx_retry;				//接收重试计数
	usart_flag_def	flag;
	RS485Def 				*pRS485;	
}usart_channel_def;
//------------------------------------------------------
typedef struct
{
	usart_channel_def	ch_usart1;
	usart_channel_def	ch_usart2;
	usart_channel_def	ch_usart3;
	usart_channel_def	ch_uart4;
	usart_channel_def	ch_uart5;
}usart_def;
//------------------------------------------------------



//==============================================================================API-receive
unsigned short api_usart_receive(USART_TypeDef* USARTx,unsigned char *RevBuffer);
unsigned short api_rs485_receive(RS485Def *pRS485,unsigned char *RevBuffer);

//==============================================================================API-send
unsigned short api_usart_send(USART_TypeDef* USARTx,unsigned char *tx_buffer,unsigned short BufferSize);
unsigned short api_rs485_send(RS485Def *pRS485,unsigned char *tx_buffer,unsigned short BufferSize);

//==============================================================================API-printf
unsigned short api_usart_printf(USART_TypeDef* USARTx,const char *format,...);
unsigned short api_rs485_printf(RS485Def *pRS485,const char *format,...);





//==============================================================================API-UART-configuration
void api_usart_configuration_NR(USART_TypeDef* USARTx,u32 USART_BaudRate,unsigned short BufferSize);
void api_usart_configuration_ST(USART_TypeDef* USARTx,USART_InitTypeDef* USART_InitStructure,unsigned short BufferSize);
void api_usart_configuration_even(USART_TypeDef* USARTx,u32 USART_BaudRate,unsigned short BufferSize);	//USART--偶校验
void api_usart_configuration_odd(USART_TypeDef* USARTx,u32 USART_BaudRate,unsigned short BufferSize);		//USART配置--奇校验
void api_usart_configuration_NRRemap(USART_TypeDef* USARTx,u32 USART_BaudRate,unsigned short BufferSize);	//USART_DMA配置--完全映射，不开中断

//==============================================================================API-RS485-configuration
void api_rs485_configuration_NR(RS485Def *pRS485,u32 USART_BaudRate,unsigned short BufferSize);
void api_rs485_configuration_NRRemap(RS485Def *pRS485,u32 USART_BaudRate,unsigned short BufferSize);









//==============================================================================static2
static unsigned char set_usart_type(USART_TypeDef* USARTx,unsigned char type);		//设置串口类型：0---USART,1-RS485
static unsigned char get_usart_type(USART_TypeDef* USARTx);

static void set_rs485_addr(RS485Def *pRS485);
static RS485Def* get_rs485_addr(USART_TypeDef* USARTx);

static unsigned short set_usart_tx_dma_buffer(USART_TypeDef* USARTx,unsigned char *tx_buffer,unsigned short BufferSize);
static unsigned short get_usart_rx_dma_buffer(USART_TypeDef* USARTx,unsigned char *RevBuffer);

static unsigned char* get_usart_rx_data_addr(USART_TypeDef* USARTx);
static unsigned char* get_usart_tx_data_addr(USART_TypeDef* USARTx);



static void set_usart_rx_capacity_backup(USART_TypeDef* USARTx,unsigned short size);
static unsigned short get_usart_rx_capacity_backup(USART_TypeDef* USARTx);

static void set_tx_retry_count(USART_TypeDef* USARTx,unsigned short count);
static unsigned short get_tx_retry_count(USART_TypeDef* USARTx);
static void set_rx_retry_count(USART_TypeDef* USARTx,unsigned short count);
static unsigned short get_rx_retry_count(USART_TypeDef* USARTx);

//==============================================================================static-status
static void set_usart_rx_idle(USART_TypeDef* USARTx);
static void set_usart_tx_idle(USART_TypeDef* USARTx);

static unsigned char get_usart_rx_idle(USART_TypeDef* USARTx);
unsigned char get_usart_tx_idle(USART_TypeDef* USARTx);

static void del_usart_rx_idle(USART_TypeDef* USARTx);
static void del_usart_tx_idle(USART_TypeDef* USARTx);






//==============================================================================
static void set_rs485_tx(RS485Def *pRS485,FunctionalState NewState);
static void set_rs485_rx(RS485Def *pRS485,FunctionalState NewState);







void	USART_Send(USART_TypeDef* USARTx,u8* RevBuffer,u16 Lengh);
unsigned char UART5ReceiveData(unsigned char* RecByte);			//串口5读数据,返回0-无中断，返回1有中断无数据（发送中断），返回2-有中断有接收到数据


void  USART_TxServer(USART_TypeDef* USARTx);
void  USART_RxServer(USART_TypeDef* USARTx);		//串口接收服务程序
void	USART_Process(void);		//串口服务程序	








//==============================================================================static-initialize
static void usart_rcc_initialize(USART_TypeDef* USARTx);
static void	usart_gpio_initialize(USART_TypeDef* USARTx);
static void	usart_gpio_initialize_Remap(USART_TypeDef* USARTx);

static void usart_initialize_st(USART_TypeDef* USARTx,USART_InitTypeDef* USART_InitStructure);	//USART标准配置
static void usart_initialize_nr(USART_TypeDef* USARTx,unsigned long USART_BaudRate);
static void usart_initialize_even(USART_TypeDef* USARTx,unsigned long USART_BaudRate);	//偶校验
static void usart_initialize_odd(USART_TypeDef* USARTx,unsigned long USART_BaudRate);		//奇校验
static void	usart_dma_initialize(USART_TypeDef* USARTx,unsigned short BufferSize);
static void usart_it_initialize(USART_TypeDef* USARTx);	//串口GPIO配置

//==============================================================================static-DMA
static DMA_Channel_TypeDef* get_usart_tx_dma_channel(USART_TypeDef* USARTx);
static DMA_Channel_TypeDef* get_usart_rx_dma_channel(USART_TypeDef* USARTx);
static void DMA_Clear_Tx_Flag(USART_TypeDef* USARTx);
static void DMA_Clear_Rx_Flag(USART_TypeDef* USARTx);
static unsigned short set_usart_dma_buffer_size(USART_TypeDef* USARTx,unsigned short size);
static unsigned short get_usart_dma_buffer_size(USART_TypeDef* USARTx);



#endif

