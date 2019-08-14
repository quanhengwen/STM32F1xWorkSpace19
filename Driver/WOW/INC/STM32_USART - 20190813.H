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


typedef struct
{
	//USART:485用到的串口;串口号--USART1,USART2,USART3,UART4;//UART5不支持DMA
	USART_TypeDef* 	USARTx;			//USART1,USART2,USART3,UART4;//UART5
	//RS485_CTL
	GPIO_TypeDef* 	RS485_CTL_PORT;				//GPIOX
	unsigned short 	RS485_CTL_Pin;				//GPIO_Pin_x
	
	GPIO_TypeDef* 	RS485_TxEn_PORT;			//GPIOX
	unsigned short 	RS485_TxEn_Pin;				//GPIO_Pin_x
	
	GPIO_TypeDef* 	RS485_RxEn_PORT;			//GPIOX
	unsigned short 	RS485_RxEn_Pin;				//GPIO_Pin_x
}RS485Def;

//------------------------------------------------------API-1
unsigned short api_usart_dma_receive(USART_TypeDef* USARTx,unsigned char *RevBuffer);
unsigned short api_rs485_dma_receive(RS485Def *pRS485,unsigned char *RevBuffer);


unsigned short api_usart_dma_send(USART_TypeDef* USARTx,unsigned char *tx_buffer,unsigned short BufferSize);
unsigned short api_rs485_dma_send(RS485Def *pRS485,unsigned char *tx_buffer,unsigned short BufferSize);

unsigned short api_usart_dma_printf(USART_TypeDef* USARTx,const char *format,...);
unsigned short api_rs485_dma_printf(RS485Def *pRS485,const char *format,...);
//------------------------------------------------------


//------------------------------------------------------API-configuration
void api_usart_dma_configurationST(USART_TypeDef* USARTx,USART_InitTypeDef* USART_InitStructure,unsigned short BufferSize);	//USART_DMA配置--结构体形式，不开中断
	
void api_usart_dma_configurationNR(USART_TypeDef* USARTx,u32 USART_BaudRate,unsigned short BufferSize);

void api_rs485_dma_configurationNR(RS485Def *pRS485,u32 USART_BaudRate,unsigned short BufferSize);

void api_usart_dma_configuration_even(USART_TypeDef* USARTx,u32 USART_BaudRate,unsigned short BufferSize);
void api_usart_dma_configuration_odd(USART_TypeDef* USARTx,u32 USART_BaudRate,unsigned short BufferSize);
void api_usart_dma_configuration_idleIT(USART_TypeDef* USARTx,u32 USART_BaudRate,unsigned short BufferSize);
void api_usart_dma_configurationNR_Remap(USART_TypeDef* USARTx,u32 USART_BaudRate,unsigned short BufferSize);
void api_rs485_dma_configurationNR_Remap(RS485Def *pRS485,u32 USART_BaudRate,unsigned short BufferSize);
void api_rs485_dma_configurationIT(RS485Def *pRS485,u32 USART_BaudRate,unsigned short BufferSize);
void api_usart_configuration_it(USART_TypeDef* USARTx,u32 USART_BaudRate);


//------------------------------------------------------static1
static void usart_rcc_initialize(USART_TypeDef* USARTx);
static void	usart_gpio_initialize(USART_TypeDef* USARTx);
static void	usart_gpio_initialize_Remap(USART_TypeDef* USARTx);

static void usart_initialize_st(USART_TypeDef* USARTx,USART_InitTypeDef* USART_InitStructure);	//USART标准配置

static void usart_initialize_nr(USART_TypeDef* USARTx,unsigned long USART_BaudRate);
static void usart_initialize_even(USART_TypeDef* USARTx,unsigned long USART_BaudRate);	//偶校验
static void usart_initialize_odd(USART_TypeDef* USARTx,unsigned long USART_BaudRate);		//奇校验
static void	usart_dma_initialize(USART_TypeDef* USARTx,unsigned short BufferSize);
static void usart_it_initialize(USART_TypeDef* USARTx);	//串口GPIO配置


//------------------------------------------------------static2
static unsigned char set_usart_type(USART_TypeDef* USARTx,unsigned char type);		//设置串口类型：0---USART,1-RS485
static unsigned char get_usart_type(USART_TypeDef* USARTx);

static void set_rs485_addr(RS485Def *pRS485);
static RS485Def* get_rs485_addr(USART_TypeDef* USARTx);

static unsigned short set_usart_tx_dma_buffer(USART_TypeDef* USARTx,unsigned char *tx_buffer,unsigned short BufferSize);
static unsigned short get_usart_rx_dma_buffer(USART_TypeDef* USARTx,unsigned char *RevBuffer);

static unsigned char* get_usart_rx_data_addr(USART_TypeDef* USARTx);
static unsigned char* get_usart_tx_data_addr(USART_TypeDef* USARTx);

static unsigned short set_usart_dma_buffer_size(USART_TypeDef* USARTx,unsigned short size);
static unsigned short get_usart_dma_buffer_size(USART_TypeDef* USARTx);

static unsigned short set_usart_rx_capacity_backup(USART_TypeDef* USARTx,unsigned short size);
static unsigned short get_usart_rx_capacity_backup(USART_TypeDef* USARTx);

static unsigned short set_tx_retry_count(USART_TypeDef* USARTx,unsigned short count);
static unsigned short get_tx_retry_count(USART_TypeDef* USARTx);
static unsigned short set_rx_retry_count(USART_TypeDef* USARTx,unsigned short count);
static unsigned short get_rx_retry_count(USART_TypeDef* USARTx);

static DMA_Channel_TypeDef* get_usart_tx_dma_channel(USART_TypeDef* USARTx);
static DMA_Channel_TypeDef* get_usart_rx_dma_channel(USART_TypeDef* USARTx);
static void DMA_Clear_Tx_Flag(USART_TypeDef* USARTx);
static void DMA_Clear_Rx_Flag(USART_TypeDef* USARTx);

static unsigned char set_usart_rx_idle(USART_TypeDef* USARTx);
static unsigned char set_usart_tx_idle(USART_TypeDef* USARTx);
static unsigned char get_usart_rx_idle(USART_TypeDef* USARTx);
unsigned char get_usart_tx_idle(USART_TypeDef* USARTx);
static unsigned char del_usart_rx_idle(USART_TypeDef* USARTx);
static unsigned char del_usart_tx_idle(USART_TypeDef* USARTx);

static void set_rs485_tx(RS485Def *pRS485,FunctionalState NewState);
static void set_rs485_rx(RS485Def *pRS485,FunctionalState NewState);




void	USART_Send(USART_TypeDef* USARTx,u8* RevBuffer,u16 Lengh);
unsigned char UART5ReceiveData(unsigned char* RecByte);			//串口5读数据,返回0-无中断，返回1有中断无数据（发送中断），返回2-有中断有接收到数据


void  USART_TxServer(USART_TypeDef* USARTx);
void  USART_RxServer(USART_TypeDef* USARTx);		//串口接收服务程序
void	USART_Process(void);		//串口服务程序	

#endif

