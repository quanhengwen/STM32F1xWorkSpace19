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
#define	uart_dma_buffer_size	256	//默认串口DMA接收缓冲大小,如果配置时未输入缓存大小时使用的默认值
#define	uBaudRate	115200	//默认串口波特率

static struct
{
  unsigned short nUSART1;
  unsigned short nUSART2;
  unsigned short nUSART3;
  unsigned short nUART4;
//  unsigned short nUART5;	  //----无DMA
}DmaSize;      //配置时设置的DMA接收缓存大小

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
//*****************RS485收发控制
void RS485_TX_EN(RS485Def *pRS485);	  //发使能
void RS485_RX_EN(RS485Def *pRS485);		//收使能，已经设置为接收状态返回1，否则返回0

static unsigned char* Get_Rxd_Addr(USART_TypeDef* USARTx);
static unsigned char* Get_Txd_Addr(USART_TypeDef* USARTx);
static unsigned short Set_Dma_InfoSize(USART_TypeDef* USARTx,unsigned short size);
static unsigned short Get_Dma_InfoSize(USART_TypeDef* USARTx);
static DMA_Channel_TypeDef* Get_Usart_Tx_DMA_Channel(USART_TypeDef* USARTx);
static DMA_Channel_TypeDef* Get_Usart_Rx_DMA_Channel(USART_TypeDef* USARTx);
static void ClearFlag_Usart_Tx_DMA_Channel(USART_TypeDef* USARTx);
static void ClearFlag_Usart_Rx_DMA_Channel(USART_TypeDef* USARTx);
static unsigned short Usart_Dma_Send(USART_TypeDef* USARTx,u8 *tx_buffer,u16 BufferSize);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
static void	Usart_Dma_Initialize	(USART_TypeDef* USARTx,u16 BufferSize);	//串口DMA配置---不开中断，查询方式
static void	USART_GPIO_Initialize(USART_TypeDef* USARTx);                 	//串口GPIO配置
//static unsigned char Get_Txd_Status(USART_TypeDef* USARTx);		//获取串口发送状态
//static unsigned char Get_Rxd_Status(USART_TypeDef* USARTx);		//获取串口接收状态

/* function-------------------------------------------------------------------*/
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char* Get_Rxd_Addr(USART_TypeDef* USARTx)
{
	if(USART1	==	USARTx)
	{
		static unsigned char uRx1Addr[uart_dma_buffer_size]={0};
		return uRx1Addr;
	}
	else if(USART2	==	USARTx)
	{
		static unsigned char uRx2Addr[uart_dma_buffer_size]={0};
		return uRx2Addr;
	}
	else if(USART3	==	USARTx)
	{
		static unsigned char uRx3Addr[uart_dma_buffer_size]={0};
		return uRx3Addr;
	}
	else if(UART4	==	USARTx)
	{
		static unsigned char uRx4Addr[uart_dma_buffer_size]={0};
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
static unsigned char* Get_Txd_Addr(USART_TypeDef* USARTx)
{
	if(USART1	==	USARTx)
	{
		static unsigned char uTx1Addr[uart_dma_buffer_size]={0};
		return uTx1Addr;
	}
	else if(USART2	==	USARTx)
	{
		static unsigned char uTx2Addr[uart_dma_buffer_size]={0};
		return uTx2Addr;
	}
	else if(USART3	==	USARTx)
	{
		static unsigned char uTx3Addr[uart_dma_buffer_size]={0};
		return uTx3Addr;
	}
	else if(UART4	==	USARTx)
	{
		static unsigned char uTx4Addr[uart_dma_buffer_size]={0};
		return uTx4Addr;		
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
static unsigned short Set_Dma_InfoSize(USART_TypeDef* USARTx,unsigned short size)
{
	if(0==size)	//如果未设定缓存大小，使用默认值
		size	=	uart_dma_buffer_size;
	if(USART1	==	USARTx)
	{
		if(size>uart_dma_buffer_size)
			DmaSize.nUSART1	=	uart_dma_buffer_size;
		else
			DmaSize.nUSART1	=	size;
		return DmaSize.nUSART1;
	}
	else if(USART2	==	USARTx)
	{
		if(size>uart_dma_buffer_size)
			DmaSize.nUSART2	=	uart_dma_buffer_size;
		else
			DmaSize.nUSART2	=	size;
		return DmaSize.nUSART2;
	}
	else if(USART3	==	USARTx)
	{
		if(size>uart_dma_buffer_size)
			DmaSize.nUSART3	=	uart_dma_buffer_size;
		else
			DmaSize.nUSART3	=	size;
		return DmaSize.nUSART3;
	}
	else if(UART4	==	USARTx)
	{
		if(size>uart_dma_buffer_size)
			DmaSize.nUART4	=	uart_dma_buffer_size;
		else
			DmaSize.nUART4	=	size;
		return DmaSize.nUART4;
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
static unsigned short Get_Dma_InfoSize(USART_TypeDef* USARTx)
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
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static DMA_Channel_TypeDef* Get_Usart_Tx_DMA_Channel(USART_TypeDef* USARTx)
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
static DMA_Channel_TypeDef* Get_Usart_Rx_DMA_Channel(USART_TypeDef* USARTx)
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
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void ClearFlag_Usart_Tx_DMA_Channel(USART_TypeDef* USARTx)
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
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void ClearFlag_Usart_Rx_DMA_Channel(USART_TypeDef* USARTx)
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
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
static void	Usart_Dma_Initialize(USART_TypeDef* USARTx,u16 BufferSize)	//USART_DMA配置--查询方式，不开中断
{
	//1)**********定义变量	
	DMA_InitTypeDef DMA_Initstructure;
	
	unsigned char*	uart_rxd	=	Get_Rxd_Addr(USARTx);
	unsigned short dma_size	=	Set_Dma_InfoSize(USARTx,BufferSize);
	
	DMA_Channel_TypeDef* DMAx_Channeltx=Get_Usart_Tx_DMA_Channel(USARTx);			//DMA发送通道请求信号---当DMA串口发送数据完成时，会发起DMA中断
	DMA_Channel_TypeDef* DMAx_Channelrx=Get_Usart_Rx_DMA_Channel(USARTx);			//DMA接收通道请求信号---DMA串口接收由串口发起中断，因此此处接收通道中断不使用	
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
	DMA_Initstructure.DMA_PeripheralBaseAddr  = (u32)(&USARTx->DR);					//DMA外设源地址
	DMA_Initstructure.DMA_MemoryBaseAddr      = (u32)uart_rxd;							//DMA数据内存地址
	DMA_Initstructure.DMA_DIR                 = DMA_DIR_PeripheralDST;			  //DMA_DIR_PeripheralDST（外设作为DMA的目的端），DMA_DIR_PeripheralSRC（外设作为数据传输的来源）
	DMA_Initstructure.DMA_BufferSize          = dma_size; 								  //指定DMA通道的DMA缓存的大小
	DMA_Initstructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;	  //DMA_PeripheralInc_Enable（外设地址寄存器递增），DMA_PeripheralInc_Disable（外设地址寄存器不变），
	DMA_Initstructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;				  //DMA_MemoryInc_Enable（内存地址寄存器递增），DMA_MemoryInc_Disable（内存地址寄存器不变）
	DMA_Initstructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte;	//外设数据宽度--DMA_PeripheralDataSize_Byte（数据宽度为8位），DMA_PeripheralDataSize_HalfWord（数据宽度为16位），DMA_PeripheralDataSize_Word（数据宽度为32位）
	DMA_Initstructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;		  //内存数据宽度--DMA_MemoryDataSize_Byte（数据宽度为8位），DMA_MemoryDataSize_HalfWord（数据宽度为16位），DMA_MemoryDataSize_Word（数据宽度为32位）
	DMA_Initstructure.DMA_Mode                = DMA_Mode_Normal;						  //DMA工作模式--DMA_Mode_Normal（只传送一次）, DMA_Mode_Circular（不停地传送）
	DMA_Initstructure.DMA_Priority            = DMA_Priority_High; 					  //DMA通道的转输优先级--DMA_Priority_VeryHigh（非常高）DMA_Priority_High（高)，DMA_Priority_Medium（中），DMA_Priority_Low（低）
	DMA_Initstructure.DMA_M2M                 = DMA_M2M_Disable;						  //DMA通道的内存到内存传输--DMA_M2M_Enable(设置为内存到内存传输)，DMA_M2M_Disable（非内存到内存传输）
	DMA_Init(DMAx_Channeltx,&DMA_Initstructure);														  //初始化DMA

	//6)**********DMA接收初始化，外设作为DMA的源端
	DMA_Initstructure.DMA_PeripheralBaseAddr  = (u32)(&USARTx->DR);					//DMA外设源地址
	DMA_Initstructure.DMA_MemoryBaseAddr      = (u32)uart_rxd;						  //DMA数据内存地址
	DMA_Initstructure.DMA_DIR                 = DMA_DIR_PeripheralSRC;			  //DMA_DIR_PeripheralDST（外设作为DMA的目的端），DMA_DIR_PeripheralSRC（外设作为数据传输的来源）
	DMA_Initstructure.DMA_BufferSize          = dma_size; 								  //指定DMA通道的DMA缓存的大小
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
	ClearFlag_Usart_Tx_DMA_Channel(USARTx);	// 清除串口DMA方式发送中断全局标志
	ClearFlag_Usart_Rx_DMA_Channel(USARTx);	// 清除串口DMA方式接收中断全局标志	
}
/*******************************************************************************
*函数名			:	USART_DMASend
*功能描述		:	串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
*输入				: 
*返回值			:	如果数据已经传入到DMA，返回Buffer大小，否则返回0（发送器忙）
*******************************************************************************/
static unsigned short Usart_Dma_Send(USART_TypeDef* USARTx,u8 *tx_buffer,u16 BufferSize)		//串口DMA发送程序
{
	unsigned char*	uart_txd	=	Get_Txd_Addr(USARTx);				//发送缓存地址
	unsigned short	uart_size	=	Get_Dma_InfoSize(USARTx);		//发送缓存大小:限定大小，防止溢出
	DMA_Channel_TypeDef* DMAx_Channeltx=Get_Usart_Tx_DMA_Channel(USARTx);			//DMA发送通道请求信号---当DMA串口发送数据完成时，会发起DMA中断
	
	if(UART5==USARTx)		//UART5不支持DMA
		return 0;
	if(0==uart_txd)
		return 0;
	if(uart_size>BufferSize)
	{
		uart_size	=	BufferSize;
	}

	if(
			(DMAx_Channeltx->CNDTR==0)								  //通道空闲--已发完数据
		||((DMAx_Channeltx->CCR&0x00000001)==0)				//通道未开启
		)
	{
		if(SET  ==  USART_GetFlagStatus(USARTx,USART_FLAG_TXE))    //发送完成
		{
			memcpy(uart_txd,tx_buffer,uart_size);
			USART_ClearFlag(USART1, USART_FLAG_TC);
			DMAx_Channeltx->CCR    &= (u32)0xFFFFFFFE;				//DMA_Cmd(DMA1_Channel4,DISABLE);//DMA发送关闭，只能在DMA关闭情况下才可以写入CNDTR					
			ClearFlag_Usart_Tx_DMA_Channel(USARTx);					//DMA_ClearFlag(DMA_FLAG);清除标志		
			DMAx_Channeltx->CNDTR 	=   uart_size;					  //设定待发送缓冲区大小
			DMAx_Channeltx->CMAR 		=   (u32)uart_txd;			  //发送缓冲区
			DMAx_Channeltx->CCR 		|=  (u32)0x00000001;			//DMA_Cmd(DMA1_Channel4,ENABLE);//DMA发送开启3
			return uart_size;
		}
	}
	return 0;
}
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
static void	USART_GPIO_Initialize(USART_TypeDef* USARTx)	//串口GPIO配置
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
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//USART1时钟开启			
					break;
		case 	USART2_BASE:

					GPIO_TX=GPIOA;
					GPIO_RX=GPIOA;
					TXD_Pin=GPIO_Pin_2;		//USART2-TX>PA2
					RXD_Pin=GPIO_Pin_3;		//USART2-RX>PA3
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//USART2时钟开启		
					break;
		case 	USART3_BASE:

					GPIO_TX=GPIOB;
					GPIO_RX=GPIOB;
					TXD_Pin=GPIO_Pin_10;	//USART3-TX>PB10
					RXD_Pin=GPIO_Pin_11;	//USART3-RX>PB11
					
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);				//关闭AFIO时钟,为关闭JTAG功能
					GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  //关闭JTAG功能
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//USART3时钟开启		
					break;
		case 	UART4_BASE:

					GPIO_TX=GPIOC;
					GPIO_RX=GPIOC;
					TXD_Pin=GPIO_Pin_10;	//USART1-TX>PC10
					RXD_Pin=GPIO_Pin_11;	//USART1-RX>PC11
					
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE);
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);	//UART4时钟开启	
					break;
		case 	UART5_BASE:   //UART5不支持DMA
          GPIO_TX=GPIOC;
					GPIO_RX=GPIOD;
					TXD_Pin=GPIO_Pin_10;	//USART1-TX>PC12
					RXD_Pin=GPIO_Pin_2; 	//USART1-RX>PD2
          RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,ENABLE);
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);	//UART4时钟开启	
					break;
		default :break;
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
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
void	USART_IT_Initialize(USART_TypeDef* USARTx)	//串口GPIO配置
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
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
void	USART_DMA_ConfigurationNR(USART_TypeDef* USARTx,u32 USART_BaudRate,u16 BufferSize)	//USART_DMA配置--查询方式，不开中断
{
	//1)**********定义变量	

	USART_InitTypeDef USART_InitStructure;				//USART结构体	
  //2)******************************GPIO配置
  USART_GPIO_Initialize(USARTx);	//串口GPIO配置
	
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
//	
	USART_Cmd(USARTx, ENABLE);
  //2)******************************DMA
	Usart_Dma_Initialize	(USARTx,BufferSize);	//USART_DMA配置--查询方式，不开中断
}
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
void	USART_DMA_ConfigurationIDLEIT(USART_TypeDef* USARTx,u32 USART_BaudRate,u16 BufferSize)	//USART_DMA配置--查询方式，不开中断
{
	//1)**********定义变量	
	USART_InitTypeDef USART_InitStructure;				//USART结构体	
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
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//USART1时钟开启			
					break;
		case 	USART2_BASE:
					GPIO_TX=GPIOA;
					GPIO_RX=GPIOA;
					TXD_Pin=GPIO_Pin_2;		//USART2-TX>PA2
					RXD_Pin=GPIO_Pin_3;		//USART2-RX>PA3
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//USART2时钟开启		
					break;
		case 	USART3_BASE:
					GPIO_TX=GPIOB;
					GPIO_RX=GPIOB;
					TXD_Pin=GPIO_Pin_10;	//USART3-TX>PB10
					RXD_Pin=GPIO_Pin_11;	//USART3-RX>PB11
					
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);				//关闭AFIO时钟,为关闭JTAG功能
					GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  //关闭JTAG功能
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//USART3时钟开启		
					break;
		case 	UART4_BASE:
					GPIO_TX=GPIOC;
					GPIO_RX=GPIOC;
					TXD_Pin=GPIO_Pin_10;	//USART1-TX>PC10
					RXD_Pin=GPIO_Pin_11;	//USART1-RX>PC11
					
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE);
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);	//UART4时钟开启	
					break;
		case 	UART5_BASE:
					//UART5不支持DMA
					return;
		default :break;
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
	
	//3.3)**********初始化串口参数
	USART_DeInit(USARTx);
	USART_InitStructure.USART_BaudRate    = USART_BaudRate; 					      //波特率
	USART_InitStructure.USART_WordLength  = USART_WordLength_8b;		        //数据位
	USART_InitStructure.USART_StopBits    = USART_StopBits_1;				        //停止位
	USART_InitStructure.USART_Parity      = USART_Parity_No ; 					    //奇偶校验
	USART_InitStructure.USART_Mode        = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//流控
	USART_Init(USARTx, &USART_InitStructure);											//初始化串口	

	USART_ITConfig(USARTx,USART_IT_IDLE, ENABLE);				//使用空闲中断，DMA自动接收，检测到总线空闲表示发送端已经发送完成，数据保存在DMA缓冲器中
	USART_ITConfig(USARTx,USART_IT_TC, ENABLE);					//发送完成中断
	USART_ClearITPendingBit(USARTx,USART_IT_IDLE); 				//清除空闲串口标志位
	USART_ClearITPendingBit(USARTx,USART_IT_TC); 					//清除发送完成中断标志位
	
	USART_Cmd(USARTx, ENABLE);
  //2)******************************DMA
  Usart_Dma_Initialize	(USARTx,BufferSize);	//USART_DMA配置--查询方式，不开中断

}
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
void	USART_DMA_ConfigurationEV(USART_TypeDef* USARTx,u32 USART_BaudRate,u16 BufferSize)	//USART_DMA配置--查询方式，不开中断,偶校验
{
	//1)**********定义变量	
	
	USART_InitTypeDef USART_InitStructure;				//USART结构体
  //2)******************************GPIO配置	
	USART_GPIO_Initialize(USARTx);	//串口GPIO配置
	
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
  
  //2)******************************DMA
  Usart_Dma_Initialize	(USARTx,BufferSize);	//USART_DMA配置--查询方式，不开中断
}
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
void	USART_DMA_ConfigurationOD(USART_TypeDef* USARTx,u32 USART_BaudRate,u16 BufferSize)	//USART_DMA配置--查询方式，不开中断--奇校验
{
	//1)**********定义变量	
	USART_InitTypeDef USART_InitStructure;				//USART结构体
  //2)******************************GPIO配置	
	USART_GPIO_Initialize(USARTx);	//串口GPIO配置
		
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
  //2)******************************DMA
  Usart_Dma_Initialize	(USARTx,BufferSize);	//USART_DMA配置--查询方式，不开中断
}

/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--中断方式
*输入				: 
*返回值			:	无
*******************************************************************************/
void	USART_DMA_ConfigurationIT(USART_TypeDef* USARTx,u32 USART_BaudRate,u32 BufferSize)	//USART_DMA配置--中断方式
{
  //2)******************************GPIO配置
  USART_GPIO_Initialize(USARTx);                //串口GPIO配置
  //2)******************************中断配置
  USART_IT_Initialize(USARTx);	                //串口中断配置	
	
	USART_ITConfig(USARTx,USART_IT_IDLE, ENABLE);					//使用空闲中断，DMA自动接收，检测到总线空闲表示发送端已经发送完成，数据保存在DMA缓冲器中
	USART_ClearITPendingBit(USARTx,USART_IT_IDLE); 				//清除空闲串口标志位
	
	USART_Cmd(USARTx, ENABLE);
  
  //2)******************************DMA
  Usart_Dma_Initialize	(USARTx,BufferSize);	//USART_DMA配置--查询方式，不开中断
}

/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断
*输入				: 
*返回值			:	无
*******************************************************************************/
void	USART_DMA_ConfigurationNRRemap(USART_TypeDef* USARTx,u32 USART_BaudRate,u32 *RXDBuffer,u32 BufferSize)	//USART_DMA配置--查询方式，不开中断
{
	//1)**********定义变量	
	USART_InitTypeDef USART_InitStructure;				//USART结构体	
	GPIO_InitTypeDef GPIO_InitStructure;					//GPIO结构体
	
	u16 TXD_Pin=0;																//串口发送脚
	u16 RXD_Pin=0;																//串口接收脚
	GPIO_TypeDef* GPIO_TX=0;
	GPIO_TypeDef* GPIO_RX=0;
	
//	u8 USARTx_IRQChannel=0;
	//2)******************************配置相关GPIO/串口时钟打开
	//2.1)**********USART1
	if(USARTx==USART1)
	{

		TXD_Pin=GPIO_Pin_6;									  //USART1-TX>PA9
		RXD_Pin=GPIO_Pin_7;										//USART1-RX>PA10
		
		GPIO_TX=GPIOB;
		GPIO_RX=GPIOB;
		
//		USARTx_IRQChannel=USART1_IRQChannel;		//中断
		
		GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);				//I/O口重映射开启
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//USART1时钟开启

	}
	//2.2)**********USART2
	else if(USARTx==USART2)
	{
		
		TXD_Pin=GPIO_Pin_2;		//USART2-TX>PA2
		RXD_Pin=GPIO_Pin_3;		//USART2-RX>PA3
		
		GPIO_TX=GPIOA;
		GPIO_RX=GPIOA;
		
//		USARTx_IRQChannel=USART2_IRQChannel;		//中断
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//USART1时钟开启

	}
	//2.3)**********USART3
	else if(USARTx==USART3)
	{		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);				//关闭AFIO时钟,为关闭JTAG功能
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  //关闭JTAG功能
		
		TXD_Pin=GPIO_Pin_10;	//USART3-TX>PB10
		RXD_Pin=GPIO_Pin_11;	//USART3-RX>PB11
		
		GPIO_TX=GPIOB;
		GPIO_RX=GPIOB;
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//USART1时钟开启
		

	}
	//2.4)**********USART4
	else if(USARTx==UART4)
	{
		TXD_Pin=GPIO_Pin_10;	//USART1-TX>PC10
		RXD_Pin=GPIO_Pin_11;	//USART1-RX>PC11
		
		GPIO_TX=GPIOC;
		GPIO_RX=GPIOC;
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);	//USART1时钟开启
	}
	//2.5)**********USART5
	else if(USARTx==UART5)
	{
		
		TXD_Pin=GPIO_Pin_12;	//USART1-TX>PC12
		RXD_Pin=GPIO_Pin_2;		//USART1-RX>PD2
		
		GPIO_TX=GPIOC;
		GPIO_RX=GPIOD;
		
//		USARTx_IRQChannel=UART5_IRQChannel;		//中断
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);	//USART1时钟开启
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
	
	//3.3)**********初始化串口参数
	USART_DeInit(USARTx);
	USART_InitStructure.USART_BaudRate = USART_BaudRate; 					//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;				//停止位
	USART_InitStructure.USART_Parity = USART_Parity_No ; 					//奇偶校验
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//流控
	USART_Init(USARTx, &USART_InitStructure);											//初始化串口
	
	USART_ITConfig(USARTx,USART_IT_IDLE, DISABLE);					//使用空闲中断，DMA自动接收，检测到总线空闲表示发送端已经发送完成，数据保存在DMA缓冲器中
	USART_ClearITPendingBit(USARTx,USART_IT_IDLE); 					//清除空闲串口标志位
	
	USART_Cmd(USARTx, ENABLE);
  
  //2)******************************DMA
  Usart_Dma_Initialize	(USARTx,BufferSize);	//USART_DMA配置--查询方式，不开中断
}
/*******************************************************************************
* 函数名			:	USART_ReadBuffer
* 功能描述		:	串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数，然后重新将接收缓冲区地址指向RxdBuffer， 
* 输入			: void
* 返回值			: void
*******************************************************************************/
u16	API_USART_ReadBufferIDLE(USART_TypeDef* USARTx,u8 *RevBuffer)	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数
{
	u16 length=0;
	unsigned char*	uart_rxd	=	Get_Rxd_Addr(USARTx);
	unsigned short 	dma_size	=	Get_Dma_InfoSize(USARTx);
	DMA_Channel_TypeDef* DMAx_Channelrx=Get_Usart_Rx_DMA_Channel(USARTx);			//DMA接收通道请求信号---DMA串口接收由串口发起中断，因此此处接收通道中断不使用
	
	if(UART5==USARTx)		//UART5不支持DMA
		return 0;
	if(0==uart_rxd)
		return 0;
	if(USART_GetITStatus(USARTx,USART_IT_IDLE)||USART_GetFlagStatus(USARTx,USART_FLAG_IDLE))
	{
		USART_ClearITPendingBit(USARTx,USART_IT_IDLE); 							//清除空闲串口标志位
		USART_ClearFlag(USARTx,USART_FLAG_IDLE); 										//清除空闲串口标志位
		DMAx_Channelrx->CCR&= (u32)0xFFFFFFFE;											//DMA_Cmd(DMAx_Channelx,DISABLE);//DMA接收关闭，只能在DMA关闭情况下才可以写入CNDTR
		length 	= DMAx_Channelrx->CNDTR;														//DMA_GetCurrDataCounter(DMA1_Channel5);	//得到真正接收数据个数(DMA_GetCurrDataCounter返回当前DMA通道x剩余的待传输数据数目)
		length	=	dma_size-length;												    			//设定缓冲区大小减剩余缓冲区大小得到实际接收到的数据个数
		memcpy(RevBuffer,uart_rxd,length);													//复制指定大小的数据
		//------重新指向接收缓冲区地址并使能DMA接收			
		DMAx_Channelrx->CMAR=(u32)uart_rxd;													//重新设置DMA接收地址
		DMAx_Channelrx->CNDTR=dma_size;			  											//重新设置接收数据个数			
		DMAx_Channelrx->CCR |= ((u32)0x00000001);										//DMA_Cmd(DMAx_Channelrx,ENABLE);//开启接收DMA
		return length;			//返回接收到的数据个数
	}
	return 0;
}

/*******************************************************************************
*函数名		: function
*功能描述	:	串口接收服务函数
*输入			: 
*输出			:	无
*返回值		:	无
*例程			:	USART_DMASend(USART2,"中文ENG=%d\n",num);
*特别说明	:	在DMA发送完成后需要释放动态空间，free(USART_BUFFER);
					:	USART_BUFFER定义在STM32_USART.H
*******************************************************************************/
u16 USART_DMAPrintf(USART_TypeDef* USARTx,const char *format,...)		//后边的省略号就是可变参数
{
	
//		va_list ap; 										//VA_LIST 是在C语言中解决变参问题的一组宏，所在头文件：#include <stdarg.h>,用于获取不确定个数的参数
//    static char string[ 256 ];			//定义数组，
//    va_start( ap, format );
//    vsprintf( string , format, ap );    
//    va_end( ap );
	
	
//8)**********将等发送缓冲区大小（数据个数）及缓冲区地址发给DMA开启发送
//8)**********DMA发送完成后注意应该释放缓冲区：free(USART_BUFFER);
//	if(DMAPrintf_Buffer!=NULL)
//	{
//		free(DMAPrintf_Buffer);						//释放动态空间
//	}
//	u8	*USART_BUFFER;
	//1)**********获取数据宽度
//	u32 num=strlen((const char*)format);		//获取数据宽度
	//2)**********定义缓冲区大小变量
	unsigned int BufferSize=0;
	unsigned char DMAPrintf_Buffer[256]={0};
	//3)**********args为定义的一个指向可变参数的变量，va_list以及下边要用到的va_start,va_end都是是在定义，可变参数函数中必须要用到宏， 在stdarg.h头文件中定义
	va_list args;  
//	free(DMAPrintf_Buffer);						//释放动态空间
//	DMAPrintf_Buffer=NULL;
	//4)**********申请动态空间
//	DMAPrintf_Buffer = (char*)malloc(sizeof(char) * num);	
	//5)**********初始化args的函数，使其指向可变参数的第一个参数，format是可变参数的前一个参数
	va_start(args, format);
	//6)**********正常情况下返回生成字串的长度(除去\0),错误情况返回负值
	BufferSize = vsprintf((char*)DMAPrintf_Buffer, format, args);
	//7)**********结束可变参数的获取
	va_end(args); 
	//8)**********将等发送缓冲区大小（数据个数）及缓冲区地址发给DMA开启发送
	//8)**********DMA发送完成后注意应该释放缓冲区：free(USART_BUFFER);

  BufferSize=Usart_Dma_Send(USARTx,(u8*)DMAPrintf_Buffer,BufferSize);	//串口DMA发送程序

	return BufferSize;			//返回发送数据大小
}


/*******************************************************************************
*函数名			:	USART_DMASend
*功能描述		:	串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
*输入				: 
*返回值			:	如果数据已经传入到DMA，返回Buffer大小，否则返回0（发送器忙）
*******************************************************************************/
unsigned short API_USART_DMA_Send(USART_TypeDef* USARTx,u8 *tx_buffer,u16 BufferSize)		//串口DMA发送程序
{
	unsigned short SendedSize	=	Usart_Dma_Send(USARTx,tx_buffer,BufferSize);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
	return SendedSize;
}
//----------------------RS485---------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	RS485_TX_EN
*功能描述		:	使能RS485发送---拉高控制脚
*输入				: 
*返回值			:	无
*******************************************************************************/
void RS485_TX_EN(RS485Def *pRS485)
{
	pRS485->RS485_CTL_PORT->BSRR		= pRS485->RS485_CTL_Pin;
//	GPIO_SetBits(RS485_Info->RS485_CTL_PORT,RS485_Info->RS485_CTL_Pin);
}
/*******************************************************************************
*函数名			:	RS485_RX_EN
*功能描述		:	使能RS485接收---拉低控制脚,使能前需要检测发送状态，如果在发送中，则不使能，
*输入				: 
*返回值			:	已经设置为接收状态返回1，否则返回0
*******************************************************************************/
void RS485_RX_EN(RS485Def *pRS485)
{
	pRS485->RS485_CTL_PORT->BRR 		= pRS485->RS485_CTL_Pin;	
}

/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
*输入				: 
*返回值			:	无
*******************************************************************************/
void	RS485_DMA_ConfigurationNR(RS485Def *pRS485,u32 USART_BaudRate,u16 BufferSize)	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
{
  GPIO_Configuration_OPP50	(pRS485->RS485_CTL_PORT,pRS485->RS485_CTL_Pin);			//将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度50MHz----V20170605
	pRS485->RS485_CTL_PORT->BRR 		= pRS485->RS485_CTL_Pin;				//RS485接收开启
	USART_DMA_ConfigurationNR	(pRS485->USARTx,USART_BaudRate,BufferSize);		//USART_DMA配置--查询方式，不开中断
	GPIO_Configuration_OPP50	(pRS485->RS485_CTL_PORT,pRS485->RS485_CTL_Pin);			//将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度50MHz----V20170605
	pRS485->RS485_CTL_PORT->BRR 		= pRS485->RS485_CTL_Pin;				//RS485接收开启
}
/*******************************************************************************
*函数名			:	USART_DMA_ConfigurationNr
*功能描述		:	USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
*输入				: 
*返回值			:	无
*******************************************************************************/
void	RS485_DMA_ConfigurationNRRemap(RS485Def *pRS485,u32 USART_BaudRate,u32 *RXDBuffer,u32 BufferSize)	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
{
	USART_DMA_ConfigurationNRRemap	(pRS485->USARTx,USART_BaudRate,RXDBuffer,BufferSize);		//USART_DMA配置--查询方式，不开中断
	GPIO_Configuration_OPP50				(pRS485->RS485_CTL_PORT,pRS485->RS485_CTL_Pin);			//将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度50MHz----V20170605
	pRS485->RS485_CTL_PORT->BRR 		= pRS485->RS485_CTL_Pin;				//RS485接收开启
}
/*******************************************************************************
*函数名			:	RS485_ReadBufferIDLE
*功能描述		:	串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数，然后重新将接收缓冲区地址指向RxdBuffer
*输入				: 
*返回值			:	无
*******************************************************************************/
u16	RS485_ReadBufferIDLE(RS485Def *pRS485,u8 *RevBuffer)	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数，然后重新将接收缓冲区地址指向RxdBuffer，
{
	u16 length=0;
	if(USART_GetFlagStatus(pRS485->USARTx,USART_FLAG_TC))    //发送完成
	{
		RS485_RX_EN(pRS485);
		length=API_USART_ReadBufferIDLE(pRS485->USARTx,RevBuffer);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数，然后重新将接收缓冲区地址指向RxdBuffer
	}
	return length;
}

/*******************************************************************************
*函数名			:	function
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*******************************************************************************/
u16	RS485_DMAPrintf(RS485Def *pRS485,const char *format,...)						//自定义printf串口DMA发送程序,后边的省略号就是可变参数
{
		
//		va_list ap; 										//VA_LIST 是在C语言中解决变参问题的一组宏，所在头文件：#include <stdarg.h>,用于获取不确定个数的参数
//    static char string[ 256 ];			//定义数组，
//    va_start( ap, format );
//    vsprintf( string , format, ap );    
//    va_end( ap );
	
//8)**********将等发送缓冲区大小（数据个数）及缓冲区地址发给DMA开启发送
//8)**********DMA发送完成后注意应该释放缓冲区：free(USART_BUFFER);
//	if(DMAPrintf_Buffer!=NULL)
//	{
//		free(DMAPrintf_Buffer);						//释放动态空间
//	}
//	u8	*USART_BUFFER;
	//1)**********获取数据宽度
	u32 num=strlen((const char*)format);		//获取数据宽度
	//2)**********定义缓冲区大小变量
	unsigned int BufferSize;
	unsigned char DMAPrintf_Buffer[256]={0};
	//3)**********args为定义的一个指向可变参数的变量，va_list以及下边要用到的va_start,va_end都是是在定义，可变参数函数中必须要用到宏， 在stdarg.h头文件中定义
	va_list args;  
	free(DMAPrintf_Buffer);						//释放动态空间
//	DMAPrintf_Buffer=NULL;
	//4)**********申请动态空间
//	DMAPrintf_Buffer = (char*)malloc(sizeof(char) * num);	
	//5)**********初始化args的函数，使其指向可变参数的第一个参数，format是可变参数的前一个参数
	va_start(args, format);
	//6)**********正常情况下返回生成字串的长度(除去\0),错误情况返回负值
	BufferSize = vsprintf((char*)DMAPrintf_Buffer, format, args);
	//7)**********结束可变参数的获取
	va_end(args); 
	//8)**********将等发送缓冲区大小（数据个数）及缓冲区地址发给DMA开启发送
	//8)**********DMA发送完成后注意应该释放缓冲区：free(USART_BUFFER);
	BufferSize=RS485_DMASend(pRS485,(u8*)DMAPrintf_Buffer,BufferSize);	//RS485-DMA发送程序
	return BufferSize;			//返回发送数据大小

}
/*******************************************************************************
*函数名			:	USART_DMASend
*功能描述		:	串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
*输入				: 
*返回值			:	如果数据已经传入到DMA，返回Buffer大小，否则返回0（发送器忙）
*******************************************************************************/
u16 RS485_DMASend(
									RS485Def *pRS485,		//包含RS485选用的串口号和收发控制脚信息
									u8 *tx_buffer,								//待发送数据缓冲区地址
									u16 BufferSize								//设定发送数据大小
)		//RS485-DMA发送程序
{
	//----发送前检查相关串口发送状态，如果下在发送其它数据，则等待（返回0），否则清除相关标志位后开启发送
	
//	u32	DMA_status=0;			//DMA状态
  unsigned short sendedlen  =0;
	USART_TypeDef* USARTx	=	pRS485->USARTx;
	if(USART_GetFlagStatus(USARTx,USART_FLAG_TXE))
	{
		RS485_TX_EN(pRS485);
		sendedlen = Usart_Dma_Send(USARTx,(u8*)tx_buffer,BufferSize);		//串口DMA发送程序
	}
	return sendedlen;
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
void	USART_ConfigurationIT(
																USART_TypeDef* USARTx,			//串口号--USART1,USART2,USART3,UART4;//UART5不支持DMA
																u32 USART_BaudRate,					//波特率
																u8 PreemptionPriority,			//中断优先级
																u8 SubPriority							//抢占优先级
)	//USART_DMA配置--查询方式，不开中断--奇校验
{
	//1)**********定义变量	

	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;				//USART结构体	
	GPIO_InitTypeDef GPIO_InitStructure;					//GPIO结构体

	
	u16 TXD_Pin=0;																//串口发送脚
	u16 RXD_Pin=0;																//串口接收脚
	GPIO_TypeDef* GPIO_TX=0;
	GPIO_TypeDef* GPIO_RX=0;
	
	u8 USARTx_IRQChannel=0;
	//2)******************************配置相关GPIO/串口时钟打开
	//2.1)**********USART1
	if(USARTx==USART1)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);				//关闭AFIO时钟,为关闭JTAG功能
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//USART1时钟开启
		
		TXD_Pin=GPIO_Pin_9;											//USART1-TX>PA9
		RXD_Pin=GPIO_Pin_10;										//USART1-RX>PA10
		
		GPIO_TX=GPIOA;
		GPIO_RX=GPIOA;
		
		USARTx_IRQChannel=USART1_IRQChannel;		//中断
		

	}
	//2.2)**********USART2
	else if(USARTx==USART2)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);				//关闭AFIO时钟,为关闭JTAG功能
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//USART1时钟开启
		
		TXD_Pin=GPIO_Pin_2;		//USART2-TX>PA2
		RXD_Pin=GPIO_Pin_3;		//USART2-RX>PA3
		
		GPIO_TX=GPIOA;
		GPIO_RX=GPIOA;
		
		USARTx_IRQChannel=USART2_IRQChannel;		//中断
	}
	//2.3)**********USART3
	else if(USARTx==USART3)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);				//关闭AFIO时钟,为关闭JTAG功能
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//USART1时钟开启
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  //关闭JTAG功能
		
		TXD_Pin=GPIO_Pin_10;	//USART3-TX>PB10
		RXD_Pin=GPIO_Pin_11;	//USART3-RX>PB11
		
		GPIO_TX=GPIOB;
		GPIO_RX=GPIOB;
		
		USARTx_IRQChannel=USART3_IRQChannel;		//中断
	}
	//2.4)**********USART4
	else if(USARTx==UART4)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);	//USART1时钟开启
		
		TXD_Pin=GPIO_Pin_10;	//USART1-TX>PC10
		RXD_Pin=GPIO_Pin_11;	//USART1-RX>PC11
		
		GPIO_TX=GPIOC;
		GPIO_RX=GPIOC;
		
		USARTx_IRQChannel=UART4_IRQChannel;		//中断
		
	}
	//2.5)**********USART5
	else if(USARTx==UART5)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);	//USART1时钟开启
		
		TXD_Pin=GPIO_Pin_12;	//USART1-TX>PC12
		RXD_Pin=GPIO_Pin_2;		//USART1-RX>PD2
		
		GPIO_TX=GPIOC;
		GPIO_RX=GPIOD;
		
		USARTx_IRQChannel=UART5_IRQChannel;		//中断		
	}
	
//	/* Configure the NVIC Preemption Priority Bits */  
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	/* Enable the USART2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	PreemptionPriority;			//中断优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;										//抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	//3)**********初始化串口
	//3.1)**********初始化TXD引脚
	GPIO_InitStructure.GPIO_Pin = TXD_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_TX,&GPIO_InitStructure);

	//3.2)**********初始化RXD引脚
	GPIO_InitStructure.GPIO_Pin = RXD_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;							//上拉输入
	GPIO_Init(GPIO_RX,&GPIO_InitStructure);
	
	//3.3)**********初始化串口参数
	USART_DeInit(USARTx);
	USART_InitStructure.USART_BaudRate = USART_BaudRate; 					//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;				//停止位
	USART_InitStructure.USART_Parity = USART_Parity_No ; 					//奇偶校验
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//流控
	USART_Init(USARTx, &USART_InitStructure);											//初始化串口
	
	
	
	
	

	/* Enable USART1 Receive and Transmit interrupts */
  USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
//  USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);
//	USART_ITConfig(USARTx, USART_IT_TC, ENABLE);
	
//	USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
//	USART_ClearITPendingBit(USARTx, USART_IT_TC);
	
	USART_Cmd(USARTx, ENABLE);

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
int fputc(int ch, FILE *f)				//printf重定义
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


































