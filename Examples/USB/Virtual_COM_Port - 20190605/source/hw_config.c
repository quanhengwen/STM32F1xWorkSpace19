/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : hw_config.c
* Author             : MCD Application Team
* Version            : V2.2.0
* Date               : 06/13/2008
* Description        : Hardware Configuration & Setup
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/******************************** 说明20160912**********************************
********************************************************************************
* 功能：用于 USB硬件配置
* 
* 
* 
* 
* 
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
//#include "stm32f10x_it.h"
//#include "usb_lib.h"
//#include "usb_prop.h"
//#include "usb_desc.h"
//#include "usb_init.h"			//用于端点数据输入输入中断处理
#include "hw_config.h"
////#include "platform_config.h"
//#include "usb_pwr.h"
//#include	"string.h"			//memcpy
//#include "STM32_USART.H"
//#include "STM32_GPIO.H"
//#include "stm32f10x_nvic.h"
//#include "Virtual_COM_Port.H"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//USART_InitTypeDef USART_InitStructure;
//ErrorStatus HSEStartUpStatus;

/* Extern variables ----------------------------------------------------------*/
//u8 buffer_in[VIRTUAL_COM_PORT_DATA_SIZE];

//u8 buffer_rx[VIRTUAL_COM_PORT_DATA_SIZE];
//u8 buffer_tx[VIRTUAL_COM_PORT_DATA_SIZE];

//u32 USART_Rx_ptr_in = 0;
//u32 USART_Rx_ptr_out = 0;
//u32 USART_Rx_length  = 0;

//u8 Usart_tx_flg=0;

//extern u32 count_in;
//extern LINE_CODING linecoding;

//extern u8 buffer_out[VIRTUAL_COM_PORT_DATA_SIZE];
//extern u32 count_out;

//u8  USB_Tx_State = 0;


//GPIO_TypeDef*		usb_en_port;
//unsigned short	usb_en_pin;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
///*******************************************************************************
//* Function Name  : Set_System
//* Description    : Configures Main system clocks & power
//* Input          : None.
//* Return         : None.
//*******************************************************************************/
//void api_usb_virtual_com_configuration(usb_en_def* pInfo)		//虚拟串口配置
//{	
//	usb_en_port	=	(GPIO_TypeDef*)pInfo->usb_connect_port;
//	usb_en_pin	=	pInfo->usb_connect_pin;
//	
//	GPIO_Configuration_OPP50(usb_en_port,usb_en_pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
//	
//	usb_hw_set_connect(DISABLE);	//关闭USB上拉电阻
//	
//	usb_hw_set_clock();			//设置USB时钟
//	
//	usb_hw_set_Interrupt();	//设置USB中断
//	
//	USB_Init();						//USB初始化
//}

//-----------------------------------------------------------------------------










///*******************************************************************************
//* Function Name  : USB_Cable_Config
//* Description    : Software Connection/Disconnection of USB Cable
//* Input          : None.
//* Return         : Status
//*******************************************************************************/
//void usb_hw_set_connect(FunctionalState NewState)
//{
//  if (NewState != DISABLE)
//  {
//		//GPIO_ResetBits(usb_en_port, usb_en_pin);
//		GPIO_SetBits(usb_en_port, usb_en_pin);
//  }
//  else
//  {
//		//GPIO_SetBits(usb_en_port, usb_en_pin);
//		GPIO_ResetBits(usb_en_port, usb_en_pin);
//  }
//}
///*******************************************************************************
//* Function Name  :  UART0_Config_Default.
//* Description    :  configure the UART 0 with default values.	串口的默认配置值
//* Input          :  None.
//* Return         :  None.
//*******************************************************************************/
//void usb_hw_set_usart_default(void)
//{
//	USART_InitTypeDef USART_InitStructure;
////--------------原程序
////	linecoding
//	USART_DeInit(ComPort);
//	
//	USART_InitStructure.USART_BaudRate    = 19200; 					  //波特率
//	USART_InitStructure.USART_WordLength  = USART_WordLength_8b;		    //数据位
//	USART_InitStructure.USART_StopBits    = USART_StopBits_1;				    //停止位
//	USART_InitStructure.USART_Parity      = USART_Parity_No ; 					//奇偶校验
//	USART_InitStructure.USART_Mode        = USART_Mode_Rx | USART_Mode_Tx;
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//流控
//	USART_Init(ComPort, &USART_InitStructure);											//初始化串口
//	
//	api_usart_dma_configurationST(ComPort,&USART_InitStructure,usart_buffer_size);	//USART_DMA配置--结构体形式，不开中断	
//}
///*******************************************************************************
//* Function Name  :  UART0_Config.
//* Description    :  Configure the UART 1 according to the linecoding structure.
//										根据line coding 结构体配置串口.
//* Input          :  None.
//* Return         :  Configuration status
//                    TRUE : configuration done with success
//                    FALSE : configuration aborted.
//*******************************************************************************/
//bool usb_hw_set_usart_config(void)
//{
//	USART_InitTypeDef USART_InitStructure;
//  /* set the Stop bit*/
//	/**************设置停止位**************/
//  switch (linecoding.format)
//  {
//    case 0:
//      USART_InitStructure.USART_StopBits = USART_StopBits_1;		//1位停止位
//      break;
//    case 1:
//      USART_InitStructure.USART_StopBits = USART_StopBits_1_5;	//1.5为停止位
//      break;
//    case 2:
//      USART_InitStructure.USART_StopBits = USART_StopBits_2;		//2位停止位
//      break;
//    default :
//    {
//      usb_hw_set_usart_default();																			//默认配置
//      return (FALSE);
//    }
//  }

//  /* set the parity bit*/
//	/**************设置校验位**************/
//  switch (linecoding.paritytype)
//  {
//    case 0:
//      USART_InitStructure.USART_Parity = USART_Parity_No;			//没有校验
//      break;
//    case 1:
//      USART_InitStructure.USART_Parity = USART_Parity_Even;		//偶校验
//      break;	
//    case 2:
//      USART_InitStructure.USART_Parity = USART_Parity_Odd;		//奇校验
//      break;
//    default :
//    {
//      usb_hw_set_usart_default();																	//默认配置
//      return (FALSE);
//    }
//  }

//  /*set the data type : only 8bits and 9bits is supported */
//	/**************设置数据位: 8位或9位**************/
//  switch (linecoding.datatype)
//  {
//    case 0x07:
//      USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//8为数据位，这个选项就校验位必须设置(奇校验/偶校验)
//      break;
//    case 0x08:
//      USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//USART_WordLength_9b;	//9位数据位
//      break;
//    default :
//    {
//      usb_hw_set_usart_default();																//默认配置
//      return (FALSE);
//    }
//  }
//  USART_InitStructure.USART_BaudRate = linecoding.bitrate;													//设置波特率
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		//设置没有硬件数据流控制
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;										//使能接收、发送
//  USART_Init(ComPort, &USART_InitStructure);																				//初始化串口
//  USART_Cmd(ComPort, ENABLE);																												//使能串口
//	
//	//api_usart_dma_configurationNR(ComPort,linecoding.bitrate,usart_buffer_size);
//	api_usart_dma_configurationST(ComPort,&USART_InitStructure,usart_buffer_size);	//USART_DMA配置--结构体形式，不开中断
//	
//  return (TRUE);
//}
///*******************************************************************************
//* Function Name  : USB_To_UART_Send_Data.
//* Description    : send the received data from USB to the UART 0.将USB接收到的数据从串口发送数据
//* Input          : data_buffer: data address.
//                   Nb_bytes: number of bytes to send.
//* Return         : none.
//*******************************************************************************/
//void usb_hw_receive_from_usb(u8* data_buffer, u8 Nb_bytes)
//{
//	Usart_tx_flg=1;
//	api_usart_dma_send(ComPort,data_buffer,(u16)Nb_bytes);		//自定义printf串口DMA发送程序
//}

///*******************************************************************************
//* Function Name  : UART_To_USB_Send_Data.
//* Description    : send the received data from UART 0 to USB.	发送串口接收到的数据到USB
//* Input          : None.
//* Return         : none.
//*******************************************************************************/
//void usb_hw_send_to_usb(void)
//{	
//	u16	num	=	api_usart_dma_receive(ComPort,buffer_in);
//	if(num)
//	{
//		UserToPMABufferCopy(buffer_in, ENDP1_TXADDR, num);
//		SetEPTxCount(ENDP1, num);																					//设置端点数据长度
//		SetEPTxValid(ENDP1);																							//使能端点
//	}
//}

///*******************************************************************************
//* Function Name  : Handle_USBAsynchXfer.
//* Description    : send data to USB.
//* Input          : None.
//* Return         : none.
//*******************************************************************************/
//void usb_hw_Handle_USBAsynchXfer (void)
//{
//	count_in=api_usart_dma_receive(ComPort,buffer_rx);
//	if(count_in)
//	{
//		UserToPMABufferCopy(buffer_rx, ENDP1_TXADDR, count_in);
//		SetEPTxCount(ENDP1, count_in);
//		SetEPTxValid(ENDP1);
//	}
//}
///*******************************************************************************
//* Function Name  : usb_hw_Handle_USBAsynchXfer.
//* Description    : send data to USB.
//* Input          : None.
//* Return         : none.
//*******************************************************************************/
//void usb_hw_Handle_USBAsynchXferBAC1 (void)
//{
//  
//  u16 USB_Tx_ptr;
//  u16 USB_Tx_length;
//  
//  if(USB_Tx_State != 1)
//  {
//    if (USART_Rx_ptr_out == VIRTUAL_COM_PORT_DATA_SIZE)
//    {
//      USART_Rx_ptr_out = 0;
//    }
//    
//    if(USART_Rx_ptr_out == USART_Rx_ptr_in) 
//    {
//      USB_Tx_State = 0; 
//      return;
//    }
//    
//    if(USART_Rx_ptr_out > USART_Rx_ptr_in) /* rollback */
//    { 
//      USART_Rx_length = VIRTUAL_COM_PORT_DATA_SIZE - USART_Rx_ptr_out;
//    }
//    else 
//    {
//      USART_Rx_length = USART_Rx_ptr_in - USART_Rx_ptr_out;
//    }
//    
//    if (USART_Rx_length > VIRTUAL_COM_PORT_DATA_SIZE)
//    {
//      USB_Tx_ptr = USART_Rx_ptr_out;
//      USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;
//      
//      USART_Rx_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;	
//      USART_Rx_length -= VIRTUAL_COM_PORT_DATA_SIZE;	
//    }
//    else
//    {
//      USB_Tx_ptr = USART_Rx_ptr_out;
//      USB_Tx_length = USART_Rx_length;
//      
//      USART_Rx_ptr_out += USART_Rx_length;
//      USART_Rx_length = 0;
//    }
//    USB_Tx_State = 1; 
//    UserToPMABufferCopy(&buffer_in[USB_Tx_ptr], ENDP1_TXADDR, USB_Tx_length);
//    SetEPTxCount(ENDP1, USB_Tx_length);
//    SetEPTxValid(ENDP1); 
//  }  
//  
//}



//-----------------------------------------------------------------------------




////-----------------------------------------------------------------------------static
///*******************************************************************************
//* Function Name  : Set_USBClock
//* Description    : Configures USB Clock input (48MHz)
//* Input          : None.
//* Return         : None.
//*******************************************************************************/
//static void usb_hw_set_clock(void)
//{
//  /* USBCLK = PLLCLK / 1.5 */
//  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
//  /* Enable USB clock */
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
//}
///*******************************************************************************
//* Function Name  : USB_Interrupts_Config
//* Description    : Configures the USB interrupts
//* Input          : None.
//* Return         : None.
//*******************************************************************************/
//static void usb_hw_set_Interrupt(void)
//{
//  NVIC_InitTypeDef NVIC_InitStructure;

//#ifdef  VECT_TAB_RAM
//  /* Set the Vector Table base location at 0x20000000 */
//  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
//#else  /* VECT_TAB_FLASH */
//  /* Set the Vector Table base location at 0x08000000 */
//  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
//#endif

//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

//  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN_RX0_IRQChannel;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);

////  /* Enable USART1 Interrupt */
////  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
////  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
////  NVIC_Init(&NVIC_InitStructure);
//}
//-----------------------------------------------------------------------------



/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
