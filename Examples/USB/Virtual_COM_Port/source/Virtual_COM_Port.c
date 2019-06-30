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
#ifdef Virtual_COM_Port

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
#include "Virtual_COM_Port.h"
//#include "stm32f10x_it.h"



//#include "hw_config.h"
//#include "platform_config.h"
#include	"string.h"			//memcpy


#include "STM32_GPIO.H"
#include "STM32_USART.H"

#include "STM32_SYSTICK.H"
#include "STM32_PWM.H"

//#include "stm32f10x_nvic.h"

#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "usb_type.h"
#include "usb_core.h"			//USB总线数据处理的核心文件

#include "usb_endp.h"
#include "usb_data.h"
#include "hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define lora	0
#define bq26100	1
#define STM32_FSMC 0
#define STM32_USB_TEST 0


#if PS005
		#define USB_DISCONNECT            GPIOA  
		#define USB_DISCONNECT_PIN        GPIO_Pin_8
		#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
	#elif lora
	  #define USB_DISCONNECT            GPIOA  
		#define USB_DISCONNECT_PIN        GPIO_Pin_15
		#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
	#elif bq26100
	  #define USB_DISCONNECT            GPIOA  
		#define USB_DISCONNECT_PIN        GPIO_Pin_15
		
		#define G1Port   	GPIOB  
		#define G1Pin    	GPIO_Pin_4
		
		#define G2Port   	GPIOB  
		#define G2Pin    	GPIO_Pin_3
		
		#define G3Port   	GPIOB  
		#define G3Pin    	GPIO_Pin_9
		
		#define V24Port   	GPIOB  
		#define V24Pin    	GPIO_Pin_8
		
		#define V05Port   	GPIOB  
		#define V05Pin    	GPIO_Pin_5
		
		#define MtxPort   	GPIOB  
		#define MtxPin    	GPIO_Pin_6
		
		#define MrxPort   	GPIOB  
		#define MrxPin    	GPIO_Pin_7
		
	#elif STM32_USB_TEST
	  #define USB_DISCONNECT            GPIOA  
		#define USB_DISCONNECT_PIN        GPIO_Pin_15
		#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
	#elif STM32_FSMC
	  #define USB_DISCONNECT            GPIOA  
		#define USB_DISCONNECT_PIN        GPIO_Pin_8
		#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
	#else
	  #define USB_DISCONNECT            GPIOA  
		#define USB_DISCONNECT_PIN        GPIO_Pin_8
		#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
#endif

#define ComPort	USART1
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//USART_InitTypeDef USART_InitStructure;


/* Extern variables ----------------------------------------------------------*/
u8 buffer_in[USB_BUFFER_SIZE];

u8 buffer_rx[VIRTUAL_COM_PORT_DATA_SIZE];
u8 buffer_tx[VIRTUAL_COM_PORT_DATA_SIZE];

u32 USART_Rx_ptr_in = 0;
u32 USART_Rx_ptr_out = 0;
u32 USART_Rx_length  = 0;

u8 Usart_tx_flg=0;
unsigned short virtual_com_time = 0;

extern u32 count_in;
extern LINE_CODING linecoding;

//extern u8 buffer_out[VIRTUAL_COM_PORT_DATA_SIZE];
//extern u32 count_out;




/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void api_usb_virtual_gpio_configuration(void);

extern void api_usb_hw_initialize(void);
void USB_CMD(FunctionalState NewState);
static void bq26100test(void);

/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
void api_usb_virtual_com_configuration(void)		//虚拟串口配置
{	
	
	api_usb_virtual_gpio_configuration();
	
	api_usb_hw_initialize();
  
//	while(1)
//	{
//		api_usb_virtual_com_server();
//	}
//	 PWM_OUT(TIM2,PWM_OUTChannel1,1,500);	//PWM设定-20161127版本	占空比1/1000
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
void api_usb_virtual_com_server(void)
{	
	if(virtual_com_time<1000)	//等待USB初始化1秒
	{
		virtual_com_time ++;
		return ;
	}
	usb_to_uart_server();
	USART_To_USB_Send_Data();
	bq26100test();
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
static void bq26100test(void)
{
	static unsigned short time=0;
	time++;
	if(time==500)
		GPIO_SetBits(G2Port,G2Pin);
	else if(time>=5000)
	{
		GPIO_ResetBits(G2Port,G2Pin);
		time=0;
	}
//GPIO_SetBits(G2Port,G2Pin);
	//GPIO_ResetBits(G2Port,G2Pin);
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
void api_usb_virtual_gpio_configuration(void)
{
	GPIO_Configuration_OPP50(USB_DISCONNECT,USB_DISCONNECT_PIN);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	USB_CMD(DISABLE);
#if bq26100
	GPIO_Configuration_OPP50(G1Port,G1Pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	//GPIO_SetBits(G1Port,G1Pin);
	GPIO_ResetBits(G1Port,G1Pin);
	
	GPIO_Configuration_OPP50(G2Port,G2Pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	//GPIO_SetBits(G2Port,G2Pin);
	GPIO_ResetBits(G2Port,G2Pin);
	
	GPIO_Configuration_OPP50(G3Port,G3Pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	//GPIO_SetBits(G3Port,G3Pin);
	GPIO_ResetBits(G3Port,G3Pin);
	
	GPIO_Configuration_OPP50(V24Port,V24Pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	//GPIO_SetBits(V24Port,V24Pin);
	GPIO_ResetBits(V24Port,V24Pin);
	
	GPIO_Configuration_OPP50(V05Port,V05Pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	//GPIO_SetBits(V05Port,V05Pin);
	GPIO_ResetBits(V05Port,V05Pin);
	
	GPIO_Configuration_OPP50(MtxPort,MtxPin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	//GPIO_SetBits(MtxPort,MtxPin);
	GPIO_ResetBits(MtxPort,MtxPin);
	
	GPIO_Configuration_OPP50(MrxPort,MrxPin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	//GPIO_SetBits(MrxPort,MrxPin);
	GPIO_ResetBits(MrxPort,MrxPin);
	
#endif
}
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_CMD(FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
//    GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
		GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
  else
  {
//    GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
		GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
}
unsigned long sendnum=0;
unsigned long sendednum=0;

extern unsigned long usbsavenum;

/*******************************************************************************
* Function Name  :  UART0_Config_Default.
* Description    :  configure the UART 0 with default values.	串口的默认配置值
* Input          :  None.
* Return         :  None.
*******************************************************************************/
void USART_Config_Default(void)
{
	USART_InitTypeDef USART_InitStructure;
//--------------原程序
//	linecoding
	USART_DeInit(ComPort);
	
	USART_InitStructure.USART_BaudRate    = 19200; 					  //波特率
	USART_InitStructure.USART_WordLength  = USART_WordLength_8b;		    //数据位
	USART_InitStructure.USART_StopBits    = USART_StopBits_1;				    //停止位
	USART_InitStructure.USART_Parity      = USART_Parity_No ; 					//奇偶校验
	USART_InitStructure.USART_Mode        = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//流控
	USART_Init(ComPort, &USART_InitStructure);											//初始化串口
	
	api_usart_dma_configurationST(ComPort,&USART_InitStructure,USB_BUFFER_SIZE);	//USART_DMA配置--结构体形式，不开中断	
}

/*******************************************************************************
* Function Name  :  UART0_Config.
* Description    :  Configure the UART 1 according to the linecoding structure.
										根据line coding 结构体配置串口.
* Input          :  None.
* Return         :  Configuration status
                    TRUE : configuration done with success
                    FALSE : configuration aborted.
*******************************************************************************/
char USART_Config(void)
{
	USART_InitTypeDef USART_InitStructure;
  /* set the Stop bit*/
	/**************设置停止位**************/
  switch (linecoding.format)
  {
    case 0:
      USART_InitStructure.USART_StopBits = USART_StopBits_1;		//1位停止位
      break;
    case 1:
      USART_InitStructure.USART_StopBits = USART_StopBits_1_5;	//1.5为停止位
      break;
    case 2:
      USART_InitStructure.USART_StopBits = USART_StopBits_2;		//2位停止位
      break;
    default :
    {
      USART_Config_Default();																		//默认配置
      return (0);
    }
  }

  /* set the parity bit*/
	/**************设置校验位**************/
  switch (linecoding.paritytype)
  {
    case 0:
      USART_InitStructure.USART_Parity = USART_Parity_No;			//没有校验
      break;
    case 1:
      USART_InitStructure.USART_Parity = USART_Parity_Even;		//偶校验
      break;	
    case 2:
      USART_InitStructure.USART_Parity = USART_Parity_Odd;		//奇校验
      break;
    default :
    {
      USART_Config_Default();																	//默认配置
      return (0);
    }
  }

  /*set the data type : only 8bits and 9bits is supported */
	/**************设置数据位: 8位或9位**************/
  switch (linecoding.datatype)
  {
    case 0x07:
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//8为数据位，这个选项就校验位必须设置(奇校验/偶校验)
      break;
    case 0x08:
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//USART_WordLength_9b;	//9位数据位
      break;
    default :
    {
      USART_Config_Default();																//默认配置
      return (0);
    }
  }
  USART_InitStructure.USART_BaudRate = linecoding.bitrate;													//设置波特率
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		//设置没有硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;										//使能接收、发送
  USART_Init(ComPort, &USART_InitStructure);																				//初始化串口
  USART_Cmd(ComPort, ENABLE);																												//使能串口
	
	//api_usart_dma_configurationNR(ComPort,linecoding.bitrate,usart_buffer_size);
	api_usart_dma_configurationST(ComPort,&USART_InitStructure,USB_BUFFER_SIZE);	//USART_DMA配置--结构体形式，不开中断
	
  return (1);
}
/*******************************************************************************
* Function Name  : USB_To_UART_Send_Data.
* Description    : send the received data from USB to the UART 0.将USB接收到的数据从串口发送数据
* Input          : data_buffer: data address.
                   Nb_bytes: number of bytes to send.
* Return         : none.
*******************************************************************************/
void USB_To_USART_Send_Data(u8* data_buffer, u8 Nb_bytes)
{
	Usart_tx_flg=1;
	api_usart_dma_send(ComPort,data_buffer,(u16)Nb_bytes);		//自定义printf串口DMA发送程序
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
void usb_to_uart_server(void)
{
	unsigned short len	=	0;
	static unsigned char time=0;
	unsigned char buffer[USB_BUFFER_SIZE]={0};
	if(bDeviceState != CONFIGURED)
  {
		return;
	}
	if(0==get_usart_tx_idle(ComPort))		//串口状态检查
	{
		return ;
	}
	//-----------------------------------无数据
	if(0!=api_usb_out_get_complete_flag())
	{
		return;
	}
	if(time++<10)		//连续帧间隔8ms
		return ;
	time = 0;

	len	=	api_usb_out_get_data(buffer);
	if(len)
	{
		sendnum+=len;
		sendednum+=api_usart_dma_send(ComPort,buffer,(u16)len);		//自定义printf串口DMA发送程序
	}
}
/*******************************************************************************
* Function Name  : UART_To_USB_Send_Data.
* Description    : send the received data from UART 0 to USB.	发送串口接收到的数据到USB
* Input          : None.
* Return         : none.
*******************************************************************************/
void USART_To_USB_Send_Data(void)
{	
	u16	num	=	api_usart_dma_receive(ComPort,buffer_in);
	if(num)
	{
		api_usb_in_set_data(buffer_in,num);
	}
}
/*******************************************************************************
* Function Name  : Handle_USBAsynchXfer.
* Description    : send data to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void usb_virtual_com_AsynchXfer (void)
{
//	unsigned short count_in	=	0;
//	count_in=api_usart_dma_receive(ComPort,buffer_rx);
//	if(count_in)
//	{
//		UserToPMABufferCopy(buffer_rx, ENDP1_TXADDR, count_in);
//		SetEPTxCount(ENDP1, count_in);
//		SetEPTxValid(ENDP1);
//	}
}
//------------------------------------------------------------------------------




//-----------------------------------usb_prop-----------------------------------


static unsigned char Request = 0;

void Virtual_Com_Port_init(void);
void Virtual_Com_Port_Reset(void);
void Virtual_Com_Port_SetConfiguration(void);
void Virtual_Com_Port_SetDeviceAddress (void);
void Virtual_Com_Port_Status_In (void);
void Virtual_Com_Port_Status_Out (void);
RESULT Virtual_Com_Port_Data_Setup(u8);
RESULT Virtual_Com_Port_NoData_Setup(u8);
RESULT Virtual_Com_Port_Get_Interface_Setting(u8 Interface, u8 AlternateSetting);
u8 *Virtual_Com_Port_GetDeviceDescriptor(u16 );
u8 *Virtual_Com_Port_GetConfigDescriptor(u16);
u8 *Virtual_Com_Port_GetStringDescriptor(u16);

u8 *Virtual_Com_Port_GetLineCoding(u16 Length);	//获取串口通信信息
u8 *Virtual_Com_Port_SetLineCoding(u16 Length);	//设置串口通讯信息

LINE_CODING linecoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* no. of bits 8*/
  };

/* -------------------------------------------------------------------------- */
/*  Structures initializations */
/* -------------------------------------------------------------------------- */

DEVICE Device_Table =
  {
    EP_NUM,				//被使用的端点数
    1							//可以使用的端点数
  };

/*设备属性*/
DEVICE_PROP Device_Property =								//
  {
    Virtual_Com_Port_init,									//Virtual_Com_Port的初始化函数
    Virtual_Com_Port_Reset,									//Virtual_Com_Port的复位函数
    Virtual_Com_Port_Status_In,							//CustomVirtual_Com_PortHID状态输入函数
    Virtual_Com_Port_Status_Out,						//CustomHID状态输出函数
    Virtual_Com_Port_Data_Setup,						//CustomHID的处理有数据阶段的特殊类请求函数
    Virtual_Com_Port_NoData_Setup,					//CustomHID的处理没有数据阶段的特殊类请求函数
    Virtual_Com_Port_Get_Interface_Setting,	//CustomHID获取接口及备用接口设置（是否可用）
    Virtual_Com_Port_GetDeviceDescriptor,		//CustomHID获取设备描述符
    Virtual_Com_Port_GetConfigDescriptor,		//CustomHID获取配置描述符
    Virtual_Com_Port_GetStringDescriptor,		//CustomHID获取字符串描述符
    0,																			//当前库未使用
    0x40 /*MAX PACKET SIZE*/								//最大的包长度为64字节
  };

/*注册USB标准请求的实现函数*/
USER_STANDARD_REQUESTS User_Standard_Requests =		
  {
    Virtual_Com_Port_GetConfiguration,			//获取配置请求
    Virtual_Com_Port_SetConfiguration,			//设置配置请求
    Virtual_Com_Port_GetInterface,					//获取接口请求
    Virtual_Com_Port_SetInterface,					//设置接口请求
    Virtual_Com_Port_GetStatus,							//获取状态请求
    Virtual_Com_Port_ClearFeature,					//清除属性请求
    Virtual_Com_Port_SetEndPointFeature,		//设置端点属性请求
    Virtual_Com_Port_SetDeviceFeature,			//设置设备属性请求
    Virtual_Com_Port_SetDeviceAddress				//设置设备地址请求
  };

/*注册设备描述符信息*/
ONE_DESCRIPTOR Device_Descriptor =
  {
    (u8*)Virtual_Com_Port_DeviceDescriptor,	//注册设备描述符数组
    VIRTUAL_COM_PORT_SIZ_DEVICE_DESC				//设备描述符的长度
  };
	
/*注册设备描述符信息*/
ONE_DESCRIPTOR Qualifier_Descriptor =
  {
    (u8*)USBD_DeviceQualifier,	//注册设备描述符数组
    10				//设备描述符的长度
  };

/*注册配置描述符信息*/
ONE_DESCRIPTOR Config_Descriptor =
  {
    (u8*)Virtual_Com_Port_ConfigDescriptor,										//注册配置描述符数组
//		sizeof((char*)Virtual_Com_Port_ConfigDescriptor)
    VIRTUAL_COM_PORT_SIZ_CONFIG_DESC												//配置描述符的长度
  };

/*注册字符串描述符，包括语言ID、厂商、产品、序列号描述符*/
ONE_DESCRIPTOR String_Descriptor[4] =
  {
    {(u8*)Virtual_Com_Port_StringLangID, VIRTUAL_COM_PORT_SIZ_STRING_LANGID},		//注册语言字符串描述符数组
    {(u8*)Virtual_Com_Port_StringVendor, VIRTUAL_COM_PORT_SIZ_STRING_VENDOR},		//注册厂商字符串描述符数组
    {(u8*)Virtual_Com_Port_StringProduct, VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT},	//注册产品字符串描述符数组
    {(u8*)Virtual_Com_Port_StringSerial, VIRTUAL_COM_PORT_SIZ_STRING_SERIAL}		//注册序列号字符串描述符数组
  };

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : Virtual_Com_Port_init.
* Description    : Virtual COM Port Mouse init routine.	初始化
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Virtual_Com_Port_init(void)
{

  /* Update the serial number string descriptor with the data from the unique
  ID*/
  Get_SerialNum();	//**************************************获取序列号,设置芯片序列号，将描述符中的例如STM 等字符串修改没太大意义。

  pInformation->Current_Configuration = 0;	//配置状态

  /* Connect the device */
  PowerOn();				//*******上电usb_pwr.c->Line59:开启USB上拉，强制USB复位，开启相关中断使能标志；CNTR_RESETM | CNTR_SUSPM | CNTR_WKUPM;		//允许以下中断
  /* USB interrupts initialization */
  /* clear pending interrupts */
  _SetISTR(0);			//**************************************执行基本的初始化操作，比如说设备IP和端点0的初始化
  wInterrupt_Mask = IMR_MSK;		//#define IMR_MSK (CNTR_CTRM  | CNTR_SOFM  | CNTR_RESETM )
																//CNTR_CTRM：正确传输(CTR)中断屏蔽位 (Correct transfer interrupt mask)
																//CNTR_SOFM：帧首中断屏蔽位 (Start of frame interrupt mask)
																//CNTR_RESETM：USB复位中断屏蔽位 (USB reset interrupt mask)
  /* set interrupts mask */
  _SetCNTR(wInterrupt_Mask);		////使能相应中断

  /* configure the USART 1 to the default settings */
  USART_Config_Default();		//配置串口至缺省状态---在这里波特率被设为9600，并且允许了接收中断。发送中断没有允许。

  bDeviceState = UNCONNECTED;		//usb_pwr.h->DEVICE_STATE**************************将当前的状态定义为未连接状态
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Reset
* Description    : Virtual_Com_Port Mouse reset routine	复位
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Virtual_Com_Port_Reset(void)
{
  /* Set Virtual_Com_Port DEVICE as not configured */
  pInformation->Current_Configuration = 0;				//设置当前的配置为0，表示没有配置过

  /* Current Feature initialization */
  pInformation->Current_Feature = Virtual_Com_Port_ConfigDescriptor[7];	//当前的属性，bmAttributes:设备的一些特性，0xc0表示自供电，不支持远程唤醒

  /* Set Virtual_Com_Port DEVICE with the default Interface*/
  pInformation->Current_Interface = 0;
  SetBTABLE(BTABLE_ADDRESS);

  /* Initialize Endpoint 0 */
	/* *****初始化端点0***** */
  SetEPType(ENDP0, EP_CONTROL);													//设置端点0为控制端点
  SetEPTxStatus(ENDP0, EP_TX_STALL);										//设置端点0发送延时
  SetEPRxAddr(ENDP0, ENDP0_RXADDR);											//设置端点0的接收缓冲区地址
  SetEPTxAddr(ENDP0, ENDP0_TXADDR);											//设置端点0的发送缓冲区地址
  Clear_Status_Out(ENDP0);															//清除端点0的状态
  SetEPRxCount(ENDP0, Device_Property.MaxPacketSize);		//设置端点0的接收最大包
  SetEPRxValid(ENDP0);																	//使能接收状态

  /* Initialize Endpoint 1 */
	/* *****初始化端点1***** */
  SetEPType(ENDP1, EP_BULK);														//设置端点1为进批量传输
  SetEPTxAddr(ENDP1, ENDP1_TXADDR);											//设置端点发送地址
  SetEPTxStatus(ENDP1, EP_TX_NAK);											//设置端点1的发送不响应
  SetEPRxStatus(ENDP1, EP_RX_DIS);											//设置端点1不接收

  /* Initialize Endpoint 2 */
	/* *****初始化端点2***** */
  SetEPType(ENDP2, EP_INTERRUPT);												//设置端点2为中断传输
  SetEPTxAddr(ENDP2, ENDP2_TXADDR);											//设置端点2发送地址
  SetEPRxStatus(ENDP2, EP_RX_DIS);											//设置端点2不接收状态
  SetEPTxStatus(ENDP2, EP_TX_NAK);											//设置端点2端点2为接收不响应

  /* Initialize Endpoint 3 */
	/* *****初始化端点3***** */
  SetEPType(ENDP3, EP_BULK);														//设置端点3为仅批量传输
  SetEPRxAddr(ENDP3, ENDP3_RXADDR);											//设置端点3接收地址
  SetEPRxCount(ENDP3, VIRTUAL_COM_PORT_DATA_SIZE);			//设置端点3的计数值
  SetEPRxStatus(ENDP3, EP_RX_VALID);										//设置端点3接收有效
  SetEPTxStatus(ENDP3, EP_TX_DIS);											//设置端点3不发送

  /* Set this device to response on default address */
  SetDeviceAddress(0);																	//设置设备为默认地址为0

  bDeviceState = ATTACHED;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetConfiguration.
* Description    : Udpade the device state to configured.更新设备配置状态
* 描述			    	: 配置
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Virtual_Com_Port_SetConfiguration(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetConfiguration.
* Description    : Udpade the device state to addressed.更新设备的编址状态
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Virtual_Com_Port_SetDeviceAddress (void)
{
  bDeviceState = ADDRESSED;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Status_In.
* Description    : Virtual COM Port Status In Routine.状态输入函数
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Virtual_Com_Port_Status_In(void)
{
  if (Request == SET_LINE_CODING)
  {
    USART_Config();
    Request = 0;
  }
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Status_Out
* Description    : Virtual COM Port Status OUT Routine.状态输出函数
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Virtual_Com_Port_Status_Out(void)
{}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Data_Setup
* Description    : handle the data class specific requests 处理有数据阶段的特殊类请求
* Input          : Request Nb.
* Output         : None.
* Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
RESULT Virtual_Com_Port_Data_Setup(u8 RequestNo)
{
  u8    *(*CopyRoutine)(u16);

  CopyRoutine = NULL;
	//请求的字段
  if (RequestNo == GET_LINE_CODING)		//****************************获取串口通信信息请求														
  {
    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))	//类请求，请求的接收者是接口
    {
      CopyRoutine = Virtual_Com_Port_GetLineCoding;	//**************指针函数指向Virtual_Com_Port_GetLineCoding函数
    }
  }
  else if (RequestNo == SET_LINE_CODING)	//************************设置串口通讯信息请求
  {
    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))	//类请求，请求的接收者是接口
    {
      CopyRoutine = Virtual_Com_Port_SetLineCoding;	//**************指针函数指向Virtual_Com_Port_SetLineCoding函数
    }
    Request = SET_LINE_CODING;
  }

  if (CopyRoutine == NULL)
  {
    return USB_UNSUPPORT;
  }

  pInformation->Ctrl_Info.CopyData = CopyRoutine;		//**************注册指针指向的函数
  pInformation->Ctrl_Info.Usb_wOffset = 0;
  (*CopyRoutine)(0);					//************************************调用该函数
  return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_NoData_Setup.
* Description    : handle the no data class specific requests.处理没有数据阶段特殊类请求
* Input          : Request Nb.
* Output         : None.
* Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
RESULT Virtual_Com_Port_NoData_Setup(u8 RequestNo)
{

  if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))	//***类请求，请求的接收者是接口
  {
    if (RequestNo == SET_COMM_FEATURE)		//*************************设置虚拟串口特性
    {
      return USB_SUCCESS;
    }
    else if (RequestNo == SET_CONTROL_LINE_STATE)		//***************设置控制信息状态
    {
      return USB_SUCCESS;
    }
  }

  return USB_UNSUPPORT;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetDeviceDescriptor.
* Description    : Gets the device descriptor. 	数据类请求--获取设备描述符
* Input          : Length.
* Output         : None.
* Return         : The address of the device descriptor.
*******************************************************************************/
u8 *Virtual_Com_Port_GetDeviceDescriptor(u16 Length)
{
  return Standard_GetDescriptorData(Length, &Device_Descriptor);
}
/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetDeviceDescriptor.
* Description    : Gets the device descriptor. 	数据类请求--获取设备描述符
* Input          : Length.
* Output         : None.
* Return         : The address of the device descriptor.
*******************************************************************************/
u8 *Qualifier_GetDeviceDescriptor(u16 Length)
{
  return Standard_GetDescriptorData(10,&Qualifier_Descriptor);
}
/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetConfigDescriptor.
* Description    : get the configuration descriptor.数据类请求--获取配置描述符
* Input          : Length.
* Output         : None.
* Return         : The address of the configuration descriptor.
*******************************************************************************/
u8 *Virtual_Com_Port_GetConfigDescriptor(u16 Length)
{
  return Standard_GetDescriptorData(Length, &Config_Descriptor);
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetStringDescriptor
* Description    : Gets the string descriptors according to the needed index	数据类请求--根据索引获取字符描述符
* Input          : Length.
* Output         : None.
* Return         : The address of the string descriptors.
*******************************************************************************/
u8 *Virtual_Com_Port_GetStringDescriptor(u16 Length)
{
  u8 wValue0 = pInformation->USBwValue0;
  if (wValue0 > 4)
  {
    return NULL;
  }
  else
  {
    return Standard_GetDescriptorData(Length, &String_Descriptor[wValue0]);
  }
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Get_Interface_Setting.
* Description    : test the interface and the alternate setting according to the
*                  supported one.		测试接口及其备用接口是否可用
* Input1         : u8: Interface : interface number.
* Input2         : u8: AlternateSetting : Alternate Setting number.
* Output         : None.
* Return         : The address of the string descriptors.
*******************************************************************************/
RESULT Virtual_Com_Port_Get_Interface_Setting(u8 Interface, u8 AlternateSetting)
{
  if (AlternateSetting > 0)		//备用的编号大于0
  {
    return USB_UNSUPPORT;
  }
  else if (Interface > 1)			//接口的编号大于1
  {
    return USB_UNSUPPORT;
  }
  return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetLineCoding.
* Description    : send the linecoding structure to the PC host.	发送linecoding结构体到PC机
* Input          : Length.
* Output         : None.
* Return         : Inecoding structure base address.  Linecoding结构体的基地址
*******************************************************************************/
u8 *Virtual_Com_Port_GetLineCoding(u16 Length)
{
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength = sizeof(linecoding);
    return NULL;
  }
  return(u8 *)&linecoding;			//串口通信信息
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetLineCoding.
* Description    : Set the linecoding structure fields.	设置linecoding结构体
* Input          : Length.
* Output         : None.
* Return         : Linecoding structure base address.		Linecoding结构体的基地址.
*******************************************************************************/
u8 *Virtual_Com_Port_SetLineCoding(u16 Length)
{
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength = sizeof(linecoding);
    return NULL;
  }
  return(u8 *)&linecoding;
}


#endif
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
