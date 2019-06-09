/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Virtual Com Port Demo main file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

#ifdef Virtual_COM_Port
	#include "Virtual_COM_Port.H"
	

	
	
	#include "STM32_SYS.H"
	#include "STM32_PWM.H"
	#include "STM32_USART.H"
	#include "STM32_GPIO.H"
	
	

/* Includes ------------------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_TypeDef*		usb_en_port;
unsigned short	usb_en_pin;

u8 buffer_in[VIRTUAL_COM_PORT_DATA_SIZE];
u8 buffer_rx[VIRTUAL_COM_PORT_DATA_SIZE];
u8 buffer_tx[VIRTUAL_COM_PORT_DATA_SIZE];
u32 USART_Rx_ptr_in = 0;
u32 USART_Rx_ptr_out = 0;
u32 USART_Rx_length  = 0;
u8  USB_Tx_State = 0;
/* Extern variables ----------------------------------------------------------*/
extern LINE_CODING linecoding;
extern u32 count_in;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Virtual_COM_Port_Configuration(void)
{	
	usb_en_def usb_en;		//USB使能控制端口
	
	SYS_Configuration();
	
	usb_en.usb_connect_port	=	(unsigned long*)USB_DISCONNECT;
	usb_en.usb_connect_pin	=	USB_DISCONNECT_PIN;
	
	PWM_OUT(TIM2,PWM_OUTChannel1,2,500);	//PWM设定-20161127版本	占空比1/1000
	
	api_usb_virtual_com_configuration(&usb_en);		//虚拟串口配置
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*******************************************************************************/
void Virtual_COM_Port_Server(void)
{

}
//------------------------------------------------------------------------------


/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
void api_usb_virtual_com_configuration(usb_en_def* pInfo)		//虚拟串口配置
{	
	usb_en_port	=	(GPIO_TypeDef*)pInfo->usb_connect_port;
	usb_en_pin	=	pInfo->usb_connect_pin;
	
	GPIO_Configuration_OPP50(usb_en_port,usb_en_pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	
	usb_hw_set_connect(DISABLE);	//关闭USB上拉电阻
	
	usb_hw_set_clock();			//设置USB时钟
	
	usb_hw_set_Interrupt();	//设置USB中断
	
	USB_Init();						//USB初始化
}
//------------------------------------------------------------------------------





//------------------------------------usb_hw(hw_config)-------------------------
/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : Status
*******************************************************************************/
void usb_hw_set_connect(FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
		//GPIO_ResetBits(usb_en_port, usb_en_pin);
		GPIO_SetBits(usb_en_port, usb_en_pin);
  }
  else
  {
		//GPIO_SetBits(usb_en_port, usb_en_pin);
		GPIO_ResetBits(usb_en_port, usb_en_pin);
  }
}
/*******************************************************************************
* Function Name  :  UART0_Config_Default.
* Description    :  configure the UART 0 with default values.	串口的默认配置值
* Input          :  None.
* Return         :  None.
*******************************************************************************/
void usb_hw_set_usart_default(void)
{
	//linecoding
	api_usart_dma_configurationNR(ComPort,linecoding.bitrate,usart_buffer_size);
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
bool usb_hw_set_usart_config(void)
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
      usb_hw_set_usart_default();																			//默认配置
      return (FALSE);
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
      usb_hw_set_usart_default();																	//默认配置
      return (FALSE);
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
      usb_hw_set_usart_default();																//默认配置
      return (FALSE);
    }
  }
  USART_InitStructure.USART_BaudRate = linecoding.bitrate;													//设置波特率
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		//设置没有硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;										//使能接收、发送
  USART_Init(ComPort, &USART_InitStructure);																				//初始化串口
  USART_Cmd(ComPort, ENABLE);																												//使能串口
	
	//api_usart_dma_configurationNR(ComPort,linecoding.bitrate,usart_buffer_size);
	api_usart_dma_configurationST(ComPort,&USART_InitStructure,usart_buffer_size);	//USART_DMA配置--结构体形式，不开中断
	
  return (TRUE);
}
/*******************************************************************************
* Function Name  : USB_To_UART_Send_Data.
* Description    : send the received data from USB to the UART 0.将USB接收到的数据从串口发送数据
* Input          : data_buffer: data address.
                   Nb_bytes: number of bytes to send.
* Return         : none.
*******************************************************************************/
void usb_hw_receive_from_usb(u8* data_buffer, u8 Nb_bytes)
{
	api_usart_dma_send(ComPort,data_buffer,(u16)Nb_bytes);		//自定义printf串口DMA发送程序
}
/*******************************************************************************
* Function Name  : UART_To_USB_Send_Data.
* Description    : send the received data from UART 0 to USB.	发送串口接收到的数据到USB
* Input          : None.
* Return         : none.
*******************************************************************************/
void usb_hw_send_to_usb(void)
{	
	u16	num	=	api_usart_dma_receive(ComPort,buffer_in);
	if(num)
	{
		UserToPMABufferCopy(buffer_in, ENDP1_TXADDR, num);
		SetEPTxCount(ENDP1, num);																					//设置端点数据长度
		SetEPTxValid(ENDP1);																							//使能端点
	}
}
/*******************************************************************************
* Function Name  : Handle_USBAsynchXfer.
* Description    : send data to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void usb_hw_Handle_USBAsynchXfer(void)
{
	count_in=api_usart_dma_receive(ComPort,buffer_rx);
	if(count_in)
	{
		UserToPMABufferCopy(buffer_rx, ENDP1_TXADDR, count_in);
		SetEPTxCount(ENDP1, count_in);
		SetEPTxValid(ENDP1);
	}
}
//-----------------------------------------------------------------------------static
/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void usb_hw_set_clock(void)
{
  /* USBCLK = PLLCLK / 1.5 */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  /* Enable USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}
/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void usb_hw_set_Interrupt(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM
  /* Set the Vector Table base location at 0x20000000 */
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH */
  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN_RX0_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

//  /* Enable USART1 Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  NVIC_Init(&NVIC_InitStructure);
}
//------------------------------------------------------------------------------





//------------------------------------usb_prop----------------------------------
u8 Request = 0;			//请求
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
DEVICE_PROP Device_Property =				//
  {
    usb_prop_init,									//Virtual_Com_Port的初始化函数
    usb_prop_reset,									//Virtual_Com_Port的复位函数
    usb_prop_status_in,							//CustomVirtual_Com_PortHID状态输入函数
    usb_prop_status_out,						//CustomHID状态输出函数
    usb_prop_data_setup,						//CustomHID的处理有数据阶段的特殊类请求函数
    usb_prop_no_data_setup,					//CustomHID的处理没有数据阶段的特殊类请求函数
    usb_prop_get_interface_setting,	//CustomHID获取接口及备用接口设置（是否可用）
    usb_prop_get_device_descriptor,		//CustomHID获取设备描述符
    usb_prop_get_config_descriptor,		//CustomHID获取配置描述符
    Virtual_Com_Port_GetStringDescriptor,		//CustomHID获取字符串描述符
    0,																			//当前库未使用
    0x40 /*MAX PACKET SIZE*/								//最大的包长度为64字节
  };

/*注册USB标准请求的实现函数*/
USER_STANDARD_REQUESTS User_Standard_Requests =		
  {
    Virtual_Com_Port_GetConfiguration,			//获取配置请求
    usb_prop_set_configuration,			//设置配置请求
    Virtual_Com_Port_GetInterface,					//获取接口请求
    Virtual_Com_Port_SetInterface,					//设置接口请求
    Virtual_Com_Port_GetStatus,							//获取状态请求
    Virtual_Com_Port_ClearFeature,					//清除属性请求
    Virtual_Com_Port_SetEndPointFeature,		//设置端点属性请求
    Virtual_Com_Port_SetDeviceFeature,			//设置设备属性请求
    usb_prop_set_device_address				//设置设备地址请求
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
* Description    : Virtual COM Port Mouse init routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void usb_prop_init(void)
{

  /* Update the serial number string descriptor with the data from the unique  ID*/
  Get_SerialNum();	//**************************************获取序列号,设置芯片序列号，将描述符中的例如STM 等字符串修改没太大意义。

  pInformation->Current_Configuration = 0;	//配置状态

  /* Connect the device */
  PowerOn();				//*******上电usb_pwr.c->Line59:开启USB上拉，强制USB复位，开启相关中断使能标志；CNTR_RESETM | CNTR_SUSPM | CNTR_WKUPM;		//允许以下中断
	//---------------------------------USB中断初始化
	//-----------------清除所有中断标识
  _SetISTR(0);
	//-----------------设置中断标识
  wInterrupt_Mask = IMR_MSK;		//#define IMR_MSK (CNTR_CTRM  | CNTR_SOFM  | CNTR_RESETM )
																//CNTR_CTRM：正确传输(CTR)中断屏蔽位 (Correct transfer interrupt mask)
																//CNTR_SOFM：帧首中断屏蔽位 (Start of frame interrupt mask)
																//CNTR_RESETM：USB复位中断屏蔽位 (USB reset interrupt mask)
  //-----------------使能相应中断
  _SetCNTR(wInterrupt_Mask);		////使能相应中断
	//-----------------更新USB状态为未连接
	bDeviceState = UNCONNECTED;		//usb_pwr.h->DEVICE_STATE**************************将当前的状态定义为未连接状态
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Reset
* Description    : Virtual_Com_Port Mouse reset routine	复位
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void usb_prop_reset(void)
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

  bDeviceState = ATTACHED;		//状态--已插入USB设备
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetConfiguration.
* Description    : Udpade the device state to configured.更新设备配置状态
* 描述			    	: 配置
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void usb_prop_set_configuration(void)
{
	DEVICE_INFO *pInfo = pInformation;
	
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;	//状态--已配置
  }
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetConfiguration.
* Description    : Udpade the device state to addressed.更新设备的编址状态
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void usb_prop_set_device_address(void)
{
  bDeviceState = ADDRESSED;	//状态--已分配地址
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Status_In.
* Description    : Virtual COM Port Status In Routine.状态输入函数
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void usb_prop_status_in(void)
{
  if (Request == SET_LINE_CODING)
  {
    usb_hw_set_usart_config();
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
void usb_prop_status_out(void)
{
}
/*******************************************************************************
* Function Name  : Virtual_Com_Port_Data_Setup
* Description    : handle the data class specific requests 处理有数据阶段的特殊类请求
* Input          : Request Nb.
* Output         : None.
* Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
RESULT usb_prop_data_setup(u8 RequestNo)
{
  u8    *(*CopyRoutine)(u16);

  CopyRoutine = NULL;
	//获取LineCoding地址：
  if (RequestNo == GET_LINE_CODING)		//****************************获取串口通信信息请求														
  {
    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))	//类请求，请求的接收者是接口
    {
      CopyRoutine = Virtual_Com_Port_GetLineCoding;	//**************指针函数指向Virtual_Com_Port_GetLineCoding函数
    }
  }
	//写入LineCoding地址：
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
RESULT usb_prop_no_data_setup(u8 RequestNo)
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
u8 *usb_prop_get_device_descriptor(u16 Length)
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
u8 *usb_prop_get_config_descriptor(u16 Length)
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
RESULT usb_prop_get_interface_setting(u8 Interface, u8 AlternateSetting)
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
/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  u32 Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(vu32*)(0x1FFFF7E8);
  Device_Serial1 = *(vu32*)(0x1FFFF7EC);
  Device_Serial2 = *(vu32*)(0x1FFFF7F0);

  if (Device_Serial0 != 0)
  {
    Virtual_Com_Port_StringSerial[2] 	= (u8)(Device_Serial0 & 0x000000FF);
    Virtual_Com_Port_StringSerial[4] 	= (u8)((Device_Serial0 & 0x0000FF00) >> 8);
    Virtual_Com_Port_StringSerial[6] 	= (u8)((Device_Serial0 & 0x00FF0000) >> 16);
    Virtual_Com_Port_StringSerial[8] 	= (u8)((Device_Serial0 & 0xFF000000) >> 24);

    Virtual_Com_Port_StringSerial[10] = (u8)(Device_Serial1 & 0x000000FF);
    Virtual_Com_Port_StringSerial[12] = (u8)((Device_Serial1 & 0x0000FF00) >> 8);
    Virtual_Com_Port_StringSerial[14] = (u8)((Device_Serial1 & 0x00FF0000) >> 16);
    Virtual_Com_Port_StringSerial[16] = (u8)((Device_Serial1 & 0xFF000000) >> 24);

    Virtual_Com_Port_StringSerial[18] = (u8)(Device_Serial2 & 0x000000FF);
    Virtual_Com_Port_StringSerial[20] = (u8)((Device_Serial2 & 0x0000FF00) >> 8);
    Virtual_Com_Port_StringSerial[22] = (u8)((Device_Serial2 & 0x00FF0000) >> 16);
    Virtual_Com_Port_StringSerial[24] = (u8)((Device_Serial2 & 0xFF000000) >> 24);
  }
}
//------------------------------------usb_istr----------------------------------



#include "usb_istr.h"
volatile u8 bIntPackSOF = 0;  /* SOFs received between 2 consecutive packets */
void (*pEpInt_IN[7])(void) =
  {
    usb_endp_in_1,
    EP2_IN_Callback,
    EP3_IN_Callback,
    EP4_IN_Callback,
    EP5_IN_Callback,
    EP6_IN_Callback,
    EP7_IN_Callback,
  };

void (*pEpInt_OUT[7])(void) =
  {
    EP1_OUT_Callback,
    EP2_OUT_Callback,
    usb_endp_out,
    EP4_OUT_Callback,
    EP5_OUT_Callback,
    EP6_OUT_Callback,
    EP7_OUT_Callback,
  };
/*******************************************************************************
* Function Name  : USB_Istr.
* Description    : ISTR events interrupt service routine.
*	描述说明				:		中断事件处理-根据中断状态寄存器判断中断类型，转入相应的处理函数
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_Istr(void)
{

  USB_REG.wIstr.istr	=	_GetISTR();	//wIstr = _GetISTR();			//获取中断标志，得到中断源
	//--------------------------USB复位请求 (USB reset request)
#if (IMR_MSK & ISTR_RESET)
  if (USB_REG.wIstr.istr & ISTR_RESET & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_RESET);		//清除USB复位请求 (USB reset request)
		pProperty->Reset();					//USB复位
#ifdef RESET_CALLBACK
    RESET_Callback();
#endif
  }
#endif
  //--------------------------PMA缓存溢出(Packet memory area over / underrun)
#if (IMR_MSK & ISTR_DOVR)
  if (USB_REG.wIstr.istr & ISTR_DOVR & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_DOVR);	//清除分组缓冲区溢出 (Packet memory area over / underrun)
#ifdef DOVR_CALLBACK
    DOVR_Callback();
#endif
  }
#endif
  //--------------------------传输出错：USB应用程序通常可以忽略这些错误，因为USB模块和主机在发生错误时都会启动重传机制。
	//													此位产生的中断可以用于应用程序的开发阶段，可以用来监测USB总线的传输质量，标识用户可能发生的错误(连接线松，环境干扰严重，USB线损坏等)。
#if (IMR_MSK & ISTR_ERR)			//出错 (Error)
  if (USB_REG.wIstr.istr & ISTR_ERR & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_ERR);		//清除出错 (Error)
#ifdef ERR_CALLBACK
    ERR_Callback();
#endif
  }
#endif
  //--------------------------唤醒请求 (Wakeup)
#if (IMR_MSK & ISTR_WKUP)
  if (USB_REG.wIstr.istr & ISTR_WKUP & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_WKUP);	//清除唤醒请求 (Wakeup)
    Resume(RESUME_EXTERNAL);
#ifdef WKUP_CALLBACK
    WKUP_Callback();
#endif
  }
#endif
  //--------------------------挂起模块请求 (Suspend mode request)
#if (IMR_MSK & ISTR_SUSP)
  if (USB_REG.wIstr.istr & ISTR_SUSP & wInterrupt_Mask)
  {
    /* check if SUSPEND is possible */
    if (fSuspendEnabled)
    {
      Suspend();
    }
    else
    {
      /* if not possible then resume after xx ms */
      Resume(RESUME_LATER);
    }
    /* clear of the ISTR bit must be done after setting of CNTR_FSUSP */
    _SetISTR((u16)CLR_SUSP);
#ifdef SUSP_CALLBACK
    SUSP_Callback();
#endif
  }
#endif
  //--------------------------//帧首(SOF)中断标志，中断服务程序可以通过检测SOF事件来完成与主机的1ms同
#if (IMR_MSK & ISTR_SOF)										
  if (USB_REG.wIstr.istr & ISTR_SOF & wInterrupt_Mask)		//读出的中断标志是SOF中断标志，且SOF中断使能了
  {
    _SetISTR((u16)CLR_SOF);									//清除SOF中断标志
    bIntPackSOF++;													//统计共接收到多少SOF

#ifdef SOF_CALLBACK
    SOF_Callback();													//当定义了SOF_CALLBACK，则调用SOF_Callback,像钩子函数一样，在发生SOF中断时做点什么
#endif
  }
#endif
  //--------------------------期望帧首标识位 (Expected start of frame)	如果连续发生3次ESOF中断，也就是连续3次未收到SOF分组，将产生SUSP中断。即使在挂起定时器未被锁定时发生SOF分组丢失，此位也会被置位。
#if (IMR_MSK & ISTR_ESOF)								
  if (USB_REG.wIstr.istr & ISTR_ESOF & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_ESOF);
    /* resume handling timing is made with ESOFs */
    Resume(RESUME_ESOF); /* request without change of the machine state */

#ifdef ESOF_CALLBACK
    ESOF_Callback();
#endif
  }
#endif
  //--------------------------正确的传输 (Correct transfer)：此位在端点正确完成一次数据传输后由硬件置位。应用程序可以通过DIR和EP_ID位来识别是哪个端点完成了正确的数据传输。
#if (IMR_MSK & ISTR_CTR)									
  if (USB_REG.wIstr.istr & ISTR_CTR & wInterrupt_Mask)
  {
    /* servicing of the endpoint correct transfer interrupt */
    /* clear of the CTR flag into the sub */
    CTR_LP();				//usb_int.c						//调用正确传输中断服务程序：控制端点及其它端点服务程序
#ifdef CTR_CALLBACK			//未启用
    CTR_Callback();
#endif
  }
#endif
} /* USB_Istr */
//----------------------------------------------------------------------------



//------------------------------------usb_it----------------------------------
/*******************************************************************************
* Function Name  : USB_LP_CAN_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN_RX0_IRQHandler(void)
{
  USB_Istr();
}
//----------------------------------------------------------------------------




//------------------------------------usb_endp--------------------------------
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u8 buffer_out[VIRTUAL_COM_PORT_DATA_SIZE];

u32 count_out = 0;
u32 count_in = 0;

#define VCOMPORT_IN_FRAME_INTERVAL             5

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  :	EP3_IN_Callback
* Description    :	端点3输出回调	---设备接收
* Input          : 	None.
* Output         : 	None.
* Return         : 	None.
*******************************************************************************/
void usb_endp_out(void)
{
  count_out = GetEPRxCount(ENDP3);														//获取USB接收到的数据
  PMAToUserBufferCopy(buffer_out, ENDP3_RXADDR, count_out);		//USB接口到的数据从串口发送
	SetEPRxValid(ENDP3);																				//使能端点3
	
	//----添加程序---USB发送给串口
	usb_hw_receive_from_usb(buffer_out,count_out);
	
	//返回数据
	UserToPMABufferCopy(buffer_out, ENDP1_TXADDR, count_out);
	SetEPTxCount(ENDP1, count_out);																		//设置端点数据长度
	SetEPTxValid(ENDP1);																							//使能端点
}

/*******************************************************************************
* Function Name  : 	usb_endp_in_1
* Description    :	端点1输入的回调函数	--设备发送
* Input          : 	None.
* Output         : 	None.
* Return         : 	None.
*******************************************************************************/
void usb_endp_in_1(void)
{
//	count_in = 0;
//	usb_hw_Handle_USBAsynchXfer();
//	USART_To_USB_Send_Data();
}
/*******************************************************************************
* Function Name  : SOF_Callback / INTR_SOFINTR_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SOF_Callback(void)
{
//  static u32 FrameCount = 0;
//  
//  if(bDeviceState == CONFIGURED)
//  {
//    if (FrameCount++ == VCOMPORT_IN_FRAME_INTERVAL)
//    {
//      /* Reset the frame counter */
//      FrameCount = 0;
//      
//      /* Check the data to be sent through IN pipe */
//      usb_hw_Handle_USBAsynchXfer();
//    }
//  }
	usb_hw_Handle_USBAsynchXfer();
//	USART_To_USB_Send_Data(); 
}

//----------------------------------------------------------------------------

//----------------------------------------------------------------------------



#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
