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
	usb_en_def usb_en;		//USBʹ�ܿ��ƶ˿�
	
	SYS_Configuration();
	
	usb_en.usb_connect_port	=	(unsigned long*)USB_DISCONNECT;
	usb_en.usb_connect_pin	=	USB_DISCONNECT_PIN;
	
	PWM_OUT(TIM2,PWM_OUTChannel1,2,500);	//PWM�趨-20161127�汾	ռ�ձ�1/1000
	
	api_usb_virtual_com_configuration(&usb_en);		//���⴮������
}
/*******************************************************************************
*������			:	function
*��������		:	��������˵��
*����				: 
*����ֵ			:	��
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
void api_usb_virtual_com_configuration(usb_en_def* pInfo)		//���⴮������
{	
	usb_en_port	=	(GPIO_TypeDef*)pInfo->usb_connect_port;
	usb_en_pin	=	pInfo->usb_connect_pin;
	
	GPIO_Configuration_OPP50(usb_en_port,usb_en_pin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	
	usb_hw_set_connect(DISABLE);	//�ر�USB��������
	
	usb_hw_set_clock();			//����USBʱ��
	
	usb_hw_set_Interrupt();	//����USB�ж�
	
	USB_Init();						//USB��ʼ��
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
* Description    :  configure the UART 0 with default values.	���ڵ�Ĭ������ֵ
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
										����line coding �ṹ�����ô���.
* Input          :  None.
* Return         :  Configuration status
                    TRUE : configuration done with success
                    FALSE : configuration aborted.
*******************************************************************************/
bool usb_hw_set_usart_config(void)
{
	USART_InitTypeDef USART_InitStructure;
  /* set the Stop bit*/
	/**************����ֹͣλ**************/
  switch (linecoding.format)
  {
    case 0:
      USART_InitStructure.USART_StopBits = USART_StopBits_1;		//1λֹͣλ
      break;
    case 1:
      USART_InitStructure.USART_StopBits = USART_StopBits_1_5;	//1.5Ϊֹͣλ
      break;
    case 2:
      USART_InitStructure.USART_StopBits = USART_StopBits_2;		//2λֹͣλ
      break;
    default :
    {
      usb_hw_set_usart_default();																			//Ĭ������
      return (FALSE);
    }
  }

  /* set the parity bit*/
	/**************����У��λ**************/
  switch (linecoding.paritytype)
  {
    case 0:
      USART_InitStructure.USART_Parity = USART_Parity_No;			//û��У��
      break;
    case 1:
      USART_InitStructure.USART_Parity = USART_Parity_Even;		//żУ��
      break;	
    case 2:
      USART_InitStructure.USART_Parity = USART_Parity_Odd;		//��У��
      break;
    default :
    {
      usb_hw_set_usart_default();																	//Ĭ������
      return (FALSE);
    }
  }

  /*set the data type : only 8bits and 9bits is supported */
	/**************��������λ: 8λ��9λ**************/
  switch (linecoding.datatype)
  {
    case 0x07:
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//8Ϊ����λ�����ѡ���У��λ��������(��У��/żУ��)
      break;
    case 0x08:
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//USART_WordLength_9b;	//9λ����λ
      break;
    default :
    {
      usb_hw_set_usart_default();																//Ĭ������
      return (FALSE);
    }
  }
  USART_InitStructure.USART_BaudRate = linecoding.bitrate;													//���ò�����
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		//����û��Ӳ������������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;										//ʹ�ܽ��ա�����
  USART_Init(ComPort, &USART_InitStructure);																				//��ʼ������
  USART_Cmd(ComPort, ENABLE);																												//ʹ�ܴ���
	
	//api_usart_dma_configurationNR(ComPort,linecoding.bitrate,usart_buffer_size);
	api_usart_dma_configurationST(ComPort,&USART_InitStructure,usart_buffer_size);	//USART_DMA����--�ṹ����ʽ�������ж�
	
  return (TRUE);
}
/*******************************************************************************
* Function Name  : USB_To_UART_Send_Data.
* Description    : send the received data from USB to the UART 0.��USB���յ������ݴӴ��ڷ�������
* Input          : data_buffer: data address.
                   Nb_bytes: number of bytes to send.
* Return         : none.
*******************************************************************************/
void usb_hw_receive_from_usb(u8* data_buffer, u8 Nb_bytes)
{
	api_usart_dma_send(ComPort,data_buffer,(u16)Nb_bytes);		//�Զ���printf����DMA���ͳ���
}
/*******************************************************************************
* Function Name  : UART_To_USB_Send_Data.
* Description    : send the received data from UART 0 to USB.	���ʹ��ڽ��յ������ݵ�USB
* Input          : None.
* Return         : none.
*******************************************************************************/
void usb_hw_send_to_usb(void)
{	
	u16	num	=	api_usart_dma_receive(ComPort,buffer_in);
	if(num)
	{
		UserToPMABufferCopy(buffer_in, ENDP1_TXADDR, num);
		SetEPTxCount(ENDP1, num);																					//���ö˵����ݳ���
		SetEPTxValid(ENDP1);																							//ʹ�ܶ˵�
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
u8 Request = 0;			//����
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
	EP_NUM,				//��ʹ�õĶ˵���
	1							//����ʹ�õĶ˵���
};
/*�豸����*/
DEVICE_PROP Device_Property =				//
  {
    usb_prop_init,									//Virtual_Com_Port�ĳ�ʼ������
    usb_prop_reset,									//Virtual_Com_Port�ĸ�λ����
    usb_prop_status_in,							//CustomVirtual_Com_PortHID״̬���뺯��
    usb_prop_status_out,						//CustomHID״̬�������
    usb_prop_data_setup,						//CustomHID�Ĵ��������ݽ׶ε�������������
    usb_prop_no_data_setup,					//CustomHID�Ĵ���û�����ݽ׶ε�������������
    usb_prop_get_interface_setting,	//CustomHID��ȡ�ӿڼ����ýӿ����ã��Ƿ���ã�
    usb_prop_get_device_descriptor,		//CustomHID��ȡ�豸������
    usb_prop_get_config_descriptor,		//CustomHID��ȡ����������
    Virtual_Com_Port_GetStringDescriptor,		//CustomHID��ȡ�ַ���������
    0,																			//��ǰ��δʹ��
    0x40 /*MAX PACKET SIZE*/								//���İ�����Ϊ64�ֽ�
  };

/*ע��USB��׼�����ʵ�ֺ���*/
USER_STANDARD_REQUESTS User_Standard_Requests =		
  {
    Virtual_Com_Port_GetConfiguration,			//��ȡ��������
    usb_prop_set_configuration,			//������������
    Virtual_Com_Port_GetInterface,					//��ȡ�ӿ�����
    Virtual_Com_Port_SetInterface,					//���ýӿ�����
    Virtual_Com_Port_GetStatus,							//��ȡ״̬����
    Virtual_Com_Port_ClearFeature,					//�����������
    Virtual_Com_Port_SetEndPointFeature,		//���ö˵���������
    Virtual_Com_Port_SetDeviceFeature,			//�����豸��������
    usb_prop_set_device_address				//�����豸��ַ����
  };

/*ע���豸��������Ϣ*/
ONE_DESCRIPTOR Device_Descriptor =
  {
    (u8*)Virtual_Com_Port_DeviceDescriptor,	//ע���豸����������
    VIRTUAL_COM_PORT_SIZ_DEVICE_DESC				//�豸�������ĳ���
  };
	
/*ע���豸��������Ϣ*/
ONE_DESCRIPTOR Qualifier_Descriptor =
  {
    (u8*)USBD_DeviceQualifier,	//ע���豸����������
    10				//�豸�������ĳ���
  };

/*ע��������������Ϣ*/
ONE_DESCRIPTOR Config_Descriptor =
  {
    (u8*)Virtual_Com_Port_ConfigDescriptor,										//ע����������������
//		sizeof((char*)Virtual_Com_Port_ConfigDescriptor)
    VIRTUAL_COM_PORT_SIZ_CONFIG_DESC												//�����������ĳ���
  };

/*ע���ַ�������������������ID�����̡���Ʒ�����к�������*/
ONE_DESCRIPTOR String_Descriptor[4] =
  {
    {(u8*)Virtual_Com_Port_StringLangID, VIRTUAL_COM_PORT_SIZ_STRING_LANGID},		//ע�������ַ�������������
    {(u8*)Virtual_Com_Port_StringVendor, VIRTUAL_COM_PORT_SIZ_STRING_VENDOR},		//ע�᳧���ַ�������������
    {(u8*)Virtual_Com_Port_StringProduct, VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT},	//ע���Ʒ�ַ�������������
    {(u8*)Virtual_Com_Port_StringSerial, VIRTUAL_COM_PORT_SIZ_STRING_SERIAL}		//ע�����к��ַ�������������
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
  Get_SerialNum();	//**************************************��ȡ���к�,����оƬ���кţ����������е�����STM ���ַ����޸�û̫�����塣

  pInformation->Current_Configuration = 0;	//����״̬

  /* Connect the device */
  PowerOn();				//*******�ϵ�usb_pwr.c->Line59:����USB������ǿ��USB��λ����������ж�ʹ�ܱ�־��CNTR_RESETM | CNTR_SUSPM | CNTR_WKUPM;		//���������ж�
	//---------------------------------USB�жϳ�ʼ��
	//-----------------��������жϱ�ʶ
  _SetISTR(0);
	//-----------------�����жϱ�ʶ
  wInterrupt_Mask = IMR_MSK;		//#define IMR_MSK (CNTR_CTRM  | CNTR_SOFM  | CNTR_RESETM )
																//CNTR_CTRM����ȷ����(CTR)�ж�����λ (Correct transfer interrupt mask)
																//CNTR_SOFM��֡���ж�����λ (Start of frame interrupt mask)
																//CNTR_RESETM��USB��λ�ж�����λ (USB reset interrupt mask)
  //-----------------ʹ����Ӧ�ж�
  _SetCNTR(wInterrupt_Mask);		////ʹ����Ӧ�ж�
	//-----------------����USB״̬Ϊδ����
	bDeviceState = UNCONNECTED;		//usb_pwr.h->DEVICE_STATE**************************����ǰ��״̬����Ϊδ����״̬
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Reset
* Description    : Virtual_Com_Port Mouse reset routine	��λ
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void usb_prop_reset(void)
{
  /* Set Virtual_Com_Port DEVICE as not configured */
  pInformation->Current_Configuration = 0;				//���õ�ǰ������Ϊ0����ʾû�����ù�

  /* Current Feature initialization */
  pInformation->Current_Feature = Virtual_Com_Port_ConfigDescriptor[7];	//��ǰ�����ԣ�bmAttributes:�豸��һЩ���ԣ�0xc0��ʾ�Թ��磬��֧��Զ�̻���

  /* Set Virtual_Com_Port DEVICE with the default Interface*/
  pInformation->Current_Interface = 0;
  SetBTABLE(BTABLE_ADDRESS);

  /* Initialize Endpoint 0 */
	/* *****��ʼ���˵�0***** */
  SetEPType(ENDP0, EP_CONTROL);													//���ö˵�0Ϊ���ƶ˵�
  SetEPTxStatus(ENDP0, EP_TX_STALL);										//���ö˵�0������ʱ
  SetEPRxAddr(ENDP0, ENDP0_RXADDR);											//���ö˵�0�Ľ��ջ�������ַ
  SetEPTxAddr(ENDP0, ENDP0_TXADDR);											//���ö˵�0�ķ��ͻ�������ַ
  Clear_Status_Out(ENDP0);															//����˵�0��״̬
  SetEPRxCount(ENDP0, Device_Property.MaxPacketSize);		//���ö˵�0�Ľ�������
  SetEPRxValid(ENDP0);																	//ʹ�ܽ���״̬

  /* Initialize Endpoint 1 */
	/* *****��ʼ���˵�1***** */
  SetEPType(ENDP1, EP_BULK);														//���ö˵�1Ϊ����������
  SetEPTxAddr(ENDP1, ENDP1_TXADDR);											//���ö˵㷢�͵�ַ
  SetEPTxStatus(ENDP1, EP_TX_NAK);											//���ö˵�1�ķ��Ͳ���Ӧ
  SetEPRxStatus(ENDP1, EP_RX_DIS);											//���ö˵�1������

  /* Initialize Endpoint 2 */
	/* *****��ʼ���˵�2***** */
  SetEPType(ENDP2, EP_INTERRUPT);												//���ö˵�2Ϊ�жϴ���
  SetEPTxAddr(ENDP2, ENDP2_TXADDR);											//���ö˵�2���͵�ַ
  SetEPRxStatus(ENDP2, EP_RX_DIS);											//���ö˵�2������״̬
  SetEPTxStatus(ENDP2, EP_TX_NAK);											//���ö˵�2�˵�2Ϊ���ղ���Ӧ

  /* Initialize Endpoint 3 */
	/* *****��ʼ���˵�3***** */
  SetEPType(ENDP3, EP_BULK);														//���ö˵�3Ϊ����������
  SetEPRxAddr(ENDP3, ENDP3_RXADDR);											//���ö˵�3���յ�ַ
  SetEPRxCount(ENDP3, VIRTUAL_COM_PORT_DATA_SIZE);			//���ö˵�3�ļ���ֵ
  SetEPRxStatus(ENDP3, EP_RX_VALID);										//���ö˵�3������Ч
  SetEPTxStatus(ENDP3, EP_TX_DIS);											//���ö˵�3������

  /* Set this device to response on default address */
  SetDeviceAddress(0);																	//�����豸ΪĬ�ϵ�ַΪ0

  bDeviceState = ATTACHED;		//״̬--�Ѳ���USB�豸
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetConfiguration.
* Description    : Udpade the device state to configured.�����豸����״̬
* ����			    	: ����
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
    bDeviceState = CONFIGURED;	//״̬--������
  }
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetConfiguration.
* Description    : Udpade the device state to addressed.�����豸�ı�ַ״̬
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void usb_prop_set_device_address(void)
{
  bDeviceState = ADDRESSED;	//״̬--�ѷ����ַ
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Status_In.
* Description    : Virtual COM Port Status In Routine.״̬���뺯��
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
* Description    : Virtual COM Port Status OUT Routine.״̬�������
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void usb_prop_status_out(void)
{
}
/*******************************************************************************
* Function Name  : Virtual_Com_Port_Data_Setup
* Description    : handle the data class specific requests ���������ݽ׶ε�����������
* Input          : Request Nb.
* Output         : None.
* Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
RESULT usb_prop_data_setup(u8 RequestNo)
{
  u8    *(*CopyRoutine)(u16);

  CopyRoutine = NULL;
	//��ȡLineCoding��ַ��
  if (RequestNo == GET_LINE_CODING)		//****************************��ȡ����ͨ����Ϣ����														
  {
    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))	//����������Ľ������ǽӿ�
    {
      CopyRoutine = Virtual_Com_Port_GetLineCoding;	//**************ָ�뺯��ָ��Virtual_Com_Port_GetLineCoding����
    }
  }
	//д��LineCoding��ַ��
  else if (RequestNo == SET_LINE_CODING)	//************************���ô���ͨѶ��Ϣ����
  {
    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))	//����������Ľ������ǽӿ�
    {
      CopyRoutine = Virtual_Com_Port_SetLineCoding;	//**************ָ�뺯��ָ��Virtual_Com_Port_SetLineCoding����
    }
    Request = SET_LINE_CODING;
  }

  if (CopyRoutine == NULL)
  {
    return USB_UNSUPPORT;
  }

  pInformation->Ctrl_Info.CopyData = CopyRoutine;		//**************ע��ָ��ָ��ĺ���
  pInformation->Ctrl_Info.Usb_wOffset = 0;
  (*CopyRoutine)(0);					//************************************���øú���
  return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_NoData_Setup.
* Description    : handle the no data class specific requests.����û�����ݽ׶�����������
* Input          : Request Nb.
* Output         : None.
* Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
RESULT usb_prop_no_data_setup(u8 RequestNo)
{

  if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))	//***����������Ľ������ǽӿ�
  {
    if (RequestNo == SET_COMM_FEATURE)		//*************************�������⴮������
    {
      return USB_SUCCESS;
    }
    else if (RequestNo == SET_CONTROL_LINE_STATE)		//***************���ÿ�����Ϣ״̬
    {
      return USB_SUCCESS;
    }
  }

  return USB_UNSUPPORT;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetDeviceDescriptor.
* Description    : Gets the device descriptor. 	����������--��ȡ�豸������
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
* Description    : Gets the device descriptor. 	����������--��ȡ�豸������
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
* Description    : get the configuration descriptor.����������--��ȡ����������
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
* Description    : Gets the string descriptors according to the needed index	����������--����������ȡ�ַ�������
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
*                  supported one.		���Խӿڼ��䱸�ýӿ��Ƿ����
* Input1         : u8: Interface : interface number.
* Input2         : u8: AlternateSetting : Alternate Setting number.
* Output         : None.
* Return         : The address of the string descriptors.
*******************************************************************************/
RESULT usb_prop_get_interface_setting(u8 Interface, u8 AlternateSetting)
{
  if (AlternateSetting > 0)		//���õı�Ŵ���0
  {
    return USB_UNSUPPORT;
  }
  else if (Interface > 1)			//�ӿڵı�Ŵ���1
  {
    return USB_UNSUPPORT;
  }
  return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetLineCoding.
* Description    : send the linecoding structure to the PC host.	����linecoding�ṹ�嵽PC��
* Input          : Length.
* Output         : None.
* Return         : Inecoding structure base address.  Linecoding�ṹ��Ļ���ַ
*******************************************************************************/
u8 *Virtual_Com_Port_GetLineCoding(u16 Length)
{
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength = sizeof(linecoding);
    return NULL;
  }
  return(u8 *)&linecoding;			//����ͨ����Ϣ
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetLineCoding.
* Description    : Set the linecoding structure fields.	����linecoding�ṹ��
* Input          : Length.
* Output         : None.
* Return         : Linecoding structure base address.		Linecoding�ṹ��Ļ���ַ.
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
*	����˵��				:		�ж��¼�����-�����ж�״̬�Ĵ����ж��ж����ͣ�ת����Ӧ�Ĵ�����
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_Istr(void)
{

  USB_REG.wIstr.istr	=	_GetISTR();	//wIstr = _GetISTR();			//��ȡ�жϱ�־���õ��ж�Դ
	//--------------------------USB��λ���� (USB reset request)
#if (IMR_MSK & ISTR_RESET)
  if (USB_REG.wIstr.istr & ISTR_RESET & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_RESET);		//���USB��λ���� (USB reset request)
		pProperty->Reset();					//USB��λ
#ifdef RESET_CALLBACK
    RESET_Callback();
#endif
  }
#endif
  //--------------------------PMA�������(Packet memory area over / underrun)
#if (IMR_MSK & ISTR_DOVR)
  if (USB_REG.wIstr.istr & ISTR_DOVR & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_DOVR);	//������黺������� (Packet memory area over / underrun)
#ifdef DOVR_CALLBACK
    DOVR_Callback();
#endif
  }
#endif
  //--------------------------�������USBӦ�ó���ͨ�����Ժ�����Щ������ΪUSBģ��������ڷ�������ʱ���������ش����ơ�
	//													��λ�������жϿ�������Ӧ�ó���Ŀ����׶Σ������������USB���ߵĴ�����������ʶ�û����ܷ����Ĵ���(�������ɣ������������أ�USB���𻵵�)��
#if (IMR_MSK & ISTR_ERR)			//���� (Error)
  if (USB_REG.wIstr.istr & ISTR_ERR & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_ERR);		//������� (Error)
#ifdef ERR_CALLBACK
    ERR_Callback();
#endif
  }
#endif
  //--------------------------�������� (Wakeup)
#if (IMR_MSK & ISTR_WKUP)
  if (USB_REG.wIstr.istr & ISTR_WKUP & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_WKUP);	//����������� (Wakeup)
    Resume(RESUME_EXTERNAL);
#ifdef WKUP_CALLBACK
    WKUP_Callback();
#endif
  }
#endif
  //--------------------------����ģ������ (Suspend mode request)
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
  //--------------------------//֡��(SOF)�жϱ�־���жϷ���������ͨ�����SOF�¼��������������1msͬ
#if (IMR_MSK & ISTR_SOF)										
  if (USB_REG.wIstr.istr & ISTR_SOF & wInterrupt_Mask)		//�������жϱ�־��SOF�жϱ�־����SOF�ж�ʹ����
  {
    _SetISTR((u16)CLR_SOF);									//���SOF�жϱ�־
    bIntPackSOF++;													//ͳ�ƹ����յ�����SOF

#ifdef SOF_CALLBACK
    SOF_Callback();													//��������SOF_CALLBACK�������SOF_Callback,���Ӻ���һ�����ڷ���SOF�ж�ʱ����ʲô
#endif
  }
#endif
  //--------------------------����֡�ױ�ʶλ (Expected start of frame)	�����������3��ESOF�жϣ�Ҳ��������3��δ�յ�SOF���飬������SUSP�жϡ���ʹ�ڹ���ʱ��δ������ʱ����SOF���鶪ʧ����λҲ�ᱻ��λ��
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
  //--------------------------��ȷ�Ĵ��� (Correct transfer)����λ�ڶ˵���ȷ���һ�����ݴ������Ӳ����λ��Ӧ�ó������ͨ��DIR��EP_IDλ��ʶ�����ĸ��˵��������ȷ�����ݴ��䡣
#if (IMR_MSK & ISTR_CTR)									
  if (USB_REG.wIstr.istr & ISTR_CTR & wInterrupt_Mask)
  {
    /* servicing of the endpoint correct transfer interrupt */
    /* clear of the CTR flag into the sub */
    CTR_LP();				//usb_int.c						//������ȷ�����жϷ�����򣺿��ƶ˵㼰�����˵�������
#ifdef CTR_CALLBACK			//δ����
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
* Description    :	�˵�3����ص�	---�豸����
* Input          : 	None.
* Output         : 	None.
* Return         : 	None.
*******************************************************************************/
void usb_endp_out(void)
{
  count_out = GetEPRxCount(ENDP3);														//��ȡUSB���յ�������
  PMAToUserBufferCopy(buffer_out, ENDP3_RXADDR, count_out);		//USB�ӿڵ������ݴӴ��ڷ���
	SetEPRxValid(ENDP3);																				//ʹ�ܶ˵�3
	
	//----��ӳ���---USB���͸�����
	usb_hw_receive_from_usb(buffer_out,count_out);
	
	//��������
	UserToPMABufferCopy(buffer_out, ENDP1_TXADDR, count_out);
	SetEPTxCount(ENDP1, count_out);																		//���ö˵����ݳ���
	SetEPTxValid(ENDP1);																							//ʹ�ܶ˵�
}

/*******************************************************************************
* Function Name  : 	usb_endp_in_1
* Description    :	�˵�1����Ļص�����	--�豸����
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
