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

/******************************** ˵��20160912**********************************
********************************************************************************
* ���ܣ����� USBӲ������
* 
* 
* 
* 
* 
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "Virtual_COM_Port.h"
//#include "stm32f10x_it.h"

#include "BqApp.h"

//#include "hw_config.h"
//#include "platform_config.h"
#include	"stdio.h"			//����printf
#include	"string.h"		//����printf
#include	"stdarg.h"		//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"		//malloc��̬�����ڴ�ռ�


#include "STM32_GPIO.H"
#include "STM32_USART.H"

#include "STM32_SYSTICK.H"
#include "STM32_TIM.H"

#include "BQ26100.H"

#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "usb_type.h"
#include "usb_core.h"			//USB�������ݴ���ĺ����ļ�

#include "usb_endp.h"
#include "usb_data.h"
#include "hw_config.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define lora	0
#define bq26100V1	0
#define bq26100V2	1
#define STM32_FSMC 0
#define STM32_USB_TEST 0


#if PS005
		#define USB_DISCONNECT            GPIOA  
		#define USB_DISCONNECT_PIN        GPIO_Pin_8
		#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
		
		#define ComPort	USART1
	#elif lora
	  #define USB_DISCONNECT            GPIOA  
		#define USB_DISCONNECT_PIN        GPIO_Pin_15
		#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
		
		#define ComPort	USART1
	#elif bq26100V1
	
	  #define USB_DISCONNECT            GPIOA  
		#define USB_DISCONNECT_PIN        GPIO_Pin_15
		
		#define ComPort	USART1

#elif bq26100V2
	
	  #define USB_DISCONNECT            GPIOA  
		#define USB_DISCONNECT_PIN        GPIO_Pin_15
		
		#define ComPort	USART1		//
		
	#elif STM32_USB_TEST
	  #define USB_DISCONNECT            GPIOA  
		#define USB_DISCONNECT_PIN        GPIO_Pin_15
		#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
		
		#define ComPort	USART1
	#elif STM32_FSMC
	  #define USB_DISCONNECT            GPIOA  
		#define USB_DISCONNECT_PIN        GPIO_Pin_8
		#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
		
		#define ComPort	USART1
	#else
	  #define USB_DISCONNECT            GPIOA  
		#define USB_DISCONNECT_PIN        GPIO_Pin_8
		#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
		
		#define ComPort	USART1
#endif


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//USART_InitTypeDef USART_InitStructure;


/* Extern variables ----------------------------------------------------------*/
u8 buffer_in[USB_BUFFER_SIZE];

//u8 buffer_rx[VIRTUAL_COM_PORT_DATA_SIZE];
//u8 buffer_tx[VIRTUAL_COM_PORT_DATA_SIZE];

//u32 USART_Rx_ptr_in = 0;
//u32 USART_Rx_ptr_out = 0;
//u32 USART_Rx_length  = 0;

u8 Usart_tx_flg=0;
unsigned short virtual_com_time = 0;


extern u32 count_in;
extern LINE_CODING linecoding;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void api_usb_virtual_gpio_configuration(void);

extern void api_usb_hw_initialize(void);
void USB_CMD(FunctionalState NewState);

/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
void api_usb_virtual_com_configuration(void)		//���⴮������
{
	#include "BqApp.h"
	api_usb_virtual_gpio_configuration();
	
	api_usb_hw_initialize();
	
//#if bq26100V2	
//	api_BqApp_configuration();
//#endif
  
//	while(1)
//	{
//		api_usb_virtual_com_server();
//	}
//	 PWM_OUT(TIM2,PWM_OUTChannel1,1,500);	//PWM�趨-20161127�汾	ռ�ձ�1/1000
	SysTick_Configuration(1000);	//ϵͳ���ʱ������72MHz,��λΪuS
	

}
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void api_usb_virtual_com_server(void)
{	
	static unsigned short time = 0;
	
	if(bDeviceState != CONFIGURED)
	{
		virtual_com_time=0;
		return;
	}
	if(virtual_com_time<2000)	//�ȴ�USB��ʼ��1��
	{
		virtual_com_time ++;
		return ;
	}
	
#if bq26100V2
	api_BqApp_server();
#else
	usb_to_uart_server();
	USART_To_USB_Send_Data();
#endif	
}
/*******************************************************************************
*������		: function
*��������	:	���ڽ��շ�����
*����			: 
*���			:	��
*����ֵ		:	��
*����			:	api_usart_dma_printf(USART2,"����ENG=%d\n",num);
*�ر�˵��	:	��DMA������ɺ���Ҫ�ͷŶ�̬�ռ䣬free(USART_BUFFER);
					:	USART_BUFFER������STM32_USART.H
*******************************************************************************/
unsigned short api_usb_printf(const char *format,...)
{
	
//		va_list ap; 										//VA_LIST ����C�����н����������һ��꣬����ͷ�ļ���#include <stdarg.h>,���ڻ�ȡ��ȷ�������Ĳ���
//    static char string[ 256 ];			//�������飬
//    va_start( ap, format );
//    vsprintf( string , format, ap );    
//    va_end( ap );	

	unsigned short	str_len=0;
	unsigned char		format_buffer[256]={0};
	
	//3)**********argsΪ�����һ��ָ��ɱ�����ı�����va_list�Լ��±�Ҫ�õ���va_start,va_end�������ڶ��壬�ɱ���������б���Ҫ�õ��꣬ ��stdarg.hͷ�ļ��ж���
	va_list args;  
	//5)**********��ʼ��args�ĺ�����ʹ��ָ��ɱ�����ĵ�һ��������format�ǿɱ������ǰһ������
	va_start(args, format);
	//6)**********��������·��������ִ��ĳ���(��ȥ\0),����������ظ�ֵ
	str_len = vsprintf((char*)format_buffer, format, args);
	//7)**********�����ɱ�����Ļ�ȡ
	va_end(args); 
	//8)**********���ȷ��ͻ�������С�����ݸ���������������ַ����DMA��������	
	return api_usb_in_add_data((u8*)format_buffer,str_len);
	//return set_usart_tx_dma_buffer(USARTx,(u8*)format_buffer,str_len);		//����DMA���ͳ���
}

/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void api_usb_virtual_gpio_configuration(void)
{
	GPIO_Configuration_OPP50(USB_DISCONNECT,USB_DISCONNECT_PIN);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	USB_CMD(DISABLE);
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
* Description    :  configure the UART 0 with default values.	���ڵ�Ĭ������ֵ
* Input          :  None.
* Return         :  None.
*******************************************************************************/
void USART_Config_Default(void)
{
	USART_InitTypeDef USART_InitStructure;
//--------------ԭ����
//	linecoding
	USART_DeInit(ComPort);
	
	USART_InitStructure.USART_BaudRate    = 19200; 					  //������
	USART_InitStructure.USART_WordLength  = USART_WordLength_8b;		    //����λ
	USART_InitStructure.USART_StopBits    = USART_StopBits_1;				    //ֹͣλ
	USART_InitStructure.USART_Parity      = USART_Parity_No ; 					//��żУ��
	USART_InitStructure.USART_Mode        = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//����
	USART_Init(ComPort, &USART_InitStructure);											//��ʼ������
	
	api_usart_configuration_ST(ComPort,&USART_InitStructure,USB_BUFFER_SIZE);	//USART_DMA����--�ṹ����ʽ�������ж�	
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
char USART_Config(void)
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
      USART_Config_Default();																		//Ĭ������
      return (0);
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
      USART_Config_Default();																	//Ĭ������
      return (0);
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
      USART_Config_Default();																//Ĭ������
      return (0);
    }
  }
  USART_InitStructure.USART_BaudRate = linecoding.bitrate;													//���ò�����
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		//����û��Ӳ������������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;										//ʹ�ܽ��ա�����
  USART_Init(ComPort, &USART_InitStructure);																				//��ʼ������
  USART_Cmd(ComPort, ENABLE);																												//ʹ�ܴ���
	
	api_usart_configuration_ST(ComPort,&USART_InitStructure,USB_BUFFER_SIZE);	//USART_DMA����--�ṹ����ʽ�������ж�
	
  return (1);
}
/*******************************************************************************
* Function Name  : USB_To_UART_Send_Data.
* Description    : send the received data from USB to the UART 0.��USB���յ������ݴӴ��ڷ�������
* Input          : data_buffer: data address.
                   Nb_bytes: number of bytes to send.
* Return         : none.
*******************************************************************************/
void USB_To_USART_Send_Data(u8* data_buffer, u8 Nb_bytes)
{
	Usart_tx_flg=1;
	api_usart_send(ComPort,data_buffer,(u16)Nb_bytes);		//�Զ���printf����DMA���ͳ���
}
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
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
	if(0!=get_usart_tx_status(ComPort))		//����״̬���
	{
		return ;
	}
	//-----------------------------------������
	if(0==api_usb_out_get_read_enable())
	{
		return;
	}
	if(time++<10)		//����֡���8ms
		return ;
	time = 0;

	len	=	api_usb_out_get_data(buffer);
	if(len)
	{
		sendnum+=len;
		sendednum+=api_usart_send(ComPort,buffer,(u16)len);		//�Զ���printf����DMA���ͳ���
	}
}

/*******************************************************************************
* Function Name  : UART_To_USB_Send_Data.
* Description    : send the received data from UART 0 to USB.	���ʹ��ڽ��յ������ݵ�USB
* Input          : None.
* Return         : none.
*******************************************************************************/
void USART_To_USB_Send_Data(void)
{	
	u16	num	=	api_usart_receive(ComPort,buffer_in);
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
//	count_in=api_usart_receive(ComPort,buffer_rx);
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

u8 *Virtual_Com_Port_GetLineCoding(u16 Length);	//��ȡ����ͨ����Ϣ
u8 *Virtual_Com_Port_SetLineCoding(u16 Length);	//���ô���ͨѶ��Ϣ

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
DEVICE_PROP Device_Property =								//
  {
    Virtual_Com_Port_init,									//Virtual_Com_Port�ĳ�ʼ������
    Virtual_Com_Port_Reset,									//Virtual_Com_Port�ĸ�λ����
    Virtual_Com_Port_Status_In,							//CustomVirtual_Com_PortHID״̬���뺯��
    Virtual_Com_Port_Status_Out,						//CustomHID״̬�������
    Virtual_Com_Port_Data_Setup,						//CustomHID�Ĵ��������ݽ׶ε�������������
    Virtual_Com_Port_NoData_Setup,					//CustomHID�Ĵ���û�����ݽ׶ε�������������
    Virtual_Com_Port_Get_Interface_Setting,	//CustomHID��ȡ�ӿڼ����ýӿ����ã��Ƿ���ã�
    Virtual_Com_Port_GetDeviceDescriptor,		//CustomHID��ȡ�豸������
    Virtual_Com_Port_GetConfigDescriptor,		//CustomHID��ȡ����������
    Virtual_Com_Port_GetStringDescriptor,		//CustomHID��ȡ�ַ���������
    0,																			//��ǰ��δʹ��
    0x40 /*MAX PACKET SIZE*/								//���İ�����Ϊ64�ֽ�
  };

/*ע��USB��׼�����ʵ�ֺ���*/
USER_STANDARD_REQUESTS User_Standard_Requests =		
  {
    Virtual_Com_Port_GetConfiguration,			//��ȡ��������
    Virtual_Com_Port_SetConfiguration,			//������������
    Virtual_Com_Port_GetInterface,					//��ȡ�ӿ�����
    Virtual_Com_Port_SetInterface,					//���ýӿ�����
    Virtual_Com_Port_GetStatus,							//��ȡ״̬����
    Virtual_Com_Port_ClearFeature,					//�����������
    Virtual_Com_Port_SetEndPointFeature,		//���ö˵���������
    Virtual_Com_Port_SetDeviceFeature,			//�����豸��������
    Virtual_Com_Port_SetDeviceAddress				//�����豸��ַ����
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
* Description    : Virtual COM Port Mouse init routine.	��ʼ��
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Virtual_Com_Port_init(void)
{

  /* Update the serial number string descriptor with the data from the unique
  ID*/
  Get_SerialNum();	//**************************************��ȡ���к�,����оƬ���кţ����������е�����STM ���ַ����޸�û̫�����塣

  pInformation->Current_Configuration = 0;	//����״̬

  /* Connect the device */
  PowerOn();				//*******�ϵ�usb_pwr.c->Line59:����USB������ǿ��USB��λ����������ж�ʹ�ܱ�־��CNTR_RESETM | CNTR_SUSPM | CNTR_WKUPM;		//���������ж�
  /* USB interrupts initialization */
  /* clear pending interrupts */
  _SetISTR(0);			//**************************************ִ�л����ĳ�ʼ������������˵�豸IP�Ͷ˵�0�ĳ�ʼ��
  wInterrupt_Mask = IMR_MSK;		//#define IMR_MSK (CNTR_CTRM  | CNTR_SOFM  | CNTR_RESETM )
																//CNTR_CTRM����ȷ����(CTR)�ж�����λ (Correct transfer interrupt mask)
																//CNTR_SOFM��֡���ж�����λ (Start of frame interrupt mask)
																//CNTR_RESETM��USB��λ�ж�����λ (USB reset interrupt mask)
  /* set interrupts mask */
  _SetCNTR(wInterrupt_Mask);		////ʹ����Ӧ�ж�

  /* configure the USART 1 to the default settings */
  USART_Config_Default();		//���ô�����ȱʡ״̬---�����ﲨ���ʱ���Ϊ9600�����������˽����жϡ������ж�û������

  bDeviceState = UNCONNECTED;		//usb_pwr.h->DEVICE_STATE**************************����ǰ��״̬����Ϊδ����״̬
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Reset
* Description    : Virtual_Com_Port Mouse reset routine	��λ
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Virtual_Com_Port_Reset(void)
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

  bDeviceState = ATTACHED;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetConfiguration.
* Description    : Udpade the device state to configured.�����豸����״̬
* ����			    	: ����
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
* Description    : Udpade the device state to addressed.�����豸�ı�ַ״̬
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
* Description    : Virtual COM Port Status In Routine.״̬���뺯��
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
* Description    : Virtual COM Port Status OUT Routine.״̬�������
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Virtual_Com_Port_Status_Out(void)
{}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Data_Setup
* Description    : handle the data class specific requests ���������ݽ׶ε�����������
* Input          : Request Nb.
* Output         : None.
* Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
RESULT Virtual_Com_Port_Data_Setup(u8 RequestNo)
{
  u8    *(*CopyRoutine)(u16);

  CopyRoutine = NULL;
	//������ֶ�
  if (RequestNo == GET_LINE_CODING)		//****************************��ȡ����ͨ����Ϣ����														
  {
    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))	//����������Ľ������ǽӿ�
    {
      CopyRoutine = Virtual_Com_Port_GetLineCoding;	//**************ָ�뺯��ָ��Virtual_Com_Port_GetLineCoding����
    }
  }
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
RESULT Virtual_Com_Port_NoData_Setup(u8 RequestNo)
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
u8 *Virtual_Com_Port_GetDeviceDescriptor(u16 Length)
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
u8 *Virtual_Com_Port_GetConfigDescriptor(u16 Length)
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
RESULT Virtual_Com_Port_Get_Interface_Setting(u8 Interface, u8 AlternateSetting)
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


#endif
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
