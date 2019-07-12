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
#include "BqApp.h"
//#include "stm32f10x_it.h"

#include "Virtual_COM_Port.h"

//#include "hw_config.h"
//#include "platform_config.h"
#include	"stdio.h"			//����printf
#include	"string.h"		//����printf
#include	"stdarg.h"		//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"		//malloc��̬�����ڴ�ռ�


#include "STM32_GPIO.H"
#include "STM32_USART.H"

#include "STM32_SYSTICK.H"
#include "STM32_PWM.H"

#include "BQ26100.H"
#include "BQ26100DATA.H"

#include "usb_lib.h"
//#include "usb_prop.h"
//#include "usb_desc.h"
//#include "usb_istr.h"
#include "usb_pwr.h"
//#include "usb_type.h"
//#include "usb_core.h"			//USB�������ݴ���ĺ����ļ�

//#include "usb_endp.h"
#include "usb_data.h"
//#include "hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define lora	0
#define bq26100V1	0
#define bq26100V2	1

#define bq26100Master 0
#define bq26100Verify 1


#if bq26100V1
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
		
		#define SDQPort   	GPIOB  
		#define SDQPin    	GPIO_Pin_7
		
	#elif bq26100V2
		
		#define ComPortIn		USART2
		#define ComPortOut	USART3
		
		//-------------------��KEY
		#define U1SDQPort   	GPIOA  
		#define U1SDQPin    	GPIO_Pin_7
		
		#define U4SDQPort   	GPIOA  
		#define U4SDQPin    	GPIO_Pin_5
		
		//-------------------�ɴ�ӿ�
		#define FDSDQPort   	GPIOA  
		#define FDSDQPin    	GPIO_Pin_6
		
		//-------------------��������ʹ��
		#define TxEnConnectPort   	GPIOB  
		#define TxEnConnectPin    	GPIO_Pin_0
		
		//-------------------��������ʹ��
		#define RxEnConnectPort   	GPIOB  
		#define RxEnConnectPin    	GPIO_Pin_1
		
		//-------------------�ӻ�������
		#define FeederCheckPort   	GPIOB  
		#define FeederCheckPin    	GPIO_Pin_2
		
		//-------------------��Ϊ�ӻ�ģ��������
		#define FeederConnectPort   	GPIOB  
		#define FeederConnectPin    	GPIO_Pin_9
		
#endif

#define bqstartnum 0
#define bqendnum	 0
#define bqusartsize 64

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned char BqAppState	=	0;
CertificationDataDef	CertificationData;
/* Extern variables ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

unsigned short api_BqApp_Smt_Command_Server(void);
static void api_BqApp_usb_to_feeder(void);
static void api_BqApp_set_sys_led(unsigned char flag);
static void api_BqApp_set_feeder_connect(unsigned char flag);
static void api_BqApp_set_usart_connect(unsigned char flag);
static void api_BqApp_master_to_feeder(void);

/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
void api_BqApp_configuration(void)
{	
	
	api_BqApp_gpio_configuration();
#if bq26100Master
	api_BqApp_set_usart_connect(0);
#else
	api_BqApp_set_usart_connect(0);
#endif
  
//	while(1)
//	{
//		api_usb_virtual_com_server();
//	}
//	 PWM_OUT(TIM2,PWM_OUTChannel1,1,500);	//PWM�趨-20161127�汾	ռ�ձ�1/1000
	//SysTick_Configuration(1000);	//ϵͳ���ʱ������72MHz,��λΪuS
}
//------------------------------------------------------------------------------


/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void api_BqApp_server(void)
{	
	static unsigned short time = 0;
	static unsigned short starttime = 0;
	
	if(bDeviceState != CONFIGURED)
	{
		starttime=0;
		BqAppState=0;
		return;
	}
	if(0==BqAppState)
	{
		if(starttime<5000)	//�ȴ�USB��ʼ��1��
		{
			starttime ++;
			if(starttime==500)
			{
				BqAppState=1;
				api_BqApp_configuration();
			}
			return ;
		}
	}
	
	
#if bq26100Master
	//api_BqApp_usb_to_feeder();
	api_BqApp_master_to_feeder();
#elif bq26100Verify
	api_BqApp_sample_data_verify();
#else
	api_BqApp_Smt_Command_Server();
#endif
}
//------------------------------------------------------------------------------


/*******************************************************************************
*������			:	bq26100Verify
*��������		:	У������
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void api_BqApp_sample_data_verify(void)
{
	
	unsigned short i = 0;
	static unsigned short time=0;
	static unsigned short start=0;
	static unsigned short printfserial=0;
	static unsigned short serial=0;
	unsigned char* message=NULL;
	unsigned char digest[20];
	
	//�����ȴ�ʱ��
	if(start<2000)
	{
		start++;
		return;
	}
	//����ʱ����
	if(time++<100)
		return ;
	time=0;
	
	if(serial>bqendnum)
	{
		serial	=	bqstartnum;
		SysTick_DeleymS(500);				//SysTick��ʱnmS
		return;
	}
	api_BqApp_set_sys_led(1);	
	if(bqstartnum==serial)
	{
		printfserial=0;
		api_usb_printf("\r\n\t{\r\n");
	}
	//---------------------��ȡ����֤ԭʼ��Ϣ
	message	=	(unsigned char*)bq26100_sample_data[serial][0];
	//---------------------��ȡ��ϢժҪ
	api_bq26100_get_digest(message,digest);
	//---------------------��ԭ��ϢժҪ�Աȼ���ӡ
	for(i=0;i<2500;i++)
	{
		if(0==memcmp(digest,bq26100_sample_data[i][1],20))
		{
			api_usb_printf("/*%0.4d:%0.4d:*/\t",printfserial,serial);
			printfserial++;
			api_usb_printf("{{");
			//-----------message
			for(i=0;i<19;i++)
			{
				api_usb_printf("0x%0.2X,",message[i]);
			}
			api_usb_printf("0x%0.2X",message[19]);
			
			api_usb_printf("},\t{");
			
			//-----------digest
			for(i=0;i<19;i++)
			{
				api_usb_printf("0x%0.2X,",digest[i]);
			}
			api_usb_printf("0x%0.2X",digest[19]);
			api_usb_printf("}},\r\n");
			
			break;
		}
	}
	if(bqendnum==serial)
		api_usb_printf("\t}\r\n");
	serial++;
	api_usb_in_set_complete_end();
	api_BqApp_set_sys_led(0);
}
//------------------------------------------------------------------------------


/*******************************************************************************
*������			:	bq26100Verify
*��������		:	У������
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void api_BqApp_sample_data_get_digest(void)
{	
	unsigned short i = 0;
	static unsigned short time=0;
	static unsigned short start=0;
	static unsigned short serial=0;
	unsigned char* message=NULL;
	unsigned char digest[20];
	
	//�����ȴ�ʱ��
	if(start<2000)
	{
		start++;
		return;
	}
	//����ʱ����
	if(time++<100)
		return ;
	time=0;
	
	if(serial>bqendnum)
	{
		SysTick_DeleymS(500);				//SysTick��ʱnmS
		return;
	}
	api_BqApp_set_sys_led(1);
	if(bqstartnum==serial)
		api_usb_printf("\r\n\t{\r\n");
	//---------------------��ȡ����֤ԭʼ��Ϣ
	message	=	(unsigned char*)bq26100_sample_data[serial][0];
	//---------------------��ȡ��ϢժҪ
	api_bq26100_get_digest(message,digest);			//��ȡ��ϢժҪ	
	//---------------------��ӡ��Ϣ����Ӧ��ժҪ	
	api_usb_printf("/*%0.4d:*/\t",serial);
	api_usb_printf("{{");
	//-----------message
	for(i=0;i<19;i++)
	{
		api_usb_printf("0x%0.2X,",message[i]);
	}
	api_usb_printf("0x%0.2X",message[19]);
	
	api_usb_printf("},\t{");
	
	//-----------digest
	for(i=0;i<19;i++)
	{
		api_usb_printf("0x%0.2X,",digest[i]);
	}
	api_usb_printf("0x%0.2X",digest[19]);
	api_usb_printf("}},\r\n");		

	if(bqendnum==serial)
		api_usb_printf("\t}\r\n");
	serial++;
	api_usb_in_set_complete_end();
	api_BqApp_set_sys_led(0);
}
//------------------------------------------------------------------------------

/*******************************************************************************
*������			:	bq26100Verify
*��������		:	У������
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
unsigned char api_BqApp_message_data_get_digest(const unsigned char* message)
{	
	unsigned short i = 0;
	static unsigned short time=0;
	static unsigned short start=0;
	static unsigned long serial=0;
	unsigned char digest[20];
	
//	//�����ȴ�ʱ��
//	if(start<2000)
//	{
//		start++;
//		return;
//	}
//	//����ʱ����
//	if(time++<100)
//		return ;
//	time=0;
//	
//	if(serial>bqendnum)
//	{
//		SysTick_DeleymS(500);				//SysTick��ʱnmS
//		return;
//	}
	api_BqApp_set_sys_led(1);
	if(bqstartnum==serial)
		api_usb_printf("\r\n\t{\r\n");
	//---------------------��ȡ����֤ԭʼ��Ϣ
	//message	=	(unsigned char*)bq26100_sample_data[serial][0];
	//---------------------��ȡ��ϢժҪ
	api_bq26100_get_digest(message,digest);			//��ȡ��ϢժҪ	
	//---------------------��ӡ��Ϣ����Ӧ��ժҪ	
	api_usb_printf("/*%0.6d:*/\t",serial);
	api_usb_printf("{{");
	//-----------message
	for(i=0;i<19;i++)
	{
		api_usb_printf("0x%0.2X,",message[i]);
	}
	api_usb_printf("0x%0.2X",message[19]);
	
	api_usb_printf("},\t{");
	
	//-----------digest
	for(i=0;i<19;i++)
	{
		api_usb_printf("0x%0.2X,",digest[i]);
	}
	api_usb_printf("0x%0.2X",digest[19]);
	api_usb_printf("}},\r\n");		

//	if(bqendnum==serial)
//		api_usb_printf("\t}\r\n");
	serial++;
	api_usb_in_set_complete_end();
	api_BqApp_set_sys_led(0);
	return 1;
}
//------------------------------------------------------------------------------


/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
unsigned short api_BqApp_Smt_Command_Server(void)
{
	#include 	"TOOL.H"
	unsigned short i	=	0;
	unsigned short len	=	0;
	static unsigned char time=0;
	static unsigned short connecttime=0;
	unsigned char rxbuffer[bqusartsize]={0};
	unsigned char txbuffer[bqusartsize]={0};
	unsigned char message[20]={0};
	
	if(bDeviceState != CONFIGURED)
  {
		//return 0;
	}
	connecttime++;
	if(connecttime==50)
	{
		api_BqApp_set_feeder_connect(1);
	}
	else if(connecttime>600)
	{
		connecttime=0;
		api_BqApp_set_feeder_connect(0);
	}
	
	len	=	api_usart_dma_receive(ComPortIn,rxbuffer);	
	if(0==len)			//������
	{
		return 0;
	}
	//----------------------------------��ѯ״ָ̬�0x00,0x00,0xFF����ָ��,
	//���ڳ�ʼ�����أ�0x02,0xFD
	//��ʼ����ɷ��أ�0x01,0xFE	Ӧ��Ϊ����ָ��
	//����У�鷵�أ�0x03,0xFC
	//У����ɷ��أ�0x01,0xFE	Ӧ��Ϊ����ָ��
	if((3==len)&&(0x00==rxbuffer[0]))
	{
		txbuffer[0]=0x01;
		txbuffer[1]=0xFE;
		connecttime=0;
		api_usart_dma_send(ComPortIn,txbuffer,2);
	}
	//----------------------------------0xCA,0x00,0x35����ָ��,����0xCA,0x02,0x01,0x01,0x31
	if((3==len)&&(0xCA==rxbuffer[0]))
	{
		txbuffer[0]=0xCA;
		txbuffer[1]=0x02;
		txbuffer[2]=0x01;
		txbuffer[3]=0x01;
		txbuffer[4]=0x31;
		api_usart_dma_send(ComPortIn,txbuffer,5);
		
		connecttime=0;
	}
	//----------------------------------0x4C,0x14������֤��Ϣָ��,��֤��Ϣ����0x4C,0x02,0x03,0x00,0xAE
	if((23==len)&&(0x4C==rxbuffer[0]))
	{
		txbuffer[0]=0x4C;
		txbuffer[1]=0x02;
		txbuffer[2]=0x03;
		txbuffer[3]=0x00;
		txbuffer[4]=0xAE;
		api_usart_dma_send(ComPortIn,txbuffer,5);
		
		connecttime=0;
		
		//-------------------------------��ȡ����֤��Ϣ������ǰ�ֽ�����֤ʱ��÷��͵�BQ
		for(i=0;i<20;i++)
		{
			message[19-i]=rxbuffer[2+i];
		}
		api_BqApp_set_feeder_connect(0);
		SysTick_DeleymS(10);				//SysTick��ʱnmS		
		
		api_BqApp_set_feeder_connect(1);
		//api_BqApp_set_certification_message(message);
		api_BqApp_message_data_get_digest(message);		//��֤��ϢժҪ		
	}
	return 0;
}
//------------------------------------------------------------------------------
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
static void api_BqApp_usb_to_feeder(void)
{
	unsigned short len	=	0;
	static unsigned char time=0;
	unsigned char buffer[USB_BUFFER_SIZE]={0};
	if(bDeviceState != CONFIGURED)
  {
		return;
	}
	if(0==get_usart_tx_idle(ComPortOut))		//����״̬���
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
		api_usart_dma_send(ComPortOut,buffer,(u16)len);		//�Զ���printf����DMA���ͳ���
	}
}
//------------------------------------------------------------------------------
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
static void api_BqApp_master_to_feeder(void)
{
	unsigned short len	=	0;
	static unsigned char time	=	0;
	static unsigned char time2	=	0;
	static unsigned char step	=	0;
	static unsigned char ledflag=0;
	static unsigned short ledtime=0;
	unsigned char rxbuffer[bqusartsize]={0};
	unsigned char txbuffer[bqusartsize]={0};
	unsigned char sample[20]={0xB8,0xD3,0xAA,0x20,0x82,0xFE,0xD1,0xF0,0x7C,0x0A,0x07,0x8E,0x26,0xC7,0xDE,0x10,0xCA,0xA3,0x2C,0xC8};
	
	if(ledflag)
	{
		ledtime++;
		if(ledtime%300<150)
		{
			api_BqApp_set_sys_led(1);
		}
		else
		{
			api_BqApp_set_sys_led(0);
		}
	}
	if(ledtime>1500)
	{
		ledtime=0;
		ledflag=0;
		api_BqApp_set_sys_led(0);
	}
	
	
	//GPIO_Toggle(GPIOA,GPIO_Pin_0);		//��GPIO��Ӧ�ܽ������ת----V20170605
	
	//GPIO_Toggle(GPIOA,GPIO_Pin_0);		//��GPIO��Ӧ�ܽ������ת----V20170605
	//st1----------��ȡ״̬
	len	=	api_usart_dma_receive(ComPortOut,rxbuffer);
	//----����״̬
	if(2==len)
	{
		if(0x01==rxbuffer[0])	//����
		{
			if(0==step)
			{
				step=1;
				time2=0;
			}
		}
	}
	else if(5==len)
	{
		if(0xCA==rxbuffer[0])
		{
			step=2;
			time2=0;
		}
		else if(0x4C==rxbuffer[0])
		{
			step=3;
			time2=0;
		}
	}
	
	//-------------------�ӻ�δ����ʱ������
	if(GPIO_ReadInputDataBit(FeederCheckPort,FeederCheckPin))
	{
		time	=	0;
		time2	=	0;
		step	=	0;
		ledtime=0;
		//ledflag=0;
		return;
	}
	
	if(3==step)
	{
		return;
	}
	
	ledflag	=	1;
	if(0==get_usart_tx_idle(ComPortOut))		//����״̬���
	{
		return ;
	}
	
	if(time++<2)		//����֡���8ms
		return ;	
	time = 0;
	
	//ST2-----------------------
	if(0==step)
	{
		txbuffer[0]=0x00;
		txbuffer[1]=0x00;
		txbuffer[2]=0xFF;
		api_usart_dma_send(ComPortOut,txbuffer,(u16)3);		//�Զ���printf����DMA���ͳ���
	}
	else if(1==step)
	{
		txbuffer[0]=0xCA;
		txbuffer[1]=0x00;
		txbuffer[2]=0x35;
		api_usart_dma_send(ComPortOut,txbuffer,(u16)3);		//�Զ���printf����DMA���ͳ���
	}
	else if(2==step)
	{
		txbuffer[0]=0x4C;
		txbuffer[1]=0x14;
		txbuffer[22]=0x7D;
		for(len=0;len<20;len++)
		{
			txbuffer[len+2]=sample[19-len];
		}
		api_usart_dma_send(ComPortOut,txbuffer,(u16)23);		//�Զ���printf����DMA���ͳ���
	}
	else if(3==step)
	{
		txbuffer[0]=0x00;
		txbuffer[1]=0x00;
		txbuffer[2]=0xFF;
		//api_usart_dma_send(ComPortOut,txbuffer,(u16)3);		//�Զ���printf����DMA���ͳ���
	}
	//time2++;
}
//------------------------------------------------------------------------------

/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
static void api_BqApp_set_sys_led(unsigned char flag)
{
	if(0==flag)	//���
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_0);	
	}
	else
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	}
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
static void api_BqApp_set_feeder_connect(unsigned char flag)
{
	if(0==flag)	//�Ͽ�����
	{		
		GPIO_ResetBits(FeederConnectPort,FeederConnectPin);		
	}
	else
	{
		//GPIO_ResetBits(FeederConnectPort,FeederConnectPin);
		GPIO_SetBits(FeederConnectPort,FeederConnectPin);
	}
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
static void api_BqApp_set_usart_connect(unsigned char flag)
{
	if(0==flag)	//�Ͽ�����
	{
		GPIO_ResetBits(TxEnConnectPort,TxEnConnectPin);
		GPIO_ResetBits(RxEnConnectPort,RxEnConnectPin);
#if bq26100Master		
		api_usart_dma_configurationNR(ComPortIn,625000,bqusartsize);
		api_usart_dma_configurationNR(ComPortOut,625000,bqusartsize);
#else
		api_usart_dma_configurationNR(ComPortIn,625000,bqusartsize);
		api_usart_dma_configurationNR(ComPortOut,625000,bqusartsize);
#endif
	}
	else
	{
		GPIO_Configuration_INF(TxEnConnectPort,	TxEnConnectPin);			//��GPIO��Ӧ�ܽ�����Ϊ��������ģʽ----V20170605
		GPIO_Configuration_INF(RxEnConnectPort,	RxEnConnectPin);			//��GPIO��Ӧ�ܽ�����Ϊ��������ģʽ----V20170605
		
		GPIO_SetBits(TxEnConnectPort,TxEnConnectPin);
		GPIO_SetBits(RxEnConnectPort,RxEnConnectPin);
	}
}
//------------------------------------------------------------------------------

/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void api_BqApp_certification_message_server(void)
{
	if(CertificationData.flag)	//������
	{
		api_BqApp_message_data_get_digest(CertificationData.message);		//��֤��ϢժҪ
		CertificationData.flag=0;
	}
}
//------------------------------------------------------------------------------
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
static unsigned char api_BqApp_set_certification_message(const unsigned char* message)
{
	if(CertificationData.flag)	//������
	{
		return 0;
	}
	else
	{
		unsigned char i = 0;
		for(i=0;i<20;i++)
		{
			CertificationData.message[i]=message[i];
		}
		CertificationData.flag=1;
		return 1;
	}
	return 0;
}
//------------------------------------------------------------------------------

/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void api_BqApp_gpio_configuration(void)
{
	//PWM_OUT(TIM2,PWM_OUTChannel1,2,500);	//PWM�趨-20161127�汾	ռ�ձ�1/1000
	GPIO_Configuration_OPP50(GPIOA,GPIO_Pin_0);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	api_BqApp_set_sys_led(0);
#if bq26100Master
	//PWM_OUT(TIM2,PWM_OUTChannel1,2,500);	//PWM�趨-20161127�汾	ռ�ձ�1/1000
#else
	//GPIO_Configuration_OPP50(GPIOA,GPIO_Pin_0);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
#endif
	
#if bq26100V1
	GPIO_Configuration_OPP50(G1Port,G1Pin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	//GPIO_SetBits(G1Port,G1Pin);
	GPIO_ResetBits(G1Port,G1Pin);
	
	GPIO_Configuration_OPP50(G2Port,G2Pin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	//GPIO_SetBits(G2Port,G2Pin);
	GPIO_ResetBits(G2Port,G2Pin);
	
	GPIO_Configuration_OPP50(G3Port,G3Pin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	//GPIO_SetBits(G3Port,G3Pin);
	GPIO_ResetBits(G3Port,G3Pin);
	
	GPIO_Configuration_OPP50(V24Port,V24Pin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	//GPIO_SetBits(V24Port,V24Pin);
	GPIO_ResetBits(V24Port,V24Pin);
	
	GPIO_Configuration_OPP50(V05Port,V05Pin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	//GPIO_SetBits(V05Port,V05Pin);
	GPIO_ResetBits(V05Port,V05Pin);
	
	GPIO_Configuration_OPP50(MtxPort,MtxPin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	//GPIO_SetBits(MtxPort,MtxPin);
	GPIO_ResetBits(MtxPort,MtxPin);
	
	GPIO_Configuration_OPP50(MrxPort,MrxPin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	//GPIO_SetBits(MrxPort,MrxPin);
	GPIO_ResetBits(MrxPort,MrxPin);	
#elif bq26100V2
	
	//api_usart_dma_configurationNR(ComPortIn,625000,128);
	//-------------------��������ʹ��
	GPIO_Configuration_OPP50(TxEnConnectPort,TxEnConnectPin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	//GPIO_SetBits(TxEnConnectPort,TxEnConnectPin);
	GPIO_ResetBits(TxEnConnectPort,TxEnConnectPin);
	
	GPIO_Configuration_OPP50(RxEnConnectPort,RxEnConnectPin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	//GPIO_SetBits(RxEnConnectPort,RxEnConnectPin);
	GPIO_ResetBits(RxEnConnectPort,RxEnConnectPin);
	
	//-------------------�ӻ�������
	GPIO_Configuration_IPU(FeederCheckPort,FeederCheckPin);				//��GPIO��Ӧ�ܽ�����Ϊ��������ģʽ----V20170605
	
	//-------------------��Ϊ�ӻ�ģ��������
	GPIO_Configuration_OPP50(FeederConnectPort,FeederConnectPin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_ResetBits(FeederConnectPort,FeederConnectPin);
#endif
}
//------------------------------------------------------------------------------
#endif
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
