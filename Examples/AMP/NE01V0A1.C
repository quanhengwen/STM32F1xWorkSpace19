#ifdef NE01V0A1

#include "NE01V0A1.H"

#include	"AMP_Protocol.H"

#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_TIM.H"
#include "STM32_USART.H"
#include "STM32_SPI.H"

#include "SWITCHID.H"
#include "IOT5302W.H"     //������

#include 	"CRC.H"

#include "string.h"				//�����ڴ��������ͷ�ļ�

//------------------ϵͳLEDָʾ��
#define ampSYSLEDPort    						GPIOA
#define ampSYSLEDPin     						GPIO_Pin_0
//------------------ͨѶ�ӿ�--����ư�����
#define ampCommBusPort     					USART1
#define ampCommBusCtlPort 					GPIOA
#define ampCommBusCtlPin  					GPIO_Pin_8
#define ampCommBusBaudRate        	115200
//------------------ͨѶ�ӿ�--��LCD����
#define ampCommLayPort     					USART2
#define ampCommLayCtlPort 					GPIOA
#define ampCommLayCtlPin  					GPIO_Pin_1
#define ampCommLayBaudRate        	115200

/* Private variables ---------------------------------------------------------*/

static RS485Def ampRS485Bus;   //uart4,PA15   		//���ӿ�
static RS485Def ampRS485lay;   //usart1,PA8    	//����ӿ�

#define ampCommBusBufferSize 512

unsigned char ampCommBusRxd[ampCommBusBufferSize]={0};
unsigned char ampCommBusTxd[ampCommBusBufferSize]={0};

unsigned char ampCommLayRxd[ampCommBusBufferSize]={0};
unsigned char ampCommLayTxd[ampCommBusBufferSize]={0};

/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void API_NE01V0A1_Configuration(void)
{	
	SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H	

	Communication_Configuration();
  GPIO_Configuration_OPP50(ampSYSLEDPort,ampSYSLEDPin);
	
	IWDG_Configuration(3000);			//�������Ź�����---������λms
  
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
void API_NE01V0A1_Server(void)
{ 
	IWDG_Feed();														//�������Ź�ι��
	
	SysLed_server();
	Communication_Server();
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
static void SysLed_server(void)
{
	static unsigned short time = 0;
	if(time++>1000)
	{
		time=0;
		api_gpio_toggle(ampSYSLEDPort,ampSYSLEDPin);		//��GPIO��Ӧ�ܽ������ת----V20170605
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
static void Communication_Configuration(void)
{
	//-----------------------------����ӿ�UART4
  ampRS485Bus.USARTx  				=	ampCommBusPort;
	ampRS485Bus.RS485_CTL_PORT	=	ampCommBusCtlPort;
	ampRS485Bus.RS485_CTL_Pin		=	ampCommBusCtlPin;
	api_rs485_configuration_NR(&ampRS485Bus,ampCommBusBaudRate,maxFramesize);
	
  //-----------------------------���ӿ�USART2
  ampRS485lay.USARTx  = ampCommLayPort;
  ampRS485lay.RS485_CTL_PORT  = ampCommLayCtlPort;
  ampRS485lay.RS485_CTL_Pin   = ampCommLayCtlPin;
  api_rs485_configuration_NR(&ampRS485lay,ampCommLayBaudRate,maxFramesize);
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
static void Communication_Server(void)
{
	unsigned short len=0;
  len	=	api_rs485_receive(&ampRS485Bus,ampCommLayTxd);
	if(len)
	{
		api_rs485_send_force(&ampRS485lay,ampCommLayTxd,len);
	}
	len	=	api_rs485_receive(&ampRS485lay,ampCommBusTxd);
	if(len)
	{
		api_rs485_send_force(&ampRS485Bus,ampCommBusTxd,len);
	}
}
//------------------------------------------------------------------------------
#endif