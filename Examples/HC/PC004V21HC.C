#ifdef PC004V21HC				//��Ԫ���ư�

#include "PC004V21HC.H"
#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_PWM.H"


#include "STM32_CAN.H"
#include "STM32_USART.H"


#include "HC_PHY.H"

#include "SWITCHID.H"


#include "string.h"				//�����ڴ��������ͷ�ļ�

//=============================RS485A���߶˿�(���ϲ�/��Ԫ��ͨѶ�ӿ�)
RS485Def gRS485Bus;			//���ϲ����߽ӿ�(���ذ�)
RS485Def gRS485lay;			//���¼����߽ӿ�(���)	

SwitchDef gSwitch;
unsigned char PowerFlag	=	0;
HCResult	Result;
//extern RS485FrameDef	*RS485Node;
unsigned short time	=	0;
unsigned char RS485BufferU[1024]={0};		//���ϲ�ͨѶ������ݻ���
unsigned char RS485BufferD[1024]={0};		//���²�ͨѶ������ݻ���
unsigned char TestBuffer[]={0x7E,0x01,0x00,0x04,0x04,0x0E,0x00,0x05,0x01,0x02,0x01,0x00,0x00,0x76,0x7F};
/*******************************************************************************
* ������		:
* ��������	:
* ����		:
* ���		:
* ���� 		:
*******************************************************************************/
void PC004V21HC_Configuration(void)
{
	
	SYS_Configuration();							//ϵͳ����---��ϵͳʱ�� STM32_SYS.H	
	GPIO_DeInitAll();									//�����е�GPIO�ر�----V20170605
	
	Communiction_Configuration();			//ͨѶ�ӿ����ã�����RS485,RS232,CAN
	Switch_Configuration();						//���뿪������
	Lock_Configuration();							//���˿�����
	
	
	PWM_OUT(TIM2,PWM_OUTChannel1,1,800);	//PWM�趨-20161127�汾

	HCBoradSet(1,gSwitch.nSWITCHID);
  
  HCBoradSet(1,1);
		
//	IWDG_Configuration(2000);						//�������Ź�����---������λms
	SysTick_Configuration(1000);					//ϵͳ���ʱ������72MHz,��λΪuS
	
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PC004V21HC_Server(void)
{
//  u8 *buffer;
	unsigned short length;
	static	USARTStatusDef	Status;
	
	IWDG_Feed();								//�������Ź�ι��
	
//	goto PC004Test;
	
  //======================================��������
	//--------------------------------------����
	length	=	RS485_ReadBufferIDLE(&gRS485Bus,RS485BufferU);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(length)
	{
		time	=	0;
		Result	=	APIUplinkReceiveDataProcess(RS485BufferU,length);
	}
	//--------------------------------------����
	Status	=	USART_Status(gRS485Bus.USARTx);		//����״̬���
	if(0	==	Status.USART_IDLESTD)
	{
		length	=	APIUplinkGetData(RS485BufferU);
		if(length)
		{
			time	=	0;
			RS485_DMASend(&gRS485Bus,RS485BufferU,length);		//����DMA���ͳ�����������Ѿ����뵽DMA������Buffer��С�����򷵻�0
		}
	}	
	
	
	
	
	
	//======================================��������
	//--------------------------------------����
  length	=	RS485_ReadBufferIDLE(&gRS485lay,RS485BufferD);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(length)
	{
		time	=	0;
		Result	=	APIDownlinkReceiveDataProcess(RS485BufferD,length);
	}
	//--------------------------------------����
	Status	=	USART_Status(gRS485lay.USARTx);		//����״̬���
	if(0	==	Status.USART_IDLESTD)
	{
		length	=	APIDownlinkGetData(RS485BufferD);
		if(length)
		{
			time	=	0;
			RS485_DMASend(&gRS485lay,RS485BufferD,length);		//����DMA���ͳ�����������Ѿ����뵽DMA������Buffer��С�����򷵻�0
		}
	}

	
	
	
	return; 	
	PC004Test:
	length	=	RS485_ReadBufferIDLE(&gRS485Bus,RS485BufferU);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(length)
	{
		RS485_DMASend(&gRS485lay,RS485BufferU,length);	//RS485-DMA���ͳ���
		USART_DMASendList(USART1,RS485BufferU,length);		//����DMA�������ͳ�����������Ѿ����뵽DMA������Buffer��С���������ݴ�������
	}
	length	=	RS485_ReadBufferIDLE(&gRS485lay,RS485BufferD);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(length)
	{
		RS485_DMASend(&gRS485Bus,RS485BufferD,length);	//RS485-DMA���ͳ���
		USART_DMASendList(USART1,RS485BufferD,length);		//����DMA�������ͳ�����������Ѿ����뵽DMA������Buffer��С���������ݴ�������
	}
}
/*******************************************************************************
* ������			:	Communiction_Configuration
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void Communiction_Configuration(void)
{
	
	//=============================RS485Bus���߶˿�(���ϲ�/��Ԫ��ͨѶ�ӿ�)
	gRS485Bus.USARTx						=	RS485BusSerialPort;
	gRS485Bus.RS485_CTL_PORT		=	RS485BusCtlPort;
	gRS485Bus.RS485_CTL_Pin			=	RS485BusCtlPin;
	RS485_DMA_ConfigurationNR(&gRS485Bus,RS485BusBaudRate,RS485BusDataSize);			//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
	
	//=============================RS485Bus���߶˿�(���ϲ�/��Ԫ��ͨѶ�ӿ�)
	gRS485lay.USARTx						=	RS485laySerialPort;
	gRS485lay.RS485_CTL_PORT		=	RS485layCtlPort;
	gRS485lay.RS485_CTL_Pin			=	RS485layCtlPin;
	RS485_DMA_ConfigurationNR(&gRS485lay,RS485layBaudRate,RS485layDataSize);			//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
	//=============================RS232A�˿�(USART1)
	USART_DMA_ConfigurationNR	(RS232ASerialPort,RS232ABaudRate,RS232ADataSize);	//USART_DMA����--��ѯ��ʽ�������ж�
	
	//=============================RS232B�˿�(USART3)
	USART_DMA_ConfigurationNR	(RS232BSerialPort,RS232BBaudRate,RS232BDataSize);	//USART_DMA����--��ѯ��ʽ�������ж�
	
	USART_DMA_ConfigurationNR	(USART1,115200,RS232BDataSize);	//USART_DMA����--��ѯ��ʽ�������ж�
	//=============================CAN
	CAN_Configuration_NR(CANBaudRate);													//CAN1����---��־λ��ѯ��ʽ�������ж�
	CAN_FilterInitConfiguration_StdData(0X01,0X000,0X000);			//CAN�˲�������---��׼����֡ģʽ---������

}
/*******************************************************************************
* ������			:	Communiction_Configuration
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void Switch_Configuration(void)
{
	gSwitch.NumOfSW		=	NumOfSwitch;
	
	gSwitch.SW1_PORT	=	GPIOxSW1;
	gSwitch.SW1_Pin		=	PinxSW1;
	
	gSwitch.SW2_PORT	=	GPIOxSW2;
	gSwitch.SW2_Pin		=	PinxSW2;
	
	gSwitch.SW3_PORT	=	GPIOxSW3;
	gSwitch.SW3_Pin		=	PinxSW3;
	
	gSwitch.SW4_PORT	=	GPIOxSW4;
	gSwitch.SW4_Pin		=	PinxSW4;
	
	gSwitch.SW5_PORT	=	GPIOxSW5;
	gSwitch.SW5_Pin		=	PinxSW5;
	
	gSwitch.SW6_PORT	=	GPIOxSW6;
	gSwitch.SW6_Pin		=	PinxSW6;
	
	gSwitch.SW7_PORT	=	GPIOxSW7;
	gSwitch.SW7_Pin		=	PinxSW7;
	
	gSwitch.SW8_PORT	=	GPIOxSW8;
	gSwitch.SW8_Pin		=	PinxSW8;
	
	SwitchIdInitialize(&gSwitch);
}
/*******************************************************************************
* ������			:	Communiction_Configuration
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void Lock_Configuration(void)
{

}
#endif