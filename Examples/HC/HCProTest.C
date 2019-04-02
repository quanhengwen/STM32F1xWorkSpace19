#ifdef HCProTest				//��Ԫ���ư�

#include "HCProTest.H"
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

#include "stm32f10x_crc.h"

//=============================RS485A���߶˿�(���ϲ�/��Ԫ��ͨѶ�ӿ�)
RS485Def gRS485Bus;			//���ϲ����߽ӿ�(���ذ�)
//RS485Def gRS485lay;			//���¼����߽ӿ�(���)	

SwitchDef gSwitch;
unsigned char ReSendTime	=	0;
unsigned char Recc=0;
unsigned short len	=	0;
unsigned char PowerFlag	=	0;
//extern RS485FrameDef	*RS485Node;
unsigned short time	=	0;
unsigned char BufferU[1024]={0};		//���ϲ�ͨѶ������ݻ���
unsigned char BufferD[1024]={0};		//���²�ͨѶ������ݻ���
unsigned short length;
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void HCProTest_Configuration(void)
{
	
	SYS_Configuration();							//ϵͳ����---��ϵͳʱ�� STM32_SYS.H	
	GPIO_DeInitAll();									//�����е�GPIO�ر�----V20170605
	
	Communiction_Configuration();			//ͨѶ�ӿ����ã�����RS485,RS232,CAN
//	Switch_Configuration();						//���뿪������
//	Lock_Configuration();							//���˿�����

	/* Enable CRC clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
	
	PWM_OUT(TIM2,PWM_OUTChannel1,2,800);	//PWM�趨-20161127�汾
	
//	RS232Buffer[1]	=	gSwitch.nSWITCHID;
	HCBoradSet(0,0);
		
//	IWDG_Configuration(2000);							//�������Ź�����---������λms
	SysTick_Configuration(1000);					//ϵͳ���ʱ������72MHz,��λΪuS
	
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void HCProTest_Server(void)
{
//  u8 *buffer;
	
	USARTStatusDef	Status;
	IWDG_Feed();								//�������Ź�ι��
	
  //======================================��������
	length	=	USART_ReadBufferIDLE(RS232ASerialPort,BufferU);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ���
	if(length)
	{
		HCResult	res;
		time	=	0;
		Recc++;
		len	=	length;
//		res	=	APISetDataProcess(BufferU,length);
		APIRS232UplinkSetData(BufferU,length);
//		RS485_DMASend(&gRS485Bus,BufferU,length);	//RS485-DMA���ͳ���
	}
	
	//======================================��������
  length	=	RS485_ReadBufferIDLE(&gRS485Bus,BufferD);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(length)
	{
		APIRS485DownlinkSetData(BufferD,length);		
	}
	length	=	APIRS485DownlinkGetAck(BufferD);				//��ȡ��Ҫ�ϴ�������
	if(length)
	{
		RS485_DMASend(&gRS485Bus,BufferD,length);			//RS485-DMA���ͳ���
	}
	
	
	
	
	length	=	APIRS485UplinkGetData(BufferU);
	if(length)
	{
		time	=	0;
		USART_DMASend(RS232ASerialPort,BufferU,length);		//����DMA���ͳ�����������Ѿ����뵽DMA������Buffer��С�����򷵻�0
	}
	
	Status	=	USART_Status(gRS485Bus.USARTx);		//����״̬���
	if(0	==	Status.USART_IDLESTD)
	{
		if(ReSendTime++>50)
		{
			ReSendTime	=	0;
			length	=	APIRS485DownlinkGetData(BufferD);				//��ȡ��Ҫ�ϴ�������
			if(length)
			{
				
				RS485_DMASend(&gRS485Bus,BufferD,length);	//RS485-DMA���ͳ���
			}
		}
	}
	else
	{
		ReSendTime	=	0;
	}
  
 
//	length	=	APIRS485GetdownlinkData(BufferD);
//	if(length)
//	{
//		time	=	0;
//		RS485_DMASend(&gRS485Bus,BufferD,length);	//RS485-DMA���ͳ���
//	}

  
	//======================================ģ�����
	if(time++>50)
	{
//		time	=	0;
//		length	=	APIRS485GetDownlinkData(BufferD);
//		if(length)
//		{
//			RS485_DMASend(&gRS485Bus,BufferD,length);	//RS485-DMA���ͳ���
//		}
//		if(0 == PowerFlag)
//		{
//			PowerFlag	=	1;
//			GetSubOnlineAddr();
//		}
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
	
//	//=============================RS485Bus���߶˿�(���ϲ�/��Ԫ��ͨѶ�ӿ�)
//	gRS485lay.USARTx						=	RS485laySerialPort;
//	gRS485lay.RS485_CTL_PORT		=	RS485layCtlPort;
//	gRS485lay.RS485_CTL_Pin			=	RS485layCtlPin;
//	RS485_DMA_ConfigurationNR(&gRS485lay,RS485layBaudRate,RS485layDataSize);			//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
	//=============================RS232A�˿�(USART1)
	USART_DMA_ConfigurationNR	(RS232ASerialPort,RS232ABaudRate,RS232ADataSize);	//USART_DMA����--��ѯ��ʽ�������ж�

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
#ifndef MBLayer
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
#endif
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