/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : PC001V21.c
* Author             : WOW
* Version            : V2.0.1
* Date               : 06/26/2017
* Description        : PC001V21����ư�.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/********************************��������*************************************/
//	����	SwitchID	����
//	��1��07 ID 01	
//	��2��07 ID 02
//	��3��07 ID 03
//	��4��07 ID 04

/********************************��������*************************************/
//	����	SwitchID
//	05 ID	


#ifdef PC001V10				//����ҩ�����ذ�

#include "PC001V10.H"



#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_USART.H"
#include "STM32_PWM.H"
#include "STM32_CAN.H"
#include "STM32F10x_BitBand.H"


#include "LOCK.H"
#include "SWITCHID.H"
#include "STM32_USART.H"

#include "string.h"				//�����ڴ��������ͷ�ļ�
#include "stm32f10x_dma.h"



SWITCHID_CONF	SWITCHID;
u8 SwitchID	=	0;

u32 CAN_BaudRate	=	500000;

u32 GetBufferDelayTime;

u32 LockTestTime	=	0;
sLockSeDef LockSe1;
sLockSeDef LockSe2;
sLockSeDef LockSe3;
sLockSeDef LockSe4;




RS485_TypeDef SL485;

u8 SL485Flg	=	0;
u32 SL485TestTime	=	0;
u8 SL485TestData	=	0;
u8 SL485Rx[16]={0};
u8 SL485Re[16]={0};
u8 SL485Tx[16]={0};

u32 SYSLEDTime	=	0;

CanRxMsg RxMessage;				//CAN���� 
CanTxMsg TxMessage;				//CAN����
u8 CanData[8]={0};
u8 CanFlag	=	0;
u16 CanCnt	=	0;
u16 CanTime	=	0;

void CAN_Server(void);
//PWM_TimDef PWM_Tim;
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PC001V10_Configuration(void)
{
	SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H
	
	GPIO_DeInitAll();							//�����е�GPIO�ر�----V20170605
		
	SysTick_Configuration(1000);		//ϵͳ���ʱ������72MHz,��λΪuS
	
//	IWDG_Configuration(1000);			//�������Ź�����---������λms	
	
	Lock_Configuration();
	
	SwitchID_Configuration();
	
	RS485_Configuration();
	
	GPIO_Configuration_OPP50	(GPIOC,	GPIO_Pin_9);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	
	CAN_Configuration_NR(CAN_BaudRate);				//CAN1����---��־λ��ѯ��ʽ�������ж�
	CAN_FilterInitConfiguration_StdData(0x01,		0x00,			0x00);			//CAN�˲�������---��׼����֡ģʽ---������ư�����ⲿ��������ӿ�
//	PWM_OUT(TIM2,PWM_OUTChannel1,1,900);						//PWM�趨-20161127�汾	
	
//	PWM_OUT_TIMConf(&PWM_Tim);									//PWM�������---���100KHz
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PC001V10_Server(void)
{	
//	IWDG_Feed();													//�������Ź�ι��

	
	if(GetBufferDelayTime++>1000)			//100ms
	{
		GetBufferDelayTime	=	0;
		SwitchID	=	SWITCHID_Read(&SWITCHID);		//
	}
	LockTest();
	RS485_Server();
	CAN_Server();
	if(SYSLEDTime++>500)
	{
		SYSLEDTime	=	0;
		GPIO_Toggle	(GPIOC,	GPIO_Pin_9);		//��GPIO��Ӧ�ܽ������ת----V20170605
	}
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void CAN_Server(void)
{
	u8 status	=	0;		//CAN��ȡ����0��ʾ��Ч
//	u8 CanFlag	=	0;
//	u16 CanCnt	=	0;
#if 0			//����
	status	=	CAN_RX_DATA(&RxMessage);									//���CAN������������
	if(status)
	{
		CanTime	=	0;
	}
	else
	{
		CanTime++;
		if(CanTime>=300)
		{
			memset(RxMessage.Data,0x00,8);
		}
		if(CanTime>=500)
		{
			CanTime	=	0;			
//			memset(CanData,0x55,8);
			CanData[0]	=	'M';
			CanData[1]	=	'A';
			CanData[2]	=	'S';
			CanData[3]	=	'T';
			CanData[4]	=	'E';
			CanData[5]	=	'R';
			CanData[6]	=	'M';
			CanData[7]	=	'M';
			CAN_StdTX_DATA(0xFA,8,CanData);			//CANʹ�ñ�׼֡��������
		}
	}
#else		//�ӻ�
	status	=	CAN_RX_DATA(&RxMessage);									//���CAN������������
	if(status)
	{
		CanTime	=	0;
//		memset(CanData,0x75,8);
		CanData[0]	=	'S';
		CanData[1]	=	'L';
		CanData[2]	=	'A';
		CanData[3]	=	'V';
		CanData[4]	=	'E';
		CanData[5]	=	'R';
		CanData[6]	=	'M';
		CanData[7]	=	'M';
		CAN_StdTX_DATA(0xBD,8,CanData);			//CANʹ�ñ�׼֡��������
	}
	else
	{
		if(CanTime++>=200)
		{
			CanTime	=	0;
			memset(RxMessage.Data,0x00,8);
		}
	}
#endif
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void RS485_Configuration(void)
{
	SL485.USARTx	=	USART2;
	SL485.RS485_CTL_PORT	= GPIOC;
	SL485.RS485_CTL_Pin		=	GPIO_Pin_8;
	RS485_DMA_ConfigurationNR	(&SL485,19200,9);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void RS485_Server(void)
{
	u8	Num	=	0;	
//	Num	=	RS485_ReadBufferIDLE(&SL485,(u32*)SL485Re,(u32*)SL485Rx);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
//	if(SL485TestTime++>2000)
//	{		
//		SL485TestTime	=	0;
//		if(Num>=2)
//		{
//			if(SL485TestData>200)
//			{
//				SL485TestData=0;
//			}
//			SL485TestData+=1;
//		}
//		else
//		{
//			if(SL485TestData>200)
//			{
//				SL485TestData=0;
//			}
//			SL485TestData+=2;
//		}
//		memset(SL485Tx,SL485TestData,9);
//		SL485Tx[0]	=	SwitchID;
//		RS485_DMASend	(&SL485,(u32*)SL485Tx,9);	//RS485-DMA���ͳ���		
//	}
	
//	u32 Bus485TestTime	=	0;
//	u8 Bus485TestData	=	0;

#if 1	
	
	Num	=	RS485_ReadBufferIDLE(&SL485,SL485Re);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(Num)
	{
		SL485TestTime	=	0;
	}
	if(SL485TestTime++>1000)
	{		
		SL485TestTime	=	0;
		if(SL485TestData++>=100)
		{
			SL485TestData=0;
		}

		memset(SL485Tx,SL485TestData,9);
		SL485Tx[0]	=	SwitchID;
		RS485_DMASend	(&SL485,SL485Tx,9);	//RS485-DMA���ͳ���		
	}

#else			//�ӻ�
	Num	=	RS485_ReadBufferIDLE(&SL485,(u32*)SL485Re,(u32*)SL485Rx);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(Num)
	{
		SL485Flg	=	1;
		SL485TestData	=	SL485Re[1]+1;
		SL485TestTime	=	0;
	}
	if(SL485Flg)
	{
		if(SL485TestTime++>50)
		{
			SL485Flg	=	0;
			SL485TestTime	=	0;
			
			memset(SL485Tx,SL485TestData,9);
			SL485Tx[0]	=	SwitchID;
			RS485_DMASend	(&SL485,(u32*)SL485Tx,9);	//RS485-DMA���ͳ���
		}
	}

	//============================================
#endif
	
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void SwitchID_Configuration(void)
{
	SWITCHID.NumOfSW	=	8;
	
	SWITCHID.SW1_PORT	=	GPIOB;
	SWITCHID.SW1_Pin	=	GPIO_Pin_0;
	
	SWITCHID.SW2_PORT	=	GPIOB;
	SWITCHID.SW2_Pin	=	GPIO_Pin_1;
	
	SWITCHID.SW3_PORT	=	GPIOB;
	SWITCHID.SW3_Pin	=	GPIO_Pin_10;
	
	SWITCHID.SW4_PORT	=	GPIOB;
	SWITCHID.SW4_Pin	=	GPIO_Pin_11;
	
	SWITCHID.SW5_PORT	=	GPIOB;
	SWITCHID.SW5_Pin	=	GPIO_Pin_14;
	
	SWITCHID.SW6_PORT	=	GPIOB;
	SWITCHID.SW6_Pin	=	GPIO_Pin_15;
	
	SWITCHID.SW7_PORT	=	GPIOC;
	SWITCHID.SW7_Pin	=	GPIO_Pin_6;
	
	SWITCHID.SW8_PORT	=	GPIOC;
	SWITCHID.SW8_Pin	=	GPIO_Pin_7;
	
	SwitchIdInitialize(&SWITCHID);
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void Lock_Configuration(void)
{

	LockSe1.DrGPIOx	=	GPIOC;
	LockSe1.DrGPIO_Pin_n	=	GPIO_Pin_1;
	
	LockSe1.SeGPIOx	=	GPIOC;
	LockSe1.SeGPIO_Pin_n	=	GPIO_Pin_3;
	LockInitialize(&LockSe1);		//������--����
	//------------------------------------------
	LockSe2.DrGPIOx	=	GPIOB;
	LockSe2.DrGPIO_Pin_n	=	GPIO_Pin_5;
	
	LockSe2.SeGPIOx	=	GPIOC;
	LockSe2.SeGPIO_Pin_n	=	GPIO_Pin_11;
	LockInitialize(&LockSe2);		//������--����
	//------------------------------------------
	
	LockSe3.DrGPIOx	=	GPIOC;
	LockSe3.DrGPIO_Pin_n	=	GPIO_Pin_10;
	
	LockSe3.SeGPIOx	=	GPIOA;
	LockSe3.SeGPIO_Pin_n	=	GPIO_Pin_1;
	LockInitialize(&LockSe3);		//������--����
	//------------------------------------------
	
	LockSe4.DrGPIOx	=	GPIOC;
	LockSe4.DrGPIO_Pin_n	=	GPIO_Pin_5;
	
	LockSe4.SeGPIOx	=	GPIOA;
	LockSe4.SeGPIO_Pin_n	=	GPIO_Pin_7;
	LockInitialize(&LockSe4);		//������--����
	//------------------------------------------

}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void LockTest(void)
{
	if(LockTestTime++>2000)			//100ms
	{
		LockTestTime	=	0;
		LockSetOn(&LockSe1,2000);		//����
		LockSetOn(&LockSe2,2000);		//����
		LockSetOn(&LockSe3,2000);		//����
		LockSetOn(&LockSe4,2000);		//����
	}
	LockServer(&LockSe1);
	LockServer(&LockSe2);
	LockServer(&LockSe3);
	LockServer(&LockSe4);
}
#endif