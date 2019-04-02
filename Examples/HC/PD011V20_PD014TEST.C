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

#ifdef PD011V20_PD014TEST			//8·������ư�

#include "PD011V20_PD014TEST.H"

#include "STM32_USART.H"
#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_PWM.H"

#include "string.h"				//�����ڴ��������ͷ�ļ�
#include "stm32f10x_dma.h"

#define RxBufferSize	128

u16 SensTestNum	=	0;

u8 RevBuffer1[RxBufferSize]={0};
u8 RxdBuffer1[RxBufferSize]={0};
u8 TxdBuffer1[RxBufferSize]={0};

u8 RevBuffer4[RxBufferSize]={0};
u8 RxdBuffer4[RxBufferSize]={0};
u8 TxdBuffer4[RxBufferSize]={0};

u8 RevBuffer2[RxBufferSize]={0};
u8 RxdBuffer2[RxBufferSize]={0};
u8 TxdBuffer2[RxBufferSize]={0};

u8 RevBuffer3[RxBufferSize]={0};
u8 RxdBuffer3[RxBufferSize]={0};
u8 TxdBuffer3[RxBufferSize]={0};

u16	SYSTime=0;							//ѭ����ʱ����
u16	RS485BTime=0;							//ѭ����ʱ����
u8 RS485NUM	=	0;
u16	DelayTime=0;
u8 testFlg=0;
u16	SensorData	=	0;
u16	SensorDataBac	=	0;
RS485_TypeDef RS485A,RS485B;

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PD011V20_Configuration(void)
{
	SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H
	
	GPIO_DeInitAll();							//�����е�GPIO�ر�----V20170605
	
	PD011V20_UsartConfiguration();
	PD011V20_GpioConfiguration();
	
	
//	RS485_RX_EN(&RS485_4);
	
	SysTick_Configuration(1000);	//ϵͳ���ʱ������72MHz,��λΪuS
	
//	IWDG_Configuration(1000);			//�������Ź�����---������λms	
	
	PWM_OUT(TIM2,PWM_OUTChannel1,1,500);						//PWM�趨-20161127�汾
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PD011V20_Server(void)
{
	
	
	IWDG_Feed();								//�������Ź�ι��
	SYSTime++;
	PD011V20_SensorServer();		//�������ӿ�
	
	if(SYSTime>=1000)
	{
		SYSTime=0;							//ѭ����ʱ����
		PD011V20_OutServer();		//����ӿ�
	}
	PD011V20_UsartServer();		//����
	
	
}

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PD011V20_UsartConfiguration(void)
{	
	GPIO_Configuration_OPP50	(GPIOD,	GPIO_Pin_4);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	PD4=0;
	
	RS485B.USARTx=USART2;
	RS485B.RS485_CTL_PORT=GPIOA;
	RS485B.RS485_CTL_Pin=GPIO_Pin_1;
	
	RS485A.USARTx=UART4;
	RS485A.RS485_CTL_PORT=GPIOA;
	RS485A.RS485_CTL_Pin=GPIO_Pin_15;	
	
	RS485_DMA_ConfigurationNR	(&RS485B,19200,RxBufferSize);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
	RS485_DMA_ConfigurationNR	(&RS485A,19200,RxBufferSize);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
	
	USART_DMA_ConfigurationNR	(USART1,115200,RxBufferSize);	//USART_DMA����--��ѯ��ʽ�������ж�
	USART_DMA_ConfigurationNR	(USART3,9600,RxBufferSize);	//USART_DMA����--��ѯ��ʽ�������ж�
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
void PD011V20_GpioConfiguration(void)
{
	//������
	GPIO_Configuration_IPU	(GPIOE,	GPIO_Pin_All);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	//PMOS�ӿ�
	GPIO_Configuration_OPP50	(GPIOD,	GPIO_Pin_8);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOD,	GPIO_Pin_9);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOB,	GPIO_Pin_14);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOB,	GPIO_Pin_15);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	//����ӿ�
	GPIO_Configuration_OPP50	(GPIOD,	GPIO_Pin_15);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOD,	GPIO_Pin_13);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOD,	GPIO_Pin_11);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOD,	GPIO_Pin_14);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOD,	GPIO_Pin_12);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOD,	GPIO_Pin_10);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	
	GPIO_Configuration_OPP50	(GPIOC,	GPIO_Pin_4);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOB,	GPIO_Pin_0);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOB,	GPIO_Pin_12);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOC,	GPIO_Pin_5);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOB,	GPIO_Pin_1);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOB,	GPIO_Pin_13);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	
	GPIO_Configuration_OPP50	(GPIOD,	GPIO_Pin_7);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOC,	GPIO_Pin_9);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOC,	GPIO_Pin_7);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOA,	GPIO_Pin_8);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOC,	GPIO_Pin_8);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOC,	GPIO_Pin_6);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	
	GPIO_Configuration_OPP50	(GPIOD,	GPIO_Pin_0);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOD,	GPIO_Pin_3);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOD,	GPIO_Pin_5);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOD,	GPIO_Pin_1);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOD,	GPIO_Pin_4);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOD,	GPIO_Pin_6);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
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
void PD011V20_UsartServer(void)		//����
{
	u8 RxNum=0;
	if(RS485BTime++>=500)
	{
		RS485BTime	=	0;
		RS485NUM++;
		RS485_DMAPrintf(&RS485B,"RS485B���Ͳ�������%0.3d",RS485NUM);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
//		RS485_DMAPrintf(&RS485A,"RS485A���Ͳ�������%0.3d",RS485NUM);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
//		RS485_DMASend(&RS485A,&RS485NUM,1);
		memset(TxdBuffer4,RS485NUM,16);
		RS485_DMASend(&RS485A,TxdBuffer4,16);
		
	}
	RxNum=RS485_ReadBufferIDLE(&RS485B,RevBuffer2);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(RxNum)
	{
		RS485_DMAPrintf(&RS485B,"RS485B�յ�����\r\n");					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
		USART_DMAPrintf(USART1,"RS485B�յ�����\r\n");						//RS485-DMA���ͳ���
	}
	RxNum=RS485_ReadBufferIDLE(&RS485A,RevBuffer4);					//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(RxNum)
	{
		RS485_DMAPrintf(&RS485A,"RS485A�յ�����\r\n");					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
		USART_DMAPrintf(USART1,"RS485A�յ�����\r\n");						//RS485-DMA���ͳ���
	}
	if(SensorData	!=	SensorDataBac)
	{
		SensorDataBac	=	SensorData;
		if(SensorDataBac	!=	0xFFFF)
		USART_DMAPrintf(USART1,"����������0x%0.4X\r\n",0xFFFF^SensorDataBac);						//RS485-DMA���ͳ���
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
void PD011V20_SensorServer(void)		//�������ӿ�
{
	if(0	==	SYSTime%10)
	{
		SensTestNum++;
	}
//	GPIO_Toggle	(GPIOE,	GPIO_Pin_All);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
//	GPIO_Write(GPIOE,~SensTestNum);
	SensorData	=	GPIO_ReadInputData(GPIOE);
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
void PD011V20_OutServer(void)		//����ӿ�
{	
//		GPIO_Toggle(GPIOB,	GPIO_Pin_13);		
	GPIO_Toggle	(GPIOD,	GPIO_Pin_8);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOD,	GPIO_Pin_9);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOB,	GPIO_Pin_14);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOB,	GPIO_Pin_15);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605

	GPIO_Toggle	(GPIOD,	GPIO_Pin_15);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOD,	GPIO_Pin_13);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOD,	GPIO_Pin_11);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOD,	GPIO_Pin_14);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOD,	GPIO_Pin_12);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOD,	GPIO_Pin_10);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605

	GPIO_Toggle	(GPIOC,	GPIO_Pin_4);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOB,	GPIO_Pin_0);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOB,	GPIO_Pin_12);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOC,	GPIO_Pin_5);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOB,	GPIO_Pin_1);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOB,	GPIO_Pin_13);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605

	GPIO_Toggle	(GPIOD,	GPIO_Pin_7);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOC,	GPIO_Pin_9);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOC,	GPIO_Pin_7);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOA,	GPIO_Pin_8);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOC,	GPIO_Pin_8);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOC,	GPIO_Pin_6);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605

	GPIO_Toggle	(GPIOD,	GPIO_Pin_0);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOD,	GPIO_Pin_3);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOD,	GPIO_Pin_5);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOD,	GPIO_Pin_1);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOD,	GPIO_Pin_4);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Toggle	(GPIOD,	GPIO_Pin_6);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
}
#endif