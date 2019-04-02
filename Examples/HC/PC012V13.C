#ifdef PC012V13

#include "PC012V13.H"


#include "stm32f10x_exti.h"

#include "SWITCHID.H"
#include "STM32_USART.H"
#include "STM32_SPI.H"
#include "STM32_PWM.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_WDG.H"
#include "STM32_SYSTICK.H"
#include "STM32F10x_BitBand.H"


#include	"stdio.h"			//����printf
#include	"string.h"		//����printf
#include	"stdarg.h"		//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"		//malloc��̬�����ڴ�ռ�

SWITCHID_CONF	SWITCHID;
u8 SwitchID	=	0;


RS485_TypeDef  IN485,OUT485,PORT485A,PORT485B,PORT485C;
u8 RxdBuffe[16]={0};
u8 RevBuffe[16]={0};
u8 RxdBuffe1[8]={0};
u8 RevBuffe1[8]={0};
u8 RxdBuffe2[8]={0};
u8 RevBuffe2[8]={0};
u8 RxdBuffe3[8]={0};
u8 RevBuffe3[8]={0};

u8 U5Time	=	0;
u8 U5Num	=	0;
u8 RecByte	=	0;

void Switch_Configuration(void);	//���뿪�س�ʼ��������
void RS485_Configuration(void);		//RS485����
void RS485_Server(void);
void UART5_Server(void);
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PC012V13_Configuration(void)
{
	//========================ϵͳʱ�ӳ�ʼ��
	SYS_Configuration();							//ϵͳ����---��ϵͳʱ�� STM32_SYS.H		
	//========================��ʱ1�룬�ȴ��ϵ��ȶ�
	Switch_Configuration();						//���뿪�س�ʼ��������
	RS485_Configuration();						//RS485����
	//========================����ָʾ�Ƴ�ʼ����Ƶ��1�룬ռ�ձ�500/1000
	PWM_OUT(TIM2,PWM_OUTChannel1,1,500);
	//========================SysTick��ʼ������������ɨ��PC016V20_Server
	SysTick_Configuration(1000);	//ϵͳ���ʱ������72MHz,��λΪuS	
	//========================�������Ź���ʼ����1��	
	IWDG_Configuration(1000);					//�������Ź�����---������λms
}
/*******************************************************************************
* ������		:	PC016V20_Server
* ��������	:	1msɨ������ 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PC012V13_Server(void)
{
	IWDG_Feed();								//�������Ź�ι��
	
	SwitchID	=	SWITCHID_Read(&SWITCHID);		//
	
	RS485_Server();
	
	UART5_Server();

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
	u8 Num	=	0;
	Num	=	RS485_ReadBufferIDLE			(&PORT485B,RevBuffe2);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(Num)
	{
		RS485_DMASend	(&PORT485C,RevBuffe2,Num);	//RS485-DMA���ͳ���
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
void UART5_Server(void)
{
//	U5Time	=	0;
//	U5Num	=	0;
//	RecByte	=	0;
	unsigned char Num	=	0;
	
	U5Time++;
	
	Num	=	UART5ReceiveData(&RecByte);			//����5������,����0-���жϣ�����1���ж������ݣ������жϣ�������2-���ж��н��յ�����
	if(Num	==	2)
	{
		RxdBuffe[U5Num]	=	RecByte;
		U5Num++;
		U5Time	=	0;
	}
	
	if(U5Time>=2)	//���ճ�ʱ
	{
		if(U5Num)
		{
			GPIO_SetBits(GPIOB,	GPIO_Pin_3);
			memcpy(RevBuffe,RxdBuffe,U5Num);
			USART_Send(UART5,RevBuffe,U5Num);
			memset(RevBuffe,0xFF,U5Num);			
		}
		GPIO_ResetBits(GPIOB,	GPIO_Pin_3);
		U5Time	=	0;
		U5Num		=	0;
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
void Switch_Configuration(void)
{
	SWITCHID.NumOfSW	=	6;
	
	SWITCHID.SW1_PORT	=	GPIOB;
	SWITCHID.SW1_Pin	=	GPIO_Pin_15;
	
	SWITCHID.SW2_PORT	=	GPIOC;
	SWITCHID.SW2_Pin	=	GPIO_Pin_6;
	
	SWITCHID.SW3_PORT	=	GPIOC;
	SWITCHID.SW3_Pin	=	GPIO_Pin_7;
	
	SWITCHID.SW4_PORT	=	GPIOC;
	SWITCHID.SW4_Pin	=	GPIO_Pin_8;
	
	SWITCHID.SW5_PORT	=	GPIOC;
	SWITCHID.SW5_Pin	=	GPIO_Pin_9;
	
	SWITCHID.SW6_PORT	=	GPIOA;
	SWITCHID.SW6_Pin	=	GPIO_Pin_8;
	
	SwitchIdInitialize(&SWITCHID);
	
	SwitchID	=	SWITCHID_Read(&SWITCHID);		//
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
	//IN485,OUT485,PORT485A,PORT485B,PORT485C;
	
//	IN485.USARTx=UART5;
//	IN485.RS485_CTL_PORT=GPIOB;
//	IN485.RS485_CTL_Pin=GPIO_Pin_3;	
//	RS485_DMA_ConfigurationNR	(&IN485,19200,(u32*)RxdBuffe,8);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
	
	GPIO_Configuration_OPP50	(GPIOB,	GPIO_Pin_3);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_ResetBits(GPIOB,	GPIO_Pin_3);
	USART_ConfigurationIT(UART5,19200,1,1);	//USART_����---�����жϷ�ʽ
	
	
	OUT485.USARTx=UART4;
	OUT485.RS485_CTL_PORT=GPIOA;
	OUT485.RS485_CTL_Pin=GPIO_Pin_15;	
	RS485_DMA_ConfigurationNR	(&OUT485,19200,8);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬	
	
	
	PORT485A.USARTx=USART3;
	PORT485A.RS485_CTL_PORT=GPIOB;
	PORT485A.RS485_CTL_Pin=GPIO_Pin_1;	
	RS485_DMA_ConfigurationNR	(&PORT485A,19200,8);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
	
	PORT485B.USARTx=USART2;
	PORT485B.RS485_CTL_PORT=GPIOA;
	PORT485B.RS485_CTL_Pin=GPIO_Pin_1;	
	RS485_DMA_ConfigurationNR	(&PORT485B,19200,8);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
	
	PORT485C.USARTx=USART1;
	PORT485C.RS485_CTL_PORT=GPIOA;
	PORT485C.RS485_CTL_Pin=GPIO_Pin_11;	
	RS485_DMA_ConfigurationNR	(&PORT485C,19200,8);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
}
//================

#endif
