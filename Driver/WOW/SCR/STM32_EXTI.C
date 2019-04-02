/******************************** User_library *********************************
* �ļ��� 	: STM32_SDCard.H
* ����   	: wegam@sina.com
* �汾   	: V
* ����   	: 2016/01/01
* ˵��   	: 
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/


#include "STM32_EXTI.H"

#include "stm32f10x_exti.h"
#include "stm32f10x_nvic.h"
#include "stm32f10x_rcc.h"

#include "STM32_WOW.H"
//#include "STM32F10x_BitBand.H"

Trigger_LineType Trigger_Line;

/*******************************************************************************
*������			:	EXTI_ClockConf
*��������		:	���ⲿ�ж���Ӧ�ܽ�ʱ��
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	20171213
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void EXTI_ClockConf(GPIO_TypeDef* GPIOx,						//GPIO�˿�			
										u16 GPIO_Pin_x									//GPIO����
										)		//���ⲿ�ж���Ӧ�ܽ�ʱ��--20171213	
{
	/* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin_x));
	switch (*(u32*)&GPIOx)
	{
		//********************GPIOAʱ��ʹ��********************	
		case GPIOA_BASE:
//			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			if(((GPIO_Pin_x&GPIO_Pin_13)==GPIO_Pin_13)||((GPIO_Pin_x&GPIO_Pin_14)==GPIO_Pin_14)||((GPIO_Pin_x&GPIO_Pin_15)==GPIO_Pin_15)||(GPIO_Pin_x==GPIO_Pin_All))
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
				//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);			//�ر�SW����
				GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);		//�ر�JTAG,SW���ܿ���
			}
			else
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			break;
		//********************GPIOBʱ��ʹ��********************
		case GPIOB_BASE:
			if(((GPIO_Pin_x&GPIO_Pin_3)==GPIO_Pin_3)||((GPIO_Pin_x&GPIO_Pin_4)==GPIO_Pin_4)||(GPIO_Pin_x==GPIO_Pin_All))
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
				GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);				//�ر�JTAG
			}
			else
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			break;
		//********************GPIOCʱ��ʹ��********************
		case GPIOC_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
			if(((GPIO_Pin_x&GPIO_Pin_14)==GPIO_Pin_14)||((GPIO_Pin_x&GPIO_Pin_15)==GPIO_Pin_15))
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_AFIO, ENABLE);
			break;
		//********************GPIODʱ��ʹ��********************
		case GPIOD_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
			break;
		//********************GPIOEʱ��ʹ��********************
		case GPIOE_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
			break;
		//********************GPIOFʱ��ʹ��********************
		case GPIOF_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
			break;
		//********************GPIOGʱ��ʹ��********************
		case GPIOG_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
			break;		
		default: break;		
	}
}
/*******************************************************************************
*������			:	EXTI_ClockConf
*��������		:	��������ģʽ��ʼ��GPIO
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	20171213
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void EXTI_GPIOConf(GPIO_TypeDef* GPIOx,						//GPIO�˿�			
										u16 GPIO_Pin_x,								//GPIO����
										GPIOMode_TypeDef GPIO_Mode		//ģʽ
										)		//��������ģʽ��ʼ��GPIO--20171213	
{
	//1��GPIO�ṹ�嶨��
	GPIO_InitTypeDef	GPIO_InitStructure;
	/* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin_x));
	
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_x;
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;					//����ģʽ��ʱ��������Ч
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode;								//����ģʽ
	//3����ʼ��GPIO
	GPIO_Init(GPIOx,&GPIO_InitStructure);												//��ʼ��
}
/*******************************************************************************
*������			:	EXTI_LineConf
*��������		:	�����ж���
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	20171213
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void EXTI_LineConf(GPIO_TypeDef* GPIOx,						//GPIO�˿�			
										u16 GPIO_Pin_x								//GPIO����
										)		//�����ж���--20171213	
{
	u8 GPIO_PortSource=0;
	u8 GPIO_PinSource=0;
	
	/* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin_x));
	
	//==============GPIO_PortSourceѡ��
	switch (*(u32*)&GPIOx)
	{
		//********************�ж�Դ�˿�ΪGPIOA********************	
		case GPIOA_BASE:
			GPIO_PortSource	=	GPIO_PortSourceGPIOA;
			break;
		//********************�ж�Դ�˿�ΪGPIOA********************	
		case GPIOB_BASE:
			GPIO_PortSource	=	GPIO_PortSourceGPIOB;
			break;
		//********************�ж�Դ�˿�ΪGPIOA********************	
		case GPIOC_BASE:
			GPIO_PortSource	=	GPIO_PortSourceGPIOC;
			break;
		//********************�ж�Դ�˿�ΪGPIOA********************	
		case GPIOD_BASE:
			GPIO_PortSource	=	GPIO_PortSourceGPIOD;
			break;
		//********************�ж�Դ�˿�ΪGPIOA********************	
		case GPIOE_BASE:
			GPIO_PortSource	=	GPIO_PortSourceGPIOE;
			break;
		//********************�ж�Դ�˿�ΪGPIOA********************	
		case GPIOF_BASE:
			GPIO_PortSource	=	GPIO_PortSourceGPIOF;
			break;
		//********************�ж�Դ�˿�ΪGPIOA********************	
		case GPIOG_BASE:
			GPIO_PortSource	=	GPIO_PortSourceGPIOG;
			break;
		default:	
			break;
	}
	//==============GPIO_PinSourceѡ��
	switch ((u16)GPIO_Pin_x)
	{
		case GPIO_Pin_0:
			GPIO_PinSource	=	GPIO_PinSource0;
			break;
		case GPIO_Pin_1:
			GPIO_PinSource	=	GPIO_PinSource1;
			break;
		case GPIO_Pin_2:
			GPIO_PinSource	=	GPIO_PinSource2;
			break;
		case GPIO_Pin_3:
			GPIO_PinSource	=	GPIO_PinSource3;
			break;
		case GPIO_Pin_4:
			GPIO_PinSource	=	GPIO_PinSource4;
			break;
		case GPIO_Pin_5:
			GPIO_PinSource	=	GPIO_PinSource5;
			break;
		case GPIO_Pin_6:
			GPIO_PinSource	=	GPIO_PinSource6;
			break;
		case GPIO_Pin_7:
			GPIO_PinSource	=	GPIO_PinSource7;
			break;
		case GPIO_Pin_8:
			GPIO_PinSource	=	GPIO_PinSource8;
			break;
		case GPIO_Pin_9:
			GPIO_PinSource	=	GPIO_PinSource9;
			break;
		case GPIO_Pin_10:
			GPIO_PinSource	=	GPIO_PinSource10;
			break;
		case GPIO_Pin_11:
			GPIO_PinSource	=	GPIO_PinSource11;
			break;
		case GPIO_Pin_12:
			GPIO_PinSource	=	GPIO_PinSource12;
			break;
		case GPIO_Pin_13:
			GPIO_PinSource	=	GPIO_PinSource13;
			break;
		case GPIO_Pin_14:
			GPIO_PinSource	=	GPIO_PinSource14;
			break;
		case GPIO_Pin_15:
			GPIO_PinSource	=	GPIO_PinSource15;
			break;
		
		default:	
			break;
	}
	//====================�����ⲿ�ж�Դ
	GPIO_EXTILineConfig(GPIO_PortSource,GPIO_PinSource);
}
/*******************************************************************************
*������			:	EXTI_TTMConf
*��������		:	�ⲿ�жϴ���ģʽ����
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	20171213
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void EXTI_TTMConf(u16 GPIO_Pin_x,										//GPIO�˿�			
									EXTITrigger_TypeDef EXTI_Trigger	//����ģʽ
									)		//�ⲿ�жϴ���ģʽ����--20171213
{
	u32 EXTI_Line;
	EXTI_InitTypeDef EXTI_Initstructure;
	assert_param(IS_GPIO_PIN(GPIO_Pin_x));
	//==============GPIO_PinSourceѡ��
	switch ((u16)GPIO_Pin_x)
	{
		case GPIO_Pin_0:
			EXTI_Line	=	EXTI_Line0;
			break;
		case GPIO_Pin_1:
			EXTI_Line	=	EXTI_Line1;
			break;
		case GPIO_Pin_2:
			EXTI_Line	=	EXTI_Line2;
			break;
		case GPIO_Pin_3:
			EXTI_Line	=	EXTI_Line3;
			break;
		case GPIO_Pin_4:
			EXTI_Line	=	EXTI_Line4;
			break;
		case GPIO_Pin_5:
			EXTI_Line	=	EXTI_Line5;
			break;
		case GPIO_Pin_6:
			EXTI_Line	=	EXTI_Line6;
			break;
		case GPIO_Pin_7:
			EXTI_Line	=	EXTI_Line7;
			break;
		case GPIO_Pin_8:
			EXTI_Line	=	EXTI_Line8;
			break;
		case GPIO_Pin_9:
			EXTI_Line	=	EXTI_Line9;
			break;
		case GPIO_Pin_10:
			EXTI_Line	=	EXTI_Line10;
			break;
		case GPIO_Pin_11:
			EXTI_Line	=	EXTI_Line11;
			break;
		case GPIO_Pin_12:
			EXTI_Line	=	EXTI_Line12;
			break;
		case GPIO_Pin_13:
			EXTI_Line	=	EXTI_Line13;
			break;
		case GPIO_Pin_14:
			EXTI_Line	=	EXTI_Line14;
			break;
		case GPIO_Pin_15:
			EXTI_Line	=	EXTI_Line15;
			break;
		
		default:	
			break;
	}
	EXTI_Initstructure.EXTI_Line=EXTI_Line;												//�ⲿ�ж���·
	EXTI_Initstructure.EXTI_Mode=EXTI_Mode_Interrupt;							//�ж�ģʽ
	EXTI_Initstructure.EXTI_Trigger=EXTI_Trigger;									//������ʽ-
	EXTI_Initstructure.EXTI_LineCmd=ENABLE;												//����ʹ��
	EXTI_Init(&EXTI_Initstructure);
	
	EXTI_GenerateSWInterrupt(EXTI_Line);													//ʹ���ж�
	EXTI_ClearITPendingBit(EXTI_Line);
}
/*******************************************************************************
*������			:	EXTI_IRQChannelConf
*��������		:	�ж�ͨ�������ȼ�����
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	20171213
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void EXTI_NVICConf(GPIO_TypeDef* GPIOx,						//GPIO�˿�			
									u16 GPIO_Pin_x								//GPIO����
									)		//�ж�ͨ�������ȼ�����--20171213
{
	u8 EXTI_IRQChannel;
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
	assert_param(IS_GPIO_PIN(GPIO_Pin_x));
	
	//==============�ж�ͨ��ѡ��
	switch ((u16)GPIO_Pin_x)
	{
		case GPIO_Pin_0:
			EXTI_IRQChannel	=	EXTI0_IRQChannel;
			break;
		case GPIO_Pin_1:
			EXTI_IRQChannel	=	EXTI1_IRQChannel;
			break;
		case GPIO_Pin_2:
			EXTI_IRQChannel	=	EXTI2_IRQChannel;
			break;
		case GPIO_Pin_3:
			EXTI_IRQChannel	=	EXTI3_IRQChannel;
			break;
		case GPIO_Pin_4:
			EXTI_IRQChannel	=	EXTI4_IRQChannel;
			break;
		
		case GPIO_Pin_5:
			EXTI_IRQChannel	=	EXTI9_5_IRQChannel;
			break;
		case GPIO_Pin_6:
			EXTI_IRQChannel	=	EXTI9_5_IRQChannel;
			break;
		case GPIO_Pin_7:
			EXTI_IRQChannel	=	EXTI9_5_IRQChannel;
			break;
		case GPIO_Pin_8:
			EXTI_IRQChannel	=	EXTI9_5_IRQChannel;
			break;
		case GPIO_Pin_9:
			EXTI_IRQChannel	=	EXTI9_5_IRQChannel;
			break;
		
		case GPIO_Pin_10:
			EXTI_IRQChannel	=	EXTI15_10_IRQChannel;
			break;
		case GPIO_Pin_11:
			EXTI_IRQChannel	=	EXTI15_10_IRQChannel;
			break;
		case GPIO_Pin_12:
			EXTI_IRQChannel	=	EXTI15_10_IRQChannel;
			break;
		case GPIO_Pin_13:
			EXTI_IRQChannel	=	EXTI15_10_IRQChannel;
			break;
		case GPIO_Pin_14:
			EXTI_IRQChannel	=	EXTI15_10_IRQChannel;
			break;
		case GPIO_Pin_15:
			EXTI_IRQChannel	=	EXTI15_10_IRQChannel;
			break;
		
		default:	
			break;
	}
	//7)**********�����ж�
	NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQChannel;					//ѡ���ж�ͨ��-�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;			//��ռ���ȼ�---��ʱĬ��1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;						//��Ӧ���ȼ�---��ʱĬ��1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ��
	NVIC_Init(&NVIC_InitStructure);

}
/*******************************************************************************
*������			:	EXTI_Configuration_ITR
*��������		:	�ⲿ�������ж�����
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	20171213
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void EXTI_Configuration_ITR(GPIO_TypeDef* GPIOx,						//GPIO�˿�			
														u16 GPIO_Pin_x									//GPIO����
														)		//�ⲿ�������ж�����--20171213
{
	/* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
	assert_param(IS_GPIO_PIN(GPIO_Pin_x));
	
	EXTI_ClockConf(GPIOx,GPIO_Pin_x);								//���ⲿ�ж���Ӧ�ܽ�ʱ��--20171213
	EXTI_GPIOConf(GPIOx,GPIO_Pin_x,GPIO_Mode_IPD);	//��������ģʽ��ʼ��GPIO--20171213---����
	EXTI_LineConf(GPIOx,GPIO_Pin_x);								//�����ж���--20171213	
	EXTI_TTMConf(GPIO_Pin_x,EXTI_Trigger_Rising);		//�ⲿ�жϴ���ģʽ����--20171213--�������ж�
	EXTI_NVICConf(GPIOx,GPIO_Pin_x);								//�ж�ͨ�������ȼ�����--20171213	
}
/*******************************************************************************
*������			:	EXTI_Configuration_ITF
*��������		:	�ⲿ�½����ж�����
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	20171213
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void EXTI_Configuration_ITF(GPIO_TypeDef* GPIOx,						//GPIO�˿�			
														u16 GPIO_Pin_x									//GPIO����
														)		//�ⲿ�½����ж�����--20171213
{
	/* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
	assert_param(IS_GPIO_PIN(GPIO_Pin_x));
	
	EXTI_ClockConf(GPIOx,GPIO_Pin_x);								//���ⲿ�ж���Ӧ�ܽ�ʱ��--20171213
	EXTI_GPIOConf(GPIOx,GPIO_Pin_x,GPIO_Mode_IPU);	//��������ģʽ��ʼ��GPIO--20171213---����
	EXTI_LineConf(GPIOx,GPIO_Pin_x);								//�����ж���--20171213	
	EXTI_TTMConf(GPIO_Pin_x,EXTI_Trigger_Falling);	//�ⲿ�жϴ���ģʽ����--20171213--�½����ж�
	EXTI_NVICConf(GPIOx,GPIO_Pin_x);								//�ж�ͨ�������ȼ�����--20171213	
}
/*******************************************************************************
*������			:	EXTI_Configuration_ITRF
*��������		:	�ⲿ���ش����ж�����
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	20171213
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void EXTI_Configuration_ITRF(GPIO_TypeDef* GPIOx,						//GPIO�˿�			
														u16 GPIO_Pin_x									//GPIO����
														)		//�ⲿ���ش����ж�����--20171213
{
	/* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
	assert_param(IS_GPIO_PIN(GPIO_Pin_x));
	
	EXTI_ClockConf(GPIOx,GPIO_Pin_x);												//���ⲿ�ж���Ӧ�ܽ�ʱ��--20171213
	EXTI_GPIOConf(GPIOx,GPIO_Pin_x,GPIO_Mode_IN_FLOATING);	//��������ģʽ��ʼ��GPIO--20171213---����
	EXTI_LineConf(GPIOx,GPIO_Pin_x);												//�����ж���--20171213	
	EXTI_TTMConf(GPIO_Pin_x,EXTI_Trigger_Rising_Falling);		//�ⲿ�жϴ���ģʽ����--20171213--�����ж�
	EXTI_NVICConf(GPIOx,GPIO_Pin_x);												//�ж�ͨ�������ȼ�����--20171213	
}

/*******************************************************************************
* ������		:	
* ��������	:	�¼��ж� 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void EXTI_Event_Configuration(void)
{
	if(EXTI_GetITStatus(EXTI_Line0))
	{
		Trigger_Line.Trigger_Line0	=	1;
//		EXTI_ClearITPendingBit(EXTI_Line0);
	}
	else if(EXTI_GetITStatus(EXTI_Line1))
	{
		Trigger_Line.Trigger_Line1	=	1;
//		EXTI_ClearITPendingBit(EXTI_Line1);
	}
	else if(EXTI_GetITStatus(EXTI_Line2))
	{
		Trigger_Line.Trigger_Line2	=	1;
//		EXTI_ClearITPendingBit(EXTI_Line2);
	}
	else if(EXTI_GetITStatus(EXTI_Line3))
	{
		Trigger_Line.Trigger_Line3	=	1;
//		EXTI_ClearITPendingBit(EXTI_Line3);
	}
	else if(EXTI_GetITStatus(EXTI_Line4))
	{
		Trigger_Line.Trigger_Line4	=	1;
//		EXTI_ClearITPendingBit(EXTI_Line4);
	}
	else if(EXTI_GetITStatus(EXTI_Line5))
	{
		Trigger_Line.Trigger_Line5	=	1;
//		EXTI_ClearITPendingBit(EXTI_Line5);
	}
	else if(EXTI_GetITStatus(EXTI_Line6))
	{
		Trigger_Line.Trigger_Line6	=	1;
//		EXTI_ClearITPendingBit(EXTI_Line6);
	}
	else if(EXTI_GetITStatus(EXTI_Line7))
	{
		Trigger_Line.Trigger_Line7	=	1;
//		EXTI_ClearITPendingBit(EXTI_Line7);
	}
	else if(EXTI_GetITStatus(EXTI_Line8))
	{
		Trigger_Line.Trigger_Line8	=	1;
//		EXTI_ClearITPendingBit(EXTI_Line8);
	}
	else if(EXTI_GetITStatus(EXTI_Line9))
	{
		Trigger_Line.Trigger_Line9	=	1;
//		EXTI_ClearITPendingBit(EXTI_Line9);
	}
	else if(EXTI_GetITStatus(EXTI_Line10))
	{
		Trigger_Line.Trigger_Line10	=	1;
//		EXTI_ClearITPendingBit(EXTI_Line10);
	}
	else if(EXTI_GetITStatus(EXTI_Line11))
	{
		Trigger_Line.Trigger_Line11	=	1;
//		EXTI_ClearITPendingBit(EXTI_Line11);
	}
	else if(EXTI_GetITStatus(EXTI_Line12))
	{
		Trigger_Line.Trigger_Line12	=	1;
//		EXTI_ClearITPendingBit(EXTI_Line12);
	}
	else if(EXTI_GetITStatus(EXTI_Line13))
	{
		Trigger_Line.Trigger_Line13	=	1;
//		EXTI_ClearITPendingBit(EXTI_Line13);
	}
	else if(EXTI_GetITStatus(EXTI_Line14))
	{
		Trigger_Line.Trigger_Line14	=	1;
//		EXTI_ClearITPendingBit(EXTI_Line14);
	}
	else if(EXTI_GetITStatus(EXTI_Line15))
	{
		Trigger_Line.Trigger_Line15	=	1;
//		EXTI_ClearITPendingBit(EXTI_Line15);
	}
}

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void EXTI_Server(void)
{
	unsigned short temp	=	0;
	EXTI_Event_Configuration();
	WOW_Server();				//������

//	Trigger_Line	=	0;
	Trigger_Line	=	*(Trigger_LineType*)&temp;			//�����־
	EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_ClearITPendingBit(EXTI_Line1);
	EXTI_ClearITPendingBit(EXTI_Line2);
	EXTI_ClearITPendingBit(EXTI_Line3);
	EXTI_ClearITPendingBit(EXTI_Line4);
	EXTI_ClearITPendingBit(EXTI_Line5);
	EXTI_ClearITPendingBit(EXTI_Line6);
	EXTI_ClearITPendingBit(EXTI_Line7);
	EXTI_ClearITPendingBit(EXTI_Line8);
	EXTI_ClearITPendingBit(EXTI_Line9);
	EXTI_ClearITPendingBit(EXTI_Line10);
	EXTI_ClearITPendingBit(EXTI_Line11);
	EXTI_ClearITPendingBit(EXTI_Line12);
	EXTI_ClearITPendingBit(EXTI_Line13);
	EXTI_ClearITPendingBit(EXTI_Line14);
	EXTI_ClearITPendingBit(EXTI_Line15);
}

