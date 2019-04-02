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


#include "STM32_TIM.H"
#include "STM32_WOW.H"
//#include "STM32F10x_BitBand.H"



///* ��ʱ���ṹ�� */
///* TIM Time Base Init structure definition */
//typedef struct
//{
//  u16 TIM_Prescaler;								//-------��Ƶϵ��,=======ȡֵ0x0000~0xFFFF��������ƵTIM clock
//  u16 TIM_CounterMode;							//-------������ʽ========TIM_CounterMode_Up(���ϼ���ģʽ),TIM_CounterMode_Down(���¼���ģʽ),
																			//-----------------------TIM_CounterMode_CenterAligned1(�������ģʽ1����ģʽ),TIM_CounterMode_CenterAligned2(�������ģʽ2����ģʽ),TIM_CounterMode_CenterAligned3(�������ģʽ3����ģʽ)
//  u16 TIM_Period;										//-------����ֵ==========��һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ,ȡֵ0x0000~0xFFFF,����TIM_Period+1�����ĺ�������
//  u16 TIM_ClockDivision;						//-------�趨ʱ��ָ�ֵ===Ĭ��Ϊ0,���ⳡ��ʱTIM_ClockDivision������һ����ʱ,TIM_CKD_DIV1,TIM_CKD_DIV2,TIM_CKD_DIV3
//  u8 TIM_RepetitionCounter;					//-------�ظ���������=====�ظ����ٴ������Ŵ���һ������жϣ�
//} TIM_TimeBaseInitTypeDef;


/*******************************************************************************
* ������		:	PWM_OUT	
* ��������	:		 
* ����		:	PWM_Frequency Ƶ�ʣ���λHz	
* ���		:
* ���� 		:
*******************************************************************************/
void TIM_ConfigurationFreq(TIM_TypeDef* TIMx,u32 Frequency)		//��ʱ��Ƶ�����÷�ʽ����СƵ��1Hz,���100KHz
{
	//*1,�ṹ�嶨��
	//*2,��������
	//*3,�ܽ�ȷ��
	//*4,����Ӧ��ʱ��
	//*5,�ܽ����ã���ʼ����
	//*6,��ʱ�����ã���ʼ����
	//*7,PWM������ã���ʼ����
	//*8,ռ�ձ�����	
		
	//*1,�ṹ�嶨��***********************************************************************
	//1��============================�ṹ�嶨��
//	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;	//��ʱ���ṹ�嶨��	
	RCC_ClocksTypeDef RCC_ClocksStatus;							//ʱ��״̬---ʱ��ֵ
	NVIC_InitTypeDef	NVIC_InitStructure;						//�жϽṹ��
	
	//1��============================��ʱ��������

//	u16 GPIO_Pin_n				=	PWM_Tim->PWM_BasicData.GPIO_Pin_n;
//	double PWM_Frequency	=	2*(PWM_Tim->PWM_BasicData.PWM_Frequency);
	
	u8 TIM_IRQChannel=0;
	u32	Tim_temp				=	2*Frequency;	//���ڷ�ת��Ҫ˫��Ƶ��
	u32	TIMx_Frequency				=	0;			//	��ʱ��ʱ��
	u16 TIMx_Prescaler				=	0	;			//	��ʱ��ʱ�ӷ�Ƶֵ		ȡֵ��Χ��0x0000~0xFFFF
  u16 TIMx_Period						=	0	;			//	��ʱ���Զ���װ��ֵ	ȡֵ��Χ��0x0000~0xFFFF



	//1��============================�򿪶�ʱ��ʱ��
	switch ((u32)TIMx)
	{
		case TIM1_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
			TIM_IRQChannel=TIM1_UP_IRQChannel;	
			TIM_TimeBaseStructure.TIM_RepetitionCounter	=	0;
			break;
		
		case TIM2_BASE:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
			TIM_IRQChannel=TIM2_IRQChannel;
			break;
		
		case TIM3_BASE:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
			TIM_IRQChannel=TIM3_IRQChannel;
			break;
		
		case TIM4_BASE:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
			TIM_IRQChannel=TIM4_IRQChannel;
			break;
		
		case TIM5_BASE:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
			TIM_IRQChannel=TIM5_IRQChannel;
			break;
		
		case TIM6_BASE:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
			TIM_IRQChannel=TIM6_IRQChannel;
			break;
		
		case TIM7_BASE:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
			TIM_IRQChannel=TIM7_IRQChannel;
			break;
		
		case TIM8_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
			TIM_IRQChannel=TIM8_UP_IRQChannel;
			TIM_TimeBaseStructure.TIM_RepetitionCounter	=	0;
			break;
		
		default:
			break;		
	}
	//1��============================��ȡTIMxʱ��Ƶ��
	//1��-----��Ƶֵ���Զ���װ��ֵ���㣨PWM_Frequency Ƶ�ʣ���λHz��
	//--------1MHz 1us=1000ns,1KHz 10us=10000ns
	RCC_GetClocksFreq(&RCC_ClocksStatus);	//��ȡʱ�Ӳ���
	TIMx_Frequency = RCC_ClocksStatus.SYSCLK_Frequency;
	if ((((u32)TIMx)&APB2PERIPH_BASE) == APB2PERIPH_BASE)
  {
    TIMx_Frequency = RCC_ClocksStatus.PCLK2_Frequency;	//APB2
  }
  else
  {
    TIMx_Frequency = RCC_ClocksStatus.PCLK1_Frequency;	//APB1
  }
	//1��============================�����Ƶֵ����װ��ֵ
//	TIMx_Frequency = 72000000;
	//*6.2.4,���㶨ʱ������*********************************************************************
	//Fsys==Fpwm*Count==Fpwm*(Prescaler*Period)	
	//	TIMx_Prescaler				=	72-1		;		// 	��ʱ��ʱ�ӷ�Ƶֵ
	//	TIMx_Period						=	1000-1	;		// 	��ʱ���Զ���װ��ֵ
	//	Tim_num1							=	0				;		//	��ʱ����1
	if(Tim_temp>100000)		//>100KHz
	{
		TIMx_Prescaler=0;
		TIMx_Period=(u16)(TIMx_Frequency/Tim_temp-1);
	}
	else if(Tim_temp>1000)	//>1KHz
	{
		TIMx_Prescaler=10-1;
		TIMx_Period=(u16)((TIMx_Frequency/Tim_temp)/10-1);
	}
	else if(Tim_temp>100)		//>100Hz
	{
		TIMx_Prescaler=100-1;
		TIMx_Period=(u16)((TIMx_Frequency/Tim_temp)/100-1);
	}
	else if(Tim_temp>10)		//>10Hz
	{
		TIMx_Prescaler=1000-1;
		TIMx_Period=(u16)((TIMx_Frequency/Tim_temp)/1000-1);
	}
	else if(Tim_temp<=10)		//<=10Hz
	{
		TIMx_Prescaler=2000-1;
		TIMx_Period=(u16)((TIMx_Frequency/Tim_temp)/2000-1);
	}
	
//	TIMx_Prescaler=0;
//	TIMx_Period=(u16)(5-1);
	
	//6.3��ʱ����ʼ��*********************************************************************
	TIM_TimeBaseStructure.TIM_Prescaler = TIMx_Prescaler; 				//�趨��Ƶֵ
	TIM_TimeBaseStructure.TIM_Period 		= TIMx_Period;        		//�趨�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  				//���ָ�
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  		//���ϼ���
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);		//��ʼ��	
		
	//*6,�ж�����============================================================================
	NVIC_InitStructure.NVIC_IRQChannel = TIM_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Clear TIMx update pending flag[���TIMx����ж�] */
	TIM_ClearFlag(TIMx, TIM_FLAG_Update);

	/* Enable TIM2 Update interrupt [TIMx����ж�����]*/
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE); 
	
	TIM_Cmd(TIMx, DISABLE); 									//ʹ��TIM
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
void TIM_SetFreq(TIM_TypeDef* TIMx,u32 Frequency)		//�趨Ƶ��
{
	
		
	//*1,�ṹ�嶨��***********************************************************************
	//1��============================�ṹ�嶨��

	RCC_ClocksTypeDef RCC_ClocksStatus;							//ʱ��״̬---ʱ��ֵ

	
	//1��============================��ʱ��������


	u32	PWM_Frequency				=	2*Frequency;	//���ڷ�ת��Ҫ˫��Ƶ��
//	u32 RCC_APB2Periph_GPIOx	=	0x00;		//x=A/B/C/D/E/F/G	
	u32	TIMx_Frequency				=	0;			//	��ʱ��ʱ��
	u16 TIMx_Prescaler				=	0	;			//	��ʱ��ʱ�ӷ�Ƶֵ		ȡֵ��Χ��0x0000~0xFFFF
  u16 TIMx_Period						=	0	;			//	��ʱ���Զ���װ��ֵ	ȡֵ��Χ��0x0000~0xFFFF

//	TIMx->CR1 &= ((u16)0x03FE);		//CR1_CEN_Reset�رն�ʱ��
	
	//1��============================��ȡTIMxʱ��Ƶ��
	//1��-----��Ƶֵ���Զ���װ��ֵ���㣨PWM_Frequency Ƶ�ʣ���λHz��
	//--------1MHz 1us=1000ns,1KHz 10us=10000ns
	RCC_GetClocksFreq(&RCC_ClocksStatus);	//��ȡʱ�Ӳ���
	TIMx_Frequency = RCC_ClocksStatus.SYSCLK_Frequency;
	if ((((u32)TIMx)&APB2PERIPH_BASE) == APB2PERIPH_BASE)
  {
    TIMx_Frequency = RCC_ClocksStatus.PCLK2_Frequency;	//APB2
  }
  else
  {
    TIMx_Frequency = RCC_ClocksStatus.PCLK1_Frequency;	//APB1
  }
	//1��============================�����Ƶֵ����װ��ֵ
//	TIMx_Frequency = 72000000;
	//*6.2.4,���㶨ʱ������*********************************************************************
	//Fsys==Fpwm*Count==Fpwm*(Prescaler*Period)	
	//	TIMx_Prescaler				=	72-1		;		// 	��ʱ��ʱ�ӷ�Ƶֵ
	//	TIMx_Period						=	1000-1	;		// 	��ʱ���Զ���װ��ֵ
	//	Tim_num1							=	0				;		//	��ʱ����1
	if(PWM_Frequency<=500)		//<=500Hz
	{
		TIMx_Prescaler=2000;
		TIMx_Period=(u16)(TIMx_Frequency/TIMx_Prescaler/PWM_Frequency);
	}
	else if(PWM_Frequency<=1000)		//<=1KHz
	{
		TIMx_Prescaler=10;
		TIMx_Period=(u16)(TIMx_Frequency/TIMx_Prescaler/PWM_Frequency);
	}
	else if(PWM_Frequency<=5000)	//<=5KHz
	{
		TIMx_Prescaler=2;
		TIMx_Period=(u16)(TIMx_Frequency/TIMx_Prescaler/PWM_Frequency);
	}
	else	//>5KHz
	{
		TIMx_Prescaler=1;
		TIMx_Period=(u16)(TIMx_Frequency/TIMx_Prescaler/PWM_Frequency);
	}
//	else if(PWM_Frequency<100)		//>100kHz
//	{
//		TIMx_Prescaler=100-1;
//		TIMx_Period=(u16)((TIMx_Frequency/PWM_Frequency)/100-1);
//	}
//	else if(PWM_Frequency<10)		//>10Hz
//	{
//		TIMx_Prescaler=1000-1;
//		TIMx_Period=(u16)((TIMx_Frequency/PWM_Frequency)/1000-1);
//	}
//	else if(PWM_Frequency<=10)		//<=10Hz
//	{
//		TIMx_Prescaler=2000-1;
//		TIMx_Period=(u16)((TIMx_Frequency/PWM_Frequency)/2000-1);
//	}
	
//	if(PWM_Frequency>100000)		//>100KHz
//	{
//		TIMx_Prescaler=0;
//		TIMx_Period=(u16)(TIMx_Frequency/PWM_Frequency-1);
//	}
//	else if(PWM_Frequency>1000)	//>1KHz
//	{
//		TIMx_Prescaler=10-1;
//		TIMx_Period=(u16)((TIMx_Frequency/PWM_Frequency)/10-1);
//	}
//	else if(PWM_Frequency>100)		//>100Hz
//	{
//		TIMx_Prescaler=100-1;
//		TIMx_Period=(u16)((TIMx_Frequency/PWM_Frequency)/100-1);
//	}
//	else if(PWM_Frequency>10)		//>10Hz
//	{
//		TIMx_Prescaler=1000-1;
//		TIMx_Period=(u16)((TIMx_Frequency/PWM_Frequency)/1000-1);
//	}
//	else if(PWM_Frequency<=10)		//<=10Hz
//	{
//		TIMx_Prescaler=2000-1;
//		TIMx_Period=(u16)((TIMx_Frequency/PWM_Frequency)/2000-1);
//	}

//		TIMx_Prescaler=0;
//		TIMx_Period=(u16)(5-1);

	//6.3��ʱ����ʼ��*********************************************************************
	
	
	
  /* Set the Prescaler value */
  TIMx->PSC = TIMx_Prescaler-1;
	
	
	
	/* Set the Autoreload value */
  TIMx->ARR = TIMx_Period-1;
	
	/*   */
//  TIMx->CNT = 0;										//�������

	/* Set or reset the UG Bit */
  TIMx->EGR = ((u16)0x0001);								//������Ч��ƵPrescaler
	
//	TIMx->CR1 |= ((u16)0x0001);							//CR1_CEN_Set������ʱ��
}


/*******************************************************************************
*������		: TIM_Configuration
*��������	:��ʱʱ���趨
*����			:TIMx--TIMx--��ʱ����
						x����Ϊ1,2,3,4,5,6,7����8
						Prescaler---��Ƶϵ��,(ȡֵ0x0000~0xFFFF)������ƵTIM clock
						Period	---����ֵ,(ȡֵ0x0000~0xFFFF),����TIM_Period+1�����ĺ������?
						TIM_ClockDivision----�趨ʱ��ָ�ֵ,(Ĭ��Ϊ0,���ⳡ��ʱTIM_ClockDivision������һ����ʱ)
						TIM_CounterMode---������ʽ
						TIM_RepetitionCounter---�ظ���������,(�ظ����ٴ������Ŵ���һ������ж�)
*���			:	��
*����ֵ		:	��
*����			:
*******************************************************************************/
void TIM_Configuration(TIM_TypeDef* TIMx,u16 Prescaler,u16 Period)	//��ʱʱ���趨
{
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;	//��ʱ���ṹ�嶨��
	NVIC_InitTypeDef	NVIC_InitStructure;						//�жϽṹ��
	u8 TIM_IRQChannel=0;
	//1)**********������ر���	
	switch (*(u32*)&TIMx)
	{
		case TIM1_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
			TIM_IRQChannel=TIM1_UP_IRQChannel;
			break;
		
		case TIM2_BASE:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
			TIM_IRQChannel=TIM2_IRQChannel;
			break;
		
		case TIM3_BASE:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
			TIM_IRQChannel=TIM3_IRQChannel;
			break;
		
		case TIM4_BASE:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
			TIM_IRQChannel=TIM4_IRQChannel;
			break;
		
		case TIM5_BASE:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
			TIM_IRQChannel=TIM5_IRQChannel;
			break;
		
		case TIM6_BASE:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
			TIM_IRQChannel=TIM6_IRQChannel;
			break;
		
		case TIM7_BASE:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
			TIM_IRQChannel=TIM7_IRQChannel;
			break;
		
		case TIM8_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
			TIM_IRQChannel=TIM8_UP_IRQChannel;
			break;
		
		default:
			break;
		
	}
	
	TIMx_RCC_ENABLE(TIMx);													//����Ӧ��ʱ��ʱ��
	//1)**********������ر���	
	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler-1; 		// �趨��Ƶֵ
	TIM_TimeBaseStructure.TIM_Period = Period-1;        	//�趨�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //���ָ�
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���
	//	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	//????,???????????????????-??????????
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);		//��ʼ��
	//1)**********������ر���	
	TIM_ARRPreloadConfig(TIMx, ENABLE);
	TIM_Cmd(TIMx, ENABLE); 
	
	//1)**********������ر���	
	NVIC_InitStructure.NVIC_IRQChannel = TIM_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Clear TIMx update pending flag[���TIMx����ж�] */
	TIM_ClearFlag(TIMx, TIM_FLAG_Update);

	/* Enable TIM2 Update interrupt [TIMx����ж�����]*/
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE); 

	/* TIM2 enable counter [ʹ��TIMx����]*/
	TIM_Cmd(TIMx, ENABLE);	//ʹ��TIMx����
}
/*******************************************************************************
*������		:TIMx_RCC_ENABLE
*��������	:����Ӧ��ʱ��ʱ��
*����			:TIMx--��ʱ����
						x����Ϊ1,2,3,4,5,6,7����8
*���			:��
*����ֵ		:��
*����			:TIMx_RCC_ENABLE(TIM1);
*******************************************************************************/
void TIMx_RCC_ENABLE(TIM_TypeDef* TIMx)	//����Ӧ��ʱ��ʱ��
{
	assert_param(IS_TIM_ALL_PERIPH(TIMx)); 
	switch (*(u32*)&TIMx)
	{
		case TIM1_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
			break;
		
		case TIM2_BASE:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
			break;
		
		case TIM3_BASE:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
			break;
		
		case TIM4_BASE:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
			break;
		
		case TIM5_BASE:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
			break;
		
		case TIM6_BASE:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
			break;
		
		case TIM7_BASE:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
			break;
		
		case TIM8_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
			break;
		
		default:
break;
		
	}
}
/*******************************************************************************
*������		: TIM_TIME_SET
*��������	:��ʱʱ���趨
*����			:TIMx--TIMx--��ʱ����
					 x����Ϊ1,2,3,4,5,6,7����8
					Prescaler---��Ƶϵ��,(ȡֵ0x0000~0xFFFF)������ƵTIM clock
					 Period	---����ֵ,(ȡֵ0x0000~0xFFFF),����TIM_Period+1�����ĺ�������
					TIM_ClockDivision----�趨ʱ��ָ�ֵ,(Ĭ��Ϊ0,���ⳡ��ʱTIM_ClockDivision������һ����ʱ)
					TIM_CounterMode---������ʽ
					TIM_RepetitionCounter---�ظ���������,(�ظ����ٴ������Ŵ���һ������ж�)
*���			:��
*����ֵ		:��
*����			:
*******************************************************************************/
void TIM_TIME_SET(TIM_TypeDef* TIMx,u16 Prescaler,u16 Period)	//��ʱʱ���趨
{
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;	//��ʱ���ṹ�嶨��

	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler-1; 		// �趨��Ƶֵ
	TIM_TimeBaseStructure.TIM_Period = Period-1;        	//�趨�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //���ָ�
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���
	//	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	//????,???????????????????-??????????
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);		//��ʼ��
//	TIM_PrescalerConfig(TIM2,Prescaler,TIM_PSCReloadMode_Update);
	TIM_ARRPreloadConfig(TIMx, ENABLE);
//	TIM_Cmd(TIM2, ENABLE);
//	TIM_SetCompare1(TIM2,4000);
}
/*******************************************************************************
*������		:TIM_Interrupt
*��������	:ADS1230�ܽų�ʼ��
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
void TIM_Interrupt(TIM_TypeDef* TIMx,u16 Prescaler,u16 Period)
{
	NVIC_InitTypeDef	NVIC_InitStructure;
	u8 TIM_IRQChannel=0;
	assert_param(IS_TIM_ALL_PERIPH(TIMx)); 
	
	TIM_Configuration(TIMx,Prescaler,Period);
	
	switch (*(u32*)&TIMx)
	{
		case TIM1_BASE:
			TIM_IRQChannel=TIM1_UP_IRQChannel;
			break;
		
		case TIM2_BASE:
			TIM_IRQChannel=TIM2_IRQChannel;
			break;
		
		case TIM3_BASE:
			TIM_IRQChannel=TIM3_IRQChannel;
			break;
		
		case TIM4_BASE:
			TIM_IRQChannel=TIM4_IRQChannel;
			break;
		
		case TIM5_BASE:
			TIM_IRQChannel=TIM5_IRQChannel;
			break;
		
		case TIM6_BASE:
			TIM_IRQChannel=TIM6_IRQChannel;
			break;
		
		case TIM7_BASE:
			TIM_IRQChannel=TIM7_IRQChannel;
			break;
		
		case TIM8_BASE:
			TIM_IRQChannel=TIM8_UP_IRQChannel;
			break;
		
		default:
			break;
		
	}
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Clear TIMx update pending flag[���TIMx����ж�] */
	TIM_ClearFlag(TIMx, TIM_FLAG_Update);

	/* Enable TIM2 Update interrupt [TIMx����ж�����]*/
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE); 

	/* TIM2 enable counter [ʹ��TIMx����]*/
	TIM_Cmd(TIMx, ENABLE);	//ʹ��TIMx����
		
}
/*******************************************************************************
*������		:TIM_Server
*��������	:ADS1230�ܽų�ʼ��
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
void TIM_Server(void)
{
	WOW_Server();															//������
//	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
//	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
//	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
//	TIM_ClearFlag(TIM5, TIM_FLAG_Update);
//	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
//	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
//	TIM_ClearFlag(TIM8, TIM_FLAG_Update);
	
//	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
//	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
//	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
//	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
//	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
//	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
//	TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
}





