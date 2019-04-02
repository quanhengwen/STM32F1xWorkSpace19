/*********************************************
*�����������
*�ṹ�����������ʽ
*����ģʽ---��ʱ
*���⹦��:��ʱ,����
*������ȼ�:ֹͣ--�κ������,��ֹͣ����,����ִ��ֹͣ
*����:ɲ��
**********************************************/

#include "SteepMotor.H"

#include "STM32_GPIO.H"

#include "STM32_TIM.H"


//-----------------------------�������
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	2018/01/02
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void StepMotorConfiguration(SteepMotorDef *STEP_MOTOx)
{
	//==================������������ܽ�
	GPIO_Configuration_OPP50	(STEP_MOTOx->SetPulsPort,	STEP_MOTOx->SetPulsPin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	//==================���÷�������ܽ�
	GPIO_Configuration_OPP50	(STEP_MOTOx->SetDIRPort,	STEP_MOTOx->SetDIRPin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	//==================���ö�ʱ��
	TIM_ConfigurationFreq(STEP_MOTOx->SetTIMx,STEP_MOTOx->SetFrequency);					//��ʱ��Ƶ�����÷�ʽ����СƵ��1Hz,���100KHz
}
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	2018/01/02
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
unsigned char StepMotorServer(SteepMotorDef *STEP_MOTOx)
{
	if((STEP_MOTOx->SetTIMx->SR & TIM_IT_Update)==TIM_IT_Update)			//��ʾ��ʱ�ж�---���м���δ��
	{
		if(STEP_MOTOx->GetPulsTotal>=STEP_MOTOx->SetPulsTotal)
		{
			STEP_MOTOx->SetTIMx->CR1 &= ((u16)0x03FE);		//CR1_CEN_Reset�رն�ʱ��
//			TIM_Cmd(STEP_MOTOx->SetTIMx, DISABLE);
			STEP_MOTOx->SetPulsPort->BRR	=	STEP_MOTOx->SetPulsPin;		//����͵�ƽ
			return 1;
		}
		else if(STEP_MOTOx->PulsFlag	!=	0)
		{
			STEP_MOTOx->PulsFlag	=	0;				//ԭ״̬Ϊ�ߵ�ƽ
			STEP_MOTOx->SetPulsPort->BRR	=	STEP_MOTOx->SetPulsPin;
			STEP_MOTOx->GetPulsTotal++;				//����������
		}
		else if(STEP_MOTOx->PulsFlag	!=	1)
		{
			STEP_MOTOx->PulsFlag	=	1;				//ԭ״̬Ϊ�͵�ƽ
			STEP_MOTOx->SetPulsPort->BSRR	=	STEP_MOTOx->SetPulsPin;			
		}
		
		//==============�Ӽ��ٴ���
		if(STEP_MOTOx->SetPlusUp	&&	STEP_MOTOx->SetPlusUpNum	&&	(STEP_MOTOx->GetPulsTotal<STEP_MOTOx->SetPlusUpNum)	&&(STEP_MOTOx->PulsFlag	!=0))
		{
			STEP_MOTOx->SetFrequency+=STEP_MOTOx->SetPlusUp;
			TIM_SetFreq(STEP_MOTOx->SetTIMx,STEP_MOTOx->SetFrequency);		//�趨Ƶ��
		}
		else if(STEP_MOTOx->SetPlusDown	&&	STEP_MOTOx->SetPlusDownNum	&&	((STEP_MOTOx->GetPulsTotal+STEP_MOTOx->SetPlusDownNum)>STEP_MOTOx->SetPulsTotal)	&&	(STEP_MOTOx->PulsFlag	!=0))
		{
			STEP_MOTOx->SetFrequency-=STEP_MOTOx->SetPlusDown;
			TIM_SetFreq(STEP_MOTOx->SetTIMx,STEP_MOTOx->SetFrequency);		//�趨Ƶ��
		}
		STEP_MOTOx->SetTIMx->SR = (u16)~TIM_IT_Update;			//����жϱ�־
		return 1;
	}
	return 0;
}
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	2018/01/02
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void StepMotorCW(SteepMotorDef *STEP_MOTOx,u16	SetFrequency,u16 SetPlusUp,u16 SetPlusUpNum,u16	SetPlusDown,u16	SetPlusDownNum,u32 SetPulsTotal)		//˳ʱ����ת
{
	if(SetFrequency==0	||	SetPulsTotal==0)
	{
		(STEP_MOTOx->SetPulsPort)->BRR	=	STEP_MOTOx->SetPulsPin;
		return;
	}
	else
	{
		(STEP_MOTOx->SetPulsPort)->BRR	=	STEP_MOTOx->SetDIRPin;		//�͵�ƽ��˳ʱ��ת
		(STEP_MOTOx->SetPulsPort)->BRR	=	STEP_MOTOx->SetPulsPin;		//����͵�ƽ
		
		STEP_MOTOx->SetFrequency		=	SetFrequency;							//��ʼ���Ƶ��
		STEP_MOTOx->SetPlusUp				=	SetPlusUp;								//����Ƶ�ʼ��
		STEP_MOTOx->SetPlusUpNum		=	SetPlusUpNum;							//�����������
		
		STEP_MOTOx->SetPlusDown			=	SetPlusDown;							//����Ƶ�ʼ��
		STEP_MOTOx->SetPlusDownNum	=	SetPlusDownNum;						//�����������
		
		STEP_MOTOx->SetPulsTotal		=	SetPulsTotal;							//��Ҫ�����������
		
		
		STEP_MOTOx->RunFlag					=	0;												//0:δ���У�1-��ʱ������
		STEP_MOTOx->PulsFlag				=	0;												//һ�����������غ��½���Ҫ������ʱ���ж�
		
		STEP_MOTOx->GetFrequency		=	SetFrequency;							//Ƶ�� ��СƵ��1Hz
		STEP_MOTOx->GetPlusUpNum		=	0;												//�����������
		STEP_MOTOx->GetPulsTotal		=	0;												//�Ѿ�����������

		TIM_SetFreq(STEP_MOTOx->SetTIMx,STEP_MOTOx->SetFrequency);		//�趨Ƶ��
		
		STEP_MOTOx->SetTIMx->CR1 |= ((u16)0x0001);							//CR1_CEN_Set������ʱ��

//		TIM_Cmd(STEP_MOTOx->SetTIMx, ENABLE);		
	}
}
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	2018/01/02
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void StepMotorCCW(SteepMotorDef *STEP_MOTOx,u16	SetFrequency,u16 SetPlusUp,u16 SetPlusUpNum,u16	SetPlusDown,u16	SetPlusDownNum,u32 SetPulsTotal)		//��ʱ����ת
{
	if(SetFrequency==0	||	SetPulsTotal==0)
	{
		STEP_MOTOx->SetPulsPort->BRR	=	STEP_MOTOx->SetPulsPin;
		return;
	}
	else
	{
		STEP_MOTOx->SetDIRPort->BSRR	=	STEP_MOTOx->SetDIRPin;	//�ߵ�ƽ����ʱ��ת
		STEP_MOTOx->SetPulsPort->BRR	=	STEP_MOTOx->SetPulsPin;		//����͵�ƽ
		
		STEP_MOTOx->SetFrequency		=	SetFrequency;							//��ʼ���Ƶ��
		STEP_MOTOx->SetPlusUp				=	SetPlusUp;								//����Ƶ�ʼ��
		STEP_MOTOx->SetPlusUpNum		=	SetPlusUpNum;							//�����������
		
		STEP_MOTOx->SetPlusDown			=	SetPlusDown;							//����Ƶ�ʼ��
		STEP_MOTOx->SetPlusDownNum	=	SetPlusDownNum;						//�����������
		
		STEP_MOTOx->SetPulsTotal		=	SetPulsTotal;							//��Ҫ�����������
		
		STEP_MOTOx->RunFlag					=	0;												//0:δ���У�1-��ʱ������
		STEP_MOTOx->PulsFlag				=	0;												//һ�����������غ��½���Ҫ������ʱ���ж�
		
		STEP_MOTOx->GetFrequency		=	SetFrequency;							//Ƶ�� ��СƵ��1Hz
		STEP_MOTOx->GetPlusUpNum		=	0;												//�����������
		STEP_MOTOx->GetPulsTotal		=	0;												//�Ѿ�����������
		
		TIM_SetFreq(STEP_MOTOx->SetTIMx,STEP_MOTOx->SetFrequency);		//�趨Ƶ��
		
		STEP_MOTOx->SetTIMx->CR1 |= ((u16)0x0001);							//CR1_CEN_Set������ʱ��
//		TIM_Cmd(STEP_MOTOx->SetTIMx, ENABLE);		
	}
}
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	2018/01/02
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void StepMotorStop(SteepMotorDef *STEP_MOTOx)
{
	STEP_MOTOx->SetTIMx->CR1 &= ((u16)0x03FE);		//CR1_CEN_Reset�رն�ʱ��
	STEP_MOTOx->SetPulsPort->BRR	=	STEP_MOTOx->SetPulsPin;		//����͵�ƽ
	
	STEP_MOTOx->SetFrequency		=	0;						//��ʼ���Ƶ��
	STEP_MOTOx->SetPlusUp				=	0;						//����Ƶ�ʼ��
	STEP_MOTOx->SetPlusUpNum		=	0;						//�����������
	
	STEP_MOTOx->SetPlusDown			=	0;						//����Ƶ�ʼ��
	STEP_MOTOx->SetPlusDownNum	=	0;						//�����������
	
	STEP_MOTOx->SetPulsTotal		=	0;						//��Ҫ�����������
	
	STEP_MOTOx->RunFlag					=	0;						//0:δ���У�1-��ʱ������
	STEP_MOTOx->PulsFlag				=	0;						//һ�����������غ��½���Ҫ������ʱ���ж�
	
	STEP_MOTOx->GetFrequency		=	0;						//Ƶ�� ��СƵ��1Hz
	STEP_MOTOx->GetPlusUpNum		=	0;						//�����������
	STEP_MOTOx->GetPulsTotal		=	0;						//�Ѿ�����������
}
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	2018/01/02
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void StepMotorPause(SteepMotorDef *STEP_MOTOx)
{

}


