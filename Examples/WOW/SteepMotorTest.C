#ifdef SteepMotorTest
#include "SteepMotorTest.H"


#include "SteepMotor.H"
#include "STM32_TIM.H"
#include "SWITCHID.H"
#include "STM32_WDG.H"
#include "STM32_EXTI.H"
#include "STM32_GPIO.H"
#include "STM32_PWM.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_CAN.H"
#include "STM32F10x_BitBand.H"


SteepMotor_Def	SteepMotor1;

u32 SYSTIME	=	0;
u8	Flg	=	0;

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void SteepMotorTest_Configuration(void)
{
	SYS_Configuration();						//ϵͳ���� STM32_SYS.H	
	SysTick_DeleyS(2);						//SysTick��ʱnS
	SysTick_Configuration(1000);		//ϵͳ���ʱ������72MHz,��λΪuS----��ʱɨ��PC006V21_Server
	Motor_Configuration();
}

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void SteepMotorTest_Server(void)
{
	if(StepMotorServer(&SteepMotor1))
	{
		SYSTIME	=	0;
		return;
	}
	SYSTIME++;
//	if(SYSTIME>=10000)
//		SYSTIME	=0;
	if(SYSTIME>=1000)
	{
		SYSTIME	=0;
//		StepMotorCW(&SteepMotor1,40000,1,10000,1,10000,22000);		//˳ʱ����ת
		
		if(Flg	==	0)
		{
			Flg	=	1;
			StepMotorCW(&SteepMotor1,40000,1,10000,1,10000,22000);		//˳ʱ����ת
		}
		else
		{
			Flg	=	0;
			StepMotorCCW(&SteepMotor1,4000,1,10000,1,10000,22000);		//˳ʱ����ת
		}
//		SteepMotor1.SetDIRPort->BRR	=	SteepMotor1.SetDIRPin;		//�͵�ƽ��˳ʱ��ת
//		SteepMotor1.SetPulsPort->BSRR	=	SteepMotor1.SetPulsPin;		//�͵�ƽ��˳ʱ��ת
//		StepMotorCW(&SteepMotor1,500,100,1000,1,50,1000);		//˳ʱ����ת
	}
	else if(SYSTIME%200==100)
	{
//		SteepMotor1.SetDIRPort->BSRR	=	SteepMotor1.SetDIRPin;		//�͵�ƽ��˳ʱ��ת
//		SteepMotor1.SetPulsPort->BRR	=	SteepMotor1.SetPulsPin;		//�͵�ƽ��˳ʱ��ת
//		StepMotorCCW(&SteepMotor1,500,100,1000,1,50,1000);		//˳ʱ����ת
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
void Motor_Configuration(void)
{
	SteepMotor1.SetTIMx			=	TIM1;
	
	SteepMotor1.SetPulsPort	=	GPIOA;
	SteepMotor1.SetPulsPin	=	GPIO_Pin_11;
	
	SteepMotor1.SetDIRPort	=	GPIOA;
	SteepMotor1.SetDIRPin		=	GPIO_Pin_12;
	
	SteepMotor1.SetFrequency	=	1000;
	
	StepMotorConfiguration(&SteepMotor1);		//
}



#endif