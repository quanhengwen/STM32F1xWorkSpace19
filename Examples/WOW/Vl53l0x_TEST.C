#ifdef Vl53l0x_TEST

#include "Vl53l0x_TEST.H"

#include "vl53l0x.h"


#include "STM32_PWM.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_WDG.H"
#include "STM32_SYSTICK.H"
#include "STM32_USART.H"
#include "STM32F10x_BitBand.H"

stvl53l0xDef stvl53l0x;

//#include "STM32_SPI.H"
void APIConfiguration(void);

unsigned	short time=	0;
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void Vl53l0xTest_Configuration(void)
{	
//	SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H
  RCC_Configuration_HSI();
	
	GPIO_DeInitAll();							//�����е�GPIO�ر�----V20170605
	
	SysTick_Configuration(1000);	//ϵͳ���ʱ������72MHz,��λΪuS
	
	APIConfiguration();
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
void Vl53l0xTest_Server(void)
{
	if(time++>1000)
	{
		time	=	0;
		vl53l0x_ReadData();
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
void APIConfiguration(void)
{
	stvl53l0x.Port.SDA_Port	=	GPIOA;	
	stvl53l0x.Port.SDA_Pin	=	GPIO_Pin_10;		//RXD
	
	stvl53l0x.Port.SCL_Port	=	GPIOA;
	stvl53l0x.Port.SCL_Pin	=	GPIO_Pin_9;			//TXD
	
	vl53l0x_Initialize(&stvl53l0x);
	
	vl53l0x_ReadData();
}	


#endif

