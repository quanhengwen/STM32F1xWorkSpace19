#ifdef GPIO_TEST
#include "GPIO_TEST.H"

#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_GPIO.H"
#include "STM32F10x_BitBand.H"


volatile GPIO_TypeDef* 	GPIOx;			//x=A/B/C/D/E/F/G
volatile TIM_TypeDef* 	TIMx;
volatile u32* p	=	0;
volatile u32 temp	=	0;
u16 systime	=	0;
volatile u16 Pin	=	0;
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void GPIO_TEST_Configuration(void)
{
	SYS_Configuration();					//ϵͳ����
	GPIO_DeInitAll();							//�����е�GPIO�ر�----V20170605
	
	api_gpio_set_rcc_enable(GPIOA);
	
	API_GPIO_SET_OPP50_REG_Pin_xx(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	
	API_GPIO_Toggle(GPIOA,GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_6);		//��GPIO��Ӧ�ܽ������ת----V20170605
	
	SysTick_Configuration(1000);	//ϵͳ���ʱ������72MHz,��λΪuS
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void GPIO_TEST_Server(void)
{
	API_GPIO_Toggle(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);		//��GPIO��Ӧ�ܽ������ת----V20170605
}


#endif