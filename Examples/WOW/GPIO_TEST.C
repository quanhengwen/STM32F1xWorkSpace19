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
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_TEST_Configuration(void)
{
	SYS_Configuration();					//系统配置
	GPIO_DeInitAll();							//将所有的GPIO关闭----V20170605
	
	api_gpio_set_rcc_enable(GPIOA);
	
	API_GPIO_SET_OPP50_REG_Pin_xx(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	
	API_GPIO_Toggle(GPIOA,GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_6);		//将GPIO相应管脚输出翻转----V20170605
	
	SysTick_Configuration(1000);	//系统嘀嗒时钟配置72MHz,单位为uS
}
/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_TEST_Server(void)
{
	API_GPIO_Toggle(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);		//将GPIO相应管脚输出翻转----V20170605
}


#endif
