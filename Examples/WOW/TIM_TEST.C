#ifdef TIM_TEST

#include "TIM_TEST.H"

#include "string.h"
#include "math.h"


#include "STM32_TIM.H"

#include "STM32_EXTI.H"
#include "STM32_USART.H"

#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_GPIO.H"
#include "STM32F10x_BitBand.H"

#define RxBufferSize	16
#define Ncycle	0





/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void TIM_TEST_Configuration(void)
{
	SYS_Configuration();				//系统配置
	GPIO_DeInitAll();													//将所有的GPIO关闭----V20170605
	//SysTick_Configuration(1000);	//系统嘀嗒时钟配置72MHz,单位为uS

	pwm_test_initialize();
	//time_test_initialize();
	
	api_pwm_capture_configuration();
}
/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void TIM_TEST_Server(void)
{
	time_Interrupt_test();
}

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void time_Interrupt_test(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update))
	{
		api_gpio_toggle(GPIOA,GPIO_Pin_0);		//将GPIO相应管脚输出翻转----V20170605
	}
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void time_test_initialize(void)
{	
	api_gpio_set_rcc_enable(GPIOA);
	api_gpio_set_OPP50_reg_pin_0(GPIOA);
	api_tim_configuration(TIM2,1000);
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void pwm_test_initialize(void)
{
	api_pwm_oc_configurationN(TIM1,PWM_OUTChannel1,100000,100);
	api_pwm_oc_configurationN(TIM1,PWM_OUTChannel2,100000,200);
	api_pwm_oc_configurationN(TIM1,PWM_OUTChannel3,100000,400);
	
	api_pwm_oc_configuration(TIM1,PWM_OUTChannel1,100000,100);
	api_pwm_oc_configuration(TIM1,PWM_OUTChannel2,100000,200);
	api_pwm_oc_configuration(TIM1,PWM_OUTChannel3,100000,400);
	api_pwm_oc_configuration(TIM1,PWM_OUTChannel4,100000,800);

	//api_pwm_oc_configuration(TIM1,PWM_OUTChannel4,5000000,800);
	
	api_pwm_oc_configuration(TIM2,PWM_OUTChannel1,100000,500);
	api_pwm_oc_configuration(TIM2,PWM_OUTChannel2,100000,500);
	api_pwm_oc_configuration(TIM2,PWM_OUTChannel3,100000,500);
	api_pwm_oc_configuration(TIM2,PWM_OUTChannel4,100000,500);
}
//------------------------------------------------------------------------------


#endif
