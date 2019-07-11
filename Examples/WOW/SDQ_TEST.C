

#ifdef SDQ_TEST
#include "SDQ_TEST.H"



#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_PWM.H"
#include "STM32_GPIO.H"


#include "SDQ.H"

sdq_def SDQ_SLAVE;

/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void SDQ_TEST_Configuration(void)
{
  SYS_Configuration();					//系统配置---打开系统时钟 STM32_SYS.H	

	//PWM_OUT(TIM2,PWM_OUTChannel1,1000000,500);	//PWM设定-20161127版本	占空比1/1000
	 GPIO_Configuration_OPP50(GPIOA,GPIO_Pin_0);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	
	SDQ_TEST_SYS_Configuration();
	
	SysTick_Configuration(1000);
//	while(1)
//	{
//		if(1000==TIM3->CR1)
//		{
//			TIM3->CR1=0;
//			GPIO_Toggle(GPIOA,GPIO_Pin_0);		//将GPIO相应管脚输出翻转----V20170605
//		}
//	}
}

/*******************************************************************************
* 函数名		:
* 功能描述	:
* 输入		:
* 输出		:
* 返回 		:
*******************************************************************************/
void SDQ_TEST_Server(void)
{
	api_sdq_slave_server();		//SDQ从机设备配置
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
static void SDQ_TEST_SYS_Configuration(void)
{
	SDQ_SLAVE.port.SDQ_Port	=	GPIOA;
	SDQ_SLAVE.port.SDQ_Pin	=	GPIO_Pin_6;
	
	api_sdq_slave_configuration(&SDQ_SLAVE);		//SDQ从机设备配置
}
//------------------------------------------------------------------------------



#endif
