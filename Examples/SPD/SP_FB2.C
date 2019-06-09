#ifdef SP_FB2

#include "SP_FB2.H"

#include "string.h"				//串和内存操作函数头文件
//#include "stm32f10x_dma.h"



#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_ADC.H"
#include "STM32_WDG.H"

#include "STM32_PWM.H"
//#include "STM32_PWM.H"
//#include "STM32_GPIO.H"
#include "STM32_USART.H"
#include "STM32_RTC.H"
#include 	"CRC.H"



/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void SP_FB2_Configuration(void)
{	
//  RCC_ClocksTypeDef RCC_ClocksStatus;							//时钟状态---时钟值
	SYS_Configuration();					//系统配置---打开系统时钟 STM32_SYS.H	
	

	PWM_OUT(TIM2,PWM_OUTChannel1,2,500);						//PWM设定-20161127版本
//	PWM_OUT(TIM3,PWM_OUTChannel1,2700,900);						//PWM设定-20161127版本
//	PWM_OUT(TIM3,PWM_OUTChannel1,2700,900);						//PWM设定-20161127版本
	GPIO_Configuration_OPP50(GPIOA,GPIO_Pin_7);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	GPIO_Configuration_OPP50(GPIOB,GPIO_Pin_0);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	GPIO_SetBits(GPIOA,GPIO_Pin_7);
//  IWDG_Configuration(1000);													//独立看门狗配置---参数单位ms
  SysTick_Configuration(1000);    //系统嘀嗒时钟配置72MHz,单位为uS
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
void SP_FB2_Server(void)
{
	static unsigned char flag = 0;
	static unsigned short time = 0;
	if(time++<500)
		return ;
	time = 0;
	if(0!=flag)
	{
		flag	=	0;
		PWM_OUT(TIM3,PWM_OUTChannel1,2000,1);						//PWM设定-20161127版本
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);
		//GPIO_SetBits(GPIOA,GPIO_Pin_7);
	}
	else
	{
		flag	=	1;
		PWM_OUT(TIM3,PWM_OUTChannel1,3000,50);						//PWM设定-20161127版本
		GPIO_SetBits(GPIOB,GPIO_Pin_0);
		GPIO_ResetBits(GPIOA,GPIO_Pin_7);
	}	
}
//------------------------------------------------------------------------------



#endif
