#ifdef mfrc522test

#include "mfrc522test.H"

#include "STM32_SYS.H"
#include "STM32_GPIO.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_PWM.H"

#include "MFRC522.H"

#define mfrc522sysledport GPIOD
#define	mfrc522sysledpin	GPIO_Pin_2
mfrc522def	mfrc522;


/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void mfrc522test_Configuration(void)
{	
	
	SYS_Configuration();						//系统配置
	sys_led_conf();
  IWDG_Configuration(2000);				//独立看门狗配置---参数单位ms
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
void mfrc522test_Server(void)
{  
	IWDG_Feed();								//独立看门狗喂狗
	sys_led_run();
}
//------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void hardware_configuration(void)
{
	mfrc522.port.nss_port		=	GPIOA;
	mfrc522.port.nss_pin		=	GPIO_Pin_4;
	
	mfrc522.port.clk_port		=	GPIOA;
	mfrc522.port.clk_pin		=	GPIO_Pin_5;
	
	mfrc522.port.miso_port	=	GPIOA;
	mfrc522.port.miso_pin		=	GPIO_Pin_6;
	
	mfrc522.port.mosi_port	=	GPIOA;
	mfrc522.port.mosi_pin		=	GPIO_Pin_7;
	
	api_mfrc522_configuration(&mfrc522);
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
static void sys_led_conf(void)
{
	GPIO_Configuration_OPP50(mfrc522sysledport,mfrc522sysledpin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
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
static void sys_led_run(void)
{
	GPIO_ResetBits(mfrc522sysledport,mfrc522sysledpin);
	//GPIO_Toggle(mfrc522sysledport,mfrc522sysledpin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
}
#endif
