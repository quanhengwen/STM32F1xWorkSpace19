#ifdef mfrc522test

#include "mfrc522test.H"

#include "STM32_SYS.H"
#include "STM32_GPIO.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_TIM.H"

#include "MFRC522.H"

#define mfrc522sysledport GPIOD
#define	mfrc522sysledpin	GPIO_Pin_2
mfrc522def	mfrc522;

unsigned char UID[5]={0};
unsigned char mfrc522_UID[5],mfrc522_Temp[4];

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
	hardware_configuration();
//  IWDG_Configuration(2000);				//独立看门狗配置---参数单位ms
  SysTick_Configuration(1000);    //系统嘀嗒时钟配置72MHz,单位为uS
	while(1)
	{
		
	}
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
//	IWDG_Feed();								//独立看门狗喂狗
//	sys_led_run();
	MFRC522_LOOP();
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
	
	mfrc522.port.rst_port		=	GPIOA;
	mfrc522.port.rst_pin		=	GPIO_Pin_3;
	
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
	GPIO_Configuration_OPP50(GPIOC,GPIO_Pin_13);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
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
	static unsigned short time	=	0;
	if(time++>500)
	{
		time = 0;
//		API_GPIO_Toggle(GPIOC,GPIO_Pin_13);
	}
	GPIO_ResetBits(mfrc522sysledport,mfrc522sysledpin);
	//API_GPIO_Toggle(mfrc522sysledport,mfrc522sysledpin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
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
void MFRC522_LOOP(void)
{
	static unsigned short time = 0;
//	mfrc522_test();
	if(time<10)
	{
		time++;
		return ;
	}
	time	=	0;
//	api_mfrc522_self_test(&mfrc522);
	
//	mfrc522_test();
//	return ;
	

	if(mfrc522_PcdRequest(PICC_REQALL)!=Mifare_none)	//寻卡
	{
		//寻卡成功
		if(mfrc522_PcdAnticoll(mfrc522_UID)==MI_OK)
		{ 
			API_GPIO_Toggle(GPIOC,GPIO_Pin_13);
		}
	}
	
//	if(PcdRequest(PICC_REQALL,mfrc522_Temp)==MI_OK)	//寻卡
//	{
//		//寻卡成功
//		if(PcdAnticoll(mfrc522_UID)==MI_OK)
//		{ 
//			API_GPIO_Toggle(GPIOC,GPIO_Pin_13);
//		}
//	}
		

}
#endif
