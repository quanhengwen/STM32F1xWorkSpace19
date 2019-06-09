#include "STM32_WOW.H"

#include "STM32_USART.H"
#include "STM32_WDG.H"


__weak void Hardware_Configuration(void){}
__weak void Software_Configuration(void){}
__weak void Data_Initialize(void){}
	
__weak void Data_Server(void){}
__weak void SYS_Server(void){}

/*******************************************************************************
* 函数名	:	WOW_Configuration
* 功能描述	:	配置函数	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void WOW_Configuration(void)
{
	Hardware_Configuration();
	Software_Configuration();
	Data_Initialize();
//	GPIO_DeInit(GPIOA);
//	GPIO_DeInit(GPIOB);
//	GPIO_DeInit(GPIOC);
//	GPIO_DeInit(GPIOD);
//	GPIO_DeInit(GPIOE);
//	GPIO_DeInit(GPIOF);
//	GPIO_DeInit(GPIOG);
	
//	IWDG_Configuration(5000);	//独立看门狗配置 1000ms
	
//	SysTick_Configuration(1000);	//系统嘀嗒时钟配置72MHz,单位为uS
	

	
//***********************************CS5530_DEMO***********************************//
#ifdef SP_FB2	
	SP_FB2_Configuration();
#endif


}













/*******************************************************************************
* 函数名	:	WOW_Server
* 功能描述	:	服务函数 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void WOW_Server(void)
{
//	IWDG_Feed();								//独立看门狗喂狗
  MainServer();          //

//***********************************数字调音板服务程序***********************************//
#ifdef SP_FB2
	SP_FB2_Server();
#endif



//IWDG_Feed();								//独立看门狗喂狗

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
void MainServer(void)
{
  IWDG_Feed();			  //独立看门狗喂狗
  USART_Process();     //串口服务程序
}




