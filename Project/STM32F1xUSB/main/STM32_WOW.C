#include "STM32_WOW.H"


//#include "STM32F10x_BitBand.H"
//#include "stm32f10x_map.h"
//#include "STM32_WDG.H"
//#include "STM32_SYSTICK.H"
//#include "STM32_SYS.H"



//#ifndef	Usart_Test
//#define	Usart_Test
//	#include "USART_TEST.H"
//#endif
//#define R2A15908SP
//#define Usart_Test



/*******************************************************************************
* 函数名	:	WOW_Configuration
* 功能描述	:	配置函数	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void WOW_Configuration(void)
{
//	GPIO_DeInit(GPIOA);
//	GPIO_DeInit(GPIOB);
//	GPIO_DeInit(GPIOC);
//	GPIO_DeInit(GPIOD);
//	GPIO_DeInit(GPIOE);
//	GPIO_DeInit(GPIOF);
//	GPIO_DeInit(GPIOG);
	
//	IWDG_Configuration(5000);	//独立看门狗配置 1000ms
	
//	SysTick_Configuration(1000);	//系统嘀嗒时钟配置72MHz,单位为uS

//*********************************Audio_Speaker*********************************//
#ifdef Audio_Speaker
	Audio_Speaker_Configuration();
#endif

//*********************************Audio_Speaker_WOW*********************************//
#ifdef Audio_Speaker_WOW
	Audio_Speaker_Configuration();
#endif
//*********************************Custom_HID*********************************//
#ifdef Custom_HID
	Custom_HID_Configuration();
#endif

//*********************************Device_Firmware_Upgrade*********************************//
#ifdef Device_Firmware_Upgrade
	Device_Firmware_Upgrade_Configuration();
#endif

//*********************************JoyStickMouse*********************************//
#ifdef JoyStickMouse
	JoyStickMouse_Configuration();
#endif

//*********************************Mass_Storage*********************************//
#ifdef Mass_Storage
	Mass_Storage_Configuration();
#endif

//*********************************Virtual_COM_Port*********************************//
#ifdef Virtual_COM_Port
	api_usb_virtual_com_configuration();
#endif

//*********************************Virtual_COM_Port_IAD*********************************//
#ifdef Virtual_COM_Port_IAD
	Virtual_COM_Port_IAD_Configuration();
#endif

//*********************************IAP*********************************//
#ifdef IAP
	IAP_Configuration();
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

//*********************************Audio_Speaker*********************************//
#ifdef Audio_Speaker
	Audio_Speaker_Server();
#endif

//*********************************Audio_Speaker_WOW*********************************//
#ifdef Audio_Speaker
	Audio_Speaker_Server();
#endif
//*********************************Custom_HID*********************************//
#ifdef Custom_HID
	Custom_HID_Server();
#endif

//*********************************Device_Firmware_Upgrade*********************************//
#ifdef Device_Firmware_Upgrade
	Device_Firmware_Upgrade_Server();
#endif

//*********************************JoyStickMouse*********************************//
#ifdef JoyStickMouse
	JoyStickMouse_Server();
#endif

//*********************************Mass_Storage*********************************//
#ifdef Mass_Storage
	Mass_Storage_Server();
#endif

//*********************************Virtual_COM_Port*********************************//
#ifdef Virtual_COM_Port
	api_usb_virtual_com_server();
#endif

//*********************************Virtual_COM_Port_IAD*********************************//
#ifdef Virtual_COM_Port_IAD
	Virtual_COM_Port_IAD_Server();
#endif


//*********************************IAP*********************************//
#ifdef IAP
	IAP_Server();
#endif

}




