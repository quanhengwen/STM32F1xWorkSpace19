#ifdef BQ26100_TEST
#include "BQ26100_TEST.H"

#include "BQ26100.H"
#include 	"CRC.H"

#include "STM32_SYS.H"
#include "STM32_GPIO.H"
#include "STM32_TIM.H"
#include "STM32_SPI.H"
#include "STM32_SYSTICK.H"
#include "STM32F10x_BitBand.H"


#include	"stdio.h"			//用于printf
#include	"string.h"		//用于printf
#include	"stdarg.h"		//用于获取不确定个数的参数
#include	"stdlib.h"		//malloc动态申请内存空间

unsigned char api_bq26100_crc=0;
unsigned short bqtestcount=0;
unsigned long testdata=0x68A03D97;
unsigned long retestdata=0;
/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void BQ26100_TEST_Configuration(void)
{
  SYS_Configuration();					//系统配置---打开系统时钟 STM32_SYS.H	
	api_pwm_oc_configuration(TIM2,PWM_OUTChannel1,0.5,500);	//PWM设定-20161127版本	占空比1/1000
	
	api_bq26100_configuration();
	
	SysTick_Configuration(1000);
	
//	while(1)
//	{
//		api_bq26100_test_example_getkey();
//	}
}

/*******************************************************************************
* 函数名		:
* 功能描述	:
* 输入		:
* 输出		:
* 返回 		:
*******************************************************************************/
void BQ26100_TEST_Server(void)
{	
	static unsigned short time = 0;
	
	//static unsigned short i = 0;
	unsigned char buffer[128]={0};
	if(time++<50)
		return ;
	time = 0;
	api_bq26100_test_example();
	//api_bq26100_read_id(buffer);	//读ID0x33
//	api_bq26100_skip_id();	//0xCC
	//api_bq26100_read_memory(buffer);
	//api_bq26100_read_status(buffer);
	//retestdata=api_bq26100_test_example_getkey2(testdata);
	return ;
	
	
//	if(bqtestcount<0x0100)
//	{
//		bqtestcount++;
//		api_bq26100_test_example();
//	}	

}




#endif
