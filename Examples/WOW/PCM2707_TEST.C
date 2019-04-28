#ifdef PCM2707_TEST
#include "PCM2707_TEST.H"
#include "PCM2707_TEST.H"

#include "STM32_SYS.H"
#include "STM32_GPIO.H"
#include "STM32_PWM.H"
#include "STM32_SPI.H"
#include "STM32_SYSTICK.H"
#include "STM32F10x_BitBand.H"


#include	"stdio.h"			//用于printf
#include	"string.h"		//用于printf
#include	"stdarg.h"		//用于获取不确定个数的参数
#include	"stdlib.h"		//malloc动态申请内存空间

#include "EC11Encoder.H"

spi_def PCM2707_spi;
EC11_def	EC11_4;

EC11_status_def	status;

#define testlen 128
unsigned  char testrxbuffer[testlen]={0x05};
unsigned  char testtxbuffer[testlen]={0x05};

unsigned long time=	0;
unsigned char Flag=	0;


static void PCM2707_SPI_Configuration(void);
static void EC11_Configuration(void);
/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void PCM2707_TEST_Configuration(void)
{
  SYS_Configuration();					//系统配置---打开系统时钟 STM32_SYS.H	
	PCM2707_SPI_Configuration();
	EC11_Configuration();
  SysTick_Configuration(10);
}

/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void PCM2707_TEST_Server(void)
{	
	api_EC11_run_loop(&EC11_4);
	if(time<1000)
	{
		time++;
		return;
	}
	time=	0;
	status	=	api_EC11_get_status(&EC11_4);
	if(status.u8b.trigger_key)
	{
		api_spi_write_register_gpio(0x00,0x08);
		api_spi_write_register_gpio(0x00,0x00);
//		api_spi_write_register_gpio(0x00,0x02);
//		api_spi_write_register_gpio(0x00,0x00);
	}
	if(status.u8b.trigger_key)
	{
		api_spi_write_register_gpio(0x00,0x08);
		api_spi_write_register_gpio(0x00,0x00);
	}
	else if(status.u8b.trigger_right)
	{
		api_spi_write_register_gpio(0x00,0x02);
		api_spi_write_register_gpio(0x00,0x00);
	}
	else if(status.u8b.trigger_left)
	{
		api_spi_write_register_gpio(0x00,0x04);
		api_spi_write_register_gpio(0x00,0x00);
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
static void PCM2707_SPI_Configuration(void)
{
	PCM2707_spi.port.nss_port		=	GPIOC;
	PCM2707_spi.port.nss_pin		=	GPIO_Pin_6;
	PCM2707_spi.port.clk_port		=	GPIOC;
	PCM2707_spi.port.clk_pin		=	GPIO_Pin_7;
	PCM2707_spi.port.mosi_port	=	GPIOC;
	PCM2707_spi.port.mosi_pin		=	GPIO_Pin_8;
	
	api_spi_configuration_gpio(&PCM2707_spi);
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
static void EC11_Configuration(void)
{
	EC11_4.port.button_port	=	GPIOB;
	EC11_4.port.button_pin	=	GPIO_Pin_4;
	
	EC11_4.port.A_port			=	GPIOB;
	EC11_4.port.A_pin				=	GPIO_Pin_3;
	
	EC11_4.port.B_port			=	GPIOD;
	EC11_4.port.B_pin				=	GPIO_Pin_2;
	
	EC11_4.statictime.ScanTime	=	10;
	api_EC11_configuration_gpio(&EC11_4);
}



#endif
