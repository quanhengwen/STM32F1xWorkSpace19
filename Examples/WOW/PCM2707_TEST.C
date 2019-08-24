#ifdef PCM2707_TEST
#include "PCM2707_TEST.H"
#include "PCM2707_TEST.H"

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

#include "EC11Encoder.H"

spi_def PCM2707_spi;
EC11_def	EC11_1;
EC11_def	EC11_2;

EC11_status_def	status;

#define testlen 128
//------------------USB控制接口,高电平使能
#define pcm2707_usb_ctl_port    GPIOB        
#define pcm2707_usb_ctl_pin     GPIO_Pin_11
#define pcm2707_usb_enable     	pcm2707_usb_ctl_port->BSRR  = pcm2707_usb_ctl_pin    //
#define pcm2707_usb_disable    	pcm2707_usb_ctl_port->BRR   = pcm2707_usb_ctl_pin    //
//------------------外部电源控制接口，高电平使能
#define pcm2707_power_port    	GPIOB        
#define pcm2707_power_pin     	GPIO_Pin_2
#define pcm2707_power_enable  	pcm2707_power_port->BSRR  = pcm2707_power_pin    //
#define pcm2707_power_disable  	pcm2707_power_port->BRR   = pcm2707_power_pin    //
//------------------供电:L-自供电;H-总线供电:自供电时，必须设置电流模式为500mA,否则枚举失败
#define pcm2707_PSEL_port 	GPIOA        
#define pcm2707_PSEL_pin  	GPIO_Pin_12
#define pcm2707_PSEL_bus   	pcm2707_PSEL_port->BSRR  = pcm2707_PSEL_pin    //
#define pcm2707_PSEL_self  	pcm2707_PSEL_port->BRR   = pcm2707_PSEL_pin    //
//------------------数字输出:L-I2S out,H-S/PDIF
#define pcm2707_FSEL_port 	GPIOA        
#define pcm2707_FSEL_pin  	GPIO_Pin_10
#define pcm2707_FSEL_pdif  	pcm2707_FSEL_port->BSRR  = pcm2707_FSEL_pin    //
#define pcm2707_FSEL_i2s  	pcm2707_FSEL_port->BRR   = pcm2707_FSEL_pin    //
//------------------HOST:L-100mA,H-500mA:自供电时，必须设置电流模式为500mA,否则枚举失败
#define pcm2707_HOST_port 	GPIOA        
#define pcm2707_HOST_pin  	GPIO_Pin_8
#define pcm2707_HOST_500ma	pcm2707_HOST_port->BSRR  = pcm2707_HOST_pin    //
#define pcm2707_HOST_100ma 	pcm2707_HOST_port->BRR   = pcm2707_HOST_pin    //
//------------------EC11-1
#define pcm2707_EC11_1_key_port 	GPIOA        
#define pcm2707_EC11_1_key_pin  	GPIO_Pin_4
#define pcm2707_EC11_1_A_port 	GPIOA        
#define pcm2707_EC11_1_A_pin  	GPIO_Pin_5
#define pcm2707_EC11_1_B_port 	GPIOA        
#define pcm2707_EC11_1_B_pin  	GPIO_Pin_6

//------------------EC11-2
#define pcm2707_EC11_2_key_port 	GPIOA        
#define pcm2707_EC11_2_key_pin  	GPIO_Pin_7
#define pcm2707_EC11_2_A_port 	GPIOB       
#define pcm2707_EC11_2_A_pin  	GPIO_Pin_0
#define pcm2707_EC11_2_B_port 	GPIOB        
#define pcm2707_EC11_2_B_pin  	GPIO_Pin_1

//------------------背光灯
#define pcm2707_RGB1_R_port 	GPIOA        
#define pcm2707_RGB1_R_pin  	GPIO_Pin_3
#define pcm2707_RGB1_R_ON			pcm2707_RGB1_R_port->BSRR  = pcm2707_RGB1_R_pin    //
#define pcm2707_RGB1_R_OFF 		pcm2707_RGB1_R_port->BRR   = pcm2707_RGB1_R_pin    //
#define pcm2707_RGB1_G_port 	GPIOA        
#define pcm2707_RGB1_G_pin  	GPIO_Pin_2
#define pcm2707_RGB1_G_ON			pcm2707_RGB1_G_port->BSRR  = pcm2707_RGB1_G_pin    //
#define pcm2707_RGB1_G_OFF 		pcm2707_RGB1_G_port->BRR   = pcm2707_RGB1_G_pin    //
#define pcm2707_RGB1_B_port 	GPIOA        
#define pcm2707_RGB1_B_pin  	GPIO_Pin_1
#define pcm2707_RGB1_B_ON			pcm2707_RGB1_B_port->BSRR  = pcm2707_RGB1_B_pin    //
#define pcm2707_RGB1_B_OFF 		pcm2707_RGB1_B_port->BRR   = pcm2707_RGB1_B_pin    //

#define pcm2707_RGB2_R_port 	GPIOC        
#define pcm2707_RGB2_R_pin  	GPIO_Pin_15
#define pcm2707_RGB2_R_ON			pcm2707_RGB2_R_port->BSRR  = pcm2707_RGB2_R_pin    //
#define pcm2707_RGB2_R_OFF 		pcm2707_RGB2_R_port->BRR   = pcm2707_RGB2_R_pin    //
#define pcm2707_RGB2_G_port 	GPIOC        
#define pcm2707_RGB2_G_pin  	GPIO_Pin_14
#define pcm2707_RGB2_G_ON			pcm2707_RGB2_G_port->BSRR  = pcm2707_RGB2_G_pin    //
#define pcm2707_RGB2_G_OFF 		pcm2707_RGB2_G_port->BRR   = pcm2707_RGB2_G_pin    //
#define pcm2707_RGB2_B_port 	GPIOC        
#define pcm2707_RGB2_B_pin  	GPIO_Pin_13
#define pcm2707_RGB2_B_ON			pcm2707_RGB2_B_port->BSRR  = pcm2707_RGB2_B_pin    //
#define pcm2707_RGB2_B_OFF 		pcm2707_RGB2_B_port->BRR   = pcm2707_RGB2_B_pin    //

unsigned  char testrxbuffer[testlen]={0x05};
unsigned  char testtxbuffer[testlen]={0x05};

unsigned long time=	0;
unsigned char Flag=	0;

static void PCM2707_PORT_Configuration(void);
static void PCM2707_RGB_Configuration(void);
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
	api_pwm_oc_configuration(TIM2,PWM_OUTChannel1,0.5,500);	//PWM设定-20161127版本	占空比1/1000
	EC11_Configuration();
	PCM2707_PORT_Configuration();
	PCM2707_RGB_Configuration();
	SysTick_DeleyS(2);					//SysTick延时nS
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
	static unsigned char mute		=	0;
	api_EC11_run_loop(&EC11_1);
	api_EC11_run_loop(&EC11_2);
	
	if(time<1000)
	{
		time++;
		return;
	}
	time=	0;
	//---------------------------------------------------编码器1
	status	=	api_EC11_get_status(&EC11_1);
	if(status.u8b.trigger_key)				//静音
	{
//		if(mute==0)
//		{
//			GPIO_SetBits(GPIOA,GPIO_Pin_15);		//TPA3116D2_MUTE-EN
//			mute	=	1;
//		}
//		else
//		{
//			GPIO_ResetBits(GPIOA,GPIO_Pin_15);	//TPA3116D2_MUTE-DEN
//			mute	=	0;
//		}
		api_spi_write_register_gpio(&PCM2707_spi,0x00,0x01);
		api_spi_write_register_gpio(&PCM2707_spi,0x00,0x00);
	}
	else if(status.u8b.trigger_right)	//声音加
	{
		api_spi_write_register_gpio(&PCM2707_spi,0x00,0x02);
		api_spi_write_register_gpio(&PCM2707_spi,0x00,0x00);
	}
	else if(status.u8b.trigger_left)	//声音减
	{
		api_spi_write_register_gpio(&PCM2707_spi,0x00,0x04);
		api_spi_write_register_gpio(&PCM2707_spi,0x00,0x00);
	}
	//---------------------------------------------------编码器2
	status	=	api_EC11_get_status(&EC11_2);
	if(status.u8b.trigger_key)				//暂停/播放
	{
		api_spi_write_register_gpio(&PCM2707_spi,0x00,0x40);
		api_spi_write_register_gpio(&PCM2707_spi,0x00,0x00);
	}
	else if(status.u8b.trigger_right)	//下一首
	{
		api_spi_write_register_gpio(&PCM2707_spi,0x00,0x08);
		api_spi_write_register_gpio(&PCM2707_spi,0x00,0x00);
	}
	else if(status.u8b.trigger_left)	//上一首
	{
		api_spi_write_register_gpio(&PCM2707_spi,0x00,0x10);
		api_spi_write_register_gpio(&PCM2707_spi,0x00,0x00);
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
static void PCM2707_PORT_Configuration(void)
{
	//供电:L-自供电;H-总线供电
	GPIO_Configuration_OPP50(pcm2707_PSEL_port,pcm2707_PSEL_pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	pcm2707_PSEL_bus;
	pcm2707_PSEL_self;
	
	//HOST:L-100mA,H-500mA
	GPIO_Configuration_OPP50(pcm2707_HOST_port,pcm2707_HOST_pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	pcm2707_HOST_500ma;
	
	//数字输出:L-I2S out,H-S/PDIF
	GPIO_Configuration_OPP50(pcm2707_FSEL_port,pcm2707_FSEL_pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	pcm2707_FSEL_i2s;
	pcm2707_FSEL_pdif;
	
	//USB控制接口,高电平使能
	GPIO_Configuration_OPP50(pcm2707_usb_ctl_port,pcm2707_usb_ctl_pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	pcm2707_usb_enable;
	
	//外部电源控制接口，高电平使能
	GPIO_Configuration_OPP50(pcm2707_power_port,pcm2707_power_pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	pcm2707_power_enable;
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
static void PCM2707_RGB_Configuration(void)
{
	

//------------------背光灯
	GPIO_Configuration_OPP50(pcm2707_RGB1_R_port,pcm2707_RGB1_R_pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	pcm2707_RGB1_R_ON;
	pcm2707_RGB1_R_OFF;
	
	GPIO_Configuration_OPP50(pcm2707_RGB1_G_port,pcm2707_RGB1_G_pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	pcm2707_RGB1_G_ON;
	pcm2707_RGB1_G_OFF;
	
	GPIO_Configuration_OPP50(pcm2707_RGB1_B_port,pcm2707_RGB1_B_pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	pcm2707_RGB1_B_ON;
	pcm2707_RGB1_B_OFF;
	

	GPIO_Configuration_OPP50(pcm2707_RGB2_R_port,pcm2707_RGB2_R_pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	pcm2707_RGB2_R_ON;
	pcm2707_RGB2_R_OFF;
	
	GPIO_Configuration_OPP50(pcm2707_RGB2_G_port,pcm2707_RGB2_G_pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	pcm2707_RGB2_G_ON;
	pcm2707_RGB2_G_OFF;
	
	GPIO_Configuration_OPP50(pcm2707_RGB2_B_port,pcm2707_RGB2_B_pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	pcm2707_RGB2_B_ON;
	pcm2707_RGB2_B_OFF;

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
	PCM2707_spi.port.nss_port		=	GPIOB;
	PCM2707_spi.port.nss_pin		=	GPIO_Pin_12;
	PCM2707_spi.port.clk_port		=	GPIOB;
	PCM2707_spi.port.clk_pin		=	GPIO_Pin_13;
	PCM2707_spi.port.mosi_port	=	GPIOB;
	PCM2707_spi.port.mosi_pin		=	GPIO_Pin_15;	
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
	EC11_1.port.button_port	=	pcm2707_EC11_1_key_port;
	EC11_1.port.button_pin	=	pcm2707_EC11_1_key_pin;
	
	EC11_1.port.A_port			=	pcm2707_EC11_1_A_port;
	EC11_1.port.A_pin				=	pcm2707_EC11_1_A_pin;
	
	EC11_1.port.B_port			=	pcm2707_EC11_1_B_port;
	EC11_1.port.B_pin				=	pcm2707_EC11_1_B_pin;
	
	EC11_1.statictime.ScanTime	=	10;
	api_EC11_configuration_gpio(&EC11_1);
	
	EC11_2.port.button_port	=	pcm2707_EC11_2_key_port;
	EC11_2.port.button_pin	=	pcm2707_EC11_2_key_pin;
	
	EC11_2.port.A_port			=	pcm2707_EC11_2_A_port;
	EC11_2.port.A_pin				=	pcm2707_EC11_2_A_pin;
	
	EC11_2.port.B_port			=	pcm2707_EC11_2_B_port;
	EC11_2.port.B_pin				=	pcm2707_EC11_2_B_pin;
	
	EC11_2.statictime.ScanTime	=	10;
	api_EC11_configuration_gpio(&EC11_2);
}



#endif
