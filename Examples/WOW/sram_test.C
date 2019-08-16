#ifdef sram_test

#include "sram_test.H"

#include "STM32_SYS.H"
#include "STM32_GPIO.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_TIM.H"

#include "STM32_FSMC.H"

#include "string.h"

#define len 100
const unsigned long sram_addr	=	0x64000000;
unsigned char write_byte[]="单字节系统嘀嗒时钟配置72MHz,单位为uS";
unsigned short write_word[]={0x5A5A,0X5B5B,0X5C6A,0XABCD};
unsigned char read_byte[len]	=	{0};
unsigned short read_word[len]	=	{0};
unsigned short data_word = 0;
unsigned short data_byte = 0;
unsigned char num=0;
static void datatest(void);
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void sram_test_Configuration(void)
{	
	SYS_Configuration();						//系统配置
	api_fsmc_sram_configuration(sram_addr);
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
void sram_test_Server(void)
{  
	IWDG_Feed();								//独立看门狗喂狗
	
	datatest();
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
static void datatest(void)
{	
	//api_fsmc_write_buffer_byte(sram_addr,write_byte,strlen(write_byte));
	api_fsmc_read_buffer_byte(sram_addr,read_byte,strlen((char*)write_byte));
	
	api_fsmc_write_buffer_word(sram_addr,write_word,strlen((const char*)write_word));
	api_fsmc_read_buffer_word(sram_addr,read_word,30);
	if(0x05==data_word)
	{
		num	=	0;
		data_byte	=	0;
		memset(read_byte,0x00,len);
		memset(read_word,0x00,len);
	}
	if(0x05==data_byte)
	{
		num	=	0;
		data_byte	=	0;
		memset(read_byte,0x00,len);
	}
	num++;
}
#endif
