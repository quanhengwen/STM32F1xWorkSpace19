/******************************** User_library *********************************
* 文件名 	: STM32_FSMC.H
* 作者   	: wegam@sina.com
* 版本   	: V
* 日期   	: 2018/11/06
* 说明   	: 
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#include "STM32_FSMC.H"

#include "stm32f10x_rcc.h"
#include "stm32f10x_fsmc.h"

//#include	"stdio.h"			//用于printf
//#include	"string.h"		//用于printf
//#include	"stdarg.h"		//用于获取不确定个数的参数
//#include	"stdlib.h"		//malloc动态申请内存空间
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void FSMC_Initialize(void)
{
 
}
/*******************************************************************************
*函数名			:	LCDFsmc_Initialize
*功能描述		:	FSMC接口LCD初始化：
*输入				: 
*返回值			:	无
*******************************************************************************/
void api_fsmc_sram_configuration(unsigned long address)
{ 
	fsmc_gpio_initialize(address);
	fsmc_sram_initialize(address);
}
//------------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	api_fsmc_write
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_fsmc_write_byte(unsigned long address,unsigned char Data)
{  
  *(volatile unsigned char*)address  = Data;
}
/*******************************************************************************
*函数名			:	api_fsmc_read
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned char api_fsmc_read_byte(unsigned long address)
{  
  return *(volatile unsigned char*)address;
}
/*******************************************************************************
*函数名			:	api_fsmc_write
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_fsmc_write_word(unsigned long address,unsigned short Data)
{  
  *(volatile unsigned short*)address  = Data;
}
/*******************************************************************************
*函数名			:	api_fsmc_read
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short api_fsmc_read_word(unsigned long address)
{  
  return *(volatile unsigned short*)address;
}
/*******************************************************************************
*函数名			:	api_fsmc_write
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_fsmc_write_buffer_byte(unsigned long startaddress,unsigned char* pbuffer,unsigned short len)
{  
	unsigned short i	=	0;
	for(i=0;i<len;i++)
	{
		*(volatile unsigned char*)(startaddress+i)  = pbuffer[i];
	}
}
/*******************************************************************************
*函数名			:	api_fsmc_read
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short api_fsmc_read_buffer_byte(unsigned long startaddress,unsigned char* pbuffer,unsigned short len)
{  
	unsigned short i	=	0;
	for(i=0;i<len;i++)
	{
		pbuffer[i]=*(volatile unsigned char*)(startaddress+i);
	}
	return i;
}
/*******************************************************************************
*函数名			:	api_fsmc_write
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_fsmc_write_buffer_word(unsigned long startaddress,unsigned short* pbuffer,unsigned short len)
{  
	unsigned short i	=	0;
	unsigned long j	=	0;
	for(i=0,j=0;i<len;i++)
	{
		*(volatile unsigned short*)(startaddress+j)  = pbuffer[i];
		j+=2;
	}
}
/*******************************************************************************
*函数名			:	api_fsmc_read
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short api_fsmc_read_buffer_word(unsigned long startaddress,unsigned short* pbuffer,unsigned short len)
{  
	unsigned short i	=	0;
	unsigned long j	=	0;
	for(i=0,j=0;i<len;i++)
	{
		pbuffer[i]=*(volatile unsigned short*)(startaddress+j);
		j+=2;
	}
	return i;
}
//------------------------------------------------------------------------------














//------------------------------------------static
/*******************************************************************************
*函数名			:	fsmc_gpio_initialize
*功能描述		:	FSMC接口GPIO初始化：
*输入				: 
*返回值			:	无
*******************************************************************************/
static void fsmc_gpio_initialize(unsigned long address)
{ 
	#include "stm32f10x_gpio.h"
	unsigned long FSMC_Bank;
	GPIO_InitTypeDef      		GPIO_InitStructure;	
	//--------------------------计算存储块
	if((address<0x60000000)||(address>0x6FFFFFFF))	//非块1，SRAM区
		return;
	if(address<0x64000000)
		FSMC_Bank	=	FSMC_Bank1_NORSRAM1;		//NE1 0x60000000
	else if(address<0x68000000)
		FSMC_Bank	=	FSMC_Bank1_NORSRAM2;		//NE2 0x64000000
	else if(address<0x6C000000)
		FSMC_Bank	=	FSMC_Bank1_NORSRAM3;		//NE3 0x68000000
	else
		FSMC_Bank	=	FSMC_Bank1_NORSRAM4;		//NE4 0x6C000000
	//==========================开时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO, ENABLE);
    
  //===========================FSMC初始化
  
  /*-- GPIO Configuration ------------------------------------------------------*/
	
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;					//最大速度频率50MHz
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF_PP;					//APP(复用推挽)输出模式

	//--------------------------------地址线
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Init(GPIOF,&GPIO_InitStructure);			//GPIOF
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_13|GPIO_Pin_14;
	GPIO_Init(GPIOG,&GPIO_InitStructure);			//GPIOG
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	GPIO_Init(GPIOE,&GPIO_InitStructure);			//GPIOE
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;
	GPIO_Init(GPIOD,&GPIO_InitStructure);			//GPIOD
	
	//--------------------------------数据线
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Init(GPIOD,&GPIO_InitStructure);			//GPIOD
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Init(GPIOE,&GPIO_InitStructure);			//GPIOE
	
	//--------------------------------NOE/RD
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_4;
	GPIO_Init(GPIOD,&GPIO_InitStructure);			//GPIOD
	
	//--------------------------------NWE/WR
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_5;
	GPIO_Init(GPIOD,&GPIO_InitStructure);			//GPIOD
	
	//--------------------------------NADV/ALE
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_7;
	GPIO_Init(GPIOB,&GPIO_InitStructure);			//GPIOB
	
	//--------------------------------NBL0/LB
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_0;
	GPIO_Init(GPIOE,&GPIO_InitStructure);			//GPIOE
	
	//--------------------------------NBL1/UB
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_1;
	GPIO_Init(GPIOE,&GPIO_InitStructure);			//GPIOE
	
	//--------------------------------NE1/CS1
	if(FSMC_Bank1_NORSRAM1	==	FSMC_Bank)		//NE1 0x600000000
	{	
		GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_7;
		GPIO_Init(GPIOD,&GPIO_InitStructure);			//GPIOD
	}
	
	//--------------------------------NE2/CS2
	if(FSMC_Bank1_NORSRAM2	==	FSMC_Bank)		//NE2 0x640000000
	{
		GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_9;
		GPIO_Init(GPIOG,&GPIO_InitStructure);			//GPIOG
	}
	
	//--------------------------------NE3/CS3
	if(FSMC_Bank1_NORSRAM3	==	FSMC_Bank)		//NE3 0x680000000
	{
		GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_10;
		GPIO_Init(GPIOG,&GPIO_InitStructure);			//GPIOG
	}
	
	//--------------------------------NE4/CS4
	if(FSMC_Bank1_NORSRAM4	==	FSMC_Bank)		//NE4 0x6C0000000
	{
		GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_12;
		GPIO_Init(GPIOG,&GPIO_InitStructure);			//GPIOG
	}	
	//--------------------------------CLK
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_3;
	GPIO_Init(GPIOD,&GPIO_InitStructure);			//GPIOD
	
	//--------------------------------NWAIT
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_6;
	GPIO_Init(GPIOD,&GPIO_InitStructure);			//GPIOD

}
/*******************************************************************************
*函数名			:	LCDFsmc_Initialize
*功能描述		:	FSMC接口LCD初始化：
*输入				: 
*返回值			:	无
*******************************************************************************/
static void fsmc_sram_initialize(unsigned long address)
{ 	
	unsigned long FSMC_Bank;
	FSMC_NORSRAMInitTypeDef         FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef   FSMC_NORSRAMTimingInitStructure;	
	//--------------------------计算存储块
	if((address<0x60000000)||(address>0x6FFFFFFF))	//非块1，SRAM区
		return;
	if(address<0x64000000)
		FSMC_Bank	=	FSMC_Bank1_NORSRAM1;		//NE1 0x600000000
	else if(address<0x68000000)
		FSMC_Bank	=	FSMC_Bank1_NORSRAM2;		//NE2 0x640000000
	else if(address<0x6C000000)
		FSMC_Bank	=	FSMC_Bank1_NORSRAM3;		//NE3 0x680000000
	else
		FSMC_Bank	=	FSMC_Bank1_NORSRAM4;		//NE4 0x6C0000000
	//==========================开时钟
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
    
  //===========================FSMC初始化
  
  /*-- FSMC Configuration ------------------------------------------------------*/
  /*----------------------- SRAM Bank 4 ----------------------------------------*/
  /* FSMC_Bank1_NORSRAM4 configuration */
  FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime 	= 1;
  FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime 		= 0;
  FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime 			= 2;
  FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0;
  FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision 				= 0;
  FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency 				= 0;
  FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode 				= FSMC_AccessMode_B;

  /* Color LCD configuration ------------------------------------
     LCD configured as follow:
        - Data/Address MUX = Disable
        - Memory Type = SRAM
        - Data Width = 16bit
        - Write Operation = Enable
        - Extended Mode = Enable
        - Asynchronous Wait = Disable */
  FSMC_NORSRAMInitStructure.FSMC_Bank 							= FSMC_Bank;						//NOR/PSRAM块的第四区，NE4/CS4
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux 		= FSMC_DataAddressMux_Disable;
  FSMC_NORSRAMInitStructure.FSMC_MemoryType 				= FSMC_MemoryType_SRAM;
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth 		= FSMC_MemoryDataWidth_16b;				//数据宽度
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode 		= FSMC_BurstAccessMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStructure.FSMC_WrapMode 					= FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive 	= FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation 		= FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal 				= FSMC_WaitSignal_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode 			= FSMC_ExtendedMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst 				= FSMC_WriteBurst_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct 	= &FSMC_NORSRAMTimingInitStructure;
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct 			= &FSMC_NORSRAMTimingInitStructure;

  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

  /* - BANK 3 (of NOR/SRAM Bank 0~3) is enabled */
  FSMC_NORSRAMCmd(FSMC_Bank, ENABLE);  
}
//-------------------------------------------------------------------------------



