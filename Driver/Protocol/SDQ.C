#include "SDQ.H"	

#include "stm32f10x_exti.h"

#include "STM32_TIM.H"
#include "STM32_EXTI.H"
#include "STM32_GPIO.H"
#include "STM32_SYSTICK.H"

#include 	"CRC.H"

sdq_def* SDQ_SAMPLE=0;
RCC_ClocksTypeDef bqRCC_ClocksStatus;				//时钟状态---时钟值
unsigned short time_idle=0;										//根据系统时钟计时的时间分频
//unsigned char timeper=0;										//根据系统时钟计时的时间分频
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_sdq_slave_configuration(sdq_def *sSDQ)		//SDQ从机设备配置
{
	if(0==sSDQ->port.SDQ_Port)
		return ;
	SDQ_SAMPLE	=	sSDQ;
	EXTI_Configuration_ITF(sSDQ->port.SDQ_Port, sSDQ->port.SDQ_Pin);		//外部边沿触发中断配置,抢占1，响应1--20171213
	
	
	RCC_GetClocksFreq(&bqRCC_ClocksStatus);		//获取时钟参数
	SDQ_SAMPLE->time.timeper=bqRCC_ClocksStatus.SYSCLK_Frequency/8000000;
//	api_tim3_1us_configuration();
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_sdq_slave_server(void)		//SDQ从机设备服务
{
	//api_sdq_get_byte();
	if(time_idle++>50)
	{
		time_idle	=	0;
		SDQ_SAMPLE->data.messgaelen	=	0;
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
static void api_sdq_process(void)
{
	unsigned long fitime=0;
	//----滤波
	SysTick_DeleyuS(1);						//SysTick延时nuS
	SDQ_SAMPLE->data.intrrcout++;	//中断次数计数
	if(0==api_sdq_get_level())		//低电平
	{
		time_idle	=	0;
		api_sdq_get_byte();
		api_sdq_data_process();
	}
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void api_sdq_data_process(void)
{
	//跳过ID一字节，命令一字节，地址两字节
	if(SDQ_SAMPLE->data.messgaelen>4)
	{
	}
}
//------------------------------------------------------------------------------















/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned char api_sdq_get_bit(void)
{
	unsigned char flag=0;
	SDQ_SAMPLE->time.count=0;
	SDQ_SAMPLE->time.time_sys=0;

	SDQ_SAMPLE->time.time1=SysTick_ReLoad();
	while(0==api_sdq_get_level())
	{
		if(SysTick_GetVal()<2)
		{
			if(0==flag)
			{
				flag=1;
				SDQ_SAMPLE->time.time_sys+=1;
			}
		}
		else
		{
			flag=0;
		}
	}
	SDQ_SAMPLE->time.time2=SysTick_GetVal();							//获取当前SysTick计数值
	
	//------开始前时间少于500，去掉一次计数
	//------计算时长:9000为SYSTICK设置为1us时的重装载值
//	if(SDQ_SAMPLE->time.time2<10)
//	{
//		if(SDQ_SAMPLE->time.time_sys>0)
//			SDQ_SAMPLE->time.time_sys-=1;
//	}
	
	SDQ_SAMPLE->time.time_sys=SDQ_SAMPLE->time.time_sys*9000+SDQ_SAMPLE->time.time1-SDQ_SAMPLE->time.time2;

	//------转换为us
	SDQ_SAMPLE->time.count=SDQ_SAMPLE->time.time_sys/SDQ_SAMPLE->time.timeper;
	
	//------
	//------启动条件：主机拉低信号480us-960us
	if(SDQ_SAMPLE->time.count>400)
	{	
		return 0xFF;
	}
	//------主机写0:拉低信号60us-120us
	else if(SDQ_SAMPLE->time.count>50)
	{
		return 0;
	}
	//------主机写1:拉低信号大于1us，总时间60us
	else if((SDQ_SAMPLE->time.count<15))
	{
		return 1;
	}
	return 0x80;
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned char api_sdq_get_byte(void)
{
	unsigned char temp=api_sdq_get_bit();
	if(0==temp)
	{
		SDQ_SAMPLE->data.temp>>=1;
		SDQ_SAMPLE->data.temp&=0x7F;		
		SDQ_SAMPLE->data.bitlen+=1;
	}
	else if(1==temp)
	{
		SDQ_SAMPLE->data.temp>>=1;
		SDQ_SAMPLE->data.temp|=0x80;		
		SDQ_SAMPLE->data.bitlen+=1;
	}
	else if(0xFF==temp)
	{
		api_sdq_set_start();
	}
	else
	{
		return 0;
	}
	if((SDQ_SAMPLE->data.bitlen>=8))
	{
		if(SDQ_SAMPLE->data.messgaelen<255)
		{
			SDQ_SAMPLE->data.message[SDQ_SAMPLE->data.messgaelen]=SDQ_SAMPLE->data.temp;
			SDQ_SAMPLE->data.messgaelen+=1;
		}
		SDQ_SAMPLE->data.bitlen=0;
		SDQ_SAMPLE->data.temp=0;
	}
	return 1;
}
//------------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned char api_sdq_set_start(void)
{	
	//------启动条件：主机拉低信号480us-960us
	SDQ_SAMPLE->status	=	sdq_start;
	
	SDQ_SAMPLE->time.high	=	0;
	SDQ_SAMPLE->time.low	=	0;

	SDQ_SAMPLE->data.startcout++;

	//SDQ_SAMPLE->data.startcout=0;
//		SDQ_SAMPLE->data.messgaelen	=	0;

	SDQ_SAMPLE->data.bitlen	=	0;	
	SDQ_SAMPLE->data.temp	=	0;
	
	//------等待15us-60us后从机拉低总线60us-240us作为对主机的应答
	GPIO_Configuration_OOD50(SDQ_SAMPLE->port.SDQ_Port, SDQ_SAMPLE->port.SDQ_Pin);			//将GPIO相应管脚配置为OD(开漏)输出模式，最大速度50MHz----V20170605
	GPIO_SetBits(SDQ_SAMPLE->port.SDQ_Port, SDQ_SAMPLE->port.SDQ_Pin);
	SysTick_DeleyuS(20);				//SysTick延时nuS
	GPIO_ResetBits(SDQ_SAMPLE->port.SDQ_Port, SDQ_SAMPLE->port.SDQ_Pin);
	SysTick_DeleyuS(100);				//SysTick延时nuS
	GPIO_SetBits(SDQ_SAMPLE->port.SDQ_Port, SDQ_SAMPLE->port.SDQ_Pin);
	
	//------恢复中断
	EXTI_Configuration_ITF(SDQ_SAMPLE->port.SDQ_Port, SDQ_SAMPLE->port.SDQ_Pin);		//外部边沿触发中断配置,抢占1，响应1--20171213
	
}
//------------------------------------------------------------------------------




//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned char api_sdq_get_level(void)
{
	return GPIO_ReadInputDataBit(SDQ_SAMPLE->port.SDQ_Port, SDQ_SAMPLE->port.SDQ_Pin);
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void EXTI9_5_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line6);
	//=======================高电平:上升沿
	if(0==api_sdq_get_level())	//低电平
	{		
		api_sdq_process();
	}
	//=======================低电平:下降沿
	else
	{
		//----------启动总线
		if(SDQ_SAMPLE->time.count>1000)
		{
			SDQ_SAMPLE->status=sdq_start;
		}
	}
	//EXTI_ClearITPendingBit(EXTI_Line6);
}



