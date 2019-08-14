/******************************** User_library *********************************
* 文件名 	: STM32_SDCard.H
* 作者   	: wegam@sina.com
* 版本   	: V
* 日期   	: 2016/01/01
* 说明   	: 
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/


#include "STM32_TIM.H"
#include "STM32_WOW.H"

#include "stm32f10x_nvic.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

//==============================================================================定时器20190813
/*******************************************************************************
*函数名			:	api_tim_configuration
*功能描述		:	配置定时时间--单位us
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_tim_configuration(TIM_TypeDef* TIMx,unsigned long microsecond)
{
	//=============================打开时钟
	tim_rcc_initialize(TIMx);
	//=============================根据时间配置定时器
	tim_time_initialize(TIMx,microsecond);
	//=============================配置中断
	tim_Interrupt_initialize(TIMx);
}
//------------------------------------------------------------------------------

//==============================================================================PWM输入捕获20190813
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_pwm_capture_configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_ICInitTypeDef TIM_ICInitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    //开启TIM2和GPIO时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    //PA0初始化
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;            //下拉输入
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    //TIM2定时器初始化
    TIM_TimeBaseInitStruct.TIM_Period = 1;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 72-1;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;        
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;        //向上计数
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
    
    //TIM2_CH1输入捕获初始化
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICFilter = 0x00;                            //不滤波
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;        //上升沿捕获
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;                //输入器不分频
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;    //映射到IC1
    TIM_ICInit(TIM2, &TIM_ICInitStruct);
    
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStruct.TIM_ICFilter = 0x00;                            //不滤波
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;        //上升沿捕获
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;                //输入器不分频
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;    //映射到IC2
    TIM_ICInit(TIM2, &TIM_ICInitStruct);
    
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_3;
    TIM_ICInitStruct.TIM_ICFilter = 0x00;                            //不滤波
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;        //上升沿捕获
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;                //输入器不分频
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;    //映射到IC3
    TIM_ICInit(TIM2, &TIM_ICInitStruct);
    
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_4;
    TIM_ICInitStruct.TIM_ICFilter = 0x00;                            //不滤波
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;        //上升沿捕获
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;                //输入器不分频
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;    //映射到IC4
    TIM_ICInit(TIM2, &TIM_ICInitStruct);
    
    
    //中断分组初始化
    NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQChannel;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStruct);    
    
    TIM_ITConfig(TIM2, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE);            //更新中断和CC1IE捕获中断
    
    TIM_Cmd(TIM2, ENABLE);
}
//------------------------------------------------------------------------------


//==============================================================================PWM输出20190813
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				:	TIMx							所使用的定时器
							PWM_OUTChanneln		PWM输出通道号
							PWM_Frequency			输出频率，最小频率0.02Hz
							PWM_Ratio					输出占空比，分辨率1/1000
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_pwm_oc_configuration(TIM_TypeDef* TIMx,PWM_OUTChannelTypeDef PWM_OUTChanneln,double PWM_Frequency,u16 PWM_Ratio)
{
	//=============================打开时钟
	tim_rcc_initialize(TIMx);
	
	//=============================配置GPIO为复用推挽输出PB14
	pwm_gpio_initialize(TIMx,PWM_OUTChanneln);
	
	//=============================设置定时时间1000Hz-72MHz(72000000)
	tim_frequency_initialize(TIMx,PWM_Frequency);		
	
	//=============================捕获/比较寄存器1(TIMx_CCR1)---通道1
	pwm_oc_initialize(TIMx,PWM_OUTChanneln,PWM_Ratio);
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				:	TIMx							所使用的定时器
							PWM_OUTChanneln		PWM输出通道号
							PWM_Frequency			输出频率，最小频率0.02Hz
							PWM_Ratio					输出占空比，分辨率1/1000
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_pwm_oc_configurationN(TIM_TypeDef* TIMx,PWM_OUTChannelTypeDef PWM_OUTChanneln,double PWM_Frequency,u16 PWM_Ratio)
{
	//=============================打开时钟
	tim_rcc_initialize(TIMx);
	
	//=============================配置GPIO为复用推挽输出PB14
	pwm_gpio_initializeN(TIMx,PWM_OUTChanneln);
	
	//=============================设置定时时间1000Hz-72MHz(72000000)
	tim_frequency_initialize(TIMx,PWM_Frequency);		
	
	//=============================捕获/比较寄存器1(TIMx_CCR1)---通道1
	pwm_oc_initializeN(TIMx,PWM_OUTChanneln,PWM_Ratio);
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				:	TIMx							所使用的定时器
							PWM_OUTChanneln		PWM输出通道号
							PWM_Frequency			输出频率，最小频率0.02Hz
							PWM_Ratio					输出占空比，分辨率1/1000
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_pwm_oc_set_ratio(TIM_TypeDef* TIMx,PWM_OUTChannelTypeDef PWM_OUTChanneln,u16 PWM_Ratio)
{
	unsigned long TIMx_Ratio	=	0;
	TIMx_Ratio	=	TIMx->ARR+1;
	TIMx_Ratio	=	TIMx_Ratio*PWM_Ratio;
	TIMx_Ratio	=	TIMx_Ratio/1000;

	switch(PWM_OUTChanneln)
	{
		case PWM_OUTChannel1:			//PB13
					TIMx->CCR1 	=	TIMx_Ratio;
		break;
		case PWM_OUTChannel2:			//PB14
					TIMx->CCR2 	=	TIMx_Ratio;
		break;
		case PWM_OUTChannel3:			//PB15	
					TIMx->CCR3 	=	TIMx_Ratio;
		break;
		case PWM_OUTChannel4:			//PB15	
					TIMx->CCR4 	=	TIMx_Ratio;
		break;
		default:
		break;
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
static void tim_rcc_initialize(TIM_TypeDef* TIMx)
{
	//=============================打开时钟
	switch (*(u32*)&TIMx)
	{
		case TIM1_BASE:			
			RCC->APB2ENR |= RCC_APB2Periph_TIM1;
		break;
		case TIM2_BASE:			
			RCC->APB1ENR |= RCC_APB1Periph_TIM2;
		break;
		case TIM3_BASE:			
			RCC->APB1ENR |= RCC_APB1Periph_TIM3;
		break;
		case TIM4_BASE:			
			RCC->APB1ENR |= RCC_APB1Periph_TIM4;
		break;
		case TIM5_BASE:			
			RCC->APB1ENR |= RCC_APB1Periph_TIM5;
		break;
		case TIM6_BASE:			
			RCC->APB1ENR |= RCC_APB1Periph_TIM6;
		break;
		case TIM7_BASE:			
			RCC->APB1ENR |= RCC_APB1Periph_TIM7;
		break;
		case TIM8_BASE:
			RCC->APB2ENR |= RCC_APB2Periph_TIM8;
		break;
		default :break;
	}
	RCC->APB2ENR |= RCC_APB2Periph_AFIO;
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
static void tim_frequency_initialize(TIM_TypeDef* TIMx,double PWM_Frequency)
{
	unsigned long		TIMx_Frequency				=	0;		//	定时器时钟
	unsigned long 	TIMx_Count						=	0;		//	计数个数
	unsigned short	TIMx_Prescaler				=	0	;		//	定时器时钟分频值		取值范围：0x0000~0xFFFF
  unsigned short	TIMx_Period						=	0	;		//	定时器自动重装载值	取值范围：0x0000~0xFFFF
	
	RCC_ClocksTypeDef RCC_ClocksStatus;		//时钟状态---时钟值
	
	if(PWM_Frequency<0)
		return ;
	TIMx->CR1	&=	0xFFFE;		//关定时器
	//=============================获取定时器时钟
	RCC_GetClocksFreq(&RCC_ClocksStatus);	//获取时钟参数	
	TIMx_Frequency = RCC_ClocksStatus.SYSCLK_Frequency;
	if (((*(u32*)&TIMx)&APB2PERIPH_BASE) == APB2PERIPH_BASE)
  {
    TIMx_Frequency = RCC_ClocksStatus.PCLK2_Frequency;	//APB2
  }
  else
  {
    TIMx_Frequency = RCC_ClocksStatus.PCLK1_Frequency;	//APB1
		TIMx_Frequency = TIMx_Frequency*2;									//定时器的时钟频率等于 APB1 的频率两倍
  }
	//=============================根据频率计算周期---单位us
	TIMx_Count	=	TIMx_Frequency/PWM_Frequency;
	//-----------------------------0.02-0.1
	if(PWM_Frequency<1)
	{
		TIMx_Prescaler=60000;
	}
	else if(PWM_Frequency<10)
	{
		TIMx_Prescaler=2000;
	}
	else if(PWM_Frequency<20)
	{
		TIMx_Prescaler=200;
	}
	else if(PWM_Frequency<40)
	{
		TIMx_Prescaler=60;
	}
	else if(PWM_Frequency<80)
	{
		TIMx_Prescaler=30;
	}
	else if(PWM_Frequency<100)
	{
		TIMx_Prescaler=20;
	}
	else if(PWM_Frequency<200)
	{
		TIMx_Prescaler=15;
	}
	else if(PWM_Frequency<400)
	{
		TIMx_Prescaler=6;
	}
	else if(PWM_Frequency<800)
	{
		TIMx_Prescaler=3;
	}
	else if(PWM_Frequency<2000)
	{
		TIMx_Prescaler=2;
	}
	else
	{
		TIMx_Prescaler=1;
	}
	
	TIMx_Period=TIMx_Count/TIMx_Prescaler;

	//=============================预分频器(TIMx_PSC)---分频值
	TIMx->PSC		=	TIMx_Prescaler-1;

	//=============================自动重装载寄存器(TIMx_ARR)
	TIMx->ARR		=	TIMx_Period-1;
	
	TIMx->DIER	=	0x0000;		//关闭中断
	
	TIMx->CR1	|=	0x0001;		//启动定时器
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	tim_time_initialize
*功能描述		:	配置定时时间--单位us
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void tim_time_initialize(TIM_TypeDef* TIMx,unsigned long microsecond)
{	
	unsigned short	TIMx_Prescaler				=	0;		//	定时器时钟分频值		取值范围：0x0000~0xFFFF
  unsigned short	TIMx_Period						=	0;		//	定时器自动重装载值	取值范围：0x0000~0xFFFF
	unsigned long		TIMx_Frequency				=	0;		//	定时器时钟	
	unsigned long		Pre										=	0;
	unsigned long		TimCount							=	0;
	RCC_ClocksTypeDef 										RCC_ClocksStatus;		//时钟状态---时钟值
	
	TIMx->CR1	&=	0xFFFE;		//关定时器
	//=============================获取定时器时钟
	RCC_GetClocksFreq(&RCC_ClocksStatus);	//获取时钟参数	
	TIMx_Frequency = RCC_ClocksStatus.SYSCLK_Frequency;
	if (((*(u32*)&TIMx)&APB2PERIPH_BASE) == APB2PERIPH_BASE)
  {
    TIMx_Frequency = RCC_ClocksStatus.PCLK2_Frequency;	//APB2
  }
  else
  {
    TIMx_Frequency = RCC_ClocksStatus.PCLK1_Frequency;	//APB1
		TIMx_Frequency = TIMx_Frequency*2;									//定时器的时钟频率等于 APB1 的频率两倍
  }	
	//=============================根据频率计算周期---单位us
	Pre=TIMx_Frequency/1000000;
	
	TimCount	=	Pre*microsecond;
	

	if(TimCount/10000<0xFFFF)
	{
		TIMx_Prescaler	=	TimCount/10000+1;
	}
	else if(TimCount/20000<0xFFFF)
	{
		TIMx_Prescaler	=	TimCount/20000+1;
	}
	else if(TimCount/30000<0xFFFF)
	{
		TIMx_Prescaler	=	TimCount/30000+1;			
	}
	else if(TimCount/40000<0xFFFF)
	{
		TIMx_Prescaler	=	TimCount/40000+1;			
	}
	else if(TimCount/50000<0xFFFF)
	{
		TIMx_Prescaler	=	TimCount/50000+1;			
	}
	else if(TimCount/60000<0xFFFF)
	{
		TIMx_Prescaler	=	TimCount/60000+1;			
	}
	TIMx_Period			=	TimCount/TIMx_Prescaler;


	//=============================预分频器(TIMx_PSC)---分频值
	TIMx->PSC		=	TIMx_Prescaler-1;
	
	//=============================自动重装载寄存器(TIMx_ARR)
	TIMx->ARR		=	TIMx_Period-1;
	
	TIMx->DIER	=	0x0000;		//关闭中断
	
	TIMx->CR1	|=	0x0001;		//启动定时器
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名		:TIM_Interrupt
*功能描述	:ADS1230管脚初始化
*输入			:无
*输出			:无
*返回值		:无
*例程			：
*******************************************************************************/
static void tim_Interrupt_initialize(TIM_TypeDef* TIMx)
{
	NVIC_InitTypeDef	NVIC_InitStructure;
	u8 TIM_IRQChannel=0;
	assert_param(IS_TIM_ALL_PERIPH(TIMx)); 
	
	TIM_Cmd(TIMx, DISABLE);	//使能TIMx计数	
	
	switch (*(u32*)&TIMx)
	{
		case TIM1_BASE:
			TIM_IRQChannel=TIM1_UP_IRQChannel;
			break;
		
		case TIM2_BASE:
			TIM_IRQChannel=TIM2_IRQChannel;
			break;
		
		case TIM3_BASE:
			TIM_IRQChannel=TIM3_IRQChannel;
			break;
		
		case TIM4_BASE:
			TIM_IRQChannel=TIM4_IRQChannel;
			break;
		
		case TIM5_BASE:
			TIM_IRQChannel=TIM5_IRQChannel;
			break;
		
		case TIM6_BASE:
			TIM_IRQChannel=TIM6_IRQChannel;
			break;
		
		case TIM7_BASE:
			TIM_IRQChannel=TIM7_IRQChannel;
			break;
		
		case TIM8_BASE:
			TIM_IRQChannel=TIM8_UP_IRQChannel;
			break;
		
		default:
			break;		
	}
	
	NVIC_InitStructure.NVIC_IRQChannel 										= TIM_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 				= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd 								= ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Clear TIMx update pending flag[清除TIMx溢出中断] */
	TIM_ClearFlag(TIMx, TIM_FLAG_Update);

	/* Enable TIM2 Update interrupt [TIMx溢出中断允许]*/
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE); 

	/* TIM2 enable counter [使能TIMx计数]*/
	TIM_Cmd(TIMx, ENABLE);	//使能TIMx计数		
}
//------------------------------------------------------------------------------




/*******************************************************************************
*函数名			:	API_PWM_GPIO_Initialize
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void pwm_gpio_initialize(TIM_TypeDef* TIMx,PWM_OUTChannelTypeDef PWM_OUTChanneln)
{
	switch (*(u32*)&TIMx)
	{
		case TIM1_BASE:
			RCC->APB2ENR |= RCC_APB2Periph_GPIOA;
			switch(PWM_OUTChanneln)
			{
				case PWM_OUTChannel1:			//PA8
						GPIOA->CRH&=0xFFFFFFF0;
						GPIOA->CRH|=0x0000000B;
				break;
				case PWM_OUTChannel2:			//PA9
						GPIOA->CRH&=0xFFFFFF0F;
						GPIOA->CRH|=0x000000B0;
				break;
				case PWM_OUTChannel3:			//PA10	
						GPIOA->CRH&=0xFFFFF0FF;
						GPIOA->CRH|=0x00000B00;
				break;
				case PWM_OUTChannel4:			//PA11	
						GPIOA->CRH&=0xFFFF0FFF;
						GPIOA->CRH|=0x0000B000;
				break;
				default:
				break;
			}
		break;
			
		case TIM2_BASE:
			RCC->APB2ENR |= RCC_APB2Periph_GPIOA;
			switch(PWM_OUTChanneln)
			{
				case PWM_OUTChannel1:			//PA0
						GPIOA->CRL&=0xFFFFFFF0;
						GPIOA->CRL|=0x0000000B;
				break;
				case PWM_OUTChannel2:			//PA1
						GPIOA->CRL&=0xFFFFFF0F;
						GPIOA->CRL|=0x000000B0;
				break;
				case PWM_OUTChannel3:			//PA2	
						GPIOA->CRL&=0xFFFFF0FF;
						GPIOA->CRL|=0x00000B00;
				break;
				case PWM_OUTChannel4:			//PA3	
						GPIOA->CRL&=0xFFFF0FFF;
						GPIOA->CRL|=0x0000B000;
				break;
				default:
				break;
			}
		break;
			
		case TIM3_BASE:			
			switch(PWM_OUTChanneln)
			{
				case PWM_OUTChannel1:			//PA6
						RCC->APB2ENR |= RCC_APB2Periph_GPIOA;
						GPIOA->CRL&=0xF0FFFFFF;
						GPIOA->CRL|=0x0B000000;
				break;
				case PWM_OUTChannel2:			//PA7
						RCC->APB2ENR |= RCC_APB2Periph_GPIOA;
						GPIOA->CRL&=0x0FFFFFFF;
						GPIOA->CRL|=0xB0000000;
				break;
				case PWM_OUTChannel3:			//PB0
						RCC->APB2ENR |= RCC_APB2Periph_GPIOB;
						GPIOB->CRL&=0xFFFFFFF0;
						GPIOB->CRL|=0x0000000B;
				break;
				case PWM_OUTChannel4:			//PB1
						RCC->APB2ENR |= RCC_APB2Periph_GPIOB;
						GPIOB->CRL&=0xFFFFFF0F;
						GPIOB->CRL|=0x000000B0;
				break;
				default:
				break;
			}
		break;
			
		case TIM4_BASE:
			RCC->APB2ENR |= RCC_APB2Periph_GPIOB;
			switch(PWM_OUTChanneln)
			{
				case PWM_OUTChannel1:			//PB6
						GPIOB->CRL&=0xF0FFFFFF;
						GPIOB->CRL|=0x0B000000;
				break;
				case PWM_OUTChannel2:			//PB7
						GPIOB->CRL&=0x0FFFFFFF;
						GPIOB->CRL|=0xB0000000;
				break;
				case PWM_OUTChannel3:			//PB8
						GPIOB->CRH&=0xFFFFFFF0;
						GPIOB->CRH|=0x0000000B;
				break;
				case PWM_OUTChannel4:			//PB9
						GPIOB->CRH&=0xFFFFFF0F;
						GPIOB->CRH|=0x000000B0;
				break;
				default:
				break;
			}
		break;
			
		case TIM5_BASE:			
			RCC->APB2ENR |= RCC_APB2Periph_GPIOA;
			switch(PWM_OUTChanneln)
			{
				case PWM_OUTChannel1:			//PA0
						GPIOA->CRL&=0xFFFFFFF0;
						GPIOA->CRL|=0x0000000B;
				break;
				case PWM_OUTChannel2:			//PA1
						GPIOA->CRL&=0xFFFFFF0F;
						GPIOA->CRL|=0x000000B0;
				break;
				case PWM_OUTChannel3:			//PA2	
						GPIOA->CRL&=0xFFFFF0FF;
						GPIOA->CRL|=0x00000B00;
				break;
				case PWM_OUTChannel4:			//PA3	
						GPIOA->CRL&=0xFFFF0FFF;
						GPIOA->CRL|=0x0000B000;
				break;
				default:
				break;
			}
			
			case TIM6_BASE:				
			RCC->APB2ENR |= RCC_APB2Periph_GPIOA;
			switch(PWM_OUTChanneln)
			{
				case PWM_OUTChannel1:			//PA0
						GPIOA->CRL&=0xFFFFFFF0;
						GPIOA->CRL|=0x0000000B;
				break;
				case PWM_OUTChannel2:			//PA1
						GPIOA->CRL&=0xFFFFFF0F;
						GPIOA->CRL|=0x000000B0;
				break;
				case PWM_OUTChannel3:			//PA2	
						GPIOA->CRL&=0xFFFFF0FF;
						GPIOA->CRL|=0x00000B00;
				break;
				case PWM_OUTChannel4:			//PA3	
						GPIOA->CRL&=0xFFFF0FFF;
						GPIOA->CRL|=0x0000B000;
				break;
				default:
				break;
			}
		
		default :break;
	}
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	API_PWM_GPIO_InitializeN
*功能描述		:	互补输出
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void pwm_gpio_initializeN(TIM_TypeDef* TIMx,PWM_OUTChannelTypeDef PWM_OUTChanneln)
{
	switch (*(u32*)&TIMx)
	{
		case TIM1_BASE:
			RCC->APB2ENR |= RCC_APB2Periph_GPIOB;
			switch(PWM_OUTChanneln)
			{
				case PWM_OUTChannel1:			//PB13
						GPIOB->CRH&=0xFF0FFFFF;
						GPIOB->CRH|=0x00B00000;
				break;
				case PWM_OUTChannel2:			//PB14
						GPIOB->CRH&=0xF0FFFFFF;
						GPIOB->CRH|=0x0B000000;
				break;
				case PWM_OUTChannel3:			//PB15	
						GPIOB->CRH&=0x0FFFFFFF;
						GPIOB->CRH|=0xB0000000;
				break;
				default:
				break;
			}
		break;
		case TIM8_BASE:
			switch(PWM_OUTChanneln)
			{
				case PWM_OUTChannel1:			//PA7
						RCC->APB2ENR |= RCC_APB2Periph_GPIOA;
						GPIOA->CRL&=0x0FFFFFFF;
						GPIOA->CRL|=0xB0000000;
				break;
				case PWM_OUTChannel2:			//PB0
						RCC->APB2ENR |= RCC_APB2Periph_GPIOB;
						GPIOB->CRL&=0xFFFFFFF0;
						GPIOB->CRL|=0x0000000B;
				break;
				case PWM_OUTChannel3:			//PB1
						RCC->APB2ENR |= RCC_APB2Periph_GPIOB;
						GPIOB->CRL&=0xFFFFFF0F;
						GPIOB->CRL|=0x000000B0;
				break;
				default:
				break;
			}
		break;
		default :break;
	}
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	PWM_OC_InitializeN
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void pwm_oc_initialize(TIM_TypeDef* TIMx,PWM_OUTChannelTypeDef PWM_OUTChanneln,u16 PWM_Ratio)
{
	unsigned long TIMx_Ratio	=	0;
	TIMx_Ratio	=	TIMx->ARR+1;
	TIMx_Ratio	=	TIMx_Ratio*PWM_Ratio;
	TIMx_Ratio	=	TIMx_Ratio/1000;

	switch(PWM_OUTChanneln)
	{
		case PWM_OUTChannel1:
					TIMx->CCR1 	=	TIMx_Ratio;
					TIMx->CR2	&=	0xFCFF;	
					TIMx->CR2	|=	0x0300;
					TIMx->CCMR1 &= 0xFF00;
					TIMx->CCMR1 |= 0x006C;
					//TIMx->CCER	&=	0xFFFC;
					TIMx->CCER	&=	~(0x0003<<0);
					TIMx->CCER	|=	(TIM_OutputState_Enable|TIM_OCPolarity_High)<<0;					
		break;
		case PWM_OUTChannel2:
					TIMx->CCR2 	=	TIMx_Ratio;
					TIMx->CR2	&=	0xF3FF;	
					TIMx->CR2	|=	0x0C00;
					TIMx->CCMR1 &= 0x00FF;
					TIMx->CCMR1 |= 0x6C00;
					TIMx->CCER	&=	~(0x0003<<4);
					TIMx->CCER	|=	(TIM_OutputState_Enable|TIM_OCPolarity_High)<<4;
		break;
		case PWM_OUTChannel3:
					TIMx->CCR3 	=	TIMx_Ratio;
					TIMx->CR2	&=	0xCFFF;	
					TIMx->CR2	|=	0x3000;
					TIMx->CCMR2 &= 0xFF00;
					TIMx->CCMR2 |= 0x006C;
					TIMx->CCER	&=	~(0x0003<<8);
					TIMx->CCER	|=	(TIM_OutputState_Enable|TIM_OCPolarity_High)<<8;
		break;
		case PWM_OUTChannel4:
					TIMx->CCR4 	=	TIMx_Ratio;
					TIMx->CR2	&=	0xBFFF;	
					TIMx->CR2	|=	0x4000;
					TIMx->CCMR2 &= 0x00FF;
					TIMx->CCMR2 |= 0x6C00;
					TIMx->CCER	&=	~(0x0003<<12);
					TIMx->CCER	|=	(TIM_OutputState_Enable|TIM_OCPolarity_High)<<12;
		break;
		default:
		break;
	}
	TIMx->BDTR |= 0x8000;
	TIMx->EGR		=	TIM_PSCReloadMode_Immediate;
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	PWM_OC_InitializeN
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void pwm_oc_initializeN(TIM_TypeDef* TIMx,PWM_OUTChannelTypeDef PWM_OUTChanneln,u16 PWM_Ratio)
{
	unsigned long TIMx_Ratio	=	0;
	TIMx_Ratio	=	TIMx->ARR+1;
	TIMx_Ratio	=	TIMx_Ratio*PWM_Ratio;
	TIMx_Ratio	=	TIMx_Ratio/1000;

	switch(PWM_OUTChanneln)
	{
		case PWM_OUTChannel1:			//PB13
					TIMx->CCR1 	=	TIMx_Ratio;
					TIMx->CR2	&=	0xFCFF;	
					TIMx->CR2	|=	0x0300;
					TIMx->CCMR1 &= 0xFF00;
					TIMx->CCMR1 |= 0x006C;
					TIMx->CCER	&=	~(0x000C<<0);
					TIMx->CCER	|=	(TIM_OutputNState_Enable|TIM_OCNPolarity_High)<<0;					
		break;
		case PWM_OUTChannel2:			//PB14
					TIMx->CCR2 	=	TIMx_Ratio;
					TIMx->CR2	&=	0xF3FF;	
					TIMx->CR2	|=	0x0C00;
					TIMx->CCMR1 &= 0x00FF;
					TIMx->CCMR1 |= 0x6C00;
					TIMx->CCER	&=	~(0x000C<<4);
					TIMx->CCER	|=	(TIM_OutputNState_Enable|TIM_OCNPolarity_High)<<4;
		break;
		case PWM_OUTChannel3:			//PB15	
					TIMx->CCR3 	=	TIMx_Ratio;
					TIMx->CR2	&=	0xCFFF;	
					TIMx->CR2	|=	0x3000;
					TIMx->CCMR2 &= 0xFF00;
					TIMx->CCMR2 |= 0x006C;
					TIMx->CCER	&=	~(0x000C<<8);
					TIMx->CCER	|=	(TIM_OutputNState_Enable|TIM_OCNPolarity_High)<<8;
		break;
		default:
		break;
	}
	TIMx->BDTR |= 0x8000;
	TIMx->EGR		=	TIM_PSCReloadMode_Immediate;

}
//------------------------------------------------------------------------------







/*******************************************************************************
*函数名		:TIM_Server
*功能描述	:ADS1230管脚初始化
*输入			:无
*输出			:无
*返回值		:无
*例程			：
*******************************************************************************/
void TIM_Server(void)
{
	WOW_Server();															//服务函数
//	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
//	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
//	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
//	TIM_ClearFlag(TIM5, TIM_FLAG_Update);
//	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
//	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
//	TIM_ClearFlag(TIM8, TIM_FLAG_Update);
	
//	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
//	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
//	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
//	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
//	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
//	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
//	TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
}
//------------------------------------------------------------------------------

//==============================================================================中断服务程序20190813
/*******************************************************************************
* Function Name  : TIM2_IRQHandler
* Description    : This function handles TIM2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM2_IRQHandler(void)
{
	WOW_Server();
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	
}


