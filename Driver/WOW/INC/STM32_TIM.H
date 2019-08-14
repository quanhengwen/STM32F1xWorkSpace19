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


#ifndef __STM32_TIM_H
#define __STM32_TIM_H

#include "stm32f10x_tim.h"


//#include "STM32_WOW.H"

typedef enum
{
  PWM_OUTChannel1	=	0,
	PWM_OUTChannel2,
	PWM_OUTChannel3,
	PWM_OUTChannel4,
} PWM_OUTChannelTypeDef;

//==============================================================================定时器20190813
void api_tim_configuration(TIM_TypeDef* TIMx,unsigned long microsecond);		//配置定时时间--单位us,最小10us比较稳定

//==============================================================================PWM输入捕获20190813
void api_pwm_capture_configuration(void);

//==============================================================================PWM输出20190813
void api_pwm_oc_configuration(TIM_TypeDef* TIMx,PWM_OUTChannelTypeDef PWM_OUTChanneln,double PWM_Frequency,u16 PWM_Ratio);
void api_pwm_oc_configurationN(TIM_TypeDef* TIMx,PWM_OUTChannelTypeDef PWM_OUTChanneln,double PWM_Frequency,u16 PWM_Ratio);
void api_pwm_oc_set_ratio(TIM_TypeDef* TIMx,PWM_OUTChannelTypeDef PWM_OUTChanneln,u16 PWM_Ratio);



//===========================STATIC--Time
static void tim_rcc_initialize(TIM_TypeDef* TIMx);

static void tim_frequency_initialize(TIM_TypeDef* TIMx,double PWM_Frequency);
static void tim_time_initialize(TIM_TypeDef* TIMx,unsigned long microsecond);
static void tim_Interrupt_initialize(TIM_TypeDef* TIMx);

//===========================STATIC--Pwm
static void pwm_gpio_initialize(TIM_TypeDef* TIMx,PWM_OUTChannelTypeDef PWM_OUTChanneln);
static void pwm_gpio_initializeN(TIM_TypeDef* TIMx,PWM_OUTChannelTypeDef PWM_OUTChanneln);
static void pwm_oc_initialize(TIM_TypeDef* TIMx,PWM_OUTChannelTypeDef PWM_OUTChanneln,u16 PWM_Ratio);
static void pwm_oc_initializeN(TIM_TypeDef* TIMx,PWM_OUTChannelTypeDef PWM_OUTChanneln,u16 PWM_Ratio);


void TIM_Server(void);

#endif


