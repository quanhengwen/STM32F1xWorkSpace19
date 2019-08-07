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


#ifndef __STM32_GPIO_H
#define __STM32_GPIO_H

#include "stm32f10x_gpio.h"
//#include "stm32f10x_rcc.h"
//#include "stm32f10x_type.h"
//#include "stm32f10x_conf.h"
//#include "stm32f10x_map.h"

#include "STM32F10x_BitBand.H"


//#include "STM32_WOW.H"


//#define CPA(x)	(GPIO_TypeDef*)GPIOA,(u16)GPIO_Pin_##x
//#define CPB(x)	(GPIO_TypeDef*)GPIOB,(u16)GPIO_Pin_##x
//#define CPC(x)	(GPIO_TypeDef*)GPIOC,(u16)GPIO_Pin_##x
//#define CPD(x)	(GPIO_TypeDef*)GPIOD,(u16)GPIO_Pin_##x
//#define CPE(x)	(GPIO_TypeDef*)GPIOE,(u16)GPIO_Pin_##x
//#define CPF(x)	(GPIO_TypeDef*)GPIOF,(u16)GPIO_Pin_##x
//#define CPG(x)	(GPIO_TypeDef*)GPIOG,(u16)GPIO_Pin_##x


	
//void GPIO_ClockConf(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_x);																//开启相关GPIO端口时钟----不对外使用

void GPIO_DeInitAll(void);																							//将所有的GPIO关闭----V20170605
void GPIO_Configuration(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n,GPIOMode_TypeDef GPIO_Mode,GPIOSpeed_TypeDef GPIO_Speed);			//GPIO配置

//输入模式
void GPIO_Configuration_INA(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);			//将GPIO相应管脚配置为模拟输入模式----V20170605
void GPIO_Configuration_INF(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);			//将GPIO相应管脚配置为浮空输入模式----V20170605


void GPIO_Configuration_IPD(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);			//将GPIO相应管脚配置为下拉输入模式----V20170605
void GPIO_Configuration_IPU(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);			//将GPIO相应管脚配置为上拉输入模式----V20170605



//输出模式
void GPIO_Configuration_OOD2	(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);			//将GPIO相应管脚配置为OD(开漏)输出模式，最大速度2MHz----V20170605
void GPIO_Configuration_OOD10	(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);			//将GPIO相应管脚配置为OD(开漏)输出模式，最大速度10MHz----V20170605
void GPIO_Configuration_OOD50	(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);			//将GPIO相应管脚配置为OD(开漏)输出模式，最大速度50MHz----V20170605

void GPIO_Configuration_OPP2	(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度2MHz----V20170605
void GPIO_Configuration_OPP10	(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度10MHz----V20170605
void GPIO_Configuration_OPP50	(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
void GPIO_RegConfiguration_OPP50	(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);	//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20190104--寄存器版本

void GPIO_Configuration_AOD2	(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);			//将GPIO相应管脚配置为AOD(复用开漏)输出模式，最大速度2MHz----V20170605
void GPIO_Configuration_AOD10	(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);			//将GPIO相应管脚配置为AOD(复用开漏)输出模式，最大速度10MHz----V20170605
void GPIO_Configuration_AOD50	(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);			//将GPIO相应管脚配置为AOD(复用开漏)输出模式，最大速度50MHz----V20170605

void GPIO_Configuration_APP2	(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);			//将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度2MHz----V20170605
void GPIO_Configuration_APP10	(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);			//将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度10MHz----V20170605
void GPIO_Configuration_APP50	(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);			//将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度50MHz----V20170605



void GPIO_Toggle	(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);		//将GPIO相应管脚输出翻转----V20170605








//=================================================
void API_GPIO_SET_Clock_Eable(GPIO_TypeDef* GPIOx);

//=================================================将GPIO相应管脚配置为PP(推挽)输出模式-寄存器版本，最大速度50MHz----V20190805
void API_GPIO_SET_OPP50_REG_Pin_0(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OPP50_REG_Pin_1(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OPP50_REG_Pin_2(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OPP50_REG_Pin_3(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OPP50_REG_Pin_4(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OPP50_REG_Pin_5(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OPP50_REG_Pin_6(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OPP50_REG_Pin_7(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OPP50_REG_Pin_8(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OPP50_REG_Pin_9(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OPP50_REG_Pin_10(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OPP50_REG_Pin_11(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OPP50_REG_Pin_12(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OPP50_REG_Pin_13(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OPP50_REG_Pin_14(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OPP50_REG_Pin_15(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OPP50_REG_Pin_All(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OPP50_REG_Pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n);

//=================================================将GPIO相应管脚配置为OD(开漏)输出模式-寄存器版本，最大速度50MHz----V20190805
void API_GPIO_SET_OOD50_REG_Pin_0(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OOD50_REG_Pin_1(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OOD50_REG_Pin_2(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OOD50_REG_Pin_3(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OOD50_REG_Pin_4(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OOD50_REG_Pin_5(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OOD50_REG_Pin_6(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OOD50_REG_Pin_7(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OOD50_REG_Pin_8(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OOD50_REG_Pin_9(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OOD50_REG_Pin_10(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OOD50_REG_Pin_11(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OOD50_REG_Pin_12(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OOD50_REG_Pin_13(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OOD50_REG_Pin_14(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OOD50_REG_Pin_15(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OOD50_REG_Pin_All(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_OOD50_REG_Pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n);

//=================================================将GPIO相应管脚配置为APP(复用推挽)输出模式-寄存器版本，最大速度50MHz----V20190805
void API_GPIO_SET_APP50_REG_Pin_0(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_APP50_REG_Pin_1(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_APP50_REG_Pin_2(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_APP50_REG_Pin_3(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_APP50_REG_Pin_4(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_APP50_REG_Pin_5(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_APP50_REG_Pin_6(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_APP50_REG_Pin_7(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_APP50_REG_Pin_8(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_APP50_REG_Pin_9(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_APP50_REG_Pin_10(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_APP50_REG_Pin_11(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_APP50_REG_Pin_12(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_APP50_REG_Pin_13(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_APP50_REG_Pin_14(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_APP50_REG_Pin_15(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_APP50_REG_Pin_All(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_APP50_REG_Pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n);

//=================================================将GPIO相应管脚配置为AOD(复用开漏)输出模式-寄存器版本，最大速度50MHz----V20190805
void API_GPIO_SET_AOD50_REG_Pin_0(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_AOD50_REG_Pin_1(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_AOD50_REG_Pin_2(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_AOD50_REG_Pin_3(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_AOD50_REG_Pin_4(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_AOD50_REG_Pin_5(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_AOD50_REG_Pin_6(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_AOD50_REG_Pin_7(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_AOD50_REG_Pin_8(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_AOD50_REG_Pin_9(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_AOD50_REG_Pin_10(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_AOD50_REG_Pin_11(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_AOD50_REG_Pin_12(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_AOD50_REG_Pin_13(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_AOD50_REG_Pin_14(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_AOD50_REG_Pin_15(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_AOD50_REG_Pin_All(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_AOD50_REG_Pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n);


//*************************************************输入模式

//=================================================将GPIO相应管脚配置为INA(模拟)输入模式-寄存器版本----V20190805
void API_GPIO_SET_INA_REG_Pin_0(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INA_REG_Pin_1(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INA_REG_Pin_2(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INA_REG_Pin_3(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INA_REG_Pin_4(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INA_REG_Pin_5(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INA_REG_Pin_6(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INA_REG_Pin_7(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INA_REG_Pin_8(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INA_REG_Pin_9(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INA_REG_Pin_10(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INA_REG_Pin_11(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INA_REG_Pin_12(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INA_REG_Pin_13(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INA_REG_Pin_14(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INA_REG_Pin_15(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INA_REG_Pin_All(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INA_REG_Pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n);

//=================================================将GPIO相应管脚配置为INF(浮空)输入模式-寄存器版本----V20190805
void API_GPIO_SET_INF_REG_Pin_0(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INF_REG_Pin_1(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INF_REG_Pin_2(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INF_REG_Pin_3(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INF_REG_Pin_4(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INF_REG_Pin_5(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INF_REG_Pin_6(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INF_REG_Pin_7(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INF_REG_Pin_8(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INF_REG_Pin_9(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INF_REG_Pin_10(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INF_REG_Pin_11(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INF_REG_Pin_12(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INF_REG_Pin_13(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INF_REG_Pin_14(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INF_REG_Pin_15(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INF_REG_Pin_All(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_INF_REG_Pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n);

//=================================================将GPIO相应管脚配置为IPU(上拉)输入模式-寄存器版本----V20190805
void API_GPIO_SET_IPU_REG_Pin_0(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPU_REG_Pin_1(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPU_REG_Pin_2(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPU_REG_Pin_3(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPU_REG_Pin_4(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPU_REG_Pin_5(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPU_REG_Pin_6(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPU_REG_Pin_7(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPU_REG_Pin_8(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPU_REG_Pin_9(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPU_REG_Pin_10(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPU_REG_Pin_11(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPU_REG_Pin_12(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPU_REG_Pin_13(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPU_REG_Pin_14(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPU_REG_Pin_15(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPU_REG_Pin_All(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPU_REG_Pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n);

//=================================================将GPIO相应管脚配置为IPD(下拉)输入模式-寄存器版本----V20190805
void API_GPIO_SET_IPD_REG_Pin_0(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPD_REG_Pin_1(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPD_REG_Pin_2(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPD_REG_Pin_3(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPD_REG_Pin_4(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPD_REG_Pin_5(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPD_REG_Pin_6(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPD_REG_Pin_7(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPD_REG_Pin_8(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPD_REG_Pin_9(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPD_REG_Pin_10(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPD_REG_Pin_11(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPD_REG_Pin_12(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPD_REG_Pin_13(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPD_REG_Pin_14(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPD_REG_Pin_15(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPD_REG_Pin_All(GPIO_TypeDef* GPIOx);
void API_GPIO_SET_IPD_REG_Pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n);

//void GPIO_Conf(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_x,GPIOMode_TypeDef GPIO_Mode,GPIOSpeed_TypeDef GPIO_Speed);								//GPIO配置
//void GPIO_Configuration(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_x,GPIOMode_TypeDef GPIO_Mode,GPIOSpeed_TypeDef GPIO_Speed);			//GPIO配置
//void GPIO_InitStructure(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_x,GPIOMode_TypeDef GPIO_Mode,GPIOSpeed_TypeDef GPIO_Speed);			//GPIO初始化

#endif


