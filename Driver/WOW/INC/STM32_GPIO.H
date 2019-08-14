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


void GPIO_DeInitAll(void);																							//将所有的GPIO关闭----V20170605


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













void api_gpio_toggle(GPIO_TypeDef* GPIOx,	u16 GPIO_Pin_n);		//将GPIO相应管脚输出翻转----V20170605


//=================================================
void api_gpio_set_rcc_enable(GPIO_TypeDef* GPIOx);

//=================================================将GPIO相应管脚配置为PP(推挽)输出模式-寄存器版本，最大速度50MHz----V20190805
void api_gpio_set_OPP50_reg_pin_0(GPIO_TypeDef* GPIOx);
void api_gpio_set_OPP50_reg_pin_1(GPIO_TypeDef* GPIOx);
void api_gpio_set_OPP50_reg_pin_2(GPIO_TypeDef* GPIOx);
void api_gpio_set_OPP50_reg_pin_3(GPIO_TypeDef* GPIOx);
void api_gpio_set_OPP50_reg_pin_4(GPIO_TypeDef* GPIOx);
void api_gpio_set_OPP50_reg_pin_5(GPIO_TypeDef* GPIOx);
void api_gpio_set_OPP50_reg_pin_6(GPIO_TypeDef* GPIOx);
void api_gpio_set_OPP50_reg_pin_7(GPIO_TypeDef* GPIOx);
void api_gpio_set_OPP50_reg_pin_8(GPIO_TypeDef* GPIOx);
void api_gpio_set_OPP50_reg_pin_9(GPIO_TypeDef* GPIOx);
void api_gpio_set_OPP50_reg_pin_10(GPIO_TypeDef* GPIOx);
void api_gpio_set_OPP50_reg_pin_11(GPIO_TypeDef* GPIOx);
void api_gpio_set_OPP50_reg_pin_12(GPIO_TypeDef* GPIOx);
void api_gpio_set_OPP50_reg_pin_13(GPIO_TypeDef* GPIOx);
void api_gpio_set_OPP50_reg_pin_14(GPIO_TypeDef* GPIOx);
void api_gpio_set_OPP50_reg_pin_15(GPIO_TypeDef* GPIOx);
void api_gpio_set_OPP50_reg_pin_All(GPIO_TypeDef* GPIOx);
void api_gpio_set_OPP50_reg_pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n);

//=================================================将GPIO相应管脚配置为OD(开漏)输出模式-寄存器版本，最大速度50MHz----V20190805
void api_gpio_set_OOD50_reg_pin_0(GPIO_TypeDef* GPIOx);
void api_gpio_set_OOD50_reg_pin_1(GPIO_TypeDef* GPIOx);
void api_gpio_set_OOD50_reg_pin_2(GPIO_TypeDef* GPIOx);
void api_gpio_set_OOD50_reg_pin_3(GPIO_TypeDef* GPIOx);
void api_gpio_set_OOD50_reg_pin_4(GPIO_TypeDef* GPIOx);
void api_gpio_set_OOD50_reg_pin_5(GPIO_TypeDef* GPIOx);
void api_gpio_set_OOD50_reg_pin_6(GPIO_TypeDef* GPIOx);
void api_gpio_set_OOD50_reg_pin_7(GPIO_TypeDef* GPIOx);
void api_gpio_set_OOD50_reg_pin_8(GPIO_TypeDef* GPIOx);
void api_gpio_set_OOD50_reg_pin_9(GPIO_TypeDef* GPIOx);
void api_gpio_set_OOD50_reg_pin_10(GPIO_TypeDef* GPIOx);
void api_gpio_set_OOD50_reg_pin_11(GPIO_TypeDef* GPIOx);
void api_gpio_set_OOD50_reg_pin_12(GPIO_TypeDef* GPIOx);
void api_gpio_set_OOD50_reg_pin_13(GPIO_TypeDef* GPIOx);
void api_gpio_set_OOD50_reg_pin_14(GPIO_TypeDef* GPIOx);
void api_gpio_set_OOD50_reg_pin_15(GPIO_TypeDef* GPIOx);
void api_gpio_set_OOD50_reg_pin_All(GPIO_TypeDef* GPIOx);
void api_gpio_set_OOD50_reg_pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n);

//=================================================将GPIO相应管脚配置为APP(复用推挽)输出模式-寄存器版本，最大速度50MHz----V20190805
void api_gpio_set_APP50_reg_pin_0(GPIO_TypeDef* GPIOx);
void api_gpio_set_APP50_reg_pin_1(GPIO_TypeDef* GPIOx);
void api_gpio_set_APP50_reg_pin_2(GPIO_TypeDef* GPIOx);
void api_gpio_set_APP50_reg_pin_3(GPIO_TypeDef* GPIOx);
void api_gpio_set_APP50_reg_pin_4(GPIO_TypeDef* GPIOx);
void api_gpio_set_APP50_reg_pin_5(GPIO_TypeDef* GPIOx);
void api_gpio_set_APP50_reg_pin_6(GPIO_TypeDef* GPIOx);
void api_gpio_set_APP50_reg_pin_7(GPIO_TypeDef* GPIOx);
void api_gpio_set_APP50_reg_pin_8(GPIO_TypeDef* GPIOx);
void api_gpio_set_APP50_reg_pin_9(GPIO_TypeDef* GPIOx);
void api_gpio_set_APP50_reg_pin_10(GPIO_TypeDef* GPIOx);
void api_gpio_set_APP50_reg_pin_11(GPIO_TypeDef* GPIOx);
void api_gpio_set_APP50_reg_pin_12(GPIO_TypeDef* GPIOx);
void api_gpio_set_APP50_reg_pin_13(GPIO_TypeDef* GPIOx);
void api_gpio_set_APP50_reg_pin_14(GPIO_TypeDef* GPIOx);
void api_gpio_set_APP50_reg_pin_15(GPIO_TypeDef* GPIOx);
void api_gpio_set_APP50_reg_pin_All(GPIO_TypeDef* GPIOx);
void api_gpio_set_APP50_reg_pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n);

//=================================================将GPIO相应管脚配置为AOD(复用开漏)输出模式-寄存器版本，最大速度50MHz----V20190805
void api_gpio_set_AOD50_reg_pin_0(GPIO_TypeDef* GPIOx);
void api_gpio_set_AOD50_reg_pin_1(GPIO_TypeDef* GPIOx);
void api_gpio_set_AOD50_reg_pin_2(GPIO_TypeDef* GPIOx);
void api_gpio_set_AOD50_reg_pin_3(GPIO_TypeDef* GPIOx);
void api_gpio_set_AOD50_reg_pin_4(GPIO_TypeDef* GPIOx);
void api_gpio_set_AOD50_reg_pin_5(GPIO_TypeDef* GPIOx);
void api_gpio_set_AOD50_reg_pin_6(GPIO_TypeDef* GPIOx);
void api_gpio_set_AOD50_reg_pin_7(GPIO_TypeDef* GPIOx);
void api_gpio_set_AOD50_reg_pin_8(GPIO_TypeDef* GPIOx);
void api_gpio_set_AOD50_reg_pin_9(GPIO_TypeDef* GPIOx);
void api_gpio_set_AOD50_reg_pin_10(GPIO_TypeDef* GPIOx);
void api_gpio_set_AOD50_reg_pin_11(GPIO_TypeDef* GPIOx);
void api_gpio_set_AOD50_reg_pin_12(GPIO_TypeDef* GPIOx);
void api_gpio_set_AOD50_reg_pin_13(GPIO_TypeDef* GPIOx);
void api_gpio_set_AOD50_reg_pin_14(GPIO_TypeDef* GPIOx);
void api_gpio_set_AOD50_reg_pin_15(GPIO_TypeDef* GPIOx);
void api_gpio_set_AOD50_reg_pin_All(GPIO_TypeDef* GPIOx);
void api_gpio_set_AOD50_reg_pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n);


//*************************************************输入模式

//=================================================将GPIO相应管脚配置为INA(模拟)输入模式-寄存器版本----V20190805
void api_gpio_set_INA_reg_pin_0(GPIO_TypeDef* GPIOx);
void api_gpio_set_INA_reg_pin_1(GPIO_TypeDef* GPIOx);
void api_gpio_set_INA_reg_pin_2(GPIO_TypeDef* GPIOx);
void api_gpio_set_INA_reg_pin_3(GPIO_TypeDef* GPIOx);
void api_gpio_set_INA_reg_pin_4(GPIO_TypeDef* GPIOx);
void api_gpio_set_INA_reg_pin_5(GPIO_TypeDef* GPIOx);
void api_gpio_set_INA_reg_pin_6(GPIO_TypeDef* GPIOx);
void api_gpio_set_INA_reg_pin_7(GPIO_TypeDef* GPIOx);
void api_gpio_set_INA_reg_pin_8(GPIO_TypeDef* GPIOx);
void api_gpio_set_INA_reg_pin_9(GPIO_TypeDef* GPIOx);
void api_gpio_set_INA_reg_pin_10(GPIO_TypeDef* GPIOx);
void api_gpio_set_INA_reg_pin_11(GPIO_TypeDef* GPIOx);
void api_gpio_set_INA_reg_pin_12(GPIO_TypeDef* GPIOx);
void api_gpio_set_INA_reg_pin_13(GPIO_TypeDef* GPIOx);
void api_gpio_set_INA_reg_pin_14(GPIO_TypeDef* GPIOx);
void api_gpio_set_INA_reg_pin_15(GPIO_TypeDef* GPIOx);
void api_gpio_set_INA_reg_pin_All(GPIO_TypeDef* GPIOx);
void api_gpio_set_INA_reg_pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n);

//=================================================将GPIO相应管脚配置为INF(浮空)输入模式-寄存器版本----V20190805
void api_gpio_set_INF_reg_pin_0(GPIO_TypeDef* GPIOx);
void api_gpio_set_INF_reg_pin_1(GPIO_TypeDef* GPIOx);
void api_gpio_set_INF_reg_pin_2(GPIO_TypeDef* GPIOx);
void api_gpio_set_INF_reg_pin_3(GPIO_TypeDef* GPIOx);
void api_gpio_set_INF_reg_pin_4(GPIO_TypeDef* GPIOx);
void api_gpio_set_INF_reg_pin_5(GPIO_TypeDef* GPIOx);
void api_gpio_set_INF_reg_pin_6(GPIO_TypeDef* GPIOx);
void api_gpio_set_INF_reg_pin_7(GPIO_TypeDef* GPIOx);
void api_gpio_set_INF_reg_pin_8(GPIO_TypeDef* GPIOx);
void api_gpio_set_INF_reg_pin_9(GPIO_TypeDef* GPIOx);
void api_gpio_set_INF_reg_pin_10(GPIO_TypeDef* GPIOx);
void api_gpio_set_INF_reg_pin_11(GPIO_TypeDef* GPIOx);
void api_gpio_set_INF_reg_pin_12(GPIO_TypeDef* GPIOx);
void api_gpio_set_INF_reg_pin_13(GPIO_TypeDef* GPIOx);
void api_gpio_set_INF_reg_pin_14(GPIO_TypeDef* GPIOx);
void api_gpio_set_INF_reg_pin_15(GPIO_TypeDef* GPIOx);
void api_gpio_set_INF_reg_pin_All(GPIO_TypeDef* GPIOx);
void api_gpio_set_INF_reg_pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n);

//=================================================将GPIO相应管脚配置为IPU(上拉)输入模式-寄存器版本----V20190805
void api_gpio_set_IPU_reg_pin_0(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPU_reg_pin_1(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPU_reg_pin_2(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPU_reg_pin_3(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPU_reg_pin_4(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPU_reg_pin_5(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPU_reg_pin_6(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPU_reg_pin_7(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPU_reg_pin_8(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPU_reg_pin_9(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPU_reg_pin_10(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPU_reg_pin_11(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPU_reg_pin_12(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPU_reg_pin_13(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPU_reg_pin_14(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPU_reg_pin_15(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPU_reg_pin_All(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPU_reg_pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n);

//=================================================将GPIO相应管脚配置为IPD(下拉)输入模式-寄存器版本----V20190805
void api_gpio_set_IPD_reg_pin_0(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPD_reg_pin_1(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPD_reg_pin_2(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPD_reg_pin_3(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPD_reg_pin_4(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPD_reg_pin_5(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPD_reg_pin_6(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPD_reg_pin_7(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPD_reg_pin_8(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPD_reg_pin_9(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPD_reg_pin_10(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPD_reg_pin_11(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPD_reg_pin_12(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPD_reg_pin_13(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPD_reg_pin_14(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPD_reg_pin_15(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPD_reg_pin_All(GPIO_TypeDef* GPIOx);
void api_gpio_set_IPD_reg_pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n);

//void GPIO_Conf(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_x,GPIOMode_TypeDef GPIO_Mode,GPIOSpeed_TypeDef GPIO_Speed);								//GPIO配置
//void GPIO_Configuration(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_x,GPIOMode_TypeDef GPIO_Mode,GPIOSpeed_TypeDef GPIO_Speed);			//GPIO配置
//void GPIO_InitStructure(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_x,GPIOMode_TypeDef GPIO_Mode,GPIOSpeed_TypeDef GPIO_Speed);			//GPIO初始化

#endif


