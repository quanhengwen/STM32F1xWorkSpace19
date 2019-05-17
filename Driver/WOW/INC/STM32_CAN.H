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



#ifndef __STM32_CAN_H
#define __STM32_CAN_H

#include "stm32f10x_can.h"
//#include "stm32f10x_rcc.h"
//#include "stm32f10x_gpio.h"
//#include "stm32f10x_nvic.h"

#include "stm32f10x_type.h"

//#include "CAN_TEST.H"
//#include "STM32_WOW.H"

/* CAN引脚重定义 ----------------------------------------*/
typedef enum
{ 
  CANRemapNone = 0,
  CANRemap1, 
  CANRemap2
}CANRemap_TypeDef;

#define IS_CANRemap_Status(CANRemap_Status) (((CANRemap_Status) == CANRemapNone) || ((CANRemap_Status) == CANRemap1) || \
                              ((CANRemap_Status) == CANRemap2))




//------------------------------------------------------API-配置
void api_can_configuration_std_NR(u32 CAN_BaudRate,u8 CAN_Group,u16 CAN_ID,u16 MaskId);		//can标准数据帧模式配置--使用查询方式获取数据
void api_can_configuration_ext_NR(u32 CAN_BaudRate,u8 CAN_Group,u16 CAN_ID,u16 MaskId);		//can扩展数据帧模式配置--使用查询方式获取数据

void api_can_configuration_std_IT(u32 CAN_BaudRate,u8 CAN_Group,u16 CAN_ID,u16 MaskId);		//can标准数据帧模式配置--使用中断方式获取数据
void api_can_configuration_ext_IT(u32 CAN_BaudRate,u8 CAN_Group,u16 CAN_ID,u16 MaskId);		//can扩展数据帧模式配置--使用中断方式获取数据

//------------------------------------------------------API-数据发送
void api_can_std_send(u16 CAN_ID,u8 length,u8 *Data);			//CAN使用标准帧发送数据
void api_can_ext_send(u32 CAN_ID,u8 length,u8 *Data);			//CAN使用扩展帧发送数据
//------------------------------------------------------API-数据读取
unsigned char api_can_receive(CanRxMsg *RxMessage);									//检查CAN接收有无数据









#endif


