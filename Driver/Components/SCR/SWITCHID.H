
#ifndef __SWITCHID_H
#define __SWITCHID_H

#include "stm32f10x_gpio.h"

//HC165_CONF	HC165x_TEST;			//定义结构体变量



//typedef struct _SwitchData
//{
//	unsigned short SW1:1;
//	unsigned short SW2:1;
//	unsigned short SW3:1;
//	unsigned short SW4:1;
//	unsigned short SW5:1;
//	unsigned short SW6:1;
//	unsigned short SW7:1;
//	unsigned short SW8:1;
//	unsigned short SW9:1;
//	unsigned short SW10:1;
//	unsigned short SW11:1;
//	unsigned short SW12:1;
//	unsigned short SW13:1;
//	unsigned short SW14:1;
//	unsigned short SW15:1;
//	unsigned short SW16:1;	
//}SwitchDataType;			//根据输入的时间相关参数计算出来的相关运行参数值
typedef union _SwitchData
{
	unsigned short id_u16;
	struct{
		unsigned short SW1:1;		//LSB
		unsigned short SW2:1;
		unsigned short SW3:1;
		unsigned short SW4:1;
		unsigned short SW5:1;
		unsigned short SW6:1;
		unsigned short SW7:1;
		unsigned short SW8:1;
		unsigned short SW9:1;
		unsigned short SW10:1;
		unsigned short SW11:1;
		unsigned short SW12:1;
		unsigned short SW13:1;
		unsigned short SW14:1;
		unsigned short SW15:1;
		unsigned short SW16:1;	//MSB
	}id_bit;
}SwitchDataType;			//根据输入的时间相关参数计算出来的相关运行参数值

typedef struct _SwitchConf
{
	
	//____________SW1
	GPIO_TypeDef* 	SW1_PORT;					//GPIOX
	unsigned short 	SW1_Pin;					//GPIO_Pin_x
	
	//____________SW2
	GPIO_TypeDef* 	SW2_PORT;					//GPIOX
	unsigned short 	SW2_Pin;					//GPIO_Pin_x
	
	//____________SW3
	GPIO_TypeDef* 	SW3_PORT;					//GPIOX
	unsigned short 	SW3_Pin;					//GPIO_Pin_x
	
	//____________SW4
	GPIO_TypeDef* 	SW4_PORT;					//GPIOX
	unsigned short 	SW4_Pin;					//GPIO_Pin_x
	
	//____________SW5
	GPIO_TypeDef* 	SW5_PORT;					//GPIOX
	unsigned short 	SW5_Pin;					//GPIO_Pin_x
	
	//____________SW6
	GPIO_TypeDef* 	SW6_PORT;					//GPIOX
	unsigned short 	SW6_Pin;					//GPIO_Pin_x
	
	//____________SW7
	GPIO_TypeDef* 	SW7_PORT;					//GPIOX
	unsigned short 	SW7_Pin;					//GPIO_Pin_x
	
	//____________SW8
	GPIO_TypeDef* 	SW8_PORT;					//GPIOX
	unsigned short 	SW8_Pin;					//GPIO_Pin_x
	
	//____________SW9
	GPIO_TypeDef* 	SW9_PORT;					//GPIOX
	unsigned short 	SW9_Pin;					//GPIO_Pin_x
	
	//____________SW10
	GPIO_TypeDef* 	SW10_PORT;					//GPIOX
	unsigned short 	SW10_Pin;					//GPIO_Pin_x
	
	//____________SW11
	GPIO_TypeDef* 	SW11_PORT;					//GPIOX
	unsigned short 	SW11_Pin;					//GPIO_Pin_x
	
	//____________SW12
	GPIO_TypeDef* 	SW12_PORT;					//GPIOX
	unsigned short 	SW12_Pin;					//GPIO_Pin_x
	
	//____________SW13
	GPIO_TypeDef* 	SW13_PORT;					//GPIOX
	unsigned short 	SW13_Pin;					//GPIO_Pin_x
	
	//____________SW14
	GPIO_TypeDef* 	SW14_PORT;					//GPIOX
	unsigned short 	SW14_Pin;					//GPIO_Pin_x
	
	//____________SW15
	GPIO_TypeDef* 	SW15_PORT;					//GPIOX
	unsigned short 	SW15_Pin;					//GPIO_Pin_x
	
	//____________SW16
	GPIO_TypeDef* 	SW16_PORT;					//GPIOX
	unsigned short 	SW16_Pin;					//GPIO_Pin_x	
		
	unsigned char		NumOfSW;					//接的拔码开关位数,最大16位:配置时需要填入所接的拔码开关位数

	SwitchDataType	SWData;						//结构体位形式:读取的拔码开关值---不需要配置
}SwitchDef;

void api_SwitchId_initialize(SwitchDef *pSwitch);
unsigned short api_get_SwitchId_data_right(SwitchDef *pSwitch);	//按拨码最右边为最高位读取ID
unsigned short api_get_SwitchId_data_left(SwitchDef *pSwitch);	//按拨码最左边为最高位读取ID


#endif


