#ifndef __EC11Encoder_H
#define __EC11Encoder_H
#include "stm32f10x_gpio.h"

typedef union	_EC11_status
{
	unsigned char u8s;
	struct
	{
		unsigned char trigger_key 		:1;	//bit0//按键触发标志
		unsigned char trigger_right 	:1;	//bit1//A触发标志--CW顺时针信号-右转
		unsigned char trigger_left 		:1;	//bit2//B触发标志--CCW逆时针信号-左转
		unsigned char trigger_long 		:1;	//bit3//长按标志--连续检测到达一定时间
		unsigned char trigger_ecoder 	:1;	//bit4//编码器已检测标志
	}u8b;
}EC11_status_def;				//编码器输出值
typedef struct	_EC11_port
{
	//需要配置参数：
	//Button
	GPIO_TypeDef* 	button_port;			//GPIOX
	unsigned short 	button_pin;			//GPIO_Pin_x	
	//KEY-A
	GPIO_TypeDef* 	A_port;					//GPIOX
	unsigned short 	A_pin;						//GPIO_Pin_x	
	//KEY-B
	GPIO_TypeDef* 	B_port;					//GPIOX
	unsigned short 	B_pin;						//GPIO_Pin_x
}EC11_port_def;				//编码器输出值
typedef struct	_EC11_conf_time
{
	//需要配置参数：
	unsigned 	char	ScanTime;									//扫描时间：单位us
	unsigned	char	LongPressFlag;						//长按标志，LongPressFlag=0表示每次只允许一次按压，长按只当一次，LongPressFlag=1；长按达到LongPressStartTime时间后表示长按，后面每计时达到LongPressEffectiveTime后按键次数增加一次
	unsigned 	long	LongPressStartTime;				//长按有效起始时间--LongPressStartTime计时到有效时间表示按键有效，长按3秒生效
	unsigned 	long	LongPressEffectiveTime;		//长按按键单步有效时间--ButtonTimeCount计时到有效时间表示按键有效0.2秒返回一个有效按键
	
	unsigned 	long	ButtonEffectiveTime;			//按键有效时间--ButtonTimeCount计时到有效时间表示按键有效---默认干扰时间10us
	unsigned 	long	EncoderEffectiveTime;			//编码器有效时间---默认干扰时间10us
}EC11_conf_time_def;				//编码器输出值
typedef struct	_EC11_running_time
{
	//不需要配置参数：
	//按键计时器
	unsigned 	long	PressTime;		//按键时间
	unsigned 	long	PressNoiseTime;		//按键时间
	//编码计时器
	unsigned	long	EncoderTimeCountA;		//引脚A计时时间---计时到EncoderEffectiveTime表示编码有效，过滤干扰使用
	unsigned	long	EncoderTimeCountB;		//引脚B计时时间---计时到EncoderEffectiveTime表示编码有效，过滤干扰使用
	
	unsigned	long	EncoderNoiseTimeA;		//引脚A计时时间---计时到EncoderEffectiveTime表示编码有效，过滤干扰使用
	unsigned	long	EncoderNoiseTimeB;		//引脚B计时时间---计时到EncoderEffectiveTime表示编码有效，过滤干扰使用
}EC11_running_time_def;				//编码器输出值
typedef struct	_EC11
{
	EC11_port_def						port;				//端口
	EC11_conf_time_def			statictime;		//需要配置参数
	EC11_running_time_def		livetime;		//运行参数
	EC11_status_def					status;			//状态：长按
}EC11_def;
//------------------------------------------------------------------


typedef enum	_EC11_Status
{
	EC11_IDLE=	0,						//无触发
	EC11_Button,							//按键
	EC11_ClockWise,						//CW顺时针信号
	EC11_AntiClockWise				//CCW逆时针信号
}EC11_StatusTypeDef;				//编码器输出值


typedef struct	_EC11_Pin			//输入参数按1us时间计算
{
	//需要配置参数：
	//Button
	GPIO_TypeDef* 	EC11_Button_PORT;			//GPIOX
	unsigned short 	EC11_Button_Pin;			//GPIO_Pin_x	
	//KEY-A
	GPIO_TypeDef* 	EC11_A_PORT;					//GPIOX
	unsigned short 	EC11_A_Pin;						//GPIO_Pin_x	
	//KEY-B
	GPIO_TypeDef* 	EC11_B_PORT;					//GPIOX
	unsigned short 	EC11_B_Pin;						//GPIO_Pin_x

	unsigned	char	LongPressFlag;						//长按标志，LongPressFlag=0表示每次只允许一次按压，长按只当一次，LongPressFlag=1；长按达到LongPressStartTime时间后表示长按，后面每计时达到LongPressEffectiveTime后按键次数增加一次
	unsigned 	int		LongPressStartTime;				//长按有效起始时间--LongPressStartTime计时到有效时间表示按键有效，长按3秒生效
	unsigned 	int		LongPressEffectiveTime;		//长按按键单步有效时间--ButtonTimeCount计时到有效时间表示按键有效0.2秒返回一个有效按键
	
	unsigned 	int		ButtonEffectiveTime;		//按键有效时间--ButtonTimeCount计时到有效时间表示按键有效---默认干扰时间10us
	unsigned 	long	EncoderEffectiveTime;		//编码器有效时间---默认干扰时间10us
	
	//不需要配置参数
	unsigned	char	ButtonActiveFlag;			//按键有效标志----针对单次按键,ButtonActiveFlag==0;按键时长未到，按键无效，ButtonActiveFlag==1，按键有效，ButtonTimeCount不再计时，防止溢出
	unsigned	int		ButtonTimeCount;			//按键计时时间---计时到ButtonEffectiveTime表示按键有效，过滤干扰使用
	unsigned	long	EncoderTimeCountA;		//引脚A计时时间---计时到EncoderEffectiveTime表示编码有效，过滤干扰使用
	unsigned	long	EncoderTimeCountB;		//引脚B计时时间---计时到EncoderEffectiveTime表示编码有效，过滤干扰使用
	

}EC11_ConfTypeDef;

//----------------------------------------------------------------API
void 	api_EC11_configuration_gpio(EC11_def* pInfo);
void	api_EC11_run_loop(EC11_def* pInfo);
EC11_status_def	api_EC11_get_status(EC11_def* pInfo);

void EC11_PinConf(EC11_ConfTypeDef *EC11_Conf);
EC11_StatusTypeDef EC11_GetStatus(EC11_ConfTypeDef *EC11_Conf);


#endif

