#ifndef __EC11Encoder_H
#define __EC11Encoder_H
#include "stm32f10x_gpio.h"

typedef union	_EC11_status
{
	unsigned char u8s;
	struct
	{
		unsigned char trigger_key 		:1;	//bit0//����������־
		unsigned char trigger_right 	:1;	//bit1//A������־--CW˳ʱ���ź�-��ת
		unsigned char trigger_left 		:1;	//bit2//B������־--CCW��ʱ���ź�-��ת
		unsigned char trigger_long 		:1;	//bit3//������־--������⵽��һ��ʱ��
		unsigned char trigger_ecoder 	:1;	//bit4//�������Ѽ���־
	}u8b;
}EC11_status_def;				//���������ֵ
typedef struct	_EC11_port
{
	//��Ҫ���ò�����
	//Button
	GPIO_TypeDef* 	button_port;			//GPIOX
	unsigned short 	button_pin;			//GPIO_Pin_x	
	//KEY-A
	GPIO_TypeDef* 	A_port;					//GPIOX
	unsigned short 	A_pin;						//GPIO_Pin_x	
	//KEY-B
	GPIO_TypeDef* 	B_port;					//GPIOX
	unsigned short 	B_pin;						//GPIO_Pin_x
}EC11_port_def;				//���������ֵ
typedef struct	_EC11_conf_time
{
	//��Ҫ���ò�����
	unsigned 	char	ScanTime;									//ɨ��ʱ�䣺��λus
	unsigned	char	LongPressFlag;						//������־��LongPressFlag=0��ʾÿ��ֻ����һ�ΰ�ѹ������ֻ��һ�Σ�LongPressFlag=1�������ﵽLongPressStartTimeʱ����ʾ����������ÿ��ʱ�ﵽLongPressEffectiveTime�󰴼���������һ��
	unsigned 	long	LongPressStartTime;				//������Ч��ʼʱ��--LongPressStartTime��ʱ����Чʱ���ʾ������Ч������3����Ч
	unsigned 	long	LongPressEffectiveTime;		//��������������Чʱ��--ButtonTimeCount��ʱ����Чʱ���ʾ������Ч0.2�뷵��һ����Ч����
	
	unsigned 	long	ButtonEffectiveTime;			//������Чʱ��--ButtonTimeCount��ʱ����Чʱ���ʾ������Ч---Ĭ�ϸ���ʱ��10us
	unsigned 	long	EncoderEffectiveTime;			//��������Чʱ��---Ĭ�ϸ���ʱ��10us
}EC11_conf_time_def;				//���������ֵ
typedef struct	_EC11_running_time
{
	//����Ҫ���ò�����
	//������ʱ��
	unsigned 	long	PressTime;		//����ʱ��
	unsigned 	long	PressNoiseTime;		//����ʱ��
	//�����ʱ��
	unsigned	long	EncoderTimeCountA;		//����A��ʱʱ��---��ʱ��EncoderEffectiveTime��ʾ������Ч�����˸���ʹ��
	unsigned	long	EncoderTimeCountB;		//����B��ʱʱ��---��ʱ��EncoderEffectiveTime��ʾ������Ч�����˸���ʹ��
	
	unsigned	long	EncoderNoiseTimeA;		//����A��ʱʱ��---��ʱ��EncoderEffectiveTime��ʾ������Ч�����˸���ʹ��
	unsigned	long	EncoderNoiseTimeB;		//����B��ʱʱ��---��ʱ��EncoderEffectiveTime��ʾ������Ч�����˸���ʹ��
}EC11_running_time_def;				//���������ֵ
typedef struct	_EC11
{
	EC11_port_def						port;				//�˿�
	EC11_conf_time_def			statictime;		//��Ҫ���ò���
	EC11_running_time_def		livetime;		//���в���
	EC11_status_def					status;			//״̬������
}EC11_def;
//------------------------------------------------------------------


typedef enum	_EC11_Status
{
	EC11_IDLE=	0,						//�޴���
	EC11_Button,							//����
	EC11_ClockWise,						//CW˳ʱ���ź�
	EC11_AntiClockWise				//CCW��ʱ���ź�
}EC11_StatusTypeDef;				//���������ֵ


typedef struct	_EC11_Pin			//���������1usʱ�����
{
	//��Ҫ���ò�����
	//Button
	GPIO_TypeDef* 	EC11_Button_PORT;			//GPIOX
	unsigned short 	EC11_Button_Pin;			//GPIO_Pin_x	
	//KEY-A
	GPIO_TypeDef* 	EC11_A_PORT;					//GPIOX
	unsigned short 	EC11_A_Pin;						//GPIO_Pin_x	
	//KEY-B
	GPIO_TypeDef* 	EC11_B_PORT;					//GPIOX
	unsigned short 	EC11_B_Pin;						//GPIO_Pin_x

	unsigned	char	LongPressFlag;						//������־��LongPressFlag=0��ʾÿ��ֻ����һ�ΰ�ѹ������ֻ��һ�Σ�LongPressFlag=1�������ﵽLongPressStartTimeʱ����ʾ����������ÿ��ʱ�ﵽLongPressEffectiveTime�󰴼���������һ��
	unsigned 	int		LongPressStartTime;				//������Ч��ʼʱ��--LongPressStartTime��ʱ����Чʱ���ʾ������Ч������3����Ч
	unsigned 	int		LongPressEffectiveTime;		//��������������Чʱ��--ButtonTimeCount��ʱ����Чʱ���ʾ������Ч0.2�뷵��һ����Ч����
	
	unsigned 	int		ButtonEffectiveTime;		//������Чʱ��--ButtonTimeCount��ʱ����Чʱ���ʾ������Ч---Ĭ�ϸ���ʱ��10us
	unsigned 	long	EncoderEffectiveTime;		//��������Чʱ��---Ĭ�ϸ���ʱ��10us
	
	//����Ҫ���ò���
	unsigned	char	ButtonActiveFlag;			//������Ч��־----��Ե��ΰ���,ButtonActiveFlag==0;����ʱ��δ����������Ч��ButtonActiveFlag==1��������Ч��ButtonTimeCount���ټ�ʱ����ֹ���
	unsigned	int		ButtonTimeCount;			//������ʱʱ��---��ʱ��ButtonEffectiveTime��ʾ������Ч�����˸���ʹ��
	unsigned	long	EncoderTimeCountA;		//����A��ʱʱ��---��ʱ��EncoderEffectiveTime��ʾ������Ч�����˸���ʹ��
	unsigned	long	EncoderTimeCountB;		//����B��ʱʱ��---��ʱ��EncoderEffectiveTime��ʾ������Ч�����˸���ʹ��
	

}EC11_ConfTypeDef;

//----------------------------------------------------------------API
void 	api_EC11_configuration_gpio(EC11_def* pInfo);
void	api_EC11_run_loop(EC11_def* pInfo);
EC11_status_def	api_EC11_get_status(EC11_def* pInfo);

void EC11_PinConf(EC11_ConfTypeDef *EC11_Conf);
EC11_StatusTypeDef EC11_GetStatus(EC11_ConfTypeDef *EC11_Conf);


#endif
