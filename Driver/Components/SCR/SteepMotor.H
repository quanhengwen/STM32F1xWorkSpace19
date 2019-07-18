
#ifndef __SteepMotor_H
#define __SteepMotor_H

#include "stm32f10x_gpio.h"
#include "stm32f10x_type.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_nvic.h"

typedef struct
{
	//====================���ò�����ʹ��ʱ��Ҫ����
	TIM_TypeDef*	SetTIMx;				//ʹ�õ��Ķ�ʱ��
	//--------------------�������
	GPIO_TypeDef*	SetPulsPort;		//GPIOx:x=A/B/C/D/E/F/G
	u16 SetPulsPin;									//GPIO_Pin_n:n=0~15;
	//--------------------�������
	GPIO_TypeDef*	SetDIRPort;			//GPIOx:x=A/B/C/D/E/F/G
	u16 SetDIRPin;								//GPIO_Pin_n:n=0~15;
	//--------------------���Ƶ��
	u16	SetFrequency;				//Ƶ�� ��СƵ��1Hz
	u16	SetPlusUp;					//����Ƶ�ʼ��	
	u16	SetPlusUpNum;				//�����������
	
	u16	SetPlusDown;				//����Ƶ�ʼ��
	u16	SetPlusDownNum;			//�����������
	u32	SetPulsTotal;				//��Ҫ�����������
	//====================���б�־��
	u8	RunFlag;						//0:δ���У�1-��ʱ������
	u8	PulsFlag;						//һ�����������غ��½���Ҫ������ʱ���ж�
	//====================���в��������б���
	u16	GetFrequency;				//Ƶ�� ��СƵ��1Hz
	u16	GetPlusUpNum;				//�����������
	u32	GetPulsTotal;				//�Ѿ�����������
}SteepMotorDef;



void StepMotorConfiguration(SteepMotorDef *STEP_MOTOx);		//
unsigned char StepMotorServer(SteepMotorDef *STEP_MOTOx);	//


void StepMotorCW(SteepMotorDef *STEP_MOTOx,u16	SetFrequency,u16 SetPlusUp,u16 SetPlusUpNum,u16	SetPlusDown,u16	SetPlusDownNum,u32 SetPulsTotal);		//˳ʱ����ת
void StepMotorCCW(SteepMotorDef *STEP_MOTOx,u16	SetFrequency,u16 SetPlusUp,u16 SetPlusUpNum,u16	SetPlusDown,u16	SetPlusDownNum,u32 SetPulsTotal);		//��ʱ����ת
void StepMotorStop(SteepMotorDef *STEP_MOTOx);



#endif

