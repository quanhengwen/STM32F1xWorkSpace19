#ifndef __LOCK_H
#define __LOCK_H

#include "stdint.h"

#include "stm32f10x_gpio.h"


#include "STM32_GPIO.H"


#define	DefaultLockOnTime	50	//Ĭ��������ʱ��������

typedef	struct	_LockSe				//�����������Ʊ�־������ʱ��
{
	struct
	{
		unsigned char LockFlg		:1;	//bit0��0-�޲���,	1-�в�������(��������״̬/��Ҫ�ϱ�״̬��
		unsigned char LockOn		:1;	//bit1��0-��״̬��	1-��������
		unsigned char LockOff		:1;	//bit2��0-��״̬��	1-���Ѵ�
		unsigned char LockSts		:1;	//bit3��0-���ͷţ�	1-�ѱպ�
		unsigned char LockTout	:1;	//bit4��0-δ��ʱ��	1-�ѳ�ʱ(���򿪳�ʱ������״̬�����쳣)
	}sLockFlg;		//ҩ��״̬
	unsigned char	SeTimeCountUp;				//�������źţ����з���ʱ�������ӵ�һ��ֵ��ʾ������Ч�����˲�Ч��
	unsigned long	LockTimeCountdown;		//��������ʱ ��ʱ��0ʱ�ͷ��������LockTimeOut��Ϊ0������
	//-------------------------������
	GPIO_TypeDef*	DrGPIOx;
	u16	DrGPIO_Pin_n;
	//-------------------------������
	GPIO_TypeDef*	SeGPIOx;
	u16	SeGPIO_Pin_n;
}sLockSeDef;



void LockInitialize(sLockSeDef *LockSe);		//������--����
void LockServer(sLockSeDef *LockSe);
void LockSetOn(sLockSeDef *LockSe,unsigned long LockOnTime);		//����
void LockSetOff(sLockSeDef *LockSe);		//�ͷ���
unsigned char LockGetStatus(sLockSeDef *LockSe);
    
#endif
