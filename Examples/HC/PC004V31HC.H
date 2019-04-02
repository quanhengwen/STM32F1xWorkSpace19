#ifndef __PC004V31HC_H
#define __PC004V31HC_H

#include "stm32f10x_type.h"
#include "HC_PHY.H"

//#define	RS485MaxDataSize	64			//RS485����������ݳ���
//#define	Rs232MaxDataSize	64			//RS232������ݳ���

//=============================RS485A���߶˿�(���ϲ�/��Ԫ��ͨѶ�ӿ�)

	#define	RS485BusDataSize		64			//RS485����������ݳ���
	#define	RS485BusBaudRate		19200
	#define	RS485BusSerialPort	USART2
	#define	RS485BusCtlPort			GPIOA
	#define	RS485BusCtlPin			GPIO_Pin_1
	//=============================RS485B���߶˿�(���²�/���ͨѶ�ӿ�)
	#define	RS485layDataSize		64			//RS485����������ݳ���
	#define	RS485layBaudRate		19200
	#define	RS485laySerialPort	UART4
	#define	RS485layCtlPort			GPIOC
	#define	RS485layCtlPin			GPIO_Pin_12
	//=============================RS232A�˿�
	#define	RS232ADataSize			64			//RS485����������ݳ���
	#define	RS232ABaudRate			19200
	#define	RS232ASerialPort		USART1

	//=============================RS232A�˿�
	#define	RS232BDataSize			64			//RS485����������ݳ���
	#define	RS232BBaudRate			19200
	#define	RS232BSerialPort		USART3

	//=============================CAN.
	#define	CANBaudRate					100000		//100K
	
	//=============================���뿪�ض˿�
	#define NumOfSwitch			8			//����λ��
	#define	GPIOxSW1		GPIOC
	#define	PinxSW1			GPIO_Pin_0

	#define	GPIOxSW2		GPIOC
	#define	PinxSW2			GPIO_Pin_1

	#define	GPIOxSW3		GPIOC
	#define	PinxSW3			GPIO_Pin_2

	#define	GPIOxSW4		GPIOC
	#define	PinxSW4			GPIO_Pin_3

	#define	GPIOxSW5		GPIOC
	#define	PinxSW5			GPIO_Pin_4

	#define	GPIOxSW6		GPIOC
	#define	PinxSW6			GPIO_Pin_5

	#define	GPIOxSW7		GPIOC
	#define	PinxSW7			GPIO_Pin_6

	#define	GPIOxSW8		GPIOC
	#define	PinxSW8			GPIO_Pin_7
	
	//=============================�������źż��
	#define	Lock1DrPort		GPIOB					//�����ƶ˿�
	#define	Lock1DrPin		GPIO_Pin_13		//�����ƹܽ�
	#define	Lock1SiPort		GPIOB					//�����ƶ˿�
	#define	Lock1SiPin		GPIO_Pin_14		//�����ƹܽ�

	#define	Lock2DrPort		GPIOB					//�����ƶ˿�
	#define	Lock2DrPin		GPIO_Pin_0		//�����ƹܽ�
	#define	Lock2SiPort		GPIOB					//�����ƶ˿�
	#define	Lock2SiPin		GPIO_Pin_1		//�����ƹܽ�

	#define	Lock3DrPort		GPIOA					//�����ƶ˿�
	#define	Lock3DrPin		GPIO_Pin_6		//�����ƹܽ�
	#define	Lock3SiPort		GPIOA					//�����ƶ˿�
	#define	Lock3SiPin		GPIO_Pin_7		//�����ƹܽ�

	#define	Lock4DrPort		GPIOA					//�����ƶ˿�
	#define	Lock4DrPin		GPIO_Pin_4		//�����ƹܽ�
	#define	Lock4SiPort		GPIOA					//�����ƶ˿�
	#define	Lock4SiPin		GPIO_Pin_5		//�����ƹܽ�



void PC004V31HC_Configuration(void);
void PC004V31HC_Server(void);

void AMPTEST(void);


void Communiction_Configuration(void);			//ͨѶ�ӿ����ã�����RS485,RS232,CAN
void Switch_Configuration(void);						//���뿪������
void Lock_Configuration(void);							//���˿�����



#endif
