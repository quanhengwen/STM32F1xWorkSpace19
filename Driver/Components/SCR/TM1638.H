#ifndef __TM1638_H
#define __TM1638_H
#include "stm32f10x_gpio.h"



void TM1638_Write(unsigned char	DATA);			//д���ݺ���
unsigned char TM1638_Read(void);						//�����ݺ���
void Write_COM(unsigned char cmd);					//����������
unsigned char Read_key(void);
void Write_DATA(unsigned char add,unsigned char DATA);					//ָ����ַд��һ�ֽ�����
void Write_DataFX(unsigned char add,unsigned char DATA);				//�̶���ַ��ʽ
void Write_DataAI(unsigned char add,unsigned char DATA);				//��ַ����
void Write_allLED(unsigned char LED_flag);											//����ȫ��LED������LED_flag��ʾ����LED״̬



void Write_DataNum(unsigned char Startadd,unsigned long Num);		//��ַ������ʽд��8λ����
void init_TM1638(void);


#endif
