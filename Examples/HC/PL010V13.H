#ifndef __PL010V13_H
#define __PL010V13_H



void PL010V13_Configuration(void);
void PL010V13_Server(void);
void PL010V13_PinSet(void);
void PL010Delay(unsigned long time);
void LCD_WS(void);		//λ��˸
void LCD_WXS(void);		//λ��ʾ
void LCD_DDSP(void);		//��ʾ�ܹ������������ѷ�����
#endif
