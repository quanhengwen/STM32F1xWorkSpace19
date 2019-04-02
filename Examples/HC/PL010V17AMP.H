#ifndef __PL010V17AMP_H
#define __PL010V17AMP_H



//typedef struct _PL010V15
//{

//}PL010V15Def;


void PL010V17AMP_Configuration(void);
void PL010V17AMP_Server(void);


void LCD_Server(void);	//��ʾ�������
void RS485_Server(void);		//ͨѶ����---������Ϣ�Ľ����뷢��
void SwitchID_Server(void);	//���뿪�ش���--��̬���²����ַ

void SwitchID_Configuration(void);

void LCD_Configuration(void);

void RS485_Configuration(void);
void USART_Configuration(void);

void LCD_PowerUp(void);
void LCD_Display(void);

void PL010Delay(unsigned long time);


#endif
