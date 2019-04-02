#ifndef __PL009V31HC_H
#define __PL009V31HC_H



typedef struct _LcdDriver
{
    void ( *Init )( void );
    void ( *Reset )( void );
    void ( *StartRx )( void );
    void ( *GetRxPacket )( void *buffer, char *size );
    void ( *SetTxPacket )( const void *buffer, char size );
    unsigned long ( *Process )( void );
}LcdDriverDef;


void PL009V31HC_Configuration(void);
void PL009V31HC_Server(void);


void CS5530_Server(void);		//���ط���ADֵ��������ȡ�ȶ�ֵ
void LCD_Server(void);	//��ʾ�������
void RS485_Server(void);		//ͨѶ����---������Ϣ�Ľ����뷢��
void SwitchID_Server(void);	//���뿪�ش���--��̬���²����ַ

void SwitchID_Configuration(void);
void GT32L32_Configuration(void);
void LCD_Configuration(void);
void CS5530_Configuration(void);
void RS485_Configuration(void);


void LCD_PowerUp(void);
void LCD_Display(void);

void PL010Delay(unsigned long time);


#endif
