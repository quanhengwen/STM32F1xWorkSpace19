#ifndef __PL010V15_H
#define __PL010V15_H



typedef enum
{
	Cmd0	=	0x01,
	Cmd1,
	Cmd2
}TcmdDef;



void PL010V15TEST_Configuration(void);
void PL010V15TEST_Server(void);


void CS5530_Server(void);		//���ط���ADֵ��������ȡ�ȶ�ֵ
void R61509V_Server(void);	//��ʾ�������
void RS485_Server(void);		//ͨѶ����---������Ϣ�Ľ����뷢��
void SwitchID_Server(void);	//���뿪�ش���--��̬���²����ַ

void SwitchID_Configuration(void);
void GT32L32_Configuration(void);
void R61509V_Configuration(void);
void CS5530_Configuration(void);
void RS485_Configuration(void);


void LCD_PowerUp(void);
void LCD_Display(void);

unsigned int PL010V13_PrintfString(unsigned short x,unsigned short y,unsigned char font,const char *format,...);				//��ߵ�ʡ�Ժž��ǿɱ����
void PL010Delay(unsigned long time);


#endif
