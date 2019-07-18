#ifndef __IOT5302W_H
#define __IOT5302W_H


#include "stm32f10x_type.h"
#include "STM32_USART.H"      //

//#include "font.h"

//#include	"stdio.h"			//����printf
//#include	"string.h"			//����printf
//#include	"stdarg.h"			//���ڻ�ȡ��ȷ�������Ĳ���
//#include	"stdlib.h"			//malloc��̬�����ڴ�ռ�

/*
M1����������1Kbyte
16������0~15
ÿ������4����0~4
ÿ����16�ֽ�
*/
/********************************************************************************/
/*																���ݰ���ʽ																			*/
/*------------------------------------------------------------------------------*/
/* ��ʼ�ֽ�  �ӻ���ַ  ���ݳ���  ����/״̬  ����n  BCCУ��  ������									*/
/* ��ʼ�� 		��0x0A																															*/
/* �ӻ���ַ 	������ͨѶ��ͨ�õ�ַ0x00																							*/
/* ���ݳ��� 	������+���ݵ��ֽ�������������ʼ�룬BCC��ͽ�����												*/
/* ����/״̬	�������·�����ӻ��ϴ�״̬																					*/
/* BCCУ��	������ʼ��ͽ�����֮��������������У��																*/


#define IotReaderSTX  (unsigned char)0xAA   //Э��ͷ��ʶ
#define IotReaderETX  (unsigned char)0xBB   //Э��β��־
  
#define IOT5302WReadCycle   300     //�����ڣ���λ0.3�룬����ʱʱ��
#define IOT5302WReadTimeOut 2000    //����ʱʱ��		//ԭ1500
#define IOT5302WReadTimeOut_UID 	500    	//δ���յ�UID��ʱ���
#define IOT5302WReadTimeOut_NACK 	3000    //������δ��Ӧ��ʱʱ��3��
#define IOT5302WInitializeTimeOut 2000    //����ʱʱ��2��	//ԭ1500
#define IOT5302WBufferSize  100    //

//--------------------����������
typedef enum  _Cmd //STATUS
{
  //----------------------ISO14443 TYPE A Commands (0x03~0x06)
  REQA          = 0x03,   //ISO14443 -AѰ������
  Anticoll      = 0x04,   //����ͻ
  Select        = 0x05,   //ѡ����
  //----------------------Mifare Application Commands (0x20~0x2F)
  IotRead     	= 0x20, //����Ѱ��������ͻ��ѡ������֤���룬�����Ȳ�����һ��������ɶ���������
  IotWrite    	= 0x21, //����Ѱ��������ͻ��ѡ������֤���룬д���Ȳ�����һ���������д��������.
  InitVal       = 0x22, //����Ѱ��������ͻ��ѡ������֤����Ȳ�����һ��������ɿ�ֵ��ʼ��������
  Decrement     = 0x23, //������Ѱ��������ͻ��ѡ������֤���룬��ֵ��������һ��������ɿ��ֵ������
  Increment     = 0x24, //������Ѱ��������ͻ��ѡ������֤���룬��ֵ�ӵȲ�����һ��������ɿ�ֵ�Ӳ�����
  GET_SNR       = 0x25, //������Ѱ��������ͻ��ѡ���Ȳ�����һ��������ɶ�ȡ��Ƭ���кŵĲ���
  ISO14443CC    = 0x28, //ISO14443 TypeA ͨ��������Ը���ISO14443 TypeA �򿨷��κ�����
  //----------------------ϵͳ����(0x80~0xFF)
  SetAddress    = 0x80, //���ö�д����ַ
  SetBaudrate   = 0x81, //����ͨѶ������
  SetSerlNum    = 0x82, //���ö�д�������к�
  GetSerlNum    = 0x83, //��ȡ��д�������к�
  WriteUserInfo = 0x84, //�����û�������Ϣ
  ReadUserInfo  = 0x85, //��ȡ�û�������Ϣ
  GetVersionNum = 0x86, //������ȡģ��İ汾��Ϣ
  ControlLed    = 0x88, //����led�Ĺ�����ʽ
  ControlBuzzer = 0x89, //����buzzer�Ĺ�����ʽ
}Cmddef;

//--------------------������״̬��
typedef enum  _status //STATUS
{
  CMDOK       = 0x00,          //��ʾ����ִ�гɹ�
  CMDFAILED   = 0x01,   //��ʾ�������ʧ�ܣ�����˵���μ�������
  SETOK       = 0x80,     //��ʾ�������óɹ�
  SETFAILED   = 0x81, //��ʾ��������ʧ��
  CMMTimeOut  = 0x82, //��ʾͨѶ��ʱ.
  NoCard      = 0x83, //��ʾ��������
  DataErr     = 0x84, //��ʾ���տ����ݳ���
  PraErr      = 0x85, //��ʾ��������������������ʽ����
  UnknowErr   = 0x87, //��ʾδ֪�Ĵ���
  UnknowCmd   = 0x8F, //��ʾ�����ָ����벻����
}stsdef;
//--------------------���������ݽṹ��
typedef struct _IOTReader
{
  unsigned char STX;  //0xAA �C ����ʼ�ֽڡ� �C ��׼�����ֽ�. ��ʾһ�����ݰ��Ŀ�ʼ
  unsigned char ID;   //�豸��ַ���ڶ��ͨѶ�����裬��д�����յ����ݰ����жϰ��ڵĵ�ַ������Ԥ���ַ�Ƿ����������Ż���Ӧ����ַ0x00 ��һ���ڵ���ģʽ��ʹ�õ������ַ����д������Ӧ�κδ�0��ַ�����ݰ�(�����е�ַ�ж�).1
  unsigned char Len;  //���ݰ��������ֽڵĳ���.����CMD/STATUS ��DATA field,��������BCC.LENGTH= �ֽ���(CMD/STATUS + DATA[0.. N])
  unsigned char Cts;  //�������״̬CMD/status
  unsigned char Data[128];  //����0�C123������һ���������������йص���������Ҳ�в��������Ҫ��������.
  unsigned char BCC;  //8bits��У���ֽ�.��������STX, ETX�������ֽڵ����У��.
  unsigned char ETX;  //0xBB:����ֹ�ֽڡ� �C ��׼�����ֽڣ���ʾ���ݰ��Ľ���.
}IRDdef;
//--------------------������ʹ��ǰ��Ҫ���õĲ���
typedef struct _IOT5302WConf
{
  RS485Def  IOT5302WPort;         //ͨѶ�ӿ�
  unsigned long USART_BaudRate;   //ʵ��Ӧ����Ҫ�Ĳ�����
}IOT5302WConfWdef;
//--------------------���������ݽṹ��
typedef struct _IOT5302WData
{
  unsigned char   UID[4];         //��ƬUID
  unsigned char   UIDbac[4];      //��ƬUID���ݣ���������б仯������������һ��ʱ���ⷢ
  unsigned char   Data[IOT5302WBufferSize]; //���������ݽ��ջ���
  unsigned char   DataCount;      	//���������ݽ��ռ���
  unsigned char   Initialized;    	//��ʼ����־��0-δ��ʼ���ɹ���1-��ʼ���ɹ�
  unsigned short  Time;           	//��ʱ��
//  unsigned short  TimeOut;        	//��ʱ��ʱ��
	unsigned short  TimeOut_UID;  		//δ���յ�UID��ʱ
	unsigned short  TimeOut_NACK;  		//����������Ӧ��ʱ��ʱ
	unsigned short  Time_Initialized;	//��������ʼ��ʱ����
	unsigned short  TimeGetData;  		//��ѯ����������ʱ������ʱ
  unsigned short  TimeUIDCmp;   		//UID��UIDbac�Ա�������ͬʱ������2�룬���ϴ�����
  unsigned long   USART_BaudRate; 	//����ʹ�õĲ�����
//  unsigned char   BaudRateSetStep; //���ò����ʲ���
}IOT5302WDataWdef;
//--------------------���������ݽṹ��
typedef struct _IOT5302W
{
  IOT5302WConfWdef  Conf;   //������ʹ��ǰ��Ҫ���õĲ���
  IOT5302WDataWdef  Data;  
}IOT5302Wdef;






unsigned short IotMifareGetCmdFrame(Cmddef Cmd,unsigned char Sec,unsigned char* Buffer,unsigned char* KeyBuffer);




void api_iot5302w_configuration(IOT5302Wdef* pIOT5302W);
void api_iot5302w_server(void);

unsigned short api_get_iot5302w_uid(unsigned char* Buffer);		//��ȡUID

//---------------------------------------------------------hardware
static void hw_port_configuration(unsigned long USART_BaudRate);
static unsigned short iot5302w_send_msg(unsigned char* TxdBuffer,unsigned short length);
static unsigned short iot5302w_read_msg(unsigned char* RxdBuffer);

//---------------------------------------------------------software
static void iot5302w_initialize(void);
static void iot5302w_data_receive_process(void);	//���ݽ��մ���������ƴ����Э����
static void iot5302w_data_process(void);		//����Э����
static void iot5302w_get_data(void);
static unsigned short iot5302w_GetFrame_GetSNR(unsigned char* Buffer);    		//���ɻ�ȡUID����Ϣ֡����ȡUID
static unsigned short iot5302w_set_frame_SetBaudrate(unsigned char* Buffer,unsigned char Num);  //�������ö����������ʵ���Ϣ֡
#endif


