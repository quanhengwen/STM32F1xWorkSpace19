#ifndef __PD002V30ADC_H
#define __PD002V30ADC_H


#include "STM32_USART.H"
#include "AT24C02.H"
#include "CS5530.H"
#include "74HC595.H"
#include "SWITCHID.H"



enum
{
	head1	=	0xFA,
	head2	=	0xF5
};
typedef	enum _PD002V30Cmd
{
	APP_CMD_Null						=	0x00,	/*������*/
	APP_CMD_CTFAYAO					=	0x21,	/*���س��뷢ҩ*/
	APP_CMD_CTPANDIAN				=	0x22,	/*���س����̵�*/
	APP_CMD_CTJIAYAO				=	0x23,	/*���س����ҩ*/
	APP_CMD_CTCLEAR					=	0x25,	/*���س�������*/
	APP_CMD_CTBIAODINGS1		=	0x26,	/*���س���궨����1:����һ��������ҩƷ*/
	APP_CMD_CTBIAODINGS2		=	0x27,	/*���س���궨����2:��������--�궨*/
	APP_CMD_CTHUISHOUFLICK	=	0x2A,	/*�������ճ���*/
	APP_CMD_CTFORCEPOPUP		=	0x2B,	/*���س���ǿ�Ƶ���*/
	APP_CMD_CTHUISHOUCOUNT	=	0x2C,	/*���տ�ҩƿ����*/
	APP_CMD_CTHUISHOUCLEAR	=	0x2D	/*���տ�ҩƿ���*/
}PD002V30CmdDef;
typedef enum _WorkStatebac 
{
	SYS_IDLE=0,									/*����״̬*/
	SYS_ORIGIN_TRACK_REQ,				/*�������ԭ��*/
	SYS_ORIGIN_TRACK_WAIT,			/*�ȴ�����ԭ��*/
	SYS_ORIGIN_TRACK_EXE,				/*ִ�и���ԭ��*/
	SYS_ORIGIN_TRACK_OPEN_SUS,	/*�����,����ԭ�����*/
	SYS_ORIGIN_TRACK_OPEN_REQ,	/*���󿪳������һ��ԭ��*/
	SYS_ORIGIN_TRACK_OPEN_EXE,	/*ִ�п��������һ��ԭ��*/
	SYS_GET_QUANTITY_REQ,				/*�õ���������*/
	SYS_GET_QUANTITY_OTINQ,			/*�õ������Ƿ�ʱ��ѯ*/
	SYS_GET_QUANTITY_EXE_OT,		/*�õ�����+��ѯ�Ƿ�ʱ*/
	SYS_GET_QUANTITY_EXE,				/*�õ�����*/
	SYS_DEMAR_REQ,							/*�궨����*/
	SYS_DEMAR_OTINQ,						/*�궨ִ���Ƿ�ʱ��ѯ*/
	SYS_DEMAR_EXE_OT,						/*�궨ִ��+��ѯ�Ƿ�ʱ*/
	SYS_DEMAR_EXE,							/*�궨ִ��*/
	SYS_SENSOR_ERROR_SUS				/*�������������ڴ���״̬*/
} WorkStatebacDef;
typedef enum _WorkRequst
{
	RequstIdle=0,					/*����������*/
	RequstBackupData,			/*���󱸷ݲ���ǰADֵ*/
	RequstNewData,				/*���󱸷ݲ�����ADֵ*/
	RequstQuantity,				/*�����ȡ����*/	
	RequstPiece,					/*�����ȡ����ADֵ*/
} WorkRequstDef;
typedef enum _WorkState 
{
	StateIdle=0,								/*����״̬*/
	StateSuspend,								/*��ִ�в�������*/
	StateGetFiltDataReq,				/*�����ȡ�˲�ADֵ*/
	StateGetFiltDataWait,				/*�ȴ���ȡ�˲�ADֵ*/
	StateGetFiltDataExe,				/*ִ�л�ȡ�˲�ADֵ*/	
	
	StateProcessDataExe,				/*ִ�м���*/
	
	StateReadI2CDataReq,				/*�����ȡ����EEPROM��ֵ*/
	StateSaveI2CDataReq,				/*���󱣴汸��EEPROM��ֵ*/
	StateClearI2CDataReq,				/*�����������EEPROM��ֵ*/
	
//	StateGetNewDataReq,					/*�����ȡ������ADֵ*/
//	StateGetNewDataWait,				/*�ȴ���ȡ������ADֵ*/
//	StateGetNewDataExe,					/*ִ�л�ȡ������ADֵ*/
//	
//	StateGetNewNumReq,					/*�����ȡ����*/
//	StateGetNewNumWait,					/*�ȴ���ȡ����**/
//	StateGetNewNumExe,					/*ִ�л�ȡ����**/	
//	
//	StateGetPieceDataReq,				/*�����ȡ����ADֵ*/
	
} WorkStateDef;
typedef	struct
{
	unsigned short RunningTime;		//��ʱ��
	unsigned char	 StartAddr;			//���ݴ洢��ʼ��ַ
	struct
	{
		unsigned short	Quantity;			//����
		unsigned short	PieceWeight;	//����
	}CH1;	//ͨ��1
	struct
	{
		unsigned short	Quantity;			//����
		unsigned short	PieceWeight;	//����
	}CH2;	//ͨ��2
	
	sI2CDef 			Port;
}AT24C02Def;
typedef	struct	_MS200Pro
{
	struct
	{
		unsigned char h1;
		unsigned char h2;
	}Head;
	unsigned char		SerialNumber;	//��ˮ��
	unsigned char		Receipt;			//��ִλ
	unsigned char		Number;				//���
	unsigned char		Address;			//λ��	//ȡ��4λ
	PD002V30CmdDef	Cmd;					//����
	unsigned char		Data;					//����
	unsigned char		Reserve;			//����λ
	unsigned char		State;				//����״̬λ
	unsigned char		ErrFlag;			//������
	unsigned char		Bcc8;					//���У��(��ˮ�ŵ������룩
}MS200ProCCDef;
typedef	struct	_PD002V30RS485Pro
{
	struct
	{
		unsigned char Time;
		unsigned char Retry;
	}Data;
	MS200ProCCDef	Pro;
}RS485ProCCDef;

typedef	struct _PD002V30Data
{
	unsigned short	Time;					//��ʱ��
	unsigned short	Retry;				//�������Լ���
	unsigned short	Quantity;			//����
	unsigned short	WeightPiece;	//����
	unsigned long		WeightBackUp;	//����ǰ����ADֵ
	unsigned long		WeightNew;		//��������ADֵ
}PD002V30DataDef;
typedef	struct _PD002V30CH
{
	PD002V30DataDef	Data;					//����
	WorkStateDef		WorkState;		//����״̬
	WorkRequstDef		WorkRequst;		//��������
	CS5530Def 			ADC;					//ADC���������ӿڣ�����
	RS485ProCCDef		RS485Rx;			//485���ջ���
	RS485ProCCDef		RS485Bc;			//485���ձ��ݻ���
	RS485ProCCDef		RS485Tx;			//485���ͻ���
}PD002V30CHDef;
typedef	struct _PD002V30
{
	PD002V30CHDef	CH1SS3;			//ͨ��1��SS3�ӿڣ�����
	PD002V30CHDef	CH2SS4;			//ͨ��2��SS4�ӿڣ�����
	MS200ProCCDef	Ack;
	AT24C02Def		AT24C02;		//EEPROM
	RS485Def 			BUS485;			//485
	SwitchDef			SWITCHID;
}PD002V30Def;


















void PD002V30ADC_Configuration(void);

void PD002V30ADC_Server(void);

void PD002V30_USART_Cofiguration(void);
	
void SwitchID_Configuration(void);

unsigned char PD002V30_GetSwitchID(void);				//��ȡ���뿪�ص�ַ


void CS5530_Configuration(void);		//CS5530��ʼ��
void CS5530_Server(void);				//��ȡADֵ


#endif
