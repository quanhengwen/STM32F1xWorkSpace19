#ifndef __HCTYPE_H
#define __HCTYPE_H



typedef enum _ErrCode
{
	GBL_FAULT_NON							=	0x00,	//�޴���
	GBL_FAULT_BUSY						=	0x01,	//æ״̬
	GBL_FAULT_ILLAGALPARA			=	0x02,	//�����Ƿ�
	GBL_FAULT_INACTIVE				=	0x03,	//����ͨ��δ����
	GBL_FAULT_DIRECTION				=	0x04,	//�����������
	GBL_FAULT_DIFFOVF					=	0x05,	//���ز�ֵ����
	GBL_FAULT_SENSOR					=	0x06,	//����������
	GBL_FAULT_ILLIGALWEIGHT		=	0x07,	//������ֵ�Ƿ�
	GBL_FAULT_WEIGHTOVF				=	0x08,	//�����������
	GBL_FAULT_NOCALI					=	0x09	//����δ�궨
}ErrDef;			//������
typedef enum _CmdCode
{
	CMD_WRITE_QTY							=	0x01,	//д������
	CMD_UNLOCK								=	0x02,	//����
	CMD_ADD_MTRL							=	0x04,	//����
	CMD_RDC_MTRL							=	0x06,	//ȡ��
	CMD_CHECK_QTY							=	0x07,	//�̵�
	CMD_WRITE_HINT						=	0x08,	//д����ʾ
	CMD_WRITE_NOTE						=	0x09,	//д��ע��
	CMD_CNTRL_LED							=	0x0A,	//��ʾ�ƿ��� ����ָ���ɵ�Ԫ��ִ��
	CMD_CNTRL_GATE						=	0x0B,	//�����ſ��� ����ָ��������ִ�У������²㴫��
	CMD_CHECK_VERSION					=	0x0C	//��ѯ�����汾
}CmdDef;			//������
typedef enum _FrameType
{
	FRAME_TYPE_DATA						=	0x01,		//֡����--����֡
	FRAME_TYPE_ACK						=	0x02,		//֡����--Ӧ��֡
	FRAME_TYPE_INVALID				=	0x00		//֡����--��Ч֡
}FrameTypeDef;	//֡����
typedef enum _OrderState
{
	ORDER_STAT_IDLE						=	0,			//����״̬
	ORDER_STAT_WORK						=	1				//����״̬--æ״̬
}OrderStateDef;	//����״̬


typedef struct	_RS232Fram
{
	unsigned short 	FarmLen;
	unsigned char		Data[];
}RS232FramDef;		//RS232��Ϣ֡��ʽ(PC��ͨѶ)
typedef struct	_RS485Fram
{
	unsigned short 	FarmLen;
	unsigned char		Data[];
}RS485FramDef;		//RS485��Ϣ֡��ʽ()

#endif
