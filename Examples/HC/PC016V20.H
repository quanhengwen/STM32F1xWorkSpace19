#ifndef __PC016V20_H
#define __PC016V20_H


#include "stm32f10x_exti.h"

#include "CardReader.H"			//������

//#include "STM32_TOOLS.H"		//���У�鹤��
#include "SWITCHID.H"
#include "STM32_CRC.H"
#include "STM32_USART.H"
#include "STM32_SPI.H"
#include "STM32_PWM.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_WDG.H"
#include "STM32_SYSTICK.H"
#include "STM32F10x_BitBand.H"


#include	"stdio.h"			//����printf
#include	"string.h"		//����printf
#include	"stdarg.h"		//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"		//malloc��̬�����ڴ�ռ�

#define	MinPortNum		1			//��С/��ʼ�˿ں� 1~4
#define	MaxPortNum		4			//���˿ں� 1~4
#define	MaxSysTime		1000	//TimeSYS����ʱʱ�䣬��λms����������ʱʱ������м�ʱ������

//==============�˿�---������
#define	ICCardReadPort1					USART3		//J1
#define	ICCardReadPort2					USART1		//J2
#define	ICCardReadPort3					UART4			//J3
#define	ICCardReadPort4					UART5			//J4---��֧��DMA����
#define	ICCardReadBaudRate			19200
#define	CardDataSize						50				//���յ�IC�������������ݴ�С��Э��+���ݹ�22byte������10byte
#define	ICDataStartByte					2					//16�ֽ���������Ч������ʼλ 0~15
#define	ICDataNum								3					//16�ֽ���������Ч�����ֽ���

#define	CardStartByte						0					//16�ֽ���������Ч������ʼλ 0~15
#define	CardLength							4					//16�ֽ���������Ч�����ֽ���

//==============���ö˿�

//==============�˿�---����485
#define	Bus485Port				USART2
#define	Bus485CtlPort			GPIOA
#define	Bus485CtlPin			GPIO_Pin_1
#define	Bus485BaudRate		19200											//������
#define	BusDataSize				64												//һ����ϢЯ����������ݸ���
#define	Bus485BufferSize	80												//һ����Ϣ����󻺴��С��Э������+���ݣ�
#define	CommProtocolLenth		11											//��׼Э�鲿�����ݳ���//sizeof(sCommHeadDef)+sizeof(sCommStartDef)+sizeof(sCommAddrDef)+sizeof(eCommErrorCodeDef)
#define	CommBasicLenth			4												//���ݳ���nLength�е�ַ��״̬��ĳ���
#define	FixedAddrLenth			4												//�̶����ݳ���nLength�е�ַ��״̬��ĳ���
#define	FixedDataLenth			3												//�̶����ݳ���-----��������

//==============�˿�---�����485
#define	Seg485PortRmap		USART1		//�˿��ض���
#define	Seg485CtlPort			GPIOB
#define	Seg485CtlPin			GPIO_Pin_5
#define	Seg485BaudRate		19200
#define	Seg485BufferSize	22
//==============�˿�---��
#define	LockPort(n)		LockPort##n
#define	LockPin(n)		LockPin##n
#define	LockOn(n)			GPIO_SetBits(LockPort(n),	LockPin(n))			//����
#define	LockOff(n)		GPIO_ResetBits(LockPort(n),	LockPin(n))		//�ͷ���

#define	LockPort1			GPIOC
#define	LockPin1			GPIO_Pin_3
#define	LockPort2			GPIOB
#define	LockPin2			GPIO_Pin_9
#define	LockPort3			GPIOB
#define	LockPin3			GPIO_Pin_3
#define	LockPort4			GPIOB
#define	LockPin4			GPIO_Pin_4


#define	DefaultOnTime	20000				//Ĭ��������ʱ��20S
#define	MaxOnTime			120000			//���������ʱ��120S	2����
#define	MinOnTime			20000				//��������Сʱ��10S	
#define	LockOffTime		5000				//ҩ��ȡ�ߺ����ͷ�ʱ��5S
#define	SegOnTime			60000				//����ҩ�������ܳ���ʱ��60S

#define	PowerT				6000				//�ϵ�ʱ��	

#define	Pt(n)	GPIO_SetBits(LockPort(n),	LockPin(n))

//u8 ICCARD_CMD_GetDataBlock1[] =
//{
//	0xE1,   /* F-Head 	*/			//Ϊ֡ͷ����λ���·�ʱ�̶�Ϊ0XE1����λ��Ӧ��ʱ�̶�Ϊ0XD2��
//	0x24,  	/* CmdType 	*/			//��������
//	0x01,		/* Sector 	*/			//1�ֽ�������
//	0x01,   /* Block 		*/			//1�ֽڿ��
//	0x60,   /* KeyType	*/			//1�ֽ���Կģʽ KEYA��0x60��/KEYB(0x61)
//	0xFF,   /* PassWord0 */			//6�ֽ�����
//	0xFF,   /* PassWord1 */
//	0xFF,   /* PassWord2 */
//	0xFF,		/* PassWord3 */
//	0xFF,   /* PassWord4 */
//	0xFF,		/* PassWord5 */
//	0x1C,   /* Crc16-HIGN */
//	0x0D,		/* Crc16-LOW */
//	0x1E   	/* F-End*/						//Ϊ֡β����λ���·�ʱ�̶�Ϊ0X1E����λ��Ӧ��ʱ�̶�Ϊ0X2D��
//};
//============================================================================================ͨѶЭ��Start
//***************485����ͨѶ��������
//-----------------------------------------------------------------------------------ö����
typedef enum	_ePro485Us			//485Э��ͷβ��ʶ��
{
	ePro485Head			=	0x7E,			//ͷ��ʶ
	ePro485End			=	0x7F,			//������ʶ
}ePro485UsDef;
//typedef enum	_ePro485Cmd			//������(���������������������
//{
//	eGetID	=	0x92,							//��λ��������ȡID����Ӧ������
//	eSendID	=	0x93,							//��λ�������ϴ�ID������
//	eLock		=	0x93,							//�����������ϱ�
//	eGetBox	=	0x93,							//ȡҩ�����ϱ�
//}ePro485CmdDef;
//typedef enum	_ePro485Ack			//������(���������������������
//{
//	eGetID	=	0x92,							//��λ��������ȡID����Ӧ������
//	eSendID	=	0x93,							//��λ�������ϴ�ID������
//	eLock		=	0x93,							//�����������ϱ�
//	eGetBox	=	0x93,							//ȡҩ�����ϱ�
//}ePro485AckDef;
typedef enum	_ePro485Com			//������(���������������������
{
		//===============��λ���·�����
		CGetID	=	0x12,							//��λ��������ȡID����Ӧ������
		CGetBox	=	0x13,							//ȡҩ�����ϱ�
		
		//===============��λ���ϱ�����
		AGetID	=	0x92,							//��λ��������ȡID����Ӧ������
		ASendID	=	0x93,							//��λ�������ϴ�ID������
		ALock		=	0x93,							//�����������ϱ�
		AGetBox	=	0x93,							//ȡҩ�����ϱ�

}ePro485ComDef;
typedef enum	_ePro485Sts			//״̬��
{
	eNoErr		=	0x00,		//�޴���
	eNoData		=	0x23,		//��ҩ������
	eBoxTout	=	0x24,		//ȡҩ�䳬ʱ
	eBoxOff		=	0x25,		//ҩ��ȡ��
	eBoxBack	=	0x26,		//ҩ��ȡ�ߺ��ջ�
	eReadErr	=	0x27,		//������ͨѶ�쳣
	eBoxTake	=	0x25,		//ҩ�䱻ǿ��ȡ��,ԭ28
}ePro485StsDef;
//-----------------------------------------------------------------------------------�ṹ��
typedef	struct	_sPro485Addr			//ͨѶ��ַ�ṹ�壨����ʱ��Addr1,Addr2,Addr3���������ݣ�����ʱ�����ַΪ0x00��
{
	u8 	Addr1;		//���ַ(��Ԫ���)  ����ʱΪ���ַ������ʱ�����ַΪ0x00��
	u8 	Addr2;		//���ַ
	u8	Addr3;		//�۵�ַ���˿ںţ�	
}sPro485AddrDef;
typedef	struct	_sMs485Farme			//485ͨѶ��Ϣ֡�ṹ��MessageFarme  
{
	ePro485UsDef		Head;				//b0ʶ���	0x7E
	u8							dAdd;				//b1Ŀ���ַ����ʱ����Ϊ��Ԫ���ַ���·�Ϊ���յ�ַ���ϴ�Ϊ0x00
	u8							sAdd;				//b2Դ��ַ �·�Ϊ0x00���ϴ�Ϊ����ַ
	u8							nSerial;		//b3��ˮ�� 0x01~0xFF ÿ��ʹ�ú��1
	ePro485ComDef		Cmd;				//b4����ţ�0x12-��ȡָ��IC���ţ�0x13ȡҩ,�ϱ�����Ҳ�ڴ�λ
	u8							Ucode;			//b5�û��룺��������ԭ������
	u8							Length;			//b6��ַ+�쳣��̶�����Ϊ4�������ݳ���Ϊ3	
	sPro485AddrDef	Addr;				//ͨѶ��ַ�ṹ��	
	ePro485StsDef		StsCode;		//״̬��/������
	
	u8							data[BusDataSize];		//BusDataSizeΪ������ݽ��ջ��壬ʵ�����ݴ�С����Length-4(4Ϊ��ַ��״̬�볤��)
	
	u8							Bcc8;				//���У��	//���㷶ΧΪ��ʼ�ε����ݶ�(dAdd~data[n])��������
	ePro485UsDef		End;				//������ 0x7F
}sMs485FrmDef;
//============================================================================================ͨѶЭ��End

//**********************************************************************************************************
//**********************************************************************************************************
//**********************************************************************************************************

//============================================================================================������Э��Start
typedef	struct	_ICCRC16
{
	u8 CRC_H;
	u8 CRC_L;
}ICCRC16Def;

typedef	struct	_ICBuffer			//���������ݸ�ʽ
{
	u8 Head;
	u8 CmdType;			//�������� 0x24Ϊ����
	u8 ErrCode;			//״̬�� ����ʱ0x00�޴���0x01�޿�
	u8 data[16];
	ICCRC16Def CRC16;
	u8 End;
}ICBufferDef;

typedef	struct	_ICReadCmd		//���������ʽ
{
	u8	Head;						//0xE1,   /* F-Head 	*/			//Ϊ֡ͷ����λ���·�ʱ�̶�Ϊ0XE1����λ��Ӧ��ʱ�̶�Ϊ0XD2��
	u8	CmdType;				//0x24,  	/* CmdType 	*/			//��������
	u8	Sector;					//0x01,		/* Sector 	*/			//1�ֽ�������
	u8	Block;					//0x01,   /* Block 		*/			//1�ֽڿ��
	u8	KeyType;				//0x60,   /* KeyType	*/			//1�ֽ���Կģʽ KEYA��0x60��/KEYB(0x61)
	u8	PassWord[6];		//6�ֽ�����0xFF
	ICCRC16Def	CRC16;
	u8 End;
}ReadCmdDef;
//============================================================================================������Э��End



//=======================================================�����ͨѶ�ṹ�壺����ҩ�䳣��30S��ȡҩ������һ��ʱ������˸����ҩ�䣺С����һֱ��˸������ͨѶ���ϣ�С���㳣��
typedef struct _SegCmd
{
	unsigned char DispEnNum	:1;		//bit0��ʾ��ֵ	��	0-����ʾ��		1-��ʾ
	unsigned char DispEnDp	:1;		//bit1��ʾ��		��	0-����ʾ��		1-��ʾ
	unsigned char DispMdNum	:1;		//bit2��ֵģʽ	��	0-��̬��ʾ��	1-0.5S��˸
	unsigned char DispMdDp	:1;		//bit3��ģʽ		��	0-��̬��ʾ��	1-0.5S��˸
	unsigned char DispTime	:1;		//bit4��ʾʱ��	��	0-������			1-����ʾʱ���ڸ�����ʾģʽ��ʾ
} SegCmdDef;
typedef struct _sSegFarme 	//�����ͨѶ���ݸ�ʽ
{
	unsigned char desAddr;				//Ŀ�ĵ�ַ---����ܰ����ַ
	unsigned char srcAddr;				/*Դ��ַ*/
	unsigned char runningNumber;	/*��ˮ��*/
	SegCmdDef			cmd;						/*����*/
	unsigned char len;						/*����*/
	unsigned char data[8];				/*����,��λ��ǰ,data[0~3]Ϊ��ʾ����,data[4~7]Ϊ��˸ʱ��*/ 
} sSegFarmeDef;			//��������ݽṹ��
//=======================================================����������
typedef	struct	_CardReader		//�������˿ڼ�����
{
	struct
	{
		unsigned char ReadFlg		:1;	//bit0��0-�޲���,			
		unsigned char ReadData	:1;	//bit1��0-�޲���,			1-�ѷ��Ͷ�������
		unsigned char ReadID		:1;	//bit2��0-�޲���,			1-����ID(��Ҫ����������ݼ���־)		
		unsigned char ReadErr		:1;	//bit3��0-������������	1-������ͨѶ�쳣
		unsigned char NoCard		:1;	//bit5��0-�޲���,			1-�޿�����ҩ�䣩
		unsigned char Active		:1;	//bit6��0-�޲���,			1-���������ϴ�
		unsigned char Passive		:1;	//bit7��0-�޲���,			1-���ͱ����ϴ�
	}ReaderSts;										//������״̬
	unsigned short 	TimeSpan;			//����ʱ����--��λms
	unsigned short 	TimeCount;		//��ʱ��
	unsigned char 	Retry;							//����������
	unsigned short 	DataLen;						//���������ݴ�С
	unsigned long		CardNumber;					//ʮ���ƿ���
	unsigned char 	CardNum[CardLength];	//����
	unsigned char 	Rev[CardDataSize];		//���������ݽ���---���ݣ�������+Э���СΪ22��
	unsigned char 	Rxd[CardDataSize];		//���������ݽ���---���ջ�������DMA����������+Э���СΪ22��
	ICBufferDef			CardBuffer;						//������
	USART_TypeDef*	UartPort;
}CardReaderDef;
//=======================================================���ṹ��
typedef	struct	_LockFlag		//�����Ʊ�־������ʱ��
{
	struct
	{
		unsigned char LockFlg		:1;	//bit0��0-�޲���,1-�в�������(��������״̬��
		unsigned char LockOn		:1;	//bit1��0-�ͷ�����1-����
		unsigned char LockSts		:1;	//bit2��0-���ͷţ�1-�ѿ���
		unsigned char LockTout	:1;	//bit3��0-δ��ʱ��1-�ѳ�ʱ(ȡҩ��ʱ)
	}sLockSts;		//ҩ��״̬
	unsigned long	LockTimeCountdown;		//��������ʱ ��ʱ��0ʱ�ͷ��������LockTimeOut��Ϊ0������
//	unsigned long	Status			:1;		//0-��δ����1-�ѿ�
//	unsigned long	LockOnTime	;	//������ʱ�䣬���ֵ0x3FFF FFFF
	GPIO_TypeDef*	GPIOx;
	u16	GPIO_Pin_n;
}LockFlagDef;
//=======================================================ʱ��ṹ��
typedef	struct	_Time			//���ʱ���
{
	unsigned long	TimeSYS;	//ϵͳ��ʱ��
//	unsigned long	TimeSEG;	//����ܸ������ݼ�ʱ��----�����ڼ�������ͨѶ״̬ 20ms�ⷢһ�������״̬
//	unsigned long	TimeBOX;	//ҩ�����ʱ��	ÿ���˿ڷ���100mSʱ����----�����ڼ�������ͨѶ״̬
	unsigned long	TimeBUS;	//�ⲿ����ʱ��
	unsigned long	TimeCard;	//������ʱ��
}TimeDef;

//=======================================================����ܽṹ��
typedef struct _sSeg 
{
	struct
	{
		unsigned char SegFlg			:1;	//bit0��0-�޲���,		1-�в�������(��Ҫ���������״̬��
		unsigned char Sync				:1;	//bit1��0-�޲���,		1-����
	}sSegSts;			//����ܱ�־
	unsigned short	DisplayTime;		//��ʾʱ��
	unsigned long		SyncTime;				//������ʾʱ��
	sSegFarmeDef 		SegFarme;					//�����ͨѶЭ��
} sSegDef;			//��������ݽṹ��

//=======================================================��������ṹ��
typedef	struct	_sBus		//�˿�״̬--���п�/�޿�
{
	struct
	{
		unsigned short BusFlg		:1;	//bit0��0-�޲���,	1-�д������¼�(�ϱ����ݻ���״̬��
		unsigned short GetID		:1;	//bit1��0-������	1-������	����ȡID������Ҫ�ϱ�ID
		unsigned short GetBox		:1;	//bit2��0-������	1-������	��ȡҩ(�������ֵͬʱ��˸)
		unsigned short GotBox		:1;	//bit3��0-�޲�����	1-ҩ���ѱ�ȡ��
		unsigned short TakeBox	:1;	//bit4��0-�޲�����	1-ҩ�䱻ǿ��ȡ
		unsigned short BoxBack	:1;	//bit5��0-�޲�����	1-ҩ���ջ�,�ջغ���Ҫ�ϱ�ҩ��ţ���GotBox��־ʱ���ô�λ��
		unsigned short TimeOut	:1;	//bit6��0-�޲�����	1-ȡҩ��ʱ
	}BusqSts;		//ҩ��״̬
	unsigned long		GetBoxTimeCountdown;			//ȡҩ��ʱ�������Ϊ0����δ��ʱ������ʱ
	unsigned char 	CardData[CardLength];	//��������CardData[0]Ϊ��λ��CardData[1]Ϊʮλ��CardData[2]Ϊ��λ
	unsigned char		Ucode;								//�û���
	sMs485FrmDef		sBusFarme;		//485���߽ṹ��
}sBusDef;
typedef	struct	_sBusData		//�˿�״̬--���п�/�޿�
{
	sMs485FrmDef	sFarmeTxd;		//����485���ݽṹ��---����
	sMs485FrmDef	sFarmeRxd;		//����485���ݽṹ��---����
	sMs485FrmDef	sFarmeRev;		//����485���ݽṹ��---����
}sBusDataDef;


//=======================================================�˿�/��λ������Ϣ�ṹ��
typedef	struct	_Port
{
	sSegDef					Seg;				//��������ݽṹ��
	LockFlagDef			Lock;				//����Ϣ
	sBusDef					sBus;				//��������ṹ��
	CardReaderDef		sReader;		//�������ṹ��

	unsigned char		PortNum;							//�۵�ַ���˿ںţ�
} PortDef;
//=======================================================���в���
//typedef	struct	_Step
//{
////	unsigned char 	ReadCard		:1;			//bit0:	0-�޲���,1-��Ҫ����
////	unsigned char 	WriteSeg		:1;			//bit1:	0-�޲���,1-��Ҫ���������(����ռ�ô��ڸ�������ܣ�
//} StepDef;
//=======================================================�������ݽṹ��
typedef struct _sPlu 
{
//	StepDef				Step;					//ִ�зֲ�����Ҫ��ֹ������������ܸ��ô������ݴ��ң�
	TimeDef				Time;					//�����ܼ�ʱ��
	sBusDataDef		BusData;			//����485���ݽṹ��---������
	u8 						PowerUpFlag;	//�ϵ��־��0δ�ϵ��꣬1�ϵ���
} sPluDef;			//��������ݽṹ��
//=======================================================���ڶ˿�/��������
typedef	struct	_sBorad		//�˿�״̬--���п�/�޿�
{
	PortDef	Port1;
	PortDef	Port2;
	PortDef	Port3;
	PortDef	Port4;
	
	sPluDef	sPlu;		//�������ݽṹ��

	u8	Nserial;		//��ˮ�� 1~0xFF
}sBoradDef;


//=======================================================ȫ�ֱ���
extern sBoradDef	sBorad;					//�˿�/��λ������Ϣ�ṹ��




//=================================================================================================================������
void PC016V20_Configuration(void);
void PC016V20_Server(void);


//=================================================================================================================��Ϣ����
void MessageServer(sBoradDef*	sBorad);		//�ⲿ������Ϣ����

void MessageAnalysis(sBoradDef*	sBorad);	//������Ϣ

void MessageAck(sBoradDef*	sBorad);			//�ⲿ����Ӧ��

void MessageGetID(sBoradDef*	sBorad);		//��ȡIDָ�����0x12ָ������߱�־,�������5��

void MessageGetBox(sBoradDef*	sBorad);		//ȡҩ����Ϣ������0x13ָ�0x13ָ���ϱ�(0x93))�����߱�־������ܱ�־

void MessageLock(sBoradDef*	sBorad);			//�������Ϣ������0x13ָ�0x13ָ���ϱ�(0x93))������־

void MessageSend(sBoradDef*	sBorad);			//������Ϣ/��Ϣ�ϱ�


//=================================================================================================================���ڴ�������
void BoardServer(sBoradDef*	sBorad);			//���ڣ������ƣ�ҩ�䣬�����

void BoardLockServer(sBoradDef*	sBorad);	//�����ش���

void BoardBoxServer(sBoradDef*	sBorad);	//ҩ�䴦������

void BoardSegServer(sBoradDef*	sBorad);	//�����

void BoardSegClear(sBoradDef*	sBorad);				//����������ʾ


//=================================================================================================================������
void CardReaderServer(sBoradDef*	sBorad);				//������

void CardReaderSetPassive(sBoradDef*	sBorad);		//���ö�����Ϊ��������

void CardReaderSetActive(sBoradDef*	sBorad);			//���ö�����Ϊ��������

void CardReaderReadCmd(sBoradDef*	sBorad);				//����ָ��

void CardDataAnalysis(sBoradDef*	sBorad);			//�����ݸ�ʽ����

void CardReaderRead(sBoradDef*	sBorad);					//�������������յ�����

void CardReaderSendData(USART_TypeDef* USARTx,u8 *tx_buffer,u16 BufferSize);	//���������������

u16 CardReaderReadData(USART_TypeDef* USARTx,u8 *RevBuffer,u8 *RxdBuffer);		//�������������յ�����

u16 Uart5ReceiveData(sBoradDef*	sBorad);			//����5������

void CardReadTimeCount(sBoradDef*	sBorad);										//ȡ���󵹼�ʱ���������ʱδ������ջؿ����ݣ��������ȶ�ʱ������



//=================================================================================================================������
//void Lock_Server(sBoradDef*	sBorad);
//=================================================================================================================Ӳ����������
void HALUsartRemapEnable(void);				//ʹ�ܴ��ڸ���
void HALUsartRemapDisable(void);			//�رմ��ڸ���
void HALSendSeg(u32* Buffer,u16 Length);					//������ܷ�������

//=================================================================================================================���ó���
void Lock_Configuration(void);				//����ʼ��
void Switch_Configuration(void);			//���뿪�س�ʼ��������
void RS485_Configuration(void);				//RS485����
void CardReader_Configuration(void);	//����������
void Data_Initialize(void);						//������ʼ��
void ResetData(void);									//������ر���


void CardReaderError(void);			//������ر���
#endif


