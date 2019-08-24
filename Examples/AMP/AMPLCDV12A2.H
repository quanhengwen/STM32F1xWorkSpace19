#ifndef __AMPLCDV12A2_H
#define __AMPLCDV12A2_H


#include	"AMP_Protocol.H"

#include "GT32L32M0180.H"
#include "ST7789V.H"
#include "STM32_USART.H"
#include "SWITCHID.H"

#define	BackColorStartAddr	0x0801F800		//ԭ��ֵ��MCU�ڲ�FLASH�Ĵ洢��ַ126K��ַ����128KflashӦ��

#define	SelectModel	6

#if 1==SelectModel		//DisplayManaStaticTest1();		//˫��ʾЧ����ʾ
	#define DisplayClrColor			0xFFFF		//��ʾ����������ɫ
	#define DisplayBackColor		0xFFFF		//Ĭ�������ɫ
	#define DisplayFontColor		0x0000		//Ĭ��������ɫ
	
	
	#define DisplayNameBkColor		0xFFFF		//���Ƶ�ɫ
	#define DisplayNameFtColor		0x054A		//������ɫ
	#define	DisplayNameFtSize			24				//���������С
	
	#define DisplayByNameBkColor	0xB71C		//������ɫ
	#define DisplayByNameFtColor	0x094A		//������ɫ
	#define	DisplayByNameFtSize		16				//���������С
	
	#define DisplaySpecBkColor		0xFFFF		//����ɫ
	#define DisplaySpecFtColor		0x5ACA		//�����ɫ
	#define	DisplaySpecFtSize			16				//��������С
	
	#define DisplayNumBkColor			0x5902		//������ɫ
	#define DisplayNumFtColor			0x07DF		//������ɫ
	#define	DisplayNumFtSize			48				//���������С
	
	#define DisplaySerialBkColor	0xFFFF		//������ɫ
	#define DisplaySerialFtColor	0xFB00		//������ɫ
	#define	DisplaySerialFtSize		16				//���������С
	
	#define DisplayTitleBkColor		0x865D		//�����ɫ
	#define DisplayTitleFtColor		0x0000		//������ɫ
	#define DisplayTitleSize			16				//���������С
	
	#define DisplaySeparBkColor		0x8450		//�ָ�����ɫ
	#define DisplaySeparWidth			2					//�ָ��߿���
	
	#define DisplayCodeBkColor		0xFFFF		//���ϱ����ɫ
	#define DisplayCodeFtColor		0x5ACA		//���ϱ�����ɫ
	#define	DisplayCodeFtSize			16				//���ϱ��������С
	
	#define DisplayVenderBkColor	0xFFFF		//�������Ƶ�ɫ
	#define DisplayVenderFtColor	0x5ACA		//����������ɫ
	#define	DisplayVenderFtSize		16				//�������������С
	
	#define DisplayTopStartY			16
	#define DisplayTopEndY				8	
	#define DisplayBotStartY			ST7789V_H-8
	#define DisplayBotEndY				ST7789V_H
	
	
	
	#define DisplayStartX					2
	#define DisplayEndX						ST7789V_V	
#elif 2==SelectModel		//DisplayManaStaticTest2();	//����ʾЧ����ʾ
	#define DisplayClrColor			0xFFFF		//��ʾ����������ɫ
	#define DisplayBackColor		0xFFFF		//Ĭ�������ɫ
	#define DisplayFontColor		0x0000		//Ĭ��������ɫ
	
	
	#define DisplayNameBkColor	0xFFFF		//���Ƶ�ɫ
	#define DisplayNameFtColor	0x054A		//������ɫ
	#define	DisplayNameFtSize		24				//���������С
	
	#define DisplayByNameBkColor	0xB71C		//������ɫ
	#define DisplayByNameFtColor	0x094A		//������ɫ
	#define	DisplayByNameFtSize		16				//���������С
	
	#define DisplaySpecBkColor	0xFFFF		//����ɫ
	#define DisplaySpecFtColor	0x5ACA		//�����ɫ
	#define	DisplaySpecFtSize		16				//��������С
	
	#define DisplayNumBkColor		0x5902		//������ɫ
	#define DisplayNumFtColor		0x07DF		//������ɫ
	#define	DisplayNumFtSize		32				//���������С
	
	#define DisplaySerialBkColor	0xFFFF		//������ɫ
	#define DisplaySerialFtColor	0xFB00		//������ɫ
	#define	DisplaySerialFtSize		16				//���������С
	
	#define DisplayCodeBkColor		0xFFFF		//���ϱ����ɫ
	#define DisplayCodeFtColor		0x5ACA		//���ϱ�����ɫ
	#define	DisplayCodeFtSize			16				//���ϱ��������С
	
	#define DisplayVenderBkColor		0xFFFF		//�������Ƶ�ɫ
	#define DisplayVenderFtColor		0x5ACA		//����������ɫ
	#define	DisplayVenderFtSize			16				//�������������С
	
	#define DisplayTitleBkColor		0x865D		//�����ɫ
	#define DisplayTitleFtColor		0x0000		//������ɫ
	#define DisplayTitleSize			16				//���������С
	
	#define DisplaySeparBkColor		0x8450		//�ָ�����ɫ
	#define DisplaySeparWidth	2							//�ָ��߿���
	
	#define DisplayTopStartY	48
	#define DisplayTopEndY		8	
	#define DisplayBotStartY	ST7789V_H-48
	#define DisplayBotEndY		ST7789V_H
	
	
	
	#define DisplayStartX	2
	#define DisplayEndX		ST7789V_V
#elif 3==SelectModel							//DisplayManaModel1();	//˫��ʾ--�����������
	#define DisplayClrColor					0xFFFF		//��ʾ����������ɫ
	#define DisplayBackColor				0xFFFF		//Ĭ�������ɫ
	#define DisplayFontColor				0x0000		//Ĭ��������ɫ
	
	
	#define DisplayNameBkColor			0xFFFF		//���Ƶ�ɫ
	#define DisplayNameFtColor			0x054A		//������ɫ
	#define	DisplayNameFtSize				24				//���������С
	
	#define DisplayByNameBkColor		0xB71C		//������ɫ
	#define DisplayByNameFtColor		0x094A		//������ɫ
	#define	DisplayByNameFtSize			16				//���������С
	
	#define DisplaySpecBkColor			0xFFFF		//����ɫ
	#define DisplaySpecFtColor			0x5ACA		//�����ɫ
	#define	DisplaySpecFtSize				16				//��������С
	
	#define DisplayNumBkColor				0x5902		//������ɫ
	#define DisplayNumFtColor				0x07DF		//������ɫ
	#define	DisplayNumFtSize				48				//���������С
	
	#define DisplaySerialBkColor		0xFFFF		//������ɫ
	#define DisplaySerialFtColor		0xFB00		//������ɫ
	#define	DisplaySerialFtSize			16				//���������С
	
	#define DisplayCodeBkColor			0xFFFF		//���ϱ����ɫ
	#define DisplayCodeFtColor			0x5ACA		//���ϱ�����ɫ
	#define	DisplayCodeFtSize				16				//���ϱ��������С
	
	#define DisplayVenderBkColor		0xFFFF		//�������Ƶ�ɫ
	#define DisplayVenderFtColor		0x5ACA		//����������ɫ
	#define	DisplayVenderFtSize			16				//�������������С
	
	#define DisplayTitleBkColor			0x865D		//�����ɫ
	#define DisplayTitleFtColor			0x0000		//������ɫ
	#define DisplayTitleSize				16				//���������С
	
	#define DisplaySeparBkColor			0x8450		//�ָ�����ɫ
	#define DisplaySeparWidth				2					//�ָ��߿���
	
	#define DisplayTopStartY				16
	#define DisplayTopEndY					8	
	#define DisplayBotStartY				ST7789V_H-8
	#define DisplayBotEndY					ST7789V_H	
	
	#define DisplayStartX	2
	#define DisplayEndX		ST7789V_V	
	
#elif 4==SelectModel		//DisplayManaModel2();	//˫��ʾ--����32������
	#define DisplayClrColor					0xFFFF		//��ʾ����������ɫ
	#define DisplayBackColor				0xFFFF		//Ĭ�������ɫ
	#define DisplayFontColor				0x0000		//Ĭ��������ɫ
	
	
	#define DisplayNameBkColor			0x1C9F		//���Ƶ�ɫ
	#define DisplayNameFtColor			0x0000		//������ɫ
	#define	DisplayNameFtSize				24				//���������С
	
	#define DisplayByNameBkColor		0xB71C		//������ɫ
	#define DisplayByNameFtColor		0x094A		//������ɫ
	#define	DisplayByNameFtSize			16				//���������С
	
	#define DisplaySpecBkColor			0xFFFF		//����ɫ
	#define DisplaySpecFtColor			0x5ACA		//�����ɫ
	#define	DisplaySpecFtSize				16				//��������С
	
	#define DisplayNumBkColor				0x5902		//������ɫ
	#define DisplayNumFtColor				0x07DF		//������ɫ
	#define	DisplayNumFtSize				32				//���������С
	
	#define DisplaySerialBkColor		0xFFFF		//������ɫ
	#define DisplaySerialFtColor		0xFB00		//������ɫ
	#define	DisplaySerialFtSize			16				//���������С
	
	#define DisplayCodeBkColor			0xFFFF		//���ϱ����ɫ
	#define DisplayCodeFtColor			0x5ACA		//���ϱ�����ɫ
	#define	DisplayCodeFtSize				16				//���ϱ��������С
	
	#define DisplayVenderBkColor		0xFFFF		//�������Ƶ�ɫ
	#define DisplayVenderFtColor		0x5ACA		//����������ɫ
	#define	DisplayVenderFtSize			16				//�������������С
	
	#define DisplayTitleBkColor			0x865D		//�����ɫ
	#define DisplayTitleFtColor			0x0000		//������ɫ
	#define DisplayTitleSize				16				//���������С
	
	#define DisplaySeparBkColor			0x8450		//�ָ�����ɫ
	#define DisplaySeparWidth				2					//�ָ��߿���
	
	#define DisplayTopStartY	24
	#define DisplayTopEndY		8	
	#define DisplayBotStartY	ST7789V_H-16
	#define DisplayBotEndY		ST7789V_H	
	
	#define DisplayStartX	2
	#define DisplayEndX		ST7789V_V	
#elif 5==SelectModel		//DisplayManaModel3();	//����ʾ--����32������
	#define DisplayClrColor			0xFFFF		//��ʾ����������ɫ
	#define DisplayBackColor		0xFFFF		//Ĭ�������ɫ
	#define DisplayFontColor		0x0000		//Ĭ��������ɫ
	
	
	#define DisplayNameBkColor		0xFFFF		//���Ƶ�ɫ
	#define DisplayNameFtColor		0x054A		//������ɫ
	#define	DisplayNameFtSize			24				//���������С
	
	#define DisplayByNameBkColor	0xB71C		//������ɫ
	#define DisplayByNameFtColor	0x094A		//������ɫ
	#define	DisplayByNameFtSize		16				//���������С
	
	#define DisplaySpecBkColor		0xFFFF		//����ɫ
	#define DisplaySpecFtColor		0x5ACA		//�����ɫ
	#define	DisplaySpecFtSize			16				//��������С
	
	#define DisplayNumBkColor			0x5902		//������ɫ
	#define DisplayNumFtColor			0x07DF		//������ɫ
	#define	DisplayNumFtSize			32				//���������С
	
	#define DisplaySerialBkColor	0xFFFF		//������ɫ
	#define DisplaySerialFtColor	0xFB00		//������ɫ
	#define	DisplaySerialFtSize		16				//���������С
	
	#define DisplayCodeBkColor		0xFFFF		//���ϱ����ɫ
	#define DisplayCodeFtColor		0x5ACA		//���ϱ�����ɫ
	#define	DisplayCodeFtSize			16				//���ϱ��������С
	
	#define DisplayVenderBkColor	0xFFFF		//�������Ƶ�ɫ
	#define DisplayVenderFtColor	0x5ACA		//����������ɫ
	#define	DisplayVenderFtSize		16				//�������������С
	
	#define DisplayTitleBkColor		0x865D		//�����ɫ
	#define DisplayTitleFtColor		0x0000		//������ɫ
	#define DisplayTitleSize			24				//���������С
	
	#define DisplaySeparBkColor		0x8450		//�ָ�����ɫ
	#define DisplaySeparWidth			2					//�ָ��߿���
	
	#define DisplayTopStartY			48
	#define DisplayTopEndY				8	
	#define DisplayBotStartY			ST7789V_H-48
	#define DisplayBotEndY				ST7789V_H
	
	
	
	#define DisplayStartX	2
	#define DisplayEndX		ST7789V_V
#else			//DisplayManaModel1();
	#define DisplayClrColor				0xFFFF		//��ʾ����������ɫ
	#define DisplayBackColor			0xFFFF		//Ĭ�������ɫ
	#define DisplayFontColor			0x0000		//Ĭ��������ɫ
	#define DisplayFontSize				16				//Ĭ�������С
	
	
	#define DisplayNameBkColor		0xB71C		//���Ƶ�ɫ
	#define DisplayNameFtColor		0x054A		//������ɫ
	#define	DisplayNameFtSize			24				//���������С
	
	#define DisplayByNameBkColor	0xFFFF		//������ɫ
	#define DisplayByNameFtColor	0x0000		//������ɫ
	#define	DisplayByNameFtSize		16				//���������С
	
	#define DisplaySpecBkColor		0xFFFF		//����ɫ
	#define DisplaySpecFtColor		0x0000		//�����ɫ
	#define	DisplaySpecFtSize			16				//��������С
	
	#define DisplayNumBkColor			0xFBC8		//������ɫ
	#define DisplayNumFtColor			0x8981		//������ɫ
	#define	DisplayNumFtSize			32				//���������С
	
	#define DisplaySerialBkColor	0xFFFF		//��ŵ�ɫ
	#define DisplaySerialFtColor	0xFB00		//�����ɫ
	#define	DisplaySerialFtSize		16				//��������С
	
	#define DisplayCodeBkColor		0xFFFF		//���ϱ����ɫ
	#define DisplayCodeFtColor		0x0000		//���ϱ�����ɫ
	#define	DisplayCodeFtSize			16				//���ϱ��������С
	
	#define DisplayVenderBkColor	0xFFFF		//�������Ƶ�ɫ
	#define DisplayVenderFtColor	0x0000		//����������ɫ
	#define	DisplayVenderFtSize		16				//�������������С
	
	#define DisplayTitleBkColor		0xfffF		//�����ɫ
	#define DisplayTitleFtColor		0x0000		//������ɫ
	#define DisplayTitleSize			16				//���������С
	
	#define DisplaySeparBkColor		0x8450		//�ָ�����ɫ
	#define DisplaySeparWidth			2							//�ָ��߿���
	
	#define DisplayTopFillWidth		16				//�������߶�
	#define DisplayTopEndY				8	
	#define DisplayBotFillWidth		16				//�ײ����߶�
	#define DisplayBotEndY				ST7789V_H
	
	#define DisplayLeftFillWidth	9					//���������/��ʼ��	
	
	#define DisplayStartX	DisplayLeftFillWidth
	#define DisplayEndX						ST7789V_V
	
	
	#define ampLcdTopFillWidth		16				//����������
	#define ampLcdBotFillWidth		16				//�ײ����߶�
	#define ampLcdLeftFillWidth		5					//������߶�
	#define ampLcdRigthFillWidth	10				//�ұ�������
	#define ampLcdTitleWidth			16				//����������
	
#endif

#define	ampLcdListSize					5					//������������������
#define	FresPageTime						2000			//ҳ��ˢ��ʱ��--ms
#define	AMPLcdMaxStringLen			256-9

//-----------------------------------LCD��ʾ��ؽṹ��
typedef enum _ampLcdParaTye						//���ͣ�0x01-�Ĳ����ƣ�0x02-�ĲĹ��0x03-�Ĳ�����,0x04-������0x05-�������ƣ�0x06-�Ĳı��룬0x07-������λ
{
	ampLcdParaNone		=0x00,
	ampLcdParaName		=0x01,	//0x01-�Ĳ�����
	ampLcdParaSpec		=0x02,	//0x02-�ĲĹ��
	ampLcdParaNum			=0x03,	//0x03-�Ĳ�����
	ampLcdParaByName	=0x04,	//0x04-����
	ampLcdParaVender	=0x05,	//0x05-��������
	ampLcdParaCode		=0x06,	//0x06-�Ĳı���
	ampLcdParaUnit		=0x07,	//0x07-������λ
}ampLcdParaTyeDef;
//-----------------------------------Ӧ����Ϣ
typedef struct _ampAck       //�������ݣ�
{
  unsigned char head;     	//0x7E
  unsigned char length;   	//����λ������head,leng,crcl,crch,end����	
	ampcmddef			cmd;				//����/״̬
	unsigned char address1;		//���ַ
  unsigned char address2;		//���ַ
  unsigned char address3;		//λ��ַ
	unsigned char status;			//״̬-Ĭ��0
	unsigned char Crc16L;
	unsigned char Crc16H;
	unsigned char end;				//0x7F
	
	unsigned char flag;				//��־:0-��Ӧ��1-��Ҫ�ϱ�Ӧ��
}ampAckDef;
//-----------------------------------LCD��ʾ��ؽṹ��
typedef struct _ampLcdMana						//�Ĳ����ݽṹ�壬���ƣ��������ʹ����ͬ�Ľṹ��
{
	unsigned char		len;					//�������ȣ���ָ��ʾ���ַ�������
	unsigned char		Offset;				//�ַ�����ַƫ�ƣ��������ַ������ͺͳ���λ
	unsigned short	XH;						//X--ˮƽռ�õ���
	unsigned short	YV;						//Y--��ֱռ�õ���
}ampLcdManaDef;
//---------------------------------------------------------------------
typedef struct _ampLcdDataNode			//��ʾ���ݽṹ��
{	
	ampLcdManaDef	Name;						//��Ʒ�����ַ�������
	ampLcdManaDef	ByName;					//�����ַ�������
	ampLcdManaDef	Spec;						//����ַ�������
	ampLcdManaDef	Vender;					//���������ַ�������
	ampLcdManaDef	Code;						//�����ַ�������
	ampLcdManaDef	Num;						//�����ַ�������
	ampLcdManaDef	Unit;						//��λ�ַ�������
	//----------------�ַ�������
	unsigned char		Serial;				//��Ŀ���/���
	unsigned char		SumLen;				//���ַ�������
	unsigned short	SumPixelY;		//Y--��ֱռ�õ���
	unsigned char		String[AMPLcdMaxStringLen];		//���ֺ͹���ַ������棬������ǰ
}ampLcdDataNodeDef;










//-----------------------------------LCD��ʾ��ؽṹ��
typedef struct _Mana						//�Ĳ����ݽṹ�壬���ƣ��������ʹ����ͬ�Ľṹ��
{
	ampLcdParaTyeDef	type;					//���ͣ�0x01-�Ĳ����ƣ�0x02-�ĲĹ��0x03-�Ĳ�����,0x04-������0x05-�������ƣ�0x06-�Ĳı��룬0x07-������λ
	unsigned char			len;					//�������ȣ�String���ݳ���--hex�ֽڵ�λ
	unsigned char			String[AMPLcdMaxStringLen];		//���建�棬������������������ֵ������Ǹı���ɫʱ����Ϊ�գ���������ƻ��߹��ʱ������ַ�����GBK��
}ManaDef;
//---------------------------------------------------------------------
typedef struct _Para						//�Ĳ����ݽṹ�壬���ƣ��������ʹ����ͬ�Ľṹ��
{
	unsigned char		len;					//��������--�ַ�����
	unsigned char		Offset;				//�洢ƫ�Ƶ�ַ--��String�ַ����е���ʼ��ַ	
	unsigned short	XH;						//X--ˮƽռ�õ���
	unsigned short	YV;						//Y--��ֱռ�õ���
}ParaDef;
//---------------------------------------------------------------------
typedef struct _ListData			//��ʾ���ݽṹ��
{	
	unsigned char		Serial;			//��Ŀ���/���
	ParaDef	ParaName;						//��Ʒ�����ַ�������
	ParaDef	ParaByName;					//�����ַ�������
	ParaDef	ParaSpec;						//����ַ�������
	ParaDef	ParaVender;					//���������ַ�������
	ParaDef	ParaCode;						//�����ַ�������
	ParaDef	ParaNum;						//�����ַ�������
	ParaDef	ParaUnit;						//��λ�ַ�������
	//----------------�ַ�������
	
	unsigned char		len;				//�ַ�������
	unsigned short	Ycount;			//Y--��ֱռ�õ���
	unsigned char		String[AMPLcdMaxStringLen];		//���ֺ͹���ַ������棬������ǰ
}ampLcdListDef;
//---------------------------------------------------------------------
typedef struct _Font					//���壺����ɫ������ɫ�������С
{	
	unsigned short	BackColor;	//�����ɫ
	unsigned short	PenColor;		//������ɫ
	unsigned char		Size;				//�����С16,24,32
}FontDef;
//---------------------------------------------------------------------
typedef struct _XYPointCount					//XY��Ҫ����
{	
	unsigned short	XH;					//X--ˮƽ
	unsigned short	YV;					//Y--��ֱ
}PxyDef;
//---------------------------------------------------------------------
typedef struct _ampLcdAre					//XY��Ҫ����
{	
	unsigned short	xs;					//X--ˮƽ��ʼ��
	unsigned short	ys;					//Y--��ֱ��ʼ��
	unsigned short	xe;					//X--ˮƽ��ֹ��
	unsigned short	ye;					//Y--��ֱ��ֹ��
}ampLcdAreDef;
//---------------------------------------------------------------------







typedef struct _ampLcdSysData		//ϵͳ����
{
	unsigned char 	AddrCab;			//���ַ
	unsigned char 	AddrLay;			//���ַ
	unsigned char 	AddrSeg;			//λ��ַ
}ampLcdSysDataDef;
//---------------------------------------------------------------------

typedef struct _ampLcdHal	
{
	RS485Def 				RS485Port;   	//RS485ͨѶ
	sST7789VDef			LcdPort;			//LCD
	GT32L32Def 			GT32L32;			//�ֿ�
	SwitchDef 			sSwitch;			//���뿪��
}ampLcdHalDef;
//---------------------------------------------------------------------

typedef struct _ampLcdComm			//ͨѶ����
{
	unsigned	char	Rxd[maxFramesize];	//����
	unsigned	char	Txd[maxFramesize];	//����
	unsigned	short TxLen;							//��Ҫ���͵�����
}ampLcdCommDef;
//---------------------------------------------------------------------

typedef struct _ampLcdCof				//��ʼ������
{
	//----------------------------------�������
	FontDef	FtName;						//��Ʒ�����������
	FontDef	FtByName;					//��Ʒ�����������
	FontDef	FtSpec;						//��Ʒ����������
	FontDef	FtNum;						//��Ʒ�����������
	FontDef	FtSeril;					//��ʾ����������
	FontDef	FtCode;						//���ϱ����������
	FontDef	FtVender;					//���������������
	FontDef	FtTitle;					//�����������
	FontDef	FtSepar;					//�ָ��߲���
	FontDef	FtDefault;				//Ĭ�������������
	//----------------------------------��ʾ��
	PxyDef	PxyPixel;					//������
	PxyDef	PxyTopFill;				//�������߶�
	PxyDef	PxyBotFill;				//�ײ����߶�
	PxyDef	PxyLeftFill;			//���������
	PxyDef	PxyValid;					//��Ч��ʾ����
	
	ampLcdAreDef	AreMain;		//����ʾ����:��ʾ�Ĳ�����
	ampLcdAreDef	AreTitle;		//������ʾ����
}ampLcdCofDef;
//---------------------------------------------------------------------
typedef struct _ampLcdDisplayData						//��ʾ���ݽṹ��
{
	ampLcdListDef	List[ampLcdListSize];	//��ʾ����
	
	ampLcdDataNodeDef	Node[ampLcdListSize];	//��ʾ����

	
	unsigned char		Serial;			//��ǰ��ʾ���
	unsigned char		Count;			//����ʹ�ø���
	unsigned char		Sync;				//0-����Ҫͬ����1-��Ҫͬ��
	unsigned short	StartY;			//��ǰ��ʾY��ʼ��
	
	ampLcdAreDef	FillStart;								//��ʾǰ�����ʾ��ʼ����
	ampLcdAreDef	FillEnd;									//��ʾǰ�����ʾ��������
	
	ampLcdAreDef	ShowStart;								//�ַ���ʾ��ʼ����
	ampLcdAreDef	ShowEnd;									//�ַ���ʾ��������
	
}ampLcdDisplayDataDef;
//---------------------------------------------------------------------
typedef struct _ampLcdRun						//��������
{
	//----------------------------------��������
	PxyDef	PxyFillStart;							//��ʾǰ�����ʾ��ʼ����
	PxyDef	PxyFillEnd;								//��ʾǰ�����ʾ��������

	PxyDef	PixelStart;								//�ַ���ʾ��ʼ����
	PxyDef	PixelEnd;									//�ַ���ʾ��������
	
	unsigned short	TopDisplayStartY;	//�ϰ����ַ���ʾ��ʼ��ַ
	unsigned short	BotDisplayStartY;	//�°�ҳ�ַ���ʾ��ʼ��ַ
	unsigned short	StartPixelY;			//��ǰ��ʾʱ��Y��ʼ��
}ampLcdRunDef;
//---------------------------------------------------------------------

//---------------------------------------------------------------------
typedef struct _LcdData	//����
{
	ampLcdCommDef					CommData;
	ampLcdSysDataDef			SysData;
	ampLcdRunDef					RunData;
	ampLcdDisplayDataDef	Display;
	const ampLcdCofDef		Cof;				//��������
}ampLcdDataDef;
//---------------------------------------------------------------------

//---------------------------------------------------------------------
typedef struct _sAmpLcd
{
	ampLcdHalDef		Hal;	
	ampLcdDataDef		Data;	
}sAmpLcdDef;
//---------------------------------------------------------------------


void AMPLCDV12A2_Configuration(void);
void AMPLCDV12A2_Server(void);



void ampLcd_data_receive(void);
void ampLcd_data_send(void);
void ampLcd_data_process(unsigned char* pBuffer,unsigned short len);		//Э�顢��ַ���
void ampLcd_data_extract(unsigned char* String,unsigned short len);		//��ȡ�ַ�������
void ampLcd_data_add(ampLcdDataNodeDef* pNode,ampLcdDataNodeDef Node);
unsigned short ampLcd_data_get_pixel(ampLcdManaDef* Mana,unsigned char font);

void ampLcd_display(void);
void ampLcd_display_string(void);
void ampLcd_display_Name(ampLcdDataNodeDef Node);
void ampLcd_display_ByName(ampLcdDataNodeDef Node);
void ampLcd_display_Vender(ampLcdDataNodeDef Node);
void ampLcd_display_Specs(ampLcdDataNodeDef Node);
void ampLcd_display_Code(ampLcdDataNodeDef Node);
void ampLcd_display_Number(ampLcdDataNodeDef Node);



void ampLcd_Display_Clear(void);


static void HW_Configuration(void);
static void DataInitialize(void);		//���ݳ�ʼ��

static void DisplayString(void);
static void DisplayGui(void);				//��ʾ����
static void DisplayName(const ampLcdListDef Node);				//��ʾ����������
static void DisplayByName(const ampLcdListDef Node);			//��ʾ����
static void DisplayVender(const ampLcdListDef Node);			//��ʾ��������
static void DisplaySpec(const ampLcdListDef Node);				//��ʾ���
static void DisplayCode(const ampLcdListDef Node);				//��ʾ����
static void DisplayNumber(const ampLcdListDef Node);			//��ʾ�����͵�λ
static void DisplayTitle(unsigned char DualFlag);			//��ʾ����
static unsigned short GetYVLen(const ampLcdListDef Node);//��ȡ�ڵ�����ռ��Y��ĵ���

static void GetManaData(const unsigned char* Databuffer,unsigned short datalen);
static void SetManaData(ampLcdListDef* pNode);		//������ʾ����---���ݻ�ȡ�ɹ���������ص���ʾ����


void ampLcd_SetManaDataInfo(const unsigned char* String,unsigned short datalen);
unsigned char* ampLcd_get_type_data_address(const unsigned char* String,unsigned short datalen,ampLcdParaTyeDef	ampLcdParaTye);


static void AckData(void);


static void Lcd_Process(void);
static void AddressNoneProcess(void);
static void NoDataProcess(void);
static void SwitchID_Server(void);
static void SysLed_server(void);




void SetBackColor(unsigned short BKColor);
unsigned short GetBackColor(void);


static void ampLcd_Display_Server(void);
static void ampLcd_Display_String(const unsigned char* String);

//unsigned short HW_SendBuff(enCCPortDef Port,unsigned char* pBuffer,unsigned short length);
//=================================ʱ�亯��ST==============================================================






#endif