#ifndef __AMPLCDV12A2_H
#define __AMPLCDV12A2_H


#include	"AMP_Protocol.H"

#include "GT32L32M0180.H"
#include "ST7789V.H"
#include "STM32_USART.H"
#include "SWITCHID.H"

#define	BackColorStartAddr	0x0801F800		//原点值在MCU内部FLASH的存储地址126K地址，按128Kflash应用

#define	SelectModel	6

#if 1==SelectModel		//DisplayManaStaticTest1();		//双显示效果演示
	#define DisplayClrColor			0xFFFF		//显示区域清屏颜色
	#define DisplayBackColor		0xFFFF		//默认字体底色
	#define DisplayFontColor		0x0000		//默认字体颜色
	
	
	#define DisplayNameBkColor		0xFFFF		//名称底色
	#define DisplayNameFtColor		0x054A		//名称颜色
	#define	DisplayNameFtSize			24				//名称字体大小
	
	#define DisplayByNameBkColor	0xB71C		//别名底色
	#define DisplayByNameFtColor	0x094A		//别名颜色
	#define	DisplayByNameFtSize		16				//别名字体大小
	
	#define DisplaySpecBkColor		0xFFFF		//规格底色
	#define DisplaySpecFtColor		0x5ACA		//规格颜色
	#define	DisplaySpecFtSize			16				//规格字体大小
	
	#define DisplayNumBkColor			0x5902		//数量底色
	#define DisplayNumFtColor			0x07DF		//数量颜色
	#define	DisplayNumFtSize			48				//数量字体大小
	
	#define DisplaySerialBkColor	0xFFFF		//数量底色
	#define DisplaySerialFtColor	0xFB00		//数量颜色
	#define	DisplaySerialFtSize		16				//数量字体大小
	
	#define DisplayTitleBkColor		0x865D		//标题底色
	#define DisplayTitleFtColor		0x0000		//标题颜色
	#define DisplayTitleSize			16				//标题字体大小
	
	#define DisplaySeparBkColor		0x8450		//分隔线颜色
	#define DisplaySeparWidth			2					//分隔线宽度
	
	#define DisplayCodeBkColor		0xFFFF		//物料编码底色
	#define DisplayCodeFtColor		0x5ACA		//物料编码颜色
	#define	DisplayCodeFtSize			16				//物料编码字体大小
	
	#define DisplayVenderBkColor	0xFFFF		//厂家名称底色
	#define DisplayVenderFtColor	0x5ACA		//厂家名称颜色
	#define	DisplayVenderFtSize		16				//厂家名称字体大小
	
	#define DisplayTopStartY			16
	#define DisplayTopEndY				8	
	#define DisplayBotStartY			ST7789V_H-8
	#define DisplayBotEndY				ST7789V_H
	
	
	
	#define DisplayStartX					2
	#define DisplayEndX						ST7789V_V	
#elif 2==SelectModel		//DisplayManaStaticTest2();	//单显示效果演示
	#define DisplayClrColor			0xFFFF		//显示区域清屏颜色
	#define DisplayBackColor		0xFFFF		//默认字体底色
	#define DisplayFontColor		0x0000		//默认字体颜色
	
	
	#define DisplayNameBkColor	0xFFFF		//名称底色
	#define DisplayNameFtColor	0x054A		//名称颜色
	#define	DisplayNameFtSize		24				//名称字体大小
	
	#define DisplayByNameBkColor	0xB71C		//别名底色
	#define DisplayByNameFtColor	0x094A		//别名颜色
	#define	DisplayByNameFtSize		16				//别名字体大小
	
	#define DisplaySpecBkColor	0xFFFF		//规格底色
	#define DisplaySpecFtColor	0x5ACA		//规格颜色
	#define	DisplaySpecFtSize		16				//规格字体大小
	
	#define DisplayNumBkColor		0x5902		//数量底色
	#define DisplayNumFtColor		0x07DF		//数量颜色
	#define	DisplayNumFtSize		32				//数量字体大小
	
	#define DisplaySerialBkColor	0xFFFF		//数量底色
	#define DisplaySerialFtColor	0xFB00		//数量颜色
	#define	DisplaySerialFtSize		16				//数量字体大小
	
	#define DisplayCodeBkColor		0xFFFF		//物料编码底色
	#define DisplayCodeFtColor		0x5ACA		//物料编码颜色
	#define	DisplayCodeFtSize			16				//物料编码字体大小
	
	#define DisplayVenderBkColor		0xFFFF		//厂家名称底色
	#define DisplayVenderFtColor		0x5ACA		//厂家名称颜色
	#define	DisplayVenderFtSize			16				//厂家名称字体大小
	
	#define DisplayTitleBkColor		0x865D		//标题底色
	#define DisplayTitleFtColor		0x0000		//标题颜色
	#define DisplayTitleSize			16				//标题字体大小
	
	#define DisplaySeparBkColor		0x8450		//分隔线颜色
	#define DisplaySeparWidth	2							//分隔线宽度
	
	#define DisplayTopStartY	48
	#define DisplayTopEndY		8	
	#define DisplayBotStartY	ST7789V_H-48
	#define DisplayBotEndY		ST7789V_H
	
	
	
	#define DisplayStartX	2
	#define DisplayEndX		ST7789V_V
#elif 3==SelectModel							//DisplayManaModel1();	//双显示--数量大号字体
	#define DisplayClrColor					0xFFFF		//显示区域清屏颜色
	#define DisplayBackColor				0xFFFF		//默认字体底色
	#define DisplayFontColor				0x0000		//默认字体颜色
	
	
	#define DisplayNameBkColor			0xFFFF		//名称底色
	#define DisplayNameFtColor			0x054A		//名称颜色
	#define	DisplayNameFtSize				24				//名称字体大小
	
	#define DisplayByNameBkColor		0xB71C		//别名底色
	#define DisplayByNameFtColor		0x094A		//别名颜色
	#define	DisplayByNameFtSize			16				//别名字体大小
	
	#define DisplaySpecBkColor			0xFFFF		//规格底色
	#define DisplaySpecFtColor			0x5ACA		//规格颜色
	#define	DisplaySpecFtSize				16				//规格字体大小
	
	#define DisplayNumBkColor				0x5902		//数量底色
	#define DisplayNumFtColor				0x07DF		//数量颜色
	#define	DisplayNumFtSize				48				//数量字体大小
	
	#define DisplaySerialBkColor		0xFFFF		//数量底色
	#define DisplaySerialFtColor		0xFB00		//数量颜色
	#define	DisplaySerialFtSize			16				//数量字体大小
	
	#define DisplayCodeBkColor			0xFFFF		//物料编码底色
	#define DisplayCodeFtColor			0x5ACA		//物料编码颜色
	#define	DisplayCodeFtSize				16				//物料编码字体大小
	
	#define DisplayVenderBkColor		0xFFFF		//厂家名称底色
	#define DisplayVenderFtColor		0x5ACA		//厂家名称颜色
	#define	DisplayVenderFtSize			16				//厂家名称字体大小
	
	#define DisplayTitleBkColor			0x865D		//标题底色
	#define DisplayTitleFtColor			0x0000		//标题颜色
	#define DisplayTitleSize				16				//标题字体大小
	
	#define DisplaySeparBkColor			0x8450		//分隔线颜色
	#define DisplaySeparWidth				2					//分隔线宽度
	
	#define DisplayTopStartY				16
	#define DisplayTopEndY					8	
	#define DisplayBotStartY				ST7789V_H-8
	#define DisplayBotEndY					ST7789V_H	
	
	#define DisplayStartX	2
	#define DisplayEndX		ST7789V_V	
	
#elif 4==SelectModel		//DisplayManaModel2();	//双显示--数量32号字体
	#define DisplayClrColor					0xFFFF		//显示区域清屏颜色
	#define DisplayBackColor				0xFFFF		//默认字体底色
	#define DisplayFontColor				0x0000		//默认字体颜色
	
	
	#define DisplayNameBkColor			0x1C9F		//名称底色
	#define DisplayNameFtColor			0x0000		//名称颜色
	#define	DisplayNameFtSize				24				//名称字体大小
	
	#define DisplayByNameBkColor		0xB71C		//别名底色
	#define DisplayByNameFtColor		0x094A		//别名颜色
	#define	DisplayByNameFtSize			16				//别名字体大小
	
	#define DisplaySpecBkColor			0xFFFF		//规格底色
	#define DisplaySpecFtColor			0x5ACA		//规格颜色
	#define	DisplaySpecFtSize				16				//规格字体大小
	
	#define DisplayNumBkColor				0x5902		//数量底色
	#define DisplayNumFtColor				0x07DF		//数量颜色
	#define	DisplayNumFtSize				32				//数量字体大小
	
	#define DisplaySerialBkColor		0xFFFF		//数量底色
	#define DisplaySerialFtColor		0xFB00		//数量颜色
	#define	DisplaySerialFtSize			16				//数量字体大小
	
	#define DisplayCodeBkColor			0xFFFF		//物料编码底色
	#define DisplayCodeFtColor			0x5ACA		//物料编码颜色
	#define	DisplayCodeFtSize				16				//物料编码字体大小
	
	#define DisplayVenderBkColor		0xFFFF		//厂家名称底色
	#define DisplayVenderFtColor		0x5ACA		//厂家名称颜色
	#define	DisplayVenderFtSize			16				//厂家名称字体大小
	
	#define DisplayTitleBkColor			0x865D		//标题底色
	#define DisplayTitleFtColor			0x0000		//标题颜色
	#define DisplayTitleSize				16				//标题字体大小
	
	#define DisplaySeparBkColor			0x8450		//分隔线颜色
	#define DisplaySeparWidth				2					//分隔线宽度
	
	#define DisplayTopStartY	24
	#define DisplayTopEndY		8	
	#define DisplayBotStartY	ST7789V_H-16
	#define DisplayBotEndY		ST7789V_H	
	
	#define DisplayStartX	2
	#define DisplayEndX		ST7789V_V	
#elif 5==SelectModel		//DisplayManaModel3();	//单显示--数量32号字体
	#define DisplayClrColor			0xFFFF		//显示区域清屏颜色
	#define DisplayBackColor		0xFFFF		//默认字体底色
	#define DisplayFontColor		0x0000		//默认字体颜色
	
	
	#define DisplayNameBkColor		0xFFFF		//名称底色
	#define DisplayNameFtColor		0x054A		//名称颜色
	#define	DisplayNameFtSize			24				//名称字体大小
	
	#define DisplayByNameBkColor	0xB71C		//别名底色
	#define DisplayByNameFtColor	0x094A		//别名颜色
	#define	DisplayByNameFtSize		16				//别名字体大小
	
	#define DisplaySpecBkColor		0xFFFF		//规格底色
	#define DisplaySpecFtColor		0x5ACA		//规格颜色
	#define	DisplaySpecFtSize			16				//规格字体大小
	
	#define DisplayNumBkColor			0x5902		//数量底色
	#define DisplayNumFtColor			0x07DF		//数量颜色
	#define	DisplayNumFtSize			32				//数量字体大小
	
	#define DisplaySerialBkColor	0xFFFF		//数量底色
	#define DisplaySerialFtColor	0xFB00		//数量颜色
	#define	DisplaySerialFtSize		16				//数量字体大小
	
	#define DisplayCodeBkColor		0xFFFF		//物料编码底色
	#define DisplayCodeFtColor		0x5ACA		//物料编码颜色
	#define	DisplayCodeFtSize			16				//物料编码字体大小
	
	#define DisplayVenderBkColor	0xFFFF		//厂家名称底色
	#define DisplayVenderFtColor	0x5ACA		//厂家名称颜色
	#define	DisplayVenderFtSize		16				//厂家名称字体大小
	
	#define DisplayTitleBkColor		0x865D		//标题底色
	#define DisplayTitleFtColor		0x0000		//标题颜色
	#define DisplayTitleSize			24				//标题字体大小
	
	#define DisplaySeparBkColor		0x8450		//分隔线颜色
	#define DisplaySeparWidth			2					//分隔线宽度
	
	#define DisplayTopStartY			48
	#define DisplayTopEndY				8	
	#define DisplayBotStartY			ST7789V_H-48
	#define DisplayBotEndY				ST7789V_H
	
	
	
	#define DisplayStartX	2
	#define DisplayEndX		ST7789V_V
#else			//DisplayManaModel1();
	#define DisplayClrColor				0xFFFF		//显示区域清屏颜色
	#define DisplayBackColor			0xFFFF		//默认字体底色
	#define DisplayFontColor			0x0000		//默认字体颜色
	#define DisplayFontSize				16				//默认字体大小
	
	
	#define DisplayNameBkColor		0xB71C		//名称底色
	#define DisplayNameFtColor		0x054A		//名称颜色
	#define	DisplayNameFtSize			24				//名称字体大小
	
	#define DisplayByNameBkColor	0xFFFF		//别名底色
	#define DisplayByNameFtColor	0x0000		//别名颜色
	#define	DisplayByNameFtSize		16				//别名字体大小
	
	#define DisplaySpecBkColor		0xFFFF		//规格底色
	#define DisplaySpecFtColor		0x0000		//规格颜色
	#define	DisplaySpecFtSize			16				//规格字体大小
	
	#define DisplayNumBkColor			0xFBC8		//数量底色
	#define DisplayNumFtColor			0x8981		//数量颜色
	#define	DisplayNumFtSize			32				//数量字体大小
	
	#define DisplaySerialBkColor	0xFFFF		//序号底色
	#define DisplaySerialFtColor	0xFB00		//序号颜色
	#define	DisplaySerialFtSize		16				//序号字体大小
	
	#define DisplayCodeBkColor		0xFFFF		//物料编码底色
	#define DisplayCodeFtColor		0x0000		//物料编码颜色
	#define	DisplayCodeFtSize			16				//物料编码字体大小
	
	#define DisplayVenderBkColor	0xFFFF		//厂家名称底色
	#define DisplayVenderFtColor	0x0000		//厂家名称颜色
	#define	DisplayVenderFtSize		16				//厂家名称字体大小
	
	#define DisplayTitleBkColor		0xfffF		//标题底色
	#define DisplayTitleFtColor		0x0000		//标题颜色
	#define DisplayTitleSize			16				//标题字体大小
	
	#define DisplaySeparBkColor		0x8450		//分隔线颜色
	#define DisplaySeparWidth			2							//分隔线宽度
	
	#define DisplayTopFillWidth		16				//顶端填充高度
	#define DisplayTopEndY				8	
	#define DisplayBotFillWidth		16				//底部填充高度
	#define DisplayBotEndY				ST7789V_H
	
	#define DisplayLeftFillWidth	9					//左边填充宽度/起始点	
	
	#define DisplayStartX	DisplayLeftFillWidth
	#define DisplayEndX						ST7789V_V
	
	
	#define ampLcdTopFillWidth		16				//顶端填充宽度
	#define ampLcdBotFillWidth		16				//底部填充高度
	#define ampLcdLeftFillWidth		5					//左边填充高度
	#define ampLcdRigthFillWidth	10				//右边填充宽度
	#define ampLcdTitleWidth			16				//标题栏宽度
	
#endif

#define	ampLcdListSize					5					//最大接收名字类型数量
#define	FresPageTime						2000			//页面刷新时间--ms
#define	AMPLcdMaxStringLen			256-9

//-----------------------------------LCD显示相关结构体
typedef enum _ampLcdParaTye						//类型：0x01-耗材名称，0x02-耗材规格，0x03-耗材数量,0x04-别名，0x05-厂家名称，0x06-耗材编码，0x07-数量单位
{
	ampLcdParaNone		=0x00,
	ampLcdParaName		=0x01,	//0x01-耗材名称
	ampLcdParaSpec		=0x02,	//0x02-耗材规格
	ampLcdParaNum			=0x03,	//0x03-耗材数量
	ampLcdParaByName	=0x04,	//0x04-别名
	ampLcdParaVender	=0x05,	//0x05-厂家名称
	ampLcdParaCode		=0x06,	//0x06-耗材编码
	ampLcdParaUnit		=0x07,	//0x07-数量单位
}ampLcdParaTyeDef;
//-----------------------------------应答消息
typedef struct _ampAck       //拨码数据：
{
  unsigned char head;     	//0x7E
  unsigned char length;   	//长度位不包含head,leng,crcl,crch,end长度	
	ampcmddef			cmd;				//命令/状态
	unsigned char address1;		//柜地址
  unsigned char address2;		//层地址
  unsigned char address3;		//位地址
	unsigned char status;			//状态-默认0
	unsigned char Crc16L;
	unsigned char Crc16H;
	unsigned char end;				//0x7F
	
	unsigned char flag;				//标志:0-无应答，1-需要上报应答
}ampAckDef;
//-----------------------------------LCD显示相关结构体
typedef struct _ampLcdMana						//耗材数据结构体，名称，规格，数量使用相同的结构体
{
	unsigned char		len;					//参数长度：仅指显示的字符串长度
	unsigned char		Offset;				//字符串地址偏移：不包含字符串类型和长度位
	unsigned short	XH;						//X--水平占用点数
	unsigned short	YV;						//Y--垂直占用点数
}ampLcdManaDef;
//---------------------------------------------------------------------
typedef struct _ampLcdDataNode			//显示数据结构体
{	
	ampLcdManaDef	Name;						//物品名称字符串参数
	ampLcdManaDef	ByName;					//别名字符串参数
	ampLcdManaDef	Spec;						//规格字符串参数
	ampLcdManaDef	Vender;					//厂家名称字符串参数
	ampLcdManaDef	Code;						//编码字符串参数
	ampLcdManaDef	Num;						//数量字符串参数
	ampLcdManaDef	Unit;						//单位字符串参数
	//----------------字符串数据
	unsigned char		Serial;				//条目编号/序号
	unsigned char		SumLen;				//总字符串长度
	unsigned short	SumPixelY;		//Y--垂直占用点数
	unsigned char		String[AMPLcdMaxStringLen];		//名字和规格字符串缓存，名字在前
}ampLcdDataNodeDef;










//-----------------------------------LCD显示相关结构体
typedef struct _Mana						//耗材数据结构体，名称，规格，数量使用相同的结构体
{
	ampLcdParaTyeDef	type;					//类型：0x01-耗材名称，0x02-耗材规格，0x03-耗材数量,0x04-别名，0x05-厂家名称，0x06-耗材编码，0x07-数量单位
	unsigned char			len;					//参数长度，String数据长度--hex字节单位
	unsigned char			String[AMPLcdMaxStringLen];		//字体缓存，如果是数量类型则存数值，如果是改背景色时，此为空，如果是名称或者规格时，存放字符数据GBK码
}ManaDef;
//---------------------------------------------------------------------
typedef struct _Para						//耗材数据结构体，名称，规格，数量使用相同的结构体
{
	unsigned char		len;					//参数长度--字符个数
	unsigned char		Offset;				//存储偏移地址--在String字符串中的起始地址	
	unsigned short	XH;						//X--水平占用点数
	unsigned short	YV;						//Y--垂直占用点数
}ParaDef;
//---------------------------------------------------------------------
typedef struct _ListData			//显示数据结构体
{	
	unsigned char		Serial;			//条目编号/序号
	ParaDef	ParaName;						//物品名称字符串参数
	ParaDef	ParaByName;					//别名字符串参数
	ParaDef	ParaSpec;						//规格字符串参数
	ParaDef	ParaVender;					//厂家名称字符串参数
	ParaDef	ParaCode;						//编码字符串参数
	ParaDef	ParaNum;						//数量字符串参数
	ParaDef	ParaUnit;						//单位字符串参数
	//----------------字符串数据
	
	unsigned char		len;				//字符串长度
	unsigned short	Ycount;			//Y--垂直占用点数
	unsigned char		String[AMPLcdMaxStringLen];		//名字和规格字符串缓存，名字在前
}ampLcdListDef;
//---------------------------------------------------------------------
typedef struct _Font					//字体：背景色，字体色，字体大小
{	
	unsigned short	BackColor;	//字体底色
	unsigned short	PenColor;		//字体颜色
	unsigned char		Size;				//字体大小16,24,32
}FontDef;
//---------------------------------------------------------------------
typedef struct _XYPointCount					//XY需要点数
{	
	unsigned short	XH;					//X--水平
	unsigned short	YV;					//Y--垂直
}PxyDef;
//---------------------------------------------------------------------
typedef struct _ampLcdAre					//XY需要点数
{	
	unsigned short	xs;					//X--水平起始点
	unsigned short	ys;					//Y--垂直起始点
	unsigned short	xe;					//X--水平终止点
	unsigned short	ye;					//Y--垂直终止点
}ampLcdAreDef;
//---------------------------------------------------------------------







typedef struct _ampLcdSysData		//系统数据
{
	unsigned char 	AddrCab;			//柜地址
	unsigned char 	AddrLay;			//层地址
	unsigned char 	AddrSeg;			//位地址
}ampLcdSysDataDef;
//---------------------------------------------------------------------

typedef struct _ampLcdHal	
{
	RS485Def 				RS485Port;   	//RS485通讯
	sST7789VDef			LcdPort;			//LCD
	GT32L32Def 			GT32L32;			//字库
	SwitchDef 			sSwitch;			//拔码开关
}ampLcdHalDef;
//---------------------------------------------------------------------

typedef struct _ampLcdComm			//通讯数据
{
	unsigned	char	Rxd[maxFramesize];	//接收
	unsigned	char	Txd[maxFramesize];	//发送
	unsigned	short TxLen;							//需要发送的数量
}ampLcdCommDef;
//---------------------------------------------------------------------

typedef struct _ampLcdCof				//初始化数据
{
	//----------------------------------字体参数
	FontDef	FtName;						//商品名称字体参数
	FontDef	FtByName;					//商品别名字体参数
	FontDef	FtSpec;						//商品规格字体参数
	FontDef	FtNum;						//商品数量字体参数
	FontDef	FtSeril;					//显示序号字体参数
	FontDef	FtCode;						//物料编码字体参数
	FontDef	FtVender;					//厂家名称字体参数
	FontDef	FtTitle;					//标题字体参数
	FontDef	FtSepar;					//分隔线参数
	FontDef	FtDefault;				//默认字体字体参数
	//----------------------------------显示类
	PxyDef	PxyPixel;					//总像素
	PxyDef	PxyTopFill;				//顶部填充高度
	PxyDef	PxyBotFill;				//底部填充高度
	PxyDef	PxyLeftFill;			//左边填充宽度
	PxyDef	PxyValid;					//有效显示区域
	
	ampLcdAreDef	AreMain;		//主显示区域:显示耗材区域
	ampLcdAreDef	AreTitle;		//标题显示区域
}ampLcdCofDef;
//---------------------------------------------------------------------
typedef struct _ampLcdDisplayData						//显示数据结构体
{
	ampLcdListDef	List[ampLcdListSize];	//显示内容
	
	ampLcdDataNodeDef	Node[ampLcdListSize];	//显示内容

	
	unsigned char		Serial;			//当前显示序号
	unsigned char		Count;			//缓存使用个数
	unsigned char		Sync;				//0-不需要同步，1-需要同步
	unsigned short	StartY;			//当前显示Y起始点
	
	ampLcdAreDef	FillStart;								//显示前清除显示起始坐标
	ampLcdAreDef	FillEnd;									//显示前清除显示结束坐标
	
	ampLcdAreDef	ShowStart;								//字符显示起始坐标
	ampLcdAreDef	ShowEnd;									//字符显示结束坐标
	
}ampLcdDisplayDataDef;
//---------------------------------------------------------------------
typedef struct _ampLcdRun						//运行数据
{
	//----------------------------------运行数据
	PxyDef	PxyFillStart;							//显示前清除显示起始坐标
	PxyDef	PxyFillEnd;								//显示前清除显示结束坐标

	PxyDef	PixelStart;								//字符显示起始坐标
	PxyDef	PixelEnd;									//字符显示结束坐标
	
	unsigned short	TopDisplayStartY;	//上半面字符显示起始地址
	unsigned short	BotDisplayStartY;	//下半页字符显示起始地址
	unsigned short	StartPixelY;			//当前显示时的Y起始点
}ampLcdRunDef;
//---------------------------------------------------------------------

//---------------------------------------------------------------------
typedef struct _LcdData	//数据
{
	ampLcdCommDef					CommData;
	ampLcdSysDataDef			SysData;
	ampLcdRunDef					RunData;
	ampLcdDisplayDataDef	Display;
	const ampLcdCofDef		Cof;				//配置数据
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
void ampLcd_data_process(unsigned char* pBuffer,unsigned short len);		//协议、地址检查
void ampLcd_data_extract(unsigned char* String,unsigned short len);		//提取字符串数据
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
static void DataInitialize(void);		//数据初始化

static void DisplayString(void);
static void DisplayGui(void);				//显示界面
static void DisplayName(const ampLcdListDef Node);				//显示别名和名称
static void DisplayByName(const ampLcdListDef Node);			//显示别名
static void DisplayVender(const ampLcdListDef Node);			//显示厂家名称
static void DisplaySpec(const ampLcdListDef Node);				//显示规格
static void DisplayCode(const ampLcdListDef Node);				//显示编码
static void DisplayNumber(const ampLcdListDef Node);			//显示数量和单位
static void DisplayTitle(unsigned char DualFlag);			//显示标题
static unsigned short GetYVLen(const ampLcdListDef Node);//获取节点数据占用Y轴的点数

static void GetManaData(const unsigned char* Databuffer,unsigned short datalen);
static void SetManaData(ampLcdListDef* pNode);		//设置显示参数---数据获取成功后设置相关的显示参数


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
//=================================时间函数ST==============================================================






#endif
