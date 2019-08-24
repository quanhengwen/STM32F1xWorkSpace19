#ifdef AMPLCDV12A2

//===========波特率9600
//===========A2波特率115200
#include "AMPLCDV12A2.H"

#include	"AMP_Protocol.H"



#include "STM32_SYS.H"
#include "STM32_GPIO.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_TIM.H"
#include "STM32_FLASH.H"

#include "LCD.H"
#include "ST7789V.H"
#include "GT32L32M0180.H"

#include "font.h"


//------------------运行指示灯
#define ampLcdSYSLEDPort    				GPIOA
#define ampLcdSYSLEDPin     				GPIO_Pin_0
//------------------通讯接口--层接口
#define ampLcdCommPort       				USART1
#define ampLcdCommTxEnPort  				GPIOA
#define ampLcdCommTxEnPin   				GPIO_Pin_12
#define ampLcdCommRxEnPort 					GPIOA
#define ampLcdCommRxEnPin						GPIO_Pin_11
#define ampLcdCommBaudRate        	115200
#define ampLcdCommDelayTime        	90			//每字节传输的时间us
#define ampLcdCommMaxAddr        		6				//最大地址


/* Private variables ---------------------------------------------------------*/
sAmpLcdDef	sAmpLcd;



unsigned	char	DaulFlag	=	0;
/* Private function prototypes -----------------------------------------------*/

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void AMPLCDV12A2_Configuration(void)
{	
	
	SYS_Configuration();				//系统配置
	
  HW_Configuration(); 

	DataInitialize();		
	
  //IWDG_Configuration(2000);													//独立看门狗配置---参数单位ms

  SysTick_Configuration(1000);    //系统嘀嗒时钟配置72MHz,单位为uS  
  while(1)
  {
		ReceiveData();
		SendData();
  }
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void AMPLCDV12A2_Server(void)
{  
	IWDG_Feed();								//独立看门狗喂狗
  SysLed_server();
	AddressNoneProcess();
	Lcd_Process();
	SwitchID_Server();
	NoDataProcess();	 
}
//------------------------------------------------------------------------------



/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void SysLed_server(void)
{
	static unsigned short time=0;
	if(time++>100)
	{
		time=	0;
		api_gpio_toggle(ampLcdSYSLEDPort,ampLcdSYSLEDPin);		//将GPIO相应管脚输出翻转----V20170605
	}
	
}
//------------------------------------------------------------------------------




/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void Lcd_Process(void)
{
	static unsigned short time=0;
	static unsigned char	Type_Count_bac=0;
//	static unsigned char color=0;
	unsigned char i=0;
	unsigned char dataflag=0;
	ampLcdListDef*		List;
	
	List	=	sAmpLcd.Data.Display.List;
	if(0==sAmpLcd.Data.SysData.AddrLay||0==sAmpLcd.Data.SysData.AddrSeg)
	{
		return;
	}
	for(i=0;i<ampLcdListSize;i++)
	{
		if(0!=List[i].ListNum)
		{
			dataflag	=	1;
			break;
		}
	}
	if(Type_Count_bac!=sAmpLcd.Data.Display.Count)
	{
		time	=	0;
		sAmpLcd.Data.Display.Serial	=	0;
		Type_Count_bac=sAmpLcd.Data.Display.Count;
	}
	
	if(0==dataflag)
		return;
	if(time==0)
  {
		if(sAmpLcd.Data.Display.Count<=2)
		{
			if((sAmpLcd.Data.Display.Serial==DaulFlag)&&(sAmpLcd.Data.Display.Serial==sAmpLcd.Data.Display.Count))
			goto FlashTimeCon;
		}
			DisplayString();
  }
	FlashTimeCon:
	if(time++>3000)
		time=0; 
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void Lcd_Processbac(void)
{
	static unsigned short time=0;
	static unsigned char	Type_Count_bac=0;
//	static unsigned char color=0;
	unsigned char i=0;
	unsigned char dataflag=0;
	ampLcdListDef*		List;
	
	List	=	sAmpLcd.Data.Display.List;
	if(0==sAmpLcd.Data.SysData.AddrLay||0==sAmpLcd.Data.SysData.AddrSeg)
	{
		return;
	}
	for(i=0;i<ampLcdListSize;i++)
	{
		if(0!=List[i].ListNum)
		{
			dataflag	=	1;
			break;
		}
	}
	if(Type_Count_bac!=sAmpLcd.Data.Display.Count)
	{
		time	=	0;
		sAmpLcd.Data.Display.Serial	=	0;
		Type_Count_bac=sAmpLcd.Data.Display.Count;
	}
	
	if(0==dataflag)
		return;
	if(time==0)
  {
		if(sAmpLcd.Data.Display.Count<=2)
		{
			if((sAmpLcd.Data.Display.Serial==DaulFlag)&&(sAmpLcd.Data.Display.Serial==sAmpLcd.Data.Display.Count))
			goto FlashTimeCon;
		}
			DisplayString();
  }
	FlashTimeCon:
	if(time++>3000)
		time=0; 
}
//------------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void AddressNoneProcess(void)
{
	static unsigned short time=0;
	static unsigned char flag=0;
	unsigned short color	=	0;
	if(0==sAmpLcd.Data.SysData.AddrLay||0==sAmpLcd.Data.SysData.AddrSeg)
	{
		if(time++>1000)
		{
			time	=	0;
			flag+=1;
			if(flag++>5)
			{
				flag=0;
			}
			if(0==flag)
			{
				color	=	LCD565_GBLUE;
			}
			else if(1==flag)
			{
				color	=	LCD565_GRAY;
			}
			else if(2==flag)
			{
				color	=	LCD565_RED;
			}
			else if(3==flag)
			{
				color	=	LCD565_LIGHTGREEN;
			}
			else if(4==flag)
			{
				color	=	LCD565_GBLUE;
			}
			else
			{
				color	=	LCD565_WHITE;
			}
			ST7789V_Clean(color);	//清除屏幕函数;
			ST7789V_Printf(5,100,32,color^0xFF,"未拨码");				//后边的省略号就是可变参数
			ST7789V_BL_ON;
		}
	}
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void NoDataProcess(void)
{
	static unsigned short time=0;
	static unsigned char flag=0;
	static unsigned char power_up_flag=0;
	unsigned short color	=	0;
	
	unsigned char i=0;
	unsigned char dataflag=0;
	ampLcdListDef*		List;
	
	List	=	sAmpLcd.Data.Display.List;
	
	if(time++<100)
	{
		return;
	}
	time	=	0;
	if((0!=sAmpLcd.Data.SysData.AddrLay)&&(0!=sAmpLcd.Data.SysData.AddrSeg))
	{
		for(i=0;i<ampLcdListSize;i++)
		{
			if(0!=List[i].ListNum)
			{
				dataflag	=	1;
				break;
			}
		}
		if(0!=dataflag)
		{
			dataflag	=	0;
			ST7789V_BL_ON;
			return;
		}
		else
		{
			ST7789V_BL_OFF;
		}		
	}
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void DisplayString(void)
{
	unsigned char i	=	0;
	unsigned char Serial	=	0;	
	//unsigned char MaxNameList=0;
	unsigned short StartPixelY	=	0;
	unsigned short	YVLen	=	0;
	
	ampLcdListDef 		Node;
	ampLcdListDef*		List;
	
//	WinInfoDef*	WinInfo;			//显示字体信息
	
	//=================================初始化数据
//	WinInfo	=	(WinInfoDef*)&sAmpLcd.Windows.WinInfo;							//显示参数信息
	
	//MaxNameList	=	sAmpLcd.Data.DisplayData.MaxNameList;
	List	=	sAmpLcd.Data.Display.List;	
	
	DisplayGui();				//显示界面
	
	if(0==sAmpLcd.Data.Display.Count)	//无数据
	{
		goto DisplayTitleStart;
	}
	else if(0==sAmpLcd.Data.Display.Serial)
	{
		sAmpLcd.Data.Display.Serial	=	1;
	}
	else if(sAmpLcd.Data.Display.Serial>sAmpLcd.Data.Display.Count)
	{
		sAmpLcd.Data.Display.Serial	=	1;
	}
	//------------------------------------------设置Y起始点
	sAmpLcd.Data.RunData.StartPixelY	=sAmpLcd.Data.RunData.TopDisplayStartY;
	
	
	CheckDataStart:
	
	Serial=sAmpLcd.Data.Display.Serial;	
	for(i=0;i<ampLcdListSize;i++)
	{
		if(Serial==List[i].ListNum)
		{
			Node	=	List[i];
			break;
		}		
	}
	if(i>=ampLcdListSize)
	{
		sAmpLcd.Data.Display.Serial=1;
		return;
	}
	DisplayName(Node);			
	DisplayByName(Node);
	DisplayVender(Node);
	DisplaySpec(Node);
	DisplayCode(Node);
	DisplayNumber(Node);
	DaulFlag	=	1;
		//--------------------------------------------------
	if(sAmpLcd.Data.RunData.StartPixelY>sAmpLcd.Data.RunData.BotDisplayStartY)	//下半页没有足够空间显示
	{
		DisplayTitle(1);			//显示标题
		if(sAmpLcd.Data.Display.Serial<sAmpLcd.Data.Display.Count)
		{
			sAmpLcd.Data.Display.Serial+=1;
		}
		else
		{
			sAmpLcd.Data.Display.Serial=1;
		}
		return;
	}
	//----------------------------------------------------还有内容待显示
	if(sAmpLcd.Data.Display.Serial<sAmpLcd.Data.Display.Count)
	{
		sAmpLcd.Data.Display.Serial+=1;
	}
	else
	{
		DisplayTitle(1);			//显示标题
		sAmpLcd.Data.Display.Serial=1;
		return;		
	}
	//=================================下半页显示
	sAmpLcd.Data.RunData.StartPixelY	=	sAmpLcd.Data.RunData.BotDisplayStartY;			//下半页显示
	Serial=sAmpLcd.Data.Display.Serial;	
	for(i=0;i<ampLcdListSize;i++)
	{
		if(Serial==List[i].ListNum)
		{
			Node	=	List[i];
			break;
		}		
	}
	if(i>=ampLcdListSize)
	{
		sAmpLcd.Data.Display.Serial=1;
		return;
	}
	
	YVLen	=	GetYVLen(Node);//获取节点数据占用Y轴的点数
	
	if(YVLen>(sAmpLcd.Data.RunData.BotDisplayStartY-sAmpLcd.Data.RunData.TopDisplayStartY))
	{
		sAmpLcd.Data.Display.Serial-=1;		//恢复上次成功显示序号
		DisplayTitle(1);			//显示标题
		sAmpLcd.Data.Display.Serial+=1;		//恢复下次显示序号
		return;
	}
	DisplayName(Node);			
	DisplayByName(Node);
	DisplayVender(Node);
	DisplaySpec(Node);
	DisplayCode(Node);
	DisplayNumber(Node);
	
	DisplayTitle(2);			//显示标题
	DaulFlag	=	2;
	if(sAmpLcd.Data.Display.Count<=2)
	{
	}
	else
	{
		if(sAmpLcd.Data.Display.Serial<sAmpLcd.Data.Display.Count)
		{
			sAmpLcd.Data.Display.Serial+=1;		//下次显示序号
		}
		else
		{
			sAmpLcd.Data.Display.Serial=1;		//下次显示序号
		}
	}	
	return;
	
	DisplayTitleStart:
	DisplayTitle(0);			//显示标题
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void DisplayGui(void)
{
	unsigned short xs=0;
	unsigned short ys=0;
	unsigned short xe=0;
	unsigned short ye=0;

	unsigned short BackColor;	//背景色
	
//	WinInfoDef*	WinInfo;			//显示字体信息
	//=================================初始化数据
//	WinInfo	=	(WinInfoDef*)&sAmpLcd.Windows.WinInfo;							//显示参数信息
	//-----------------------------------------------------顶端边框
	xs	=	0;
	xe	=	sAmpLcd.Data.Cof.PxyPixel.XH-1;
	ys	=	0;
	ye	=	sAmpLcd.Data.Cof.PxyTopFill.YV-1;
	BackColor	=	sAmpLcd.Data.Cof.FtTitle.BackColor;
	ST7789V_Fill(xs,ys,xe,ye,0x0000);	//纯黑填充
	//-----------------------------------------------------底部边框
	xs	=	0;
	xe	=	sAmpLcd.Data.Cof.PxyPixel.XH-1;
	ys	=	sAmpLcd.Data.Cof.PxyPixel.YV-sAmpLcd.Data.Cof.PxyBotFill.YV-1;
	ye	=	sAmpLcd.Data.Cof.PxyPixel.YV-1;
	BackColor	=	sAmpLcd.Data.Cof.FtTitle.BackColor;
	ST7789V_Fill(xs,ys,xe,ye,0x0000);	//纯黑填充
	//-----------------------------------------------------左边边框
	xs	=	0;
	xe	=	sAmpLcd.Data.Cof.PxyLeftFill.XH-1;
	ys	=	0;
	ye	=	sAmpLcd.Data.Cof.PxyPixel.YV-1;
	BackColor	=	sAmpLcd.Data.Cof.FtTitle.BackColor;
	ST7789V_Fill(xs,ys,xe,ye,0x0000);	//纯黑填充
	//-----------------------------------------------------右边边框
	xs	=	319-ST7789V_V;
	xe	=	319;
	ys	=	0;
	ye	=	sAmpLcd.Data.Cof.PxyPixel.YV-1;
	BackColor	=	sAmpLcd.Data.Cof.FtTitle.BackColor;
	ST7789V_Fill(xs,ys,xe,ye,0x0000);	//纯黑填充
	//-----------------------------------------------------清空显示区
	xs	=	sAmpLcd.Data.Cof.PxyLeftFill.XH;
	xe	=	sAmpLcd.Data.Cof.PxyPixel.XH;
	ys	=	sAmpLcd.Data.Cof.PxyTopFill.YV;
	ye	=	sAmpLcd.Data.Cof.PxyPixel.YV-sAmpLcd.Data.Cof.PxyBotFill.YV-2;
	BackColor	=	sAmpLcd.Data.Cof.FtDefault.BackColor;
	ST7789V_Fill(xs,ys,xe,ye,BackColor);	//默认背景色填充
	
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void DisplayName(const ampLcdListDef Node)
{
	unsigned char		strLen		=	0;		//字符串长度
	unsigned char 	offset=0;
	unsigned char		str[256];		//目标符串地址---存储
	
	unsigned short xs=0;	//字体显示坐标信息
	unsigned short ys=0;
	unsigned short xe=0;
	unsigned short ye=0;
	
	unsigned short Fxs=0;	//填充背景色坐标信息
	unsigned short Fys=0;
	unsigned short Fxe=0;
	unsigned short Fye=0;
	
	unsigned short BackColor;	//背景色
	unsigned short PenColor;	//画笔色
	unsigned short FontSize;	//字体大小
	
//	WinInfoDef*	WinInfo;			//显示字体信息
	ParaDef			Para;
	FontDef			Font;
	//=================================初始化数据
//	WinInfo	=	(WinInfoDef*)&sAmpLcd.Windows.WinInfo;							//显示参数信息
	Font		=	sAmpLcd.Data.Cof.FtName;
	Para		=	Node.ParaName;
	//=================================数据内容检查
	strLen		=	Para.len;
	if(0==strLen)
	{
		return;
	}
	//=================================复制数据
	offset		=	Para.Offset;
	memcpy(str,&Node.String[offset],strLen);
	//=================================准备数据
	BackColor	=	Font.BackColor;
	PenColor	=	Font.PenColor;
	FontSize	=	Font.Size;
	//=================================清除显示区域
	sAmpLcd.Data.RunData.PxyFillStart.XH		=	sAmpLcd.Data.Cof.PxyLeftFill.XH;
	sAmpLcd.Data.RunData.PxyFillStart.YV		=	sAmpLcd.Data.RunData.StartPixelY+1;
	sAmpLcd.Data.RunData.PxyFillEnd.XH	=	sAmpLcd.Data.RunData.PxyFillStart.XH+Para.XH;
	sAmpLcd.Data.RunData.PxyFillEnd.YV	=	sAmpLcd.Data.RunData.PxyFillStart.YV+Para.YV;
	
	xs	=	sAmpLcd.Data.RunData.PxyFillStart.XH;
	ys	=	sAmpLcd.Data.RunData.PxyFillStart.YV;
	xe	=	sAmpLcd.Data.RunData.PxyFillEnd.XH;
	ye	=	sAmpLcd.Data.RunData.PxyFillEnd.YV;
	ST7789V_Fill(xs,ys,xe,ye,BackColor);	//背景色填充/擦除/清除
	//=================================设定显示区域
	if(Para.YV	==	FontSize)			//单行居中显示
	{
		unsigned short tempx=0;
		tempx	=	strLen*(FontSize/2);				//实际显示占用的点数
		tempx	=	Para.XH	-	tempx;	//剩余空闲点数
		
		sAmpLcd.Data.RunData.PxyDisplayStart.XH	=	sAmpLcd.Data.RunData.PxyFillStart.XH+tempx/2;
		sAmpLcd.Data.RunData.PxyDisplayEnd.XH		=	sAmpLcd.Data.RunData.PxyDisplayStart.XH+strLen*(FontSize/2);
	}
	else
	{
		sAmpLcd.Data.RunData.PxyDisplayStart.XH	=	sAmpLcd.Data.RunData.PxyFillStart.XH;
		sAmpLcd.Data.RunData.PxyDisplayEnd.XH		=	sAmpLcd.Data.RunData.PxyDisplayStart.XH+Para.XH;
	}
	sAmpLcd.Data.RunData.PxyDisplayStart.YV		=	sAmpLcd.Data.RunData.PxyFillStart.YV;
	sAmpLcd.Data.RunData.PxyDisplayEnd.YV			=	sAmpLcd.Data.RunData.PxyDisplayStart.YV+Para.YV;
	//=================================显示数据
	xs	=	sAmpLcd.Data.RunData.PxyDisplayStart.XH;
	ys	=	sAmpLcd.Data.RunData.PxyDisplayStart.YV;
	xe	=	sAmpLcd.Data.RunData.PxyDisplayEnd.XH;
	ye	=	sAmpLcd.Data.RunData.PxyDisplayEnd.YV;
	ST7789V_ShowStringBKAre(xs,ys,xe,ye,FontSize,BackColor,PenColor,strLen,str);	//带背景色限定区域显示
	//-------------------------------------准备下一项内容显示参数
	sAmpLcd.Data.RunData.StartPixelY					=	sAmpLcd.Data.RunData.PxyFillEnd.YV;
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void DisplayByName(const ampLcdListDef Node)
{
	unsigned char		strLen		=	0;		//字符串长度
	unsigned char 	offset=0;
	unsigned char		str[256];		//目标符串地址---存储
	unsigned char 	Fname[]	=	"别名:";
	unsigned char 	Flen	=	strlen(Fname);
	
	unsigned short xs=0;	//字体显示坐标信息
	unsigned short ys=0;
	unsigned short xe=0;
	unsigned short ye=0;
	
	unsigned short Fxs=0;	//填充背景色坐标信息
	unsigned short Fys=0;
	unsigned short Fxe=0;
	unsigned short Fye=0;
//	
	unsigned short BackColor;	//背景色
	unsigned short PenColor;	//画笔色
	unsigned short FontSize;	//字体大小
	
//	WinInfoDef*	WinInfo;			//显示字体信息
	ParaDef			Para;
	FontDef			Font;
	//=================================初始化数据
//	WinInfo	=	(WinInfoDef*)&sAmpLcd.Windows.WinInfo;							//显示参数信息
	Font		=	sAmpLcd.Data.Cof.FtByName;
	Para		=	Node.ParaByName;
	//=================================数据内容检查
	strLen		=	Para.len;
	if(0==strLen)
	{
		return;
	}
	//=================================复制数据"别名:"
	memcpy(str,Fname,Flen);
	offset		=	Para.Offset;
	memcpy(&str[Flen],&Node.String[offset],strLen);
	strLen	=	strLen+Flen;
	//=================================准备数据
	BackColor	=	Font.BackColor;
	PenColor	=	Font.PenColor;
	FontSize	=	Font.Size;
	//=================================清除显示区域
	sAmpLcd.Data.RunData.PxyFillStart.XH			=	sAmpLcd.Data.Cof.PxyLeftFill.XH;
	sAmpLcd.Data.RunData.PxyFillStart.YV			=	sAmpLcd.Data.RunData.StartPixelY+1;
	sAmpLcd.Data.RunData.PxyFillEnd.XH		=	sAmpLcd.Data.RunData.PxyFillStart.XH+Para.XH;
	sAmpLcd.Data.RunData.PxyFillEnd.YV		=	sAmpLcd.Data.RunData.PxyFillStart.YV+Para.YV;
	xs	=	sAmpLcd.Data.RunData.PxyFillStart.XH;
	ys	=	sAmpLcd.Data.RunData.PxyFillStart.YV;
	xe	=	sAmpLcd.Data.RunData.PxyFillEnd.XH;
	ye	=	sAmpLcd.Data.RunData.PxyFillEnd.YV;
	ST7789V_Fill(xs,ys,xe,ye,BackColor);	//背景色填充/擦除/清除	
	//=================================设定显示区域
	sAmpLcd.Data.RunData.PxyDisplayStart.XH		=	sAmpLcd.Data.RunData.PxyFillStart.XH;
	sAmpLcd.Data.RunData.PxyDisplayEnd.XH			=	sAmpLcd.Data.RunData.PxyDisplayStart.XH+Para.XH;
	sAmpLcd.Data.RunData.PxyDisplayStart.YV		=	sAmpLcd.Data.RunData.PxyFillStart.YV;
	sAmpLcd.Data.RunData.PxyDisplayEnd.YV			=	sAmpLcd.Data.RunData.PxyDisplayStart.YV+Para.YV;
	//=================================显示数据
	xs	=	sAmpLcd.Data.RunData.PxyDisplayStart.XH;
	ys	=	sAmpLcd.Data.RunData.PxyDisplayStart.YV;
	xe	=	sAmpLcd.Data.RunData.PxyDisplayEnd.XH;
	ye	=	sAmpLcd.Data.RunData.PxyDisplayEnd.YV;
	ST7789V_ShowStringBKAre(xs,ys,xe,ye,FontSize,BackColor,PenColor,strLen,str);	//带背景色限定区域显示
	//-------------------------------------准备下一项内容显示参数
	sAmpLcd.Data.RunData.StartPixelY	=	sAmpLcd.Data.RunData.PxyFillEnd.YV;
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void DisplayVender(const ampLcdListDef Node)
{
	unsigned char		strLen	=	0;		//字符串长度
	unsigned char 	offset=0;
	unsigned char		str[256];		//目标符串地址---存储
	unsigned char 	Fname[]	=	"厂家:";
	unsigned char 	Flen	=	strlen(Fname);
	
	unsigned short xs=0;	//字体显示坐标信息
	unsigned short ys=0;
	unsigned short xe=0;
	unsigned short ye=0;
	
	unsigned short Fxs=0;	//填充背景色坐标信息
	unsigned short Fys=0;
	unsigned short Fxe=0;
	unsigned short Fye=0;
//	
	unsigned short BackColor;	//背景色
	unsigned short PenColor;	//画笔色
	unsigned short FontSize;	//字体大小
	
//	WinInfoDef*	WinInfo;			//显示字体信息
	ParaDef			Para;
	FontDef			Font;
	//=================================初始化数据
//	WinInfo	=	(WinInfoDef*)&sAmpLcd.Windows.WinInfo;							//显示参数信息
	Font		=	sAmpLcd.Data.Cof.FtVender;
	Para		=	Node.ParaVender;
	//=================================数据内容检查
	strLen		=	Para.len;
	if(0==strLen)
	{
		return;
	}
	//=================================复制数据
	memcpy(str,Fname,Flen);
	offset		=	Para.Offset;
	memcpy(&str[Flen],&Node.String[offset],strLen);
	strLen	=	strLen+Flen;
	//=================================准备数据
	BackColor	=	Font.BackColor;
	PenColor	=	Font.PenColor;
	FontSize	=	Font.Size;
	//=================================清除显示区域
	sAmpLcd.Data.RunData.PxyFillStart.XH	=	sAmpLcd.Data.Cof.PxyLeftFill.XH;
	sAmpLcd.Data.RunData.PxyFillStart.YV	=	sAmpLcd.Data.RunData.StartPixelY+1;
	sAmpLcd.Data.RunData.PxyFillEnd.XH		=	sAmpLcd.Data.RunData.PxyFillStart.XH+Para.XH;
	sAmpLcd.Data.RunData.PxyFillEnd.YV		=	sAmpLcd.Data.RunData.PxyFillStart.YV+Para.YV;
	xs	=	sAmpLcd.Data.RunData.PxyFillStart.XH;
	ys	=	sAmpLcd.Data.RunData.PxyFillStart.YV;
	xe	=	sAmpLcd.Data.RunData.PxyFillEnd.XH;
	ye	=	sAmpLcd.Data.RunData.PxyFillEnd.YV;
	ST7789V_Fill(xs,ys,xe,ye,BackColor);	//背景色填充/擦除/清除	
	
	//=================================设定显示区域

	sAmpLcd.Data.RunData.PxyDisplayStart.XH	=	sAmpLcd.Data.RunData.PxyFillStart.XH;
	sAmpLcd.Data.RunData.PxyDisplayEnd.XH		=	sAmpLcd.Data.RunData.PxyDisplayStart.XH+Para.XH;

	sAmpLcd.Data.RunData.PxyDisplayStart.YV		=	sAmpLcd.Data.RunData.PxyFillStart.YV;
	sAmpLcd.Data.RunData.PxyDisplayEnd.YV			=	sAmpLcd.Data.RunData.PxyDisplayStart.YV+Para.YV;
	//=================================显示数据
	xs	=	sAmpLcd.Data.RunData.PxyDisplayStart.XH;
	ys	=	sAmpLcd.Data.RunData.PxyDisplayStart.YV;
	xe	=	sAmpLcd.Data.RunData.PxyDisplayEnd.XH;
	ye	=	sAmpLcd.Data.RunData.PxyDisplayEnd.YV;
	ST7789V_ShowStringBKAre(xs,ys,xe,ye,FontSize,BackColor,PenColor,strLen,str);	//带背景色限定区域显示
	//-------------------------------------准备下一项内容显示参数
	sAmpLcd.Data.RunData.StartPixelY	=	sAmpLcd.Data.RunData.PxyFillEnd.YV;
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void DisplaySpec(const ampLcdListDef Node)
{
	unsigned char		strLen	=	0;		//字符串长度
	unsigned char 	offset=0;
	unsigned char		str[256];		//目标符串地址---存储
	unsigned char 	Fname[]	=	"规格:";
	unsigned char 	Flen		=	strlen(Fname);
	
	unsigned short xs=0;	//字体显示坐标信息
	unsigned short ys=0;
	unsigned short xe=0;
	unsigned short ye=0;
	
	unsigned short Fxs=0;	//填充背景色坐标信息
	unsigned short Fys=0;
	unsigned short Fxe=0;
	unsigned short Fye=0;
//	
	unsigned short BackColor;	//背景色
	unsigned short PenColor;	//画笔色
	unsigned short FontSize;	//字体大小
	
//	WinInfoDef*	WinInfo;			//显示字体信息
	ParaDef			Para;
	FontDef			Font;
	//=================================初始化数据
//	WinInfo	=	(WinInfoDef*)&sAmpLcd.Windows.WinInfo;							//显示参数信息
	Font		=	sAmpLcd.Data.Cof.FtSpec;
	Para		=	Node.ParaSpec;
	//=================================数据内容检查
	strLen		=	Para.len;
	if(0==strLen)
	{
		return;
	}
	//=================================复制数据
	memcpy(str,Fname,Flen);
	offset		=	Para.Offset;
	memcpy(&str[Flen],&Node.String[offset],strLen);
	strLen	=	strLen+Flen;
	//=================================准备数据
	BackColor	=	Font.BackColor;
	PenColor	=	Font.PenColor;
	FontSize	=	Font.Size;
	//=================================清除显示区域
	sAmpLcd.Data.RunData.PxyFillStart.XH	=	sAmpLcd.Data.Cof.PxyLeftFill.XH;
	sAmpLcd.Data.RunData.PxyFillStart.YV	=	sAmpLcd.Data.RunData.StartPixelY+1;
	sAmpLcd.Data.RunData.PxyFillEnd.XH		=	sAmpLcd.Data.RunData.PxyFillStart.XH+Para.XH;
	sAmpLcd.Data.RunData.PxyFillEnd.YV		=	sAmpLcd.Data.RunData.PxyFillStart.YV+Para.YV;
	xs	=	sAmpLcd.Data.RunData.PxyFillStart.XH;
	ys	=	sAmpLcd.Data.RunData.PxyFillStart.YV;
	xe	=	sAmpLcd.Data.RunData.PxyFillEnd.XH;
	ye	=	sAmpLcd.Data.RunData.PxyFillEnd.YV;
	ST7789V_Fill(xs,ys,xe,ye,BackColor);	//背景色填充/擦除/清除	
	
	//=================================设定显示区域

	sAmpLcd.Data.RunData.PxyDisplayStart.XH	=	sAmpLcd.Data.RunData.PxyFillStart.XH;
	sAmpLcd.Data.RunData.PxyDisplayEnd.XH		=	sAmpLcd.Data.RunData.PxyDisplayStart.XH+Para.XH;

	sAmpLcd.Data.RunData.PxyDisplayStart.YV		=	sAmpLcd.Data.RunData.PxyFillStart.YV;
	sAmpLcd.Data.RunData.PxyDisplayEnd.YV			=	sAmpLcd.Data.RunData.PxyDisplayStart.YV+Para.YV;
	//=================================显示数据
	xs	=	sAmpLcd.Data.RunData.PxyDisplayStart.XH;
	ys	=	sAmpLcd.Data.RunData.PxyDisplayStart.YV;
	xe	=	sAmpLcd.Data.RunData.PxyDisplayEnd.XH;
	ye	=	sAmpLcd.Data.RunData.PxyDisplayEnd.YV;
	ST7789V_ShowStringBKAre(xs,ys,xe,ye,FontSize,BackColor,PenColor,strLen,str);	//带背景色限定区域显示
	//-------------------------------------准备下一项内容显示参数
	sAmpLcd.Data.RunData.StartPixelY	=	sAmpLcd.Data.RunData.PxyFillEnd.YV;
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void DisplayCode(const ampLcdListDef Node)
{
	unsigned char		strLen	=	0;		//字符串长度
	unsigned char 	offset=0;
	unsigned char		str[256];		//目标符串地址---存储
	unsigned char 	Fname[]	=	"编码:";
	unsigned char 	Flen		=	strlen(Fname);
	
	unsigned short xs=0;	//字体显示坐标信息
	unsigned short ys=0;
	unsigned short xe=0;
	unsigned short ye=0;
	
	unsigned short Fxs=0;	//填充背景色坐标信息
	unsigned short Fys=0;
	unsigned short Fxe=0;
	unsigned short Fye=0;
//	
	unsigned short BackColor;	//背景色
	unsigned short PenColor;	//画笔色
	unsigned short FontSize;	//字体大小
	
//	WinInfoDef*	WinInfo;			//显示字体信息
	ParaDef			Para;
	FontDef			Font;
	//=================================初始化数据
//	WinInfo	=	(WinInfoDef*)&sAmpLcd.Windows.WinInfo;							//显示参数信息
	Font		=	sAmpLcd.Data.Cof.FtCode;
	Para		=	Node.ParaCode;
	//=================================数据内容检查
	strLen		=	Para.len;
	if(0==strLen)
	{
		return;
	}
	//=================================复制数据
	memcpy(str,Fname,Flen);
	offset		=	Para.Offset;
	memcpy(&str[Flen],&Node.String[offset],strLen);
	strLen	=	strLen+Flen;
	//=================================准备数据
	BackColor	=	Font.BackColor;
	PenColor	=	Font.PenColor;
	FontSize	=	Font.Size;
	//=================================清除显示区域
	sAmpLcd.Data.RunData.PxyFillStart.XH	=	sAmpLcd.Data.Cof.PxyLeftFill.XH;
	sAmpLcd.Data.RunData.PxyFillStart.YV	=	sAmpLcd.Data.RunData.StartPixelY+1;
	sAmpLcd.Data.RunData.PxyFillEnd.XH		=	sAmpLcd.Data.RunData.PxyFillStart.XH+Para.XH;
	sAmpLcd.Data.RunData.PxyFillEnd.YV		=	sAmpLcd.Data.RunData.PxyFillStart.YV+Para.YV;
	xs	=	sAmpLcd.Data.RunData.PxyFillStart.XH;
	ys	=	sAmpLcd.Data.RunData.PxyFillStart.YV;
	xe	=	sAmpLcd.Data.RunData.PxyFillEnd.XH;
	ye	=	sAmpLcd.Data.RunData.PxyFillEnd.YV;
	ST7789V_Fill(xs,ys,xe,ye,BackColor);	//背景色填充/擦除/清除	
	
	//=================================设定显示区域

	sAmpLcd.Data.RunData.PxyDisplayStart.XH	=	sAmpLcd.Data.RunData.PxyFillStart.XH;
	sAmpLcd.Data.RunData.PxyDisplayEnd.XH		=	sAmpLcd.Data.RunData.PxyDisplayStart.XH+Para.XH;

	sAmpLcd.Data.RunData.PxyDisplayStart.YV	=	sAmpLcd.Data.RunData.PxyFillStart.YV;
	sAmpLcd.Data.RunData.PxyDisplayEnd.YV		=	sAmpLcd.Data.RunData.PxyDisplayStart.YV+Para.YV;
	//=================================显示数据
	xs	=	sAmpLcd.Data.RunData.PxyDisplayStart.XH;
	ys	=	sAmpLcd.Data.RunData.PxyDisplayStart.YV;
	xe	=	sAmpLcd.Data.RunData.PxyDisplayEnd.XH;
	ye	=	sAmpLcd.Data.RunData.PxyDisplayEnd.YV;
	ST7789V_ShowStringBKAre(xs,ys,xe,ye,FontSize,BackColor,PenColor,strLen,str);	//带背景色限定区域显示
	//-------------------------------------准备下一项内容显示参数
	sAmpLcd.Data.RunData.StartPixelY	=	sAmpLcd.Data.RunData.PxyFillEnd.YV;
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void DisplayNumber(const ampLcdListDef Node)
{
	unsigned char		strLen	=	0;		//字符串长度
	unsigned char		numLen	=	0;		//数值长度
	unsigned char 	offset=0;
	unsigned char		str[256];		//目标符串地址---存储
	
	unsigned short 	xs=0;		//字体显示坐标信息
	unsigned short 	ys=0;
	unsigned short 	xe=0;
	unsigned short 	ye=0;
	
	unsigned short 	Fxs=0;	//填充背景色坐标信息
	unsigned short 	Fys=0;
	unsigned short 	Fxe=0;
	unsigned short 	Fye=0;
//	
	unsigned short 	BackColor;	//背景色
	unsigned short 	PenColor;		//画笔色
	unsigned short 	FontSize;		//字体大小
	
//	WinInfoDef*	WinInfo;			//显示字体信息
	ParaDef			Para;
	FontDef			Font;
	//=================================初始化数据
//	WinInfo	=	(WinInfoDef*)&sAmpLcd.Windows.WinInfo;							//显示参数信息
	Font		=	sAmpLcd.Data.Cof.FtNum;
	Para		=	Node.ParaNum;
	//=================================数据内容检查
	numLen		=	Para.len;
	if(0==numLen)
	{
		return;
	}
	//=================================复制数据
	offset		=	Para.Offset;
	memcpy(str,&Node.String[offset],numLen);
	
	Para		=	Node.ParaUnit;
	strLen	=	Para.len;	
	offset		=	Para.Offset;
	
	memcpy(&str[numLen],&Node.String[offset],strLen);
	strLen	=	strLen+numLen;
	//=================================准备数据
	BackColor	=	Font.BackColor;
	PenColor	=	Font.PenColor;
	FontSize	=	Font.Size;
	//=================================清除显示区域
	if(Node.ParaByName.YV+Node.ParaCode.YV+Node.ParaSpec.YV+Node.ParaVender.YV>=FontSize)
	{
		sAmpLcd.Data.RunData.PxyFillStart.YV	=	sAmpLcd.Data.RunData.StartPixelY-FontSize;
	}
	else
	{
		sAmpLcd.Data.RunData.PxyFillStart.YV	=	sAmpLcd.Data.RunData.StartPixelY;
	}
	sAmpLcd.Data.RunData.PxyFillStart.XH	=	sAmpLcd.Data.Cof.PxyLeftFill.XH+(sAmpLcd.Data.Cof.PxyValid.XH	-	(Node.ParaNum.XH+Node.ParaUnit.XH));
	
	sAmpLcd.Data.RunData.PxyFillEnd.XH		=	sAmpLcd.Data.RunData.PxyFillStart.XH+(Node.ParaNum.XH+Node.ParaUnit.XH);
	sAmpLcd.Data.RunData.PxyFillEnd.YV		=	sAmpLcd.Data.RunData.PxyFillStart.YV+Para.YV;
	xs	=	sAmpLcd.Data.RunData.PxyFillStart.XH;
	ys	=	sAmpLcd.Data.RunData.PxyFillStart.YV;
	xe	=	sAmpLcd.Data.RunData.PxyFillEnd.XH;
	ye	=	sAmpLcd.Data.RunData.PxyFillEnd.YV;
	ST7789V_Fill(xs,ys,xe,ye,BackColor);	//背景色填充/擦除/清除	
	
	//=================================设定显示区域

	sAmpLcd.Data.RunData.PxyDisplayStart.XH	=	sAmpLcd.Data.RunData.PxyFillStart.XH;
	sAmpLcd.Data.RunData.PxyDisplayEnd.XH		=	sAmpLcd.Data.RunData.PxyFillEnd.XH;

	sAmpLcd.Data.RunData.PxyDisplayStart.YV		=	sAmpLcd.Data.RunData.PxyFillStart.YV;
	sAmpLcd.Data.RunData.PxyDisplayEnd.YV			=	sAmpLcd.Data.RunData.PxyFillEnd.YV;
	//=================================显示数据
	xs	=	sAmpLcd.Data.RunData.PxyDisplayStart.XH;
	ys	=	sAmpLcd.Data.RunData.PxyDisplayStart.YV;
	xe	=	sAmpLcd.Data.RunData.PxyDisplayEnd.XH;
	ye	=	sAmpLcd.Data.RunData.PxyDisplayEnd.YV;
	ST7789V_ShowStringBKAre(xs,ys,xe,ye,FontSize,BackColor,PenColor,strLen,str);	//带背景色限定区域显示
	//-------------------------------------准备下一项内容显示参数
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void DisplayTitle(unsigned char DualFlag)
{
	unsigned char		strLen	=	0;		//字符串长度
	unsigned char 	offset=0;
	unsigned char		str[256];		//目标符串地址---存储

	
	unsigned short xs=0;	//字体显示坐标信息
	unsigned short ys=0;
	unsigned short xe=0;
	unsigned short ye=0;
	
	unsigned short Fxs=0;	//填充背景色坐标信息
	unsigned short Fys=0;
	unsigned short Fxe=0;
	unsigned short Fye=0;
//	
	unsigned short BackColor;	//背景色
	unsigned short PenColor;	//画笔色
	unsigned short FontSize;	//字体大小
	
	unsigned char 	AddrLay;		//层地址
	unsigned char 	AddrSeg;		//位地址
	
	unsigned char 	page1	=	0;		//
	unsigned char 	page2	=	0;		//
	unsigned char 	total	=	0;		//
	
//	WinInfoDef*	WinInfo;			//显示字体信息
	ParaDef			Para;
	FontDef			Font;
	
	//=================================初始化数据
//	WinInfo	=	(WinInfoDef*)&sAmpLcd.Windows.WinInfo;							//显示参数信息
	Font		=	sAmpLcd.Data.Cof.FtCode;
	
	BackColor	=	Font.BackColor;
	PenColor	=	Font.PenColor;
	FontSize	=	Font.Size;
	
	AddrLay	=	sAmpLcd.Data.SysData.AddrLay;
	AddrSeg	=	sAmpLcd.Data.SysData.AddrSeg;
	
	page1		=	sAmpLcd.Data.Display.Serial;
	total		=	sAmpLcd.Data.Display.Count;
	
	xs	=	sAmpLcd.Data.Cof.PxyLeftFill.XH;
	ys	=	sAmpLcd.Data.Cof.PxyTopFill.YV;
	
	
	ST7789V_PrintfBK(xs,ys,FontSize,BackColor,PenColor,"层:%d 位:%d",AddrLay,AddrSeg);				//后边的省略号就是可变参数
	
	if(1==DualFlag)
	{
		xs	=	sAmpLcd.Data.Cof.PxyPixel.XH-FontSize*4;
		ST7789V_PrintfBK(xs,ys,FontSize,BackColor,PenColor,"%0.2d/%0.2d页",page1,total);				//后边的省略号就是可变参数
	}
	else	if(2==DualFlag)
	{
		xs	=	sAmpLcd.Data.Cof.PxyPixel.XH-FontSize*8;
		ST7789V_PrintfBK(xs,ys,FontSize,BackColor,PenColor,"%0.2d/%0.2d页/%0.2d/%0.2d页",page1-1,total,page1,total);				//后边的省略号就是可变参数
	}
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short GetYVLen(const ampLcdListDef Node)
{
	unsigned short YVLen=0;	//Y占用点数
	YVLen=0;	//Y占用点数
	YVLen	+=	Node.ParaName.YV;
	YVLen	+=	Node.ParaByName.YV;
	YVLen	+=	Node.ParaSpec.YV;
	YVLen	+=	Node.ParaVender.YV;
	YVLen	+=	Node.ParaCode.YV;
	return YVLen;
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void SetBackColor(unsigned short BKColor)
{
//	if(sAmpLcd.Display.LcdPort.ST7789VBColor	!=	BKColor)
//	{
//		STM32_FLASH_Write(BackColorStartAddr,(unsigned short*)&BKColor,1);						//从指定地址写入指定长度的数据
//		BKColor	=	GetBackColor();
//		sAmpLcd.Display.LcdPort.ST7789VBColor=BKColor;
//	}	
//	ST7789V_Clean(BKColor);	//清除屏幕函数
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short GetBackColor(void)
{
	unsigned short BKColor;
	STM32_FLASH_Read(BackColorStartAddr,(unsigned short*)&BKColor,1);							//从指定地址开始读出指定长度的数据
	return	BKColor;
}
//------------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void SendData(void)
{ 
  unsigned  short   sendedlen = sAmpLcd.Data.CommData.TxLen;
	if(sendedlen)
	{
		sendedlen	=	api_rs485_send_force(&sAmpLcd.Hal.RS485Port,sAmpLcd.Data.CommData.Txd,sendedlen);	//RS485-DMA发送程序
		if(sendedlen)
		{
			sAmpLcd.Data.CommData.TxLen	=	0;
		}
	}
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void AckData(void)
{
//	memcpy(sAmpLcd.Comm.Txd,ackupfarme,AmpMinFrameSize);
//	sAmpLcd.Comm.TxLen	=	AmpMinFrameSize;
	unsigned short len=0;
	ampAckDef	AckFrame;
	ampphydef* frame;


	AckFrame.head	=	headcode;
	AckFrame.length	=	5;
	AckFrame.cmd.cmd	=	ampCmdAck;
	AckFrame.cmd.rv		=	0;
	AckFrame.cmd.dir	=	1;
	AckFrame.address1	=	0;
	AckFrame.address2	=	sAmpLcd.Data.SysData.AddrLay;	//层地址
	AckFrame.address3	=	sAmpLcd.Data.SysData.AddrSeg;	//位地址
	AckFrame.status=0;
	
	frame	=	(ampphydef*)&AckFrame;
	len	=	api_set_frame(frame,ampCmdAck,1);    //补充消息的CRC和结束符，返回帧长度
	
	memcpy(sAmpLcd.Data.CommData.Txd,&AckFrame,len);
	sAmpLcd.Data.CommData.TxLen	=	len;

}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void ReceiveData(void)
{
  unsigned short RxNum  = 0;
  //==========================================================接收查询
  //---------------------层板接口 USART2
  RxNum = api_rs485_receive(&sAmpLcd.Hal.RS485Port,sAmpLcd.Data.CommData.Rxd);
  if(RxNum)
  {	
    ProcessData(sAmpLcd.Data.CommData.Rxd,RxNum);              //柜消息处理
  }
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void ProcessData(unsigned char* ReceDatabuffer,unsigned short datalen)
{
  unsigned  short framlength  = datalen; 
  unsigned  char* StartAddr    = ReceDatabuffer;         //备份数据缓存起始地址	
	unsigned 	char	i=0;
	unsigned 	char	DataLen	=0;		//数据段长度

  
  ampphydef* ampframe=NULL;
  //-------------------------检查端口是否为层接口及缓存地址是否为空
  if(NULL==StartAddr||0==framlength)
  {
    return;
  }	
	ReceiveDataCheckStart:	//开始检测接收到的数据
  //-------------------------协议检查
  ampframe	=	(ampphydef*)api_get_frame(StartAddr,framlength);    //判断帧消息内容是否符合协议
  if(NULL== ampframe)
  {
    return;		//退出，未检测到有效的帧
  }
  //-------------------------检查是否为应答帧
	if(ampCmdAck==ampframe->msg.cmd.cmd)
	{
		goto ReCheckData;		//重新检测剩余的数据
	}
  //-------------------------层地址检查
	if((sAmpLcd.Data.SysData.AddrLay!=ampframe->msg.addr.address2)&&(0xFF!=ampframe->msg.addr.address2))
	{
		goto ReCheckData;		//重新检测剩余的数据
	}
	//-------------------------位地址检查
	if((sAmpLcd.Data.SysData.AddrSeg	!=	ampframe->msg.addr.address3)&&(0xFF	!=	ampframe->msg.addr.address3))
	{
		goto ReCheckData;		//重新检测剩余的数据
	} 
  //-------------------------下发数据
  if(0  ==  ampframe->msg.cmd.dir)	//最高位为0表示上往下发
  {
    //---------------------------显示数据命令
    if(ampCmdLcdData ==  ampframe->msg.cmd.cmd) 
    {	
			unsigned char* buffer=NULL;
			DataLen	=	ampframe->msg.length-4;		//数据段长度:删除命令和地址数据
			buffer=(unsigned char*)ampframe->msg.data;
			
			PackManaData:
			GetManaData(buffer,DataLen);
			if((0xFF	!=	ampframe->msg.addr.address2)&&(0xFF	!=	ampframe->msg.addr.address3))
				AckData();						
    }
		//---------------------------修改背景色命令--只带2字节数据,低8位颜色在前
		else if(ampCmdLcdConf ==  ampframe->msg.cmd.cmd)
		{
			unsigned short BackColor	=	0;			
			memcpy(&BackColor,ampframe->msg.data,2);	//2字节背景色数据，低位在前
			SetBackColor(BackColor);
			if((0xFF	!=	ampframe->msg.addr.address2)&&(0xFF	!=	ampframe->msg.addr.address3))
				AckData();			
		}
		else
		{
			if((0xFF	!=	ampframe->msg.addr.address2)&&(0xFF	!=	ampframe->msg.addr.address3))
				AckData();	
		}
		goto ReCheckData;		//重新检测剩余的数据
  }
	//------------------------------继续检查剩余数据，直到检查失败
	ReCheckData:
	StartAddr=(unsigned char*)&ampframe[1];
	framlength-=1;	//去掉一字节长度
	goto ReceiveDataCheckStart;
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	GetManaData
*功能描述		:	从消息中获取药品信息，存储在药品列表中
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void GetManaData(const unsigned char* Databuffer,unsigned short datalen)
{
	unsigned char		i=0;
	unsigned char 	strtype		=0;					//字符串类型
	unsigned char 	strLen		=0;					//字符串长度
	unsigned char 	FinishedDataLen	=	0;	//已处理数据个数
	unsigned char		Offset			=	0;			//下一个类型字符串的偏移地址
	unsigned char*	ParaOffsetAddr;				//字符串地址偏移指针
	unsigned char*	ParalenAddr;					//字符串长度指针	
	//unsigned char		MaxList		=	0;
	unsigned char* 	pSource;							//源字符串地址---源自串口接收到的数据
	unsigned char* 	pTarget;							//目标符串地址---存储
	
	ManaDef*	ManaData;
	ampLcdListDef*	List;
	ampLcdListDef		Node;
	
	//=================================初始化数据
	List			=	sAmpLcd.Data.Display.List;
	//MaxList		=	sAmpLcd.Data.DisplayData.MaxNameList;	//可接收的最大条目限制
	memset(&Node,0x00,sizeof(ampLcdListDef));
	//=================================查找空缓存
	for(i=0;i<ampLcdListSize;i++)
	{
		if(0==List[i].ListNum)						//缓存为空，可以拷贝数据
		{
			break;
		}
	}
	//=================================未查找到空缓存，缓存满
	if(i>=ampLcdListSize)
	{
		return;
	}
	//=================================分类拆装数据
	//---------------------------------初始化指针地址和数据
	ManaData	=	(ManaDef*)Databuffer;
	pSource		=	ManaData->String;			//数据源地址
	pTarget		=	Node.String;					//数据目标地址
	//---------------------------------01名称参数
	strtype		=	ManaData->type;
	strLen		=	ManaData->len;
	if(0!=ampLcd_get_type_data_address(Databuffer,datalen,ampLcdParaName))
	{
		ampLcd_SetManaDataInfo(Databuffer,datalen);
	}

	if(strLen+2>AMPLcdMaxStringLen)		//字符个数超限(字符串+type和len)
		return;
	
	GetDataStart:
	switch(strtype)	//字符串类型判断
	{
		case	0x01:	//商品名类型
			if(0!=Node.ParaName.len)	//数据异常
			{
				return;
			}
			ParaOffsetAddr	=	&Node.ParaName.Offset;
			ParalenAddr			=	&Node.ParaName.len;
		break;
		case	0x02:	//规格参数
			if(0!=Node.ParaSpec.len)	//数据异常
			{
				return;
			}
			ParaOffsetAddr	=	&Node.ParaSpec.Offset;
			ParalenAddr			=	&Node.ParaSpec.len;		
		break;
		case	0x03:	//数量参数
			if(0!=Node.ParaNum.len)	//数据异常
			{
				return;
			}
			ParaOffsetAddr	=	&Node.ParaNum.Offset;
			ParalenAddr			=	&Node.ParaNum.len;		
		break;
		
		case	0x04:	//别名
			if(0!=Node.ParaByName.len)	//数据异常
			{
				return;
			}
			ParaOffsetAddr	=	&Node.ParaByName.Offset;
			ParalenAddr			=	&Node.ParaByName.len;			
		break;
		case	0x05:	//厂商名称
			if(0!=Node.ParaVender.len)	//数据异常
			{
				return;
			}
			ParaOffsetAddr	=	&Node.ParaVender.Offset;
			ParalenAddr			=	&Node.ParaVender.len;
		break;
		case	0x06:	//商品编码
			if(0!=Node.ParaCode.len)	//数据异常
			{
				return;
			}
			ParaOffsetAddr	=	&Node.ParaCode.Offset;
			ParalenAddr			=	&Node.ParaCode.len;
		break;
		case	0x07:	//数量单位
			if(0!=Node.ParaUnit.len)	//数据异常
			{
				return;
			}
			ParaOffsetAddr	=	&Node.ParaUnit.Offset;
			ParalenAddr			=	&Node.ParaUnit.len;
		break;
		default:return;
	}
	*ParalenAddr		=	strLen;					//字符串长度
	*ParaOffsetAddr	=	Offset;					//当前字符串偏移地址

	//-------------------------------复制数据
	memcpy(pTarget,pSource,strLen);
	
	//-------------------------------统计已处理字节数
	Offset		=	Offset+strLen;				//下一个类型字符串的偏移地址
	FinishedDataLen+=strLen+2;							//已处理数据个数(字符串+type和len)
	//-------------------------------指向下一类型
	if(FinishedDataLen>=datalen)
	{
		//-------------------------------正确获取到数据
		if(0!=Node.ParaName.len||0!=Node.ParaByName.len)	//商品名称或者别名是必需数据
		{
			SetManaData(&Node);															//设置显示参数---数据获取成功后设置相关的显示参数
		}
		return;
	}
	//-------------------------------指向下一类型	
	ManaData	=	(ManaDef*)&pSource[strLen];
	pSource		=	ManaData->String;		//新源字符串地址
	pTarget		=	&pTarget[strLen];		//新目标字符串地址
	strtype		=	ManaData->type;
	strLen		=	ManaData->len;
	//-------------------------------剩下字符重新做类型检查
	if(FinishedDataLen+strLen>AMPLcdMaxStringLen)	//字符个数超限
		return;
	goto GetDataStart;
	
}
//------------------------------------------------------------------------------



/*******************************************************************************
*函数名			:	function
*功能描述		:	设置显示参数---数据获取成功后设置相关的显示参数
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void SetManaData(ampLcdListDef* pNode)
{
	unsigned char		i=0;
	unsigned char 	strtype		=	0;		//字符串类型
	unsigned char 	strLen		=	0;		//字符串长度
	unsigned char		FontSize	=	0;		//字体大小16,24,32
	unsigned char		Line			=	0;		//可显示行数
//	unsigned char 	strCount	=0;		//已处理的字符串个数计数
	unsigned char 	FinishedDataLen	=	0;	//已处理数据个数
//	unsigned char		Offset			=	0;	//下一个类型字符串的偏移地址
//	unsigned char*	ParaOffsetAddr;		//字符串地址偏移指针
//	unsigned char*	ParalenAddr;			//字符串长度指针
	
	//unsigned char		MaxList		=	0;
//	unsigned char		ListCount	=	0;
	
	unsigned char* 	pSource;		//源字符串地址---源自串口接收到的数据
	unsigned char* 	pTarget;		//目标符串地址---存储
	
	unsigned short	XH;						//X--水平占用点数
	unsigned short	YV;						//Y--垂直占用点数
	unsigned short	OffsetXH;			//X--数量和单位点用的水平点数
	
	unsigned short*	pTargetXH;		//X--数据水平有效点数地址
	unsigned short*	pTargetYV;		//Y--数据垂直有效点数地址
	
	unsigned short	ValidXH;			//X--水平有效点数
	unsigned short	ValidYV;			//Y--垂直有效点数
	
	ampLcdParaTyeDef*		Para;
	ampLcdListDef*		List;
	ampLcdListDef			Node;
//	WinInfoDef	WinInfo;			//显示字体信息
	//---------------------------------输入数据地址有效性
	if(NULL==pNode)
	{
		return;
	}
	//---------------------------------检查数据有效性
	if((0==pNode->ParaName.len)&&(0==pNode->ParaByName.len))	//商品名称或者别名是必需数据
	{
		return;
	}
	//=================================初始化数据
	List		=	sAmpLcd.Data.Display.List;
	//MaxList	=	sAmpLcd.Data.DisplayData.MaxNameList;	//可接收的最大条目限制
//	WinInfo	=	sAmpLcd.Windows.WinInfo;							//字体信息
	ValidXH	=	sAmpLcd.Data.Cof.PxyValid.XH;
	ValidYV	=	sAmpLcd.Data.Cof.PxyValid.YV;
	
	Node	=	*pNode;
	//=================================先计算数量和单位占用X点数(除名称外，右缩进需要根据数量和单位占用的X点数来调整)
	//---------------------------------获取数量和单位数据
	FontSize	=	sAmpLcd.Data.Cof.FtNum.Size;
	strLen		=	Node.ParaNum.len+Node.ParaUnit.len;		//数量和单位的字节点
	//---------------------------------计算数量占用水平点数
	Node.ParaNum.XH	=	Node.ParaNum.len*(FontSize/2);	//字符宽度为高度的一半
	Node.ParaNum.YV	=	FontSize;					//字符宽度为高度的一半
	//---------------------------------计算单位占用水平点数
	Node.ParaUnit.XH	=	Node.ParaUnit.len*(FontSize/2);	//字符宽度为高度的一半
	Node.ParaUnit.YV	=	FontSize;				//字符宽度为高度的一半
	//----------------------------------数量和单位共占用的水平点数
	OffsetXH	=	Node.ParaNum.XH+Node.ParaUnit.XH;	
	
	//=================================分类拆装数据
	//---------------------------------初始化指针地址和数据
	Para	=	(ampLcdParaTyeDef*)&Node.ParaName;

	//---------------------------------01名称参数
	strtype		=	0x01;		//从第一个参数开始
	SetDataStart:
	switch(strtype)
	{
		case	0x01:				//商品名类型
			Line	=	4;			//名称最大可显示4行
			strLen		=	Node.ParaName.len;			
			FontSize	=	sAmpLcd.Data.Cof.FtName.Size;		
			Node.ParaName.XH	=	sAmpLcd.Data.Cof.PxyValid.XH;
			
			pTargetXH	=	&Node.ParaName.XH;
			pTargetYV	=	&Node.ParaName.YV;
		break;
		case	0x02:				//规格参数
			Line	=	2;			//名称最大可显示2行
			strLen		=	Node.ParaSpec.len;			
			FontSize	=	sAmpLcd.Data.Cof.FtSpec.Size;		
			Node.ParaSpec.XH	=	sAmpLcd.Data.Cof.PxyValid.XH-OffsetXH;
		
			pTargetXH	=	&Node.ParaSpec.XH;
			pTargetYV	=	&Node.ParaSpec.YV;
		break;
		case	0x03:			//数量参数
			strtype+=1;
			goto SetDataStart;	
		break;
		
		case	0x04:			//别名
			Line	=	2;		//名称最大可显示2行
			strLen		=	Node.ParaByName.len;			
			FontSize	=	sAmpLcd.Data.Cof.FtByName.Size;		
			Node.ParaByName.XH	=	sAmpLcd.Data.Cof.PxyValid.XH-OffsetXH;
		
			pTargetXH	=	&Node.ParaByName.XH;
			pTargetYV	=	&Node.ParaByName.YV;			
		break;
		case	0x05:			//厂商名称
			Line	=	2;		//名称最大可显示2行
			strLen		=	Node.ParaVender.len;			
			FontSize	=	sAmpLcd.Data.Cof.FtVender.Size;		
			Node.ParaVender.XH	=	sAmpLcd.Data.Cof.PxyValid.XH-OffsetXH;
		
			pTargetXH	=	&Node.ParaVender.XH;
			pTargetYV	=	&Node.ParaVender.YV;
		break;
		case	0x06:			//商品编码
			Line	=	2;		//名称最大可显示2行
			strLen		=	Node.ParaCode.len;			
			FontSize	=	sAmpLcd.Data.Cof.FtCode.Size;		
			Node.ParaCode.XH	=	sAmpLcd.Data.Cof.PxyValid.XH-OffsetXH;
		
			pTargetXH	=	&Node.ParaCode.XH;
			pTargetYV	=	&Node.ParaCode.YV;
		break;
		case	0x07:			//数量单位
			strtype+=1;
			goto SetDataStart;
		break;
		default:goto AddNode;	//未找到类型数据
	}
	if(0==FontSize)
		return;
	//---------------------------------计算一行可以显示多少个对应的数据
	i=*pTargetXH/(FontSize/2);	//字符宽度为高度的一半
	//---------------------------------计算需要多少行才能显示完
	if(0==i)
		return;
	if(0==strLen%i)		//整数行
	{
		i	=	strLen/i;
	}
	else
	{
		i	=	strLen/i+1;
	}
	//---------------------------------检查行数有无超限,超出部分不计入显示(不显示)
	if(i>Line)
	{
		i=Line;
	}
	//---------------------------------计算Y轴需要多少点数
	*pTargetYV	=	i*FontSize;
	//---------------------------------继续检查其它项
	strtype+=1;
	if(strtype<8)
	{
		goto SetDataStart;
	}
	//---------------------------------添加数据：添加数据前比较缓存有无相同的数据，有则不添加
	AddNode:
	for(i=0;i<ampLcdListSize;i++)
	{
		if(0==memcmp((unsigned char*)&List[i].ParaName,&Node.ParaName,sizeof(ampLcdListDef)-2))	//检查数据内容(减2是因为List[i].ListNum，16字节对齐)
		{
			if(0!=List[i].ListNum)	//序号存在
				return;
		}
	}
//---------------------------------无相同数据，往空缓存内添加数据	
	for(i=0;i<ampLcdListSize;i++)
	{
		if(0==List[i].ListNum)		//缓存为空，可以拷贝数据
		{
			//List	=	List[i];
			memcpy((unsigned char*)&List[i],&Node,sizeof(ampLcdListDef));
			sAmpLcd.Data.Display.Count+=1;
			List[i].ListNum	=	i+1;
			break;
		}
	}	
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	SetManaDataInfo
*功能描述		:	设置显示参数---数据获取成功后设置相关的显示参数
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void ampLcd_SetManaDataInfo(const unsigned char* String,unsigned short datalen)
{
	unsigned char		step	=	0;			//获取类型步骤，总共7种类型
	unsigned char		i	=	0,j=0;
	unsigned char		GotNameFlag=0;	//获取到药品名称：数据必须要有药品名称
	unsigned short	Ycount	=	0;		//Y点数
	unsigned short 	ParaLen	=	0;		//参数长度
	unsigned short	Bufflen	=	0;
	
	ManaDef*	Mana	=	(ManaDef*)String;
	
	
	if(0!=Mana)
	{
		unsigned char			i	=	0;
		unsigned short		len=0;
		ampLcdParaTyeDef	ampLcdParaTye;	//参数类型
		ManaDef		ManaA[7];		//按照:0x01-耗材名称,0x04-别名,0x05-厂家名称,0x02-耗材规格,0x06-耗材编码,0x03-耗材数量,0x07-数量单位>提取数据排序
		ManaDef*	ManaB;
		ManaDef*	ManaC;
		//=======================分类提取数据
		for(i=0;i<7;i++)
		{
			switch(Mana->type)
			{
				case	ampLcdParaName:			//0x01-耗材名称
							ManaB	=	&ManaA[0];
				break;
				case	ampLcdParaByName:		//0x04-别名
							ManaB	=	&ManaA[1];
				break;
				case	ampLcdParaVender:		//0x05-厂家名称
							ManaB	=	&ManaA[2];
				break;
				case	ampLcdParaSpec:			//0x02-耗材规格
							ManaB	=	&ManaA[3];
				break;
				case	ampLcdParaCode:			//0x06-耗材编码
							ManaB	=	&ManaA[4];
				break;
				case	ampLcdParaNum:			//0x03-耗材数量
							ManaB	=	&ManaA[5];
				break;
				case	ampLcdParaUnit:			//0x07-数量单位
							ManaB	=	&ManaA[6];
				break;
				
				default:
				break;
					
			}
			len+=Mana->len+2;		//+2为类型和长度位
			memcpy(ManaB,Mana,Mana->len+2);
			
			if(len<datalen)
				Mana	=	(ManaDef*)&String[Mana->len+2];
		}
		//=======================合并和对比数据:必须得有耗材名称
		if((len<=datalen)&&(ManaA[0].len))
		{	
			unsigned char unlen	=	0;		//不需要参与对比的字符串:数量和单位
			unsigned char buffer[AMPLcdMaxStringLen];
			ampLcdListDef*	List	=	sAmpLcd.Data.Display.List;
			len	=	0;
			unlen	=	ManaA[5].len+ManaA[6].len;
			//---------------------合并数据:必须得有耗材名称
			for(i=0;i<7;i++)
			{
				memcpy(&buffer[len],ManaA[i].String,ManaA[i].len);
				len	+=	ManaA[i].len;
			}
			//---------------------对比数据，排除重复，如果数量更新，更新数量，否则添加数据
			for(i=0;i<sAmpLcd.Data.Display.Count;i++)
			{
				//-------------------对比除数量和单位之外的数据
				if(0==memcpy(buffer,List[i].String,len-unlen))
				{
					//-----------------对比数量和单位
					if(len	==	List[i].len)
					{
						if(0==memcpy(buffer,List[i].String,len))
						{
							break;
						}
						//---------------更新数据
						else
						{
						}
					}
				}
			}
			//=====================
	
		}
	}		
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned char* ampLcd_get_type_data_address(const unsigned char* String,unsigned short datalen,ampLcdParaTyeDef	ampLcdParaTye)
{
	unsigned char			i	=	0;
	unsigned short 		ParaLen	=	0;		//参数长度
	unsigned short 		Len	=	0;				//消息长度
	unsigned char*		address	=	(unsigned char*)String;
	unsigned char*		address2	=	0;
	
	ampLcdParaTyeDef	ampLcdParaTyeCmp;
	//=================================
	for(i=0;i<datalen;i++)
	{
		ampLcdParaTyeCmp	=	(ampLcdParaTyeDef)address[i];
		//-------------------------------耗材类型标识		
		if(ampLcdParaTye	==	ampLcdParaTyeCmp)
		{
			ParaLen		=	address[1];
			Len				=	(address-String)+ParaLen+1;		//+1为长度标识位
			if(Len<=datalen)
			{
				address2	=	&address[i];
				break;
			}
		}
		//-------------------------------
		else
		{
		}
	}
	//=================================
	return address2;
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	DataInitialize
*功能描述		:	数据初始化
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void DataInitialize(void)
{
	unsigned	char*	addr;
	ampLcdCofDef*	Cof	=	(ampLcdCofDef*)&sAmpLcd.Data.Cof;
	//==========================================显示参数初始化
//	WinInfoDef* WinInfo	=	(WinInfoDef*)&sAmpLcd.Windows.WinInfo;
	//==========================================WinFont
	Cof->FtDefault.BackColor	=	DisplayBackColor;
	Cof->FtDefault.PenColor		=	DisplayFontColor;
	Cof->FtDefault.Size				=	DisplayFontSize;
	
	Cof->FtName.BackColor			=	DisplayNameBkColor;
	Cof->FtName.PenColor			=	DisplayNameFtColor;
	Cof->FtName.Size					=	DisplayNameFtSize;
	
	Cof->FtByName.BackColor		=	DisplayByNameBkColor;
	Cof->FtByName.PenColor		=	DisplayByNameFtColor;
	Cof->FtByName.Size				=	DisplayByNameFtSize;
	
	Cof->FtSpec.BackColor			=	DisplaySpecBkColor;
	Cof->FtSpec.PenColor			=	DisplaySpecFtColor;
	Cof->FtSpec.Size					=	DisplaySpecFtSize;
	
	Cof->FtNum.BackColor			=	DisplayNumBkColor;
	Cof->FtNum.PenColor				=	DisplayNumFtColor;
	Cof->FtNum.Size						=	DisplayNumFtSize;
	
	Cof->FtSeril.BackColor		=	DisplaySerialBkColor;
	Cof->FtSeril.PenColor			=	DisplaySerialFtColor;
	Cof->FtSeril.Size					=	DisplaySerialFtSize;
	
	Cof->FtCode.BackColor			=	DisplayCodeBkColor;
	Cof->FtCode.PenColor			=	DisplayCodeFtColor;
	Cof->FtCode.Size					=	DisplayCodeFtSize;
	
	Cof->FtVender.BackColor		=	DisplayVenderBkColor;
	Cof->FtVender.PenColor		=	DisplayVenderFtColor;
	Cof->FtVender.Size				=	DisplayVenderFtSize;
	
	Cof->FtTitle.BackColor		=	DisplayTitleBkColor;
	Cof->FtTitle.PenColor			=	DisplayTitleFtColor;
	Cof->FtTitle.Size					=	DisplayTitleSize;
	
	Cof->FtSepar.BackColor		=	DisplaySeparBkColor;
	Cof->FtSepar.Size					=	DisplaySeparWidth;
	
	//==========================================WinInfo
	//------------------------------------------整屏像素点
	Cof->PxyPixel.XH					=	ST7789V_V;		//水平点个数
	Cof->PxyPixel.YV					=	ST7789V_H;		//垂直点个数
	
	//------------------------------------------顶端垂直起始/填充点
	Cof->PxyTopFill.XH				=	0;
	Cof->PxyTopFill.YV				=	DisplayTopFillWidth;	
	//------------------------------------------底部垂直起始/填充点
	Cof->PxyBotFill.XH				=	0;
	Cof->PxyBotFill.YV				=	DisplayBotFillWidth;
	//------------------------------------------左边起始/填充点
	Cof->PxyLeftFill.XH				=	DisplayLeftFillWidth;
	Cof->PxyLeftFill.YV				=	0;
	//------------------------------------------剩余有效使用点数
	Cof->PxyValid.XH					=	Cof->PxyPixel.XH	-	Cof->PxyLeftFill.XH-1;
	Cof->PxyValid.YV					=	Cof->PxyPixel.YV	-	(Cof->PxyBotFill.YV+Cof->PxyTopFill.YV);
	
	sAmpLcd.Data.RunData.TopDisplayStartY			=	Cof->PxyTopFill.YV+Cof->FtTitle.Size;
	sAmpLcd.Data.RunData.BotDisplayStartY			=	sAmpLcd.Data.RunData.TopDisplayStartY+(Cof->PxyValid.YV-Cof->FtTitle.Size)/2;
	//------------------------------------------最大可以接收商品个数
//	addr=(unsigned char*)&sAmpLcd.Windows.ManaData.MaxNameList;		//const类型
	//sAmpLcd.Data.DisplayData.MaxNameList	=	ampLcdListSize;		//const类型
//	*addr	=	DspMaxNameTypeCount;
}
//------------------------------------------------------------------------------



/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void SwitchID_Server(void)
{
	static unsigned short time=0;
	if(time++>1000)
	{
		unsigned char temp	=	0;
		time	=	0;
		temp	=	api_get_SwitchId_data_left(&sAmpLcd.Hal.sSwitch);
		sAmpLcd.Data.SysData.AddrLay=(temp>>4)&0X0F;  	//层地址
		sAmpLcd.Data.SysData.AddrSeg=temp&0x0F;      	//位地址
	}
}
//------------------------------------------------------------------------------






/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void HW_Configuration(void)
{
//	spi_def	*SPI	=	&sAmpLcd.SpiPort;
	sST7789VDef	sST7789V;
	GT32L32Def	sGT32L32;
	SwitchDef 	sSwitch;			//拔码开关
	
	unsigned short temp;
	
	//-------------------------------------------层板接口USART1 PA11-RE,PA12-TE
  sAmpLcd.Hal.RS485Port.USARTx  					= ampLcdCommPort;
	sAmpLcd.Hal.RS485Port.RS485_TxEn_PORT		=	ampLcdCommTxEnPort;
	sAmpLcd.Hal.RS485Port.RS485_TxEn_Pin		=	ampLcdCommTxEnPin;
	
	sAmpLcd.Hal.RS485Port.RS485_RxEn_PORT		=	ampLcdCommRxEnPort;
	sAmpLcd.Hal.RS485Port.RS485_RxEn_Pin		=	ampLcdCommRxEnPin;	
  api_rs485_configuration_NR(&sAmpLcd.Hal.RS485Port,ampLcdCommBaudRate,maxFramesize);	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
	
	//-------------------------------------------拨码开关
	sSwitch.NumOfSW	=	8;
  
  sSwitch.SW1_PORT	=	GPIOC;
  sSwitch.SW1_Pin		=	GPIO_Pin_7;
  
  sSwitch.SW2_PORT	=	GPIOC;
  sSwitch.SW2_Pin		=	GPIO_Pin_6;
  
  sSwitch.SW3_PORT	=	GPIOC;
  sSwitch.SW3_Pin		=	GPIO_Pin_5;
  
  sSwitch.SW4_PORT	=	GPIOC;
  sSwitch.SW4_Pin		=	GPIO_Pin_4;
  
  sSwitch.SW5_PORT	=	GPIOC;
  sSwitch.SW5_Pin		=	GPIO_Pin_3;
  
  sSwitch.SW6_PORT	=	GPIOC;
  sSwitch.SW6_Pin		=	GPIO_Pin_2;
  
  sSwitch.SW7_PORT	=	GPIOC;
  sSwitch.SW7_Pin		=	GPIO_Pin_1;
  
  sSwitch.SW8_PORT	=	GPIOC;
  sSwitch.SW8_Pin		=	GPIO_Pin_0;

	sAmpLcd.Hal.sSwitch		=	sSwitch;
	
	api_SwitchId_initialize(&sAmpLcd.Hal.sSwitch);	
	
	temp	=	api_get_SwitchId_data_left(&sAmpLcd.Hal.sSwitch);
  sAmpLcd.Data.SysData.AddrLay=(temp>>4)&0X0F;  	//层地址
  sAmpLcd.Data.SysData.AddrSeg=temp&0x0F;      	//位地址
	
	
	//-------------------------------------------拨码开关
	GPIO_Configuration_OPP50(ampLcdSYSLEDPort,ampLcdSYSLEDPin);
	
	//-------------------------------------------LCD参数初始化
	sST7789V.HWPort.sBL_PORT				=	GPIOA;
	sST7789V.HWPort.sBL_Pin					=	GPIO_Pin_1;
	
	sST7789V.HWPort.sRD_PORT				=	GPIOD;
	sST7789V.HWPort.sRD_Pin					=	GPIO_Pin_2;	
	
	sST7789V.HWPort.sREST_PORT			=	GPIOA;
	sST7789V.HWPort.sREST_Pin				=	GPIO_Pin_15;
	
	sST7789V.HWPort.sDC_PORT				=	GPIOC;
	sST7789V.HWPort.sDC_Pin					=	GPIO_Pin_11;
	
	sST7789V.HWPort.sWR_PORT				=	GPIOC;
	sST7789V.HWPort.sWR_Pin					=	GPIO_Pin_12;	
	
	sST7789V.HWPort.sCS_PORT				=	GPIOC;
	sST7789V.HWPort.sCS_Pin					=	GPIO_Pin_10;
	
	sST7789V.HWPort.sDATABUS_PORT		=	GPIOB;
	sST7789V.HWPort.sDATABUS_Pin		=	GPIO_Pin_All;	
	
	sST7789V.ST7789VRotate	=	ST7789V_Rotate_90D;		//使用旋转角度
	
	sAmpLcd.Hal.LcdPort	=	sST7789V;
	
	ST7789V_Initialize(&sAmpLcd.Hal.LcdPort);
	
	ST7789V_Clean(0xFFFF);
	ST7789V_Clean(LCD565_DARKBLUE);
	
	//-------------------------------------------字库参数初始化
	sGT32L32.SPI.port.SPIx			=	SPI1;
	sGT32L32.SPI.port.nss_port	=	GPIOA;
	sGT32L32.SPI.port.nss_pin		=	GPIO_Pin_4;
	
	sGT32L32.SPI.port.clk_port	=	GPIOA;
	sGT32L32.SPI.port.clk_pin		=	GPIO_Pin_5;
	
	sGT32L32.SPI.port.miso_port	=	GPIOA;
	sGT32L32.SPI.port.miso_pin	=	GPIO_Pin_6;
	
	sGT32L32.SPI.port.mosi_port	=	GPIOA;
	sGT32L32.SPI.port.mosi_pin	=	GPIO_Pin_7;	
	sGT32L32.SPI.port.SPI_BaudRatePrescaler_x=SPI_BaudRatePrescaler_2;
	
	sAmpLcd.Hal.GT32L32	=	sGT32L32;	
  
  api_gt32l32_configuration(&sAmpLcd.Hal.GT32L32);				//普通SPI通讯方式配置
	
	
}
//------------------------------------------------------------------------------




/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void ampLcd_Display_Server(void)
{
	ampLcdDisplayDataDef*	ampLcdDisplayData=&sAmpLcd.Data.Display;
	//============================检查有无缓存
	if((0==ampLcdDisplayData->Count)||(0==ampLcdDisplayData->Sync))
	{
		//ampLcdDisplayData->Serial	=	0;		
	}
	//============================有数据
	else
	{
		unsigned char i = 0;
		unsigned char* String=0;
		//--------------------------更新显示序列
		if(ampLcdDisplayData->Serial<ampLcdDisplayData->Count)
		{
			ampLcdDisplayData->Serial++;
		}
		else
		{
			ampLcdDisplayData->Serial=1;
		}
		//--------------------------获取当前应该显示缓存
		for(i=0;i<ampLcdListSize;i++)
		{
			if(ampLcdDisplayData->Serial	==	ampLcdDisplayData->List[i].ListNum)
			{
				String	=	ampLcdDisplayData->List[i].String;
				break;
			}
		}
		//--------------------------已经获取到缓存地址
		if(0!=String)
		{
			ampLcd_Display_String(String);
		}
	}
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void ampLcd_Display_String(const unsigned char* String)
{
	if(0!=String)
	{
		unsigned short i = 0;

	}
}
//------------------------------------------------------------------------------







#endif
