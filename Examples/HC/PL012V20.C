/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : PC001V21.c
* Author             : WOW
* Version            : V2.0.1
* Date               : 06/26/2017
* Description        : PC001V21����ư�.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifdef PL012V20				//�����LCD��

#include "PL012V20.H"

//#include "R61509V.h"
//#include "ILI9326.h"

#include "CS5530.H"

#include "LCD.H"
#include "GT32L32M0180.H"
#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_PWM.H"
#include "STM32_USART.H"



#include "SWITCHID.H"
#include 	"TOOL.H"

#include "string.h"				//�����ڴ��������ͷ�ļ�
#include "stm32f10x_dma.h"

LCDDef	sLCD;


#define	font	16

u16	DelayTime=0x1000;
u16	LCDTime=0;
u16	DSPTime=0;
u32 testADC=0;
u32 bacADC=0;
u32 bacADC2=0;



u16 SumFed[8]={0};		//�ܹ��ѷ�ҩ����
u16	SumFQ[8]={0};			//�ܹ���ҩ��������
u8	NumFW=0;		//����ҩ��λ
u8	Onlinede=0;		//����ҩ��λ


//t_Point point;
//u8 zimo[720]="R61509V_DrawRectangle(11,11,229,389,0X07FF)";

RS485Def  RS485;
u8 RS485FLG	=	0;
u32 RS485Time	=	0;
u32 RSRLen	=	0;

u8 TxdBuffe[256]={0};
u8 RxdBuffe[256]={0};
u8 RevBuffe[256]={0};
u16 RxNum=0;
char	Char_Buffer[256]={0xFF};		//��¼format����
//t_LcdCfg **pLcdpara;


SwitchDef	SWITCHID;
u8 SwitchID=0;	//���뿪�ص�ַ


u16 ATTime	=	0;
u8 ATNum	=	0;
u16	line	=	0;
u16	row	=	1;
u8 WriteFlag	=	0;
u8 PowerUP	=	0;

u8 	Serial	=	0;
u8 	Addr	=	0;
u16 Time	=	0;
u8 GetAdd	=	0;
u16 DspTime	=	0;
u16 color	=	0;

 u16 year; 
 u8 month;
 u8 day;
 u8 hour;
 u8 minute;
 u8 second;

unsigned char Version[]="PL012V2.0 RF���ܺĲĹ�����";


void GetTime(void);
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PL012V20_Configuration(void)
{
	SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H
	
	GPIO_DeInitAll();							//�����е�GPIO�ر�----V20170605
	LCD_Configuration();
//	R61509V_Configuration();
//	ILI9326_Configuration();
	RS485_Configuration();
	
//	LCD_Printf(0		,0	,32	,0,"��ţ�");				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
	
	SwitchID_Configuration();
//	SysTick_Configuration(1000);	//ϵͳ���ʱ������72MHz,��λΪuS
	
//	IWDG_Configuration(1000);			//�������Ź�����---������λms	
	PWM_OUT(TIM2,PWM_OUTChannel1,2,900);	//PWM�趨-20161127�汾--ָʾ��
	PWM_OUT(TIM3,PWM_OUTChannel3,1000,500);		//PWM�趨-20161127�汾--����
	
	GetTime();	
	
	LCD_ShowAntenna(380,0,3,LCD565_WHITE);   //��ʾ12x12���
	
	
	LCD_Printf(0,140,16,LCD565_RED,"��ʾ����:%4X",LCD_ReadData(0x0000));		//��������
	LCD_Printf(0,0,16,LCD565_YELLOW,"��Ŀ���:%s",Version);		//��Ŀ���
//  LCD_Printf(0,220,16,LCD565_RED,"����ʱ��:%4d-%0.2d-%0.2d-%s",year,month,day,__TIME__); 	//����ʱ��
  LCD_Printf(0,220,16,LCD565_WHITE,"%0.2d:",hour);		//��������
  LCD_Printf(8*3,220,16,LCD565_WHITE,"%0.2d:",minute);		//��������
  LCD_Printf(8*6,220,16,LCD565_WHITE,"%0.2d",second);		//��������
	
//	ILI9326_SetBackground(LCD565_BLUE);
  LCD_Printf(0,100,16,LCD565_RED,"%s",Version);		//��Ŀ���
	LCD_Printf(0,100,24,LCD565_RED,"ABCDEFGHJKLNMZCXQWERTPOIUY0123456789",Version);		//��Ŀ���
	LCD_PrintfScroll(0,100,32,LCD565_GREEN,"ABCDEFGHJKLNMZCXQWERTPOIUY0123456789",Version);		//��Ŀ���
  SysTick_Configuration(1000);	//ϵͳ���ʱ������72MHz,��λΪuS
}

/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void GetTime(void)
{
	BuildTimeDef*	BuildTime	=	GetBuildTime(__DATE__,__TIME__);
	
  year  	= BuildTime->year;
  month 	= BuildTime->month;
  day   	= BuildTime->day;
  hour  	= BuildTime->hour;
  minute  = BuildTime->minute;
	
  second  = BuildTime->second;
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PL012V20_Server(void)
{	
	
	IWDG_Feed();								//�������Ź�ι��
//	Time++;
//  ClockServer();
	if(Time++>999)
	{
		Time	=	0;
    ClockServer();
	}
	LCD_DisplayServer();
//	LCD_Fill(10,10,50,50,LCD565_WHITE);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
	
//	LCD_ShowBattery(360,0,2,LCD565_RED);   //��ʾ12x12���
//	LCD_ShowBattery(380,0,3,LCD565_RED);   //��ʾ12x12���
//	LCD_Printf(0,180,32,LCD565_RED,"��ȡ��ַ......");		//��ߵ�ʡ�Ժž��ǿɱ����
//	LCD_Clean(LCD565_BLACK); 					//�����Ļ����
	
//	LCD_Display();
//	return;
	PD014Test_Server();

//	RS485_Server();
	return;
}
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void ClockServer(void)
{
  if(++second>59)
  {    
    second  = 0;
    if(++minute>59)
    {  
      minute  = 0;
      if(++hour>=24)
      { 
        hour = 0;
      }
      LCD_Printf(0,220,16,LCD565_WHITE,"%0.2d",hour);		//��������      
    }
    LCD_Printf(8*3,220,16,LCD565_WHITE,"%0.2d",minute);		//��������    
  }
  LCD_Printf(8*6,220,16,LCD565_WHITE,"%0.2d",second);		//��������
}

/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void PD014Test_Server(void)
{
	u8 Num	=	0;
	
	
	Pd014AckFarmDef	RecAck;
	
	DspTime++;
	
//	PD014Test_GetAdd();
	
//	RS485_Server();
//	return;
	Num=RS485_ReadBufferIDLE(&RS485,RxdBuffe);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(Num)
	{
		PD014Test_Display(RxdBuffe,Num);
//		if(0	==	GetAdd)
//		{
//			GetAdd	=	1;			
//			LCD_Printf(0,font*11,24,LCD565_RED,"��ȡ����ַ%0.2X",Addr);		//��ߵ�ʡ�Ժž��ǿɱ����
//		}		
	}
	//=================================����ָʾ
	if(DspTime==500)
	{
		LCD_Fill(320,180,390,230,LCD565_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		LCD_Fill(335,190,375,220,LCD565_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		LCD_Fill(350,200,360,210,LCD565_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
	}
	else if(DspTime==1000)
	{
		DspTime	=	0;
		LCD_Fill(320,180,390,230,LCD565_YELLOW);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		LCD_Fill(335,190,375,220,LCD565_DARKBLUE);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		LCD_Fill(350,200,360,210,LCD565_GRAY);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
	}
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void PD014Test_GetAdd(void)
{
	Pd014Def	pInfo;
	if((50	==	Time++)&&(0	==	GetAdd))
	{
		Time	=	0;
		if(Addr++>0x03)
		{
			Addr	=	0;
			Serial+=1;
		}
			
		pInfo.Head			=	0x7E;
		pInfo.ADDdestin	=	Addr;
		pInfo.ADDsource	=	0x00;
		pInfo.Serial		=	Serial;
		pInfo.Cmd				=	0x01;
		pInfo.DataLen		=	0x00;
		pInfo.BCC8			=	BCC8(&pInfo.ADDdestin,5+pInfo.DataLen);
		pInfo.End				=	0x7F;
//		LCD_DrawLine(0,212,399,212,LCD565_RED);						//AB �������껭һ��ֱ��
		LCD_Printf(0,font*9,24,LCD565_YELLOW,"��ȡ��ַ%0.2X,��ˮ��%2d......",Addr,Serial);		//��ߵ�ʡ�Ժž��ǿɱ����
		RS485_DMASend(&RS485,(u8*)&pInfo,8+pInfo.DataLen);
		PowerUP	=	1;
	}
}
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void PD014Test_Display(u8* buffer,u8 RxNum)
{
	
	u16 temp	=	0;

	u16 PenColor	=LCD565_RED;
	if(0	!=	WriteFlag)
	{
		WriteFlag	=	0;
		PenColor	=LCD565_WHITE;
	}
	else
	{
		WriteFlag	=	1;
		PenColor	=LCD565_RED;
	}
	temp	=	font/2*3*RxNum;
	if(0<temp%400)
	{
		temp	=	(temp/400+1)*font;
	}
	else
	{
		temp	=	(temp/400)*font;
	}
	
	if(220-font*5<=temp+row)
	{
		LCD_Clean(LCD565_BLACK); 					//�����Ļ����
		row	=	0;
//		LCD_Printf(0,font*9,24,LCD565_RED,"��ȡ��ַ......");		//��ߵ�ʡ�Ժž��ǿɱ����
	}
	
	LCD_ShowHex(0,row,font,PenColor,RxNum,8,buffer);                //��ʾʮ����������
	temp	=	font/2*3*RxNum;
	if(0<temp%400)
	{
		row	+=	(temp/400+1)*font;
	}
	else
	{
		row	+=	(temp/400)*font;
	}
//	if(row>=240-font)
//	{
//		row=0;
//	}
}
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void RS485_Server(void)
{
#if 1
//	if(ATTime++>1000)
//	{
//		ATTime=0;
//		if(ATNum++>4)
//			ATNum=0;
//		LCD_Clean(LCD565_BLACK); 					//�����Ļ����
//	}
//	LCD_ShowAntenna(300,0,ATNum,200);   //��ʾ12x12����
	RxNum=RS485_ReadBufferIDLE(&RS485,RxdBuffe);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(RxNum)
	{
		u16 temp	=	0;
		u16 PenColor	=LCD565_RED;
		if(0	!=	WriteFlag)
		{
			WriteFlag	=	0;
			PenColor	=LCD565_WHITE;
		}
		else
		{
			WriteFlag	=	1;
			PenColor	=LCD565_GREEN;
		}
		temp	=	16*3*RxNum;
		if(0<temp%400)
		{
			temp	=	(temp/400+1)*32;
		}
		else
		{
			temp	=	(temp/400)*32;
		}
		if(240-32<=temp+row)
		{
			LCD_Clean(LCD565_BLACK); 					//�����Ļ����
			row	=	0;
		}
		
		LCD_ShowHex(0,row,32,PenColor,RxNum,8,RxdBuffe);                //��ʾʮ����������
		temp	=	16*3*RxNum;
		if(0<temp%400)
		{
			row	+=	(temp/400+1)*32;
		}
		else
		{
			row	+=	(temp/400)*32;
		}
		if(row>=240-32)
		{
			row=0;
//			LCD_Clean(LCD565_BLACK); 					//�����Ļ����
		}
//		LCD_Show(0,100,32,RxNum,RxdBuffe);
//		PWM_OUT(TIM3,PWM_OUTChannel3,500,800);		//PWM�趨-20161127�汾--����
	}
	return;
#endif
#if 0
	RxNum=RS485_ReadBufferIDLE(&RS485,RevBuffe);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(RxNum	==	100)
	{
		RS485Time	=	0;
		PWM_OUT(TIM3,PWM_OUTChannel3,500,800);		//PWM�趨-20161127�汾--����
	}
	if(RS485Time++>=1000)
	{
		RS485Time	=	0;
		RS485_DMASend(&RS485,(u32*)TxdBuffe,100);	//RS485-DMA���ͳ���
		PWM_OUT(TIM3,PWM_OUTChannel3,500,1);		//PWM�趨-20161127�汾--����
	}
#else
	RxNum=RS485_ReadBufferIDLE(&RS485,RevBuffe);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(RxNum)
	{
		RS485Time	=	0;
		RS485FLG	=	1;
		RSRLen	=	RxNum;
		RS485_DMASend(&RS485,RevBuffe,100);	//RS485-DMA���ͳ���
		PWM_OUT(TIM3,PWM_OUTChannel3,500,800);		//PWM�趨-20161127�汾--����
	}
	if(RS485FLG	==	1)
	{
		if(RS485Time++>=500)
		{
			RS485Time	=	0;
			RS485_DMASend(&RS485,RevBuffe,RSRLen);	//RS485-DMA���ͳ���
			RSRLen	=	0;
			RS485FLG	=	0;
			PWM_OUT(TIM3,PWM_OUTChannel3,500,800);		//PWM�趨-20161127�汾--����
		}
	}
	
#endif
}

/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void SwitchID_Configuration(void)
{
	SWITCHID.NumOfSW	=	8;
	
	SWITCHID.SW1_PORT	=	GPIOD;
	SWITCHID.SW1_Pin	=	GPIO_Pin_2;
	
	SWITCHID.SW2_PORT	=	GPIOA;
	SWITCHID.SW2_Pin	=	GPIO_Pin_15;
	
	SWITCHID.SW3_PORT	=	GPIOA;
	SWITCHID.SW3_Pin	=	GPIO_Pin_12;
	
	SWITCHID.SW4_PORT	=	GPIOA;
	SWITCHID.SW4_Pin	=	GPIO_Pin_11;
	
	SWITCHID.SW5_PORT	=	GPIOA;
	SWITCHID.SW5_Pin	=	GPIO_Pin_8;
	
	SWITCHID.SW6_PORT	=	GPIOB;
	SWITCHID.SW6_Pin	=	GPIO_Pin_15;
	
	SWITCHID.SW7_PORT	=	GPIOB;
	SWITCHID.SW7_Pin	=	GPIO_Pin_14;
	
	SWITCHID.SW8_PORT	=	GPIOB;
	SWITCHID.SW8_Pin	=	GPIO_Pin_13;
	
	SwitchIdInitialize(&SWITCHID);							//
	
//	SwitchID	=	SWITCHID_Read(&SWITCHID);		//
}
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void SwitchID_Server(void)
{
	u8 temp=0;	//���뿪����ʱ��ַ
	temp	=	SWITCHID_Read(&SWITCHID);		//��ȡ��ַ
	if(SwitchID	!=	temp)
	{
		u8 AddrH	=	0,AddrL	=	0;
		u32	w=0,n=0,d=0;
//	w	=	1;n	=	0;
//	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//	w	+=	1;n	+=	8;
//	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//	w	+=	1;n	+=	8;
//	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//	w	+=	1;n	+=	8;
//	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//	w	+=	1;n	+=	8;
//	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//	w	+=	1;n	+=	8;
//	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//	w	+=	1;n	+=	8;
//	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//	w	+=	1;n	+=	8;
//	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		
		w	=	12345678;
		LCD_Printf(0		,0	,32	,0,"��ţ�");				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		LCD_Printf(0		,32	,32	,0,"���룺");				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		LCD_Printf(96	,0	,32	,0,"%8d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		w	=	0;
		SwitchID	=	temp;
		for(n=0;n<8;n++)
		{
			if((temp&0x01)	==0x01)
			{
				d	=	1;
			}
			else
			{
				d	=	0;
			}
			temp>>=1;
			LCD_Printf(n*16+96		,32	,32	,0,"%d",d);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		}
		AddrH	=	(SwitchID>>4)&0x0F;
		AddrL	=	(SwitchID>>0)&0x0F;
		LCD_Printf(0		,64	,32	,0,"��%0.2d��λ%0.2d",AddrH,AddrL);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
	}
}



/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void LCD_Configuration(void)
{	
	//=======================LCD�˿�
	LCDPortDef	*LcdPort	=	&sLCD.Port;
	
	LcdPort->sCS_PORT	=	LcdCsPort;
	LcdPort->sCS_Pin	=	LcdCsPin;

	LcdPort->sDC_PORT	=	LcdDcPort;
	LcdPort->sDC_Pin	=	LcdDcPin;
	
	LcdPort->sWR_PORT	=	LcdWrPort;
	LcdPort->sWR_Pin	=	LcdWrPin;
	
	LcdPort->sRD_PORT	=	LcdRdPort;
	LcdPort->sRD_Pin	=	LcdRdPin;
	
	LcdPort->sREST_PORT	=	LcdRestPort;
	LcdPort->sREST_Pin	=	LcdRestPin;
	
	LcdPort->sBL_PORT		=	LcdBlPort;
	LcdPort->sBL_Pin		=	LcdBlPin;
	
	LcdPort->sTE_PORT		=	LcdTePort;
	LcdPort->sTE_Pin		=	LcdTePin;
	
	LcdPort->sDATABUS_PORT	=	LcdBusPort;
	LcdPort->sDATABUS_Pin		=	LcdBusPin;
	
	sLCD.Data.BColor	=	LCD565_BLACK;
	sLCD.Data.PColor	=	LCD565_RED;
	sLCD.Flag.Rotate	=	Draw_Rotate_270D;
	
	//=======================�ֿ�˿�
	sLCD.GT32L32.SPI.Port.SPIx	=	SPI1;
	sLCD.GT32L32.SPI.Port.CS_PORT	=	GPIOA;
	sLCD.GT32L32.SPI.Port.CS_Pin		=	GPIO_Pin_4;
	sLCD.GT32L32.SPI.Port.SPI_BaudRatePrescaler_x=SPI_BaudRatePrescaler_2;
	
	LCD_Initialize(&sLCD);
}

/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void RS485_Configuration(void)
{
	RS485.USARTx=USART2;
	RS485.RS485_CTL_PORT=GPIOA;
	RS485.RS485_CTL_Pin=GPIO_Pin_1;
	
	RS485_DMA_ConfigurationNR	(&RS485,19200,256);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void LCD_PowerUp(void)
{	
//	u16 i=0,j=0;
	
	
	
	
	
//	R61509V_DrawDot(50,50,0X5458);
//	R61509V_DrawDot_big(50,50,0X5458);
//	R61509V_DrawLine(0,0,240,400,0X5458);
//	R61509V_DrawLine(0,400,240,0,0X5458);
	
//	R61509V_DrawCircle(120,100, 50,0,0X5458);				//��һ��Բ�ο�
	
//	R61509V_DrawCircle(120,200, 120,0,0X01CF);				//��һ��Բ�ο�
	
//	R61509V_DrawCircle(120,400, 80,0,0X5458);				//��һ��Բ�ο�
//	
//	R61509V_DrawCircle(120,0, 80,0,0X5458);					//��һ��Բ�ο�
//	
//	R61509V_DrawCircle(120,200, 80,1,0X5458);				//��һ��Բ�ο�
//	
//	R61509V_DrawCircle(200,120, 50,0,0xF800);				//��һ��Բ�ο�
//	
//	R61509V_DrawCircle(120,200, 30,1,0X07FF);				//��һ��Բ�ο�
	
//	R61509V_DrawRectangle(3,3,237,397,0x07E0);			//��һ�����ο�
//	R61509V_DrawRectangle(4,4,236,396,0x07E0);			//��һ�����ο�
//	R61509V_DrawRectangle(5,5,235,395,0x07E0);			//��һ�����ο�
//	R61509V_DrawRectangle(6,6,234,394,0x07E0);			//��һ�����ο�
//	
//	R61509V_DrawRectangle(7,7,233,393,0X07FF);			//��һ�����ο�
//	R61509V_DrawRectangle(8,8,232,392,0X07FF);			//��һ�����ο�
//	R61509V_DrawRectangle(9,9,231,391,0X07FF);			//��һ�����ο�
//	R61509V_DrawRectangle(10,10,230,390,0X07FF);		//��һ�����ο�
//	R61509V_DrawRectangle(11,11,229,389,0X07FF);		//��һ�����ο�
//	R61509V_DrawRectangle(12,12,228,388,0X07FF);			//��һ�����ο�
//	R61509V_DrawLine(12,12,228,388,0X5458);						//AB �������껭һ��ֱ��
//	R61509V_DrawLine(12,388,228,12,0X5458);						//AB �������껭һ��ֱ��
	
//	R61509V_DrawLine(0,10,400,10,0X5458);						//AB �������껭һ��ֱ��
//	
//	R61509V_DrawLine(0,20,400,20,0X5458);						//AB �������껭һ��ֱ��
	
//	R61509V_DrawLine(0,100,160,100,0X5458);						//AB �������껭һ��ֱ��
//	
//	R61509V_DrawLine(80,0,80,400,0X5458);						//AB �������껭һ��ֱ��
	
//	R61509V_DrawRectangle(10,10,390,230,0X07FF);		//��һ�����ο�
	
//	R61509V_DrawCircle(200,120, 10,1,0X5458);					//��һ��Բ�ο�
//	
//	R61509V_DrawCircle(200,120, 80,0,0X5458);					//��һ��Բ�ο�
	
//	R61509V_DrawLine(240,240,20,240,0X5458);						//AB �������껭һ��ֱ��
	
//	R61509V_ShowChar(1,1,32,100,zimo);								//��ͨ�ֿ���Գ���
//	
//	R61509V_ShowCharT(50,50,15,0);
//	R61509V_ShowEn(200,120,12);
	
//	PL010V13_PrintfString(0		,16	,16	,"����ҩ��λ��%3d",RevBuffe[0]);				//��ߵ�ʡ�Ժž��ǿɱ����
//	PL010V13_PrintfString(0		,32	,16	,"����ҩ������%3d",RevBuffe[1]);				//��ߵ�ʡ�Ժž��ǿɱ����

//	PL010V13_PrintfString(0		,0	,32	,"��λ-%2d����-%2d",RevBuffe[0],RevBuffe[1]);				//��ߵ�ʡ�Ժž��ǿɱ����
	
//	PL010V13_PrintfString(0		,0	,32	,"����ҩ������%2d",RevBuffe[1]);				//��ߵ�ʡ�Ժž��ǿɱ����
//	
//	
//	PL010V13_PrintfString(0		,100	,32	,"����ҩ������%2d",RevBuffe[1]);				//��ߵ�ʡ�Ժž��ǿɱ����
//	PL010V13_PrintfString(0		,100	,32	,"����ҩ������%2d",RevBuffe[1]);				//��ߵ�ʡ�Ժž��ǿɱ����
//	
	#if 0
	PL010Delay(0xFFFF);
	PL010V13_PrintfString(1		,0	,16	,"����ҩ��λ��%2d",RevBuffe[0]);				//��ߵ�ʡ�Ժž��ǿɱ����
	PL010V13_PrintfString(1		,0	,16	,"����ҩ��λ��%2d",RevBuffe[0]);				//��ߵ�ʡ�Ժž��ǿɱ����
	PL010Delay(0xFFFF);
	PL010V13_PrintfString(1		,20	,16	,"����ҩ������%2d",RevBuffe[1]);				//��ߵ�ʡ�Ժž��ǿɱ����
	PL010Delay(0xFFFF);
	PL010V13_PrintfString(1		,40	,16	,"�ѷ�ҩ������%2d",RevBuffe[1]);				//��ߵ�ʡ�Ժž��ǿɱ����
	PL010Delay(0xFFFF);
	
//	PL010V13_PrintfString(0		,60	,16	,"�ܹ�����������%4d",SumFQ);				//�ܹ���ҩ��������
//	PL010V13_PrintfString(0		,80	,16	,"�ܹ���ҩ������%4d",SumFed);				//�ܹ��ѷ�ҩ����
	PL010V13_PrintfString(1		,100	,16	,"����״̬��");				//����״̬
	PL010Delay(0xFFFF);
	
	PL010V13_PrintfString(1		,160	,16	,"��ʾ������3���ȡ��ҩͷ");				//����״̬
	PL010Delay(0xFFFF);
	

	//ƽ����
	R61509V_DrawLine(221,0,221,400,R61509V_WHITE);						//AB �������껭һ��ֱ��
	R61509V_DrawLine(201,0,201,400,R61509V_WHITE);						//AB �������껭һ��ֱ��
	R61509V_DrawLine(181,0,181,400,R61509V_WHITE);						//AB �������껭һ��ֱ��
	R61509V_DrawLine(161,0,161,400,R61509V_WHITE);						//AB �������껭һ��ֱ��
	//��ֱ��--��
	R61509V_DrawLine(161,200,221,200,R61509V_WHITE);					//AB �������껭һ��ֱ��
	//��ֱ��--��
	R61509V_DrawLine(161,250-1,221,250-1,R61509V_WHITE);			//AB �������껭һ��ֱ��
	R61509V_DrawLine(161,300-1,221,300-1,R61509V_WHITE);			//AB �������껭һ��ֱ��
	R61509V_DrawLine(161,350-1,221,350-1,R61509V_WHITE);			//AB �������껭һ��ֱ��
	R61509V_DrawLine(161,400-1,221,400-1,R61509V_WHITE);			//AB �������껭һ��ֱ��
	
	//��ֱ��--��
	R61509V_DrawLine(161,1,221,1,					R61509V_WHITE);							//AB �������껭һ��ֱ��
	R61509V_DrawLine(161,51,221,51,				R61509V_WHITE);						//AB �������껭һ��ֱ��
	R61509V_DrawLine(161,101,221,101,			R61509V_WHITE);					//AB �������껭һ��ֱ��
	R61509V_DrawLine(161,151,221,151,			R61509V_WHITE);					//AB �������껭һ��ֱ��

	//���
	R61509V_Fill(2,180,50,200-2,R61509V_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
	
	
	
	
//	R61509V_DrawLine(10,200,10,280,R61509V_WHITE);						//AB �������껭һ��ֱ��
	
	#else
//	R61509V_Clean(R61509V_WHITE);			//�����Ļ����------
//	PL010Delay(0x8FFFFF);
	LCD_Clean(LCD565_BLUE);			//�����Ļ����------
	PL010Delay(0x8FFFFF);
	LCD_Clean(LCD565_GRED);			//�����Ļ����------
	PL010Delay(0x8FFFFF);
	LCD_Clean(LCD565_BLACK);			//�����Ļ����------
	PL010Delay(0x8FFFFF);
	LCD_Clean(LCD565_WHITE);			//�����Ļ����------

//	PL010Delay(0xFFFF);
//	PL010V13_PrintfString(0	,0,16	,"TEST");				//����״̬
	#endif
//	R61509V_Clean(R61509V_BLACK);			//�����Ļ����------
	
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void LCD_Display(void)
{
//	LCD_Printf(0		,0	,32	,0,"��ʾ����!!@#$%^&*");				//����״̬
//	LCD_Printf(0		,32	,32	,0,"��ʾ����!!@#$%^&*");				//����״̬
//	LCD_Printf(0		,64	,32	,0,"��ʾ����!!@#$%^&*");				//����״̬
//	LCD_Printf(0		,96	,32	,0,"��ʾ����!!@#$%^&*");				//����״̬
//	LCD_Printf(0		,128,32	,0,"��ʾ����!!@#$%^&*");				//����״̬
	
#if 1
	DSPTime++;
	if(DSPTime>500)
		DSPTime=0;
	if(DSPTime%500==0)
	{
		LCD_Clean(LCD565_GRED);			//�����Ļ����------
//		LCD_Printf(0		,120	,32	,0,"��ʾ����!!��������@#$%^&*");				//����״̬
	}
	if(DSPTime%500==100)
	{
		LCD_Clean(LCD565_GREEN);			//�����Ļ����------
//		DSPTime	=	0;
//		LCD_Printf(0		,120	,32	,0,"��ʾ����!!��������@#$%^&*");				//����״̬
	}
//	if(DSPTime%500==200)
//	{
//		LCD_Clean(LCD565_BLACK);			//�����Ļ����------
////		LCD_Printf(0		,120	,32	,0,"��ʾ����!!��������@#$%^&*");				//����״̬
//	}
//	if(DSPTime%500==300)
//	{
//		LCD_Clean(LCD565_LGRAY);			//�����Ļ����------
////		LCD_Printf(0		,120	,32	,0,"��ʾ����!!��������@#$%^&*");				//����״̬	
//	}
//	if(DSPTime%500==400)
//	{
//		LCD_Clean(LCD565_WHITE);			//�����Ļ����------
////		LCD_Printf(0		,120	,32	,0,"��ʾ����!!��������@#$%^&*");				//����״̬
//	}

#endif
	
#if 0		
	DSPTime++;
	if(DSPTime>500)
	{
		DSPTime=0;
		LCD_WXS();		//λ��ʾ
		LCD_DDSP();		//��ʾ�ܹ������������ѷ�����
	}	
	LCD_WS();		//λ��˸
	
	if(LCDTime>=1000)
	{			
		LCDTime=0;		
	}	
	
	RxNum=RS485_ReadBufferIDLE(&RS485,(u32*)RevBuffe,(u32*)RxdBuffe);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(RxNum==4&&RevBuffe[0]==0x00&&RevBuffe[1]==0xFF)		//RS485���յ�����
	{
		NumFW=RevBuffe[2];
		LCD_Printf(96		,0	,16	,0,"%2d",RevBuffe[2]);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		LCD_Printf(96		,20	,16	,0,"%2d",RevBuffe[3]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//		PL010V13_PrintfString(192		,68	,16	,"%2d",RevBuffe[1]);				//�ѷ�ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	}
	else if(RxNum==3&&RevBuffe[0]==0x02)		//RS485���յ�����
	{
		SumFQ[RevBuffe[1]-1]+=RevBuffe[2];
		NumFW=RevBuffe[1];
		LCD_Printf(96		,0	,16	,0,"%2d",RevBuffe[1]);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		LCD_Printf(96		,20	,16	,0,"%2d",RevBuffe[2]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		LCD_Printf(96		,40	,16	,0,"%2d",0);									//�ѷ�ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		
//		PL010V13_PrintfString(0		,60	,16	,0,"�ܹ�����������%4d",SumFQ);				//�ܹ���ҩ��������
		LCD_Printf(0		,120	,16	,0,"���ڷ�ҩ��������������������");				//����״̬
	}
	else if(RxNum==4&&RevBuffe[0]==0x82)		//RS485���յ�����
	{
		SumFed[RevBuffe[1]-1]+=RevBuffe[2];
		LCD_Printf(96		,0	,16	,0,"%2d",RevBuffe[1]);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		LCD_Printf(96		,20	,16	,0,"%2d",0);									//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		LCD_Printf(96		,40	,16	,0,"%2d",RevBuffe[2]);				//�ѷ�ҩ��������ߵ�ʡ�Ժž��ǿɱ����		
		
//		PL010V13_PrintfString(112		,80	,16	,"%4d",SumFed);					//�ܹ��ѷ�ҩ����
		
	}
	else if(RxNum==6&&RevBuffe[0]==0x81)	//��λ��Ϣ0x01--��ȡ,0x81--�ϱ�
	{
		Onlinede=RevBuffe[4];
	}
	else if(RxNum==1&&RevBuffe[0]==0x01)	//��λ��Ϣ0x01--��ȡ,0x81--�ϱ�
	{
		LCD_Printf(0		,120	,16		,0,"��ȡ��λ��Ϣ������");				//����״̬
		NumFW=0;
		Onlinede=0;
		memset(SumFed,0x00,8);
		memset(SumFQ,0x00,8);

		LCD_Printf(96		,0	,16	,0,"%2d",0);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		LCD_Printf(96		,20	,16	,0,"%2d",0);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	}
	
	for(DelayTime=0;DelayTime<1000;DelayTime++)
	{
		
	}
	if(RxNum&&(RevBuffe[0]==0x82))		//����״̬��ʾ
	{
		switch(RevBuffe[3])
		{
			case 0x00:	LCD_Printf(0		,120	,16	,0,"����������������������������");				//����״̬
				break;
			case 0x80:	LCD_Printf(0		,120	,16	,0,"ҩƷ����ס������������������");				//����״̬
				break;
			case 0x81:	LCD_Printf(0		,120	,16	,0,"ȱҩ������������������������");				//����״̬
				break;
			case 0x82:	LCD_Printf(0		,120	,16	,0,"�ȴ���������ʱ������������");				//����״̬
				break;
			case 0xC0:	LCD_Printf(0		,120	,16	,0,"��Ԫ�������ͨ���쳣��������");				//����״̬
				break;
			case 0xC1:	LCD_Printf(0		,120	,16	,0,"��ҩ�������ͨ���쳣��������");				//����״̬
				break;
		}
	}
#endif
}

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PL010V13_DISPLAY(void)
{
//	R61509V_DrawPixelEx( 100, 100,LCD_FORE_COLOR);
//	R61509V_DrawHLine( 10, 100, 200, LCD_FORE_COLOR);
}
void PL010Delay(u32 time)
{	
	while(time--);
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void LCD_WS(void)		//λ��˸
{
	if(NumFW)		//��λ��˸	DSPTime
	{
		if(NumFW==1&&(Onlinede>>0&0x01))
		{
			if(DSPTime==500)
			{
				LCD_Fill(2,180,50,200-2,LCD565_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				LCD_Fill(2,180,50,200-2,LCD565_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}
		}
		else if(NumFW==2&&(Onlinede>>1&0x01))
		{
			if(DSPTime==500)
			{
				LCD_Fill(52,180,100,200-2,LCD565_GBLUE);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				LCD_Fill(52,180,100,200-2,LCD565_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}			
		}
		else if(NumFW==3&&(Onlinede>>2&0x01))
		{
			if(DSPTime==500)
			{
				LCD_Fill(102,180,150,200-2,LCD565_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				LCD_Fill(102,180,150,200-2,LCD565_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}			
		}
		else if(NumFW==4&&(Onlinede>>3&0x01))
		{
			if(DSPTime==500)
			{
				LCD_Fill(152,180,200,200-2,LCD565_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				LCD_Fill(152,180,200,200-2,LCD565_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}			
		}
		else if(NumFW==5&&(Onlinede>>4&0x01))
		{
			if(DSPTime==500)
			{
				LCD_Fill(202,180,250,200-2,LCD565_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				LCD_Fill(202,180,250,200-2,LCD565_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}			
		}
		else if(NumFW==6&&(Onlinede>>5&0x01))
		{
			if(DSPTime==500)
			{
				LCD_Fill(252,180,300,200-2,LCD565_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				LCD_Fill(252,180,300,200-2,LCD565_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}			
		}
		else if(NumFW==7&&(Onlinede>>6&0x01))
		{
			if(DSPTime==500)
			{
				LCD_Fill(302,180,350,200-2,LCD565_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				LCD_Fill(302,180,350,200-2,LCD565_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}			
		}
		else if(NumFW==8&&(Onlinede>>7&0x01))
		{
			if(DSPTime==500)
			{
				LCD_Fill(352,180,400-2,200-2,LCD565_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				LCD_Fill(352,180,400-2,200-2,LCD565_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}			
		}
	}
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void LCD_WXS(void)		//λ��ʾ
{
		if((Onlinede>>0&0x01)==0x01)
		{
			LCD_Fill(2,180,50,200-2,LCD565_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		else
		{
			LCD_Fill(2,180,50,200-2,LCD565_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>1&0x01)==0x01)
		{
			LCD_Fill(52,180,100,200-2,LCD565_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		else
		{
			LCD_Fill(52,180,100,200-2,LCD565_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>2&0x01)==0x01)
		{
			LCD_Fill(102,180,150,200-2,LCD565_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		else
		{
			LCD_Fill(102,180,150,200-2,LCD565_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>3&0x01)==0x01)
		{
			LCD_Fill(152,180,200,200-2,LCD565_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		else
		{
			LCD_Fill(152,180,200,200-2,LCD565_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>4&0x01)==0x01)
		{
			LCD_Fill(202,180,250,200-2,LCD565_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		else
		{
			LCD_Fill(202,180,250,200-2,LCD565_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>5&0x01)==0x01)
		{
			LCD_Fill(252,180,300,200-2,LCD565_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		else
		{
			LCD_Fill(252,180,300,200-2,LCD565_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>6&0x01)==0x01)
		{
			LCD_Fill(302,180,350,200-2,LCD565_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		else
		{
			LCD_Fill(302,180,350,200-2,LCD565_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>7&0x01)==0x01)
		{
			LCD_Fill(352,180,400-2,200-2,LCD565_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		else
		{
			LCD_Fill(352,180,400-2,200-2,LCD565_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void LCD_DDSP(void)		//��ʾ�ܹ������������ѷ�����
{
//	u8 i=0;
//	u8 w=2;
	//����������ʾ
//	for(i=0;i<8;i++)
//	{
//		PL010V13_PrintfString(w,200,16	,"%5d",SumFQ[i]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//		w+=50;
//	}
	//����������ʾ
	LCD_Printf(2,200,16	,0,"%5d",SumFQ[0]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	LCD_Printf(52,200,16	,0,"%5d",SumFQ[1]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	LCD_Printf(102,200,16	,0,"%5d",SumFQ[2]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	LCD_Printf(152,200,16	,0,"%5d",SumFQ[3]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	LCD_Printf(201,200,16	,0,"%5d",SumFQ[4]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	LCD_Printf(251,200,16	,0,"%5d",SumFQ[5]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	LCD_Printf(301,200,16	,0,"%5d",SumFQ[6]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	LCD_Printf(351,200,16	,0,"%5d",SumFQ[7]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����

	//�ѷ�������ʾ	
	LCD_Printf(2,220,16		,0,"%5d",SumFed[0]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	LCD_Printf(52,220,16,0,"%5d",SumFed[1]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	LCD_Printf(102,220,16	,0,"%5d",SumFed[2]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	LCD_Printf(152,220,16	,0,"%5d",SumFed[3]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	LCD_Printf(201,220,16	,0,"%5d",SumFed[4]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	LCD_Printf(251,220,16	,0,"%5d",SumFed[5]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	LCD_Printf(301,220,16	,0,"%5d",SumFed[6]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	LCD_Printf(351,220,16	,0,"%5d",SumFed[7]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
}
#endif