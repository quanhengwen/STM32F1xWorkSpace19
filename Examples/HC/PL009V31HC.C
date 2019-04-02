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

#ifdef PL009V31HC				//����LCD��---˫����

#include "PL009V31HC.H"

#include "R61509V.h"
#include "CS5530.H"


#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_PWM.H"
#include "STM32_USART.H"


#include "SWITCHID.H"

#include "string.h"				//�����ڴ��������ͷ�ļ�
#include "stdlib.h"				//�����ڴ��������ͷ�ļ�
#include "stm32f10x_dma.h"

#define TestChN1		//��ͨ������,���κ�Ϊ˫ͨ��
#define TestFloat		//δ����ͨ�������ã����κ�δ����ͨ���������ر�,˫ͨ��ʱ��Ч


//#define	Master	1		//0--�ӻ���1-����
u16	DelayTime=0x1000;
u16	LCDTime=0;
u16	DSPTime=0;

u8 hour=23,min=15,second=40;
u16 millisecond=0;

//GT32L32Def 	GT32L32;
//char	Char_Buffer[256];		//��¼format����


RS485Def  RS485;
#define	Rs485Size	256
u8 RxdBuffe[Rs485Size]={0};
u8 RevBuffe[Rs485Size]={0};
u8 TxdBuffe[Rs485Size]={0};
u8 RxFlg=0;
u16 RxNum=0;
u32	Rs485_Time	=	0;
u8	SlaveID	=	0;
u32 Rev_ADC	=	0;
u8	Rev_ID	=	0;



SwitchDef	SWITCHID;
u8 SwitchID=0;	//���뿪�ص�ַ


LCDDef	sLCD;


//R61509VDef R61509V;
u8	DspFlg	=	0;

u16 BKlight	=	0;

u8 WriteFlag	=	0;
u16	row	=	1;



CS5530Def CS5530;
CS5530Def CS5530B;
u32	CS5530_Time	=	0;
u32	CS5530_ADC_CMP			=	0;
u32 CS5530_ADC_Value		=0xFFFFFFFF;
u32 CS5530_ADC_ValueB		=0xFFFFFFFF;
u32 CS5530_ADC_Valuebac	=0xFFFFFFFF;

u32	WeigthARR[100]	=	{0};
u8	DspFlg1	=	0;
u32	We1	=	0;
u8	NuW	=	0;
u32	BsW	=	0;	//����
u32 SsW	=	0;	//����
u8	St	=	0;	//����
u8	SN	=	0;	//����
u8	SNb	=	0;	//��������

u16 weighVar[20]={0};
u8	VarN	=	0;

u32 Rand	=	0;
//============LCD�������Ա���
u16 HX	=	0;
u16	HY	=	0;
unsigned short	Color	=	0;
u8 Rxdata	=0;
u32 SYSTIME	=	0;
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PL009V31HC_Configuration(void)
{
	SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H
	
	GPIO_DeInitAll();							//�����е�GPIO�ر�----V20170605	
	
//	SwitchID_Configuration();
//	
//	CS5530_Configuration();
	
	RS485_Configuration();
	
	LCD_Configuration();	
	
	LCD_Printf(0,32,32,LCD565_CYAN,"LCD��ʼ�����");  //��ߵ�ʡ�Ժž��ǿɱ����
	
	SysTick_Configuration(1000);	//ϵͳ���ʱ������72MHz,��λΪuS
	
//	IWDG_Configuration(2000);			//�������Ź�����---������λms	

	PWM_OUT(TIM2,PWM_OUTChannel1,500,900);	//PWM�趨-20161127�汾--����ָʾ��
	USART_DMA_ConfigurationNR	(USART1,115200,64);	//USART_DMA����--��ѯ��ʽ�������ж�
//	PWM_OUT(TIM2,PWM_OUTChannel4,500,200);		//PWM�趨-20161127�汾--����
//	LCD_Clean(LCD565_RED);			//�����Ļ����--
//	LCD_Printf(100,100,32,"ʱ��");					//��ߵ�ʡ�Ժž��ǿɱ����
//	LCD_Printf(100,130,32,"%02d:",hour);		//��ߵ�ʡ�Ժž��ǿɱ����
//	LCD_Printf(148,130,32,"%02d:",min);			//��ߵ�ʡ�Ժž��ǿɱ����
//	LCD_Printf(196,130,32,"%02d",second);		//��ߵ�ʡ�Ժž��ǿɱ����
//	R61509V_DrawCircle(200,120, 100, 1, R61509V_YELLOW );		//��һ��Բ�ο�
	
//	Display.Init	=	PL010V15_Configuration;
//	GPIO_Configuration_OPP50(GPIOB,GPIO_Pin_12);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
//	GPIO_SetBits(GPIOB,GPIO_Pin_12);
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PL009V31HC_Server(void)
{	
//	GPIO_Toggle	(GPIOB,	GPIO_Pin_3);		//��GPIO��Ӧ�ܽ������ת----V20170605
//	IWDG_Feed();				//�������Ź�ι��
//	RS485_Server();			//ͨѶ����---������Ϣ�Ľ����뷢��
	LCD_Server();				//��ʾ�������
//	CS5530_Server();	//���ط���ADֵ��������ȡ�ȶ�ֵ
//	SwitchID_Server();	//���뿪�ش���--��̬���²����ַ
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
void CS5530_Server(void)		//���ط���ADֵ��������ȡ�ȶ�ֵ
{
	CS5530_Process(&CS5530);
	
	if((CS5530_ADC_Value	!=	CS5530.Data.WeighLive)&&(0	!=	CS5530.Data.WeighLive)&&(0xFFFFFFFF	!=	CS5530.Data.WeighLive))
	{
		CS5530_ADC_Value	=	CS5530.Data.WeighLive;
		CS5530_ADC_Value	=	CS5530.Data.WeighLive;
		USART_DMAPrintf	(USART1,"CH1:%0.8X\r\n",CS5530_ADC_Value>>2);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
		return;
	}
#ifndef TestChN1	//˫ͨ��
//	return;
	CS5530_Process(&CS5530B);
	if((CS5530_ADC_Value	!=	CS5530B.Data.WeighLive)&&(0	!=	CS5530B.Data.WeighLive)&&(0xFFFFFFFF	!=	CS5530B.Data.WeighLive))
	{
		CS5530_ADC_ValueB	=	CS5530B.Data.WeighLive;
		CS5530_ADC_ValueB	=	CS5530B.Data.WeighLive;
		USART_DMAPrintf	(USART1,"CH2:%0.8X\r\n",CS5530_ADC_ValueB>>2);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
		return;
	}
#endif
//	if(CS5530_Time++>=50)		//1����
//	{		
//		CS5530_Time=0;
//		CS5530_ADC_Valuebac=CS5530_ReadData(&CS5530);	//��ȡADֵ���������0xFFFFFFFF,��δ��ȡ��24λADֵ
//		if(CS5530_ADC_Valuebac!=0xFFFFFFFF)
//		{
//			
////			CS5530_ADC_Valuebac>>=5;		//19λ����
//			
//			if(CS5530_ADC_Value==CS5530_ADC_Valuebac)
//			{
//				DspFlg	=	0;
//			}
//			if(CS5530_ADC_Value>=CS5530_ADC_Valuebac)
//			{
//				CS5530_ADC_CMP	=	CS5530_ADC_Value-CS5530_ADC_Valuebac;
//			}
//			else
//			{
//				CS5530_ADC_CMP	=	CS5530_ADC_Valuebac-CS5530_ADC_Value;				
//			}
//			DspFlg	=	1;
//			CS5530_ADC_Value	=	CS5530_ADC_Valuebac;
//			
//			if(CS5530_ADC_CMP>100)
//			{
//				NuW	=	0;
//				return;
//			}
//			WeigthARR[NuW]	=	CS5530_ADC_Value;
//			if(NuW++>=20-1)
//			{
//				long	temp	=	0;
//				//We1
//				
//				u8 i	=	0;
//				
//				temp	=	CS5530_GetWeightUseMedinaFilter(&CS5530);
//				
//				if(temp>200)
//				{
//					NuW	=	0;
//					return;
//				}
//				NuW	=	0;
//				We1	=	WeigthARR[i];
//				for(i=1;i<20;i++)
//				{
//					We1	=	(We1+WeigthARR[i])/2;
//				}
//				if(St	==	1)
//				{
//					St=0;
//					BsW	=	We1;
//				}
//				else if(St	==	2)
//				{
//					SsW	=	(We1	-	BsW)/10;
//					St	=	3;
//					SN	=	10;
//					SNb	=	10;
//				}
//				else if(St	==	3)
//				{
//					SN	=	(We1-BsW+500)/SsW;	//��������
//					if(SN!=SNb)
//					{
//						SNb	=	SN;
//						PL010V13_PrintfString(0		,128,32	,"����%0.3d",SNb);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//					}
//				}
//				DspFlg1	=	1;
//			}
//				
//		}
//		else
//		{
////			We1	=0;
////			NuW	=0;
//		}
//	}
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
void LCD_Server(void)			//��ʾ�������
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
//		temp	=	8*3*RxNum;
//		if(0<temp%400)
//		{
//			temp	=	(temp/400+1)*16;
//		}
//		else
//		{
//			temp	=	(temp/400)*16;
//		}
		if(row>=240-32)
		{
			LCD_Clean(LCD565_BLACK); 					//�����Ļ����
			row	=	0;
		}
		
		LCD_ShowHex(0,row,16,PenColor,RxNum,8,RxdBuffe);                //��ʾʮ����������
		temp	=	8*3*RxNum;
		if(0<temp%400)
		{
			row	+=	(temp/400+1)*16;
		}
		else
		{
			row	+=	(temp/400)*16;
		}
		if(row>=240-32)
		{
//			row=0;
//			LCD_Clean(LCD565_BLACK); 					//�����Ļ����
		}
//		LCD_Show(0,100,32,RxNum,RxdBuffe);
//		PWM_OUT(TIM3,PWM_OUTChannel3,500,800);		//PWM�趨-20161127�汾--����
	}
	return;
#endif
#if 0
	u32 LCDTime=100;
//	u8 i	=	0;
	for(Color	=	0;Color<=0xFFFF;)
	{
		LCD_Fill(10,10,380,20,Color);			//�����Ļ����--
//		LCD_Clean(Color);			//�����Ļ����--
		SysTick_DeleymS(LCDTime);				//SysTick��ʱnmS
		Color+=50;
//		GPIO_Toggle	(GPIOB,	GPIO_Pin_3);		//��GPIO��Ӧ�ܽ������ת----V20170605
	}
#endif
#if 0
	millisecond++;
	if(millisecond>=999)
	{
//		SSD1963_DrawRectangle(100,100,110,110);
		millisecond=0;
		second++;
		if(second>59)
		{
			second=0;
			min++;			
			if(min>59)
			{
				min=0;
				hour++;
				if(hour>23)
				{
					hour=0;
				}
				LCD_Printf(100,130,32,"%02d:",hour);		//��ߵ�ʡ�Ժž��ǿɱ����
				
			}
			LCD_Printf(148,130,32,"%02d:",min);		//��ߵ�ʡ�Ժž��ǿɱ����
		}
		LCD_Printf(196,130,32,"%02d",second);		//��ߵ�ʡ�Ժž��ǿɱ����
	}
#endif
#if 0		//��Բ
	unsigned short i	=	0;
	for(i=1;i<100;i++)
	{
		LCD_DrawCircle(200,	120, i, 0, LCD565_WHITE );			//��һ��Բ�ο�	
	}
	for(i=100;i>0;i--)
	{
		LCD_DrawCircle(200,	120, i, 0, LCD565_BLACK );			//��һ��Բ�ο�	
	}
	LCD_DrawCircle(50,	60, 50, 0, LCD565_YELLOW );			//��һ��Բ�ο�	
	LCD_DrawCircle(340,	60,	50, 0, LCD565_YELLOW );			//��һ��Բ�ο�	
	LCD_DrawCircle(50,	180, 50, 0, LCD565_YELLOW );			//��һ��Բ�ο�	
	LCD_DrawCircle(340,	180, 50, 0, LCD565_YELLOW );			//��һ��Բ�ο�	
	LCD_DrawCircle(200,	120, 100, 0, LCD565_YELLOW );			//��һ��Բ�ο�	
	
	SysTick_DeleymS(500);					//SysTick��ʱnmS
	LCD_Clean(LCD565_BLACK);		//�����Ļ����--
#endif
#if 0		//��Բ
//	LCD_Fill(10,10,50,50,LCD565_YELLOW);	//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
	
//	LCD_DrawRectangle(10,130,60,180,LCD565_YELLOW);			//��һ�����ο�
	
//	R61509V_ShowChar(10,120,32,5,"sdsdf");
	
	LCD_DrawCircle(50,	50, 50, 1, LCD565_YELLOW );			//��һ��Բ�ο�	
	LCD_DrawCircle(350,	50,	50, 1, LCD565_YELLOW );			//��һ��Բ�ο�	
	LCD_DrawCircle(50,	190, 50, 1, LCD565_YELLOW );			//��һ��Բ�ο�	
	LCD_DrawCircle(350,	190, 50, 1, LCD565_YELLOW );			//��һ��Բ�ο�	
	LCD_DrawCircle(200,	120, 100, 1, LCD565_YELLOW );			//��һ��Բ�ο�	
	
	SysTick_DeleymS(500);					//SysTick��ʱnmS
	LCD_Clean(LCD565_RED);		//�����Ļ����--
#endif
#if 0		//�������
	u16 x=0,y=0;
	for(y=0;y<sLCD.Data.MaxH;y++)
	{
		for(x=0;x<sLCD.Data.MaxV;x++)
		{
			LCD_DrawDot(x,y,LCD565_YELLOW);
//			SysTick_DeleyuS(10);				//SysTick��ʱnmS
		}
	}
	SysTick_DeleymS(100);				//SysTick��ʱnmS
	LCD_Clean(LCD565_RED);			//�����Ļ����--
#endif	
#if 0			//���Բ��λ���
	u16 x=0,y=0;
	
	Rand	=	rand();
	x	=	HX;
	y	=	R61509V_H-Rand%200;
	LCD_DrawDot(x,y,LCD565_YELLOW);
//	R61509V_DrawLine(x,HY,x,y,R61509V_YELLOW);
	HY	=	y;
	if(HX++>=R61509V_V)
	{
		HX	=	0;
		LCD_Clean(LCD565_RED);			//�����Ļ����--
//		SysTick_DeleymS(500);				//SysTick��ʱnmS
	}
//	SysTick_DeleymS(100);					//SysTick��ʱnmS
#endif
#if 0		//ˢ������
	u32 LCDTime=500;
////	u8 i	=	0;
//	for(Color	=	0;Color<=0xFFFF;)
//	{
//		LCD_Clean(Color);			//�����Ļ����--
//		SysTick_DeleymS(LCDTime);				//SysTick��ʱnmS
//		Color+=100;		
//	}
	
	
	LCD_Clean(LCD565_WHITE);		//�����Ļ����--
	SysTick_DeleymS(LCDTime);				//SysTick��ʱnmS

	LCD_Clean(LCD565_BLACK);		//�����Ļ����--
	SysTick_DeleymS(LCDTime);				//SysTick��ʱnmS
	
	LCD_Clean(LCD565_BLUE);		//�����Ļ����--
	SysTick_DeleymS(LCDTime);				//SysTick��ʱnmS
	
	LCD_Clean(LCD565_BRED);			//�����Ļ����--
	SysTick_DeleymS(LCDTime);				//SysTick��ʱnmS
	
	LCD_Clean(LCD565_GRED);			//�����Ļ����--
	SysTick_DeleymS(LCDTime);				//SysTick��ʱnmS
	
	LCD_Clean(LCD565_GBLUE);			//�����Ļ����--
	SysTick_DeleymS(LCDTime);				//SysTick��ʱnmS
	
	LCD_Clean(LCD565_RED);			//�����Ļ����--
	SysTick_DeleymS(LCDTime);				//SysTick��ʱnmS
	
	LCD_Clean(LCD565_MAGENTA);			//�����Ļ����--
	SysTick_DeleymS(LCDTime);				//SysTick��ʱnmS
	
	LCD_Clean(LCD565_GREEN);			//�����Ļ����--
	SysTick_DeleymS(LCDTime);				//SysTick��ʱnmS
	
	LCD_Clean(LCD565_CYAN);			//�����Ļ����--
	SysTick_DeleymS(LCDTime);				//SysTick��ʱnmS
	
	LCD_Clean(LCD565_YELLOW);			//�����Ļ����--
	SysTick_DeleymS(LCDTime);				//SysTick��ʱnmS
#endif
#if 0	//��ֱ��
	HX	=	0;HY	=	10;
//	for(HY	=	10;HY<=10;HY++)
//	{
//		for(HX	=	10;HX<=400;)
//		{
//			LCD_DrawDot(HX,HY,R61509V_YELLOW);
//			HX	+=	1;
//			SysTick_DeleymS(10);				//SysTick��ʱnmS
//		}
//	}
//	R61509V_SetWindowAddress(0,10,100,10);//���ô���ַ
//	R61509V_SetWindowAddress(0,10,300,10);//���ô���ַ
//	for(HX=0;HX<=400;HX++)
//	{
//		R61509V_WriteData16(R61509V_YELLOW); 	//�ʻ���ɫ	
//	}
	R61509V_DrawLine(0,100,500,100,R61509V_YELLOW);
	R61509V_DrawLine(100,0,100,230,R61509V_YELLOW);
	R61509V_DrawLine(350,0,350,230,R61509V_YELLOW);
	SysTick_DeleymS(1000);				//SysTick��ʱnmS
	R61509V_Clean(R61509V_RED);			//�����Ļ����--����
//	//���ֱ��
//	HX	=	10;HY	=	10;
//	for(HX	=	10;HX<230;HX++)
//	{
//		for(HY	=	10;HY<=380;)
//		{
//			LCD_DrawDot(HX,HY,R61509V_RED);
//			HY	+=	5;
//			SysTick_DeleymS(100);				//SysTick��ʱnmS
//		}
//	}

//	LCD_Clean(R61509V_RED);			//�����Ļ����--����
#endif
#if 0	//LCD��������.----����
	LCD_Clean(LCD565_RED);			//�����Ļ����--����
	HX	=	10;HY	=	10;
	for(HY	=	10;HY<=230;)
	{
		LCD_DrawLine(10,HY,390,HY,LCD565_YELLOW);
		HY	+=	5;
	}
	SysTick_DeleymS(1000);				//SysTick��ʱnmS
	HX	=	10;HY	=	10;
	for(HX	=	10;HX<=390;)
	{
		LCD_DrawLine(HX,10,HX,230,LCD565_YELLOW);
		HX	+=	5;
//		SysTick_DeleymS(100);				//SysTick��ʱnmS
	}
	SysTick_DeleymS(1000);				//SysTick��ʱnmS
	//���
	HX	=	10;HY	=	10;
	for(HY	=	10;HY<=230;)
	{
		LCD_DrawLine(10,HY,390,HY,LCD565_RED);
		HY	+=	5;
	}
	SysTick_DeleymS(1000);				//SysTick��ʱnmS
	HX	=	10;HY	=	10;
	for(HX	=	10;HX<=390;)
	{
		LCD_DrawLine(HX,10,HX,230,LCD565_RED);
		HX	+=	5;
	}
	SysTick_DeleymS(1000);				//SysTick��ʱnmS
	LCD_Clean(LCD565_RED);			//�����Ļ����--����
#endif
#if 0
//	LCD_Clean(LCD565_RED);			//�����Ļ����--����
	
//	LCD_Printf(0		,0,32	,"��");									//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
	
//	LCD_Printf(0		,0,16	,"��ʾ���ԡ���");									//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//	LCD_Printf(0		,16,16	,"��ʾ���ԡ���");									//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//	LCD_Printf(0		,32,32	,"��ʾ���ԡ���");									//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//	LCD_Printf(0		,64,32	,"��ʾ���ԡ���");									//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//	LCD_Printf(0		,96,32	,"��ʾ���ԡ���");									//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
	LCD_Printf(0		,128,32	,"������ʾ����");									//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
	LCD_Printf(0		,160,32	,"�T����ʾ����");									//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����

//	SysTick_DeleymS(1000);				//SysTick��ʱnmS
//	LCD_Clean(LCD565_RED);			//�����Ļ����--����	
#endif
	if(DspFlg)
	{
		DspFlg	=	0;
//		LCD_Printf(0		,32,32	,"%0.8d",CS5530_ADC_CMP);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//		LCD_Printf(0		,64	,32	,"%0.8d",CS5530_ADC_Value);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
	}
	if(DspFlg1)
	{
		DspFlg1	=	0;
//		LCD_Printf(0		,96,32	,"%0.8d",We1);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
	}
}
/*******************************************************************************
* ������			:	RS485_Server
* ��������		:	�����շ�--��ȡADֵ����:FA F5 ID 01 ,�ϱ����ݸ�ʽ��FB F6 ID B1 B2 B3 ��λ��ǰ
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void RS485_Server(void)			//ͨѶ����---������Ϣ�Ľ����뷢��
{
	
	RxNum=RS485_ReadBufferIDLE(&RS485,RevBuffe);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(RxNum)
	{
		Rxdata	=	RevBuffe[0];
		SysTick_DeleymS(10);				//SysTick��ʱnmS
		RS485_DMASend	(&RS485,&Rxdata,1);	//RS485-DMA���ͳ���
		LCD_Printf(0		,0	,32,LCD565_RED,"�յ�����%d",Rxdata);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		return;
	}
//	else
//	{
//		if(SYSTIME++>=500)
//		{
//			SYSTIME	=	0;
//			RS485_DMASend	(&RS485,(u32*)&Rxdata,1);	//RS485-DMA���ͳ���
//			SysTick_DeleymS(5);				//SysTick��ʱnmS
//		}
//	}
	return;
#if Master			//����
	RxNum=RS485_ReadBufferIDLE(&RS485,(u32*)RevBuffe,(u32*)RxdBuffe);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(		RxNum								\
			&&	(RevBuffe[0]	==	0xFB)			\
			&&	(RevBuffe[1]	==	0xF6)			\
		)
	{		
		Rs485_Time	=	0;
		Rev_ADC	=	0;
		Rev_ID	=	RevBuffe[2];
		
		Rev_ADC	+=	RevBuffe[3];
		Rev_ADC	<<=	8;
		Rev_ADC	+=	RevBuffe[4];
		Rev_ADC<<=8;
		Rev_ADC	+=	RevBuffe[5];
		
		LCD_Printf(0		,128	,16	,"���յ�ͨ��%0.2d����ֵ��%0.8d",Rev_ID,Rev_ADC);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		
	}
	if(Rs485_Time	++>5)
	{
		Rs485_Time	=	0;
		TxdBuffe[0]	=	0xFA;
		TxdBuffe[1]	=	0xF5;
		TxdBuffe[2]	=	SlaveID;
		TxdBuffe[3]	=	0x01;
		RS485_DMASend	(&RS485,(u32*)TxdBuffe,4);	//RS485-DMA���ͳ���
		if(SlaveID++>=250)
			SlaveID	=	1;
	}
#else
	
	if(		RxNum												\
			&&(RevBuffe[0]	==	0xFA)			\
			&&(RevBuffe[1]	==	0xF5)			\
			&&(RevBuffe[2]	==	SwitchID)	\
			&&(RevBuffe[3]	==	0x01)			\
		)
	{
		RxFlg	=	1;
		Rs485_Time	=	0;
	}
	if(RxFlg	==	1)
	{
		if(Rs485_Time++>=2)	//��ʱ10mS
		{
			//CS5530_ADC_Value
			RxFlg	=	0;
			Rs485_Time	=	0;
			
			TxdBuffe[0]	=	0xFB;
			TxdBuffe[1]	=	0xF6;
			TxdBuffe[2]	=	SwitchID;
			TxdBuffe[3]	=	(CS5530_ADC_Value>>16)&0xFF;
			TxdBuffe[4]	=	(CS5530_ADC_Value>>8)&0xFF;
			TxdBuffe[5]	=	(CS5530_ADC_Value>>0)&0xFF;
			RS485_DMASend	(&RS485,TxdBuffe,6);	//RS485-DMA���ͳ���
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
void SwitchID_Server(void)	//���뿪�ش���--��̬���²����ַ
{
	u8 ID_Temp	=	0;
	ID_Temp	=	SWITCHID_Read(&SWITCHID);		//
	if(SwitchID	!=	ID_Temp)
	{
		SwitchID	=	ID_Temp;
		LCD_Printf(0		,0,32	,LCD565_RED,"��ַ��%0.2d",SwitchID);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
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
void SwitchID_Configuration(void)
{
	SWITCHID.NumOfSW	=	6;
	
	SWITCHID.SW1_PORT	=	GPIOC;
	SWITCHID.SW1_Pin	=	GPIO_Pin_6;
	
	SWITCHID.SW2_PORT	=	GPIOC;
	SWITCHID.SW2_Pin	=	GPIO_Pin_7;
	
	SWITCHID.SW3_PORT	=	GPIOC;
	SWITCHID.SW3_Pin	=	GPIO_Pin_8;
	
	SWITCHID.SW4_PORT	=	GPIOC;
	SWITCHID.SW4_Pin	=	GPIO_Pin_9;
	
	SWITCHID.SW5_PORT	=	GPIOC;
	SWITCHID.SW5_Pin	=	GPIO_Pin_12;
	
	SWITCHID.SW6_PORT	=	GPIOC;
	SWITCHID.SW6_Pin	=	GPIO_Pin_13;
	
	SwitchIdInitialize(&SWITCHID);							//
	
	SwitchID	=	SWITCHID_Read(&SWITCHID);		//
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
	LCDPortDef		*Port;
	SPIDef				*SPIx;
	
	Port	=	&(sLCD.Port);
	SPIx		=	&(sLCD.GT32L32.SPI);
	
	Port->sCS_PORT		=	GPIOB;
	Port->sCS_Pin			=	GPIO_Pin_7;
	
	Port->sDC_PORT		=	GPIOB;
	Port->sDC_Pin			=	GPIO_Pin_6;
	
	Port->sWR_PORT		=	GPIOB;
	Port->sWR_Pin			=	GPIO_Pin_8;
	
	Port->sRD_PORT		=	GPIOB;
	Port->sRD_Pin			=	GPIO_Pin_5;
	
	Port->sREST_PORT	=	GPIOB;
	Port->sREST_Pin		=	GPIO_Pin_9;
	
	Port->sBL_PORT		=	GPIOB;
	Port->sBL_Pin			=	GPIO_Pin_3;
	
	Port->sTE_PORT		=	GPIOB;
	Port->sTE_Pin			=	GPIO_Pin_4;
	
	Port->sDATABUS_PORT	=	GPIOE;
	Port->sDATABUS_Pin	=	GPIO_Pin_All;
	
	sLCD.Data.BColor	=	LCD565_BLACK;
	sLCD.Data.PColor	=	LCD565_WHITE;
	sLCD.Flag.Rotate	=	Draw_Rotate_90D;	//ʹ����ת�Ƕ�	
	
	SPIx->Port.SPIx=SPI1;
	
	SPIx->Port.CS_PORT=GPIOA;
	SPIx->Port.CS_Pin=GPIO_Pin_4;
	
	SPIx->Port.CLK_PORT	=	GPIOA;
	SPIx->Port.CLK_Pin=GPIO_Pin_5;
	
	SPIx->Port.MISO_PORT	=	GPIOA;
	SPIx->Port.MISO_Pin=GPIO_Pin_6;
	
	SPIx->Port.MOSI_PORT	=	GPIOA;
	SPIx->Port.MOSI_Pin=GPIO_Pin_7;
	
	SPIx->Port.SPI_BaudRatePrescaler_x=SPI_BaudRatePrescaler_2;
	
	R61509V_Initialize(&sLCD);
	
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE);				//I/O����ӳ�俪��
	PWM_OUTRemap(TIM2,PWM_OUTChannel2,500,50);									//PWM�趨-20161127�汾--����ָʾ��
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
void CS5530_Configuration(void)
{
	CS5530.Port.CS_PORT=GPIOD;
	CS5530.Port.CS_Pin=GPIO_Pin_8;
	
	CS5530.Port.SDI_PORT=GPIOB;
	CS5530.Port.SDI_Pin=GPIO_Pin_15;
	
	CS5530.Port.SDO_PORT=GPIOB;
	CS5530.Port.SDO_Pin=GPIO_Pin_14;
	
	CS5530.Port.SCLK_PORT=GPIOB;
	CS5530.Port.SCLK_Pin=GPIO_Pin_13;
	
	CS5530_Initialize(&CS5530);

#ifndef TestChN1		//˫ͨ��
	CS5530B.Port.CS_PORT=GPIOB;
	CS5530B.Port.CS_Pin=GPIO_Pin_12;
	
	CS5530B.Port.SDI_PORT=GPIOB;
	CS5530B.Port.SDI_Pin=GPIO_Pin_15;
	
	CS5530B.Port.SDO_PORT=GPIOB;
	CS5530B.Port.SDO_Pin=GPIO_Pin_14;
	
	CS5530B.Port.SCLK_PORT=GPIOB;
	CS5530B.Port.SCLK_Pin=GPIO_Pin_13;
	
	CS5530_Initialize(&CS5530B);
#else
	#ifndef TestFloat		//δ����ͨ������Ϊ�Ǹ���
		GPIO_Configuration_OPP50(GPIOB,GPIO_Pin_12);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
		GPIO_SetBits(GPIOB,GPIO_Pin_12);
	#endif
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
void RS485_Configuration(void)
{
	RS485.USARTx=USART2;
	RS485.RS485_CTL_PORT=GPIOA;
	RS485.RS485_CTL_Pin=GPIO_Pin_1;
	
	RS485_DMA_ConfigurationNR	(&RS485,19200,Rs485Size);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
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


#endif