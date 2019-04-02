//#ifdef	SSD1963

/********************************************************************************
***SSD1963 LCD��������
********************************************************************************/
#include "SSD1963.H"
#include "LCD.H"
#include "STM32_GPIO.H"

#include	"stdio.h"			//����printf
#include	"string.h"		//����printf
#include	"stdarg.h"		//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"		//malloc��̬�����ڴ�ռ�

//#include <reg51.h>
//#include "intrins.h"
//#include "font\font.h"
//#include "sys\sys.h"
//#include "lcd\lcd.h"


LCDDef	*pSSD1963	=	0;		//�ڲ�����ʹ�ã�����ɾ��

#define	SSD1963_CS_HIGH		(pSSD1963->Port.sCS_PORT->BSRR	= pSSD1963->Port.sCS_Pin)
#define	SSD1963_CS_LOW		(pSSD1963->Port.sCS_PORT->BRR	=	pSSD1963->Port.sCS_Pin)

#define	SSD1963_DC_HIGH		(pSSD1963->Port.sDC_PORT->BSRR	= pSSD1963->Port.sDC_Pin)		//RS
#define	SSD1963_DC_LOW		(pSSD1963->Port.sDC_PORT->BRR	= pSSD1963->Port.sDC_Pin)		//RS

#define	SSD1963_WR_HIGH		(pSSD1963->Port.sWR_PORT->BSRR	= pSSD1963->Port.sWR_Pin)
#define	SSD1963_WR_LOW		(pSSD1963->Port.sWR_PORT->BRR	= pSSD1963->Port.sWR_Pin)

#define	SSD1963_RD_HIGH		(pSSD1963->Port.sRD_PORT->BSRR	= pSSD1963->Port.sRD_Pin)
#define	SSD1963_RD_LOW		(pSSD1963->Port.sRD_PORT->BRR	= pSSD1963->Port.sRD_Pin)

#define	SSD1963_RST_HIGH	(pSSD1963->Port.sREST_PORT->BSRR	= pSSD1963->Port.sREST_Pin)
#define	SSD1963_RST_LOW		(pSSD1963->Port.sREST_PORT->BRR	= pSSD1963->Port.sREST_Pin)

#define	SSD1963_TE_HIGH		(pSSD1963->Port.sTE_PORT->BSRR	= pSSD1963->Port.sTE_Pin)
#define	SSD1963_TE_LOW		(pSSD1963->Port.sTE_PORT->BRR	= pSSD1963->Port.sTE_Pin)

#define	SSD1963_BL_HIGH		(pSSD1963->Port.sBL_PORT->BSRR	= pSSD1963->Port.sBL_Pin)
#define	SSD1963_BL_LOW		(pSSD1963->Port.sBL_PORT->BRR	= pSSD1963->Port.sBL_Pin)

#define SSD1963_DATABUS_PORT	(pSSD1963->Port.sDATABUS_PORT)
#define SSD1963_DATABUS_Pin		(pSSD1963->Port.sDATABUS_Pin)

#define	SSD1963Set(n)	SSD1963_##n##_HIGH
#define	SSD1963Crl(n)	SSD1963_##n##_LOW
/*******************************************************************************
*������			:	function
*��������		:	��������˵��
*����				: 
*����ֵ			:	��
*******************************************************************************/
void SSD1963_Initialize(void*	pInfo)
{
	static unsigned short	DeviceCode	=	0;
	LCDPortDef*	Port	=	NULL;
	if(NULL==	pInfo)
		return;
	pSSD1963		=	(LCDDef*)pInfo;		//ָ��ָ��	
	
	pSSD1963		=	(LCDDef*)pInfo;		//ָ��ָ��
	Port	=	&pSSD1963->Port;
	
	//==========================GPIO����
//	GPIO_Configuration_OPP50	(Port->sBL_PORT,				Port->sBL_Pin);					//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�2MHz----V20170605
//	GPIO_Configuration_OPP50	(Port->sRD_PORT,				Port->sRD_Pin);					//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�2MHz----V20170605
//	GPIO_Configuration_OPP50	(Port->sREST_PORT,			Port->sREST_Pin);				//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�2MHz----V20170605
//	GPIO_Configuration_OPP50	(Port->sDC_PORT,				Port->sDC_Pin);					//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�2MHz----V20170605
//	GPIO_Configuration_OPP50	(Port->sWR_PORT,				Port->sWR_Pin);					//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�2MHz----V20170605
//	GPIO_Configuration_OPP50	(Port->sCS_PORT,				Port->sCS_Pin);					//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�2MHz----V20170605
//	GPIO_Configuration_OPP50	(Port->sTE_PORT,				Port->sTE_Pin);					//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�2MHz----V20170605
//	GPIO_Configuration_OPP50	(Port->sDATABUS_PORT,		Port->sDATABUS_Pin);		//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�2MHz----V20170605
	
//	DeviceCode	=	SSD1963_ReadRegister(0x00);
	
	//==========================��鱳��ɫ�뻭��ɫ�Ƿ���ͬ
	if(pSSD1963->Data.PColor	==	pSSD1963->Data.BColor)
	{
		pSSD1963->Data.PColor	=	pSSD1963->Data.BColor^0xFFFF;
	}
	
	pSSD1963->Data.MaxH	=	SSD1963_H;					//���ˮƽ����
	pSSD1963->Data.MaxV	=	SSD1963_V;					//���ֱ�߶�	
  
	//==========================�ӿں���
	pSSD1963->Display.WriteAddress		=	SSD1963_SetWindowAddress;
	pSSD1963->Display.PowerOn					=	SSD1963_PowerOn;
	pSSD1963->Display.DispOff					=	SSD1963_PowerOff;
	
	pSSD1963->Display.DrawDot					=	SSD1963_DrawDot;
	pSSD1963->Display.DrawLine				=	SSD1963_DrawLine;
	pSSD1963->Display.DrawCircle			=	SSD1963_DrawCircle;
	pSSD1963->Display.DrawRectangle		=	SSD1963_DrawRectangle;
	
	pSSD1963->Display.Fill						=	SSD1963_Fill;
	pSSD1963->Display.Clean						=	SSD1963_Clean;
	pSSD1963->Display.SetBackground		=	SSD1963_SetBackground;
	
	pSSD1963->Display.ShowChar				=	SSD1963_ShowChar;
	pSSD1963->Display.ShowWord				=	SSD1963_ShowWord;
	
  if(NULL ==  pSSD1963->Display.WriteIndex)
    pSSD1963->Display.WriteIndex  = SSD1963_WriteIndex;
  if(NULL ==  pSSD1963->Display.WriteData)
    pSSD1963->Display.WriteData  = SSD1963_WriteData;
  if(NULL ==  pSSD1963->Display.WriteCommand)
    pSSD1963->Display.WriteCommand  = SSD1963_WriteCommand;
  
	SSD1963_PowerOn();
	SSD1963_Clean(pSSD1963->Data.BColor);
	
	SSD1963Set(BL);
}
/*******************************************************************************
* ������			:	LCD_Reset
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void SSD1963_Reset(void)
{
	SSD1963Set(RST);
	LCD_DelaymS(5);				//SysTick��ʱnmS
	SSD1963Crl(RST);;
	LCD_DelaymS(5);				//SysTick��ʱnmS
	SSD1963Set(RST);
	LCD_DelaymS(10);				//SysTick��ʱnmS
}
/*******************************************************************************
* ������			:	SSD1963_WriteIndex
* ��������		:	дָ���Ĵ��� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned short SSD1963_ReadData(void)
{
	unsigned short Data	=	0;
#if	1	//Bus16Bit
	SSD1963Crl(RD);	//LCD_RD_LOW;
	Data	=	SSD1963_DATABUS_PORT->IDR;
	SSD1963Set(RD);	//LCD_RD_HIGH;
#else	
	SSD1963Crl(RD);	//LCD_RD_LOW;
	Data	=	SSD1963_DATABUS_PORT->IDR;
	SSD1963Set(RD);	//LCD_RD_HIGH;
	
	Data	<<=8;
	
	SSD1963Crl(RD);	//LCD_RD_LOW;
	Data	|=	SSD1963_DATABUS_PORT->IDR;
	SSD1963Set(RD);	//LCD_RD_HIGH;
#endif	
	return Data;
}
/*******************************************************************************
* ������			:	SSD1963_ReadRegister
* ��������		:	��ָ���Ĵ��� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned short SSD1963_ReadRegister(unsigned	short	Addr)
{
	unsigned short Data	=	0;
	SSD1963Crl(CS);	//LCD_CS_LOW;
	//---------------------Write Index
	SSD1963_WriteAddr(Addr);
	//---------------------Read Data
	GPIO_Configuration_IPU	(SSD1963_DATABUS_PORT,SSD1963_DATABUS_Pin);			//��GPIO��Ӧ�ܽ�����Ϊ��������ģʽ----V20170605
	Data	=	SSD1963_ReadData();
	SSD1963Set(CS);	//LCD_CS_HIGH;
	GPIO_Configuration_OPP50	(SSD1963_DATABUS_PORT,SSD1963_DATABUS_Pin);		//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�2MHz----V20170605	
	return Data;
}
/*******************************************************************************
* ������			:	SSD1963_WriteIndex
* ��������		:	дָ���Ĵ��� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void SSD1963_WriteData(unsigned	short	Data)
{
#if	1	//Bus16Bit
	SSD1963Crl(WR);	//LCD_WR_LOW;
	SSD1963_DATABUS_PORT->ODR = Data;
	SSD1963Set(WR);	//LCD_WR_HIGH;
#else
	SSD1963Crl(WR);	//LCD_WR_LOW;
	SSD1963_DATABUS_PORT->ODR = Data>>8;
	SSD1963Set(WR);	//LCD_WR_HIGH;
	
	SSD1963Crl(WR);	//LCD_WR_LOW;
	SSD1963_DATABUS_PORT->ODR = Data&0xFF;
	SSD1963Set(WR);	//LCD_WR_HIGH;
#endif
}
/*******************************************************************************
* ������			:	ILI9326_WriteAddr
* ��������		:	дָ���Ĵ��� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void SSD1963_WriteAddr(unsigned	short	Addr)
{
	SSD1963Crl(DC);	//LCD_DC_LOW;		//RS
	SSD1963_WriteData(Addr);
	SSD1963Set(DC);	//LCD_DC_HIGH;	//RS
}
/*******************************************************************************
* ������			:	SSD1963_WriteIndex
* ��������		:	дָ���Ĵ��� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void SSD1963_WriteIndex(unsigned	short	Index)
{
	SSD1963Crl(CS);	//LCD_CS_LOW;	
	SSD1963_WriteAddr(Index);
	SSD1963Set(CS);	//LCD_CS_HIGH;
}
/*******************************************************************************
* ������			:	SSD1963_WriteRegister
* ��������		:	дָ���Ĵ��� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void SSD1963_WriteRegister(unsigned	short	Addr,unsigned short Data)
{
	SSD1963Crl(CS);	//LCD_CS_LOW;	
	//---------------------Write Index
	SSD1963_WriteAddr(Addr);
	//---------------------Write Data
	SSD1963_WriteData(Data);	
	SSD1963Set(CS);	//LCD_CS_HIGH;	
}
/*******************************************************************************
* ������			:	SSD1963_WriteCommand
* ��������		:	��ָ���Ĵ��� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void SSD1963_WriteCommand(unsigned	short	Index,unsigned short Cmd)
{
	SSD1963_WriteRegister(Index,Cmd);
}
/*******************************************************************************
* ������			:	SSD1963_WriteGRAM
* ��������		:	дָ���Ĵ��� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void SSD1963_WriteGRAM(unsigned	short* RAM,unsigned long length)
{
	unsigned	long	i	=	0;
	SSD1963Crl(CS);	//LCD_CS_LOW;
	pSSD1963->Display.WriteIndex(0X3C);
	for(i=0;i<length;i++)
	{
		pSSD1963->Display.WriteData(RAM[i]);
	}
	SSD1963Set(CS);	//LCD_CS_HIGH;
}
/*******************************************************************************
* ������			:	SSD1963_WriteGRAM
* ��������		:	дָ���Ĵ��� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void SSD1963_ReadGRAM(unsigned	short* RAM,unsigned long length)
{
	unsigned	long	i	=	0;
	SSD1963Crl(CS);	//LCD_CS_LOW;
	SSD1963_WriteIndex(0X3E);
	GPIO_Configuration_IPU	(SSD1963_DATABUS_PORT,SSD1963_DATABUS_Pin);			//��GPIO��Ӧ�ܽ�����Ϊ��������ģʽ----V20170605
	for(i=0;i<length;i++)
	{
		RAM[i]=SSD1963_ReadData();
	}
	SSD1963Set(CS);	//LCD_CS_HIGH;
	GPIO_Configuration_OPP50	(SSD1963_DATABUS_PORT,SSD1963_DATABUS_Pin);		//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�2MHz----V20170605	
}
/**************************************************************************************************
* [Function] LCD_PowerOff:  �ر�LCD ��Դ
* [No param]
**************************************************************************************************/
void SSD1963_PowerOff( void )
{
	LCD_BL_OFF;		//�ر���
	SSD1963_WriteCommand( 0x10, 0 );	
}
/*******************************************************************************
*������		:	Lcd_Init
*��������	:	STM32�ڲ��¶ȴ���������
*����			: 
*���			:	��
*����ֵ		:	��
*����			:
*******************************************************************************/
void SSD1963_PowerOn(void)
{

	u16 time=2000;
	u16	temp=time;
//����һ����Щ��������ñ����ʱ����ʾ����
	//1������������������������λ
//	SSD1963_BACKLIGHT_OFF;	//�ر���
//	LCD_REST();
//	LCD_RST_LOW;
//	LCD_RST_HIGH;
//	while(temp--);	  //�ȴ���������
//	//2������������������������λ
//	LCD_CS_HIGH;			//��ȡ��Ƭѡ
//	LCD_RD_HIGH;
//	LCD_WR_LOW;				//���߹���Ϊд����
//	LCD_CS_LOW;  		  //ʹ��
//  LCD_DelaymS(10);
	SSD1963Crl(CS);	//LCD_CS_LOW;
	//3��������������������������ϵͳʱ��  ����Ƶ�� 10MHz  250MHz < VCO < 800MHz
	pSSD1963->Display.WriteIndex(0x00E2);						//PLL multiplier, set PLL clock to 120M Start the PLL. Before the start, the system was operated with the crystal oscillator or clock input
	pSSD1963->Display.WriteData(0x0023);	    				//���ñ�Ƶ N=0x36 for 6.5M, 0x23 for 10M crystal
	pSSD1963->Display.WriteData(0x0001);							//���÷�Ƶ
	pSSD1963->Display.WriteData(0x0004);							//�������
	//4����������������������ʹ��PLL
	pSSD1963->Display.WriteIndex(0x00E0);  					//PLL enable
	pSSD1963->Display.WriteData(0x0001);
	
	pSSD1963->Display.WriteIndex(0x00E0);
	pSSD1963->Display.WriteData(0x0003);
  LCD_DelaymS(10);
	//5����������������������������λ
	pSSD1963->Display.WriteIndex(0x0001);  					//software reset
	//6��������������������������ɨ��Ƶ��
	pSSD1963->Display.WriteIndex(0x00E6);						//PLL setting for PCLK, depends on resolution
	pSSD1963->Display.WriteData(0x0003);
	pSSD1963->Display.WriteData(0x00FF);
	pSSD1963->Display.WriteData(0x00FF);
	//7��������������������������LCD���ģʽ Set the LCD panel mode (RGB TFT or TTL)
	pSSD1963->Display.WriteIndex(0x00B0);						//LCD SPECIFICATION
	pSSD1963->Display.WriteData(0x0000);
	pSSD1963->Display.WriteData(0x0000);
	pSSD1963->Display.WriteData((LCD_HDP>>8)&0X00FF);  		//����ˮƽ���ص������8λ		Set HDP 
	pSSD1963->Display.WriteData(LCD_HDP&0X00FF);					//����ˮƽ���ص������8λ
	pSSD1963->Display.WriteData((LCD_VDP>>8)&0X00FF);  		//���ô�ֱ���ص������8λ		Set VDP
	pSSD1963->Display.WriteData(LCD_VDP&0X00FF);					//���ô�ֱ���ص������8λ
	pSSD1963->Display.WriteData(0x0000);									//������ż��RGB˳��Ĭ��0��Even line RGB sequence&Odd line RGB sequence
	//8��������������������������ˮƽ�� Set Horizontal Period
	pSSD1963->Display.WriteIndex(0x00B4);							//HSYNC
	pSSD1963->Display.WriteData((LCD_HT>>8)&0X00FF);  	//High byte of horizontal total period
	pSSD1963->Display.WriteData(LCD_HT&0X00FF);					//Low byte of the horizontal total period (display + non-display) in pixel clock (POR = 10101111)
																			//Horizontal total period = (HT + 1) pixels
	pSSD1963->Display.WriteData((LCD_HPS>>8)&0X00FF);  	//High byte of the non-display period between the start of the horizontal sync (LLINE) signal and the first
																			//display data. (POR = 000)
	pSSD1963->Display.WriteData(LCD_HPS&0X00FF);
	pSSD1963->Display.WriteData(LCD_HPW);			   //Set HPW
	pSSD1963->Display.WriteData((LCD_LPS>>8)&0X00FF);  //Set HPS
	pSSD1963->Display.WriteData(LCD_LPS&0X00FF);
	pSSD1963->Display.WriteData(0x0000);
	//9�������������������������ô�ֱ��	Set Vertical Period
	pSSD1963->Display.WriteIndex(0x00B6);							//VSYNC
	pSSD1963->Display.WriteData((LCD_VT>>8)&0X00FF);   //Set VT
	pSSD1963->Display.WriteData(LCD_VT&0X00FF);
	pSSD1963->Display.WriteData((LCD_VPS>>8)&0X00FF);  //Set VPS
	pSSD1963->Display.WriteData(LCD_VPS&0X00FF);
	pSSD1963->Display.WriteData(LCD_VPW);			   //Set VPW
	pSSD1963->Display.WriteData((LCD_FPS>>8)&0X00FF);  //Set FPS
	pSSD1963->Display.WriteData(LCD_FPS&0X00FF);	
	//10��������������������������GPIO
	pSSD1963->Display.WriteIndex(0x00B8);
	pSSD1963->Display.WriteData(0x0007);    //GPIO3=input, GPIO[2:0]=output //���ģʽ
	pSSD1963->Display.WriteData(0x0001);   	//0 GPIO0 is used to control the panel power with Enter Sleep Mode 0x10 or Exit Sleep Mode 0x11.
													//1 GPIO0 is used as normal GPIO
	//11��������������������������GPIO������ɨ�跽�� Set GPIO value for GPIO configured as output
	pSSD1963->Display.WriteIndex(0x00BA);
	pSSD1963->Display.WriteData((LCD_LR&0XFF)|(LCD_UD&0XFF));    //GPIO[3:0] out 1
	//12�������������������������õ�ַģʽ	Set Address Mode
	pSSD1963->Display.WriteIndex(0x0036); //rotation
	pSSD1963->Display.WriteData(0x0000);
	//13�����������������������������ݽӿ� Set Pixel Data Interface/Pixel Data Interface Format
	pSSD1963->Display.WriteIndex(0x00F0); //pixel data interface
	pSSD1963->Display.WriteData(0x0003);
	
	//14�������������������������ô�ֱ��
	pSSD1963->Display.WriteIndex(0x0029); //display on
	//15�������������������������ô�ֱ��
	pSSD1963->Display.WriteIndex(0x00d0); 
	pSSD1963->Display.WriteData(0x000D);
	SSD1963Set(CS);	//LCD_CS_HIGH;
}
/*******************************************************************************
*������		:	Address_set
*��������	:	STM32�ڲ��¶ȴ���������
*����			: 
*���			:	��
*����ֵ		:	��
*����			:
*******************************************************************************/
void SSD1963_SetWindowAddress(unsigned short x1,unsigned short y1,unsigned short x2,unsigned short y2)
{ 
	unsigned short MaxH,MaxV;
	unsigned short Model	=	0x5030;
	
	eRotate	Rotate	=	LCDSYS->Flag.Rotate;
	
	MaxH	=	LCDSYS->Data.MaxH;
	MaxV	=	LCDSYS->Data.MaxV;
	
if	(Rotate	==Draw_Rotate_0D)
{
	LCDSYS->Data.HSX	=	x1;
	LCDSYS->Data.HEX	=	x2;
	LCDSYS->Data.VSY	=	y1;
	LCDSYS->Data.VEY	=	y2;
	
	LCDSYS->Data.HXA	=	LCDSYS->Data.HSX;
	LCDSYS->Data.VYA	=	LCDSYS->Data.VSY;
	
	Model	=	0X00;
}
else if (Rotate	==Draw_Rotate_90D)
{
	LCDSYS->Data.HSX	=	y1;
	LCDSYS->Data.HEX	=	y2;	
	LCDSYS->Data.VSY	=	MaxV	-	x2	-	1;
	LCDSYS->Data.VEY	=	MaxV	-	x1	-	1;
	
	LCDSYS->Data.HXA	=	LCDSYS->Data.HSX;
	LCDSYS->Data.VYA	=	LCDSYS->Data.VEY;
	Model	=	0XA0;								//GRAM(Graphics RAM--ͼ���ڴ�) Data Write (R202h)׼��д��
}
else if (Rotate	==Draw_Rotate_180D)	
{
	LCDSYS->Data.HSX	=	MaxH	-	x2	-	1;
	LCDSYS->Data.HEX	=	MaxH	-	x1	-	1;
	LCDSYS->Data.VSY	=	MaxV	-	y2	-	1;
	LCDSYS->Data.VEY	=	MaxV	-	y1	-	1;
	
	LCDSYS->Data.HXA	=	LCDSYS->Data.HEX;
	LCDSYS->Data.VYA	=	LCDSYS->Data.VEY;
	
	Model	=	0XC0;
}
else //(Rotate	==Draw_Rotate_270D)
{
	LCDSYS->Data.HSX	=	y1;
	LCDSYS->Data.HEX	=	y2;
	LCDSYS->Data.VSY	=	x1;
	LCDSYS->Data.VEY	=	x2;
	
	LCDSYS->Data.HXA	=	LCDSYS->Data.HEX;
	LCDSYS->Data.VYA	=	LCDSYS->Data.VSY;

	Model	=	0X22;
}
	//======================================��������
	pSSD1963->Display.WriteIndex(0x002A);			//�����е�ַ
	pSSD1963->Display.WriteData(LCDSYS->Data.HSX>>8);		//��ʼ��ַ��8λ
	pSSD1963->Display.WriteData(LCDSYS->Data.HSX);			//��ʼ��ַ��8λ
	pSSD1963->Display.WriteData(LCDSYS->Data.HEX>>8);		//������ַ��8λ
	pSSD1963->Display.WriteData(LCDSYS->Data.HEX);			//�н�����ַ��8λ
	
	pSSD1963->Display.WriteIndex(0x002b);			//����ҳ��ַ	
	pSSD1963->Display.WriteData(LCDSYS->Data.VSY>>8);
	pSSD1963->Display.WriteData(LCDSYS->Data.VSY);
	pSSD1963->Display.WriteData(LCDSYS->Data.VEY>>8);
	pSSD1963->Display.WriteData(LCDSYS->Data.VEY);
	
	pSSD1963->Display.WriteIndex(0x0036);			//����ҳ��ַ
	pSSD1963->Display.WriteData(Model);

	pSSD1963->Display.WriteIndex(0x002c);			//д�ڴ���ʼ��ַ					 						 
}
//
//--------------------------------------------------------------GUI
//
/*******************************************************************************
*������			:	LCD_DrawPoint
*��������		:	����
*����				: 
*����ֵ			:	��
*******************************************************************************/
void SSD1963_DrawDot(
									unsigned short HSX,			//��X����*/
									unsigned short HSY,			//��Y����*/
									unsigned short Color		//����ɫ*/	
								)
{
	SSD1963_SetWindowAddress(HSX,HSY,HSX,HSY);	//���ù��λ��
	SSD1963_WriteGRAM(&Color,1);
}
/*******************************************************************************
*������		:	LCD_DrawLine
*��������	:	����
*����			: x1,y1:�������
						x2,y2:�յ�����
*���			:	��
*����ֵ		:	��
*����			:
*******************************************************************************/
void SSD1963_DrawLine(
									unsigned short HSX, unsigned short HSY, 	//x1,y1:�������
									unsigned short HEX, unsigned short HEY,		//x2,y2:�յ�����
									unsigned short Color					//��ɫ
)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 

	delta_x=HEX-HSX; //������������ 
	delta_y=HEY-HSY; 
	uRow=HSX; 
	uCol=HSY;
	
	if(delta_x>0)
		incx=1; //���õ������� 
	else if(delta_x==0)
		incx=0;//��ֱ�� 
	else
	{
		incx=-1;
		delta_x=-delta_x;
	}
		
	if(delta_y>0)
		incy=1; 
	else if(delta_y==0)
		incy=0;//ˮƽ�� 
	else
		{incy=-1;delta_y=-delta_y;}
		
	if( delta_x>delta_y)
		distance=delta_x; 								//ѡȡ�������������� 
	else
		distance=delta_y;
	
	for(t=0;t<distance+1;t++ )					//������� 
	{  
		SSD1963_DrawDot(uRow,uCol,Color);			//���� 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance;
			uCol+=incy; 
		} 
	}
}
/**************************************************************************************************
* [Function] LCD_DrawCircle:  �������ܡ�ע������ȵ�����
* [param01]t_Point point: description
* [param02]u16 R: description
* [param03]uint8_t Filled: description
* [param04]u16 color: description
**************************************************************************************************/
void SSD1963_DrawCircle(
												u16 x,u16 y,		//Բ������ԭ��
												u16 r,					//�뾶
												u8 Filled,			//�Ƿ����
												u16 color				//��ɫ
												)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);             //�ж��¸���λ�õı�־
	while(a<=b)
	{
		if(Filled)	//���ͻ���
		{
			SSD1963_DrawLine(x,y,x-b,y-a,color);             //3           
			SSD1963_DrawLine(x,y,x+b,y-a,color);             //0           
			SSD1963_DrawLine(x,y,x-a,y+b,color);             //1       
			SSD1963_DrawLine(x,y,x-b,y-a,color);             //7           
			SSD1963_DrawLine(x,y,x-a,y-b,color);             //2             
			SSD1963_DrawLine(x,y,x+b,y+a,color);             //4               
			SSD1963_DrawLine(x,y,x+a,y-b,color);             //5
			SSD1963_DrawLine(x,y,x+a,y+b,color);             //6 
			SSD1963_DrawLine(x,y,x-b,y+a,color);             
			a++;
			//ʹ��Bresenham�㷨��Բ     
			if(di<0)
				di +=4*a+6;	  
			else
			{
				di+=10+4*(a-b);   
				b--;
			}
			SSD1963_DrawLine(x,y,x+a,y+b,color);				//AB �������껭һ��ֱ��
		}
		else
		{
			SSD1963_DrawDot(x-b,y-a,color);             //3           
			SSD1963_DrawDot(x+b,y-a,color);             //0           
			SSD1963_DrawDot(x-a,y+b,color);             //1       
			SSD1963_DrawDot(x-b,y-a,color);             //7           
			SSD1963_DrawDot(x-a,y-b,color);             //2             
			SSD1963_DrawDot(x+b,y+a,color);             //4               
			SSD1963_DrawDot(x+a,y-b,color);             //5
			SSD1963_DrawDot(x+a,y+b,color);             //6 
			SSD1963_DrawDot(x-b,y+a,color);             
			a++;
			//ʹ��Bresenham�㷨��Բ     
			if(di<0)
				di +=4*a+6;	  
			else
			{
				di+=10+4*(a-b);   
				b--;
			}
				SSD1963_DrawDot(x+a,y+b,color);
		}
	}
}
/**************************************************************************************************
* [Function] LCD_DrawRectangle:  ��һ�¾��ο�
* [param01]t_Point top_p: ��������ֵ
* [param02]t_Point botton_p : �ذ�����ֵ
**************************************************************************************************/
void SSD1963_DrawRectangle(u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
	SSD1963_DrawLine( x1, y1,	x1,	y2, color );
	SSD1963_DrawLine( x1, y1,	x2,	y1, color );
	SSD1963_DrawLine( x2, y1,	x2,	y2, color );
	SSD1963_DrawLine( x1, y2,	x2,	y2, color );
}
/*******************************************************************************
*������			:	ILI9326_Fill
*��������		:	��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
*����				: 
*����ֵ			:	��
*******************************************************************************/
void SSD1963_Fill(
							unsigned short x1, unsigned short y1, 	//x1,y1:�������
							unsigned short x2, unsigned short y2,		//x2,y2:�յ�����
							u16 Color
)
{          
	unsigned int x;
	unsigned int y;	
	SSD1963_SetWindowAddress(x1,y1,x2,y2);
	pSSD1963->Display.WriteIndex( 0X2C );
	SSD1963Crl(CS);	//LCD_CS_LOW;
	for(x=0;x<=x2-x1;x++)
	{
		for(y=0;y<=y2-y1;y++)
		{
			pSSD1963->Display.WriteData(Color);							//д����
		}
	}	
	SSD1963Set(CS);	//LCD_CS_HIGH;
}
/*******************************************************************************
* ������			:	ILI9326_Clean
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void SSD1963_Clean(u16 Color)	//�����Ļ����
{
	unsigned short x,y;
	unsigned short HSX,HEX,HSY,HEY,MaxH,MaxV;
	unsigned long	length	=	0;
	eRotate	Rotate	=	pSSD1963->Flag.Rotate;
	
	MaxH	=	pSSD1963->Data.MaxH;
	MaxV	=	pSSD1963->Data.MaxV;	
	switch(Rotate)
	{
		case 	Draw_Rotate_0D:
					HSX	=	0;
					HEX	=	MaxH-1;
					HSY	=	0;
					HEY	=	MaxV-1;
			break;
		case	Draw_Rotate_90D:
					HSX	=	0;
					HEX	=	MaxV-1;
					HSY	=	0;
					HEY	=	MaxH-1;
			break;
		case	Draw_Rotate_180D:
					HSX	=	0;
					HEX	=	MaxH-1;
					HSY	=	0;
					HEY	=	MaxV-1;
			break;
		default:
					HSX	=	0;
					HEX	=	MaxV-1;
					HSY	=	0;
					HEY	=	MaxH-1;
			break;			
	}	
	SSD1963_Fill(HSX,HSY,HEX,HEY,Color);
}
/**************************************************************************************************
* [Function] ILI9326_SetBackground:  ���ñ�����ɫ
* [param01]u16 BackColor: ������ɫֵ
**************************************************************************************************/
void SSD1963_SetBackground(  u16 BackColor )
{
	pSSD1963->Data.BColor	=	BackColor;
	SSD1963_Clean(BackColor);	//�����Ļ����
}

//
//--------------------------------------------�ַ���ʾ
//

/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void SSD1963_ShowChar(
										u16 x,			//x				:���x����
										u16 y,			//y				:���y����
										u8 font,		//font		:�����С
										u16 color,	//������ɫ
										u8 num,			//num			:�ֽ���
										u8 *Buffer	//Buffer	:��ʾ�����ݻ���
										
)		//��ͨ�ֿ���Գ���
{
	u8 temp;
	u8 i=0,j=0;
	unsigned short x1=0,x2=0,y1=0,y2=0;
	unsigned short LCD_PEN_COLOR	=	0;   	//����ɫ
	x1	=	x;
	y1	=	y;
	x2	=	x+font/2-1;		//
	y2	=	y+font-1;
	
	SSD1963_SetWindowAddress(x1,y1,x2,y2);//������ʾ����	
	pSSD1963->Display.WriteIndex( 0X2C );
	SSD1963Crl(CS);	//LCD_CS_LOW;
	for(i=0;i<num;i++)
	{ 
		temp=Buffer[i];		 					//����1608����--��ά������ʽ--�ֿ�ʹ��ʱȡ��
		for(j=0;j<8;j++)
		{
			if((temp&0x80)==0X80)
			{
				LCD_PEN_COLOR=color;
			}
			else
				LCD_PEN_COLOR=pSSD1963->Data.BColor;
			pSSD1963->Display.WriteData(LCD_PEN_COLOR);
			temp=temp<<1;
		}
    //=======================δ��8λ�Ĳ��䶨��
    if((24==font)||(12==font))
    {
      temp=Buffer[i+1];		 					
      for(j=0;j<4;j++)
      {
        if((temp&0x80)==0X80)
        {
          LCD_PEN_COLOR=color;
        }
        else
          LCD_PEN_COLOR=pSSD1963->Data.BColor;
        pSSD1963->Display.WriteData(LCD_PEN_COLOR);
        temp=temp<<1;
      }
      i++;
    }		
	}
	SSD1963Set(CS);	//LCD_CS_HIGH;
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void SSD1963_ShowWord(
										u16 x,			//x				:���x����
										u16 y,			//y				:���y����
										u8 font,		//font		:�����С
										u16 color,	//������ɫ
										u8 num,			//num			:�ֽ���
										u8 *Buffer	//Buffer	:��ʾ�����ݻ���
										
)		//��ͨ�ֿ���Գ���
{
	u8 temp;
	u8 i=0,j=0;
	unsigned short x1=0,x2=0,y1=0,y2=0;
	unsigned short LCD_PEN_COLOR	=	0;   	//����ɫ
	x1	=	x;
	y1	=	y;
  x2	=	x+font-1;
  y2	=	y+font-1;
	SSD1963_SetWindowAddress(x1,y1,x2,y2);//������ʾ����
	pSSD1963->Display.WriteIndex( 0X2C );
	SSD1963Crl(CS);	//LCD_CS_LOW;
	for(i=0;i<num;i++)
	{ 
		temp=Buffer[i];		 				
		for(j=0;j<8;j++)
		{
			if((temp&0x80)==0X80)
			{
				LCD_PEN_COLOR=color;
			}
			else
				LCD_PEN_COLOR=pSSD1963->Data.BColor;
			pSSD1963->Display.WriteData(LCD_PEN_COLOR);
			temp=temp<<1;
		}
    //=======================δ��8λ�Ĳ��䶨��
    if((12==font))
    {
      temp=Buffer[i+1];		 					
      for(j=0;j<4;j++)
      {
        if((temp&0x80)==0X80)
        {
          LCD_PEN_COLOR=color;
        }
        else
          LCD_PEN_COLOR=pSSD1963->Data.BColor;
        pSSD1963->Display.WriteData(LCD_PEN_COLOR);
        temp=temp<<1;
      }
      i++;
    }			
	}
	SSD1963Set(CS);	//LCD_CS_HIGH;
}


