///******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
//* File Name          : PC001V21.c
//* Author             : WOW
//* Version            : V2.0.1
//* Date               : 06/26/2017
//* Description        : PC001V21����ư�.
//********************************************************************************
//* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
//* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
//* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
//* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
//* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
//* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//*******************************************************************************/

#ifdef PL012V30AMP				//�����LCD��

#include "PL012V30AMP.H"

#include "LCD.H"


#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_PWM.H"
#include "STM32_USART.H"
#include	"AMP_PHY.H"


#include "SWITCHID.H"

//#include "string.h"				//�����ڴ��������ͷ�ļ�


LCDDef	sLCD;
RS485Def RS485;
unsigned char RxdBuffe[128]={0};
unsigned short time	=	0;

char tep[1]={'A'};

unsigned char Version[]="PL012V3.0 RF���ܺĲĹ�����";
unsigned char DataStr[]=__DATE__;
unsigned char	TimeStr[]=__TIME__;
//SWITCHID_CONF	SWITCHID;
//u8 SwitchID=0;	//���뿪�ص�ַ

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PL012V30AMP_Configuration(void)
{
	SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H
	
	GPIO_DeInitAll();							//�����е�GPIO�ر�----V20170605
	
	LCD_Configuration();

//	GT32L32_Configuration();
//	
//	R61509V_Configuration();
//	
//	CS5530_Configuration();
//	
	RS485_Configuration();
//	
//	LCD_PowerUp();
//	
//	SwitchID_Configuration();
//	
	//LCD_Printf(0,140,16,LCD565_RED,"��ʾ����:%4X",LCD_ReadData(LCD_R000_IR));		//��������
	LCD_Printf(0,160,16,LCD565_RED,"��Ŀ���:%s",Version);		//��������
	LCD_Printf(0,180,16,LCD565_RED,"��������:%s",__DATE__);		//��������
	LCD_Printf(0,200,16,LCD565_RED,"����ʱ��:%s",__TIME__); 	//����ʱ�� 
	
	SysTick_Configuration(1000);	//ϵͳ���ʱ������72MHz,��λΪuS

//	IWDG_Configuration(1000);			//�������Ź�����---������λms	
	PWM_OUT(TIM2,PWM_OUTChannel1,1,900);	//PWM�趨-20161127�汾--ָʾ��
	
//	LCD_Printf(0,140,16,LCD565_RED,"��ʾ����:%4X",LCD_ReadData(LCD_R000_IR));		//��������
	
	PWM_OUT(TIM2,PWM_OUTChannel3,500,200);		//PWM�趨-20161127�汾--����
//	memset(TxdBuffe,0xA5,128);
//	LCD_Printf(0,0,32,"��ߵ�ʡ�Ժž��ǿɱ����ι��");		//��ߵ�ʡ�Ժž��ǿɱ����
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PL012V30AMP_Server(void)
{	
	u16 Num=0;
	IWDG_Feed();								//�������Ź�ι��
	if(time++>100)
	{
		time	=	0;
//		LCD_Clean(LCD565_BLACK);	//�����Ļ����
	}
	RS485_Server();	
//	LCD_Printf(0,30,16,0,"ABCDefgh�S�߁u���@�ʌm�ڮM");		//��ߵ�ʡ�Ժž��ǿɱ����
//	LCD_Printf(0,46,24,0,"ABCDefgh�S�߁u���@�ʌm�ڮM");		//��ߵ�ʡ�Ժž��ǿɱ����
//	LCD_Printf(0,70,32,0,"ABCDefgh�S�߁u���@�ʌm�ڮM");		//��ߵ�ʡ�Ժž��ǿɱ����
//	LCD_Show(0,70,32,strlen(tep),tep);
  LCD_ShowAntenna(220,0,0,LCD565_RED);   //��ʾ12x12����
  LCD_ShowAntenna(240,0,1,LCD565_RED);   //��ʾ12x12����
  LCD_ShowAntenna(260,0,2,LCD565_RED);   //��ʾ12x12����
  LCD_ShowAntenna(280,0,3,LCD565_RED);   //��ʾ12x12����
  LCD_ShowAntenna(300,0,4,LCD565_RED);   //��ʾ12x12����
  
  LCD_ShowBattery(320,0,0,LCD565_RED);   //��ʾ12x12���
  LCD_ShowBattery(340,0,1,LCD565_RED);   //��ʾ12x12���
  LCD_ShowBattery(360,0,2,LCD565_RED);   //��ʾ12x12���
  LCD_ShowBattery(380,0,3,LCD565_RED);   //��ʾ12x12���
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
	SPIDef				*SPIx;
	
	//==================�ֿ�����
	SPIx		=	&(sLCD.GT32L32.SPI);
	
	SPIx->Port.SPIx=SPI1;
	
	SPIx->Port.CS_PORT		=	GPIOA;
	SPIx->Port.CS_Pin			=	GPIO_Pin_4;
	
	SPIx->Port.CLK_PORT		=	GPIOA;
	SPIx->Port.CLK_Pin		=	GPIO_Pin_5;
	
	SPIx->Port.MISO_PORT	=	GPIOA;
	SPIx->Port.MISO_Pin		=	GPIO_Pin_6;
	
	SPIx->Port.MOSI_PORT	=	GPIOA;
	SPIx->Port.MOSI_Pin		=	GPIO_Pin_7;
	
	SPIx->Port.SPI_BaudRatePrescaler_x=SPI_BaudRatePrescaler_2;
	
	//==================LCD����
	sLCD.Port.sBL_PORT	=	GPIOA;
	sLCD.Port.sBL_Pin		=	GPIO_Pin_3;
	
	sLCD.Port.sCS_PORT	=	GPIOA;
	sLCD.Port.sCS_Pin		=	GPIO_Pin_12;
	
	sLCD.Port.sDC_PORT	=	GPIOA;
	sLCD.Port.sDC_Pin		=	GPIO_Pin_8;
	
	sLCD.Port.sRD_PORT	=	GPIOC;
	sLCD.Port.sRD_Pin		=	GPIO_Pin_5;
	
	sLCD.Port.sREST_PORT	=	GPIOD;
	sLCD.Port.sREST_Pin		=	GPIO_Pin_2;
	
	sLCD.Port.sTE_PORT		=	GPIOC;
	sLCD.Port.sTE_Pin			=	GPIO_Pin_4;
	
	sLCD.Port.sWR_PORT		=	GPIOA;
	sLCD.Port.sWR_Pin			=	GPIO_Pin_15;
	
	sLCD.Port.sDATABUS_PORT	=	GPIOB;
	sLCD.Port.sDATABUS_Pin	=	GPIO_Pin_All;	
	
	sLCD.Flag.Rotate			=	Draw_Rotate_90D;
	
	sLCD.Data.BColor			=	LCD565_YELLOW;
	sLCD.Data.PColor			=	LCD565_RED;
	
	//==================��ʼ��
	LCD_Initialize(&sLCD);
	
//	R61509V_Initialize(&sLCD);
	
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
	unsigned short RxNum	=	0;
#if 1
	RxNum=RS485_ReadBufferIDLE(&RS485,RxdBuffe);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(RxNum)
	{
		unsigned	short	coler	=	0;
		unsigned	char*	addr	=	NULL;
		
		LCD_Fill(0,16,399,32,sLCD.Data.BColor);
		LCD_ShowHex(0,16,16,LCD565_RED,RxNum,8,RxdBuffe);                //��ʾʮ����������
		addr	=	getheadaddr(RxdBuffe,RxNum);
		if(NULL==addr)
			return;
		coler	=	RxdBuffe[8]>>5;
		coler<<=5;
		coler|=RxdBuffe[9]>>5;
		coler<<=6;
		coler|=RxdBuffe[10]>>5;
		
		if(0	==	RxdBuffe[7])
		{
			LCD_Fill(0,64,399,160,coler);
		}
		else
		{
			LCD_Fill(0,64,399,160,LCD565_RED);
		}
	}
#else
	RxNum=RS485_ReadBufferIDLE(&RS485,(u32*)RevBuffe,(u32*)RxdBuffe);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(RxNum)
	{
		RS485Time	=	0;
		RS485FLG	=	1;
		RSRLen	=	RxNum;
		RS485_DMASend(&RS485,(u32*)RevBuffe,100);	//RS485-DMA���ͳ���
		PWM_OUT(TIM3,PWM_OUTChannel3,500,800);		//PWM�趨-20161127�汾--����
	}
	if(RS485FLG	==	1)
	{
		if(RS485Time++>=500)
		{
			RS485Time	=	0;
			RS485_DMASend(&RS485,(u32*)RevBuffe,RSRLen);	//RS485-DMA���ͳ���
			RSRLen	=	0;
			RS485FLG	=	0;
			PWM_OUT(TIM3,PWM_OUTChannel3,500,800);		//PWM�趨-20161127�汾--����
		}
	}
	
#endif
}

///*******************************************************************************
//* ������			:	function
//* ��������		:	��������˵�� 
//* ����			: void
//* ����ֵ			: void
//* �޸�ʱ��		: ��
//* �޸�����		: ��
//* ����			: wegam@sina.com
//*******************************************************************************/
//void SwitchID_Configuration(void)
//{
//	SWITCHID.NumOfSW	=	8;
//	
//	SWITCHID.SW1_PORT	=	GPIOD;
//	SWITCHID.SW1_Pin	=	GPIO_Pin_2;
//	
//	SWITCHID.SW2_PORT	=	GPIOA;
//	SWITCHID.SW2_Pin	=	GPIO_Pin_15;
//	
//	SWITCHID.SW3_PORT	=	GPIOA;
//	SWITCHID.SW3_Pin	=	GPIO_Pin_12;
//	
//	SWITCHID.SW4_PORT	=	GPIOA;
//	SWITCHID.SW4_Pin	=	GPIO_Pin_11;
//	
//	SWITCHID.SW5_PORT	=	GPIOA;
//	SWITCHID.SW5_Pin	=	GPIO_Pin_8;
//	
//	SWITCHID.SW6_PORT	=	GPIOB;
//	SWITCHID.SW6_Pin	=	GPIO_Pin_15;
//	
//	SWITCHID.SW7_PORT	=	GPIOB;
//	SWITCHID.SW7_Pin	=	GPIO_Pin_14;
//	
//	SWITCHID.SW8_PORT	=	GPIOB;
//	SWITCHID.SW8_Pin	=	GPIO_Pin_13;
//	
//	SwitchIdInitialize(&SWITCHID);							//
//	
////	SwitchID	=	SWITCHID_Read(&SWITCHID);		//
//}
///*******************************************************************************
//*������			:	function
//*��������		:	function
//*����				: 
//*����ֵ			:	��
//*�޸�ʱ��		:	��
//*�޸�˵��		:	��
//*ע��				:	wegam@sina.com
//*******************************************************************************/
//void SwitchID_Server(void)
//{
//	u8 temp=0;	//���뿪����ʱ��ַ
//	temp	=	SWITCHID_Read(&SWITCHID);		//��ȡ��ַ
//	if(SwitchID	!=	temp)
//	{
//		u8 AddrH	=	0,AddrL	=	0;
//		u32	w=0,n=0,d=0;
////	w	=	1;n	=	0;
////	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
////	w	+=	1;n	+=	8;
////	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
////	w	+=	1;n	+=	8;
////	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
////	w	+=	1;n	+=	8;
////	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
////	w	+=	1;n	+=	8;
////	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
////	w	+=	1;n	+=	8;
////	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
////	w	+=	1;n	+=	8;
////	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
////	w	+=	1;n	+=	8;
////	PL010V13_PrintfString(n		,0	,16	,"%2d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//		
//		w	=	12345678;
//		PL010V13_PrintfString(0		,0	,32	,"��ţ�");				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//		PL010V13_PrintfString(0		,32	,32	,"���룺");				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//		PL010V13_PrintfString(96	,0	,32	,"%8d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//		w	=	0;
//		SwitchID	=	temp;
//		for(n=0;n<8;n++)
//		{
//			if((temp&0x01)	==0x01)
//			{
//				d	=	1;
//			}
//			else
//			{
//				d	=	0;
//			}
//			temp>>=1;
//			PL010V13_PrintfString(n*16+96		,32	,32	,"%d",d);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//		}
//		AddrH	=	(SwitchID>>4)&0x0F;
//		AddrL	=	(SwitchID>>0)&0x0F;
//		PL010V13_PrintfString(0		,64	,32	,"��%0.2d��λ%0.2d",AddrH,AddrL);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//	}
//}

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
	RS485.USARTx=USART1;
	RS485.RS485_CTL_PORT=GPIOA;
	RS485.RS485_CTL_Pin=GPIO_Pin_11;
	
	RS485_DMA_ConfigurationNR	(&RS485,19200,256);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
}
///*******************************************************************************
//* ������		:	
//* ��������	:	 
//* ����		:	
//* ���		:
//* ���� 		:
//*******************************************************************************/
//void LCD_PowerUp(void)
//{	
////	u16 i=0,j=0;
//	
//	
//	
//	
//	
////	R61509V_DrawDot(50,50,0X5458);
////	R61509V_DrawDot_big(50,50,0X5458);
////	R61509V_DrawLine(0,0,240,400,0X5458);
////	R61509V_DrawLine(0,400,240,0,0X5458);
//	
////	R61509V_DrawCircle(120,100, 50,0,0X5458);				//��һ��Բ�ο�
//	
////	R61509V_DrawCircle(120,200, 120,0,0X01CF);				//��һ��Բ�ο�
//	
////	R61509V_DrawCircle(120,400, 80,0,0X5458);				//��һ��Բ�ο�
////	
////	R61509V_DrawCircle(120,0, 80,0,0X5458);					//��һ��Բ�ο�
////	
////	R61509V_DrawCircle(120,200, 80,1,0X5458);				//��һ��Բ�ο�
////	
////	R61509V_DrawCircle(200,120, 50,0,0xF800);				//��һ��Բ�ο�
////	
////	R61509V_DrawCircle(120,200, 30,1,0X07FF);				//��һ��Բ�ο�
//	
////	R61509V_DrawRectangle(3,3,237,397,0x07E0);			//��һ�����ο�
////	R61509V_DrawRectangle(4,4,236,396,0x07E0);			//��һ�����ο�
////	R61509V_DrawRectangle(5,5,235,395,0x07E0);			//��һ�����ο�
////	R61509V_DrawRectangle(6,6,234,394,0x07E0);			//��һ�����ο�
////	
////	R61509V_DrawRectangle(7,7,233,393,0X07FF);			//��һ�����ο�
////	R61509V_DrawRectangle(8,8,232,392,0X07FF);			//��һ�����ο�
////	R61509V_DrawRectangle(9,9,231,391,0X07FF);			//��һ�����ο�
////	R61509V_DrawRectangle(10,10,230,390,0X07FF);		//��һ�����ο�
////	R61509V_DrawRectangle(11,11,229,389,0X07FF);		//��һ�����ο�
////	R61509V_DrawRectangle(12,12,228,388,0X07FF);			//��һ�����ο�
////	R61509V_DrawLine(12,12,228,388,0X5458);						//AB �������껭һ��ֱ��
////	R61509V_DrawLine(12,388,228,12,0X5458);						//AB �������껭һ��ֱ��
//	
////	R61509V_DrawLine(0,10,400,10,0X5458);						//AB �������껭һ��ֱ��
////	
////	R61509V_DrawLine(0,20,400,20,0X5458);						//AB �������껭һ��ֱ��
//	
////	R61509V_DrawLine(0,100,160,100,0X5458);						//AB �������껭һ��ֱ��
////	
////	R61509V_DrawLine(80,0,80,400,0X5458);						//AB �������껭һ��ֱ��
//	
////	R61509V_DrawRectangle(10,10,390,230,0X07FF);		//��һ�����ο�
//	
////	R61509V_DrawCircle(200,120, 10,1,0X5458);					//��һ��Բ�ο�
////	
////	R61509V_DrawCircle(200,120, 80,0,0X5458);					//��һ��Բ�ο�
//	
////	R61509V_DrawLine(240,240,20,240,0X5458);						//AB �������껭һ��ֱ��
//	
////	R61509V_ShowChar(1,1,32,100,zimo);								//��ͨ�ֿ���Գ���
////	
////	R61509V_ShowCharT(50,50,15,0);
////	R61509V_ShowEn(200,120,12);
//	
////	PL010V13_PrintfString(0		,16	,16	,"����ҩ��λ��%3d",RevBuffe[0]);				//��ߵ�ʡ�Ժž��ǿɱ����
////	PL010V13_PrintfString(0		,32	,16	,"����ҩ������%3d",RevBuffe[1]);				//��ߵ�ʡ�Ժž��ǿɱ����

////	PL010V13_PrintfString(0		,0	,32	,"��λ-%2d����-%2d",RevBuffe[0],RevBuffe[1]);				//��ߵ�ʡ�Ժž��ǿɱ����
//	
////	PL010V13_PrintfString(0		,0	,32	,"����ҩ������%2d",RevBuffe[1]);				//��ߵ�ʡ�Ժž��ǿɱ����
////	
////	
////	PL010V13_PrintfString(0		,100	,32	,"����ҩ������%2d",RevBuffe[1]);				//��ߵ�ʡ�Ժž��ǿɱ����
////	PL010V13_PrintfString(0		,100	,32	,"����ҩ������%2d",RevBuffe[1]);				//��ߵ�ʡ�Ժž��ǿɱ����
////	
//	#if 0
//	PL010Delay(0xFFFF);
//	PL010V13_PrintfString(1		,0	,16	,"����ҩ��λ��%2d",RevBuffe[0]);				//��ߵ�ʡ�Ժž��ǿɱ����
//	PL010V13_PrintfString(1		,0	,16	,"����ҩ��λ��%2d",RevBuffe[0]);				//��ߵ�ʡ�Ժž��ǿɱ����
//	PL010Delay(0xFFFF);
//	PL010V13_PrintfString(1		,20	,16	,"����ҩ������%2d",RevBuffe[1]);				//��ߵ�ʡ�Ժž��ǿɱ����
//	PL010Delay(0xFFFF);
//	PL010V13_PrintfString(1		,40	,16	,"�ѷ�ҩ������%2d",RevBuffe[1]);				//��ߵ�ʡ�Ժž��ǿɱ����
//	PL010Delay(0xFFFF);
//	
////	PL010V13_PrintfString(0		,60	,16	,"�ܹ�����������%4d",SumFQ);				//�ܹ���ҩ��������
////	PL010V13_PrintfString(0		,80	,16	,"�ܹ���ҩ������%4d",SumFed);				//�ܹ��ѷ�ҩ����
//	PL010V13_PrintfString(1		,100	,16	,"����״̬��");				//����״̬
//	PL010Delay(0xFFFF);
//	
//	PL010V13_PrintfString(1		,160	,16	,"��ʾ������3���ȡ��ҩͷ");				//����״̬
//	PL010Delay(0xFFFF);
//	

//	//ƽ����
//	R61509V_DrawLine(221,0,221,400,R61509V_WHITE);						//AB �������껭һ��ֱ��
//	R61509V_DrawLine(201,0,201,400,R61509V_WHITE);						//AB �������껭һ��ֱ��
//	R61509V_DrawLine(181,0,181,400,R61509V_WHITE);						//AB �������껭һ��ֱ��
//	R61509V_DrawLine(161,0,161,400,R61509V_WHITE);						//AB �������껭һ��ֱ��
//	//��ֱ��--��
//	R61509V_DrawLine(161,200,221,200,R61509V_WHITE);					//AB �������껭һ��ֱ��
//	//��ֱ��--��
//	R61509V_DrawLine(161,250-1,221,250-1,R61509V_WHITE);			//AB �������껭һ��ֱ��
//	R61509V_DrawLine(161,300-1,221,300-1,R61509V_WHITE);			//AB �������껭һ��ֱ��
//	R61509V_DrawLine(161,350-1,221,350-1,R61509V_WHITE);			//AB �������껭һ��ֱ��
//	R61509V_DrawLine(161,400-1,221,400-1,R61509V_WHITE);			//AB �������껭һ��ֱ��
//	
//	//��ֱ��--��
//	R61509V_DrawLine(161,1,221,1,					R61509V_WHITE);							//AB �������껭һ��ֱ��
//	R61509V_DrawLine(161,51,221,51,				R61509V_WHITE);						//AB �������껭һ��ֱ��
//	R61509V_DrawLine(161,101,221,101,			R61509V_WHITE);					//AB �������껭һ��ֱ��
//	R61509V_DrawLine(161,151,221,151,			R61509V_WHITE);					//AB �������껭һ��ֱ��

//	//���
//	R61509V_Fill(2,180,50,200-2,R61509V_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//	
//	
//	
//	
////	R61509V_DrawLine(10,200,10,280,R61509V_WHITE);						//AB �������껭һ��ֱ��
//	
//	#else
////	R61509V_Clean(R61509V_WHITE);			//�����Ļ����------
////	PL010Delay(0x8FFFFF);
//	R61509V_Clean(R61509V_BLUE);			//�����Ļ����------
//	PL010Delay(0x8FFFFF);
//	R61509V_Clean(R61509V_GRED);			//�����Ļ����------
//	PL010Delay(0x8FFFFF);
//	R61509V_Clean(R61509V_BLACK);			//�����Ļ����------
//	PL010Delay(0x8FFFFF);
//	R61509V_Clean(R61509V_WHITE);			//�����Ļ����------

////	PL010Delay(0xFFFF);
////	PL010V13_PrintfString(0	,0,16	,"TEST");				//����״̬
//	#endif
////	R61509V_Clean(R61509V_BLACK);			//�����Ļ����------
//	
//}
///*******************************************************************************
//* ������			:	function
//* ��������		:	��������˵�� 
//* ����			: void
//* ����ֵ			: void
//* �޸�ʱ��		: ��
//* �޸�����		: ��
//* ����			: wegam@sina.com
//*******************************************************************************/
//void LCD_Display(void)
//{
//	PL010V13_PrintfString(0		,120	,16	,"��ʾ���ԣ�����������������");				//����״̬
//	
//#if 0		
//	DSPTime++;
//	if(DSPTime>500)
//	{
//		DSPTime=0;
//		LCD_WXS();		//λ��ʾ
//		LCD_DDSP();		//��ʾ�ܹ������������ѷ�����
//	}	
//	LCD_WS();		//λ��˸
//	
//	if(LCDTime>=1000)
//	{			
//		LCDTime=0;		
//	}	
//	
//	RxNum=RS485_ReadBufferIDLE(&RS485,(u32*)RevBuffe,(u32*)RxdBuffe);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
//	if(RxNum==4&&RevBuffe[0]==0x00&&RevBuffe[1]==0xFF)		//RS485���յ�����
//	{
//		NumFW=RevBuffe[2];
//		PL010V13_PrintfString(96		,0	,16	,"%2d",RevBuffe[2]);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//		PL010V13_PrintfString(96		,20	,16	,"%2d",RevBuffe[3]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
////		PL010V13_PrintfString(192		,68	,16	,"%2d",RevBuffe[1]);				//�ѷ�ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//	}
//	else if(RxNum==3&&RevBuffe[0]==0x02)		//RS485���յ�����
//	{
//		SumFQ[RevBuffe[1]-1]+=RevBuffe[2];
//		NumFW=RevBuffe[1];
//		PL010V13_PrintfString(96		,0	,16	,"%2d",RevBuffe[1]);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//		PL010V13_PrintfString(96		,20	,16	,"%2d",RevBuffe[2]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//		PL010V13_PrintfString(96		,40	,16	,"%2d",0);									//�ѷ�ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//		
////		PL010V13_PrintfString(0		,60	,16	,"�ܹ�����������%4d",SumFQ);				//�ܹ���ҩ��������
//		PL010V13_PrintfString(0		,120	,16	,"���ڷ�ҩ��������������������");				//����״̬
//	}
//	else if(RxNum==4&&RevBuffe[0]==0x82)		//RS485���յ�����
//	{
//		SumFed[RevBuffe[1]-1]+=RevBuffe[2];
//		PL010V13_PrintfString(96		,0	,16	,"%2d",RevBuffe[1]);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//		PL010V13_PrintfString(96		,20	,16	,"%2d",0);									//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//		PL010V13_PrintfString(96		,40	,16	,"%2d",RevBuffe[2]);				//�ѷ�ҩ��������ߵ�ʡ�Ժž��ǿɱ����		
//		
////		PL010V13_PrintfString(112		,80	,16	,"%4d",SumFed);					//�ܹ��ѷ�ҩ����
//		
//	}
//	else if(RxNum==6&&RevBuffe[0]==0x81)	//��λ��Ϣ0x01--��ȡ,0x81--�ϱ�
//	{
//		Onlinede=RevBuffe[4];
//	}
//	else if(RxNum==1&&RevBuffe[0]==0x01)	//��λ��Ϣ0x01--��ȡ,0x81--�ϱ�
//	{
//		PL010V13_PrintfString(0		,120	,16		,"��ȡ��λ��Ϣ������");				//����״̬
//		NumFW=0;
//		Onlinede=0;
//		memset(SumFed,0x00,8);
//		memset(SumFQ,0x00,8);

//		PL010V13_PrintfString(96		,0	,16	,"%2d",0);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
//		PL010V13_PrintfString(96		,20	,16	,"%2d",0);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//	}
//	
//	for(DelayTime=0;DelayTime<1000;DelayTime++)
//	{
//		
//	}
//	if(RxNum&&(RevBuffe[0]==0x82))		//����״̬��ʾ
//	{
//		switch(RevBuffe[3])
//		{
//			case 0x00:	PL010V13_PrintfString(0		,120	,16	,"����������������������������");				//����״̬
//				break;
//			case 0x80:	PL010V13_PrintfString(0		,120	,16	,"ҩƷ����ס������������������");				//����״̬
//				break;
//			case 0x81:	PL010V13_PrintfString(0		,120	,16	,"ȱҩ������������������������");				//����״̬
//				break;
//			case 0x82:	PL010V13_PrintfString(0		,120	,16	,"�ȴ���������ʱ������������");				//����״̬
//				break;
//			case 0xC0:	PL010V13_PrintfString(0		,120	,16	,"��Ԫ�������ͨ���쳣��������");				//����״̬
//				break;
//			case 0xC1:	PL010V13_PrintfString(0		,120	,16	,"��ҩ�������ͨ���쳣��������");				//����״̬
//				break;
//		}
////		RevBuffe[0]=0;
////		RevBuffe[1]=0;
////		RevBuffe[2]=0;
////		RevBuffe[3]=0;
//	}
//#endif
//}
///*******************************************************************************
//*������		:	LCD_ShowString
//*��������	:	��ʾ�ַ�����ͨ�ֿ�
//*����			: x,y:�������
//						*p:�ַ�����ʼ��ַ
//						��16����
//*���			:	��
//*����ֵ		:	��
//*����			:
//*******************************************************************************/
//unsigned int PL010V13_PrintfString(u16 x,u16 y,u8 font,const char *format,...)				//��ߵ�ʡ�Ժž��ǿɱ����
//{ 
//		
////		va_list ap; 										//VA_LIST ����C�����н����������һ��꣬����ͷ�ļ���#include <stdarg.h>,���ڻ�ȡ��ȷ�������Ĳ���
////		static char string[ 256 ];			//�������飬
////  	va_start( ap, format );
////		vsprintf( string , format, ap );    
////		va_end( ap );
//	
////	char	*Char_Buffer=NULL;		//��¼format����
//	u16 i=0;		//��ʾ

//	//1)**********��ȡ���ݿ���
//	u16 num=strlen((const char*)format);		//��ȡ���ݿ���
//	//2)**********���建������С����
//	unsigned int BufferSize;
//	//3)**********argsΪ�����һ��ָ��ɱ�����ı�����va_list�Լ��±�Ҫ�õ���va_start,va_end�������ڶ��壬�ɱ���������б���Ҫ�õ��꣬ ��stdarg.hͷ�ļ��ж���
//	va_list args; 
//	free(Char_Buffer);						//�ͷŶ�̬�ռ�	
//	//4)**********���붯̬�ռ�
////	Char_Buffer = (char*)malloc(sizeof(char) * num);
////	if(Char_Buffer==NULL)
////	{
////		Char_Buffer=NULL;
////		return 0;
////	}
//	//5)**********��ʼ��args�ĺ�����ʹ��ָ��ɱ�����ĵ�һ��������format�ǿɱ������ǰһ������
//	va_start(args, format);
//	//6)**********��������·��������ִ��ĳ���(��ȥ\0),����������ظ�ֵ
//	BufferSize = vsprintf(Char_Buffer, format, args);
//	num=BufferSize;
//	//7)**********�����ɱ�����Ļ�ȡ
//	va_end(args);                                      		
//	//8)**********���ȷ��ͻ�������С�����ݸ���������������ַ����DMA��������
////	while(*Char_Buffer!='\0')
//	for(i=0;i<num;i++)
//	{ 
//		unsigned char dst=Char_Buffer[i];
////		u8 GTBuffer[512]={0};		//�������ݴ洢�ռ�
//		u32 lengh=0;						//���ֵ�������ݳ���		
//		if(dst>0x80)		//˫�ֽ�--����
//		{
//			u16 word=dst<<8;			
////			Char_Buffer++;
//			dst=Char_Buffer[i+1];
//			word=word|dst;			
//			//��ʾ�����ж�
//			if(font==16&&x>R61509V_W-16)
//			{
//				x=0;
//				y+=16;
//			}
//			if(font==32&&x>R61509V_W-32)
//			{
//				x=0;
//				y+=32;
//			}
//			if(font==16&&y>R61509V_H-16)
//			{
//				y=x=0;
//			}
//			if(font==32&&y>R61509V_H-32)
//			{
//				y=x=0;
//			}
//			lengh=GT32L32_ReadBuffer(&GT32L32_Info,font,word,GT32L32_Info.GT32L32_Data.GT32L32_Buffer);		//���ֿ��ж����ݺ���
//			//д����Ļ
//			R61509V_ShowChar(x,y,font,lengh,GT32L32_Info.GT32L32_Data.GT32L32_Buffer);
//			//��ʾ��ַ����	
//			if(font==12)
//			{
//				x+=12;
//			}
//			else if(font==16)
//			{
//				x+=16;
//			}
//			else if(font==24)
//			{
//				x+=24;
//			}
//			else if(font==32)
//			{
//				x+=32;
//			}
////			Char_Buffer++;
//			i++;		//˫�ֽڣ�������
//		}
//		else		//���ֽ�
//		{			
//			if(x>R61509V_W-16)
//			{
//				x=0;
//				y+=32;
//			}
//			if(font==16&&y>R61509V_H-16)
//			{
//				y=x=0;
//			}
//			if(font==32&&y>R61509V_H-32)
//			{
//				y=x=0;
//			}
//			lengh=GT32L32_ReadBuffer(&GT32L32_Info,font,(u16)dst,GT32L32_Info.GT32L32_Data.GT32L32_Buffer);		//���ֿ��ж����ݺ���
////			//д����Ļ
//			R61509V_ShowChar(x,y,font,lengh,GT32L32_Info.GT32L32_Data.GT32L32_Buffer);			
//			//��ʾ��ַ����
//			if(font==12)
//			{
//				x+=6;
//			}
//			else if(font==16)
//			{
//				x+=8;
//			}
//			else if(font==24)
//			{
//				x+=12;
//			}
//			else if(font==32)
//			{
//				x+=16;
//			}			
////			Char_Buffer++;
////			i++;		//˫�ֽڣ�������
//		}
//	}
//	//9)**********DMA������ɺ�ע��Ӧ���ͷŻ�������free(USART_BUFFER);
////	free(Char_Buffer);		//������ɺ�ע��Ӧ���ͷŻ�������free(Char_Buffer); 
//	return BufferSize;
//}
///*******************************************************************************
//* ������		:	
//* ��������	:	 
//* ����		:	
//* ���		:
//* ���� 		:
//*******************************************************************************/
//void PL010V13_DISPLAY(void)
//{
////	R61509V_DrawPixelEx( 100, 100,LCD_FORE_COLOR);
////	R61509V_DrawHLine( 10, 100, 200, LCD_FORE_COLOR);
//}
//void PL010Delay(u32 time)
//{	
//	while(time--);
//}
///*******************************************************************************
//* ������			:	function
//* ��������		:	��������˵�� 
//* ����			: void
//* ����ֵ			: void
//*******************************************************************************/
//void LCD_WS(void)		//λ��˸
//{
//	if(NumFW)		//��λ��˸	DSPTime
//	{
//		if(NumFW==1&&(Onlinede>>0&0x01))
//		{
//			if(DSPTime==500)
//			{
//				R61509V_Fill(2,180,50,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//			}
//			else if(DSPTime==250)
//			{
//				R61509V_Fill(2,180,50,200-2,R61509V_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//			}
//		}
//		else if(NumFW==2&&(Onlinede>>1&0x01))
//		{
//			if(DSPTime==500)
//			{
//				R61509V_Fill(52,180,100,200-2,R61509V_GBLUE);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//			}
//			else if(DSPTime==250)
//			{
//				R61509V_Fill(52,180,100,200-2,R61509V_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//			}			
//		}
//		else if(NumFW==3&&(Onlinede>>2&0x01))
//		{
//			if(DSPTime==500)
//			{
//				R61509V_Fill(102,180,150,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//			}
//			else if(DSPTime==250)
//			{
//				R61509V_Fill(102,180,150,200-2,R61509V_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//			}			
//		}
//		else if(NumFW==4&&(Onlinede>>3&0x01))
//		{
//			if(DSPTime==500)
//			{
//				R61509V_Fill(152,180,200,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//			}
//			else if(DSPTime==250)
//			{
//				R61509V_Fill(152,180,200,200-2,R61509V_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//			}			
//		}
//		else if(NumFW==5&&(Onlinede>>4&0x01))
//		{
//			if(DSPTime==500)
//			{
//				R61509V_Fill(202,180,250,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//			}
//			else if(DSPTime==250)
//			{
//				R61509V_Fill(202,180,250,200-2,R61509V_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//			}			
//		}
//		else if(NumFW==6&&(Onlinede>>5&0x01))
//		{
//			if(DSPTime==500)
//			{
//				R61509V_Fill(252,180,300,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//			}
//			else if(DSPTime==250)
//			{
//				R61509V_Fill(252,180,300,200-2,R61509V_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//			}			
//		}
//		else if(NumFW==7&&(Onlinede>>6&0x01))
//		{
//			if(DSPTime==500)
//			{
//				R61509V_Fill(302,180,350,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//			}
//			else if(DSPTime==250)
//			{
//				R61509V_Fill(302,180,350,200-2,R61509V_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//			}			
//		}
//		else if(NumFW==8&&(Onlinede>>7&0x01))
//		{
//			if(DSPTime==500)
//			{
//				R61509V_Fill(352,180,400-2,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//			}
//			else if(DSPTime==250)
//			{
//				R61509V_Fill(352,180,400-2,200-2,R61509V_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//			}			
//		}
//	}
//}
///*******************************************************************************
//* ������			:	function
//* ��������		:	��������˵�� 
//* ����			: void
//* ����ֵ			: void
//*******************************************************************************/
//void LCD_WXS(void)		//λ��ʾ
//{
//		if((Onlinede>>0&0x01)==0x01)
//		{
//			R61509V_Fill(2,180,50,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//		}
//		else
//		{
//			R61509V_Fill(2,180,50,200-2,R61509V_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//		}
//		if((Onlinede>>1&0x01)==0x01)
//		{
//			R61509V_Fill(52,180,100,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//		}
//		else
//		{
//			R61509V_Fill(52,180,100,200-2,R61509V_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//		}
//		if((Onlinede>>2&0x01)==0x01)
//		{
//			R61509V_Fill(102,180,150,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//		}
//		else
//		{
//			R61509V_Fill(102,180,150,200-2,R61509V_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//		}
//		if((Onlinede>>3&0x01)==0x01)
//		{
//			R61509V_Fill(152,180,200,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//		}
//		else
//		{
//			R61509V_Fill(152,180,200,200-2,R61509V_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//		}
//		if((Onlinede>>4&0x01)==0x01)
//		{
//			R61509V_Fill(202,180,250,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//		}
//		else
//		{
//			R61509V_Fill(202,180,250,200-2,R61509V_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//		}
//		if((Onlinede>>5&0x01)==0x01)
//		{
//			R61509V_Fill(252,180,300,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//		}
//		else
//		{
//			R61509V_Fill(252,180,300,200-2,R61509V_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//		}
//		if((Onlinede>>6&0x01)==0x01)
//		{
//			R61509V_Fill(302,180,350,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//		}
//		else
//		{
//			R61509V_Fill(302,180,350,200-2,R61509V_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//		}
//		if((Onlinede>>7&0x01)==0x01)
//		{
//			R61509V_Fill(352,180,400-2,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//		}
//		else
//		{
//			R61509V_Fill(352,180,400-2,200-2,R61509V_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
//		}
//}
///*******************************************************************************
//* ������			:	function
//* ��������		:	��������˵�� 
//* ����			: void
//* ����ֵ			: void
//*******************************************************************************/
//void LCD_DDSP(void)		//��ʾ�ܹ������������ѷ�����
//{
////	u8 i=0;
////	u8 w=2;
//	//����������ʾ
////	for(i=0;i<8;i++)
////	{
////		PL010V13_PrintfString(w,200,16	,"%5d",SumFQ[i]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
////		w+=50;
////	}
//	//����������ʾ
//	PL010V13_PrintfString(2,200,16	,"%5d",SumFQ[0]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//	PL010V13_PrintfString(52,200,16	,"%5d",SumFQ[1]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//	PL010V13_PrintfString(102,200,16	,"%5d",SumFQ[2]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//	PL010V13_PrintfString(152,200,16	,"%5d",SumFQ[3]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//	PL010V13_PrintfString(201,200,16	,"%5d",SumFQ[4]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//	PL010V13_PrintfString(251,200,16	,"%5d",SumFQ[5]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//	PL010V13_PrintfString(301,200,16	,"%5d",SumFQ[6]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//	PL010V13_PrintfString(351,200,16	,"%5d",SumFQ[7]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����

//	//�ѷ�������ʾ	
//	PL010V13_PrintfString(2,220,16		,"%5d",SumFed[0]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//	PL010V13_PrintfString(52,220,16		,"%5d",SumFed[1]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//	PL010V13_PrintfString(102,220,16	,"%5d",SumFed[2]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//	PL010V13_PrintfString(152,220,16	,"%5d",SumFed[3]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//	PL010V13_PrintfString(201,220,16	,"%5d",SumFed[4]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//	PL010V13_PrintfString(251,220,16	,"%5d",SumFed[5]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//	PL010V13_PrintfString(301,220,16	,"%5d",SumFed[6]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//	PL010V13_PrintfString(351,220,16	,"%5d",SumFed[7]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//}

#endif
