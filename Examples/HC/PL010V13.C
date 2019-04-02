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

#ifdef PL010V13				//����ҩ�ܳ���LCD��

#include "PL010V13.H"

#include "R61509V.h"
#include "CS5530.H"

#include "GT32L32M0180.H"
#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_PWM.H"
#include "STM32_USART.H"

#include "string.h"				//�����ڴ��������ͷ�ļ�
#include "stm32f10x_dma.h"

u16	DelayTime=0x1000;
u16	LCDTime=0;
u16	DSPTime=0;
u32 testADC=0;
u32 bacADC=0;
u32 bacADC2=0;

u16 ADC_dotx=0;
u16 ADC_doty=0;
u16 ADC_dotx1=0;
u16 ADC_doty1=0;

u16 SumFed[32]={0};		//�ܹ��ѷ�ҩ����
u16	SumFQ[32]={0};			//�ܹ���ҩ��������
u8	NumFW=0;		//����ҩ��λ
u8	Onlinede=0;		//����ҩ��λ


LCDDef LcdInfo;

//GT32L32Def 	GT32L32Info;
u32 CS5530_ADC_Value=0xFFFFFFFF;
//t_Point point;
u8 zimo[720]="R61509V_DrawRectangle(11,11,229,389,0X07FF)";

RS485_TypeDef  RS485_Conf;

u8 RxdBuffe[256]={0};
u8 RevBuffe[256]={0};
u16 RxNum=0;
char	Char_Buffer[256]={0xFF};		//��¼format����
//t_LcdCfg **pLcdpara;

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PL010V13_Configuration(void)
{
	SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H
	
	GPIO_DeInitAll();							//�����е�GPIO�ر�----V20170605
	
	PL010V13_PinSet();
	
	RS485_DMA_ConfigurationNR	(&RS485_Conf,115200,128);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
	
	SysTick_Configuration(1000);	//ϵͳ���ʱ������72MHz,��λΪuS
	
//	IWDG_Configuration(1000);			//�������Ź�����---������λms	
	
//	PWM_OUT(TIM2,PWM_OUTChannel1,1,200);		//PWM�趨-20161127�汾
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PL010V13_Server(void)
{	
	IWDG_Feed();								//�������Ź�ι��
	
	DelayTime++;
	
	LCDTime++;
	
//	if(LCDTime==500)
//	{
//		PL010V13_PrintfString(0		,134+50+16	,16	,"��ҩ�������ͨ���쳣��������");				//����״̬
//	}
//	else if(LCDTime==1000)
//	{
//		PL010V13_PrintfString(0		,134+50+16	,16	,"XXXXXXXXXXXXXXXXXXXXXXXXXX");				//����״̬
//	}	
//	LCD_DrawLine(0,120,400,120,LCD565_WHITE);						//AB �������껭һ��ֱ��
	DSPTime++;
	if(DSPTime>500)
	{
		DSPTime=0;
//		LCD_WXS();		//λ��ʾ
//		LCD_DDSP();		//��ʾ�ܹ������������ѷ�����
	}
	LCD_Printf(96		,80	,16	,"����������������������������");				//����״̬
	return;
	LCD_WS();		//λ��˸

	if(LCDTime>=1000)
	{			
		LCDTime=0;		
	}	
	
	RxNum=RS485_ReadBufferIDLE(&RS485_Conf,RevBuffe);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(RxNum==4&&RevBuffe[0]==0x00&&RevBuffe[1]==0xFF)		//RS485���յ�����
	{
		NumFW=RevBuffe[2];
		LCD_Printf(96		,0	,16	,"%2d",RevBuffe[2]);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		LCD_Printf(96		,20	,16	,"%2d",RevBuffe[3]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//		PL010V13_PrintfString(192		,68	,16	,"%2d",RevBuffe[1]);				//�ѷ�ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	}
	else if(RxNum==3&&RevBuffe[0]==0x02)		//RS485���յ�����
	{
		SumFQ[RevBuffe[1]-1]+=RevBuffe[2];
		NumFW=RevBuffe[1];
		LCD_Printf(96		,0	,16	,"%2d",RevBuffe[1]);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		LCD_Printf(96		,20	,16	,"%2d",RevBuffe[2]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		LCD_Printf(96		,40	,16	,"%2d",0);									//�ѷ�ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		
//		PL010V13_PrintfString(0		,60	,16	,"�ܹ�����������%4d",SumFQ);				//�ܹ���ҩ��������
		LCD_Printf(0		,120	,16	,"���ڷ�ҩ��������������������");				//����״̬
	}
	else if(RxNum==4&&RevBuffe[0]==0x82)		//RS485���յ�����
	{
		SumFed[RevBuffe[1]-1]+=RevBuffe[2];
		LCD_Printf(96		,0	,16	,"%2d",RevBuffe[1]);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		LCD_Printf(96		,20	,16	,"%2d",0);									//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		LCD_Printf(96		,40	,16	,"%2d",RevBuffe[2]);				//�ѷ�ҩ��������ߵ�ʡ�Ժž��ǿɱ����		
		
//		PL010V13_PrintfString(112		,80	,16	,"%4d",SumFed);					//�ܹ��ѷ�ҩ����
		
	}
	else if(RxNum==6&&RevBuffe[0]==0x81)	//��λ��Ϣ0x01--��ȡ,0x81--�ϱ�
	{
		Onlinede=RevBuffe[4];
	}
	else if(RxNum==1&&RevBuffe[0]==0x01)	//��λ��Ϣ0x01--��ȡ,0x81--�ϱ�
	{
		LCD_Printf(0		,120	,16		,"��ȡ��λ��Ϣ������");				//����״̬
		NumFW=0;
		Onlinede=0;
		memset(SumFed,0x00,8);
		memset(SumFQ,0x00,8);

		LCD_Printf(96		,0	,16	,"%2d",0);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		LCD_Printf(96		,20	,16	,"%2d",0);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	}
	LCD_Printf(60		,80	,16	,"����������������������������");				//����״̬
	for(DelayTime=0;DelayTime<1000;DelayTime++)
	{
		
	}
	if(RxNum&&(RevBuffe[0]==0x82))		//����״̬��ʾ
	{
		switch(RevBuffe[3])
		{
			case 0x00:	LCD_Printf(96		,80	,16	,"����������������������������");				//����״̬
				break;
			case 0x80:	LCD_Printf(30		,80	,16	,"ҩƷ����ס������������������");				//����״̬
				break;
			case 0x81:	LCD_Printf(30		,80	,16	,"ȱҩ������������������������");				//����״̬
				break;
			case 0x82:	LCD_Printf(30		,80	,16	,"�ȴ���������ʱ������������");				//����״̬
				break;
			case 0xC0:	LCD_Printf(30		,80	,16	,"��Ԫ�������ͨ���쳣��������");				//����״̬
				break;
			case 0xC1:	LCD_Printf(30		,80	,16	,"��ҩ�������ͨ���쳣��������");				//����״̬
				break;
		}
//		RevBuffe[0]=0;
//		RevBuffe[1]=0;
//		RevBuffe[2]=0;
//		RevBuffe[3]=0;
	}
	
	#if 0
	if(DelayTime>=100)
	{
		
		DelayTime=0;
		CS5530_ADC_Value=CS5530_ReadData(&CS5530_Pinfo);	//��ȡADֵ���������0xFFFFFFFF,��δ��ȡ��24λADֵ
//		R61509V_Fill(0,0,64,16,R61509V_BLACK);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		R61509V_ShowEn(0,0,CS5530_ADC_Value);
		
		if(CS5530_ADC_Value!=0xFFFFFFFF)
		{
			if(testADC==0)
			{				
				testADC=CS5530_ADC_Value;
				bacADC=CS5530_ADC_Value;
//				R61509V_Clean(R61509V_BLACK);			//�����Ļ����--����	
//				R61509V_DrawLine(0,120,400,120,0X5458);						//AB �������껭һ��ֱ��				
//				R61509V_DrawLine(0,230,400,230,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,220,400,220,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,210,400,210,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,200,400,200,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,190,400,190,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,180,400,180,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,170,400,170,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,160,400,160,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,150,400,150,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,140,400,140,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,130,400,130,0X5458);						//AB �������껭һ��ֱ��
				R61509V_DrawLine(0,120,400,120,R61509V_YELLOW);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,110,400,110,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,100,400,100,R61509V_YELLOW);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,90,400,90,0X5458);							//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,80,400,80,0X5458);							//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,70,400,70,0X5458);							//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,60,400,60,0X5458);							//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,50,400,50,0X5458);							//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,40,400,40,0X5458);							//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,30,400,30,0X5458);							//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,20,400,20,0X5458);							//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,10,400,10,0X5458);							//AB �������껭һ��ֱ��
			}
			bacADC2=CS5530_ADC_Value;
			
			if(CS5530_ADC_Value>=bacADC)
			{				
				CS5530_ADC_Value=CS5530_ADC_Value-bacADC;
			}
			else
			{
				CS5530_ADC_Value=bacADC-CS5530_ADC_Value;
			}
			bacADC=bacADC2;				
//			R61509V_Fill(57,8,100,16,0xF800);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			R61509V_ShowEn(57,0,CS5530_ADC_Value);
			
			ADC_dotx++;
			
			if(ADC_dotx>=400)
			{
				ADC_dotx=0;
				R61509V_Clean(R61509V_BLACK);			//�����Ļ����--����
				
//				R61509V_DrawLine(0,120,400,120,0X5458);						//AB �������껭һ��ֱ��
				
				
//				R61509V_DrawLine(0,230,400,230,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,220,400,220,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,210,400,210,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,200,400,200,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,190,400,190,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,180,400,180,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,170,400,170,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,160,400,160,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,150,400,150,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,140,400,140,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,130,400,130,0X5458);						//AB �������껭һ��ֱ��
				R61509V_DrawLine(0,120,400,120,R61509V_YELLOW);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,110,400,110,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,100,400,100,R61509V_YELLOW);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,90,400,90,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,80,400,80,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,70,400,70,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,60,400,60,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,50,400,50,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,40,400,40,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,30,400,30,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,20,400,20,0X5458);						//AB �������껭һ��ֱ��
//				R61509V_DrawLine(0,10,400,10,0X5458);						//AB �������껭һ��ֱ��
				
				ADC_dotx1=0;
				ADC_doty1=0;
			}
			ADC_doty=CS5530_ADC_Value;
			
			if(ADC_doty>=240)
				ADC_doty=230;
			
//			ADC_doty=ADC_doty/10;
			
			ADC_doty=(R61509V_H-ADC_doty+1);
			
//			R61509V_DrawDot(ADC_dotx,ADC_doty,0X07FF);			//����
//			R61509V_DrawLine(ADC_dotx1,ADC_doty1,ADC_dotx,ADC_doty,R61509V_BRRED);						//AB �������껭һ��ֱ��
			ADC_dotx1=ADC_dotx;
			ADC_doty1=ADC_doty;
		}
		else
		{
			R61509V_PinConf(&R61509V_Pinfo);
			
		}
	}
	#else
//	R61509V_ShowEn(0,112,DelayTime);
//	R61509V_Clean(DelayTime);			//�����Ļ����
//	R61509V_Delay(5000);
	if(DelayTime==1)
	{
//		R61509V_Clean(R61509V_BLACK);			//�����Ļ����--����
//		R61509V_Fill(0,0,400,240,R61509V_BLACK);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)--90/270		
//		R61509V_Fill(0,0,240,400,R61509V_BLACK);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)--0/180
		
//		R61509V_DrawLine(0,120,400,120,R61509V_RED);						//AB �������껭һ��ֱ��--90/270
//		R61509V_DrawLine(200,0,200,240,R61509V_YELLOW);						//AB �������껭һ��ֱ��--90/270
//		R61509V_DrawLine(0,0,400,240,R61509V_MAGENTA);						//AB �������껭һ��ֱ��--90/270
//		R61509V_DrawLine(400,0,0,240,R61509V_BLUE);						//AB �������껭һ��ֱ��--90/270
	}
	else if(DelayTime==500)
	{
//			R61509V_Clean(R61509V_CYAN);			//�����Ļ����--����	
//		R61509V_Fill(0,0,400,240,R61509V_CYAN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)--90/270		
//		R61509V_Fill(0,0,240,400,R61509V_CYAN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)--0/180
		
//		R61509V_DrawLine(0,120,400,120,R61509V_YELLOW);						//AB �������껭һ��ֱ��--90/270
//		R61509V_DrawLine(200,0,200,240,R61509V_RED);						//AB �������껭һ��ֱ��--90/270
//		R61509V_DrawLine(0,0,400,240,R61509V_BLUE);						//AB �������껭һ��ֱ��--90/270
//		R61509V_DrawLine(400,0,0,240,R61509V_MAGENTA);						//AB �������껭һ��ֱ��--90/270
	}
	else if(DelayTime>=1000)
	{
			DelayTime=0;
			
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
void PL010V13_PinSet(void)
{	
	u16 i=0,j=0;
	
	RS485_Conf.USARTx=USART1;
	RS485_Conf.RS485_CTL_PORT=GPIOA;
	RS485_Conf.RS485_CTL_Pin=GPIO_Pin_11;
	
//	GT32L32Info.SPI.Port
	LcdInfo.GT32L32.SPI.Port.SPIx=SPI1;
	LcdInfo.GT32L32.SPI.Port.CS_PORT=GPIOA;
	LcdInfo.GT32L32.SPI.Port.CS_Pin=GPIO_Pin_4;
	LcdInfo.GT32L32.SPI.Port.SPI_BaudRatePrescaler_x=SPI_BaudRatePrescaler_128;

	
	//LcdInfo.Port
	LcdInfo.Port.sCS_PORT		=	GPIOA;
	LcdInfo.Port.sCS_Pin		=	GPIO_Pin_12;
	
	LcdInfo.Port.sDC_PORT		=	GPIOA;
	LcdInfo.Port.sDC_Pin		=	GPIO_Pin_8;
	
	LcdInfo.Port.sWR_PORT		=	GPIOA;
	LcdInfo.Port.sWR_Pin		=	GPIO_Pin_15;
	
	LcdInfo.Port.sRD_PORT		=	GPIOC;
	LcdInfo.Port.sRD_Pin		=	GPIO_Pin_5;
	
	LcdInfo.Port.sREST_PORT	=	GPIOD;
	LcdInfo.Port.sREST_Pin	=	GPIO_Pin_2;
	
	LcdInfo.Port.sBL_PORT		=	GPIOA;
	LcdInfo.Port.sBL_Pin		=	GPIO_Pin_3;
	
	LcdInfo.Port.sTE_PORT		=	GPIOC;
	LcdInfo.Port.sTE_Pin		=	GPIO_Pin_4;
	
	LcdInfo.Port.sDATABUS_PORT	=	GPIOB;
	LcdInfo.Port.sDATABUS_Pin	=	GPIO_Pin_All;
	
	LcdInfo.Flag.Rotate	=	Draw_Rotate_90D;	//ʹ����ת�Ƕ�	
	
	R61509V_Initialize(&LcdInfo);
	

	LCD_Clean(LCD565_YELLOW);			//�����Ļ����--��ɫ
	LCD_Delay(0xFFFF);
	
	LCD_Clean(LCD565_RED);			//�����Ļ����--��ɫ
	LCD_Delay(0xFFFF);
	
//	LCD_Clean(LCD565_GRAY);			//�����Ļ����--��ɫ
//	LCD_Delay(0xFFFFFF);
//	
//	LCD_Clean(LCD565_GREEN);			//�����Ļ����--��ɫ
//	LCD_Delay(0xFFFFFF);
//	
//	LCD_Clean(LCD565_WHITE);			//�����Ļ����--����
//	LCD_Delay(0xFFFFFF);

	LCD_Clean(LCD565_BLACK);			//�����Ļ����------
	LCD_Delay(0xFFFFFF);

	#if 1
	LCD_Delay(0xFFFF);
	LCD_Printf(1		,0	,16	,"����ҩ��λ��%2d",RevBuffe[0]);				//��ߵ�ʡ�Ժž��ǿɱ����
	LCD_Delay(0xFFFF);
	LCD_Printf(1		,20	,16	,"����ҩ������%2d",RevBuffe[1]);				//��ߵ�ʡ�Ժž��ǿɱ����
	LCD_Delay(0xFFFF);
	LCD_Printf(1		,40	,16	,"�ѷ�ҩ������%2d",RevBuffe[1]);				//��ߵ�ʡ�Ժž��ǿɱ����
	LCD_Delay(0xFFFF);
	LCD_Printf(1		,60	,16	,"��ʾ������3���ȡ��ҩͷ");				//����״̬
	LCD_Delay(0xFFFF);
	LCD_Printf(1		,80	,16	,"����״̬��");				//����״̬
	LCD_Delay(0xFFFF);
	
	
	

	//ƽ����
//	for(i	=	16;i<240;)
//	{
//		LCD_DrawLine(0,i,400,i,LCD565_WHITE);						//AB �������껭һ��ֱ��
//		LCD_Delay(0xFFFF);
//		i+=16;
//	}
	//���
	LCD_Fill(0,0,400,240,LCD565_BLACK);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)	
//	LCD_Clean(LCD565_BLACK);			//�����Ļ����------
//	LCD_Delay(0xFFFFFF);
	
	#else
//	R61509V_Clean(R61509V_WHITE);			//�����Ļ����------
//	PL010Delay(0x8FFFFF);
	LCD_Clean(LCD565_BLUE);			//�����Ļ����------
	LCD_Delay(0x8FFFFF);
	LCD_Clean(LCD565_GRED);			//�����Ļ����------
	LCD_Delay(0x8FFFFF);
//	R61509V_Clean(R61509V_BLACK);			//�����Ļ����------
//	PL010Delay(0xFFFF);
	LCD_Printf(0	,0,16	,"TEST");				//����״̬
	#endif
//	R61509V_Clean(R61509V_BLACK);			//�����Ļ����------
	
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
	unsigned short x,y;
	for(y=180;y<240;)
	{
		for(x=4;x<=400;)
		{
			LCD_Fill(x,y,x+40,y+15,LCD565_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			x+=50;
		}
		y+=18;
	}
	return;
	
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
				LCD_Fill(52,180,100,200-2,LCD565_BLUE);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
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
	unsigned short x,y;
	for(y=180;y<240;)
	{
		for(x=4;x<=400;)
		{
			LCD_Fill(x,y,x+40,y+15,LCD565_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			x+=50;
		}
		y+=18;
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
	unsigned short x,y,m,i;

	y=240-17*12;
	x	=	4;
	for(i=0;i<8;i++)
	{
		LCD_Printf(x,y,16	,"AR%0.2d",i+1);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		x+=400/8;
	}
	y=240-17*11;
	x	=	4;
	for(i=0;i<8;i++)
	{
		LCD_Printf(x,y,16	,"%0.4d",SumFQ[i]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		x+=400/8;
	}
	y=240-17*10;
	x	=	4;
	for(i=0;i<8;i++)
	{
		LCD_Printf(x,y,16	,"%0.4d",SumFed[i]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		x+=400/8;
	}	
	y=240-17*9;
	x	=	4;
	for(i=0;i<8;i++)
	{
		LCD_Printf(x,y,16	,"AR%0.2d",i+1);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		x+=400/8;
	}
	y=240-17*8;
	x	=	4;
	for(i=0;i<8;i++)
	{
		LCD_Printf(x,y,16	,"%0.4d",SumFQ[i]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		x+=400/8;
	}
	y=240-17*7;
	x	=	4;
	for(i=0;i<8;i++)
	{
		LCD_Printf(x,y,16	,"%0.4d",SumFed[i]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		x+=400/8;
	}
	
		y=240-17*6;
	x	=	4;
	for(i=0;i<8;i++)
	{
		LCD_Printf(x,y,16	,"AR%0.2d",i+1);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		x+=400/8;
	}
	y=240-17*5;
	x	=	4;
	for(i=0;i<8;i++)
	{
		LCD_Printf(x,y,16	,"%0.4d",SumFQ[i]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		x+=400/8;
	}
	y=240-17*4;
	x	=	4;
	for(i=0;i<8;i++)
	{
		LCD_Printf(x,y,16	,"%0.4d",SumFed[i]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		x+=400/8;
	}
	
		y=240-17*3;
	x	=	4;
	for(i=0;i<8;i++)
	{
		LCD_Printf(x,y,16	,"AR%0.2d",i+1);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		x+=400/8;
	}
	y=240-17*2;
	x	=	4;
	for(i=0;i<8;i++)
	{
		LCD_Printf(x,y,16	,"%0.4d",SumFQ[i]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		x+=400/8;
	}
	y=240-17*1;
	x	=	4;
	for(i=0;i<8;i++)
	{
		LCD_Printf(x,y,16	,"%0.4d",SumFed[i]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		x+=400/8;
	}
	return;
}
#endif