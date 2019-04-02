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

#ifdef PL012V11LCD				//�����LCD��--ILI9326����

#include "PL012V11LCD.H"


#define	USE_R61509V
#define	USE_ILI9326

#ifdef	USE_R61509V
	#include "R61509V.h"
	R61509V_Pindef R61509V;
#else

	#include "ILI9326.h"
	sLCD_def sLCD;
#endif

#include "CS5530.H"

#include "GT32L32M0180.H"
#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_PWM.H"
#include "STM32_USART.H"


#include "SWITCHID.H"

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

u16 SumFed[8]={0};		//�ܹ��ѷ�ҩ����
u16	SumFQ[8]={0};			//�ܹ���ҩ��������
u8	NumFW=0;		//����ҩ��λ
u8	Onlinede=0;		//����ҩ��λ



GT32L32_Info_TypeDef 	GT32L32_Info;
u32 CS5530_ADC_Value=0xFFFFFFFF;
t_Point point;
u8 zimo[720]="R61509V_DrawRectangle(11,11,229,389,0X07FF)";

RS485_TypeDef  RS485;
u8 RS485FLG	=	0;
u32 RS485Time	=	0;
u32 RSRLen	=	0;

u8 TxdBuffe[256]={0};
u8 RxdBuffe[256]={0};
u8 RevBuffe[256]={0};
u16 RxNum=0;
char	Char_Buffer[256]={0xFF};		//��¼format����
//t_LcdCfg **pLcdpara;


SWITCHID_CONF	SWITCHID;
u8 SwitchID=0;	//���뿪�ص�ַ


CS5530_Pindef CS5530;


u32 Tedata	=	0;

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PL012V11LCD_Configuration(void)
{
	SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H
	
	GPIO_DeInitAll();							//�����е�GPIO�ر�----V20170605
	
	
	
	
	
	GT32L32_Configuration();
	
	LCD_Configuration();
	
//	CS5530_Configuration();
	
	RS485_Configuration();
//	PWM_OUT(TIM3,PWM_OUTChannel3,500,200);		//PWM�趨-20161127�汾--����
	LCD_PowerUp();
	
	SwitchID_Configuration();
	
	
	SysTick_Configuration(1000);	//ϵͳ���ʱ������72MHz,��λΪuS
	
//	IWDG_Configuration(1000);			//�������Ź�����---������λms	
	PWM_OUT(TIM2,PWM_OUTChannel1,1,900);	//PWM�趨-20161127�汾--ָʾ��
	PWM_OUT(TIM3,PWM_OUTChannel3,1000,500);		//PWM�趨-20161127�汾--����
	memset(TxdBuffe,0xA5,128);
//	ILI9326_DrawHLine( 100, 300, 100, 100 );
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PL012V11LCD_Server(void)
{	
	IWDG_Feed();								//�������Ź�ι��
	
//	RS485_Server();		//ͨѶ����
	if(DelayTime++>=100)
	{
		DelayTime	=	0;
		if(Tedata++>10000)
		{
			Tedata	=	0;
		}
		PL010V13_PrintfString(0		,128	,32	,"%0.4d",Tedata);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
	}
	if(LCDTime++>=2000)
	{
		LCDTime	=	0;
//		R61509V_Clean(R61509V_BLACK); 					//�����Ļ����
//		LCD_Display();
	}
//	SwitchID_Server();
	
	
	 
//	if(LCDTime==500)
//	{
//		PL010V13_PrintfString(0		,134+50+16	,16	,"��ҩ�������ͨ���쳣��������");				//����״̬
//	}
//	else if(LCDTime==1000)
//	{
//		PL010V13_PrintfString(0		,134+50+16	,16	,"XXXXXXXXXXXXXXXXXXXXXXXXXX");				//����״̬
//	}	

	
#if 0
	if(DelayTime==0)
	{
		
		DelayTime=0;
		CS5530_ADC_Value=CS5530_ReadData(&CS5530);	//��ȡADֵ���������0xFFFFFFFF,��δ��ȡ��24λADֵ
//		R61509V_Clean(R61509V_BLACK); 					//�����Ļ����
//		R61509V_ShowEn(0,0,CS5530_ADC_Value);
//		PL010V13_PrintfString(0		,0	,16	,"%2d",SwitchID);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		PL010V13_PrintfString(120		,100	,32	,"%8d",CS5530_ADC_Value);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
	}
#endif
//	if(LCDTime==1)
//	ILI9326_Clean(ILI9326_RED);			//�����Ļ����------
//	else if(LCDTime==1000)
//	ILI9326_Clean(ILI9326_GREEN);			//�����Ļ����------
//	else if(LCDTime==200)
//	ILI9326_Clean(ILI9326_BLUE);			//�����Ļ����------
//	else if(LCDTime==300)
//	ILI9326_Clean(ILI9326_BRED);			//�����Ļ����------
//	else if(LCDTime==400)
//	ILI9326_Clean(ILI9326_GRED);			//�����Ļ����------
//	else if(LCDTime==500)
//	ILI9326_Clean(ILI9326_GBLUE);			//�����Ļ����------
//	else if(LCDTime==600)
//	ILI9326_Clean(ILI9326_RED);			//�����Ļ����------
//	else if(LCDTime==700)
//	ILI9326_Clean(ILI9326_MAGENTA);			//�����Ļ����------
//	else if(LCDTime==800)
//	ILI9326_Clean(ILI9326_GREEN);			//�����Ļ����------
//	else if(LCDTime==900)
//	ILI9326_Clean(ILI9326_CYAN);			//�����Ļ����------
//	else if(LCDTime==11)
//	ILI9326_Clean(ILI9326_YELLOW);			//�����Ļ����------
//	else if(LCDTime==12)
//	ILI9326_Clean(ILI9326_BROWN);			//�����Ļ����------
//	else if(LCDTime==13)
//	ILI9326_Clean(ILI9326_BRRED);			//�����Ļ����------
//	else if(LCDTime==14)
//	ILI9326_Clean(ILI9326_GRAY);			//�����Ļ����------

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
#if 0
	RxNum=RS485_ReadBufferIDLE(&RS485,(u32*)RevBuffe,(u32*)RxdBuffe);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
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
			PWM_OUT(TIM3,PWM_OUTChannel3,500,1);		//PWM�趨-20161127�汾--����
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
		PL010V13_PrintfString(0		,0	,32	,"��ţ�");				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		PL010V13_PrintfString(0		,32	,32	,"���룺");				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		PL010V13_PrintfString(96	,0	,32	,"%8d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
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
			PL010V13_PrintfString(n*16+96		,32	,32	,"%d",d);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		}
		AddrH	=	(SwitchID>>4)&0x0F;
		AddrL	=	(SwitchID>>0)&0x0F;
		PL010V13_PrintfString(0		,64	,32	,"��%0.2d��λ%0.2d",AddrH,AddrL);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
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
void GT32L32_Configuration(void)
{
	GT32L32_Info.GT32L32_Port.sSPIx=SPI1;
	GT32L32_Info.GT32L32_Port.sGT32L32_CS_PORT=GPIOA;
	GT32L32_Info.GT32L32_Port.sGT32L32_CS_PIN=GPIO_Pin_4;
	GT32L32_Info.GT32L32_Port.SPI_BaudRatePrescaler_x=SPI_BaudRatePrescaler_128;
	
	GT32L32_ConfigurationNR(&GT32L32_Info);				//��ͨSPIͨѶ��ʽ����
	
	GPIO_Configuration_OPP50	(GPIOA,GPIO_Pin_4);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
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
#ifdef USE_R61509V
	unsigned long	LCDTime	=	0xFFFFFF;
	u32 w	=	0,n	=	0;
	ILI9326.sCS_PORT		=	GPIOB;
	ILI9326.sCS_Pin			=	GPIO_Pin_7;
	
	ILI9326.sRS_PORT		=	GPIOB;
	ILI9326.sRS_Pin			=	GPIO_Pin_6;
	
	ILI9326.sWR_PORT		=	GPIOB;
	ILI9326.sWR_Pin			=	GPIO_Pin_8;
	
	ILI9326.sRD_PORT		=	GPIOB;
	ILI9326.sRD_Pin			=	GPIO_Pin_5;
	
	ILI9326.sREST_PORT	=	GPIOB;
	ILI9326.sREST_Pin		=	GPIO_Pin_9;
	
	ILI9326.sBL_PORT		=	GPIOB;
	ILI9326.sBL_Pin			=	GPIO_Pin_0;
	
	ILI9326.sTE_PORT		=	GPIOB;
	ILI9326.sTE_Pin			=	GPIO_Pin_4;
	
	ILI9326.sDATABUS_PORT	=	GPIOC;
	ILI9326.sDATABUS_Pin	=	GPIO_Pin_All;
	
	ILI9326_Initialize(&ILI9326);
	
	ILI9326_Delay(LCDTime);
//	w	=	12345678;
//	PL010V13_PrintfString(0		,0	,16	,"%8d",w);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
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

//	R61509V_Clean(0X01CF);			//�����Ļ����--��ɫ
//	R61509V_Delay(LCDTime);
//	
//	R61509V_Clean(0X07FF);			//�����Ļ����--��ɫ
//	R61509V_Delay(LCDTime);
//	
//	R61509V_Clean(0x07E0);			//�����Ļ����--��ɫ
//	R61509V_Delay(LCDTime);
//	
//	R61509V_Clean(0XFC07);			//�����Ļ����--��ɫ
//	R61509V_Delay(LCDTime);
//	
//	R61509V_Clean(0xF800);			//�����Ļ����--����
//	R61509V_Delay(LCDTime);

//	R61509V_Clean(R61509V_BLACK);			//�����Ļ����------
//	R61509V_Delay(LCDTime);
#endif
#ifdef	USE_ILI9326
	
	unsigned long	LCDTime	=	0xFFFFFF;
	u32 w	=	0,n	=	0;
	sLCD.sPort.sCS_PORT		=	GPIOB;
	sLCD.sPort.sCS_Pin			=	GPIO_Pin_7;
	
	sLCD.sPort.sRS_PORT		=	GPIOB;
	sLCD.sPort.sRS_Pin			=	GPIO_Pin_6;
	
	sLCD.sPort.sWR_PORT		=	GPIOB;
	sLCD.sPort.sWR_Pin			=	GPIO_Pin_8;
	
	sLCD.sPort.sRD_PORT		=	GPIOB;
	sLCD.sPort.sRD_Pin			=	GPIO_Pin_5;
	
	sLCD.sPort.sREST_PORT	=	GPIOB;
	sLCD.sPort.sREST_Pin		=	GPIO_Pin_9;
	
	sLCD.sPort.sBL_PORT		=	GPIOB;
	sLCD.sPort.sBL_Pin			=	GPIO_Pin_0;
	
	sLCD.sPort.sTE_PORT		=	GPIOB;
	sLCD.sPort.sTE_Pin			=	GPIO_Pin_4;
	
	sLCD.sPort.sDATABUS_PORT	=	GPIOC;
	sLCD.sPort.sDATABUS_Pin	=	GPIO_Pin_All;
	
	ILI9326_Initialize(&sLCD);
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
void CS5530_Configuration(void)
{
	CS5530.CS5530_CS_PORT=GPIOC;
	CS5530.CS5530_CS_Pin=GPIO_Pin_3;
	
	CS5530.CS5530_SDI_PORT=GPIOC;
	CS5530.CS5530_SDI_Pin=GPIO_Pin_2;
	
	CS5530.CS5530_SDO_PORT=GPIOC;
	CS5530.CS5530_SDO_Pin=GPIO_Pin_1;
	
	CS5530.CS5530_SCLK_PORT=GPIOC;
	CS5530.CS5530_SCLK_Pin=GPIO_Pin_0;
	
	CS5530_Initialize(&CS5530);
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
	
	RS485_DMA_ConfigurationNR	(&RS485,19200,(u32*)RxdBuffe,256);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
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
#ifdef	USE_R61509V
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
	R61509V_Clean(R61509V_BLUE);			//�����Ļ����------
	PL010Delay(0x8FFFFF);
	R61509V_Clean(R61509V_GRED);			//�����Ļ����------
	PL010Delay(0x8FFFFF);
	R61509V_Clean(R61509V_BLACK);			//�����Ļ����------
	PL010Delay(0x8FFFFF);
	R61509V_Clean(R61509V_WHITE);			//�����Ļ����------

//	PL010Delay(0xFFFF);
//	PL010V13_PrintfString(0	,0,16	,"TEST");				//����״̬
	#endif
//	R61509V_Clean(R61509V_BLACK);			//�����Ļ����------
#endif
#ifdef	USE_ILI9326
	
	ILI9326_Clean(ILI9326_YELLOW);			//�����Ļ����------
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
void LCD_Display(void)
{
#ifdef USE_R61509V
	PL010V13_PrintfString(0		,120	,16	,"��ʾ���ԣ�����������������");				//����״̬
	
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
		PL010V13_PrintfString(96		,0	,16	,"%2d",RevBuffe[2]);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		PL010V13_PrintfString(96		,20	,16	,"%2d",RevBuffe[3]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
//		PL010V13_PrintfString(192		,68	,16	,"%2d",RevBuffe[1]);				//�ѷ�ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	}
	else if(RxNum==3&&RevBuffe[0]==0x02)		//RS485���յ�����
	{
		SumFQ[RevBuffe[1]-1]+=RevBuffe[2];
		NumFW=RevBuffe[1];
		PL010V13_PrintfString(96		,0	,16	,"%2d",RevBuffe[1]);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		PL010V13_PrintfString(96		,20	,16	,"%2d",RevBuffe[2]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		PL010V13_PrintfString(96		,40	,16	,"%2d",0);									//�ѷ�ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		
//		PL010V13_PrintfString(0		,60	,16	,"�ܹ�����������%4d",SumFQ);				//�ܹ���ҩ��������
		PL010V13_PrintfString(0		,120	,16	,"���ڷ�ҩ��������������������");				//����״̬
	}
	else if(RxNum==4&&RevBuffe[0]==0x82)		//RS485���յ�����
	{
		SumFed[RevBuffe[1]-1]+=RevBuffe[2];
		PL010V13_PrintfString(96		,0	,16	,"%2d",RevBuffe[1]);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		PL010V13_PrintfString(96		,20	,16	,"%2d",0);									//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
		PL010V13_PrintfString(96		,40	,16	,"%2d",RevBuffe[2]);				//�ѷ�ҩ��������ߵ�ʡ�Ժž��ǿɱ����		
		
//		PL010V13_PrintfString(112		,80	,16	,"%4d",SumFed);					//�ܹ��ѷ�ҩ����
		
	}
	else if(RxNum==6&&RevBuffe[0]==0x81)	//��λ��Ϣ0x01--��ȡ,0x81--�ϱ�
	{
		Onlinede=RevBuffe[4];
	}
	else if(RxNum==1&&RevBuffe[0]==0x01)	//��λ��Ϣ0x01--��ȡ,0x81--�ϱ�
	{
		PL010V13_PrintfString(0		,120	,16		,"��ȡ��λ��Ϣ������");				//����״̬
		NumFW=0;
		Onlinede=0;
		memset(SumFed,0x00,8);
		memset(SumFQ,0x00,8);

		PL010V13_PrintfString(96		,0	,16	,"%2d",0);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
		PL010V13_PrintfString(96		,20	,16	,"%2d",0);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	}
	
	for(DelayTime=0;DelayTime<1000;DelayTime++)
	{
		
	}
	if(RxNum&&(RevBuffe[0]==0x82))		//����״̬��ʾ
	{
		switch(RevBuffe[3])
		{
			case 0x00:	PL010V13_PrintfString(0		,120	,16	,"����������������������������");				//����״̬
				break;
			case 0x80:	PL010V13_PrintfString(0		,120	,16	,"ҩƷ����ס������������������");				//����״̬
				break;
			case 0x81:	PL010V13_PrintfString(0		,120	,16	,"ȱҩ������������������������");				//����״̬
				break;
			case 0x82:	PL010V13_PrintfString(0		,120	,16	,"�ȴ���������ʱ������������");				//����״̬
				break;
			case 0xC0:	PL010V13_PrintfString(0		,120	,16	,"��Ԫ�������ͨ���쳣��������");				//����״̬
				break;
			case 0xC1:	PL010V13_PrintfString(0		,120	,16	,"��ҩ�������ͨ���쳣��������");				//����״̬
				break;
		}
//		RevBuffe[0]=0;
//		RevBuffe[1]=0;
//		RevBuffe[2]=0;
//		RevBuffe[3]=0;
	}
#endif
#endif
}
/*******************************************************************************
*������		:	LCD_ShowString
*��������	:	��ʾ�ַ�����ͨ�ֿ�
*����			: x,y:�������
						*p:�ַ�����ʼ��ַ
						��16����
*���			:	��
*����ֵ		:	��
*����			:
*******************************************************************************/
unsigned int PL010V13_PrintfString(u16 x,u16 y,u8 font,const char *format,...)				//��ߵ�ʡ�Ժž��ǿɱ����
{ 
//#ifdef USE_R61509V		
//		va_list ap; 										//VA_LIST ����C�����н����������һ��꣬����ͷ�ļ���#include <stdarg.h>,���ڻ�ȡ��ȷ�������Ĳ���
//		static char string[ 256 ];			//�������飬
//  	va_start( ap, format );
//		vsprintf( string , format, ap );    
//		va_end( ap );
	
//	char	*Char_Buffer=NULL;		//��¼format����
	u16 i=0;		//��ʾ

	//1)**********��ȡ���ݿ���
	u16 num=strlen((const char*)format);		//��ȡ���ݿ���
	//2)**********���建������С����
	unsigned int BufferSize;
	//3)**********argsΪ�����һ��ָ��ɱ�����ı�����va_list�Լ��±�Ҫ�õ���va_start,va_end�������ڶ��壬�ɱ���������б���Ҫ�õ��꣬ ��stdarg.hͷ�ļ��ж���
	va_list args; 
	free(Char_Buffer);						//�ͷŶ�̬�ռ�	
	//4)**********���붯̬�ռ�
//	Char_Buffer = (char*)malloc(sizeof(char) * num);
//	if(Char_Buffer==NULL)
//	{
//		Char_Buffer=NULL;
//		return 0;
//	}
	//5)**********��ʼ��args�ĺ�����ʹ��ָ��ɱ�����ĵ�һ��������format�ǿɱ������ǰһ������
	va_start(args, format);
	//6)**********��������·��������ִ��ĳ���(��ȥ\0),����������ظ�ֵ
	BufferSize = vsprintf(Char_Buffer, format, args);
	num=BufferSize;
	//7)**********�����ɱ�����Ļ�ȡ
	va_end(args);                                      		
	//8)**********���ȷ��ͻ�������С�����ݸ���������������ַ����DMA��������
//	while(*Char_Buffer!='\0')
	for(i=0;i<num;i++)
	{ 
		unsigned char dst=Char_Buffer[i];
//		u8 GTBuffer[512]={0};		//�������ݴ洢�ռ�
		u32 lengh=0;						//���ֵ�������ݳ���		
		if(dst>0x80)		//˫�ֽ�--����
		{
			u16 word=dst<<8;			
//			Char_Buffer++;
			dst=Char_Buffer[i+1];
			word=word|dst;			
			//��ʾ�����ж�
			if(font==16&&x>240-16)
			{
				x=0;
				y+=16;
			}
			if(font==32&&x>240-32)
			{
				x=0;
				y+=32;
			}
			if(font==16&&y>400-16)
			{
				y=x=0;
			}
			if(font==32&&y>400-32)
			{
				y=x=0;
			}
			lengh=GT32L32_ReadBuffer(&GT32L32_Info,font,word,GT32L32_Info.GT32L32_Data.GT32L32_Buffer);		//���ֿ��ж����ݺ���
			//д����Ļ
			ILI9326_ShowChar(x,y,font,lengh,GT32L32_Info.GT32L32_Data.GT32L32_Buffer);
			//��ʾ��ַ����	
			if(font==12)
			{
				x+=12;
			}
			else if(font==16)
			{
				x+=16;
			}
			else if(font==24)
			{
				x+=24;
			}
			else if(font==32)
			{
				x+=32;
			}
//			Char_Buffer++;
			i++;		//˫�ֽڣ�������
		}
		else		//���ֽ�
		{			
			if(x>240-16)
			{
				x=0;
				y+=32;
			}
			if(font==16&&y>400-16)
			{
				y=x=0;
			}
			if(font==32&&y>400-32)
			{
				y=x=0;
			}
			lengh=GT32L32_ReadBuffer(&GT32L32_Info,font,(u16)dst,GT32L32_Info.GT32L32_Data.GT32L32_Buffer);		//���ֿ��ж����ݺ���
//			//д����Ļ
			ILI9326_ShowChar(x,y,font,lengh,GT32L32_Info.GT32L32_Data.GT32L32_Buffer);			
			//��ʾ��ַ����
			if(font==12)
			{
				x+=6;
			}
			else if(font==16)
			{
				x+=8;
			}
			else if(font==24)
			{
				x+=12;
			}
			else if(font==32)
			{
				x+=16;
			}			
//			Char_Buffer++;
//			i++;		//˫�ֽڣ�������
		}
	}
	//9)**********DMA������ɺ�ע��Ӧ���ͷŻ�������free(USART_BUFFER);
//	free(Char_Buffer);		//������ɺ�ע��Ӧ���ͷŻ�������free(Char_Buffer);

	return BufferSize;
//#endif
	return 0;
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
#ifdef USE_R61509V
	if(NumFW)		//��λ��˸	DSPTime
	{
		if(NumFW==1&&(Onlinede>>0&0x01))
		{
			if(DSPTime==500)
			{
				R61509V_Fill(2,180,50,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				R61509V_Fill(2,180,50,200-2,R61509V_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}
		}
		else if(NumFW==2&&(Onlinede>>1&0x01))
		{
			if(DSPTime==500)
			{
				R61509V_Fill(52,180,100,200-2,R61509V_GBLUE);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				R61509V_Fill(52,180,100,200-2,R61509V_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}			
		}
		else if(NumFW==3&&(Onlinede>>2&0x01))
		{
			if(DSPTime==500)
			{
				R61509V_Fill(102,180,150,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				R61509V_Fill(102,180,150,200-2,R61509V_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}			
		}
		else if(NumFW==4&&(Onlinede>>3&0x01))
		{
			if(DSPTime==500)
			{
				R61509V_Fill(152,180,200,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				R61509V_Fill(152,180,200,200-2,R61509V_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}			
		}
		else if(NumFW==5&&(Onlinede>>4&0x01))
		{
			if(DSPTime==500)
			{
				R61509V_Fill(202,180,250,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				R61509V_Fill(202,180,250,200-2,R61509V_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}			
		}
		else if(NumFW==6&&(Onlinede>>5&0x01))
		{
			if(DSPTime==500)
			{
				R61509V_Fill(252,180,300,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				R61509V_Fill(252,180,300,200-2,R61509V_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}			
		}
		else if(NumFW==7&&(Onlinede>>6&0x01))
		{
			if(DSPTime==500)
			{
				R61509V_Fill(302,180,350,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				R61509V_Fill(302,180,350,200-2,R61509V_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}			
		}
		else if(NumFW==8&&(Onlinede>>7&0x01))
		{
			if(DSPTime==500)
			{
				R61509V_Fill(352,180,400-2,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}
			else if(DSPTime==250)
			{
				R61509V_Fill(352,180,400-2,200-2,R61509V_MAGENTA);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
			}			
		}
	}
#endif
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void LCD_WXS(void)		//λ��ʾ
{
#ifdef USE_R61509V
		if((Onlinede>>0&0x01)==0x01)
		{
			R61509V_Fill(2,180,50,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		else
		{
			R61509V_Fill(2,180,50,200-2,R61509V_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>1&0x01)==0x01)
		{
			R61509V_Fill(52,180,100,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		else
		{
			R61509V_Fill(52,180,100,200-2,R61509V_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>2&0x01)==0x01)
		{
			R61509V_Fill(102,180,150,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		else
		{
			R61509V_Fill(102,180,150,200-2,R61509V_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>3&0x01)==0x01)
		{
			R61509V_Fill(152,180,200,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		else
		{
			R61509V_Fill(152,180,200,200-2,R61509V_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>4&0x01)==0x01)
		{
			R61509V_Fill(202,180,250,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		else
		{
			R61509V_Fill(202,180,250,200-2,R61509V_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>5&0x01)==0x01)
		{
			R61509V_Fill(252,180,300,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		else
		{
			R61509V_Fill(252,180,300,200-2,R61509V_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>6&0x01)==0x01)
		{
			R61509V_Fill(302,180,350,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		else
		{
			R61509V_Fill(302,180,350,200-2,R61509V_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		if((Onlinede>>7&0x01)==0x01)
		{
			R61509V_Fill(352,180,400-2,200-2,R61509V_GREEN);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
		else
		{
			R61509V_Fill(352,180,400-2,200-2,R61509V_RED);				//��ָ�����������ָ����ɫ;�����С:(xend-xsta)*(yend-ysta)
		}
#endif
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
	PL010V13_PrintfString(2,200,16	,"%5d",SumFQ[0]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	PL010V13_PrintfString(52,200,16	,"%5d",SumFQ[1]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	PL010V13_PrintfString(102,200,16	,"%5d",SumFQ[2]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	PL010V13_PrintfString(152,200,16	,"%5d",SumFQ[3]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	PL010V13_PrintfString(201,200,16	,"%5d",SumFQ[4]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	PL010V13_PrintfString(251,200,16	,"%5d",SumFQ[5]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	PL010V13_PrintfString(301,200,16	,"%5d",SumFQ[6]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	PL010V13_PrintfString(351,200,16	,"%5d",SumFQ[7]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����

	//�ѷ�������ʾ	
	PL010V13_PrintfString(2,220,16		,"%5d",SumFed[0]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	PL010V13_PrintfString(52,220,16		,"%5d",SumFed[1]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	PL010V13_PrintfString(102,220,16	,"%5d",SumFed[2]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	PL010V13_PrintfString(152,220,16	,"%5d",SumFed[3]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	PL010V13_PrintfString(201,220,16	,"%5d",SumFed[4]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	PL010V13_PrintfString(251,220,16	,"%5d",SumFed[5]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	PL010V13_PrintfString(301,220,16	,"%5d",SumFed[6]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
	PL010V13_PrintfString(351,220,16	,"%5d",SumFed[7]);				//����ҩ��������ߵ�ʡ�Ժž��ǿɱ����
}
#endif