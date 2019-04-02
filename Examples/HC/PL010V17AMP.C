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

#ifdef PL010V17AMP				//����ҩ�ܳ���LCD��---������

#include "PL010V17AMP.H"

#include	"AMP_PHY.H"

#include "LCD.h"


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
#include "stdlib.h"				//�����ڴ��������ͷ�ļ�
#include "stm32f10x_dma.h"




#define	Master	1		//0--�ӻ���1-����
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

double	WenDubac	=	0.0;

//R61509VDef R61509V;
u8	DspFlg	=	0;

u16 BKlight	=	0;

u16 line	=	0;
u8 lineT	=	0;




u32	WeigthARR[100]	=	{0};
u8	DspFlg1	=	0;
u32	We1	=	0;
u8	NuW	=	0;
u32	BsW	=	0;	//����
u32 SsW	=	0;	//����
u8	St	=	0;	//����
u8	SN	=	0;	//����
u8	SNb	=	0;	//��������
u32 TempData	=	0;		//�¶�
u32 TempDataB	=	0;		//�¶�

u16 weighVar[20]={0};
u8	VarN	=	0;

u32 Rand	=	0;
//============LCD�������Ա���
u16 HX	=	0;
u16	HY	=	0;
unsigned short	Color	=	0;
u8 ADCc	=	0;
u32 READ_GAIN	=	0;
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PL010V17AMP_Configuration(void)
{
	SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H
	
	GPIO_DeInitAll();							//�����е�GPIO�ر�----V20170605	
	
	SwitchID_Configuration();
	
	RS485_Configuration();
	
	
//	USART_Configuration();
	
	SysTick_DeleymS(500);				//SysTick��ʱnmS
	
	LCD_Configuration();
	

	
	
	
	SysTick_Configuration(1000);	//ϵͳ���ʱ������72MHz,��λΪuS
	
//	IWDG_Configuration(500);			//�������Ź�����---������λms	

//	PWM_OUT(TIM2,PWM_OUTChannel1,1,900);	//PWM�趨-20161127�汾--����ָʾ��

	
	
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PL010V17AMP_Server(void)
{	
	IWDG_Feed();				//�������Ź�ι��
	RS485_Server();		  //ͨѶ����---������Ϣ�Ľ����뷢��
  if(millisecond++==1000)
  {
    millisecond=1001;
    LCD_Clean(LCD565_LBBLUE);	//�����Ļ����
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
	RxNum=RS485_ReadBufferIDLE(&RS485,RxdBuffe);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(RxNum)
	{
    
    unsigned char* head = NULL;
    memcpy(RevBuffe,RxdBuffe,RxNum);
    head = getheadaddr(RevBuffe,RxNum);
    if(head)
    {
      LCD_ShowHex(0	,128,16	,LCD565_RED,RxNum,8,head);				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����	
    }
    else
    {
      LCD_Printf(0,0,16	,LCD565_RED,"δʶ��ָ��");				//����ҩ��λ����ߵ�ʡ�Ժž��ǿɱ����
    }
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
	
	Port->sCS_PORT		=	GPIOA;
	Port->sCS_Pin			=	GPIO_Pin_12;
	
	Port->sDC_PORT		=	GPIOA;
	Port->sDC_Pin			=	GPIO_Pin_8;
	
	Port->sWR_PORT		=	GPIOA;
	Port->sWR_Pin			=	GPIO_Pin_15;
	
	Port->sRD_PORT		=	GPIOC;
	Port->sRD_Pin			=	GPIO_Pin_5;
	
	Port->sREST_PORT	=	GPIOD;
	Port->sREST_Pin		=	GPIO_Pin_2;
	
	Port->sBL_PORT		=	GPIOA;
	Port->sBL_Pin			=	GPIO_Pin_3;
	
	Port->sTE_PORT		=	GPIOC;
	Port->sTE_Pin			=	GPIO_Pin_4;
	
	Port->sDATABUS_PORT	=	GPIOB;
	Port->sDATABUS_Pin	=	GPIO_Pin_All;
	
	sLCD.Data.BColor	=	LCD565_LBBLUE;
	sLCD.Data.PColor	=	LCD565_RED;
	sLCD.Flag.Rotate	=	Draw_Rotate_90D;
	
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
	RS485.USARTx=USART1;
	RS485.RS485_CTL_PORT=GPIOA;
	RS485.RS485_CTL_Pin=GPIO_Pin_11;
	
	RS485_DMA_ConfigurationNR	(&RS485,115200,Rs485Size);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
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