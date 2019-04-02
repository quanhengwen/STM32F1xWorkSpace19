#ifdef FSMCTest

#include "FSMCTest.H"

//#include "string.h"				//�����ڴ��������ͷ�ļ�
//#include "stm32f10x_dma.h"

#include "LCD.H"


#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_ADC.H"
#include "STM32_WDG.H"
#include "STM32_FSMC.H"
#include "STM32_PWM.H"
//#include "STM32_PWM.H"
//#include "STM32_GPIO.H"
#include "STM32_USART.H"
//#include	"image.h"
//#include "STM32_DMA.H"

//#include 	"Image.H"
//#include 	"LinkedList.H"

//#include "TM1618.H"
//#include "MMC_SD.h"
//#include "FatFsAPI.h"			/* �Զ���API�ӿ�*/

//#include "STM32_SDCard.H"
//#include "GT32L32M0180.H"



//#define SDCardTest
//#define GT32L32M0180Test
//��TFT��������SRAM�Ĵ洢����ֻ�ܽ��� BANK1�ϡ���Ӧ����ַ��0x60000000.
//��BANK1���л���Ϊ�ĸ�Ƭѡ,�ֱ��Ӧ����ַ:
//NE1 0x600000000
//NE2 0x640000000
//NE3 0x680000000
//NE4 0x6C0000000
//��������ַ

#define Bank1_LCD_Data ((u32)0x6C100000)

//�Ĵ�������ַ

#define Bank1_LCD_Reg ((u32)0x6C000000)

LCDDef	sLCD;

u16 millisecond=0;
u8 hour=23,min=00,second=30;

//u8 GTBuffer[512]={0};		//�������ݴ洢�ռ�

u32	dspdata=0;
u16 us=0;
u16	mm=0;
u8	ss=0;
u8	hh=0;

u8 bsr  = 0x08;

u16 time=0;
u32 ADCDATA = 0;
//void GT32L32_PinSet(void);
#define FileNum 16
FRESULT result;       //FatFs ���������������
FATFS   FatFsObj[1];  //�߼��������Ĺ�����(�ļ�ϵͳ����)
DIR     dir;          //Ŀ¼����ṹ��
FILINFO fno;          //�ļ���Ϣ�ṹ��
FIL fsrc, fdst;       //�ļ�����
BYTE buffer[4096];    //�ļ�����������
char FilSearchInf[FileNum][13]={0,0};    //�洢���ҵ����ļ�������
unsigned char LenName = 0;

UINT br, bw;          //�ļ���/д�ֽڼ���
u16 xh=0,yv=0;
char Key = 0;
unsigned  short color = 0;
unsigned	short	Rait	=	0;
unsigned	long	ImageAr	=	0;


//=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>
//->������		:	
//->��������	:	 
//->����		:	
//->���		:
//->���� 		:
//<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=
void FSMCTest_Configuration(void)
{	
	SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H	
  Power_Configuration();
  LCD_Configuration();
//  FSMC_Initialize();
//  SSD1963_Init(); 
	
//  SSD1963_Init();  
//  tee:
//  SSD1963_Init();
  LCDCLER(0x5FFF);
//  goto tee;
//  SD_Configuration();
//  USART_DMA_ConfigurationNR	(USART1,115200,128);	//USART_DMA����--��ѯ��ʽ�������ж�
//  TM1618_PinSet();

//  res = f_open(&fsrc, "1:srcfile.dat", FA_OPEN_EXISTING | FA_READ);
//  ADC_TempSensorConfiguration(&ADCDATA);								//STM32�ڲ��¶ȴ���������
//  LCD_ShowBattery(780,2,2,LCD565_GREEN);   //��ʾ12x12���
//  LCD_ShowAntenna(760,2,3,LCD565_GREEN);   //��ʾ12x12����
  
//  LCD_Printf (300,0,32,LCD565_GREEN,"ͼƬ��ʾ����");					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
  
//	SysTick_Configuration(1000);											//ϵͳ���ʱ������72MHz,��λΪuS
	
//	IWDG_Configuration(1000);													//�������Ź�����---������λms
	PWM_OUT(TIM2,PWM_OUTChannel1,2000,900);						//PWM�趨-20161127�汾
	while(1)
	{
		LCDCLER(LCD565_BLACK);SysTick_DeleymS(500);
		LCDCLER(LCD565_WHITE);SysTick_DeleymS(500);	
		LCDCLER(LCD565_BLUE);SysTick_DeleymS(500);
//		LCDCLER(LCD565_BRED);SysTick_DeleymS(500);
//		LCDCLER(LCD565_GRED);SysTick_DeleymS(500);
	}
	
}

//=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>
//->������		:	
//->��������	:	 
//->����		:	
//->���		:
//->���� 		:
//<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=
void FSMCTest_Server(void)
{
//  unsigned  short i = 0;
//  for(i=0;i<0xFFFF;i++)
//  {
//    LCDCLER((u16)(i));
//    i+=89;
//  }
	LCDCLER(LCD565_BLACK);SysTick_DeleymS(1000);
	LCDCLER(LCD565_WHITE);SysTick_DeleymS(1000);	
	LCDCLER(LCD565_BLUE);SysTick_DeleymS(1000);
	LCDCLER(LCD565_BRED);SysTick_DeleymS(1000);
	LCDCLER(LCD565_GRED);SysTick_DeleymS(1000);
	LCDCLER(LCD565_RED);SysTick_DeleymS(1000);
//	NVIC_GenerateCoreReset();
//	LCDCLER(LCD565_GRAY);SysTick_DeleymS(1000);
//	LCDCLER(LCD565_LIGHTGREEN);SysTick_DeleymS(1000);
//	LCDCLER(LCD565_MAGENTA);SysTick_DeleymS(1000);
  
//  Power_Server();
//  for(mm=0;mm<800;mm++)
//  {
//    for(us=0;us<480;us++)
//    {
//      if(dspdata++>=0xFFFF)
//        dspdata=0;
//      LCD_DrawDot(mm,us,dspdata);
//    }
//  }
////	LCD_Server();			//��ʾ�������
//  if(time++>500)
//  {
//    float te  = 0.1;
//    time  = 0;
//    te  = Get_ADC_Temperature(ADCDATA);
//    LCD_Printf (0,350,32,0,"%-0.2f",te);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
//  }

//	TM1618_DIS();
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
void Power_Configuration(void)
{
  unsigned  short  temp  = 0;
  GPIO_Configuration_IPU(GPIOB,	GPIO_Pin_4);			//��GPIO��Ӧ�ܽ�����Ϊ��������ģʽ----V20170605
  GPIO_Configuration_OPP50(GPIOF,GPIO_Pin_10);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
  restart:
  PF10 = 1;
	SysTick_DeleymS(1000);					//SysTick��ʱnS
//  while(0  ==  PB4in)
//  {
//    SysTick_DeleymS(1);					//SysTick��ʱnS
//    temp++;
//    if(temp>3000)
//    {
//      Key = 0;
//      
//      break;
//    }
//    
//  }
//  if(temp<3000)
//  {
//    PF10  = 1;
//    temp  = 0;
//    goto restart;
//  }
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
void Power_Server(void)
{
  unsigned  short  temp  = 0;

  if((0  ==  PB4in)&&(1==Key))
  {
    while(0  ==  PB4in)
    {
      SysTick_DeleymS(1);					//SysTick��ʱnS
      temp++;
      if(temp>3000)
      {
        Key = 0;
        PF10  = 0;
        PWM_OUT(TIM2,PWM_OUTChannel1,1000,110);						//PWM�趨-20161127�汾
        while(0  ==  PB4in);
        NVIC_GenerateCoreReset();
      }
    }
  }
  else if(1 ==PB4in)
  {
    Key = 1;
  }
  
  if(temp<3000)
  {
    PF10  = 1;
//    while(1);
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
void LCD_Server(void)			//��ʾ�������
{

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
//	SPIDef	*SPI	=	&sLCD.GT32L32.SPI;
//	sLCD.Port.sBL_PORT				=	GPIOA;
//	sLCD.Port.sBL_Pin					=	GPIO_Pin_0;
//	
//	sLCD.Port.sRD_PORT				=	GPIOD;
//	sLCD.Port.sRD_Pin					=	GPIO_Pin_4;	
//	
//	sLCD.Port.sREST_PORT			=	GPIOA;
//	sLCD.Port.sREST_Pin				=	GPIO_Pin_0;
//	
//	sLCD.Port.sDC_PORT				=	GPIOE;
//	sLCD.Port.sDC_Pin					=	GPIO_Pin_3;
//	
//	sLCD.Port.sWR_PORT				=	GPIOD;
//	sLCD.Port.sWR_Pin					=	GPIO_Pin_5;	
//	
//	sLCD.Port.sTE_PORT				=	GPIOC;
//	sLCD.Port.sTE_Pin					=	GPIO_Pin_5;
//	
//	sLCD.Port.sCS_PORT				=	GPIOG;
//	sLCD.Port.sCS_Pin					=	GPIO_Pin_12;
//	
//	sLCD.Port.sDATABUS_PORT		=	GPIOE;
//	sLCD.Port.sDATABUS_Pin		=	GPIO_Pin_All;	
//	
//	sLCD.Flag.Rotate	=	Draw_Rotate_180D;		//ʹ����ת�Ƕ�
//  sLCD.Data.BColor  = LCD565_BLACK;
//  sLCD.Data.PColor  = LCD565_WHITE;
//	
//	SPI->Port.SPIx		=	SPI1;
//	SPI->Port.CS_PORT	=	GPIOB;
//	SPI->Port.CS_Pin	=	GPIO_Pin_14;
//	SPI->Port.SPI_BaudRatePrescaler_x=SPI_BaudRatePrescaler_128;
//	
//	LCD_Initialize(&sLCD);

  //=======================LCD�˿�
	LCDPortDef	*LcdPort	=	&sLCD.Port;
  
//  GPIO_InitTypeDef GPIO_InitStructure;
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC |RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO, ENABLE);
//  //�������
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;

//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//  
//  //LCD��λ

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;

//  GPIO_Init(GPIOE, &GPIO_InitStructure);
//  //��FSMC�����ݶ˿�D[15:0]

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;

//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

//  GPIO_Init(GPIOD, &GPIO_InitStructure);
//  

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;

//  GPIO_Init(GPIOE, &GPIO_InitStructure);

//  //��FSMC���ܶ˿ڣ�PD.4=RD(nOE)��PD.5=WR(nWE)

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;

//  GPIO_Init(GPIOD, &GPIO_InitStructure);

//  //��NE4����

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;

//  GPIO_Init(GPIOG, &GPIO_InitStructure);

//  //��RS����

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;

//  GPIO_Init(GPIOE, &GPIO_InitStructure);

//  //NE4=1

//  GPIO_SetBits(GPIOG, GPIO_Pin_12);

//  //LCD_RESET=0

//  GPIO_ResetBits(GPIOE, GPIO_Pin_1);

//  //LCD_RD=1(nOE)

//  GPIO_SetBits(GPIOD, GPIO_Pin_4);

//  //LCD_WR=1(nWE)

//  GPIO_SetBits(GPIOD, GPIO_Pin_5);

//  //����LIGHT=1

////  GPIO_SetBits(GPIOA, GPIO_Pin_0);
//  
//  //NE4=0
//  GPIO_ResetBits(GPIOG, GPIO_Pin_12);
//  SysTick_DeleymS(100);
  



  LcdPort->sBL_PORT   = GPIOA;
  LcdPort->sBL_Pin    = GPIO_Pin_0;
  
  LcdPort->sCS_PORT   = GPIOG;
  LcdPort->sCS_Pin    = GPIO_Pin_12;
  
  LcdPort->sREST_PORT = NULL;
  LcdPort->sREST_Pin  = 0;
  
  sLCD.Data.FsmcRegAddr   = (unsigned long)0x6C000000;
  sLCD.Data.FsmcDataAddr  = (unsigned long)0x6C100000;
  
  LCDFsmc_Initialize(&sLCD);
}

//LCDд�Ĵ�����ַ����

void LCD_WriteAddr1(u16 index)
{
  *(vu16*)(Bank1_LCD_Reg) = index;
}
//LCDд���ݺ���

void LCD_WriteData1(u16 val)
{
  *(vu16*)(Bank1_LCD_Data) = val; 
}

//LCDд�Ĵ�����������Ƚ������ַд��Reg�У�Ȼ���ٽ��������ֵд��Data��

//�����ַ�����ò���ILI9325��Datasheet

void LCD_WR_CMD(u16 index, u16 val)
{

  *(vu16 *)(Bank1_LCD_Reg) = index;
  *(vu16 *)(Bank1_LCD_Data) = val;
}
/*******************************************************************************
*������		:	Lcd_Init
*��������	:	STM32�ڲ��¶ȴ���������
*����			: 
*���			:	��
*����ֵ		:	��
*����			:
*******************************************************************************/
void SSD1963_Init(void)
{

	u16 time=2000;
	u16	temp=time;
//����һ����Щ��������ñ����ʱ����ʾ����
	//1������������������������λ
//	SSD1963_BACKLIGHT_OFF;	//�ر���
//	LCD_REST();
//	LCD_RST_LOW;
//	LCD_RST_HIGH;
//	while(temp--);			//�ȴ���������
	//2������������������������λ
//	LCD_CS_HIGH;			//��ȡ��Ƭѡ
//	LCD_RD_HIGH;
//	LCD_WR_LOW;				//���߹���Ϊд����
//	LCD_CS_LOW;  		//ʹ��
  SysTick_DeleymS(100);
	//3��������������������������ϵͳʱ��  ����Ƶ�� 10MHz  250MHz < VCO < 800MHz
	LCD_WriteAddr1(0x00E2);						  //PLL multiplier, set PLL clock to 120M Start the PLL. Before the start, the system was operated with the crystal oscillator or clock input
	LCD_WriteData1(0x0023);	    				//���ñ�Ƶ N=0x36 for 6.5M, 0x23 for 10M crystal
	LCD_WriteData1(0x0001);							//���÷�Ƶ
	LCD_WriteData1(0x0004);							//�������
	//4����������������������ʹ��PLL
	LCD_WriteAddr1(0x00E0);  					//PLL enable
	LCD_WriteData1(0x0001);
	
	LCD_WriteAddr1(0x00E0);
	LCD_WriteData1(0x0003);
  SysTick_DeleymS(50);
	//5����������������������������λ
	LCD_WriteAddr1(0x0001);  					//software reset
	//6��������������������������ɨ��Ƶ��
	LCD_WriteAddr1(0x00E6);						//PLL setting for PCLK, depends on resolution
	LCD_WriteData1(0x0003);
	LCD_WriteData1(0x00FF);
	LCD_WriteData1(0x00FF);
	//7��������������������������LCD���ģʽ Set the LCD panel mode (RGB TFT or TTL)
	LCD_WriteAddr1(0x00B0);						//LCD SPECIFICATION
	LCD_WriteData1(0x0000);
	LCD_WriteData1(0x0000);
	LCD_WriteData1((799>>8)&0X00FF);  		//����ˮƽ���ص������8λ		Set HDP 
	LCD_WriteData1(799&0X00FF);					//����ˮƽ���ص������8λ
	LCD_WriteData1((479>>8)&0X00FF);  		//���ô�ֱ���ص������8λ		Set VDP
	LCD_WriteData1(479&0X00FF);					//���ô�ֱ���ص������8λ
	LCD_WriteData1(0x0000);									//������ż��RGB˳��Ĭ��0��Even line RGB sequence&Odd line RGB sequence
	//8��������������������������ˮƽ�� Set Horizontal Period
	LCD_WriteAddr1(0x00B4);							//HSYNC
	LCD_WriteData1((928>>8)&0X00FF);  	//High byte of horizontal total period
	LCD_WriteData1(928&0X00FF);					//Low byte of the horizontal total period (display + non-display) in pixel clock (POR = 10101111)
																			//Horizontal total period = (HT + 1) pixels
	LCD_WriteData1((46>>8)&0X00FF);  	//High byte of the non-display period between the start of the horizontal sync (LLINE) signal and the first
																			//display data. (POR = 000)
	LCD_WriteData1(46&0X00FF);
	LCD_WriteData1(48);			   //Set HPW
	LCD_WriteData1((15>>8)&0X00FF);  //Set HPS
	LCD_WriteData1(15&0X00FF);
	LCD_WriteData1(0x0000);
	//9�������������������������ô�ֱ��	Set Vertical Period
	LCD_WriteAddr1(0x00B6);							//VSYNC
	LCD_WriteData1((525>>8)&0X00FF);   //Set VT
	LCD_WriteData1(525&0X00FF);
	LCD_WriteData1((16>>8)&0X00FF);  //Set VPS
	LCD_WriteData1(16&0X00FF);
	LCD_WriteData1(16);			   //Set VPW
	LCD_WriteData1((8>>8)&0X00FF);  //Set FPS
	LCD_WriteData1(8&0X00FF);	
	//10��������������������������GPIO
	LCD_WriteAddr1(0x00B8);
	LCD_WriteData1(0x0007);    //GPIO3=input, GPIO[2:0]=output //���ģʽ
	LCD_WriteData1(0x0001);   	//0 GPIO0 is used to control the panel power with Enter Sleep Mode 0x10 or Exit Sleep Mode 0x11.
													//1 GPIO0 is used as normal GPIO
	//11��������������������������GPIO������ɨ�跽�� Set GPIO value for GPIO configured as output
	LCD_WriteAddr1(0x00BA);
	LCD_WriteData1((0X05&0XFF)|(0X04&0XFF));    //GPIO[3:0] out 1
	//12�������������������������õ�ַģʽ	Set Address Mode
	LCD_WriteAddr1(0x0036); //rotation
	LCD_WriteData1(0x0000);
	//13�����������������������������ݽӿ� Set Pixel Data Interface/Pixel Data Interface Format
	LCD_WriteAddr1(0x00F0); //pixel data interface
	LCD_WriteData1(0x0003);
	
	//14�������������������������ô�ֱ��
	LCD_WriteAddr1(0x0029); //display on
	//15�������������������������ô�ֱ��
	LCD_WriteAddr1(0x00d0); 
	LCD_WriteData1(0x000D);
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
void LCDCLER(unsigned short color)
{
  unsigned  short i = 0,j=0;
  unsigned  short xs = 0,xe =800;
  unsigned  short ys = 0,ye =480;
  //======================================��������
	LCD_WriteAddr1(0x002A);			//�����е�ַ
	LCD_WriteData1(xs>>8);		//��ʼ��ַ��8λ
	LCD_WriteData1(xs);			//��ʼ��ַ��8λ
	LCD_WriteData1(xe>>8);		//������ַ��8λ
	LCD_WriteData1(xe);			//�н�����ַ��8λ
	
	LCD_WriteAddr1(0x002b);			//����ҳ��ַ	
	LCD_WriteData1(ys>>8);
	LCD_WriteData1(ys);
	LCD_WriteData1(ye>>8);
	LCD_WriteData1(ye);
	
	LCD_WriteAddr1(0x0036);			//����ҳ��ַ
	LCD_WriteData1(0XA0);       //��ʾģʽ

	LCD_WriteAddr1(0x002c);			//д�ڴ���ʼ��ַ 
  for(i=0;i<=(xe-xs);i++)
  {
    for(j=0;j<=(ye-ys);j++)
      LCD_WriteData1(color);
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
void ImageDisp(unsigned char* buffer,unsigned long len)
{
	unsigned  long i = 0;
  unsigned  short xs = 0,xe =800;
  unsigned  short ys = 0,ye =480;
	unsigned	short	P	=	0;
  //======================================��������
	LCD_WriteAddr1(0x002A);			//�����е�ַ
	LCD_WriteData1(xs>>8);		//��ʼ��ַ��8λ
	LCD_WriteData1(xs);			//��ʼ��ַ��8λ
	LCD_WriteData1(xe>>8);		//������ַ��8λ
	LCD_WriteData1(xe);			//�н�����ַ��8λ
	
	LCD_WriteAddr1(0x002b);			//����ҳ��ַ	
	LCD_WriteData1(ys>>8);
	LCD_WriteData1(ys);
	LCD_WriteData1(ye>>8);
	LCD_WriteData1(ye);
	
	LCD_WriteAddr1(0x0036);			//����ҳ��ַ
	LCD_WriteData1(0XA0);       //��ʾģʽ

	LCD_WriteAddr1(0x002c);			//д�ڴ���ʼ��ַ 
  for(i=0;i<=len;)
  {
		P=buffer[i];
		P=P<<8;
		P|=buffer[i+1];
      LCD_WriteData1(P);
			i+=2;
  }
}

#endif