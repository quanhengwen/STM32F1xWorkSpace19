#ifdef MmcSDTest

#include "MmcSDTest.H"

//#include "string.h"				//�����ڴ��������ͷ�ļ�
//#include "stm32f10x_dma.h"

#include "LCD.H"


#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_ADC.H"
#include "STM32_WDG.H"
//#include "STM32_PWM.H"
//#include "STM32_PWM.H"
//#include "STM32_GPIO.H"
#include "STM32_USART.H"
//#include "STM32_DMA.H"

#include 	"Image.H"
#include 	"LinkedList.H"

#include "TM1618.H"
//#include "MMC_SD.h"
#include "FatFsAPI.h"			/* �Զ���API�ӿ�*/

//#include "STM32_SDCard.H"
//#include "GT32L32M0180.H"



//#define SDCardTest
//#define GT32L32M0180Test	
//
BMPInftDef BmpInf;
LINK_NODE Node;

LCDDef	sLCD;
SPIDef	sSCport;

//GT32L32_Info_TypeDef 	GT32L32_Init;
//GT32L32Def			GT32L32;
//SSD1963_Pindef 	SSD1963_Pinfo;
TM1618_Pindef		TM1618_1,TM1618_2;

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
//=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>
//->������		:	
//->��������	:	 
//->����		:	
//->���		:
//->���� 		:
//<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=
void MmcSDTest_Configuration(void)
{	
	SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H	
	GPIO_DeInitAll();							//�����е�GPIO�ر�----V20170605	
  LCD_Configuration();
  SD_Configuration();
  USART_DMA_ConfigurationNR	(USART1,115200,128);	//USART_DMA����--��ѯ��ʽ�������ж�
//  TM1618_PinSet();

//  res = f_open(&fsrc, "1:srcfile.dat", FA_OPEN_EXISTING | FA_READ);
  ADC_TempSensorConfiguration(&ADCDATA);								//STM32�ڲ��¶ȴ���������
  LCD_ShowBattery(780,2,2,LCD565_GREEN);   //��ʾ12x12���
  LCD_ShowAntenna(760,2,3,LCD565_GREEN);   //��ʾ12x12����
  
  LCD_Printf (300,0,32,LCD565_GREEN,"ͼƬ��ʾ����");					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
  
	SysTick_Configuration(1000);											//ϵͳ���ʱ������72MHz,��λΪuS
	
	IWDG_Configuration(1000);													//�������Ź�����---������λms
	
	PWM_OUT(TIM2,PWM_OUTChannel1,1,10);						//PWM�趨-20161127�汾
	
}
//=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>
//->������		:	
//->��������	:	 
//->����		:	
//->���		:
//->���� 		:
//<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=
void MmcSDTest_Server(void)
{
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
unsigned short Clllb(unsigned char r,unsigned char g,unsigned char b)
{
  unsigned short temp = 0;
  r>>=3;      //ȡ��5λ
  temp+=r;
  temp<<=5;
  g>>=3;      //ȡ��5λ
  temp+=g;
  temp<<=6;
  b>>=3;      //ȡ��5λ
  temp+=b;
  return temp;
}
unsigned short Clll(unsigned char r,unsigned char g)
{
  unsigned short temp = 0;
  temp=r;
  temp<<=8;
  temp+=g;
  return temp;
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
void SD_Configuration(void)
{
  u16 da=0;
  u16 color;
  
  
  
  LCD_Printf (0,0,16,LCD565_GREEN,"STM32F1xWorkSpace--(MmcSDTest)SD����ȡ");					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
  LCD_Printf (0,16,16,LCD565_GREEN,"Ϊ������ע�Ṥ����......");					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
  //========================Ϊ�߼�������ע�Ṥ����
  result = f_mount(&FatFsObj[0],"0:",1);    //��FatFsģ����ע��/ע��һ��������(�ļ�ϵͳ����)
  if(0  ==  result)
  {
    u32 sd_size;
    u32 free;
    LCD_Printf (0,32,16,LCD565_GREEN,"������ע��ɹ�");					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
    //========================�õ�������Ϣ��������������ʣ������
    result = SD_disk_getcapacity("0",&sd_size,&free);
    if(0  ==  result)
    {
      LCD_Printf (0,48,16,LCD565_GREEN,"SD������%dKByte,%dMB,%0.5fGB",sd_size,sd_size>>10,(double)(sd_size>>10)/1024);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
      //========================��ȡʣ������    
      LCD_Printf (0,64,16,LCD565_GREEN,"SD��ʣ������%dKByte,%dMB,%0.5fGB",free,free>>10,(double)(free>>10)/1024);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
      free  = sd_size-free;
      LCD_Printf (0,80,16,LCD565_GREEN,"��������%dKByte,%dMB,%0.5fGB",free,free>>10,(double)(free>>10)/1024);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
    }
    else
    {
      LCD_Printf (0,48,16,LCD565_GREEN,"������Ϣ��ѯʧ��");					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
    }
  }
  else
  {
    LCD_Printf (0,32,16,LCD565_GREEN,"������ע��ʧ��");					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
  }
  //========================����ָ���ļ�
  LCD_Printf (0,96,16,LCD565_GREEN,"����ָ���ļ�:bmp");					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
  result  = FilSearch(&FatFsObj[0],&dir,"0:/","bmp",FilSearchInf);  //��ָ��·���²���ָ����չ�����ļ�������¼��(*p)[13]�����У�ע������¼����
  if(0 != result)
  {
    unsigned short    i=0;
    unsigned short    j = 0;
    unsigned short    flg=0;
    LCD_Printf (0,112,16,LCD565_GREEN,"���ҵ�bmp�ļ�");					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
    for(i=0;i<FileNum;i++)
    {
      if(0  !=  FilSearchInf[i][0])
      {
        unsigned char k = 0;
        for(k=0;k<13;++k)
        {
          if(0  ==  FilSearchInf[i][k])
            break;
        }
        //===============��ӡ���ҵ�bmp�ļ���
        LCD_Show(0,128+i*16,16,LCD565_GREEN,k,FilSearchInf[i]);
      }
    }
    while(1)
    {
//      i=0;
//      j=0;
      //===============================ͼƬ1
      for(i=0;i<FileNum;i++)
      {
        if(FilSearchInf[i][0] !=0)
        {
          yv  = 0;
          flg = 0;
          result  = f_open(&fsrc,&FilSearchInf[i][0],FA_READ);
//          if(result)
//            break;
          for(j=0;j<480;j++)
          {
            if(flg==0)
            {
              result  = f_read(&fsrc,buffer,54,&br);
            }
            else
            {
              result  = f_read(&fsrc,buffer,2400,&br);
            }
            if(FR_OK  !=  result)
            {
    //          return;
            }
            da  = 0;
            if(flg==0)
            {
              flg = 1;
              da  = 0;
              result  = f_read(&fsrc,buffer,2400,&br);
            }

            LCD_ShowBMP(0,yv,800,yv,br,buffer);    //��ʾʮ����������
            yv+=1;
            if(yv>=480)
              yv  = 0;
          }
//          SysTick_DeleyS(3);					//SysTick��ʱnS
        }
      }           
    }    
  }
  else
  {    
    LCD_Printf (0,112,16,LCD565_GREEN,"δ�ҵ�bmp�ļ�");					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����    
  }
//  f_getfree();  

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
	SPIDef	*SPI	=	&sLCD.GT32L32.SPI;
	sLCD.Port.sBL_PORT				=	GPIOC;
	sLCD.Port.sBL_Pin					=	GPIO_Pin_0;
	
	sLCD.Port.sRD_PORT				=	GPIOC;
	sLCD.Port.sRD_Pin					=	GPIO_Pin_1;	
	
	sLCD.Port.sREST_PORT			=	GPIOC;
	sLCD.Port.sREST_Pin				=	GPIO_Pin_2;
	
	sLCD.Port.sDC_PORT				=	GPIOC;
	sLCD.Port.sDC_Pin					=	GPIO_Pin_3;
	
	sLCD.Port.sWR_PORT				=	GPIOC;
	sLCD.Port.sWR_Pin					=	GPIO_Pin_4;	
	
	sLCD.Port.sTE_PORT				=	GPIOC;
	sLCD.Port.sTE_Pin					=	GPIO_Pin_5;
	
	sLCD.Port.sCS_PORT				=	GPIOC;
	sLCD.Port.sCS_Pin					=	GPIO_Pin_5;
	
	sLCD.Port.sDATABUS_PORT		=	GPIOE;
	sLCD.Port.sDATABUS_Pin		=	GPIO_Pin_All;	
	
	sLCD.Flag.Rotate	=	Draw_Rotate_180D;		//ʹ����ת�Ƕ�
  sLCD.Data.BColor  = LCD565_BLACK;
  sLCD.Data.PColor  = LCD565_WHITE;
	
	SPI->Port.SPIx		=	SPI1;
	SPI->Port.CS_PORT	=	GPIOB;
	SPI->Port.CS_Pin	=	GPIO_Pin_14;
	SPI->Port.SPI_BaudRatePrescaler_x=SPI_BaudRatePrescaler_128;
	
	LCD_Initialize(&sLCD);
}

/*******************************************************************************
*������			:	function
*��������		:	��������˵��
*����				: 
*����ֵ			:	��
*******************************************************************************/
void TM1618_PinSet(void)
{
	TM1618_1.TM1618_CLK_PORT	=GPIOC;
	TM1618_1.TM1618_CLK_Pin		=GPIO_Pin_8;
	
	TM1618_1.TM1618_DIO_PORT	=GPIOC;
	TM1618_1.TM1618_DIO_Pin		=GPIO_Pin_9;	
	
	TM1618_1.TM1618_STB_PORT	=GPIOC;
	TM1618_1.TM1618_STB_Pin		=GPIO_Pin_10;
	
	TM1618_2.TM1618_CLK_PORT	=GPIOC;
	TM1618_2.TM1618_CLK_Pin		=GPIO_Pin_8;
	
	TM1618_2.TM1618_DIO_PORT	=GPIOC;
	TM1618_2.TM1618_DIO_Pin		=GPIO_Pin_9;	
	
	TM1618_2.TM1618_STB_PORT	=GPIOC;
	TM1618_2.TM1618_STB_Pin		=GPIO_Pin_11;
	
	TM1618_PinConf(&TM1618_1);
	TM1618_PinConf(&TM1618_2);
	
	GPIO_Configuration_OPP50	(GPIOC,	GPIO_Pin_12);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	
	PC12=1;
}

/*******************************************************************************
*������			:	function
*��������		:	��������˵��
*����				: 
*����ֵ			:	��
*******************************************************************************/
void TM1618_DIS(void)
{
	if(us++>=1000)
	{
		us=0;
		mm++;
		if(mm>=60)
		{
			mm=0;
			ss++;
		}
		if(ss>=60)
		{
			ss=0;
			hh++;
		}
		if(hh>=24)
		{
			hh=0;
		}
		
		dspdata=(u32)mm+(u32)ss*100+(u32)hh*10000;
		TM1618_WriteDataN(&TM1618_1,dspdata/10000);
		TM1618_WriteDataN(&TM1618_2,dspdata%10000);	
	}
}


#endif