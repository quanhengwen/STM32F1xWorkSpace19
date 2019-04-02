/******************************** User_library *********************************
* �ļ��� 	: SWDTEST.C
* ����   	: wegam@sina.com
* �汾   	: V
* ����   	: 2017/04/16
* ˵��   	: 
********************************************************************************
SPI_FLASHʹ�ù��ܲ���
1����Ҫ�궨�� SPI_FLASH
2��ʹ��USB_TEST �����
3����Ҫ�궨��SPI����

*******************************************************************************/
#ifdef SWDTEST							//���������A3987_TEST �˹�����Ч

#include "STM32F10x_BitBand.H"
#include "stm32f10x_nvic.h"


#include	"stdio.h"				//����printf
#include	"string.h"			//����printf
#include	"stdarg.h"			//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"			//malloc��̬�����ڴ�ռ�
	
#include	"stddef.h"
#include	"stdint.h"



#include "SWDTEST.H"

#include "swd.h"

#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"

unsigned short Time	=	0;
SWJRequestDef	SWK;
unsigned char	*p	=	NULL;
unsigned char	Test	=	0;
unsigned long	*pCSW	=	0;

MEMAPCSWDef	CSW;
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void SWDTEST_Configuration(void)
{
	SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H
//	
//	GPIO_DeInitAll();							//�����е�GPIO�ر�----V20170605
//	
	SysTick_Configuration(1000);	//ϵͳ���ʱ������72MHz,��λΪuS
//	
	GPIO_Configuration_OPP50	(SWDIO_PORT,SWDIO_PIN);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(SWCLK_PORT,SWCLK_PIN);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
//	
	SW_PinInit();
//	
//	SWJ_InitDebug();
	SWK.Start	=	1;
	SWK.APnDP	=	1;
	p	=	(unsigned char*)&SWK;
	Test	=	*p;
	
	CSW.Size	=	2;
	CSW.DbgSwEnable	=	1;
	pCSW=(unsigned long*)&CSW;
	
	SWD_TransRequest(0xFF);			//������ʼ����

}

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void SWDTEST_Server(void)		//CRC--TEST
{	
  unsigned char i=0;
  unsigned long temp=0;
	if(Time++>1000)
	{
		Time	=	0;
		SWJ_InitDebug();
	}
}






























#endif