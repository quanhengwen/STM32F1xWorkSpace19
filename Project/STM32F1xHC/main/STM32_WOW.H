#ifndef __STM32_WOW_H
#define __STM32_WOW_H

/* Includes ------------------------------------------------------------------*/
//#include "stm32f10x_map.h"
//#include "STM32_WDG.H"
//#include "STM32_SYSTICK.H"
//#include "STM32_SYS.H"

//#include "STM32_SYS.H"
//#include "stm32f10x_gpio.h"
//#include "STM32_WDG.H"
//#include "STM32_TIM.H"
//#include "STM32_SYSLED.h"
//#include "STM32_EXTI.H"
//#include "STM32_NVIC.H"

//#include "STM32F10x_BitBand.H"
//#define STM32_SYS


//***********************************���ֵ�����***********************************//
#ifdef	DEMO
	#include "DEMO.H"
#endif

//***********************************HCProTest***********************************//
#ifdef	HCProTest
	#include "HCProTest.H"
#endif



//*********************************PC001V10*********************************//
#ifdef PC001V10
	#include "PC001V10.H"
#endif

//*********************************PC001V21*********************************//
#ifdef PC001V21
	#include "PC001V21.H"
#endif

//*********************************PC001V25*********************************//
#ifdef PC001V25
	#include "PC001V25.H"
#endif
//*********************************PC001V30HC*********************************//
#ifdef PC001V30HC
	#include "PC001V30HC.H"
#endif
//*********************************PC004V21*********************************//
#ifdef PC004V21
	#include "PC004V21.H"
#endif
//*********************************PC004V21HC*********************************//
#ifdef PC004V21HC
	#include "PC004V21HC.H"
#endif
//*********************************PC004V31HC*********************************//
#ifdef PC004V31HC
	#include "PC004V31HC.H"
#endif


//*********************************PC006V20*********************************//
#ifdef PC006V20
	#include "PC006V20.H"
#endif

//*********************************PC006V21*********************************//
#ifdef PC006V21
	#include "PC006V21.H"
#endif
//*********************************PC006V22*********************************//
#ifdef PC006V22
	#include "PC006V22.H"
#endif
//*********************************PC012V13*********************************//
#ifdef PC012V13
	#include "PC012V13.H"
#endif

//*********************************PC016V20*********************************//
#ifdef PC016V20
	#include "PC016V20.H"
#endif

//*********************************PC016V20Com*********************************//
#ifdef PC016V20Com
	#include "PC016V20Com.H"
#endif
//*********************************PC018V10*********************************//
#ifdef PC018V10
	#include "PC018V10.H"
#endif

//*********************************PD002V30*********************************//
#ifdef PD002V30
	#include "PD002V30.H"
#endif
//*********************************PD002V30ADC*********************************//
#ifdef PD002V30ADC
	#include "PD002V30ADC.H"
#endif

//*********************************PD003V11*********************************//
#ifdef PD003V11
	#include "PD003V11.H"
#endif
//*********************************PD003VG*********************************//
#ifdef PD003VG
	#include "PD003VG.H"
#endif
//*********************************PD014V14*********************************//
#ifdef PD014V14
	#include "PD014V14.H"
#endif
//*********************************PD014V15*********************************//
#ifdef PD014V15
	#include "PD014V15.H"
#endif

//*********************************PD004V10*********************************//
#ifdef PD004V10
	#include "PD004V10.H"
#endif

//*********************************PD008V11*********************************//
#ifdef PD008V11
	#include "PD008V11.H"
#endif
//*********************************PD010V20*********************************//
#ifdef PD010V20
	#include "PD010V20.H"
#endif
//*********************************PD010V20CAN*********************************//
#ifdef PD010V20CAN
	#include "PD010V20CAN.H"
#endif

//*********************************PD011V10*********************************//
#ifdef PD011V10
	#include "PD011V10.H"
#endif

//*********************************PD011V10*********************************//
#ifdef PD011V20
	#include "PD011V20.H"
#endif

//*********************************PD006V20*********************************//
#ifdef PD006V20
	#include "PD006V20.H"
#endif

//*********************************PL001V20*********************************//
#ifdef PL001V20
	#include "PL001V20.H"
#endif

//*********************************PL006V10*********************************//
#ifdef PL006V10
	#include "PL006V10.H"
#endif

//*********************************PL009V31*********************************//
#ifdef PL009V31
	#include "PL009V31.H"
#endif
//*********************************PL009V31HC*********************************//
#ifdef PL009V31HC
	#include "PL009V31HC.H"
#endif

//*********************************PL009V33*********************************//
#ifdef PL009V33
	#include "PL009V33.H"
#endif

//*********************************PL009V33HC*********************************//
#ifdef PL009V33HC
	#include "PL009V33HC.H"
#endif
//*********************************PL010V13*********************************//
#ifdef PL010V13
	#include "PL010V13.H"
#endif
//*********************************PL010V15*********************************//
#ifdef PL010V15
	#include "PL010V15.H"
#endif

//*********************************PL010V15TEST*********************************//
#ifdef PL010V15TEST
	#include "PL010V15TEST.H"
#endif

//*********************************PL010V17*********************************//
#ifdef PL010V17
	#include "PL010V17.H"
#endif

//*********************************PL010V17AMP*********************************//
#ifdef PL010V17AMP
	#include "PL010V17AMP.H"
#endif
//*********************************PL012V11*********************************//
#ifdef PL012V11
	#include "PL012V11.H"
#endif
//*********************************PL012V11LCD*********************************//
#ifdef PL012V11LCD
	#include "PL012V11LCD.H"
#endif
//*********************************PL012V20*********************************//
#ifdef PL012V20
	#include "PL012V20.H"
#endif
//*********************************PL012V30*********************************//
#ifdef PL012V30
	#include "PL012V30.H"
#endif

//*********************************PL012V30AMP*********************************//
#ifdef PL012V30AMP
	#include "PL012V30AMP.H"
#endif
//*********************************PL013V10*********************************//
#ifdef PL013V10
	#include "PL013V10.H"
#endif

//*********************************PL013V11*********************************//
#ifdef PL013V11
	#include "PL013V11.H"
#endif

//*********************************PL013V20*********************************//
#ifdef PL013V20
	#include "PL013V20.H"
#endif

//*********************************PM001V20*********************************//
#ifdef PM001V20
	#include "PM001V20.H"
#endif

//*********************************PM001V20HC*********************************//
#ifdef PM001V20HC
	#include "PM001V20HC.H"
#endif

//*********************************PM004V10*********************************//
#ifdef PM004V10
	#include "PM004V10.H"
#endif
//*********************************USBTEST_RS485TOUSART*********************************//
#ifdef USBTEST_RS485TOUSART
	#include "USBTEST_RS485TOUSART.H"
#endif
//*********************************PC001V22*********************************//
#ifdef PC001V22
	#include "PC001V22.H"
#endif


//*********************************PS005V10*********************************//
#ifdef PS005V10
	#include "PS005V10.H"
#endif

//*********************************PS005V11*********************************//
#ifdef PS005V11
	#include "PS005V11.H"
#endif


//*********************************IAP*********************************//
#ifdef IAP
	#include "IAP.H"
#endif




































/*****************************************************************************************************/
/*===========================================�ָ���==================================================*/
/*****************************************************************************************************/
/********************************λ������**************************************
* λ������,ʵ��51���Ƶ�GPIO���ƹ���
* ����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
* IO�ڲ����궨��
******************************************************************************/

/********************************˵��**************************************
* 1 λ�󶨹�ʽ--λ����������
* SRAM �� 0x2000 0000 ~ 0x200f ffff
* ����󶨺��µĵ�ַΪ AliasAddr = 0x22000000 +��(A-0x2000 0000)*8 +n��4��
* A �Ǳ����ĵ�ַ����SRAM ��֮�� ��������һ����֪��
* n ���ڱ����е�λ
* Ƭ������ 0x4000 0000 ~ 0x400f ffff
* ����󶨺��µĵ�ַΪ  AliasAddr = 0x42000000 +��(A-0x4000 0000)*8 +n��4��
* 									0x42000000 +��(A-0x4000 0000)*32 +n*4����
* ��Ҫע�����λ��ֻ�����һλΪ��Чֵ��
* (��Ҫ������IO��
*******************************************************************************/



//#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
// //�Ѹõ�ַת����һ��ָ��
// #define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))

// #define BIT_ADDR(addr, bitnum)   MEM_ADDR( BITBAND(addr, bitnum)  )

// #define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C
// #define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C
// #define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C
// #define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C
// #define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C

// #define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808
// #define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08
// #define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008
// #define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408
// #define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808
////-----------------------------------------------------
// //-----------------------------------------------------
// //ʹ�÷�����
// //����PAout(0)=1;PBout(3)=0;
// //-----------------------------------------------------
//#define PAout(n) BITBAND(GPIOA_ODR_Addr,n)
//#define PAin(n) BITBAND(GPIOA_IDR_Addr,n)
//#define PBout(n) BITBAND(GPIOB_ODR_Addr,n)
//#define PBin(n) BITBAND(GPIOB_IDR_Addr,n)
//#define PCout(n) BITBAND(GPIOC_ODR_Addr,n)
//#define PCin(n) BITBAND(GPIOC_IDR_Addr,n)
//#define PDout(n) BITBAND(GPIOD_ODR_Addr,n)
//#define PDin(n) BITBAND(GPIOD_IDR_Addr,n)
//#define PEout(n) BITBAND(GPIOE_ODR_Addr,n)
//#define PEin(n) BITBAND(GPIOE_IDR_Addr,n)
////-----------------------------------------------------
// //-----------------------------------------------------
// //ʹ�÷�����
// //����PA0=1;PB0=0;
// //-----------------------------------------------------
// #define PA0   BIT_ADDR(GPIOA_ODR_Addr, 0)  //���
// #define PA1   BIT_ADDR(GPIOA_ODR_Addr, 1)  //���
// #define PA2   BIT_ADDR(GPIOA_ODR_Addr, 2)  //���
// #define PA3   BIT_ADDR(GPIOA_ODR_Addr, 3)  //���
// #define PA4   BIT_ADDR(GPIOA_ODR_Addr, 4)  //���
// #define PA5   BIT_ADDR(GPIOA_ODR_Addr, 5)  //���
// #define PA6   BIT_ADDR(GPIOA_ODR_Addr, 6)  //���
// #define PA7   BIT_ADDR(GPIOA_ODR_Addr, 7)  //���
// #define PA8   BIT_ADDR(GPIOA_ODR_Addr, 8)  //���
// #define PA9   BIT_ADDR(GPIOA_ODR_Addr, 9)  //���
// #define PA10  BIT_ADDR(GPIOA_ODR_Addr, 10) //���
// #define PA11  BIT_ADDR(GPIOA_ODR_Addr, 11) //���
// #define PA12  BIT_ADDR(GPIOA_ODR_Addr, 12) //���
// #define PA13  BIT_ADDR(GPIOA_ODR_Addr, 13) //���
// #define PA14  BIT_ADDR(GPIOA_ODR_Addr, 14) //���
// #define PA15  BIT_ADDR(GPIOA_ODR_Addr, 15) //���

// #define PA0in   BIT_ADDR(GPIOA_IDR_Addr, 0)  //����
// #define PA1in   BIT_ADDR(GPIOA_IDR_Addr, 1)  //����
// #define PA2in   BIT_ADDR(GPIOA_IDR_Addr, 2)  //����
// #define PA3in   BIT_ADDR(GPIOA_IDR_Addr, 3)  //����
// #define PA4in   BIT_ADDR(GPIOA_IDR_Addr, 4)  //����
// #define PA5in   BIT_ADDR(GPIOA_IDR_Addr, 5)  //����
// #define PA6in   BIT_ADDR(GPIOA_IDR_Addr, 6)  //����
// #define PA7in   BIT_ADDR(GPIOA_IDR_Addr, 7)  //����
// #define PA8in   BIT_ADDR(GPIOA_IDR_Addr, 8)  //����
// #define PA9in   BIT_ADDR(GPIOA_IDR_Addr, 9)  //����
// #define PA10in  BIT_ADDR(GPIOA_IDR_Addr, 10) //����
// #define PA11in  BIT_ADDR(GPIOA_IDR_Addr, 11) //����
// #define PA12in  BIT_ADDR(GPIOA_IDR_Addr, 12) //����
// #define PA13in  BIT_ADDR(GPIOA_IDR_Addr, 13) //����
// #define PA14in  BIT_ADDR(GPIOA_IDR_Addr, 14) //����
// #define PA15in  BIT_ADDR(GPIOA_IDR_Addr, 15) //����

// //-----------------------------------------------------
// #define PB0   BIT_ADDR(GPIOB_ODR_Addr, 0)  //���
// #define PB1   BIT_ADDR(GPIOB_ODR_Addr, 1)  //���
// #define PB2   BIT_ADDR(GPIOB_ODR_Addr, 2)  //���
// #define PB3   BIT_ADDR(GPIOB_ODR_Addr, 3)  //���
// #define PB4   BIT_ADDR(GPIOB_ODR_Addr, 4)  //���
// #define PB5   BIT_ADDR(GPIOB_ODR_Addr, 5)  //���
// #define PB6   BIT_ADDR(GPIOB_ODR_Addr, 6)  //���
// #define PB7   BIT_ADDR(GPIOB_ODR_Addr, 7)  //���
// #define PB8   BIT_ADDR(GPIOB_ODR_Addr, 8)  //���
// #define PB9   BIT_ADDR(GPIOB_ODR_Addr, 9)  //���
// #define PB10  BIT_ADDR(GPIOB_ODR_Addr, 10) //���
// #define PB11  BIT_ADDR(GPIOB_ODR_Addr, 11) //���
// #define PB12  BIT_ADDR(GPIOB_ODR_Addr, 12) //���
// #define PB13  BIT_ADDR(GPIOB_ODR_Addr, 13) //���
// #define PB14  BIT_ADDR(GPIOB_ODR_Addr, 14) //���
// #define PB15  BIT_ADDR(GPIOB_ODR_Addr, 15) //���

// #define PB0in   BIT_ADDR(GPIOB_IDR_Addr, 0)  //����
// #define PB1in   BIT_ADDR(GPIOB_IDR_Addr, 1)  //����
// #define PB2in   BIT_ADDR(GPIOB_IDR_Addr, 2)  //����
// #define PB3in   BIT_ADDR(GPIOB_IDR_Addr, 3)  //����
// #define PB4in   BIT_ADDR(GPIOB_IDR_Addr, 4)  //����
// #define PB5in   BIT_ADDR(GPIOB_IDR_Addr, 5)  //����
// #define PB6in   BIT_ADDR(GPIOB_IDR_Addr, 6)  //����
// #define PB7in   BIT_ADDR(GPIOB_IDR_Addr, 7)  //����
// #define PB8in   BIT_ADDR(GPIOB_IDR_Addr, 8)  //����
// #define PB9in   BIT_ADDR(GPIOB_IDR_Addr, 9)  //����
// #define PB10in  BIT_ADDR(GPIOB_IDR_Addr, 10) //����
// #define PB11in  BIT_ADDR(GPIOB_IDR_Addr, 11) //����
// #define PB12in  BIT_ADDR(GPIOB_IDR_Addr, 12) //����
// #define PB13in  BIT_ADDR(GPIOB_IDR_Addr, 13) //����
// #define PB14in  BIT_ADDR(GPIOB_IDR_Addr, 14) //����
// #define PB15in  BIT_ADDR(GPIOB_IDR_Addr, 15) //����
// //----------------------------------------------------
// #define PC0   BIT_ADDR(GPIOC_ODR_Addr, 0)  //���
// #define PC1   BIT_ADDR(GPIOC_ODR_Addr, 1)  //���
// #define PC2   BIT_ADDR(GPIOC_ODR_Addr, 2)  //���
// #define PC3   BIT_ADDR(GPIOC_ODR_Addr, 3)  //���
// #define PC4   BIT_ADDR(GPIOC_ODR_Addr, 4)  //���
// #define PC5   BIT_ADDR(GPIOC_ODR_Addr, 5)  //���
// #define PC6   BIT_ADDR(GPIOC_ODR_Addr, 6)  //���
// #define PC7   BIT_ADDR(GPIOC_ODR_Addr, 7)  //���
// #define PC8   BIT_ADDR(GPIOC_ODR_Addr, 8)  //���
// #define PC9   BIT_ADDR(GPIOC_ODR_Addr, 9)  //���
// #define PC10  BIT_ADDR(GPIOC_ODR_Addr, 10) //���
// #define PC11  BIT_ADDR(GPIOC_ODR_Addr, 11) //���
// #define PC12  BIT_ADDR(GPIOC_ODR_Addr, 12) //���
// #define PC13  BIT_ADDR(GPIOC_ODR_Addr, 13) //���
// #define PC14  BIT_ADDR(GPIOC_ODR_Addr, 14) //���
// #define PC15  BIT_ADDR(GPIOC_ODR_Addr, 15) //���

// #define PC0in   BIT_ADDR(GPIOC_IDR_Addr, 0)  //����
// #define PC1in   BIT_ADDR(GPIOC_IDR_Addr, 1)  //����
// #define PC2in   BIT_ADDR(GPIOC_IDR_Addr, 2)  //����
// #define PC3in   BIT_ADDR(GPIOC_IDR_Addr, 3)  //����
// #define PC4in   BIT_ADDR(GPIOC_IDR_Addr, 4)  //����
// #define PC5in   BIT_ADDR(GPIOC_IDR_Addr, 5)  //����
// #define PC6in   BIT_ADDR(GPIOC_IDR_Addr, 6)  //����
// #define PC7in   BIT_ADDR(GPIOC_IDR_Addr, 7)  //����
// #define PC8in   BIT_ADDR(GPIOC_IDR_Addr, 8)  //����
// #define PC9in   BIT_ADDR(GPIOC_IDR_Addr, 9)  //����
// #define PC10in  BIT_ADDR(GPIOC_IDR_Addr, 10) //����
// #define PC11in  BIT_ADDR(GPIOC_IDR_Addr, 11) //����
// #define PC12in  BIT_ADDR(GPIOC_IDR_Addr, 12) //����
// #define PC13in  BIT_ADDR(GPIOC_IDR_Addr, 13) //����
// #define PC14in  BIT_ADDR(GPIOC_IDR_Addr, 14) //����
// #define PC15in  BIT_ADDR(GPIOC_IDR_Addr, 15) //����

// //----------------------------------------------------
// #define PD0   BIT_ADDR(GPIOD_ODR_Addr, 0)  //���
// #define PD1   BIT_ADDR(GPIOD_ODR_Addr, 1)  //���
// #define PD2   BIT_ADDR(GPIOD_ODR_Addr, 2)  //���
// #define PD3   BIT_ADDR(GPIOD_ODR_Addr, 3)  //���
// #define PD4   BIT_ADDR(GPIOD_ODR_Addr, 4)  //���
// #define PD5   BIT_ADDR(GPIOD_ODR_Addr, 5)  //���
// #define PD6   BIT_ADDR(GPIOD_ODR_Addr, 6)  //���
// #define PD7   BIT_ADDR(GPIOD_ODR_Addr, 7)  //���
// #define PD8   BIT_ADDR(GPIOD_ODR_Addr, 8)  //���
// #define PD9   BIT_ADDR(GPIOD_ODR_Addr, 9)  //���
// #define PD10  BIT_ADDR(GPIOD_ODR_Addr, 10) //���
// #define PD11  BIT_ADDR(GPIOD_ODR_Addr, 11) //���
// #define PD12  BIT_ADDR(GPIOD_ODR_Addr, 12) //���
// #define PD13  BIT_ADDR(GPIOD_ODR_Addr, 13) //���
// #define PD14  BIT_ADDR(GPIOD_ODR_Addr, 14) //���
// #define PD15  BIT_ADDR(GPIOD_ODR_Addr, 15) //���

// #define PD0in   BIT_ADDR(GPIOD_IDR_Addr, 0)  //����
// #define PD1in   BIT_ADDR(GPIOD_IDR_Addr, 1)  //����
// #define PD2in   BIT_ADDR(GPIOD_IDR_Addr, 2)  //����
// #define PD3in   BIT_ADDR(GPIOD_IDR_Addr, 3)  //����
// #define PD4in   BIT_ADDR(GPIOD_IDR_Addr, 4)  //����
// #define PD5in   BIT_ADDR(GPIOD_IDR_Addr, 5)  //����
// #define PD6in   BIT_ADDR(GPIOD_IDR_Addr, 6)  //����
// #define PD7in   BIT_ADDR(GPIOD_IDR_Addr, 7)  //����
// #define PD8in   BIT_ADDR(GPIOD_IDR_Addr, 8)  //����
// #define PD9in   BIT_ADDR(GPIOD_IDR_Addr, 9)  //����
// #define PD10in  BIT_ADDR(GPIOD_IDR_Addr, 10) //����
// #define PD11in  BIT_ADDR(GPIOD_IDR_Addr, 11) //����
// #define PD12in  BIT_ADDR(GPIOD_IDR_Addr, 12) //����
// #define PD13in  BIT_ADDR(GPIOD_IDR_Addr, 13) //����
// #define PD14in  BIT_ADDR(GPIOD_IDR_Addr, 14) //����
// #define PD15in  BIT_ADDR(GPIOD_IDR_Addr, 15) //����

// //----------------------------------------------------
// #define PE0   BIT_ADDR(GPIOE_ODR_Addr, 0)  //���
// #define PE1   BIT_ADDR(GPIOE_ODR_Addr, 1)  //���
// #define PE2   BIT_ADDR(GPIOE_ODR_Addr, 2)  //���
// #define PE3   BIT_ADDR(GPIOE_ODR_Addr, 3)  //���
// #define PE4   BIT_ADDR(GPIOE_ODR_Addr, 4)  //���
// #define PE5   BIT_ADDR(GPIOE_ODR_Addr, 5)  //���
// #define PE6   BIT_ADDR(GPIOE_ODR_Addr, 6)  //���
// #define PE7   BIT_ADDR(GPIOE_ODR_Addr, 7)  //���
// #define PE8   BIT_ADDR(GPIOE_ODR_Addr, 8)  //���
// #define PE9   BIT_ADDR(GPIOE_ODR_Addr, 9)  //���
// #define PE10  BIT_ADDR(GPIOE_ODR_Addr, 10) //���
// #define PE11  BIT_ADDR(GPIOE_ODR_Addr, 11) //���
// #define PE12  BIT_ADDR(GPIOE_ODR_Addr, 12) //���
// #define PE13  BIT_ADDR(GPIOE_ODR_Addr, 13) //���
// #define PE14  BIT_ADDR(GPIOE_ODR_Addr, 14) //���
// #define PE15  BIT_ADDR(GPIOE_ODR_Addr, 15) //���

// #define PE0in   BIT_ADDR(GPIOE_IDR_Addr, 0)  //����
// #define PE1in   BIT_ADDR(GPIOE_IDR_Addr, 1)  //����
// #define PE2in   BIT_ADDR(GPIOE_IDR_Addr, 2)  //����
// #define PE3in   BIT_ADDR(GPIOE_IDR_Addr, 3)  //����
// #define PE4in   BIT_ADDR(GPIOE_IDR_Addr, 4)  //����
// #define PE5in   BIT_ADDR(GPIOE_IDR_Addr, 5)  //����
// #define PE6in   BIT_ADDR(GPIOE_IDR_Addr, 6)  //����
// #define PE7in   BIT_ADDR(GPIOE_IDR_Addr, 7)  //����
// #define PE8in   BIT_ADDR(GPIOE_IDR_Addr, 8)  //����
// #define PE9in   BIT_ADDR(GPIOE_IDR_Addr, 9)  //����
// #define PE10in  BIT_ADDR(GPIOE_IDR_Addr, 10) //����
// #define PE11in  BIT_ADDR(GPIOE_IDR_Addr, 11) //����
// #define PE12in  BIT_ADDR(GPIOE_IDR_Addr, 12) //����
// #define PE13in  BIT_ADDR(GPIOE_IDR_Addr, 13) //����
// #define PE14in  BIT_ADDR(GPIOE_IDR_Addr, 14) //����
// #define PE15in  BIT_ADDR(GPIOE_IDR_Addr, 15) //����




void WOW_Configuration(void);		//���ú���
void WOW_Server(void);					//������
void MainServer(void);          //

#endif






