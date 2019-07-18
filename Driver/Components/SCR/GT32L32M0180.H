/******************************** User_library *********************************
* �ļ��� 	: STM32_GT32L32M0180.H
* ����   	: wegam@sina.com
* �汾   	: V
* ����   	: 2017/01/15
* ˵��   	: 
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifndef __GT32L32M0180_H
#define __GT32L32M0180_H

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_nvic.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_dma.h"


//#include "STM32_WOW.H"		//ʹ��λ��

#include	"stdio.h"				//����printf
#include	"string.h"			//����printf
#include	"stdarg.h"			//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"			//malloc��̬�����ڴ�ռ�


#include "STM32_SPI.H"

/* Private define ------------------------------------------------------------*/
//____________��Ŀ����
#define	STM32_GT32L32M0180
//#define	GT32L32_SPIPORT_EN		//ʹ��SPI�ӿ�
#define	STM32_LCD_GT32L32M0180
#define	HC_PL010V11			//LCD���ذ�


//____________SPI�ٶȶ���
#define GT32L32_CLK_SpeedL	0
#define GT32L32_CLK_SpeedH	1

#define	GT32L32M0180_PageSize		256			//ҳ��С-�ֽ�
#define	GT32L32M0180_SectorSize	4096		//������С4K(16ҳ)-�ֽ�
#define	GT32L32M0180_BlockSize	65536		//���С64K(16������256ҳ)-�ֽ�

//----�ַ������ֿ����ַ��------------------------------------------------------------
typedef enum
{
	ASCII_BaseAddr_None					=	(u32)0x000000,			//δ���õ�ַ
	
	ASCII_BaseAddr_zf5x7				=	(u32)0x100000,			//5x7����ASCII��׼�ַ�
	ASCII_BaseAddr_zfs7x8				=	(u32)0x100300,			//7x8����ASCII��׼�ַ�
	ASCII_BaseAddr_zfm7x8				=	(u32)0x100600,			//7x8����ASCII�����ַ�
	ASCII_BaseAddr_zf6x12				=	(u32)0x100900,			//6x12����ASCII�ַ�
	ASCII_BaseAddr_zfs8x16			=	(u32)0x100D80,			//8x16����ASCII��׼�ַ�
	ASCII_BaseAddr_zfm8x16			=	(u32)0x101580,			//8x16����ASCII�����ַ�
	ASCII_BaseAddr_zf12x24			=	(u32)0x101B80,			//12x24����ASCII��׼�ַ�
	ASCII_BaseAddr_zfs16x32			=	(u32)0x102D80,			//16x32����ASCII��׼�ַ�
	ASCII_BaseAddr_zfm16x32			=	(u32)0x104580,			//16x32����ASCII�����ַ�
	
	ASCII_BaseAddr_ar12					=	(u32)0x105D80,			//12���󲻵ȿ�ASCII��ͷ��Arial���ַ�
	ASCII_BaseAddr_ar16					=	(u32)0x106740,			//16���󲻵ȿ�ASCII��ͷ��Arial���ַ�
	ASCII_BaseAddr_ar24					=	(u32)0x107400,			//24���󲻵ȿ�ASCII��ͷ��Arial���ַ�
	ASCII_BaseAddr_ar32					=	(u32)0x108FC0,			//32���󲻵ȿ�ASCII��ͷ��Arial���ַ�
	
	ASCII_BaseAddr_TNR12				=	(u32)0x10C080,			//12���󲻵ȿ�ASCII������Times New Roman���ַ�
	ASCII_BaseAddr_TNR16				=	(u32)0x10CA50,			//16���󲻵ȿ�ASCII������Times New Roman���ַ�
	ASCII_BaseAddr_TNR24				=	(u32)0x10D740,			//24���󲻵ȿ�ASCII������Times New Roman���ַ�
	ASCII_BaseAddr_TNR32				=	(u32)0x10F340,			//32���󲻵ȿ�ASCII������Times New Roman���ַ�
	
	Digital_BaseAddr_sz14x28		=	(u32)0x112400,			//14x28���ַ����ַ�
	Digital_BaseAddr_sz20x40		=	(u32)0x112748,			//20x40���ַ����ַ�
	Digital_BaseAddr_sz28				=	(u32)0x112CE8,			//28���󲻵ȿ����ַ����ַ�
	Digital_BaseAddr_sz40				=	(u32)0x113396,			//40���󲻵ȿ����ַ����ַ�
	
	GB18030_BaseAddr_zf12x12		=	(u32)0x113D0E,			//12x12����GB18030�ַ�
	GB18030_BaseAddr_zf16x16		=	(u32)0x194FDE,			//16x16����GB18030�ַ�
	GB18030_BaseAddr_zf24x24		=	(u32)0x2743DE,			//24x24����GB18030�ַ�
	GB18030_BaseAddr_zf32x32		=	(u32)0x47AE10,			//32x32����GB18030�ַ�
	
	GB18030_BaseAddr_hz12x12		=	(u32)0x113D0E,			//12x12����GB18030����
	GB18030_BaseAddr_hz16x16		=	(u32)0x194FDE,			//16x16����GB18030����
	GB18030_BaseAddr_hz24x24		=	(u32)0x2743DE,			//24x24����GB18030����
	GB18030_BaseAddr_hz32x32		=	(u32)0x47AE10,			//32x32����GB18030����
	
	Unicode_To_GBK_BaseAddr			=	(u32)0x46A90E,			//Unicode->GBKת���
	BIG5_To_GBK_BaseAddr				=	(u32)0x7F1E10,			//BIG5->GBKת���
	
	BarCode_BaseAddr_12x27			=	(u32)0x478FD2,			//12x27�������ַ� EAN13
	BarCode_BaseAddr_16x20			=	(u32)0x479C7A,			//16x20�������ַ� CODE128
	
	AntennaCode_BaseAddr_12X12	=	(u32)0x47AD32,			//12x12���߷���
	BatteryCode_BaseAddr_12X12	=	(u32)0x47ADAA,			//12x12��ط���
	
	Reserved_BaseAddr_LSB				=	(u32)0x7F8760,			//�û����͵�ַ(��2040.5����)
	Reserved_BaseAddr_MSB				=	(u32)0x7FFFFF,			//�û����ߵ�ַ(��2048����)	112ҳ
}	GT32L32BaseAddrTypeDef;		//�ַ������ֿ����ַ��
//----�����б�------------------------------------------------------------
typedef enum
{
	GT32L32_Error_None					=	(u8)0x00,			//�޴���
	GT32L32_Error_Addr					=	(u8)0x01,			//��ַ����
	GT32L32_Error_ASCIICode			=	(u8)0x02,			//ASCII�����
	GT32L32_Error_HzEmp					=	(u8)0x03,			//���������8λΪ0x7F,Ϊ������
	
}	GT32L32FlagDef;		//��������б�
//----���ݽṹ��------------------------------------------------------------
typedef struct      /*Card Identification Data*/
{
//	GT32L32_BaseAddr_TypeDef		BaseAddr;		//�ַ������ֿ����ַ��
	unsigned long	Address;						//�ַ���ʼ��ַ
//	unsigned char	Buffer[200];				//��������
//	unsigned char	BufferSize;					//�������ݳ���
} GT32L32DataDef;
//----��Ϣ�ṹ��------------------------------------------------------------
typedef struct      _GT32L32/*Card Identification Data*/
{
	spi_def								SPI;
	GT32L32DataDef				Data;	
	GT32L32FlagDef				Flag;			//�����б�
}GT32L32Def;			//GT32L32M0180

/* Exported macro ------------------------------------------------------------*/





/* Exported functions ------------------------------------------------------- */
void api_gt32l32_configuration(GT32L32Def *pInfo);

u8 GT32L32_ReadStatus(GT32L32Def *pInfo);			//��ȡоƬ״̬
void GT32L32_ChipErase(GT32L32Def *pInfo);		//�����û�������


//----------------------------------------------------------�ӿں���
u32 api_gt32l32_get_antenna(u8	NUM,u8 *GetBuffer);						//����ͼ����ó���
u32 api_gt32l32_get_battery(u8	NUM,u8 *GetBuffer);						//���ͼ����ó���
u16	api_gt32l32_get_code(u8 font,u16 word,u8 *ReadBuffer);		//���ֿ��ж����ݲ��������ݳ���
//u8  GT32L32_ReadWriteByte(u8 Data);		//��дһ���ֽ�
//void GT32L32_SendByte(GT32L32Def *pInfo,u8 ByteData);		//����һ���ֽ�����,�ȷ��͸�λ
//u8 GT32L32_ReadByte(GT32L32Def *pInfo);		//��һ���ֽ�,�ȶ���λ

void API_GT32L32M0180_SectorErase(unsigned short UserSectorNum);
void API_GT32L32M0180_PageWrite(unsigned short UserPageNum,unsigned char* UserData);

//unsigned char UnicodeCheck(unsigned char* data,unsigned short len);
//unsigned long Unicode2GBK(unsigned short Unicode);	//UNICODEתGBK���ӳ���㷨������������,ת��ȫ��˫�ֽ�����

#endif

/*******************	wegam@sina.com	*******************/
/*********************	2017/01/21	*********************/
/**********************	END OF FILE	*********************/

