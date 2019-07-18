#ifndef __MX25L4006E_H
#define __MX25L4006E_H

#include "stm32f10x_type.h"
//#include "STM32_SPI.H"
#include "STM32_PWM.H"

#include "SPI_FLASH.H"


/******************************MX25L4006E�����*******************************/
typedef enum _MX25L4006ECmd
{
	MX25L4006E_WREN			=	(unsigned char)0x06,			//write Enable��дʹ��
	MX25L4006E_WRDI			=	(unsigned char)0x04,				//write disable��дʧ��
	MX25L4006E_WRSR			=	(unsigned char)0x01,				//write status register��д״̬�Ĵ���
	MX25L4006E_RDID			=	(unsigned char)0x9F,				//read identification�����豸��Ϣ��3byte����һ�ֽ�Ϊ�����̴��룬�ڶ������ֽ�ΪоƬ����
	MX25L4006E_RDSR			=	(unsigned char)0x05,				//read status register����״̬�Ĵ���
	MX25L4006E_READ			=	(unsigned char)0x03,				//read data��������
	MX25L4006E_FREAD		=	(unsigned char)0x0B,				//fast read data�����ٶ�ȡ����
	MX25L4006E_RDSFDP		=	(unsigned char)0x5A,				//Read SFDP:��(SFDP)Serial FlashоƬ�淶
	MX25L4006E_RES			=	(unsigned char)0xAB,				//Read Electronic Signature:��������ȡ���ӱ�ǩ
	MX25L4006E_REMS			=	(unsigned char)0x90,				//Read Electronic Manufacturer & Device ID:������������Ϣ��ID
	MX25L4006E_DREAD		=	(unsigned char)0x3B,				//Double Output Mode command��˫���ģʽ����
	MX25L4006E_SE				=	(unsigned char)0x20,				//Sector Erase����������
	MX25L4006E_BE				=	(unsigned char)0x52,				//Block Erase�������Ҳ������0XD8
	MX25L4006E_CE				=	(unsigned char)0x60,				//Chip Erase��оƬ����; ��Ƭ����; Ҳ������0XC7
	MX25L4006E_PP				=	(unsigned char)0x02,				//Page Program��дҳ��
	MX25L4006E_DP				=	(unsigned char)0xB9,				//Deep Power Down�����ʡ��״̬
	MX25L4006E_RDP			=	(unsigned char)0xAB					//Release from Deep Power-down���˳���ȵ���
}MX25L4006ECmdDef;
typedef enum _MX25L4006EMemory
{
	MX25L4006E_ChipSize			=	(unsigned long)524288,	//Flash������byte��λ--512KByte
	MX25L4006E_PageSize			=	(unsigned long)256,			//ҳ��Сbyte��λ
	MX25L4006E_SectorSize		=	(unsigned long)4096,		//������Сbyte��λ-4KByte/16Page
	MX25L4006E_BulkSize			=	(unsigned long)65536		//���Сbyte��λ-64KByte/16Sector
}MX25L4006EMemoryDef;	

//----���ݽṹ��------------------------------------------------------------
typedef struct      /*Card Identification Data*/
{
	unsigned long			Address;						//�ַ���ʼ��ַ
	unsigned char			Buffer[200];				//��������
	unsigned char			BufferSize;					//�������ݳ���
}MX25L4006EDataDef;
typedef struct	_MX25L4006E
{
	SPIFlashDef				SPIFlash;			//SPIFlash����
	MX25L4006EDataDef Data;
}MX25L4006EDef;



void MX25L4006E_Initialize(MX25L4006EDef *MX25Lx);
void MX25L4006E_Server(void);


#endif
