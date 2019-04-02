//#ifdef MX25L4006E

#include "MX25L4006E.H"

//#include	"stdio.h"			//����printf
#include	"string.h"			//����printf
//#include	"stdarg.h"			//���ڻ�ȡ��ȷ�������Ĳ���
//#include	"stdlib.h"			//malloc��̬�����ڴ�ռ�


SPIFlashCmdDef MX25L4006ECmd	=
{
	MX25L4006E_WREN,			//write Enable��дʹ��
	MX25L4006E_WRDI,				//write disable��дʧ��
	MX25L4006E_WRSR,				//write status register��д״̬�Ĵ���
	MX25L4006E_RDID,				//read identification�����豸��Ϣ��3byte����һ�ֽ�Ϊ�����̴��룬�ڶ������ֽ�ΪоƬ����
	MX25L4006E_RDSR,				//read status register����״̬�Ĵ���
	MX25L4006E_READ,				//read data��������
	MX25L4006E_FREAD,				//fast read data�����ٶ�ȡ����
	MX25L4006E_RDSFDP,				//Read SFDP:��(SFDP)Serial FlashоƬ�淶
	MX25L4006E_RES,				//Read Electronic Signature:��������ȡ���ӱ�ǩ
	MX25L4006E_REMS,				//Read Electronic Manufacturer & Device ID:������������Ϣ��ID
	MX25L4006E_DREAD,				//Double Output Mode command��˫���ģʽ����
	MX25L4006E_SE,				//Sector Erase����������
	MX25L4006E_BE,				//Block Erase�������Ҳ������0XD8
	MX25L4006E_CE,				//Chip Erase��оƬ����; ��Ƭ����; Ҳ������0XC7
	MX25L4006E_PP,				//Page Program��дҳ��
	MX25L4006E_DP,				//Deep Power Down�����ʡ��״̬
	MX25L4006E_RDP					//Release from Deep Power-down���˳���ȵ���
};
SPIFlashMemoryDef MemorySize	=	
{
	MX25L4006E_ChipSize,
	MX25L4006E_PageSize,
	MX25L4006E_SectorSize,
	MX25L4006E_BulkSize
};

#define	SPI_BUFFERSIZE	10

char SPI_TX_Buffer[SPI_BUFFERSIZE]={0X90,0X0F,0X0A,0X00};
char SPI_RX_Buffer[SPI_BUFFERSIZE]={0};

SPI_InitTypeDef  SPI_InitStructure;


vu32 FLASH_ID = 0;

u32 Temp = 0,Temp0=0,Temp1=0,Temp2=0;		//����
MX25L4006EDef MX25L4006E;
/*******************************************************************************
* ������		:
* ��������	:
* ����		:
* ���		:
* ���� 		:
*******************************************************************************/
void MX25L4006E_Initialize(MX25L4006EDef *MX25Lx)
{
//	SPIFlashCmdDef *Cmd	=	&MX25Lx->SPIFlash.Cmd;
	
	memcpy((unsigned char*)&(MX25Lx->SPIFlash.Cmd),(unsigned char*)&MX25L4006ECmd,sizeof(SPIFlashCmdDef));		//����֧�ֵ�����
	memcpy((unsigned char*)&(MX25Lx->SPIFlash.Memory),(unsigned char*)&MemorySize,sizeof(SPIFlashMemoryDef));		//�����洢������Ϣ
	
	SPI_FLASH_Initialize(&MX25Lx->SPIFlash);
//	SPI_StructConf(&SPI_InitStructure);										//���ò���
	
//	SPI_BASIC_Configuration(SPI1,&SPI_InitStructure);			//SPI��������
	
//	SPI_DMA_Configuration(SPI1,&SPI_InitStructure,(u32*)SPI_TX_Buffer,(u32*)SPI_RX_Buffer,SPI_BUFFERSIZE);				//SPI_DMA����
	
//	PWM_OUT(TIM2,PWM_OUTChannel2,1,500);	//PWM�趨-20161127�汾
	
//	SPIT_Configuration(SPI1);							//SPI����ʹ�÷�ʽ����
//	SPI_Cmd(SPI1, ENABLE);
//	SPI_I2S_ReceiveData(SPI1);


}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void MX25L4006E_Server(void)
{
//	if(Temp==0)
//		MX25L4006E_ReadID();
//	SPI_DMA_BufferRead((u32*)SPI_RX_Buffer);
//	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
//	SPI_DMASend(SPI1,(u32*)SPI_TX_Buffer,(u32*)SPI_RX_Buffer,SPI_BUFFERSIZE);	//SPI_DMA���ͺ���----��ߵ�ʡ�Ժž��ǿɱ����
//	GPIO_SetBits(GPIOA,GPIO_Pin_4);	
//	SPI_Cmd(SPI1, ENABLE);
//	SPI_DMA_ReceiveSendByte(SPI1,8);		//DMA�շ�����
//	SPI_Cmd(SPI1, DISABLE);
}


//#endif

