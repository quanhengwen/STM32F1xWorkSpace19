#ifdef ICCARD
#include "ICCARD.H"
#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_USART.H"

#include	"stdio.h"			//����printf
#include	"string.h"		//����printf
#include	"stdarg.h"		//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"		//malloc��̬�����ڴ�ռ�


#define BufferSize	30

u8	Sector	=	1;						//���� 0~1
u8	Block	=	1;						//�� 0~2

#define	ICCARD_Number	=	0;			//����0~99999999
u16 systimecount	=	0;
u16 ICCARD_SendLenth	=	0;			//ȫ�ֱ��������ڷ������ݳ���
u8 runFlag	=	0;
u8 RxdBuffer[BufferSize]={0};
u8 RevBuffer[BufferSize]={0};
u8 TxdBuffer[BufferSize]={0xE1,0x24,0x01,0x01,0x60,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x1C,0x0D,0x1E};
u8 CmdBuffer[30]	=	{0};

//=================��ȡ������
u8 ICCARD_PassWord[6] =
  {
    0xFF,   /* PassWord0 */					//6�ֽ�����
    0xFF,   /* PassWord1 */
    0xFF,   /* PassWord2 */
    0xFF,		/* PassWord3 */
    0xFF,   /* PassWord4 */
    0xFF		/* PassWord5 */
	};
//=================��ȡ������
u8 ICCARD_Data[16] =
  {
    0x01,   /* Data0 	*/			//16�ֽ�����
    0x02,   /* Data1 	*/
		0x03,   /* Data2 	*/
		0x04,   /* Data3 	*/
		0x05,   /* Data4 	*/
		0x06,   /* Data5 	*/
		0x07,   /* Data6 	*/
		0x08,   /* Data7 	*/
		0x09,   /* Data8 	*/
		0x10,   /* Data9 	*/
		0x11,   /* Data10 	*/
		0x12,   /* Data11 	*/
		0x13,   /* Data12 	*/
		0x14,   /* Data13 	*/
		0x15,   /* Data14 	*/
		0x16   	/* Data15 	*/		
	};

//=================��������
u8 ICCARD_CMD_GetDataBlock1[] =
  {
    0xE1,   /* F-Head 	*/			//Ϊ֡ͷ����λ���·�ʱ�̶�Ϊ0XE1����λ��Ӧ��ʱ�̶�Ϊ0XD2��
    0x24,  	/* CmdType 	*/			//��������
    0x01,		/* Sector 	*/			//1�ֽ�������
    0x01,   /* Block 		*/			//1�ֽڿ��
    0x60,   /* KeyType	*/			//1�ֽ���Կģʽ KEYA��0x60��/KEYB(0x61)
    0xFF,   /* PassWord0 */					//6�ֽ�����
    0xFF,   /* PassWord1 */
    0xFF,   /* PassWord2 */
    0xFF,		/* PassWord3 */
    0xFF,   /* PassWord4 */
    0xFF,		/* PassWord5 */
    0x1C,   /* Crc16-HIGN */
    0x0D,		/* Crc16-LOW */
    0x1E   	/* F-End*/						//Ϊ֡β����λ���·�ʱ�̶�Ϊ0X1E����λ��Ӧ��ʱ�̶�Ϊ0X2D��
	};
//=================д������
u8 ICCARD_CMD_SetDataBlock1[] =
  {
    ICCARD_MO_SetDataBlock(Sector,Block,ICCARD_Number),
	};
//=================д������
u8 ICCARD_CMD_SetDataBlock[] =
  {
		0xE1,   /* F-Head 	:Ϊ֡ͷ����λ���·�ʱ�̶�Ϊ0XE1����λ��Ӧ��ʱ�̶�Ϊ0XD2*/
		0x25,  	/* CmdType 	:��������*/
		0x01,		/* Sector 	:1�ֽ�������*/
		0x01,   /* Block 	:1�ֽڿ��	*/
		0x60,   /* KeyType	:1�ֽ���Կģʽ KEYA��0x60��/KEYB(0x61)*/
		0xFF,   /* PassWord0 :6�ֽ�����*/
		0xFF,   /* PassWord1 */
		0xFF,   /* PassWord2 */
		0xFF,		/* PassWord3 */
		0xFF,   /* PassWord4 */
		0xFF,		/* PassWord5 */
		0xFF,   /* Data0 	:16�ֽ�����*/
		0xFF,   /* Data1 	*/
		0xFF,   /* Data2 	*/
		0xFF,   /* Data3 	*/
		0xFF,   /* Data4 	*/
		0xFF,   /* Data5 	*/
		0xFF,   /* Data6 	*/
		0xFF,   /* Data7 	*/
		0xFF,   /* Data8 	*/
		0xFF,   /* Data9 	*/
		0xFF,   /* Data10 	*/
		0xFF,   /* Data11 	*/
		0xFF,   /* Data12 	*/
		0xFF,   /* Data13 	*/
		0xFF,   /* Data14 	*/
		0xFF,   /* Data15 	*/
		0x1C,   /* Crc16-HIGN */
		0x0D,		/* Crc16-LOW */
		0x1E   	/* F-End	:Ϊ֡β����λ���·�ʱ�̶�Ϊ0X1E����λ��Ӧ��ʱ�̶�Ϊ0X2D*/
	};
//=================������������
u8 ICCARD_CMD_SetPassWord[] =
  {
		0xE1,   /* F-Head 	:Ϊ֡ͷ����λ���·�ʱ�̶�Ϊ0XE1����λ��Ӧ��ʱ�̶�Ϊ0XD2*/
		0x26,  	/* CmdType 	:��������*/
		0x01,		/* Sector 	:1�ֽ�������*/
		0x60,   /* KeyType	:1�ֽ���Կģʽ KEYA��0x60��/KEYB(0x61)*/
		0xFF,   /* OldPassWord0 :6�ֽھ�����*/
		0xFF,   /* OldPassWord1 */
		0xFF,   /* OldPassWord2 */
		0xFF,		/* OldPassWord3 */
		0xFF,   /* OldPassWord4 */
		0xFF,		/* OldPassWord5 */
		0xFF,   /* OldPassWord0 :6�ֽ�������*/
		0xFF,   /* OldPassWord1 */
		0xFF,   /* OldPassWord2 */
		0xFF,		/* OldPassWord3 */
		0xFF,   /* OldPassWord4 */
		0xFF,		/* OldPassWord5 */
		0x26,   /* Crc16-HIGN */
		0x63,		/* Crc16-LOW */
		0x1E   	/* F-End	:Ϊ֡β����λ���·�ʱ�̶�Ϊ0X1E����λ��Ӧ��ʱ�̶�Ϊ0X2D*/
	};
//=================���ö�ͷID	
u8 ICCARD_CMD_SetReaderID[] =
  {
    0xE1,   /* F-Head 	*/			//Ϊ֡ͷ����λ���·�ʱ�̶�Ϊ0XE1����λ��Ӧ��ʱ�̶�Ϊ0XD2��
    0x42,  	/* CmdType 	*/			//��������
		0x03,		/* ID-HIGN  	*/	//1�ֽ�������
		0xE8,		/* ID-HIGN  	*/	//1�ֽ�������
    0x5A,		/* Crc16-HIGN  	*/	//1�ֽ�������
    0xB8,   /* Crc16-LOW 		*/	//1�ֽڿ��
    0x1E   	/* F-End*/					//Ϊ֡β����λ���·�ʱ�̶�Ϊ0X1E����λ��Ӧ��ʱ�̶�Ϊ0X2D��
	};	
//=================��ȡ��ͷID	
u8 ICCARD_CMD_GetReaderID[] =
  {
    0xE1,   /* F-Head 			*/	//Ϊ֡ͷ����λ���·�ʱ�̶�Ϊ0XE1����λ��Ӧ��ʱ�̶�Ϊ0XD2��
    0x43,  	/* CmdType 			*/	//��������
    0x78,		/* Crc16-HIGN  	*/	//1�ֽ�������
    0xA7,   /* Crc16-LOW 		*/	//1�ֽڿ��
    0x1E   	/* F-End				*/	//Ϊ֡β����λ���·�ʱ�̶�Ϊ0X1E����λ��Ӧ��ʱ�̶�Ϊ0X2D��
	};
//=================���ö�������������	
const u8 ICCARD_CMD_SetReaderArea[] =
  {
    0xE1,   /* F-Head 	*/			//Ϊ֡ͷ����λ���·�ʱ�̶�Ϊ0XE1����λ��Ӧ��ʱ�̶�Ϊ0XD2��
    0x44,  	/* CmdType 	*/			//��������
		0x01,  	/* Sector 	*/			//������0,1,����0Ϊֻ����Ϊ����Ϣ
		0x01,  	/* Block 	*/				//��ţ�0,1,2
		0x60,   /* KeyType	*/			//1�ֽ���Կģʽ KEYA��0x60��/KEYB(0x61)
		0xFF,   /* PassWord0 */					//6�ֽ�����
    0xFF,   /* PassWord1 */
    0xFF,   /* PassWord2 */
    0xFF,		/* PassWord3 */
    0xFF,   /* PassWord4 */
    0xFF,		/* PassWord5 */
    0x9F,		/* Crc16-HIGN  	*/	//1�ֽ�������
    0x6F,   /* Crc16-LOW 		*/	//1�ֽڿ��
    0x1E   	/* F-End*/					//Ϊ֡β����λ���·�ʱ�̶�Ϊ0X1E����λ��Ӧ��ʱ�̶�Ϊ0X2D��
	};		
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void ICCARD_Configuration(void)
{
	SYS_Configuration();
	SysTick_Configuration(1000);	//ϵͳ���ʱ������72MHz,��λΪuS
	
//	GPIO_Configuration0();
//	TIM_Configuration(TIM1,7200,3000);	//��ʱʱ���趨
//	PWM_Configuration(TIM2,7200,10000,51);
//	PWM_OUT(TIM1,PWM_OUTChannel1,20000,50);		//PWM�趨
//	PWM_OUT(TIM2,PWM_OUTChannel1,20000,500);	//PWM�趨
//	PWM_OUT(TIM3,PWM_OUTChannel1,5000,30);		//PWM�趨
//	PWM_OUT(TIM3,PWM_OUTChannel2,5000,30);		//PWM�趨
//	PWM_OUT(TIM3,PWM_OUTChannel3,1000,500);		//PWM�趨
//	PWM_OUT(TIM4,PWM_OUTChannel1,20000,40);		//PWM�趨
	
	PWM_OUT(TIM2,PWM_OUTChannel1,2,500);		//PWM�趨
	USART_DMA_ConfigurationNR	(USART2,19200,(u32*)RxdBuffer,BufferSize);	//USART_DMA����--��ѯ��ʽ�������ж�
	GPIO_Configuration_OPP50(GPIOC,	GPIO_Pin_12);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50(GPIOD,	GPIO_Pin_2);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50(GPIOB,	GPIO_Pin_3);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50(GPIOB,	GPIO_Pin_4);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50(GPIOB,	GPIO_Pin_5);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50(GPIOB,	GPIO_Pin_6);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50(GPIOB,	GPIO_Pin_7);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50(GPIOB,	GPIO_Pin_8);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50(GPIOB,	GPIO_Pin_9);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void ICCARD_Server(void)
{
	u16 Num	=	0;
	systimecount++;
	if(systimecount>=2000)
	{
		systimecount=0;
		runFlag	=	0;
		ICCARD_WriteData(12345678);
		memcpy(TxdBuffer,CmdBuffer,ICCARD_SendLenth);
		API_USART_DMA_Send	(USART2,(u32*)TxdBuffer,ICCARD_SendLenth);														//����DMA���ͳ���
		GPIO_Toggle	(GPIOB,	GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);		//��GPIO��Ӧ�ܽ������ת----V20170605
		GPIO_Toggle	(GPIOC,	GPIO_Pin_12);		//��GPIO��Ӧ�ܽ������ת----V20170605
		GPIO_Toggle	(GPIOD,	GPIO_Pin_2);		//��GPIO��Ӧ�ܽ������ת----V20170605
	}
	if(systimecount	==	1000)
	{
		memset(RevBuffer, 0x00, 22);
		memset(TxdBuffer, 0x00, 30);
	}
	Num	=	USART_ReadBufferIDLE(USART2,(u32*)RevBuffer,(u32*)RxdBuffer);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(Num)
	{
//	API_USART_DMA_Send	(USART2,(u32*)TxdBuffer,22);	//����DMA���ͳ���
//	API_USART_DMA_Send	(USART2,(u32*)ICCARD_CMD_SetDataBlock,Num);	//����DMA���ͳ���
//	API_USART_DMA_Send	(USART2,(u32*)ICCARD_CMD_SetReaderID,sizeof(ICCARD_CMD_SetReaderID));	//����DMA���ͳ���
//	API_USART_DMA_Send	(USART2,(u32*)ICCARD_CMD_GetReaderID,sizeof(ICCARD_CMD_GetReaderID));	//����DMA���ͳ���
		
//		GPIO_Toggle	(GPIOB,	GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);		//��GPIO��Ӧ�ܽ������ת----V20170605
//		GPIO_Toggle	(GPIOC,	GPIO_Pin_12);		//��GPIO��Ӧ�ܽ������ת----V20170605
//		GPIO_Toggle	(GPIOD,	GPIO_Pin_2);		//��GPIO��Ӧ�ܽ������ת----V20170605
	}
//	if(systimecount	==	0)
//	GPIO_Toggle	(GPIOB,	GPIO_Pin_0);		//��GPIO��Ӧ�ܽ������ת----V20170605
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void ICCardReader_initialze(void)
{

}
/*******************************************************************************
* ������			:	ICCARD_WriteData
* ��������		:	д���ݣ�Ĭ��ʹ������1����1,KEY60,����FF FF FF FF FF FF
* ����			: void
* ����ֵ			: void
*******************************************************************************/
u8* ICCARD_WriteData(u32	Number)
{
	u32 Temp	=	Number;
//	Temp	|=	((Number/10000000)&0x0000000F)<<28;
//	Temp	|=	((Number%10000000/1000000)&0x0000000F)<<24;
//	Temp	|=	((Number%1000000/100000)&0x0000000F)<<20;
//	Temp	|=	((Number%100000/10000)&0x0000000F)<<16;
//	Temp	|=	((Number%10000/1000)&0x0000000F)<<12;
//	Temp	|=	((Number%1000/100)&0x0000000F)<<8;
//	Temp	|=	((Number%100/10)&0x0000000F)<<4;
//	Temp	|=	((Number%10)&0x0000000F)<<0;
	
	ICCARD_SendLenth	=	sizeof(ICCARD_CMD_SetDataBlock);	
	memcpy(CmdBuffer,ICCARD_CMD_SetDataBlock,ICCARD_SendLenth);
	
	
	CmdBuffer[11]	=	(Temp>>24)&0xFF;
	CmdBuffer[12]	=	(Temp>>16)&0xFF;
	CmdBuffer[13]	=	(Temp>>8)&0xFF;
	CmdBuffer[14]	=	(Temp>>0)&0xFF;
	return CmdBuffer;
//	ICCARD_SendLenth	=	sizeof(ICCARD_CMD_SetDataBlock);
}
/*******************************************************************************
* ������			:	ICCARD_SetPassWord
* ��������		:	�޸������������������ 0x26
*							DataBlock��1�ֽ�������+1�ֽ���Կģʽ+1�ֽ�keyA��0x60��/keyB��0x61����־+6�ֽھ�����+6�ֽ�������
*
*						����
*							E1 26 01 61 00 FF FF FF FF FF FF FF FF FF FF FF FF 26 63 1E
*							E1 26 01 61 00 FF FF FF FF FF FF 11 11 11 11 11 11 6B 81 1E
* ����			: void
* ����ֵ			: void
*******************************************************************************/
u8* ICCARD_SetPassWord(u8 Sector,u8* NewPassWord)
{
	ICCARD_SendLenth	=	sizeof(ICCARD_CMD_SetPassWord);	
	memcpy(CmdBuffer,ICCARD_CMD_SetPassWord,ICCARD_SendLenth);	
	
	CmdBuffer[2]	=	Sector;		//����
	memcpy(&CmdBuffer[11],NewPassWord,6);
	
	//----CRC16

	return CmdBuffer;
//	ICCARD_SendLenth	=	sizeof(ICCARD_CMD_SetDataBlock);
}
/*******************************************************************************
* ������			:	ICCARD_SetReadArea
* ��������		:	���ö�ͷ������������š�KEYA��0x60��/KEYB(0x61)����������0x44
*							DataBlock������-01�����-02��KEYA ������-FF FF FF FF FF FF
*							Ӧ�𣺰�ִ������Ӧ��
*							������E1 44 01 01 60 FF FF FF FF FF FF 9F 6F 1E
* ����			: void
* ����ֵ			: void
*******************************************************************************/
u8* ICCARD_SetReadArea(u8 Sector,u8 Block,u8* PassWord)
{
	ICCARD_SendLenth	=	sizeof(ICCARD_CMD_SetReaderArea);	
	memcpy(CmdBuffer,ICCARD_CMD_SetReaderArea,ICCARD_SendLenth);	
	
	CmdBuffer[2]	=	Sector;		//����
	CmdBuffer[3]	=	Block;		//����
	memcpy(&CmdBuffer[5],PassWord,6);
	
	//----CRC16

	return CmdBuffer;
//	ICCARD_SendLenth	=	sizeof(ICCARD_CMD_SetDataBlock);
}

#endif