#ifdef PC018V10

#include "PC018V10.H"
#include "HC_PHY.H"

#include "DS2401.h"	


#include "stm32f10x_exti.h"


#include "STM32_USART.H"
#include "STM32_SPI.H"
#include "STM32_PWM.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_WDG.H"
#include "STM32_SYSTICK.H"
#include "STM32F10x_BitBand.H"


#include	"stdio.h"			//����printf
#include	"string.h"		//����printf
#include	"stdarg.h"		//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"		//malloc��̬�����ڴ�ռ�


#define	Board_SerialNum	0x0000		//PCB���

#define	RS485_BaudRate		256000
#define	RS485_BufferSize	32

Borad_InfoDef	PC018V10_Info;
RS485_TypeDef	RS485_Info;

SPIDef	pSPI;

u8 RS485Txd[RS485_BufferSize]	=	{0};
u8 RS485Rxd[RS485_BufferSize]	=	{0};
u8 RS485Rev[RS485_BufferSize]	=	{0};

u8 ch[120]="USART_BASIC_Configuration(USART_TypeDef* USARTx,u32 USART_BaudRate,u8 NVICPreemptionPriority,u8 NVIC_SubPriority)\n";
u8 ch2[17]={0xC0,0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71};

u8 Seg7Buffer[16]={0x00};		//���������оƬ���ݻ������������������ʾֵ

u32	SYSTIME	=	0;
u16	DisplayNum	=	0;			//����ܸ�����ֵ
u16	DisplayBac	=	0;			//������ʾ��ֵ
u16	DisplayTime	=	0;			//�����ظ�����ʾʱ��----��ֹ���ݴ���ʧ��



void SPI_Configuration(void);			//SPI����
void RS485_Configuration(void);			//RS485����
	

void RS485_Server(void);						//RS485�շ�����
void Seg7_Server(void);							//�������ʾ����

void WriteNumSeg7(unsigned short Num);		//�������д������
void WriteStatus(char StatusCode);				//�������д��״̬
void Seg7_Test(void);		//�������ʾ����

/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
*******************************************************************************/
u8 Seg7_Code[]=
{
	0x3F,			//A0:0
	0x06,			//A1:1
	0x5B,			//A2:2
	0x4F,			//A3:3
	0x66,			//A4:4
	0x6D,			//A5:5
	0x7D,			//A6:6
	0x07,			//A7:7
	0x7F,			//A8:8
	0x6F,			//A9:9
	0x77,			//A10:A
	0x7C,			//A11:B
	0x39,			//A12:C
	0x5E,			//A13:D
	0x79,			//A14:E
	0x71,			//A15:F
	0x50,			//A16:r
	0x5C,			//A17:o
	0x54,			//A18:n
	0x58,			//A19:c
	0x1C,			//A20:u
	0x38,			//A21:L
	0x73,			//A22:P
	0x40,			//A23:-
	0x30			//A24:I
};

//u8 itf=0;

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PC018V10_Configuration(void)
{
	SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H
	RCC_Configuration_HSI();			//ʹ���ڲ����پ���
	
	GPIO_DeInitAll();							//�����е�GPIO�ر�----V20170605
	
	SysTick_Configuration(1000);	//ϵͳ���ʱ������72MHz,��λΪuS
	
	PWM_OUT(TIM2,PWM_OUTChannel1,1,500);
	


//	STM32_SPI_ConfigurationNR(SPI2);	//SPI����---������ܰ巢������
	SPI_Configuration();			//SPI����
	RS485_Configuration();						//RS485����


	Dallas_Init();
//	Dallas_GetID(SegArr);
	
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PC018V10_Server(void)
{
	SYSTIME++;
	if(SYSTIME>=1)
	{
		SYSTIME	=	0;
		Seg7_Test();		//�������ʾ����
//		DisplayNum++;
//		if(DisplayNum>9999)
//			DisplayNum	=0;
//		WriteNumSeg7(DisplayNum);		//�������д������


		
//		RS485_DMAPrintf	(&RS485_Info,"�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����\t\n");					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
//		Dallas_GetID(SegArr);
	}
//	Seg7_Server();						//�������ʾ����
	RS485_Server();						//RS485�շ�����

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
void SPI_Configuration(void)
{
	pSPI.Port.SPIx	=	SPI2;
}
/*******************************************************************************
* ������			:	RS485_Configuration
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: 
*******************************************************************************/
void RS485_Configuration(void)			//RS485����
{
	RS485_Info.RS485_CTL_PORT	=	GPIOA;
	RS485_Info.RS485_CTL_Pin	=	GPIO_Pin_8;
	RS485_Info.USARTx	=	USART1;
	RS485_DMA_ConfigurationNR	(&RS485_Info,RS485_BaudRate,RS485_BufferSize);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: 
*******************************************************************************/
void RS485_Server(void)							//RS485�շ�����
{
	u16 Num	=	0;
	Num	=	RS485_ReadBufferIDLE(&RS485_Info,RS485Rev);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(Num)
	{
		DisplayNum	=	RS485Rev[0];
		DisplayNum	=	(DisplayNum<<8)+RS485Rev[1];
		memcpy(RS485Txd,RS485Rev,Num);									//��������
		RS485_DMASend	(&RS485_Info,RS485Rev,Num);	//RS485-DMA���ͳ���
	}
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: 
*******************************************************************************/
void Seg7_Server(void)		//�������ʾ����
{
	//===============================��������и���---��������
	if(DisplayBac	!=	DisplayNum)		
	{
		DisplayBac	=	DisplayNum;
		WriteNumSeg7(DisplayBac);		//�������д������
		DisplayTime	=	0;
	}
	
	//===============================ÿ1S����д��һ������
	if(DisplayTime++	>=	1000)			
	{
		DisplayTime	=	0;
		WriteNumSeg7(DisplayBac);		//�������д������
		DisplayTime	=	0;
	}
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: 
*******************************************************************************/
void Seg7_Test(void)		//�������ʾ����
{
	DisplayNum++;
	if(DisplayNum>9999)
		DisplayNum	=0;
	WriteNumSeg7(DisplayNum);		//�������д������
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void WriteNumSeg7(u16 Num)		//�������д������
{
	SPI_ReadWriteByteSPI(SPI2,0x8F);		//���� 0x80���أ�<0x88<0x89<0x8A<0x8B<0x8C<0x8D<0x8E<0x8F
	SPI_ReadWriteByteSPI(SPI2,0x40);		//��ַģʽ  0x40-����;0x44-�̶���ַ
	Seg7Buffer[0]	=	0xC0;												//��ʼ��ַ
	if(Num/1000	!=	0)
	{
		Seg7Buffer[1]=Seg7_Code[Num/1000];							//
		Seg7Buffer[3]=Seg7_Code[Num%1000/100];
		Seg7Buffer[5]=Seg7_Code[Num%100/10];
	}
	else if(Num/100	!= 0)
	{
		Seg7Buffer[1]=0x00;							//
		Seg7Buffer[3]=Seg7_Code[Num%1000/100];
		Seg7Buffer[5]=Seg7_Code[Num%100/10];
	}
	else if(Num/10	!= 0)
	{
		Seg7Buffer[1]=0x00;							//
		Seg7Buffer[3]=0x00;
		Seg7Buffer[5]=Seg7_Code[Num%100/10];
	}
	else
	{
		Seg7Buffer[1]=0x00;							//
		Seg7Buffer[3]=0x00;
		Seg7Buffer[5]=0x00;
	}
	Seg7Buffer[7]=Seg7_Code[Num%10];	
	SPI_WriteBufferSPI(SPI2,8,Seg7Buffer);			//��������
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void WriteStatus(char StatusCode)		//�������д��״̬
{
	
	if(StatusCode	==	0)
	{
		return;
	}
	//״̬1-------------"P-on"	//�ϵ磬δ��ȡ��ַ
	else if(StatusCode	==	1)	
	{
		SPI_ReadWriteByteSPI(SPI2,0x8F);		//����
		SPI_ReadWriteByteSPI(SPI2,0x40);		//��ַ����
		Seg7Buffer[1]=Seg7_Code[22];									//
		Seg7Buffer[3]=Seg7_Code[23];
		Seg7Buffer[5]=Seg7_Code[17];
		Seg7Buffer[7]=Seg7_Code[18];
		SPI_WriteBufferSPI(SPI2,8,Seg7Buffer);			//��������
	}
	//״̬2-------------"ID-n"	//��ʾ�˴��ں�
	else if(StatusCode	==	2)	
	{
		SPI_ReadWriteByteSPI(SPI2,0x8F);		//����
		SPI_ReadWriteByteSPI(SPI2,0x40);		//��ַ����
		Seg7Buffer[1]=Seg7_Code[24];							//
		Seg7Buffer[3]=Seg7_Code[13];
		Seg7Buffer[5]=Seg7_Code[23];
		Seg7Buffer[7]=Seg7_Code[6];
		SPI_WriteBufferSPI(SPI2,8,Seg7Buffer);			//��������
	}
	//״̬3-------------"uP--"	//������
	else if(StatusCode	==	3)	
	{
		SPI_ReadWriteByteSPI(SPI2,0x8F);		//����
		SPI_ReadWriteByteSPI(SPI2,0x40);		//��ַ����
		Seg7Buffer[1]=Seg7_Code[20];							//
		Seg7Buffer[3]=Seg7_Code[22];
		Seg7Buffer[5]=Seg7_Code[23];
		Seg7Buffer[7]=Seg7_Code[23];
		SPI_WriteBufferSPI(SPI2,8,Seg7Buffer);			//��������
	}
	else
	{
		return;
	}
}



#endif
