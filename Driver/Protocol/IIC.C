#include	"IIC.H"



#include	"stdio.h"


#include "STM32_SYSTICK.H"

#define IICTime	1

I2CHWdef* softiic	=	NULL;
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void IIC_Initialize(I2CHWdef* pInfo)
{
	softiic	=	pInfo;
	softiic->Start=IIC_Start;
	softiic->Stop=IIC_Stop;
	softiic->Ack=IIC_Ack;
	softiic->NAck=IIC_NAck;
	softiic->WaitAck=IIC_WaitAck;
	
	softiic->WriteOneByte=IIC_WriteOneByte;
	softiic->WriteBuffer=IIC_WriteBuffer;
	softiic->ReadOneByte=IIC_ReadOneByte;
	softiic->ReadBuffer=IIC_ReadBuffer;
}
/*******************************************************************************
*	������			:IIC_Start
*	��������		:IIC����,��SCL���Ǹߵ�ƽʱSDA�ߴӸߵ�ƽ��͵�ƽ�л���������ʾ��ʼ����
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void IIC_Start(void)
{
	softiic->SDAWriteEn();	

	softiic->SDAHigh();
	IIC_Delayus(IICTime);
	
	softiic->SCLHigh();
	IIC_Delayus(IICTime);	
	
	softiic->SDALow();	
	IIC_Delayus(IICTime);
	
	softiic->SCLLow();
}
/*******************************************************************************
*	������			:	IIC_Stop
*	��������		:	IICֹͣ,��SCL�Ǹߵ�ƽʱSDA���ɵ͵�ƽ��ߵ�ƽ�л���ʾֹͣ����
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void IIC_Stop(void)
{
	softiic->SDAWriteEn();
	
	softiic->SDALow();	
	IIC_Delayus(IICTime);
	
	softiic->SCLHigh();
	IIC_Delayus(IICTime);
	
	softiic->SDAHigh();
	IIC_Delayus(IICTime);
	
	softiic->SCLLow();
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
void IIC_Ack(void)
{
	softiic->SDAWriteEn();
	
	softiic->SCLLow();
	IIC_Delayus(IICTime);
	softiic->SDALow();
	IIC_Delayus(IICTime);
	softiic->SCLHigh();
	IIC_Delayus(IICTime);
	softiic->SCLLow();
	IIC_Delayus(IICTime);
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
void IIC_NAck(void)
{
	softiic->SDAWriteEn();
	
	softiic->SCLLow();
	IIC_Delayus(IICTime);
	softiic->SDAHigh();
	IIC_Delayus(IICTime);
	softiic->SCLHigh();
	IIC_Delayus(IICTime);
	softiic->SCLLow();
	IIC_Delayus(IICTime);
}
/*******************************************************************************
* ������			:	IIC_WaitAck
* ��������		:	�ȴ�Ӧ�� 
* ����			: void
* ����ֵ			: 0-ACK��1-NACK
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char IIC_WaitAck(void)
{
	unsigned	char ack=0;
	
	softiic->SDAReadEn();
	
	softiic->SCLLow();
	IIC_Delayus(IICTime);
	
	softiic->SCLHigh();
	IIC_Delayus(IICTime);
	
	ack	=	softiic->SDAStd();
	
	softiic->SCLLow();
	IIC_Delayus(IICTime);
	
	return	ack;
}
/*******************************************************************************
* ������			:	IIC_WriteOneByte
* ��������		:	�ȴ����λ
							��SCL���ָߵ�ƽ�ڼ䣬SDA�ϵĵ�ƽ���뱣���ȶ����͵�ƽΪ����0���ߵ�ƽΪ����1��
							ֻ����SCLΪ�͵�ƽ�ڼ䣬������SDA�ϵĵ�ƽ�ı�״̬���߼�0�ĵ�ƽΪ�͵�ѹ��
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char IIC_WriteOneByte(unsigned	char data)
{
	unsigned char i	=	0;
	softiic->SDAWriteEn();
	
	softiic->SCLLow();
	IIC_Delayus(IICTime);
	
	//-----------------����ʱ�ӿ�ʼ���ݴ���
	for(i=0;i<8;i++)
	{		
		if(data&0x80)
		{
			softiic->SDAHigh();
		}
		else
		{
			softiic->SDALow();
		}
		IIC_Delayus(IICTime);
		//----------------����һ��ʱ���ź�
		softiic->SCLHigh();
		IIC_Delayus(IICTime);	
		softiic->SCLLow();
		IIC_Delayus(IICTime);
		data<<=1;
	}	
	return	0;
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
unsigned short IIC_WriteBuffer(unsigned char* pbuffer,unsigned	short length)
{
	unsigned char i	=	0;
	unsigned char data	=	0;
	unsigned short len	=	0;

	softiic->SDAWriteEn();
	//-----------------����ʱ�ӿ�ʼ���ݴ���
	softiic->SCLLow();
	IIC_Delayus(IICTime);
	
	for(len=0;len<length;len++)
	{		
		data	=	pbuffer[len];
		for(i=0;i<8;i++)
		{
			if(data&0x80)
			{
				softiic->SDAHigh();
			}
			else
			{
				softiic->SDALow();
			}
			IIC_Delayus(IICTime);
			//----------------����һ��ʱ���ź�
			softiic->SCLHigh();
			IIC_Delayus(IICTime);
			softiic->SCLLow();
			IIC_Delayus(IICTime);
			data<<=1;
		}
	}	
	return	0;
}
/*******************************************************************************
* ������			:	IIC_ReadOneByte
* ��������		:	SCL�����شӻ��������
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char IIC_ReadOneByte(void)
{
	unsigned char i	=	0;
	unsigned char receive	=	0;
	
	softiic->SDAReadEn();
	softiic->SCLLow();
	IIC_Delayus(IICTime);
	
	for(i=0;i<8;i++)
	{
		receive<<=1;
		if(softiic->SDAStd())	//�ߵ�ƽ
			receive|=0x01;
		else
			receive&=0xFE;		
		//----------------����һ��ʱ���ź�
		softiic->SCLHigh();
		IIC_Delayus(IICTime);
		softiic->SCLLow();
		IIC_Delayus(IICTime);
	}
	return	receive;
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
unsigned short IIC_ReadBuffer(unsigned	char* pbuffer,unsigned short readlength)
{
	unsigned char i	=	0;
	unsigned char receive	=	0;
	unsigned short len	=	0;
	
	softiic->SDAReadEn();
	softiic->SCLLow();
	IIC_Delayus(IICTime);
	
	for(len=0;len<readlength;len++)
	{
		for(i=0;i<8;i++)
		{
			if(softiic->SDAStd)	//�ߵ�ƽ
				receive|=0x01;
			else
				receive&=0xFE;			
			receive<<=1;
			//----------------����һ��ʱ���ź�
			softiic->SCLHigh();
			IIC_Delayus(IICTime);
			softiic->SCLLow();
			IIC_Delayus(IICTime);
		}
		pbuffer[len]=receive;
	}
	return	len;
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
void IIC_Delayus(unsigned	short Time)
{
	SysTick_DeleyuS(Time);				//SysTick��ʱnmS
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
void IIC_Delayms(unsigned	short Time)
{
	SysTick_DeleymS(Time);				//SysTick��ʱnmS
}
