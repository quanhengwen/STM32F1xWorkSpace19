#include "EEPROM.H"	

#include	"IIC.H"
	
#include "STM32_GPIO.H"
#include "STM32_SYSTICK.H"


#define	Write_24C02Addr	(unsigned char)0xA0
#define	Read_24C02Addr	(unsigned char)0xA1


//AT24C02 2048λ��256�ֽڣ�8�ֽ�/ҳ,��32ҳ
/*******************************************************************************
* ������			:	function
* ��������		:	��at24c02�е�ָ����ַд������
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void AT24C02_Write(sI2CDef *sI2C,unsigned char Addr,unsigned char Data)
{
	IIC_Start();
	
	IIC_WriteOneByte(Write_24C02Addr);	//��I2C�����豸����8bits������ ,���ȴ���������ݵ����λ��MSB��
	
	I2C_WaitAck(sI2C);
	
	I2C_SendByte(sI2C,Addr);	//��I2C�����豸����8bits������ ,���ȴ���������ݵ����λ��MSB��
	
	I2C_WaitAck(sI2C);
	
	I2C_SendByte(sI2C,Data);	//��I2C�����豸����8bits������ ,���ȴ���������ݵ����λ��MSB��
	
	I2C_WaitAck(sI2C);
	
	I2C_Stop(sI2C);
}
/*******************************************************************************
* ������			:	function
* ��������		:	��at24c02��ָ����ַ�ж���д�������
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char AT24C02_Read(sI2CDef *sI2C,unsigned char Addr)
{
	unsigned char Data	=	0;
	
	I2C_Start(sI2C);
	
	I2C_SendByte(sI2C,Write_24C02Addr);	//��I2C�����豸����8bits������ ,���ȴ���������ݵ����λ��MSB��
	
	I2C_WaitAck(sI2C);
	
	I2C_SendByte(sI2C,Addr);						//���ʹ���ȡ��ַ
	
	I2C_WaitAck(sI2C);
	
	I2C_Start(sI2C);
	
	I2C_SendByte(sI2C,Read_24C02Addr);	//���ʹ���ȡ��ַ
	
	I2C_WaitAck(sI2C);
	
	
	
	I2C_SDASetIn(sI2C);
	
	Data	=	I2C_ReadByte(sI2C);					//��I2C���߶�ȡ8��bits������  ,���ȶ����������ݵ����λ��MSB��
	
	I2C_NAck(sI2C);								//CPU����һ��NACK�ź�(NACK����Ӧ���ź�)
	
	I2C_Stop(sI2C);
	
	return Data;
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
void AT24C02_WritePage(sI2CDef *sI2C,unsigned char StartAddr,unsigned char *Buffer)
{
	unsigned char i	=	0;

	
	I2C_Start(sI2C);
	
	I2C_SendByte(sI2C,Write_24C02Addr);	//��I2C�����豸����8bits������ ,���ȴ���������ݵ����λ��MSB��
	
	if(I2C_WaitAck(sI2C)	==	I2C_NACK)
			return;
	
	I2C_SendByte(sI2C,StartAddr);	//��I2C�����豸����8bits������ ,���ȴ���������ݵ����λ��MSB��
	
	if(I2C_WaitAck(sI2C)	==	I2C_NACK)
			return;
	
	for(i=0;i<8;i++)
	{
		I2C_SendByte(sI2C,Buffer[i]);	//��I2C�����豸����8bits������ ,���ȴ���������ݵ����λ��MSB��
	
		if(I2C_WaitAck(sI2C)	==	I2C_NACK)
			return;
	}
	
	I2C_Stop(sI2C);
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
unsigned char AT24C02_ReadBuffer(sI2CDef *sI2C,unsigned char Addr,unsigned char *Buffer,unsigned char Length)
{
	unsigned char i	=	0;
		
	I2C_Start(sI2C);
	
	I2C_SendByte(sI2C,Write_24C02Addr);	//��I2C�����豸����8bits������ ,���ȴ���������ݵ����λ��MSB��
	
	if(I2C_WaitAck(sI2C)	==	I2C_NACK)
			return 0;
	
	I2C_SendByte(sI2C,Addr);						//���ʹ���ȡ��ַ
	
	if(I2C_WaitAck(sI2C)	==	I2C_NACK)
			return 0;
	
	I2C_Start(sI2C);
	
	I2C_SendByte(sI2C,Read_24C02Addr);	//���ʹ���ȡ��ַ
	
	if(I2C_WaitAck(sI2C)	==	I2C_NACK)
			return 0;	
	
	for(i=0;i<Length;i++)
	{
		I2C_SDASetIn(sI2C);
		
		Buffer[i]	=	I2C_ReadByte(sI2C);					//��I2C���߶�ȡ8��bits������  ,���ȶ����������ݵ����λ��MSB��
	
		I2C_SDASetOut(sI2C);
		
		I2C_Ack(sI2C);											//CPU����һ��ACK�ź�
	}
	I2C_SDASetOut(sI2C);
	
	I2C_NAck(sI2C);											//CPU����һ��NACK�ź�(NACK����Ӧ���ź�)
	
	I2C_Stop(sI2C);

	return 0;
}



