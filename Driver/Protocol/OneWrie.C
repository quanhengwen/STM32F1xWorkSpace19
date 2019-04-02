#include "OneWrie.H"	


#include "STM32_GPIO.H"
#include "STM32_SYSTICK.H"

/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void OneWrie_Configuration(OneWrieDef *OneWrie)		//����
{
	//=====================================Data��
	GPIO_Configuration_OPP50	(OneWrie->Data_Port,OneWrie->Data_Pin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
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
void OneWrie_Server(OneWrieDef *OneWrie)
{
	
}
/*******************************************************************************
* ������			:	OneWrie_SetOut
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char OneWrie_SetOut(OneWrieDef *OneWrie)
{
	GPIO_Configuration_OPP50	(OneWrie->Data_Port,OneWrie->Data_Pin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	return 0;
}
/*******************************************************************************
* ������			:	OneWrie_SetOut
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char OneWrie_SetIn(OneWrieDef *OneWrie)
{
	GPIO_Configuration_IPU(OneWrie->Data_Port,OneWrie->Data_Pin);		//����ģʽ
	return 0;
}
/*******************************************************************************
* ������			:	OneWrie_SetHigh
* ��������		:	��������
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char OneWrie_SetHigh(OneWrieDef *OneWrie)
{
	GPIO_SetBits(OneWrie->Data_Port,OneWrie->Data_Pin);							//����DQ
	return 0;
}
/*******************************************************************************
* ������			:	OneWrie_SetLow
* ��������		:	��������
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char OneWrie_SetLow(OneWrieDef *OneWrie)
{
	GPIO_ResetBits(OneWrie->Data_Port,OneWrie->Data_Pin);						//����DQ
	return 0;
}
/*******************************************************************************
* ������			:	OneWrie_Read
* ��������		:	������
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char OneWrie_Read(OneWrieDef *OneWrie)
{
//	return ((OneWrie->Data_Port->IDR)	& (OneWrie->Data_Pin));
	return GPIO_ReadInputDataBit(OneWrie->Data_Port,OneWrie->Data_Pin);
}
/*******************************************************************************
* ������			:	OneWrie_ReadAck
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char OneWrie_ReadAck(OneWrieDef *OneWrie)
{
	return ((OneWrie->Data_Port->IDR)	& (OneWrie->Data_Pin));
}
/*******************************************************************************
* ������			:	OneWrie_ReadBit
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char OneWrie_ReadBit(OneWrieDef *OneWrie)
{
	unsigned char data;
	
	OneWrie_SetOut(OneWrie);			//��Ϊ���ģʽ
	OneWrie_SetLow(OneWrie); 			//��������
	SysTick_DeleyuS(5);						//����1usʱ��
	OneWrie_SetIn(OneWrie);;			//����Ϊ����ģʽ
	SysTick_DeleyuS(12);					//�ȴ��ӻ���Ӧ
	if(OneWrie_Read(OneWrie))			//��ȡ����״̬
		data = 1;
	else
		data = 0;	 
	SysTick_DeleyuS(50);	
	return data;
}
/*******************************************************************************
* ������			:	OneWrie_WriteBit
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char OneWrie_WriteBit(OneWrieDef *OneWrie,unsigned char bit)
{
	//д��϶�����֣�����д0��ʱ��϶��д1��ʱ��϶��
	//�����������ͺ���15 ~ 60 us��ʱ�䴰���ڶ������߽��в�����
	//���������Ϊ�͵�ƽ������д0�����������Ϊ�ߵ�ƽ������д1��
	//����Ҫ����һ��д1ʱ��϶���ͱ�������������ͣ���дʱ��϶��ʼ���15 us���������������ߡ�
	//����Ҫ����һ��д0ʱ��϶���ͱ�������������Ͳ�����60 us��
	OneWrie_SetLow(OneWrie); 	//��������
	SysTick_DeleyuS(2);			//15 ~ 60 us��ʱ�䴰���ڶ������߽��в���
	// =============Write 1
	if (bit&0x01)		
	{
		OneWrie_SetHigh(OneWrie);		//��������
		SysTick_DeleyuS(60);				//�ȴ��ӻ��������
	}
	// =============Write 0
	else						
	{
		SysTick_DeleyuS(60);				//����60us���ȴ��ӻ�����
		OneWrie_SetHigh(OneWrie);		//��������
		SysTick_DeleyuS(2);
	}
	return 0;
}
/*******************************************************************************
* ������			:	OneWrie_Start
* ��������		:	��ʼ�� 
							��ʼ������ = ��λ���� + �ӻ�Ӧ�����塣
							����ͨ�����͵�����480 ~ 960 us������λ���壬Ȼ���ͷ����ߣ��������ģʽ��
							�����ͷ�����ʱ��������͵�ƽ����Ϊ�ߵ�ƽ�������أ�������������⵽������֮����ʱ15 ~ 60 us��������������������60 ~ 240 us������Ӧ�����塣
							�������յ��ӻ���Ӧ������˵��������������������ʼ��������ɡ�
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char OneWrie_Start(OneWrieDef *OneWrie)
{
	uint16_t retry=0;
	GPIO_Configuration_OPP50(OneWrie->Data_Port,OneWrie->Data_Pin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_ResetBits(OneWrie->Data_Port,OneWrie->Data_Pin);								//����DQ
	SysTick_DeleyuS(750);		//����750us������480uS)
	//----------------------�����Ӧ����ʱ����Ҫ����480us
	//1�ͷ�����
	//2�ȴ�15us~60uS
	//3�ӻ���������60uS~240uS
	//4�ӻ��ͷ�/��������
	GPIO_Configuration_IPU(OneWrie->Data_Port,OneWrie->Data_Pin);		//����ģʽ
	SysTick_DeleyuS(15);		//15US---�����Ӧ��Ҫ��15uS��
	while(OneWrie_ReadAck(OneWrie)	&& (retry < 500))			//��Ӧ���ʱ�䲻����240uS
	{
		retry++;
		SysTick_DeleyuS(1);
	}
	if(retry >= 500)	//��ʱ
		return 1;
	else
		retry=0;
	//������ʱ--Ϊ�������ܵļ��ʱ��480uS
	SysTick_DeleyuS(480);		//�ܵļ����Ӧʱ����Ҫ����480uS
	return 0;
}
/*******************************************************************************
* ������			:	function
* ��������		:	дһ���ֽڵ�Dallas---�ӵ�λ��ʼд��
* ����			: dat��Ҫд����ֽ�,
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char OneWrie_WriteByte(OneWrieDef *OneWrie,unsigned char dat)
{             
	unsigned char j;
	unsigned char bit;
	
	OneWrie_SetOut(OneWrie);							//SET PG11 OUTPUT;
	
	for (j=1; j<=8; j++)
	{
		bit = dat&0x01;
		dat = dat>>1;
		OneWrie_WriteBit(OneWrie,bit);		//1-wire һλ��1bit��д����-д0&д1
	}
	return 0;
}
/*******************************************************************************
* ������			:	OneWrie_ReadByte
* ��������		:	��Dallas��һ���ֽ�---�ӵ�λ��ʼ��ȡ
* ����			:
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char OneWrie_ReadByte(OneWrieDef *OneWrie)
{             
	unsigned char j;
	unsigned char bit;
	unsigned char dat	=	0;
	
	for (j=1; j<=8; j++)
	{
		dat>>=1;
		if(OneWrie_ReadBit(OneWrie))
		{
			dat|=0x80;
		}
	}
	return dat;
}