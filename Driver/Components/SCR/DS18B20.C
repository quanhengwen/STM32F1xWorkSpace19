#include "DS18B20.H"	
#include "STM32_SYSTICK.H"

/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void DS18B20_Configuration(OneWrieDef *OneWrie)
{
	OneWrie_Configuration(OneWrie);		//����
}
/*******************************************************************************
* ������			:	function
* ��������		:	��λDallas--��������(����480uS,����48uS),Ȼ���ͷ����ߣ��ȴ��ӻ���������
* ����			: void
* ����ֵ			: ����0:��λ���ɹ�/��IC
							����1:��λ�ɹ�
*******************************************************************************/
unsigned short DS18B20_Read(OneWrieDef *OneWrie)		//��λDallas,���ؽ��
{
	unsigned short	tempr=0;
	unsigned short	tempr1=0;

	OneWrie_Start(OneWrie);
	OneWrie_WriteByte(OneWrie,0xCC); //���� ROM ����
	OneWrie_WriteByte(OneWrie,0x44); //д0X44�����¶�ת��
	
	SysTick_DeleymS(500);				//SysTick��ʱnmS
	
	OneWrie_Start(OneWrie);
	OneWrie_WriteByte(OneWrie,0xCC); //���� ROM ����
	OneWrie_WriteByte(OneWrie,0xBE); //д0X44��RAM����
	tempr		=	OneWrie_ReadByte(OneWrie);
	tempr1	=	OneWrie_ReadByte(OneWrie);
	tempr		=	tempr|(tempr1<<8);
	SysTick_DeleymS(500);				//SysTick��ʱnmS
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
////	OneWrie_SetIn(OneWrie);
//	SysTick_DeleymS(2000);				//SysTick��ʱnmS
////	OneWrie_ReadByte(OneWrie);

//	OneWrie_Start(OneWrie);
//	OneWrie_WriteByte(OneWrie,0x33); //���� ROM ����
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	OneWrie_ReadByte(OneWrie);
//	
//	SysTick_DeleymS(2000);				//SysTick��ʱnmS
	return tempr;
}