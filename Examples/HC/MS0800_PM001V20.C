#ifdef MS0800_PM001V20

#include "MS0800_PM001V20.H"

#include	"stdio.h"			//����printf
#include	"string.h"		//����printf
#include	"stdarg.h"		//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"		//malloc��̬�����ڴ�ռ�

/*##############################################################################
################################################################################
# ��Ŀ��		:	MS0800_PC004V10	
# ��������	:	����ҩ�ܵ�Ԫ��
-------------------------------------------------------------------------------	
********************************����˵��***************************************
-------------------------------------------------------------------------------
�� ��PCB�͵�Ԫ���ư�����
�� RS232ͨѶ˵����			FA F5 01 01 BCC8 00 06 01 01 01 00 01 25/FA F5 01 01 27 00 06 01 01 01 00 01 25
	�����ʣ�	9600
	���ںţ�	USART1
	���ݳ��ȣ�13Byte
	ͷ�룺		0XFA,0XF5
	
�� 485ͨѶ˵����485�뵥Ԫ���ư�ͨѶ
	�����ʣ�	9600
	���ںţ�	USART2
	���ƽţ�	PA1
	ͷ�룺		0XFA,0XFB
	˽�е�ַ�����뿪�ص�ַ
	
�� 485ͨѶ˵����485�뵥Ԫ���ư�ͨѶ�����ã�
	�����ʣ�	9600
	���ںţ�	USART3
	���ƽţ�	PB1
	ͷ�룺		0XFA,0XFB
	˽�е�ַ�����뿪�ص�ַ
-------------------------------------------------------------------------------
********************************��������***************************************
-------------------------------------------------------------------------------
1��������Ŀ
2���ϵ��������ȡ��Ԫ��ַ
3��	
4��	
5��	
-------------------------------------------------------------------------------
*****************************ָ��/Э��˵��*************************************
-------------------------------------------------------------------------------

--------------------------------CANЭ��----------------------------------------
1.1��	CANЭ�飺				������ܰ�ͨѶ
			CAN��ID��					0X3FF(StdId=0X3FF;)
			���ȣ�						1	(DLC=1;)
			��ʱ��ͬ����			0XAA;(Data[0]=0XAA;)
			��ȡ/��ʾIDָ�	0X01;(Data[0]=0X01;)
			�Լ�ָ�				0X02;(Data[0]=0X02;)
			Ϩ��ָ�				0X03;(Data[0]=0X02;)
-------------------------------------------------------------------------------		
1.2��	����ܰ��ϱ���ַ���趨������ܰ�ID������Ԫ��������ַ
			CAN��ID��			SW_ID�����뿪�ص�ַ��
			���ȣ�				1	(DLC=1;)
			��ʱ��ͬ����	0XAA;(Data[0]=0XAA;)
		
1.3��	����ܰ�������ݣ������ϼ���Ԫ�����ݣ���ʾ���߹ر�
			CAN��ID��			SW_ID�����뿪�ص�ַ��
			���ȣ�				3	(DLC=3;)
			������ʾ��ʶ��	0X08;(Data[0]=0X08;)
			��ʾ�������ݣ������ʾ999
									��λ8λ��Data[1]
									��λ8λ��Data[2]
									
1.4��	����ܰ�Ӧ�����������յ���ȷ�����ݺ�Ļ�ִ
			CAN��ID��			SW_ID�����뿪�ص�ַ��
			���ȣ�				1	(DLC=1;)
			��ʱ��ͬ����	0XA0;(Data[0]=0XA0;)
--------------------------------485Э��----------------------------------------
2.1��	485Э�飺			�����ذ�ͨѶ
			���ݳ��ȣ�	8Byte
			��ʽ��			HEAD(2)�������ţ���Ԫ�ţ���ֵ��λ����ֵ��λ�����У��
			˵����
			HEAD:				0XFA,0XFB
			���			0X00--�ر���ʾ��0X01--��ʾ��ֵ��0x02--��ȡ�����豸��ַ����ź͵�Ԫ�ţ�,0x03--��ʾ��ԪID��
			��ţ�			��Ԫ��ţ�1~0XFF
			��Ԫ�ţ�		�����ID�ţ�10~77
			��ֵ��λ��	���λ����
			��ֵ��λ��	���λ���ӣ����Ӻ�ȡֵ��Χ��0~999�������ֻ����λ
			���У�飺	Byte0~Byte6У��
-------------------------------------------------------------------------------


********************************************************************************

################################################################################
###############################################################################*/

#define RS485TX	0
#define RS485RX	1
#define PM001V20_RS4851_CTL(n)	if(n==RS485RX){GPIO_ResetBits(GPIOA,GPIO_Pin_1);}else{GPIO_SetBits(GPIOA,GPIO_Pin_1);}
#define PM001V20_RS4852_CTL(n)	if(n==RS485RX){GPIO_ResetBits(GPIOB,GPIO_Pin_1);}else{GPIO_SetBits(GPIOB,GPIO_Pin_1);}
	

#define	PM001V20_BufferSize 32		//DMA1�����С
u8 PM001V20_RXBuffer[PM001V20_BufferSize]={0};
u8 PM001V20_TXBuffer[PM001V20_BufferSize]={0};

u16 SYS_Time=0;
u8 Buzzer_time=0;
//u16 ci_test=0;
//u8 	RxData[8]={0};
//u16 txTemp=0;
//CanRxMsg RxMessage;

//u8 TX_FLag=0;
//u16 TX_Time=0;

//u8 ID_ARR[8][8]={0,0};
//u8 ON_line[8][8]={0,0};

//u8	TX_ID=0;
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void MS0800_PM001V20_Configuration(void)
{
	CAN_Configuration();	//CAN����
	
	CAN_FilterInitConfiguration(0,0X000,0X000);		//CAN�˲�������
	
	USART_DMA_Configuration(USART1,9600,1,1,(u32*)PM001V20_RXBuffer,(u32*)PM001V20_TXBuffer,PM001V20_BufferSize);	//USART1_DMA���� RS232
		
	USART_DMA_Configuration(USART2,9600,1,1,(u32*)PM001V20_TXBuffer,(u32*)PM001V20_RXBuffer,PM001V20_BufferSize);	//USART2_DMA���� RS485

//	USART_DMA_Configuration(USART3,9600,1,1,(u32*)PM001V20_Buffer,(u32*)PM001V20_Buffer,PM001V20_BufferSize);	//USART2_DMA���� RS485	
	
	GPIO_Configuration(GPIOA,GPIO_Pin_1,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);			//USART2_CTL
	GPIO_Configuration(GPIOB,GPIO_Pin_1,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);			//USART3_CTL
	
	GPIO_Configuration(GPIOC,GPIO_Pin_1,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);			//GPIO����--BUZZER
	GPIO_Configuration(GPIOC,GPIO_Pin_0,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);			//GPIO����--WORK_LED
	GPIO_ResetBits(GPIOC,GPIO_Pin_1);
	
	PM001V20_RS4851_CTL(RS485TX);
	PM001V20_RS4851_CTL(RS485RX);

}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void MS0800_PM001V20_Server(void)
{
	u8 status=0;
	SYS_Time++;	
	if(SYS_Time>=30000)
		SYS_Time=0;
	
	status=MS0800_PM001V20_TR();
	if(status)
		SYS_Time=0;
	
	if(SYS_Time==20000)
	{
		Time_Sync();
	}

	PM001V20_BUZZER(SYS_Time);					//BUZZER
	PM001V20_WORK_LED(SYS_Time);				//WORK_LED	
	MS0800_PM001V20_PROCESS(SYS_Time);	//��Ϣ����
	//***********��ʱת��485�������ŷ�ʽ
	/***********************
	if(TX_FLag==1)
	{
		TX_Time--;
		if(TX_Time==0)
		{
			TX_FLag=0;
//			TX_Time=0;
			PM001V20_RS4851_CTL(RS485RX);
		}
	}
	***********************/
	//***********����״̬������485��Ϊ���շ�ʽ
//	if(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET)
//		PM001V20_RS4851_CTL(RS485RX);

	
//	if(USART_RX_FlagClear(USART2))							//�������DMA����ȫ���жϱ�־
//	{
//		PM001V20_RS4851_CTL(RS485TX);
//		USART_DMAPrintf(USART1,"RS485�շ�����\n");		//�Զ���printf����DMA���ͳ���
//	}
//	else if(USART_TX_DMAFlagClear(USART1))
//	{	
//		memset(PM001V20_Buffer,0,PM001V20_BufferSize);			//��ʼ������		
//		PM001V20_RS4851_CTL(RS485RX);
////		GPIO_ResetBits(GPIOA,GPIO_Pin_1);
////		USART_DMAPrintf(USART1,"%d",18);		//�Զ���printf����DMA���ͳ���
//	}
	
	

}
/*******************************************************************************
* ������		:
* ��������	: 
* ����			: ��
* ���			: ��
* ����			: ��
*******************************************************************************/
void MS0800_PM001V20_PROCESS(u16 time)
{
}
/*******************************************************************************
* ������		:
* ��������	: 
* ����			: ��
* ���			: ��
* ����			: ��
*******************************************************************************/
u8 MS0800_PM001V20_TR(void)				//ͨѶ�ӿ�
{
//***********����״̬������485��Ϊ���շ�ʽ
	u16 num1=0,num2=0;
	
	num1=USART_RX_FlagClear(USART1);							//�������DMA����ȫ���жϱ�־
	num2=USART_RX_FlagClear(USART2);							//�������DMA����ȫ���жϱ�־
	//1)**********���ݴ���2���յ�����
	if(num1)			//����1���յ���λ������ͨ������2ת������Ԫ��
	{	
		PM001V20_RS4851_CTL(RS485TX);
		PM001V20_delay(1000);
		USART_DMASend(USART2,PM001V20_RXBuffer,(u32)num1);
		return 1;
	}
	else if(num2)		//����2 ���յ�����ͨ������1ת������λ��
	{
		USART_DMASend(USART1,PM001V20_TXBuffer,(u32)num2);
		return 1;
	}
	//2)**********DMA������ɺ����BUFFER
	else if(USART_TX_DMAFlagClear(USART1))
	{
//		PM001V20_delay(8000);
		memset(PM001V20_TXBuffer,0,PM001V20_BufferSize);			//�������
		return 1;
	}
	else if(USART_TX_DMAFlagClear(USART2))
	{
//		PM001V20_delay(8000);
		memset(PM001V20_RXBuffer,0,PM001V20_BufferSize);			//�������
		return 1;
	}
	//3)**********���ݷ�����ɺ�485���ƽ�����תΪ����ģʽ���������־������
	else if(USART_GetFlagStatus(USART2,USART_FLAG_TC)==SET)
	{
		PM001V20_RS4851_CTL(RS485RX);
		USART_ClearFlag(USART2,USART_FLAG_TC);
//		memset(PM001V20_RXBuffer,0,PM001V20_BufferSize);			//��ʼ������
//		memset(PM001V20_RXBuffer,0,PM001V20_BufferSize);			//��ʼ������
		return 0;
	}
	//4)**********USART1��ִ�����Ժ������־���������
	else if(USART_GetFlagStatus(USART1,USART_FLAG_TC)==SET)
	{
		USART_ClearFlag(USART1,USART_FLAG_TC);
//		memset(PM001V20_TXBuffer,0,PM001V20_BufferSize);			//��ʼ������
//		memset(PM001V20_RXBuffer,0,PM001V20_BufferSize);			//��ʼ������
		return 0;
	}
	else
	{
		return 0;
	}
/****************��ʱת��485�������ŷ�ʽ	
	u16 num1=0,num2=0;
	
	num1=USART_RX_FlagClear(USART1);							//�������DMA����ȫ���жϱ�־
	num2=USART_RX_FlagClear(USART2);							//�������DMA����ȫ���жϱ�־
	//1)**********���ݴ���2���յ�����
	if(num1)
	{	
		PM001V20_RS4851_CTL(RS485TX);
		PM001V20_delay(8000);
		TX_FLag=1;
		TX_Time=num1+5;
		USART_DMASend(USART2,(u32*)PM001V20_Buffer,(u32)num1);
		return 1;
	}
	else if(num2)
	{
		PM001V20_RS4851_CTL(RS485TX);
		PM001V20_delay(8000);
		TX_FLag=1;
		TX_Time=num2+5;
		USART_DMASend(USART1,(u32*)PM001V20_Buffer,(u32)num2);
//		USART_DMAPrintf(USART1,"RS485�շ�����\n");		//�Զ���printf����DMA���ͳ���
		return 1;
	}
	//2)**********���ݴ���2DMA��������ж�
	else if(USART_TX_DMAFlagClear(USART1))
	{
		PM001V20_delay(5000);
//		PM001V20_RS4851_CTL(RS485RX);
		memset(PM001V20_Buffer,0,PM001V20_BufferSize);			//��ʼ������
//		PM001V20_RS4851_CTL(RS485RX);
		return 1;
	}
	else if(USART_TX_DMAFlagClear(USART2))
	{
		PM001V20_delay(5000);
//		PM001V20_RS4851_CTL(RS485RX);
		memset(PM001V20_Buffer,0,PM001V20_BufferSize);			//��ʼ������
//		PM001V20_RS4851_CTL(RS485RX);
		return 1;
	}
	else
	{
		return 0;
	}
*************************************/
}

/*******************************************************************************
* ������		:
* ��������	: 
* ����			: ��
* ���			: ��
* ����			: ��
*******************************************************************************/
void PM001V20_WORK_LED(u16 time)				//WORK_LED
{
	if(time%1000<500)
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_0);				//WORK_LED		
	}
	else
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_0);			//WORK_LED
	}
}
/*******************************************************************************
* ������		:
* ��������	: 
* ����			: ��
* ���			: ��
* ����			: ��
*******************************************************************************/
void PM001V20_BUZZER(u16 time)				//WORK_LED
{
	if(Buzzer_time<6)
	{
		if(time%1000<500)
		{
			GPIO_SetBits(GPIOC,GPIO_Pin_1);				//Buzzer_ON		
		}
		else
		{
			GPIO_ResetBits(GPIOC,GPIO_Pin_1);			//Buzzer_OFF
		}
		if(time%500==0)
		{
			Buzzer_time++;
		}
	}
	else
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_1);			//Buzzer_OFF
	}
}
/*******************************************************************************
* ������		:
* ��������	: 
* ����			: ��
* ���			: ��
* ����			: ��
*******************************************************************************/
void PM001V20_delay(u16 time)
{
	while(time--);
}
/*******************************************************************************
* ������		:
* ��������	: 
* ����			: ��
* ���			: ��
* ����			: ��
*******************************************************************************/
void Time_Sync(void)
{
	PM001V20_RS4851_CTL(RS485TX);
	PM001V20_delay(500);
	//��������
	PM001V20_RXBuffer[0]=0xFA;
	PM001V20_RXBuffer[1]=0xF5;
	PM001V20_RXBuffer[2]=~PM001V20_RXBuffer[0];
	PM001V20_RXBuffer[3]=~PM001V20_RXBuffer[1];
	PM001V20_RXBuffer[5]=0x06;	
	USART_DMASend(USART2,PM001V20_RXBuffer,13);
}


#endif