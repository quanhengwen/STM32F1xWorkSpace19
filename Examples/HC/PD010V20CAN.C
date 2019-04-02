/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : PC001V21.c
* Author             : WOW
* Version            : V2.0.1
* Date               : 06/26/2017
* Description        : PC001V21����ư�.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifdef PD010V20CAN			//��·������ư�

#include "PD010V20CAN.H"


RS485Def RS485A;			//���ϲ����߽ӿ�(���ذ�)
RS485Def RS485B;			//���¼����߽ӿ�(���)	


u32 RunTime	=	0;

unsigned short	TextSize	=	0;
CanRxMsg RxMessage;
u8 PowerFlag	=	0;
u8 txBuffer[128]={0};
u8 tem	=	0;
u8 buff[128]={0};
u8 test[8]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
unsigned char arr	=	0;
unsigned char Set	=	0;
unsigned char arrBac	=	0;
//===============================================================================
//����:	PD008V11_Configuration
//����:	����������ó���
//����:
//===============================================================================
void PD010V20CAN_Configuration(void)
{
	SYS_Configuration();							//ϵͳ����ϵͳ����ʱ��72M	

	GPIO_DeInitAll();									//�����е�GPIO�ر�----V20170605

	RS232_Configuration();
	 RS485_Configuration();
	CAN_Configuration();

	SysTick_Configuration(1000);			//ϵͳ���ʱ������72MHz,��λΪuS----���������Զ�ʱɨ��ģʽ,��ʱʱ��ΪSysTickTime	
//	IWDG_Configuration(1000);					//�������Ź�����	Tout-��ʱ��λʱ�䣬��λms	
	PWM_OUT(TIM2,PWM_OUTChannel1,1,900);			//ϵͳ����LED�� Ƶ��1HZ,ռ�ձ�500/1000
}

//===============================================================================
//����:	PD008V11_Server
//����:	��������������
//����:
//===============================================================================
void PD010V20CAN_Server(void)
{
//	CAN_Server();
	RS232_Server();
	RS485_Server();
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
void RS232_Configuration(void)
{
	//-------------------------RS232ͨ��2����
	USART_DMA_ConfigurationNR	(USART1,115200,32);	//USART_DMA����--��ѯ��ʽ�������ж�
	
	//-------------------------RS232ͨ��1����
	USART_DMA_ConfigurationNR	(USART2,115200,32);	//USART_DMA����--��ѯ��ʽ�������ж�
	
	//-------------------------RS232ͨ��2����
	USART_DMA_ConfigurationNR	(USART3,19200,32);	//USART_DMA����--��ѯ��ʽ�������ж�
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
void RS485_Configuration(void)
{
	//-------------------------RS485A����
	RS485A.USARTx	=	UART4;
	RS485A.RS485_CTL_PORT	=	GPIOA;
	RS485A.RS485_CTL_Pin	=	GPIO_Pin_15;
	RS485_DMA_ConfigurationNR	(&RS485A,19200,64);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
	//-------------------------RS485A����
	RS485B.USARTx	=	USART2;
	RS485B.RS485_CTL_PORT	=	GPIOA;
	RS485B.RS485_CTL_Pin	=	GPIO_Pin_1;
	RS485_DMA_ConfigurationNR	(&RS485B,19200,64);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
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
void CAN_Configuration(void)
{
	CAN_Configuration_NR(100000);		//CAN1����---��־λ��ѯ��ʽ�������ж�
	//---------------�˲�����01��ͨ�ù㲥֡
	CAN_FilterInitConfiguration_StdData(0x01,0x00,0x0000);		//CAN�˲�������---��׼����֡ģʽ
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
void RS232_Server(void)
{
	unsigned char len	=	0;
	len	=	USART_ReadBufferIDLE(USART3,buff);
	if(len)
	{
		USART_DMASendList(USART1,buff,len);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
	}
	len	=	USART_ReadBufferIDLE(USART1,buff);
	if(len)
	{
		RS485_DMASend	(&RS485A,buff,len);	//RS485-DMA���ͳ���
		RS485_DMASend	(&RS485B,buff,len);	//RS485-DMA���ͳ���
		USART_DMASendList(USART3,buff,len);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
	}	
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
void RS485_Server(void)
{
	unsigned char len	=	0;
	len	=	RS485_ReadBufferIDLE(&RS485A,buff);
	if(len)
	{
		USART_DMASendList(USART1,buff,len);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
	}
	len	=	RS485_ReadBufferIDLE(&RS485B,buff);
	if(len)
	{
		USART_DMASendList(USART1,buff,len);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
//		USART_DMASendList(USART3,buff,len);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
	}	
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
void CAN_Server(void)
{
	unsigned char length	=	0;
	
	if(CAN_RX_DATA(&RxMessage))
	{
		arr++;
		
		length	=	RxMessage.DLC;

		txBuffer[0]=RxMessage.StdId&0XFF;
		
		memcpy(&txBuffer[1],&RxMessage.Data[0],length);
		
		Set	=	RxMessage.StdId&0XFF;
//		arr++;
		
		USART_DMAPrintfList (USART1,"Arr:%0.2d--Set:%0.2d\r\n",arr,Set);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
		
//		USART_DMASendList   (USART1,txBuffer,length+1);		//����DMA�������ͳ�����������Ѿ����뵽DMA������Buffer��С���������ݴ�������
//		USART_DMASendList   (USART2,txBuffer,length+1);		//����DMA�������ͳ�����������Ѿ����뵽DMA������Buffer��С���������ݴ�������
//		USART_DMASendList   (USART3,txBuffer,length+1);		//����DMA�������ͳ�����������Ѿ����뵽DMA������Buffer��С���������ݴ�������
	}
	else if(RunTime++>8000)
	{
		RunTime	=	0;
		if(arrBac==arr)
		{
			arrBac	=	0;
			arr			=	0;
		}
		else
		{
			arrBac	=	arr;
//			USART_DMAPrintfList (USART1,"Arr:%0.2d\r\n",arr);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
		}
		if(0	==	PowerFlag)
		{
			PowerFlag	=	1;
			memset(txBuffer,0xFF,16);
//			USART_DMAPrintfList (USART1,"POWERFLAG");					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
			USART_DMASendList   (USART1,txBuffer,16);		//����DMA�������ͳ�����������Ѿ����뵽DMA������Buffer��С���������ݴ�������
//			USART_DMAPrintfList (USART1,"%0.2d--%0.2d",arr,Set);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
		}
		
//		USART_DMASendList   (USART1,txBuffer,20);		//����DMA�������ͳ�����������Ѿ����뵽DMA������Buffer��С���������ݴ�������
//		RunTime	=	0;
//		CAN_StdTX_DATA(0x08,8,test);			//CANʹ�ñ�׼֡��������
	}
}


#endif