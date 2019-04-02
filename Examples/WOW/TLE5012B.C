#ifdef TLE5012B

#include "TLE5012B.H"

#include "string.h"				//�����ڴ��������ͷ�ļ�
//#include "stm32f10x_dma.h"



#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_PWM.H"
#include "STM32_USART.H"
#include "STM32_SPI.H"
#include "STM32_MCO.H"

#include "TM1616.H"

#include 	"CRC.H"


RS485Def stRS485BS;   //usart2,pB11 PB10
TM1616Def	Seg1,Seg2;
SPIDef	TLE5012;


#define ussize  128     //���ڻ����С
unsigned char u1txbuffer[ussize];
unsigned char u1rxbuffer[ussize];
unsigned char u2txbuffer[ussize];
unsigned char u2rxbuffer[ussize];


char	cwflg	=	0;
char	runflg	=	0;
unsigned	short organgle	=	0;
signed	short cmpgangle	=	0;
unsigned	short angle1	=	0;
unsigned	short angle2	=	0;
unsigned	short anglecount	=	0;
unsigned	short	Speed	=	0;


unsigned	short time	=	0;
unsigned	short dstime	=	0;
unsigned  short seril	=	0;
unsigned	short readdelay	=	0;
unsigned	short timedelay=0;
//=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>
//->������		:	
//->��������	:	 
//->����		:	
//->���		:
//->���� 		:
//<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=
void TLE5012B_Configuration(void)
{	
  RCC_ClocksTypeDef RCC_ClocksStatus;							//ʱ��״̬---ʱ��ֵ
	SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H	

  COMM_Configuration();  
  
	Seg_Configuration();
	
	MCO_Initialize();
	
	//hw_configuration();
	SPI5012B_Init();
	
	GPIO_Configuration_OPP50(GPIOB,GPIO_Pin_6);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50(GPIOB,GPIO_Pin_7);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	
	GPIO_Configuration_IPU(CWtargePort,CWtargePin);			//��GPIO��Ӧ�ܽ�����Ϊ��������ģʽ----V20170605
	GPIO_Configuration_IPU(CCWtargePort,CCWtargePin);		//��GPIO��Ӧ�ܽ�����Ϊ��������ģʽ----V20170605
	
	PWM_OUT(TIM2,PWM_OUTChannel1,2,300);						//PWM�趨-20161127�汾
	
//  IWDG_Configuration(1000);													//�������Ź�����---������λms
	SysTick_DeleymS(1000);				//SysTick��ʱnmS
	for(time=0;time<20;time++)
	organgle	=	ReadAngle();
	angle1	=	organgle;
	angle2	=	organgle;
	SysTick_DeleymS(200);				//SysTick��ʱnmS
  SysTick_Configuration(1000);    //ϵͳ���ʱ������72MHz,��λΪuS
	
//	cwflg	=	1;
	
	while(1)
	{
		MOTORT();
		SetOrig();
		if(anglecount>=350)
		{
//			MSTP();
			cwflg	=	0;
		}		
	}
}

//=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>
//->������		:	
//->��������	:	 
//->����		:	
//->���		:
//->���� 		:
//<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=
void TLE5012B_Server(void)
{	
	unsigned char result=0;
	goto targe;
	
	test:
	if(time++>8000)
	{		
		time	=	0;
	}
	if(0	==	time)
	{		
		angle1	=	ReadAngle();
		angle1	=	ReadAngle();
		angle2	=	angle1;
		
		cwflg	=	1;		//��ת
		anglecount	=	0;
	}
	else if(3000	==	time)
	{		
		angle1	=	ReadAngle();
		angle1	=	ReadAngle();
		angle2	=	angle1;
		
		cwflg	=	2;		//��ת
		anglecount	=	0;
	}
	else if(6000<time)
	{
		cwflg	=	0;
		anglecount	=	0;
	}	
	goto updatadsp;
	
	targe:
	if(timedelay>0)
	{
		timedelay--;
		goto updatadsp;
	}
	result=GPIO_ReadInputDataBit(CWtargePort,CWtargePin);
	if(0==result)
	{
		angle1	=	ReadAngle();
		angle1	=	ReadAngle();
		angle2	=	angle1;
		
		cwflg	=	1;		//��ת
		anglecount	=	0;
		timedelay=50;
	}
	result=GPIO_ReadInputDataBit(CCWtargePort,CCWtargePin);
	if(0==result)
	{
		angle1	=	ReadAngle();
		angle1	=	ReadAngle();
		angle2	=	angle1;
		
		cwflg	=	2;		//��ת
		anglecount	=	0;
		timedelay=50;
	}

	
	updatadsp:
	if(dstime++>100)
	{
		dstime=0;
//		unsigned	short temp	=	0;
//		dstime=0;
//		TM1616_Display(&Seg1,angle1);
//		if(cmpgangle<0)
//			cmpgangle=0-cmpgangle;
//		TM1616_Display(&Seg2,anglecount);
		
//		displaycount(anglecount);
		
		TM1616_Display(&Seg1,angle1);
		TM1616_Display(&Seg2,anglecount);
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
void MOTORT(void)
{	
	if(0==cwflg)	//������angle2>angle1
	{
		return;
	}
	if(readdelay++<2)
	{
		return;
	}
	readdelay	=	0;
	angle2	=	ReadAngle();
	
//	if(angle1	==	angle2)
//		return;
	if(1==cwflg)	//������angle2>angle1
	{	
		
		if(angle2-angle1>120)
			anglecount+=360-angle2+angle1;
		else if(angle2>angle1)
			anglecount+=angle2-angle1;
		else if(anglecount>0)
		{
			if(angle1-angle2>120)
				anglecount-=360-angle1+angle2;
			else
				anglecount-=angle1-angle2;
		}
	}
	else if(2==cwflg)	//����
	{
		if(angle1-angle2>120)
			anglecount+=360-angle1+angle2;
		else if(angle1>angle2)
			anglecount+=angle1-angle2;
		else if(anglecount>0)
		{
			if(angle2-angle1>120)
				anglecount-=360-angle2+angle1;
			else
				anglecount-=angle2-angle1;
		}
	}	
	angle1	=	angle2;
	if(1	==	cwflg)
		MCW();
	else if(2	==	cwflg)
		MCCW();
	else
		MSTP();
	
	return;
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
void SetOrig(void)
{
//	unsigned	short ccmp	=	0;
	if(0!=cwflg)
		return;
//	angle1	=	ReadAngle();
	
//	if(5<angle1&&angle1<=180)
//	{
//		MCCW();
//	}
//	else if(180<angle1&&angle1<=355)
//	{
//		MCW();
//	}
//	else
//	{
//			MSTP();
//	}
//	return;
	
	angle1	=	ReadAngle();
	if((angle1>organgle)&&(angle1-organgle>10))
	{
		MCCW();
	}
	else if((angle1<organgle)&&(organgle-angle1>10))
	{
		MCW();
	}
	else
	{
		MSTP();
	}

}
//=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>
//->������		:	
//->��������	:	 
//->����		:	
//->���		:
//->���� 		:
//<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=
void USART_Server(void)
{
  unsigned short RxNum  = 0;
  unsigned char str[]="���ڽ��յ�����\r\n";
  RxNum = USART_ReadBufferIDLE(USART1,u1rxbuffer);
  if(RxNum)
  {
    memcpy(u1txbuffer,str,sizeof(str));
    memcpy(u2txbuffer,u1rxbuffer,RxNum);

    
    API_USART_DMA_Send(USART1,u1txbuffer,sizeof(str)-1);		//����DMA���ͳ�����������Ѿ����뵽DMA������Buffer��С�����򷵻�0
    
    RS485_DMASend(&stRS485BS,u2txbuffer,RxNum);

  }
  RxNum = RS485_ReadBufferIDLE(&stRS485BS,u2rxbuffer);
  if(RxNum)
  {
    memcpy(u1txbuffer,u2rxbuffer,RxNum);
    API_USART_DMA_Send(USART1,u1txbuffer,RxNum);
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
void hw_configuration(void)
{
	TLE5012.Port.SPIx	=	SPI1;
	TLE5012.Port.CS_PORT	=	GPIOA;
	TLE5012.Port.CS_Pin		=	GPIO_Pin_4;
	
	TLE5012.Port.SPI_BaudRatePrescaler_x	=	SPI_BaudRatePrescaler_8;
	
	SPI_Initialize(&TLE5012);
	MCO_Initialize();
}
//=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>
//->������		:	
//->��������	:	 
//->����		:	
//->���		:
//->���� 		:
//<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=
void COMM_Configuration(void)
{
  unsigned short RxNum  = 0;
  USART_DMA_ConfigurationNR	(USART1,115200,ussize);	//USART_DMA����--��ѯ��ʽ�������ж�

  //-----------------------------���߽ӿ�485
  stRS485BS.USARTx  = USART2;
  stRS485BS.RS485_CTL_PORT  = GPIOB;
  stRS485BS.RS485_CTL_Pin   = GPIO_Pin_11;
  RS485_DMA_ConfigurationNR			(&stRS485BS,19200,ussize);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
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
void Seg_Configuration(void)
{
	Seg1.STB_PORT	=	GPIOB;
	Seg1.STB_Pin	=	GPIO_Pin_12;
	
	Seg1.DIO_PORT	=	GPIOB;
	Seg1.DIO_Pin	=	GPIO_Pin_15;
	
	Seg1.CLK_PORT	=	GPIOB;
	Seg1.CLK_Pin	=	GPIO_Pin_13;
	
	Seg2.STB_PORT	=	GPIOB;
	Seg2.STB_Pin	=	GPIO_Pin_2;
	
	Seg2.DIO_PORT	=	GPIOB;
	Seg2.DIO_Pin	=	GPIO_Pin_15;
	
	Seg2.CLK_PORT	=	GPIOB;
	Seg2.CLK_Pin	=	GPIO_Pin_13;
	
	TM1616_Initialize(&Seg1);
	TM1616_Initialize(&Seg2);
}


//�õ� 0~359 ��
unsigned short ReadAngle(void)
{
	return ( ReadValue(READ_ANGLE_VALUE) * 360 / 0x10000 );
}

//�õ����ٶ�
unsigned short ReadSpeed(void)
{
	return ReadValue(READ_SPEED_VALUE);
}


unsigned short ReadValue(unsigned short u16RegValue)
{
	unsigned short u16Data;
	unsigned short i	=	5;
	SPI_CS_ENABLE;
	
	SPIx_ReadWriteByte(u16RegValue);
  SPI_TX_OFF;
//	while(i--);
	//���� 0xFFFF �����õģ�������Ϊ����ʱ��
	u16Data = ( SPIx_ReadWriteByte(0xffff) & 0x7FFF ) << 1;//0x12/0xff*100k
	
	SPI_CS_DISABLE;
  SPI_TX_ON;
	
	return(u16Data);
}

unsigned short SPIx_ReadWriteByte(unsigned short byte)
{
	unsigned short retry = 0;
	while( (SPI1->SR&1<<1) == 0 )//���ͻ������ǿ�
	{
		if( ++retry > 200 )
			return 0;//�ӳ�һ��ʱ��󷵻�
	}
	SPI1->DR = byte;     //��������
	
	retry = 0;
	while( (SPI1->SR&1<<0) == 0 ) //���ջ�����Ϊ��
	{
		if( ++retry > 200 )
			return 0;//�ӳ�һ��ʱ��󷵻�
	}
	return SPI1->DR;          //��һ�»����������־
}
void SPI5012B_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1,ENABLE );
	
	//���¶��䣬������������������ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);//ʹJTDO��JTDI��JTCK ������ͨIO�ڽ��в���
	//GPIOB0 ������ͨIO�ڽ��в���
	
	/*SPI: NSS,SCK,MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//PA5--CLK--��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//GPIO_StructInit(&GPIO_InitStructure);
	
	/* Configure PA6 as encoder input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//PA6--MISO--����
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* PA7*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//PA7--MOSI--�������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_CS_Pin_Name;//PB0--CS--�������
	GPIO_Init(GPIO_CS_Pin_Type, &GPIO_InitStructure);
	
	/**********SPI****************/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//SPI1--˫��ȫ˫������
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);
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
void MCW(void)
{
	PB6	=	0;
	PB7	=	1;
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
void MCCW(void)
{
	PB6	=	1;
	PB7	=	0;
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
void MSTP(void)
{
	PB6	=	0;
	PB7	=	0;
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
void displaycount(unsigned long count)
{
	TM1616_Display(&Seg1,count/10000);
	TM1616_Display(&Seg2,count%10000);
}
#endif