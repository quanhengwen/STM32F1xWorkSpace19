#ifdef SPI_TEST
#include "SPI_TEST.H"
#include "SPI_FLASH.H"

#include "STM32_SYS.H"
#include "STM32_GPIO.H"
#include "STM32_PWM.H"
#include "STM32_SPI.H"
#include "STM32_SYSTICK.H"
#include "STM32F10x_BitBand.H"


#include	"stdio.h"			//����printf
#include	"string.h"		//����printf
#include	"stdarg.h"		//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"		//malloc��̬�����ڴ�ռ�



SPIDef stSpi;
#define testlen 128
unsigned  char testrxbuffer[testlen]={0x05};
unsigned  char testtxbuffer[testlen]={0x05};

void Power_Configuration(void);
void SPI_Configuration(void);
void SPI_Test(void);
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void SPI_TEST_Configuration(void)
{
  SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H	
  Power_Configuration();
  SPI_Configuration();
  GPIO_Configuration_OPP50	(GPIOB,	GPIO_Pin_12);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
  GPIO_Configuration_OPP50	(GPIOA,	GPIO_Pin_0);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
//  PWM_OUT(TIM2,PWM_OUTChannel1,2000,500);						//PWM�趨-20161127�汾
  memset(testtxbuffer,0xAA,testlen);
  SysTick_Configuration(10);
}

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void SPI_TEST_Server(void)
{
  GPIO_Toggle	(GPIOA,	GPIO_Pin_0);		//��GPIO��Ӧ�ܽ������ת----V20170605
//  GPIO_Toggle	(GPIOB,	GPIO_Pin_12);		//��GPIO��Ӧ�ܽ������ת----V20170605
	SPI_Test();
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
void SPI_Configuration(void)
{
  GPIO_InitTypeDef	GPIO_InitStructure;
SPI_InitTypeDef  SPI_InitStructure;
DMA_InitTypeDef	DMA_Initstructure;
  
  stSpi.Port.SPIx = SPI2;
  
  stSpi.Port.CS_PORT  = GPIOB;
  stSpi.Port.CS_Pin   = GPIO_Pin_12;
  SPI_InitializeDMA(&stSpi);
  
  return;
  
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 ,ENABLE);				//����SPIʱ��
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOB, ENABLE);
  
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;				//���÷���				��2��ȫ˫����2��ֻ���ա�һ�߷��͡�һ�߽��գ�
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															//ģʽ         	���ӻ����豸��
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;													//����         	��8��16λ��
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;																//ʱ�Ӽ���     	���ͻ�ߣ�
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;															//ʱ����λ     	����һ����ڶ��������أ�	
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												//���ȷ��͵�λ 	�����λ���������λ���ȣ�
	SPI_InitStructure.SPI_CRCPolynomial = 7;																	//����crc����ʽ	�����֣���7
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	//Ƭѡ��ʽ     	��Ӳ����������ʽ��
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;				//������Ԥ��Ƶ 	����2---256��Ƶ��
  
//  SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;																//Ƭѡ��ʽ     	��Ӳ����������ʽ��  
  
	SPI_Init(SPI2,&SPI_InitStructure);
  
//  SPI_SSOutputCmd(SPI2, ENABLE);			//���������ģʽ�µ�Ƭѡ��ʽΪӲ����SPI_NSS_Hard����ʽ���˴�����򿪣�����NSS���ź�
  
	SPI_Cmd(SPI2, ENABLE);				//ʹ��SPI
  
//  SPI_SSOutputCmd(SPI2, ENABLE);			//���������ģʽ�µ�Ƭѡ��ʽΪӲ����SPI_NSS_Hard����ʽ���˴�����򿪣�����NSS���ź�
  
//5)**********DMA���ͳ�ʼ����������ΪDMA��Ŀ�Ķ�
  DMA_Initstructure.DMA_PeripheralBaseAddr =  (u32)(&SPI2->DR);	//DMA����Դ��ַ
  DMA_Initstructure.DMA_MemoryBaseAddr     = (u32)testtxbuffer;						//DMA�����ڴ��ַ
  DMA_Initstructure.DMA_DIR = DMA_DIR_PeripheralDST;												//DMA_DIR_PeripheralDST��������ΪDMA��Ŀ�Ķˣ���DMA_DIR_PeripheralSRC��������Ϊ���ݴ������Դ��
  DMA_Initstructure.DMA_BufferSize = testlen; 													  //ָ��DMAͨ����DMA����Ĵ�С
  DMA_Initstructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;					//DMA_PeripheralInc_Enable�������ַ�Ĵ�����������DMA_PeripheralInc_Disable�������ַ�Ĵ������䣩��
  DMA_Initstructure.DMA_MemoryInc =DMA_MemoryInc_Enable;										//DMA_MemoryInc_Enable���ڴ��ַ�Ĵ�����������DMA_MemoryInc_Disable���ڴ��ַ�Ĵ������䣩
  DMA_Initstructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;		//�������ݿ���--DMA_PeripheralDataSize_Byte�����ݿ���Ϊ8λ����DMA_PeripheralDataSize_HalfWord�����ݿ���Ϊ16λ����DMA_PeripheralDataSize_Word�����ݿ���Ϊ32λ��
  DMA_Initstructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;						//�ڴ����ݿ���--DMA_MemoryDataSize_Byte�����ݿ���Ϊ8λ����DMA_MemoryDataSize_HalfWord�����ݿ���Ϊ16λ����DMA_MemoryDataSize_Word�����ݿ���Ϊ32λ��
  DMA_Initstructure.DMA_Mode = DMA_Mode_Normal;															//DMA����ģʽ--DMA_Mode_Normal��ֻ����һ�Σ�, DMA_Mode_Circular����ͣ�ش��ͣ�
  DMA_Initstructure.DMA_Priority = DMA_Priority_High; 											//DMAͨ����ת�����ȼ�--DMA_Priority_VeryHigh���ǳ��ߣ�DMA_Priority_High����)��DMA_Priority_Medium���У���DMA_Priority_Low���ͣ�
  DMA_Initstructure.DMA_M2M = DMA_M2M_Disable;															//DMAͨ�����ڴ浽�ڴ洫��--DMA_M2M_Enable(����Ϊ�ڴ浽�ڴ洫��)��DMA_M2M_Disable�����ڴ浽�ڴ洫�䣩
  DMA_Init(DMA1_Channel5,&DMA_Initstructure);															//��ʼ��DMA

  //6)**********DMA���ճ�ʼ����������ΪDMA��Դ��
  DMA_Initstructure.DMA_PeripheralBaseAddr =  (u32)(&SPI2->DR);	//DMA����Դ��ַ
  DMA_Initstructure.DMA_MemoryBaseAddr     = 	(u32)testrxbuffer;						//DMA�����ڴ��ַ
  DMA_Initstructure.DMA_DIR = DMA_DIR_PeripheralSRC;												//DMA_DIR_PeripheralDST��������ΪDMA��Ŀ�Ķˣ���DMA_DIR_PeripheralSRC��������Ϊ���ݴ������Դ��
  DMA_Initstructure.DMA_BufferSize = testlen; 													  //ָ��DMAͨ����DMA����Ĵ�С
  DMA_Initstructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;					//DMA_PeripheralInc_Enable�������ַ�Ĵ�����������DMA_PeripheralInc_Disable�������ַ�Ĵ������䣩��
  DMA_Initstructure.DMA_MemoryInc =DMA_MemoryInc_Enable;										//DMA_MemoryInc_Enable���ڴ��ַ�Ĵ�����������DMA_MemoryInc_Disable���ڴ��ַ�Ĵ������䣩
  DMA_Initstructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;		//�������ݿ���--DMA_PeripheralDataSize_Byte�����ݿ���Ϊ8λ����DMA_PeripheralDataSize_HalfWord�����ݿ���Ϊ16λ����DMA_PeripheralDataSize_Word�����ݿ���Ϊ32λ��
  DMA_Initstructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;						//�ڴ����ݿ���--DMA_MemoryDataSize_Byte�����ݿ���Ϊ8λ����DMA_MemoryDataSize_HalfWord�����ݿ���Ϊ16λ����DMA_MemoryDataSize_Word�����ݿ���Ϊ32λ��
  DMA_Initstructure.DMA_Mode = DMA_Mode_Normal;															//DMA����ģʽ--DMA_Mode_Normal��ֻ����һ�Σ�, DMA_Mode_Circular����ͣ�ش��ͣ�
  DMA_Initstructure.DMA_Priority = DMA_Priority_High; 											//DMAͨ����ת�����ȼ�--DMA_Priority_VeryHigh���ǳ��ߣ�DMA_Priority_High����)��DMA_Priority_Medium���У���DMA_Priority_Low���ͣ�
  DMA_Initstructure.DMA_M2M = DMA_M2M_Disable;															//DMAͨ�����ڴ浽�ڴ洫��--DMA_M2M_Enable(����Ϊ�ڴ浽�ڴ洫��)��DMA_M2M_Disable�����ڴ浽�ڴ洫�䣩
  DMA_Init(DMA1_Channel4,&DMA_Initstructure);															//��ʼ��DMA
  
  
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);								//����DMA����
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);								//����DMA����
  
  DMA_Cmd(DMA1_Channel4,ENABLE);	
  DMA_Cmd(DMA1_Channel5,ENABLE);
    
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
void SPI_Test(void)
{
//  rets:
//  PB12  = 0;
//  SPI_DMASend(stSpi.Port.SPIx,testtxbuffer,testlen);
////  
//  while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);		//���ָ��SPI������ɱ�־�������
////  goto rets;
//  PB12  = 1;
  unsigned  short num = 0;
  PB12  = 0;
  num = SPI_DMAReadWrite(SPI2,testtxbuffer,testrxbuffer,testlen);
  if((0==num)||(0xFFFF==num))
  {
    PB12  = 0;
  }
  else
  {
    PB12  = 1;
  }

  
  
  
//  SPI_Cmd(SPI2, ENABLE);				//ʹ��SPI
//  SPI_I2S_SendData(SPI2, 0x55);				//��������
//  SPI_Cmd(SPI2, DISABLE);				//ʹ��SPI
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
void Power_Configuration(void)
{
  unsigned  short  temp  = 0;
  GPIO_Configuration_IPU(GPIOB,	GPIO_Pin_4);			//��GPIO��Ӧ�ܽ�����Ϊ��������ģʽ----V20170605
  GPIO_Configuration_OPP50(GPIOF,GPIO_Pin_10);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
  restart:
  PF10 = 1;
	SysTick_DeleymS(1000);					//SysTick��ʱnS
//  while(0  ==  PB4in)
//  {
//    SysTick_DeleymS(1);					//SysTick��ʱnS
//    temp++;
//    if(temp>3000)
//    {
//      Key = 0;
//      
//      break;
//    }
//    
//  }
//  if(temp<3000)
//  {
//    PF10  = 1;
//    temp  = 0;
//    goto restart;
//  }
}




#endif