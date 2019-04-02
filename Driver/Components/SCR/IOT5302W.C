/********************************************************************************
***IOT5302W������--RS485
********************************************************************************/
#include  "IOT5302W.H"
#include  "TOOL.H"


#include	"stdio.h"			//����printf
#include	"string.h"			//����printf
#include	"stdarg.h"			//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"			//malloc��̬�����ڴ�ռ�

IOT5302Wdef IOT5302W;

unsigned char CmdBuffer[16]={0};

void IOT5302W_Initialize(void);     //��������ʼ��
void IOT5302W_DataProcess(void);    //���ݴ���������ƴ����Э����
void IOT5302W_DataCheck(void);

unsigned short IOT5302W_HWSendData(unsigned char* TxdBuffer,unsigned short length);
unsigned short IOT5302W_HWReadData(unsigned char* Buffer);
/*******************************************************************************
*������			:	IOT5302W_Configuration
*��������		:	IOT5302W����
*����				: pRS485ʹ�õ�RS485�ṹ��
              USART_BaudRateϣ�����óɵĲ�����
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void API_IOT5302WConfiguration(IOT5302Wdef* pIOT5302W)
{
  IOT5302W  = *pIOT5302W;
  IOT5302W.Data.Initialized = 0;
  IOT5302W.Data.Time        = 0;
  IOT5302W.Data.TimeOut     = 0;
  IOT5302W.Data.USART_BaudRate  = 0;
  IOT5302W.Data.BaudRateSetStep = 0;
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
void API_IOT5302WServer(void)
{ 
  //==================================������
  if((0==IOT5302W.Conf.USART_BaudRate)||((0==IOT5302W.Conf.IOT5302WPort.USARTx)||(0==IOT5302W.Conf.IOT5302WPort.RS485_CTL_PORT)))  //��������
  {
    return;
  }
  
  //==================================���ݴ���������ƴ����Э����
  IOT5302W_DataProcess();
  //==================================���δ��ʼ���ɹ�����ִ��
  if(0==IOT5302W.Data.Initialized)  //δ��ʼ���ɹ�
  {
    IOT5302W_Initialize();    //��������ʼ��
    return;
  }
  IOT5302W.Data.Time++;
  //==================================����ɨ������
  if(IOT5302W.Data.Time%IOT5302WReadCycle==0)   //0.5��ɨ��һ��
  {
    unsigned char TxdLen  = 0;
    IOT5302W.Data.Time=0; //����
    IOT5302WGetSNR(CmdBuffer);
    TxdLen  = IOT5302WGetSNR(CmdBuffer);                    //���ö�����������
    TxdLen  = IOT5302W_HWSendData(CmdBuffer,TxdLen);	      //Ѱ������ȡ����UID��ÿ��M1������һ��Ψһ�����кţ����ǳ�Ϊ��UID������32λ�ģ�Ҳ����4���ֽڡ�
  }
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
unsigned short API_IOT5302WGetUID(unsigned char* Buffer)
{
  unsigned char i=0;
  unsigned char n=0;
  for(i=0;i<4;i++)
  {
    if(IOT5302W.Data.UID[i])
      n++;
  }
  if(0!=n)  //������
  {
    if(memcmp(IOT5302W.Data.UID,IOT5302W.Data.UIDbac,4)) //�Ա��������ݲ�һ��
    {
      memcpy(Buffer,IOT5302W.Data.UID,4);
      memcpy(IOT5302W.Data.UIDbac,IOT5302W.Data.UID,4);
      memset(IOT5302W.Data.UID,0x00,4);
      IOT5302W.Data.TimeCmp = 0;
      return 4; //4�ֽ�UID
    }
    else if(IOT5302W.Data.TimeCmp++>2000)
    {
      memcpy(Buffer,IOT5302W.Data.UID,4);
      IOT5302W.Data.TimeCmp = 0;
      return 4; //4�ֽ�UID
    }
  }
  else
  {
    memset(IOT5302W.Data.UIDbac,0x00,4);
  }
  return 0;
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
unsigned short IOT5302W_HWSendData(unsigned char* TxdBuffer,unsigned short length)
{
  unsigned short TxdLen = 0;
  TxdLen = api_rs485_dma_send(&IOT5302W.Conf.IOT5302WPort,TxdBuffer,length);	//RS485-DMA���ͳ���
  return TxdLen;
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
unsigned short IOT5302W_HWReadData(unsigned char* Buffer)
{
  unsigned short RxdLen = 0;
  RxdLen = api_rs485_dam_receive(&IOT5302W.Conf.IOT5302WPort,Buffer);
  return RxdLen;
}
/*******************************************************************************
*������			:	function
*��������		:	���ö������ӿ�
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void IOT5302W_HWPortInitialize(unsigned long USART_BaudRate)
{  
  api_rs485_dma_configurationNR(&IOT5302W.Conf.IOT5302WPort,USART_BaudRate,IOT5302WBufferSize);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
}
/*******************************************************************************
*������			:	IOT5302W_Initialize
*��������		:	���ݶ���������Ӧʱ�䣬ʱ������������Ϊ200ms
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void IOT5302W_Initialize(void)
{  
  IOT5302W_DataProcess(); ////���ݴ���������ƴ����Э����
  if(0!=IOT5302W.Data.Initialized)  //δ��ʼ���ɹ�
  {
    return;
  }
  //---------------------1������һ�β�����,������������Ϊ19200
  if(IOT5302W.Data.Time>=IOT5302WInitializeTimeOut)
  {
    IOT5302W.Data.Time  = 0;
  }
  //---------------------ѡ����Ӧ�Ĳ��Բ�����
  if(IOT5302W.Data.Time%IOT5302WReadCycle==0)
  {    
    if(0==IOT5302W.Data.BaudRateSetStep)
    {
      IOT5302W.Data.USART_BaudRate=9600;
    }
    else if(1==IOT5302W.Data.BaudRateSetStep)
    {
      IOT5302W.Data.USART_BaudRate=19200;
    }
    else if(2==IOT5302W.Data.BaudRateSetStep)
    {
      IOT5302W.Data.USART_BaudRate=38400;
    }
    else if(3==IOT5302W.Data.BaudRateSetStep)
    {
      IOT5302W.Data.USART_BaudRate=57600;
    }
    else if(4==IOT5302W.Data.BaudRateSetStep)
    {
      IOT5302W.Data.USART_BaudRate=115200;
    }
    else
    {
      IOT5302W.Data.USART_BaudRate=IOT5302W.Conf.USART_BaudRate;
      IOT5302W.Data.BaudRateSetStep = 0;
    }
    IOT5302W_HWPortInitialize(IOT5302W.Data.USART_BaudRate);	//���ö������ӿ�
    IOT5302W.Data.Time  = 0;
  }
  //---------------------0.5�뷢��һ������
  if((50==IOT5302W.Data.Time))
  {
    
    unsigned char TxdLen  = 0;
    unsigned char BaudRateCode  = 0;
    switch(IOT5302W.Conf.USART_BaudRate)    //����ʵ��Ӧ����Ҫ�Ĳ��������û�ȡ���ö����������ʵĴ���
    {
      case 9600:BaudRateCode=0;
      break;
      case 19200:BaudRateCode=1;
      break;
      case 38400:BaudRateCode=2;
      break;
      case 57600:BaudRateCode=3;
      break;
      case 115200:BaudRateCode=4;
      break;
      default:BaudRateCode=0; //Ĭ������
      break;
    }
    TxdLen  = IOT5302WSetBaudrate(CmdBuffer,BaudRateCode);  //���ö�����������
    TxdLen  = IOT5302W_HWSendData(CmdBuffer,TxdLen);	      //RS485-DMA���ͳ���
    IOT5302W.Data.BaudRateSetStep += 1;
  }
  IOT5302W.Data.Time++;
}
/*******************************************************************************
*������			:	IOT5302W_DataProcess
*��������		:	���ݴ���������ƴ����Э����
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void IOT5302W_DataProcess(void)
{
  unsigned char IOT5302WRx[64];
  unsigned short RxdLen = 0;  
  RxdLen  = IOT5302W_HWReadData(IOT5302WRx);
  if(RxdLen)
  {
    if(IOT5302W.Data.DataCount+RxdLen>IOT5302WBufferSize)
    {
      IOT5302W.Data.DataCount=0;
      memset(IOT5302W.Data.UID,0x00,4);   //���UID��¼
      IOT5302W.Data.TimeOut = 0;    //��ʱ��ʱ���
    }
    memcpy(&IOT5302W.Data.Data[IOT5302W.Data.DataCount],IOT5302WRx,RxdLen); //������յ�������
    IOT5302W.Data.DataCount+=RxdLen;    
  }
  else if(IOT5302W.Data.TimeOut++>=IOT5302WReadTimeOut) //��ʱ���޶��������߲����ʲ���
  {
    IOT5302W.Data.Initialized = 0;  //��ʼ����־��0-δ��ʼ���ɹ���1-��ʼ���ɹ�
  }
  IOT5302W_DataCheck();
}
/*******************************************************************************
*������			:	IOT5302W_DataCheck
*��������		:	�������Э��
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void IOT5302W_DataCheck(void)
{
  if(0==IOT5302W.Data.DataCount)  //û������
  {
    return;
  }
  else
  {
    unsigned char i=0;
    unsigned char n=0;
    unsigned char bcc=0;
    for(i=0;i<IOT5302W.Data.DataCount;i++)
    {
      if(IotReaderSTX==IOT5302W.Data.Data[i]) //���ҵ�ͷ��ʶ
      {
        n=IOT5302W.Data.Data[i+2];
        if(i+2+n+1+1>IOT5302WBufferSize)  //��������
        {
          IOT5302W.Data.DataCount=0;
        }
        if(IotReaderETX==IOT5302W.Data.Data[i+2+n+1+1]) //β��ʶ��2�ֽ�ΪID�ͳ��ȣ�n�ֽ����ݣ�1�ֽ�У�飬1�ֽ�β��ʶ)
        {
          bcc=BCC8(&IOT5302W.Data.Data[i+1],n+2);   //����BCC(��ID��ʼ������CMD/STDλ��n������
          if(bcc==IOT5302W.Data.Data[i+2+n+1])//BCC�ֽڣ�2�ֽ�ΪID�ͳ��ȣ�n�ֽ����ݣ�����ֽ�ΪBCC)
          {
            IOT5302W.Data.Initialized = 1;  //��ʼ����־��0-δ��ʼ���ɹ���1-��ʼ���ɹ�
            if(0x06==IOT5302W.Data.Data[i+2]) //UID��Ϣ�����ݳ���
            {
              memcpy(IOT5302W.Data.UID,&IOT5302W.Data.Data[i+5],4); //��ȡ��UID
              memset(IOT5302W.Data.Data,0x00,IOT5302WBufferSize);   //�������
              IOT5302W.Data.DataCount=0;
              IOT5302W.Data.TimeOut = 0;    //��ʱ��ʱ���
              return;
            }
          }
        }
      }
    }
    if(i>=IOT5302WBufferSize-2)//δ�ҵ�UID
    {
      memset(IOT5302W.Data.Data,0x00,IOT5302WBufferSize);   //�������
      IOT5302W.Data.DataCount=0;
    }
  }
  if(IOT5302W.Data.TimeOut++>IOT5302WReadTimeOut)
  {
    memset(IOT5302W.Data.UID,0x00,4);   //���UID��¼
    IOT5302W.Data.TimeOut = 0;    //��ʱ��ʱ���
  }
}
/*******************************************************************************
*������			:	IOT5302WGetSNR
*��������		:	Ѱ������ȡ����UID��ÿ��M1������һ��Ψһ�����кţ����ǳ�Ϊ��UID������32λ�ģ�Ҳ����4���ֽڡ�
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
unsigned short IOT5302WGetSNR(unsigned char* Buffer)
{
  unsigned short FrameLen=8;
  
  Buffer[0]   = IotReaderSTX; //ͷ��ʶ
  Buffer[1]   = 0x00;         //�豸��ַ
  Buffer[2]   = 0x03;         //���ȣ�������STX,ETX,BCC,�豸��ַ
  Buffer[3]   = GET_SNR;      //����
  Buffer[4]   = 0x26;         //0x26��IDLEģʽ��ֻ��һ�ſ�����
                              //0x52��ALLģʽ���ɶԶ��ſ�����
  Buffer[5]   = 00;           //0x00����д������Ҫִ��halt
                              //0x01����д����Ҫִ��halt
  Buffer[6]  = BCC8(&Buffer[1],5);		//���У��;
  Buffer[7]  = IotReaderETX;		//������־��
  
  return  FrameLen;
}
/*******************************************************************************
*������			:	IOT5302WSetBaudrate
*��������		:	���ö�����������
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
unsigned short IOT5302WSetBaudrate(unsigned char* Buffer,unsigned char BaudRateCode)
{
  unsigned short FrameLen=7;
  
  Buffer[0]   = IotReaderSTX; //ͷ��ʶ
  Buffer[1]   = 0x00;         //�豸��ַ
  Buffer[2]   = 0x02;         //���ȣ�������STX,ETX,BCC,�豸��ַ
  Buffer[3]   = SetBaudrate;  //����
  Buffer[4]   = BaudRateCode; //0x00��9600
                              //0x01��19200
                              //0x02��38400
                              //0x03��57600
                              //0x04��115200
                              //������9600
  Buffer[5]  = BCC8(&Buffer[1],4);		//���У��;
  Buffer[6]  = IotReaderETX;		//������־��
  
  return  FrameLen;
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
unsigned short IotReaderGetCmdFrame(Cmddef Cmd,unsigned char* Buffer)
{
  unsigned short FrameLen=0;
  IRDdef* IotReader=(IRDdef*)Buffer;
  IotReader->STX  = IotReaderSTX;   //ͷ��ʶ
  IotReader->ID   = 0x00;
  IotReader->Len  = 0X00;     //����
  IotReader->Cts  = Cmd;
  switch((unsigned char)Cmd)
  {
    case  WriteUserInfo:
          FrameLen  = 0;
    case  ReadUserInfo:
          FrameLen  = 0;
    default:
      FrameLen  = 0;
  }
  return  FrameLen;
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
unsigned short IotMifareGetCmdFrame(Cmddef Cmd,unsigned char Sec,unsigned char* Buffer,unsigned char* KeyBuffer)
{
  unsigned short FrameLen=0;
//  if(Sec<1&&Sec>4)  //��ȡ�Ŀ鷶Χ����
//  {
//    return  0;
//  }
  switch((unsigned char)Cmd)
  {
    case  IotRead:
          Buffer[0]   = IotReaderSTX; //ͷ��ʶ
          Buffer[1]   = 0x00;         //�豸��ַ
          Buffer[2]   = 0x0A;         //���ȣ�������STX,ETX,BCC,�豸��ַ
          Buffer[3]   = Cmd;          //����
          Buffer[4]   = 0x01;         //Data[0]��ȡģʽ
          Buffer[5]   = Sec;          //Data[1]Ҫ���Ŀ飬�������ٿ顣ȡֵ��Χ01-04(һ��������4���飬����һ�ζ�ȡ�����)
          Buffer[6]   = 0x01;         //Data[2]Ҫ���Ŀ������ַ��ȡֵ��Χ��ʮ������00-3F��00�鵽63�顣(һ������4���飬һ����16�ֽڣ�һ�������ܹ�64�ֽ�)
          Buffer[7]   = KeyBuffer[0]; //Data[3]key[0]
          Buffer[8]   = KeyBuffer[1]; //Data[4]key[1]
          Buffer[9]   = KeyBuffer[2]; //Data[5]key[2]
          Buffer[10]  = KeyBuffer[3]; //Data[6]key[3]
          Buffer[11]  = KeyBuffer[4]; //Data[7]key[4]
          Buffer[12]  = KeyBuffer[5]; //Data[8]key[5]
          Buffer[13]  = BCC8(&Buffer[1],12);		//���У��;         //Data[8]key[5]
          Buffer[14]  = IotReaderETX;		//���У��;         //Data[8]key[5]
          FrameLen    = 15;
    break;
    case  IotWrite:
          FrameLen  = 0;
    break;
    default:
      FrameLen  = 0;
  }
  return  FrameLen;
}


