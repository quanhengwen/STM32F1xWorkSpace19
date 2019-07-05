/********************************************************************************
***IOT5302W读卡器--RS485
********************************************************************************/
#include  "IOT5302W.H"
#include  "TOOL.H"


#include	"stdio.h"			//用于printf
#include	"string.h"			//用于printf
#include	"stdarg.h"			//用于获取不确定个数的参数
#include	"stdlib.h"			//malloc动态申请内存空间

IOT5302Wdef IOT5302W;

static unsigned char CmdBuffer[16]={0};



//--------------------------------------V3自动适用读卡器波特率，按照一帧数据接收
/*******************************************************************************
*函数名			:	IOT5302W_Configuration
*功能描述		:	IOT5302W配置
*输入				: pRS485使用的RS485结构体
              USART_BaudRate希望配置成的波特率
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_iot5302w_configuration(IOT5302Wdef* pIOT5302W)
{
  IOT5302W  = *pIOT5302W;
  IOT5302W.Data.Initialized = 0;
  IOT5302W.Data.USART_BaudRate  = 0;
	//------------串口配置在iot5302w_initialize函数中
	//api_rs485_dma_configurationNR(&IOT5302W.Conf.IOT5302WPort,9600,IOT5302WBufferSize);	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_iot5302w_server(void)
{
	//先处理数据，再执行初始化，更改初始化参数后发现数据验证已通过
  //==================================检查参数
  if(0==IOT5302W.Conf.IOT5302WPort.USARTx)  //参数错误--未配置串口
  {
    return;
  }
	//==================================数据接收：对接收的数据进行拼包，并且做接收超时判断，如果超时未接收到任何数据，则设置初始化标志为未初始化
  iot5302w_data_receive_process();
	
	//==================================数据处理：对接收的数据进行协议检查并获取相关的数据，如果协议检查通过，设置初始化标志为已初始化
  iot5302w_data_process();
  
  //==================================如果未初始化成功，不执行
	iot5302w_initialize();
	
	//==================================如果读卡器已初始化，则周期性去查询读卡器的数据
	iot5302w_get_data();								
}
//------------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	iot5302w_get_data
*功能描述		:	如果读卡器已初始化，则周期性去查询读卡器的数据
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void iot5302w_get_data(void)
{
	if(0==IOT5302W.Data.TimeGetData)   //0.3秒扫描一次
  {
    unsigned char TxdLen  = 0;
    IOT5302W.Data.TimeGetData	=	IOT5302WReadCycle; 		//重新开始倒计时
    TxdLen  = iot5302w_GetFrame_GetSNR(CmdBuffer);  	//生成获取UID的消息帧
    TxdLen  = iot5302w_send_msg(CmdBuffer,TxdLen);		//寻卡，获取卡的UID，每个M1卡都有一个唯一的序列号，我们称为“UID”，是32位的，也就是4个字节。
  }
	IOT5302W.Data.TimeGetData--;			//倒计时
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	4字节卡片UID
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short api_get_iot5302w_uid(unsigned char* Buffer)
{
  unsigned char i=0;
  unsigned char n=0;
	//-------------------------检查UID缓存有无数据
  for(i=0;i<4;i++)
  {
    if(IOT5302W.Data.UID[i])	//检查UID是否为0
      n++;
  }
	//-------------------------有数据
  if(0!=n)  //有数据
  {
		//-----------------------检查缓存数据是否更新：新数据则立即上传
    if(memcmp(IOT5302W.Data.UID,IOT5302W.Data.UIDbac,4)) //对比两组数据不一样
    {
      memcpy(Buffer,IOT5302W.Data.UID,4);
      memcpy(IOT5302W.Data.UIDbac,IOT5302W.Data.UID,4);		//拷贝到备份区，防止快速连续上传相同数据
      memset(IOT5302W.Data.UID,0x00,4);			//清除数据
      IOT5302W.Data.TimeUIDCmp = 0;
      return 4; //4字节UID
    }
		//-----------------------相同新数据:间隔2秒上传一次
    else if(IOT5302W.Data.TimeUIDCmp++>2000)		//相同的数据，2秒钟上传一次
    {
      memcpy(Buffer,IOT5302W.Data.UID,4);
      IOT5302W.Data.TimeUIDCmp = 0;
      return 4; //4字节UID
    }
  }
//  else
//  {
//		if(no_data_wait_time++>300)	//200ms
//		{
//			no_data_wait_time	=	0;
//			memset(IOT5302W.Data.UIDbac,0x00,4);
//		}    
//  }
  return 0;
}
//------------------------------------------------------------------------------





//---------------------------------------------------------software
/*******************************************************************************
*函数名			:	iot5302w_initialize
*功能描述		:	以不同的波特率发送读卡器波特率配置命令，检查返回
							根据读卡器的响应时间，时间间隔可以设置为200ms
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void iot5302w_initialize(void)
{  
	unsigned char TxdLen  = 0;
	//---------------------读卡器已初始化，则退出
  if(0!=IOT5302W.Data.Initialized)  //初始化成功
  {
    return;
  }
  //---------------------0.3秒发送一次查询UID如果读卡器有返回任何数据，则当前波特率正确
  if(IOT5302W.Data.Time_Initialized++<300)
  {
   return; 
  }
	IOT5302W.Data.Time_Initialized  = 0;
	
  //---------------------循环选择相应的测试波特率	   
	if((0==IOT5302W.Data.USART_BaudRate)||(115200==IOT5302W.Data.USART_BaudRate))
	{
		IOT5302W.Data.USART_BaudRate=9600;
	}
	else if(9600==IOT5302W.Data.USART_BaudRate)
	{
		IOT5302W.Data.USART_BaudRate=19200;
	}
	else if(19200==IOT5302W.Data.USART_BaudRate)
	{
		IOT5302W.Data.USART_BaudRate=38400;
	}
	else if(38400==IOT5302W.Data.USART_BaudRate)
	{
		IOT5302W.Data.USART_BaudRate=57600;
	}
	else if(57600==IOT5302W.Data.USART_BaudRate)
	{
		IOT5302W.Data.USART_BaudRate=115200;
	}
	else
	{
		IOT5302W.Data.USART_BaudRate=9600;
	}
	//---------------------将串口配置为相应的波特率
	hw_port_configuration(IOT5302W.Data.USART_BaudRate);			//配置波特率，根据当前波特率与读卡器通讯是否成功
	//---------------------使用当前波特率查询一次读卡器UID，查看UID返回情况，如果有返回数据，表示当前波特率匹配读卡器成功
	//iot5302w_get_data();
	//-------------------下一次再次发送获取数据时间清零:重新计时
	IOT5302W.Data.TimeGetData	=	0;		//获取数据清零，表示立即发送获取数据消息
}
/*******************************************************************************
*函数名			:	iot5302w_data_receive_process
*功能描述		:	数据处理，接收拼包，协议检查
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void iot5302w_data_receive_process(void)
{
  unsigned char IOT5302WRx[64];
  unsigned short RxdLen = 0;  
  RxdLen  = iot5302w_read_msg(IOT5302WRx);
	//=====================接收到读卡器的数据
  if(RxdLen)
  {
		//-------------------接收的数据超出缓存：重新从缓存起始位置开始存储数据
    if(IOT5302W.Data.DataCount+RxdLen>IOT5302WBufferSize)
    {
      IOT5302W.Data.DataCount=0;
    }
		//-------------------存储数据
    memcpy(&IOT5302W.Data.Data[IOT5302W.Data.DataCount],IOT5302WRx,RxdLen); //保存接收到的数据
		//-------------------记录存储数据个数
    IOT5302W.Data.DataCount+=RxdLen; 
		//-------------------读卡器响应超时清零
		IOT5302W.Data.TimeOut_NACK	=	0;		//
  }
	//=====================未接收到读卡器的数据:做超时判断
  else if(IOT5302W.Data.TimeOut_NACK++>=IOT5302WReadTimeOut_NACK) //读卡器未响应超时时间
  {
		IOT5302W.Data.TimeOut_NACK	=	0;
		IOT5302W.Data.DataCount			=	0;
    IOT5302W.Data.Initialized 	= 0;  //初始化标志，0-未初始化成功，1-初始化成功
  }  
}
/*******************************************************************************
*函数名			:	IOT5302W_DataCheck
*功能描述		:	检查数据协议
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void iot5302w_data_process(void)
{
  if(0==IOT5302W.Data.DataCount)  //没有数据
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
			//-----------------------查起始码
      if(IotReaderSTX==IOT5302W.Data.Data[i]) //查找到头标识
      {
				//---------------------获取数据长度
        n=IOT5302W.Data.Data[i+2];
        if(i+2+n+1+1>IOT5302WBufferSize)  //超出缓存
        {
          IOT5302W.Data.DataCount=0;
        }
				//---------------------获取结束符
        if(IotReaderETX==IOT5302W.Data.Data[i+2+n+1+1]) //尾标识（2字节为ID和长度，n字节数据，1字节校验，1字节尾标识)
        {
          bcc=BCC8(&IOT5302W.Data.Data[i+1],n+2);   //计算BCC(从ID开始，包含CMD/STD位和n个数据
					//---------------------数据校验
          if(bcc==IOT5302W.Data.Data[i+2+n+1])//BCC字节（2字节为ID和长度，n字节数据，最后字节为BCC)
          {            
						if(0==IOT5302W.Data.Initialized)	//原为未初始化
						{
							IOT5302W.Data.Initialized = 1;  //初始化标志，0-未初始化成功，1-初始化成功
						}
						//---------------------获取UID
            if(0x06==IOT5302W.Data.Data[i+2]) //UID消息的数据长度
            {
              memcpy(IOT5302W.Data.UID,&IOT5302W.Data.Data[i+5],4); //读取到UID
              memset(IOT5302W.Data.Data,0x00,IOT5302WBufferSize);   //清除缓存
              IOT5302W.Data.DataCount=0;
							IOT5302W.Data.TimeOut_UID = 0;    	//未接收到UID计时清零
              return;
            }
          }
        }
      }
    }
		//-------------------未找到UID
    if(i>=IOT5302WBufferSize-2)
    {
      memset(IOT5302W.Data.Data,0x00,IOT5302WBufferSize);   //清除缓存
      IOT5302W.Data.DataCount=0;
    }
  }
	//--------------------------------------------超时未收到数据:清除UID
  if(IOT5302W.Data.TimeOut_UID++>IOT5302WReadTimeOut_UID)
  {
    memset(IOT5302W.Data.UID,0x00,4);   	//清除UID记录
		memset(IOT5302W.Data.UIDbac,0x00,4);	//清除UID记录--备份区
    IOT5302W.Data.TimeOut_UID = 0;    		//未接收到UID计时清零
  }
}
//------------------------------------------------------------------------------


//---------------------------------------------------------software-SetFrame
/*******************************************************************************
*函数名			:	iot5302w_GetFrame_GetSNR
*功能描述		:	生成获取UID的消息帧
							寻卡，获取卡的UID，每个M1卡都有一个唯一的序列号，我们称为“UID”，是32位的，也就是4个字节。
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned short iot5302w_GetFrame_GetSNR(unsigned char* Buffer)
{
  unsigned short FrameLen=8;
  
  Buffer[0]   = IotReaderSTX; //头标识
  Buffer[1]   = 0x00;         //设备地址
  Buffer[2]   = 0x03;         //长度：不包括STX,ETX,BCC,设备地址
  Buffer[3]   = GET_SNR;      //命令
  Buffer[4]   = 0x26;         //0x26：IDLE模式，只对一张卡操作
                              //0x52：ALL模式，可对多张卡操作
  Buffer[5]   = 00;           //0x00：读写器不需要执行halt
                              //0x01：读写器需要执行halt
  Buffer[6]  = BCC8(&Buffer[1],5);		//异或校验;
  Buffer[7]  = IotReaderETX;		//结束标志符
  
  return  FrameLen;
}
/*******************************************************************************
*函数名			:	iot5302w_set_frame_SetBaudrate
*功能描述		:	生成配置读卡器波特率的消息帧
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned short iot5302w_set_frame_SetBaudrate(unsigned char* Buffer,unsigned char BaudRateCode)
{
  unsigned short FrameLen=7;
  
  Buffer[0]   = IotReaderSTX; //头标识
  Buffer[1]   = 0x00;         //设备地址
  Buffer[2]   = 0x02;         //长度：不包括STX,ETX,BCC,设备地址
  Buffer[3]   = SetBaudrate;  //命令
  Buffer[4]   = BaudRateCode; //0x00：9600
                              //0x01：19200
                              //0x02：38400
                              //0x03：57600
                              //0x04：115200
                              //其它：9600
  Buffer[5]  = BCC8(&Buffer[1],4);		//异或校验;
  Buffer[6]  = IotReaderETX;		//结束标志符
  
  return  FrameLen;
}
//------------------------------------------------------------------------------



//---------------------------------------------------------hardware
/*******************************************************************************
*函数名			:	function
*功能描述		:	配置读卡器接口
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void hw_port_configuration(unsigned long USART_BaudRate)
{  
	//api_usart_dma_configurationNR(IOT5302W.Conf.IOT5302WPort.USARTx,USART_BaudRate,IOT5302WBufferSize);
	api_rs485_dma_configurationNR(&IOT5302W.Conf.IOT5302WPort,USART_BaudRate,IOT5302WBufferSize);	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned short iot5302w_send_msg(unsigned char* TxdBuffer,unsigned short length)
{
  unsigned short TxdLen = 0;
  TxdLen = api_rs485_dma_send(&IOT5302W.Conf.IOT5302WPort,TxdBuffer,length);	//RS485-DMA发送程序
  return TxdLen;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned short iot5302w_read_msg(unsigned char* RxdBuffer)
{
  unsigned short RxdLen = 0;
  RxdLen = api_rs485_dma_receive(&IOT5302W.Conf.IOT5302WPort,RxdBuffer);
  return RxdLen;
}
//------------------------------------------------------------------------------

//--------------------------------------------------------------------------------


//--------------------------------------V2配置为9600
///*******************************************************************************
//*函数名			:	IOT5302W_Configuration
//*功能描述		:	IOT5302W配置
//*输入				: pRS485使用的RS485结构体
//              USART_BaudRate希望配置成的波特率
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//void api_iot5302w_configuration(IOT5302Wdef* pIOT5302W)
//{
//  IOT5302W  = *pIOT5302W;
//  IOT5302W.Data.Initialized = 0;
//  IOT5302W.Data.Time        = 0;
//  IOT5302W.Data.TimeOut     = 0;
//  IOT5302W.Data.USART_BaudRate  = 0;
//  IOT5302W.Data.BaudRateSetStep = 0;
//	
//	api_rs485_dma_configurationNR(&IOT5302W.Conf.IOT5302WPort,9600,IOT5302WBufferSize);	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
//}
///*******************************************************************************
//*函数名			:	function
//*功能描述		:	function
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//void api_iot5302w_server(void)
//{
//	//先处理数据，再执行初始化，更改初始化参数后发现数据验证已通过
//  //==================================检查参数
//  if((0==IOT5302W.Conf.USART_BaudRate)||(0==IOT5302W.Conf.IOT5302WPort.USARTx))  //参数错误
//  {
//    return;
//  }
//	//==================================数据处理，协议检查
//  iot5302w_data_process();
//  //==================================接收拼包，协议检查
//  iot5302w_data_receive_process();
//  //==================================如果未初始化成功，不执行
//	iot5302w_initialize();
//	//==================================如果读卡器已初始化，则周期性去查询读卡器的数据
//	iot5302w_get_data();								
//}
////------------------------------------------------------------------------------


///*******************************************************************************
//*函数名			:	iot5302w_get_data
//*功能描述		:	如果读卡器已初始化，则周期性去查询读卡器的数据
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//static void iot5302w_get_data(void)
//{
//	if(0==IOT5302W.Data.Initialized)  	//未初始化成功
//  {
//    return;
//  }
//	IOT5302W.Data.TimeGetData++;
//	if(IOT5302W.Data.TimeGetData%IOT5302WReadCycle==0)   //0.3秒扫描一次
//  {
//    unsigned char TxdLen  = 0;
//    IOT5302W.Data.TimeGetData=0; 		//清零
//    TxdLen  = iot5302w_set_frame_GetSNR(CmdBuffer);     	//生成获取UID的消息帧
//    TxdLen  = iot5302w_send_msg(CmdBuffer,TxdLen);	      //寻卡，获取卡的UID，每个M1卡都有一个唯一的序列号，我们称为“UID”，是32位的，也就是4个字节。
//  }
//}
///*******************************************************************************
//*函数名			:	function
//*功能描述		:	function
//*输入				: 
//*返回值			:	4字节卡片UID
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//unsigned short api_get_iot5302w_uid(unsigned char* Buffer)
//{
//  unsigned char i=0;
//  unsigned char n=0;
//	static unsigned short no_data_wait_time	=	0;
//  for(i=0;i<4;i++)
//  {
//    if(IOT5302W.Data.UID[i])	//检查UID是否为0
//      n++;
//  }
//	//-------------------------有数据
//  if(0!=n)  //有数据
//  {
//		no_data_wait_time	=	0;
//		//-----------------------新数据:立即上传
//    if(memcmp(IOT5302W.Data.UID,IOT5302W.Data.UIDbac,4)) //对比两组数据不一样
//    {
//      memcpy(Buffer,IOT5302W.Data.UID,4);
//      memcpy(IOT5302W.Data.UIDbac,IOT5302W.Data.UID,4);
//      memset(IOT5302W.Data.UID,0x00,4);			//清除数据
//      IOT5302W.Data.TimeUIDCmp = 0;
//      return 4; //4字节UID
//    }
//		//-----------------------相同新数据:间隔2秒上传一次
//    else if(IOT5302W.Data.TimeUIDCmp++>2000)		//相同的数据，2秒钟上传一次
//    {
//      memcpy(Buffer,IOT5302W.Data.UID,4);
//      IOT5302W.Data.TimeUIDCmp = 0;
//      return 4; //4字节UID
//    }
//  }
////  else
////  {
////		if(no_data_wait_time++>300)	//200ms
////		{
////			no_data_wait_time	=	0;
////			memset(IOT5302W.Data.UIDbac,0x00,4);
////		}    
////  }
//  return 0;
//}
////------------------------------------------------------------------------------


////---------------------------------------------------------hardware
///*******************************************************************************
//*函数名			:	function
//*功能描述		:	配置读卡器接口
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//static void hw_port_configuration(unsigned long USART_BaudRate)
//{  
//	//api_usart_dma_configurationNR(IOT5302W.Conf.IOT5302WPort.USARTx,USART_BaudRate,IOT5302WBufferSize);
//	api_rs485_dma_configurationNR(&IOT5302W.Conf.IOT5302WPort,USART_BaudRate,IOT5302WBufferSize);	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
//}
///*******************************************************************************
//*函数名			:	function
//*功能描述		:	function
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//static unsigned short iot5302w_send_msg(unsigned char* TxdBuffer,unsigned short length)
//{
//  unsigned short TxdLen = 0;
//  TxdLen = api_rs485_dma_send(&IOT5302W.Conf.IOT5302WPort,TxdBuffer,length);	//RS485-DMA发送程序
//  return TxdLen;
//}
///*******************************************************************************
//*函数名			:	function
//*功能描述		:	function
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//static unsigned short iot5302w_read_msg(unsigned char* RxdBuffer)
//{
//  unsigned short RxdLen = 0;
//  RxdLen = api_rs485_dma_receive(&IOT5302W.Conf.IOT5302WPort,RxdBuffer);
//  return RxdLen;
//}
////------------------------------------------------------------------------------


////---------------------------------------------------------software
///*******************************************************************************
//*函数名			:	iot5302w_initialize
//*功能描述		:	以不同的波特率发送读卡器波特率配置命令，检查返回
//							根据读卡器的响应时间，时间间隔可以设置为200ms
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//static void iot5302w_initialize(void)
//{  
//	unsigned char TxdLen  = 0;
//	unsigned char BaudRateCode  = 0;
//	//---------------------读卡器已初始化，则退出
//  if(0!=IOT5302W.Data.Initialized)  //初始化成功
//  {
//    return;
//  }
//  //---------------------0.3秒设置一次波特率,将波特率设置为19200
//  if(IOT5302W.Data.Time_Initialized++<300)
//  {
//   return; 
//  }
//	IOT5302W.Data.Time_Initialized  = 0;	
//  //---------------------选择相应的测试波特率	
//   
//	if(0==IOT5302W.Data.BaudRateSetStep)
//	{
//		IOT5302W.Data.USART_BaudRate=9600;
//		BaudRateCode=0;
//	}
//	else if(1==IOT5302W.Data.BaudRateSetStep)
//	{
//		IOT5302W.Data.USART_BaudRate=19200;
//		BaudRateCode=1;
//	}
//	else if(2==IOT5302W.Data.BaudRateSetStep)
//	{
//		IOT5302W.Data.USART_BaudRate=38400;
//		BaudRateCode=2;
//	}
//	else if(3==IOT5302W.Data.BaudRateSetStep)
//	{
//		IOT5302W.Data.USART_BaudRate=57600;
//		BaudRateCode=3;
//	}
//	else if(4==IOT5302W.Data.BaudRateSetStep)
//	{
//		IOT5302W.Data.USART_BaudRate=115200;
//		BaudRateCode=4;
//	}
//	else
//	{
//		IOT5302W.Data.USART_BaudRate=IOT5302W.Conf.USART_BaudRate;
//		IOT5302W.Data.BaudRateSetStep = 0;
//	}
//	//---------------------将串口配置到相应的波特率
//	hw_port_configuration(IOT5302W.Data.USART_BaudRate);			//配置波特率，根据当前波特率与读卡器通讯是否成功
//	//---------------------获取设置读卡器波特率消息帧
//	TxdLen  = iot5302w_set_frame_SetBaudrate(CmdBuffer,0x00);  	//获取设置读卡器波特率消息帧9600
//	//---------------------RS485-DMA发送程序
//	TxdLen  = iot5302w_send_msg(CmdBuffer,TxdLen);	      							//RS485-DMA发送程序
//	//---------------------下一组配置步骤
//	if(4==BaudRateCode)
//	{
//		IOT5302W.Data.BaudRateSetStep=0;
//	}
//	else
//	{
//		IOT5302W.Data.BaudRateSetStep += 1;
//	}
//}
///*******************************************************************************
//*函数名			:	iot5302w_data_receive_process
//*功能描述		:	数据处理，接收拼包，协议检查
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//static void iot5302w_data_receive_process(void)
//{
//  unsigned char IOT5302WRx[64];
//  unsigned short RxdLen = 0;  
//  RxdLen  = iot5302w_read_msg(IOT5302WRx);
//  if(RxdLen)
//  {
//    if(IOT5302W.Data.DataCount+RxdLen>IOT5302WBufferSize)
//    {
//      IOT5302W.Data.DataCount=0;
//      memset(IOT5302W.Data.UID,0x00,4);   //清除UID记录
//    }
//    memcpy(&IOT5302W.Data.Data[IOT5302W.Data.DataCount],IOT5302WRx,RxdLen); //保存接收到的数据
//    IOT5302W.Data.DataCount+=RxdLen; 

//		IOT5302W.Data.TimeOut_NACK	=	0;		//
//  }
//  else if(IOT5302W.Data.TimeOut_NACK++>=IOT5302WReadTimeOut_NACK) //读卡器未响应超时时间
//  {
//		IOT5302W.Data.TimeOut_NACK	=	0;
//		IOT5302W.Data.DataCount			=	0;
//    IOT5302W.Data.Initialized 	= 0;  //初始化标志，0-未初始化成功，1-初始化成功
//		IOT5302W.Data.TimeOut = 0;    			//超时计时清除
//  }  
//}
///*******************************************************************************
//*函数名			:	IOT5302W_DataCheck
//*功能描述		:	检查数据协议
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//static void iot5302w_data_process(void)
//{
//  if(0==IOT5302W.Data.DataCount)  //没有数据
//  {
//    return;
//  }
//  else
//  {
//    unsigned char i=0;
//    unsigned char n=0;
//    unsigned char bcc=0;
//    for(i=0;i<IOT5302W.Data.DataCount;i++)
//    {
//			//-----------------------查找帧头
//      if(IotReaderSTX==IOT5302W.Data.Data[i]) //查找到头标识
//      {
//        n=IOT5302W.Data.Data[i+2];
//        if(i+2+n+1+1>IOT5302WBufferSize)  //超出缓存
//        {
//          IOT5302W.Data.DataCount=0;
//        }
//        if(IotReaderETX==IOT5302W.Data.Data[i+2+n+1+1]) //尾标识（2字节为ID和长度，n字节数据，1字节校验，1字节尾标识)
//        {
//          bcc=BCC8(&IOT5302W.Data.Data[i+1],n+2);   //计算BCC(从ID开始，包含CMD/STD位和n个数据
//          if(bcc==IOT5302W.Data.Data[i+2+n+1])//BCC字节（2字节为ID和长度，n字节数据，最后字节为BCC)
//          {
//            
//						if(0==IOT5302W.Data.Initialized)	//原为未初始化
//						{
//							IOT5302W.Data.Initialized = 1;  //初始化标志，0-未初始化成功，1-初始化成功
//							IOT5302W.Data.USART_BaudRate	=	9600;
//							hw_port_configuration(IOT5302W.Data.USART_BaudRate);			//配置波特率，根据当前波特率与读卡器通讯是否成功
//						}
//            if(0x06==IOT5302W.Data.Data[i+2]) //UID消息的数据长度
//            {
//              memcpy(IOT5302W.Data.UID,&IOT5302W.Data.Data[i+5],4); //读取到UID
//              memset(IOT5302W.Data.Data,0x00,IOT5302WBufferSize);   //清除缓存
//              IOT5302W.Data.DataCount=0;
//              IOT5302W.Data.TimeOut = 0;    			//超时计时清除
//							IOT5302W.Data.TimeOut_UID = 0;    	//未接收到UID计时清零
//              return;
//            }
//          }
//        }
//      }
//    }
//    if(i>=IOT5302WBufferSize-2)//未找到UID
//    {
//      memset(IOT5302W.Data.Data,0x00,IOT5302WBufferSize);   //清除缓存
//      IOT5302W.Data.DataCount=0;
//    }
//  }
//	//--------------------------------------------超时未收到数据:清除UID
//  if(IOT5302W.Data.TimeOut_UID++>IOT5302WReadTimeOut_UID)
//  {
//    memset(IOT5302W.Data.UID,0x00,4);   	//清除UID记录
//		memset(IOT5302W.Data.UIDbac,0x00,4);	//清除UID记录--备份区
//    IOT5302W.Data.TimeOut_UID = 0;    		//未接收到UID计时清零
//  }
//}
////------------------------------------------------------------------------------


////---------------------------------------------------------software-SetFrame
///*******************************************************************************
//*函数名			:	iot5302w_set_frame_GetSNR
//*功能描述		:	生成获取UID的消息帧
//							寻卡，获取卡的UID，每个M1卡都有一个唯一的序列号，我们称为“UID”，是32位的，也就是4个字节。
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//static unsigned short iot5302w_set_frame_GetSNR(unsigned char* Buffer)
//{
//  unsigned short FrameLen=8;
//  
//  Buffer[0]   = IotReaderSTX; //头标识
//  Buffer[1]   = 0x00;         //设备地址
//  Buffer[2]   = 0x03;         //长度：不包括STX,ETX,BCC,设备地址
//  Buffer[3]   = GET_SNR;      //命令
//  Buffer[4]   = 0x26;         //0x26：IDLE模式，只对一张卡操作
//                              //0x52：ALL模式，可对多张卡操作
//  Buffer[5]   = 00;           //0x00：读写器不需要执行halt
//                              //0x01：读写器需要执行halt
//  Buffer[6]  = BCC8(&Buffer[1],5);		//异或校验;
//  Buffer[7]  = IotReaderETX;		//结束标志符
//  
//  return  FrameLen;
//}
///*******************************************************************************
//*函数名			:	iot5302w_set_frame_SetBaudrate
//*功能描述		:	生成配置读卡器波特率的消息帧
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//static unsigned short iot5302w_set_frame_SetBaudrate(unsigned char* Buffer,unsigned char BaudRateCode)
//{
//  unsigned short FrameLen=7;
//  
//  Buffer[0]   = IotReaderSTX; //头标识
//  Buffer[1]   = 0x00;         //设备地址
//  Buffer[2]   = 0x02;         //长度：不包括STX,ETX,BCC,设备地址
//  Buffer[3]   = SetBaudrate;  //命令
//  Buffer[4]   = BaudRateCode; //0x00：9600
//                              //0x01：19200
//                              //0x02：38400
//                              //0x03：57600
//                              //0x04：115200
//                              //其它：9600
//  Buffer[5]  = BCC8(&Buffer[1],4);		//异或校验;
//  Buffer[6]  = IotReaderETX;		//结束标志符
//  
//  return  FrameLen;
//}
////--------------------------------------------------------------------------------




//==========================================V1配置为19200
///*******************************************************************************
//*函数名			:	IOT5302W_Configuration
//*功能描述		:	IOT5302W配置
//*输入				: pRS485使用的RS485结构体
//              USART_BaudRate希望配置成的波特率
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//void api_iot5302w_configuration(IOT5302Wdef* pIOT5302W)
//{
//  IOT5302W  = *pIOT5302W;
//  IOT5302W.Data.Initialized = 0;
//  IOT5302W.Data.Time        = 0;
//  IOT5302W.Data.TimeOut     = 0;
//  IOT5302W.Data.USART_BaudRate  = 0;
//  IOT5302W.Data.BaudRateSetStep = 0;
//	
//	api_rs485_dma_configurationNR(&IOT5302W.Conf.IOT5302WPort,9600,IOT5302WBufferSize);	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
//}
///*******************************************************************************
//*函数名			:	function
//*功能描述		:	function
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//void api_iot5302w_server(void)
//{ 
//  //==================================检查参数
//  if((0==IOT5302W.Conf.USART_BaudRate)||(0==IOT5302W.Conf.IOT5302WPort.USARTx))  //参数错误
//  {
//    return;
//  }
//  
//  //==================================数据处理，接收拼包，协议检查
//  iot5302w_data_process();
//  //==================================如果未初始化成功，不执行
//  if(0==IOT5302W.Data.Initialized)  //未初始化成功
//  {
//    iot5302w_initialize();    //读卡器初始化
//    return;
//  }
//  IOT5302W.Data.Time++;
//  //==================================周期扫描数据
//  if(IOT5302W.Data.Time%IOT5302WReadCycle==0)   //0.3秒扫描一次
//  {
//    unsigned char TxdLen  = 0;
//    IOT5302W.Data.Time=0; //清零
//    TxdLen  = iot5302w_set_frame_GetSNR(CmdBuffer);     	//生成获取UID的消息帧
//    TxdLen  = iot5302w_send_msg(CmdBuffer,TxdLen);	      //寻卡，获取卡的UID，每个M1卡都有一个唯一的序列号，我们称为“UID”，是32位的，也就是4个字节。
//  }
//}
////------------------------------------------------------------------------------



///*******************************************************************************
//*函数名			:	function
//*功能描述		:	function
//*输入				: 
//*返回值			:	4字节卡片UID
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//unsigned short api_get_iot5302w_uid(unsigned char* Buffer)
//{
//  unsigned char i=0;
//  unsigned char n=0;
//	static unsigned short no_data_wait_time	=	0;
//  for(i=0;i<4;i++)
//  {
//    if(IOT5302W.Data.UID[i])
//      n++;
//  }
//  if(0!=n)  //有数据
//  {
//		no_data_wait_time	=	0;
//    if(memcmp(IOT5302W.Data.UID,IOT5302W.Data.UIDbac,4)) //对比两组数据不一样
//    {
//      memcpy(Buffer,IOT5302W.Data.UID,4);
//      memcpy(IOT5302W.Data.UIDbac,IOT5302W.Data.UID,4);
//      memset(IOT5302W.Data.UID,0x00,4);			//清除数据
//      IOT5302W.Data.TimeCmp = 0;
//      return 4; //4字节UID
//    }
//    else if(IOT5302W.Data.TimeCmp++>2000)		//相同的数据，2秒钟上传一次
//    {
//      memcpy(Buffer,IOT5302W.Data.UID,4);
//      IOT5302W.Data.TimeCmp = 0;
//      return 4; //4字节UID
//    }
//  }
//  else
//  {
//		if(no_data_wait_time++>300)	//200ms
//		{
//			no_data_wait_time	=	0;
//			memset(IOT5302W.Data.UIDbac,0x00,4);
//		}    
//  }
//  return 0;
//}
////------------------------------------------------------------------------------


////---------------------------------------------------------hardware
///*******************************************************************************
//*函数名			:	function
//*功能描述		:	配置读卡器接口
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//static void hw_port_configuration(unsigned long USART_BaudRate)
//{  
//	//api_usart_dma_configurationNR(IOT5302W.Conf.IOT5302WPort.USARTx,USART_BaudRate,IOT5302WBufferSize);
//	api_rs485_dma_configurationNR(&IOT5302W.Conf.IOT5302WPort,USART_BaudRate,IOT5302WBufferSize);	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
//}
///*******************************************************************************
//*函数名			:	function
//*功能描述		:	function
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//static unsigned short iot5302w_send_msg(unsigned char* TxdBuffer,unsigned short length)
//{
//  unsigned short TxdLen = 0;
//  TxdLen = api_rs485_dma_send(&IOT5302W.Conf.IOT5302WPort,TxdBuffer,length);	//RS485-DMA发送程序
//  return TxdLen;
//}
///*******************************************************************************
//*函数名			:	function
//*功能描述		:	function
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//static unsigned short iot5302w_read_msg(unsigned char* RxdBuffer)
//{
//  unsigned short RxdLen = 0;
//  RxdLen = api_rs485_dma_receive(&IOT5302W.Conf.IOT5302WPort,RxdBuffer);
//  return RxdLen;
//}
////------------------------------------------------------------------------------


////---------------------------------------------------------software
///*******************************************************************************
//*函数名			:	iot5302w_initialize
//*功能描述		:	以不同的波特率发送读卡器波特率配置命令，检查返回
//							根据读卡器的响应时间，时间间隔可以设置为200ms
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//static void iot5302w_initialize(void)
//{  
//  iot5302w_data_process(); 					//数据处理，接收拼包，协议检查
//  if(0!=IOT5302W.Data.Initialized)  //初始化成功
//  {
//    return;
//  }
//  //---------------------1秒设置一次波特率,将波特率设置为19200
//  if(IOT5302W.Data.Time>=IOT5302WInitializeTimeOut)
//  {
//    IOT5302W.Data.Time  = 0;
//  }
//  //---------------------选择相应的测试波特率
//  if(IOT5302W.Data.Time%IOT5302WReadCycle==0)
//  {    
//    if(0==IOT5302W.Data.BaudRateSetStep)
//    {
//      IOT5302W.Data.USART_BaudRate=9600;
//    }
//    else if(1==IOT5302W.Data.BaudRateSetStep)
//    {
//      IOT5302W.Data.USART_BaudRate=19200;
//    }
//    else if(2==IOT5302W.Data.BaudRateSetStep)
//    {
//      IOT5302W.Data.USART_BaudRate=38400;
//    }
//    else if(3==IOT5302W.Data.BaudRateSetStep)
//    {
//      IOT5302W.Data.USART_BaudRate=57600;
//    }
//    else if(4==IOT5302W.Data.BaudRateSetStep)
//    {
//      IOT5302W.Data.USART_BaudRate=115200;
//    }
//    else
//    {
//      IOT5302W.Data.USART_BaudRate=IOT5302W.Conf.USART_BaudRate;
//      IOT5302W.Data.BaudRateSetStep = 0;
//    }
//    hw_port_configuration(IOT5302W.Data.USART_BaudRate);			//配置波特率，根据当前波特率与读卡器通讯是否成功
//    IOT5302W.Data.Time  = 0;
//  }
//  //---------------------按照上面的硬件接口波特率0.5秒发送一次配置读卡器波特率命令，
//  if((100==IOT5302W.Data.Time))
//  {
//    
//    unsigned char TxdLen  = 0;
//    unsigned char BaudRateCode  = 0;
//    switch(IOT5302W.Conf.USART_BaudRate) 	//根据实际应用需要的波特率设置获取设置读卡器波特率的代码
//    {
//      case 9600:BaudRateCode=0;
//      break;
//      case 19200:BaudRateCode=1;
//      break;
//      case 38400:BaudRateCode=2;
//      break;
//      case 57600:BaudRateCode=3;
//      break;
//      case 115200:BaudRateCode=4;
//      break;
//      default:BaudRateCode=0; //默认配置
//      break;
//    }
//    TxdLen  = iot5302w_set_frame_SetBaudrate(CmdBuffer,BaudRateCode);  //设置读卡器波特率
//    TxdLen  = iot5302w_send_msg(CmdBuffer,TxdLen);	      //RS485-DMA发送程序
//    IOT5302W.Data.BaudRateSetStep += 1;
//  }
//  IOT5302W.Data.Time++;
//}
///*******************************************************************************
//*函数名			:	IOT5302W_DataProcess
//*功能描述		:	数据处理，接收拼包，协议检查
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//static void iot5302w_data_process(void)
//{
//  unsigned char IOT5302WRx[64];
//  unsigned short RxdLen = 0;  
//  RxdLen  = iot5302w_read_msg(IOT5302WRx);
//  if(RxdLen)
//  {
//    if(IOT5302W.Data.DataCount+RxdLen>IOT5302WBufferSize)
//    {
//      IOT5302W.Data.DataCount=0;
//      memset(IOT5302W.Data.UID,0x00,4);   //清除UID记录
//      IOT5302W.Data.TimeOut = 0;    			//超时计时清除
//    }
//    memcpy(&IOT5302W.Data.Data[IOT5302W.Data.DataCount],IOT5302WRx,RxdLen); //保存接收到的数据
//    IOT5302W.Data.DataCount+=RxdLen;    
//  }
//  else if(IOT5302W.Data.TimeOut++>=IOT5302WReadTimeOut) //超时：无读卡器或者波特率不对
//  {
//    IOT5302W.Data.Initialized = 0;  //初始化标志，0-未初始化成功，1-初始化成功
//  }
//  iot5302w_data_check();
//}
///*******************************************************************************
//*函数名			:	IOT5302W_DataCheck
//*功能描述		:	检查数据协议
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//static void iot5302w_data_check(void)
//{
//  if(0==IOT5302W.Data.DataCount)  //没有数据
//  {
//    return;
//  }
//  else
//  {
//    unsigned char i=0;
//    unsigned char n=0;
//    unsigned char bcc=0;
//    for(i=0;i<IOT5302W.Data.DataCount;i++)
//    {
//      if(IotReaderSTX==IOT5302W.Data.Data[i]) //查找到头标识
//      {
//        n=IOT5302W.Data.Data[i+2];
//        if(i+2+n+1+1>IOT5302WBufferSize)  //超出缓存
//        {
//          IOT5302W.Data.DataCount=0;
//        }
//        if(IotReaderETX==IOT5302W.Data.Data[i+2+n+1+1]) //尾标识（2字节为ID和长度，n字节数据，1字节校验，1字节尾标识)
//        {
//          bcc=BCC8(&IOT5302W.Data.Data[i+1],n+2);   //计算BCC(从ID开始，包含CMD/STD位和n个数据
//          if(bcc==IOT5302W.Data.Data[i+2+n+1])//BCC字节（2字节为ID和长度，n字节数据，最后字节为BCC)
//          {
//            IOT5302W.Data.Initialized = 1;  //初始化标志，0-未初始化成功，1-初始化成功
//            if(0x06==IOT5302W.Data.Data[i+2]) //UID消息的数据长度
//            {
//              memcpy(IOT5302W.Data.UID,&IOT5302W.Data.Data[i+5],4); //读取到UID
//              memset(IOT5302W.Data.Data,0x00,IOT5302WBufferSize);   //清除缓存
//              IOT5302W.Data.DataCount=0;
//              IOT5302W.Data.TimeOut = 0;    //超时计时清除
//              return;
//            }
//          }
//        }
//      }
//    }
//    if(i>=IOT5302WBufferSize-2)//未找到UID
//    {
//      memset(IOT5302W.Data.Data,0x00,IOT5302WBufferSize);   //清除缓存
//      IOT5302W.Data.DataCount=0;
//    }
//  }
//  if(IOT5302W.Data.TimeOut++>IOT5302WReadTimeOut)
//  {
//    memset(IOT5302W.Data.UID,0x00,4);   //清除UID记录
//    IOT5302W.Data.TimeOut = 0;    			//超时计时清除
//  }
//}
////------------------------------------------------------------------------------


////---------------------------------------------------------software-SetFrame
///*******************************************************************************
//*函数名			:	iot5302w_set_frame_GetSNR
//*功能描述		:	生成获取UID的消息帧
//							寻卡，获取卡的UID，每个M1卡都有一个唯一的序列号，我们称为“UID”，是32位的，也就是4个字节。
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//static unsigned short iot5302w_set_frame_GetSNR(unsigned char* Buffer)
//{
//  unsigned short FrameLen=8;
//  
//  Buffer[0]   = IotReaderSTX; //头标识
//  Buffer[1]   = 0x00;         //设备地址
//  Buffer[2]   = 0x03;         //长度：不包括STX,ETX,BCC,设备地址
//  Buffer[3]   = GET_SNR;      //命令
//  Buffer[4]   = 0x26;         //0x26：IDLE模式，只对一张卡操作
//                              //0x52：ALL模式，可对多张卡操作
//  Buffer[5]   = 00;           //0x00：读写器不需要执行halt
//                              //0x01：读写器需要执行halt
//  Buffer[6]  = BCC8(&Buffer[1],5);		//异或校验;
//  Buffer[7]  = IotReaderETX;		//结束标志符
//  
//  return  FrameLen;
//}
///*******************************************************************************
//*函数名			:	iot5302w_set_frame_SetBaudrate
//*功能描述		:	生成配置读卡器波特率的消息帧
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//static unsigned short iot5302w_set_frame_SetBaudrate(unsigned char* Buffer,unsigned char BaudRateCode)
//{
//  unsigned short FrameLen=7;
//  
//  Buffer[0]   = IotReaderSTX; //头标识
//  Buffer[1]   = 0x00;         //设备地址
//  Buffer[2]   = 0x02;         //长度：不包括STX,ETX,BCC,设备地址
//  Buffer[3]   = SetBaudrate;  //命令
//  Buffer[4]   = BaudRateCode; //0x00：9600
//                              //0x01：19200
//                              //0x02：38400
//                              //0x03：57600
//                              //0x04：115200
//                              //其它：9600
//  Buffer[5]  = BCC8(&Buffer[1],4);		//异或校验;
//  Buffer[6]  = IotReaderETX;		//结束标志符
//  
//  return  FrameLen;
//}
//------------------------------------------------------------------------------




