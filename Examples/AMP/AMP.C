#include	"AMP.H"

#include	"CRC.H"		//

#include	"stdio.h"			//用于printf
#include	"string.h"		//用于printf
#include	"stdarg.h"		//用于获取不确定个数的参数
#include	"stdlib.h"		//malloc动态申请内存空间

#include	"stdbool.h"

typedef struct _ampword
{
	unsigned char lsb;		//低8位
	unsigned char msb;   //不包含此位
}ampworddef;

typedef struct _protocol
{
	unsigned char head;
	unsigned char length;   //不包含此位
	unsigned char	cmd;
	struct
	{
		unsigned char addr1;
		unsigned char addr2;
		unsigned char addr3;
	}addr;
	unsigned char data[256];
}protocoldef;


unsigned  char  ackupfarme[7]=
{
  0x7E,   //头起始符
  0x02,   //长度
  0x81,   //CMD=1：应答，DIR=1：上传
  0x00,   //状态
  0xB0,   //CRC16L
  0x50,   //CRC16H
  0x7F    //尾结束符
};
unsigned  char  ackdownfarme[7]=
{
  0x7E,
  0x02,
  0x01,
  0x00,
  0xD1,
  0x90,
  0x7F
};

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short api_get_frame(unsigned char* pbuffer,unsigned char* frame,unsigned short len)
{
	unsigned short	msg_len			=	0;
	unsigned short	frame_len	 	=	len;    //当前缓存数据长度
  unsigned short	DataValidLength	    =	len;    //当前缓存数据长度
	unsigned short	crclen			=	0;
	unsigned short	temp				=	0;
	
	unsigned  char* headaddr    	= pbuffer;
	unsigned	char* CrcStartAddr	=	NULL;
  
	ampworddef		crc16;
  protocoldef* 	ampframe;
  
  //=====================基本检查(空地址或者长度不足最小帧)
  if(NULL  ==  headaddr||DataValidLength<7)
  {
    return  NULL;   //退出此函数--空地址或者长度不够
  }
	if(NULL	==	frame)	//接收地址为空
		return NULL;
  FrameGetHeadCodeAddr:
  //-------------------------------------------查找头标识地址
  headaddr	=	(unsigned char*)memchr(headaddr,0x7E,DataValidLength);   //找头标识
  if(NULL==headaddr)
  {
    return 0;   	//退出此函数--未找到头/尾标识符
  }  
  //-------------------------------------------剩余有效数据长度
  DataValidLength  = DataValidLength-((unsigned long)headaddr-(unsigned long)pbuffer); //剩余数据长度
  if(DataValidLength<7)
  {
    return  0;   //退出此函数--帧长度不够
  }
	//-------------------------------------------数据校验
	ampframe			=	(protocoldef*)headaddr;
	frame_len			=	ampframe->length+5;	//完整帧长度，head,length,crc16,end为5个字节
	if(frame_len>DataValidLength)
	{
		//数据溢出
		return 0;
	}
	msg_len				=	ampframe->length;		//不包含length位
	crclen				=	msg_len+1;							//包含length位
	CrcStartAddr	=	&ampframe->length;	
	crc16.lsb		=	CrcStartAddr[crclen+1];
	crc16.msb		=	CrcStartAddr[crclen+2];
	temp					=	CRC16_MODBUS(CrcStartAddr,crclen);
	
	if(((temp&0xFF)==crc16.lsb)&&(((temp>>8)&0xFF)==crc16.msb))
  {
		memcpy(frame,headaddr,frame_len);
    return frame_len;
  }
	else
	{
		headaddr	=	&headaddr[1];
		DataValidLength	-=1;				//去掉headaddr[0]长度
		goto FrameGetHeadCodeAddr;	//重新检测，直到成功或者所有的数据检测完
	} 
}


