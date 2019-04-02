#include	"AMP_Protocol.H"

#include	"CRC.H"		//

#include	"stdio.h"			//用于printf
#include	"string.h"		//用于printf
#include	"stdarg.h"		//用于获取不确定个数的参数
#include	"stdlib.h"		//malloc动态申请内存空间

#include	"stdbool.h"


/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
ampphydef* api_get_frame(unsigned char* pbuffer,unsigned short len)
{
	unsigned short	msg_len			=	0;
	unsigned short	frame_len	 	=	len;    //当前缓存数据长度
  unsigned short	DataValidLength	    =	len;    //当前缓存数据长度
	unsigned short	crclen			=	0;
	unsigned short	temp				=	0;
	
	unsigned  char* headaddr    	= pbuffer;
	unsigned	char* CrcStartAddr	=	NULL;
  
	ampcrc16def	crc16;
  ampphydef* ampframe;
  
  //=====================基本检查(空地址或者长度不足最小帧)
  if(NULL  ==  headaddr||DataValidLength<7)
  {
    return  NULL;   //退出此函数--空地址或者长度不够
  }
  FrameGetHeadCodeAddr:
  //-------------------------------------------查找头标识地址
  headaddr	=	(unsigned char*)memchr(headaddr,headcode,DataValidLength);   //找头标识
  if(NULL==headaddr)
  {
    return	NULL;   	//退出此函数--未找到头/尾标识符
  }  
  //-------------------------------------------剩余有效数据长度
  DataValidLength  = DataValidLength-((unsigned long)headaddr-(unsigned long)pbuffer); //剩余数据长度
  if(DataValidLength<7)
  {
    return	NULL;   //退出此函数--帧长度不够
  }
	//-------------------------------------------数据校验
	ampframe			=	(ampphydef*)headaddr;
	frame_len			=	ampframe->msg.length+5;	//完整帧长度，head,length,crc16,end为5个字节
	if(frame_len>DataValidLength)
	{
		//数据溢出
		return	NULL;
	}
	msg_len				=	ampframe->msg.length;		//不包含length位
	crclen				=	msg_len+1;							//包含length位
	CrcStartAddr	=	&ampframe->msg.length;	
	crc16.crcl		=	CrcStartAddr[crclen+0];
	crc16.crch		=	CrcStartAddr[crclen+1];
	temp					=	CRC16_MODBUS(CrcStartAddr,crclen);
	
	if(((temp&0xFF)==crc16.crcl)&&(((temp>>8)&0xFF)==crc16.crch))
  {
    return ampframe;
  }
	else
	{
		headaddr	=	&headaddr[1];
		DataValidLength	-=1;				//去掉headaddr[0]长度
		goto FrameGetHeadCodeAddr;	//重新检测，直到成功或者所有的数据检测完
	} 
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
unsigned short api_set_frame(ampphydef* frame,ampecmddef cmd,ampedirdef dir)
{
	unsigned short framlen	=0;
	unsigned short msglen		=0;
	unsigned short crclen		=0;
	unsigned short crc16		=0;
	
	unsigned	char* CrcStartAddr	=	&frame->msg.length;	//从len开始计算CRC
	
	msglen	=	frame->msg.length;
	crclen	=	msglen+1;
	
	frame->msg.cmd.cmd	=	cmd;
	frame->msg.cmd.rv		=	0;
	if(dir)
		frame->msg.cmd.dir	=	1;
	else
		frame->msg.cmd.dir	=	0;
	
	crc16	=	CRC16_MODBUS(CrcStartAddr,crclen);	//计算crc
	
	CrcStartAddr[crclen+0]=crc16&0xFF;	//CRC低8位
	CrcStartAddr[crclen+1]=(crc16>>8)&0xFF;	//CRC高8位	
	CrcStartAddr[crclen+2]	=	endcode;	//设置结束符
	
	framlen	=	msglen+1+2+2;		//len,crc,head,end
	return framlen;
}
