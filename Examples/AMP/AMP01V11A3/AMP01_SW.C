#include "AMP01_SW.H"

//#include	"AMP_PHY.H"
#include 	"CRC.H"
#include	"string.h"				//串和内存操作函数头文件


static unsigned char layer_id		=	0;	//板层级:0-未定义，1-柜板，2-LCD
static unsigned char board_id1	=	0;	//板地址:如果是柜控制板就是板地址，如果是LCD板为层地址
static unsigned char board_id2	=	0;	//板地址:如果是柜控制板无效，如果LCD板为位地址


static void set_board_id(unsigned char layer_id,unsigned char id1,unsigned char id2);
static unsigned short get_frame(unsigned char* pbuffer,unsigned char* frame,unsigned short len);

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_pc_data_process(unsigned char* pbuffer,unsigned short len)
{
	unsigned short framelen=0;
	ampphydef ampframe;
	framelen	= get_frame(pbuffer,(unsigned char*)&ampframe,len);
	
	//-----------------------------------------------------------------------------
	if(0==framelen)		//帧错误
		return;	
	if(0==layer_id)		//未定义ID
		return;
	if(1==layer_id)		//柜控制板
	{
		if((board_id1!=ampframe.msg.addr.address1)&&(0xFF!=ampframe.msg.addr.address1))
		{
			return;
		}
	}
	else if(2==layer_id)		//LCD板
	{
		if((board_id1!=ampframe.msg.addr.address2)&&(0xFF!=ampframe.msg.addr.address2))
		{
			return;
		}
		else if((board_id2!=ampframe.msg.addr.address3)&&(0xFF!=ampframe.msg.addr.address3))
		{
			return;
		}
	}
	else
	{
		return;
	}
	//-----------------------------------------------------------------------------
}
//-----------------------------------------------------------------------------











//-----------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void set_board_id(unsigned char layer_id,unsigned char id1,unsigned char id2)
{
	if(0==layer_id)
		return;
	else if(1==layer_id)
		board_id1	=	id1;
	else if(2==layer_id)
	{
		board_id1	=	id1;
		board_id2	=	id2;
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
static unsigned short get_frame(unsigned char* pbuffer,unsigned char* frame,unsigned short len)
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
    return 0;   	//退出此函数--未找到头/尾标识符
  }  
  //-------------------------------------------剩余有效数据长度
  DataValidLength  = DataValidLength-((unsigned long)headaddr-(unsigned long)pbuffer); //剩余数据长度
  if(DataValidLength<7)
  {
    return  0;   //退出此函数--帧长度不够
  }
	//-------------------------------------------数据校验
	ampframe			=	(ampphydef*)headaddr;
	frame_len			=	ampframe->msg.length+5;	//完整帧长度，head,length,crc16,end为5个字节
	if(frame_len>DataValidLength)
	{
		//数据溢出
		return 0;
	}
	msg_len				=	ampframe->msg.length;		//不包含length位
	crclen				=	msg_len+1;							//包含length位
	CrcStartAddr	=	&ampframe->msg.length;	
	crc16.crcl		=	CrcStartAddr[crclen+1];
	crc16.crch		=	CrcStartAddr[crclen+2];
	temp					=	CRC16_MODBUS(CrcStartAddr,crclen);
	
	if(((temp&0xFF)==crc16.crcl)&&(((temp>>8)&0xFF)==crc16.crch))
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
///*******************************************************************************
//*函数名			:	Check_AmpCrc16
//*功能描述		:	crc校验检查
//*输入				: 
//*返回值			:	无
//*修改时间		:	无
//*修改说明		:	无
//*注释				:	wegam@sina.com
//*******************************************************************************/
//static unsigned char Check_AmpCrc16(unsigned char* pframe,unsigned short Framelen)
//{  
//	unsigned short msglen	= 0;
//	unsigned short crclen	=	0;
//	unsigned short crc16cmp	=	0;
//	unsigned short frame_Len	=	0;    //当前缓存数据长度
//	
//	unsigned char* Get_Crc_StartAddr	=	NULL;
//	
//	stampcrc16def	crc16;
//	stampphydef	frame;
//	
//	memcpy((unsigned char*)&frame,pframe,Framelen);
//	
//	msglen	=	frame.msg.length;
//	frame_Len  = msglen+5;   //完整帧长度，head,length,crc16,end为5个字节
//	
//	if(frame_Len>Framelen)
//	{
//		return 0;
//	}
//	crclen	=	msglen+1;		//CRC计算包含frame.msg.length
//	Get_Crc_StartAddr	=	&frame.msg.length;	//从frame.msg.length开始计算
//	
//	crc16.crcl	=	Get_Crc_StartAddr[crclen+1];
//	crc16.crch	=	Get_Crc_StartAddr[crclen+2];
//	
//	Get_AmpCrc16(pframe,Framelen);
//	
//	if(((crc16cmp&0xFF)==crc16.crcl)&&(((crc16cmp>>8)&0xFF)==crc16.crch))
//  {
//    return 1;
//  }	
//	return	0;
//}


