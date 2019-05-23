#include "BT459A.H"


/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_bt459a_configuration(void)
{

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
void api_bt459a_server(void)
{

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
unsigned char api_bt459a_get_frame_ReadQuantity(unsigned char addr,unsigned char* pbuffer)
{
	unsigned char frame[128]={0};
	unsigned short crc16	=	0;
	frame[0]=addr;
	frame[1]=ReadHoldingRegisters;	//读保持寄存器，字节指令操作，可读单个或者多个
	//---------------寄存器地址
	frame[2]=reg_quantity>>8;				//寄存器起始地址高8位
	frame[3]=reg_quantity&0xFF;			//寄存器起始地址低8位
	//---------------
	frame[4]=0x00;									//寄存器数量高8位
	frame[5]=0x01;
	
	crc16	=	CRC16_MODBUS(frame,6);
	
	frame[6]=crc16&0xFF;
	frame[7]=crc16>>8;	
	
	pbuffer	=	frame;
	
	return 8;
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
unsigned char api_bt459a_get_frame_ReadWeight(unsigned char addr,unsigned char* pbuffer)
{
	unsigned char frame[128]={0};
	unsigned short crc16	=	0;
	frame[0]=addr;
	frame[1]=ReadHoldingRegisters;	//读保持寄存器，字节指令操作，可读单个或者多个
	//---------------寄存器地址
	frame[2]=reg_weight>>8;				//寄存器起始地址高8位
	frame[3]=reg_weight&0xFF;			//寄存器起始地址低8位
	//---------------
	frame[4]=0x00;									//寄存器数量高8位
	frame[5]=0x01;
	
	crc16	=	CRC16_MODBUS(frame,6);
	
	frame[6]=crc16&0xFF;
	frame[7]=crc16>>8;	
	
	pbuffer	=	frame;
	
	return 8;
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned char api_bt459a_get_frame_SetQuantity(unsigned char addr,unsigned short Quantity,unsigned char* pbuffer)
{
	unsigned char frame[128]={0};
	unsigned short crc16	=	0;
	frame[0]=addr;
	frame[1]=WriteSingleRegister;		//写单个保持寄存器，字节指令操作，只能写一个
	//---------------寄存器地址
	frame[2]=reg_quantity>>8;				//寄存器起始地址高8位
	frame[3]=reg_quantity&0xFF;			//寄存器起始地址低8位
	//---------------
	frame[4]=Quantity>>8;									//寄存器数量高8位
	frame[5]=Quantity&0xFF;
	
	crc16	=	CRC16_MODBUS(frame,6);
	
	frame[6]=crc16&0xFF;
	frame[7]=crc16>>8;	
	
	frame[0]=0x01;
	frame[1]=0x06;
	frame[2]=0x00;
	frame[3]=0x0a;
	frame[4]=0x00;
	frame[5]=0x00;
	
	crc16	=	CRC16_MODBUS(frame,6);
	
	frame[6]=crc16&0xFF;
	frame[7]=crc16>>8;
	
	pbuffer	=	frame;
	
//	//-----------------------------
//	frame[0]=0x01;
//	frame[1]=0x06;
//	frame[2]=0x00;
//	frame[3]=0x0a;
//	frame[4]=0x00;
//	frame[5]=0x00;	
//	crc16	=	CRC16_MODBUS(frame,6);	
//	frame[6]=crc16&0xFF;
//	frame[7]=crc16>>8;	
//	pbuffer	=	frame;
//	
//	//-----------------------------
//	frame[0]=0x02;
//	frame[1]=0x06;
//	frame[2]=0x00;
//	frame[3]=0x0a;
//	frame[4]=0x00;
//	frame[5]=0x00;	
//	crc16	=	CRC16_MODBUS(frame,6);	
//	frame[6]=crc16&0xFF;
//	frame[7]=crc16>>8;	
//	pbuffer	=	frame;
//	
//	//-----------------------------
//	frame[0]=0x03;
//	frame[1]=0x06;
//	frame[2]=0x00;
//	frame[3]=0x0a;
//	frame[4]=0x00;
//	frame[5]=0x00;	
//	crc16	=	CRC16_MODBUS(frame,6);	
//	frame[6]=crc16&0xFF;
//	frame[7]=crc16>>8;	
//	pbuffer	=	frame;
//	
//	//-----------------------------
//	frame[0]=0x04;
//	frame[1]=0x06;
//	frame[2]=0x00;
//	frame[3]=0x0a;
//	frame[4]=0x00;
//	frame[5]=0x00;	
//	crc16	=	CRC16_MODBUS(frame,6);	
//	frame[6]=crc16&0xFF;
//	frame[7]=crc16>>8;	
//	pbuffer	=	frame;
	
	
	return 8;
}
//------------------------------------------------------------------------------



