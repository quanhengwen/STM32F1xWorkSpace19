/************************************ CRC ************************************
* 文件名 	: CRC校验
* 作者   	: wegam@sina.com
* 版本   	: V
* 日期   	: 2017/09/11
* 说明   	: 
********************************************************************************
其它说明:
*
*
*
*
*
*
*
*
*******************************************************************************/
#include 	"CRC.H"

//#include	"stdio.h"				//用于printf
//#include	"string.h"			//用于printf
//#include	"stdarg.h"			//用于获取不确定个数的参数
//#include	"stdlib.h"			//malloc动态申请内存空间


//CRC16常见的标准有以下几种，被用在各个规范中，其算法原理基本一致，就是在数据的输入和输出有所差异，下边把这些标准的差异列出，并给出C语言的算法实现。
//CRC16_CCITT：				多项式x16+x12+x5+1（0x1021），初始值0x0000，低位在前，高位在后，结果与0x0000异或
//CRC16_CCITT_FALSE：	多项式x16+x12+x5+1（0x1021），初始值0xFFFF，低位在后，高位在前，结果与0x0000异或
//CRC16_XMODEM：			多项式x16+x12+x5+1（0x1021），初始值0x0000，低位在后，高位在前，结果与0x0000异或
//CRC16_X25：					多项式x16+x12+x5+1（0x1021），初始值0x0000，低位在前，高位在后，结果与0xFFFF异或

//CRC16_MODBUS：			多项式x16+x15+x5+1（0x8005），初始值0xFFFF，低位在前，高位在后，结果与0x0000异或
//CRC16_IBM：					多项式x16+x15+x5+1（0x8005），初始值0x0000，低位在前，高位在后，结果与0x0000异或
//CRC16_MAXIM：				多项式x16+x15+x5+1（0x8005），初始值0x0000，低位在前，高位在后，结果与0xFFFF异或
//CRC16_USB：					多项式x16+x15+x5+1（0x8005），初始值0xFFFF，低位在前，高位在后，结果与0xFFFF异或

//CRC16的算法原理：
//1.根据CRC16的标准选择初值CRCIn的值。
//2.将数据的第一个字节与CRCIn高8位异或。
//3.判断最高位，若该位为 0 左移一位，若为 1 左移一位再与多项式Hex码异或。
//4.重复3直至8位全部移位计算结束。
//5.重复将所有输入数据操作完成以上步骤，所得16位数即16位CRC校验码。 

/*******************************************************************************
*函数名			:	InvertUint8
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void InvertUint8(unsigned char *dBuf,unsigned char *srcBuf)
{
	int	i	=	0;
	unsigned char tmp[4];
	tmp[0] = 0;
	for(i=0; i< 8; i++)
	{
		if(srcBuf[0]& (1 << i))
		tmp[0]|=1<<(7-i);
	}
	dBuf[0] = tmp[0];
}
/*******************************************************************************
*函数名			:	InvertUint16
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void InvertUint16(unsigned short *dBuf,unsigned short *srcBuf)
{
	int	i	=	0;
	unsigned short tmp[4];
	tmp[0] = 0;
	for(i=0; i< 16; i++)
	{
		if(srcBuf[0]& (1 << i))
			tmp[0]|=1<<(15 - i);
	}
	dBuf[0] = tmp[0];
} 
/*******************************************************************************
*函数名			:	function
*功能描述		:	多项式x16+x12+x5+1（0x1021），初始值0x0000，低位在前，高位在后，结果与0x0000异或
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short CRC16_CCITT(unsigned char *puchMsg, unsigned int usDataLen)
{
  unsigned short 	wCRCin 	= 0x0000;
  unsigned short 	wCPoly 	= 0x1021;
  unsigned char 	wChar 	= 0;
  int	i	=	0;
	
  while (usDataLen--) 	
  {
			wChar = *(puchMsg++);
			InvertUint8(&wChar,&wChar);
			wCRCin ^= (wChar << 8);
			for(i = 0;i < 8;i++)
			{
				if(wCRCin & 0x8000)
					wCRCin = (wCRCin << 1) ^ wCPoly;
				else
					wCRCin = wCRCin << 1;
			}
  }
  InvertUint16(&wCRCin,&wCRCin);
  return (wCRCin) ;
}
/*******************************************************************************
*函数名			:	CRC16_CCITT_FALSE
*功能描述		:	多项式x16+x12+x5+1（0x1021），初始值0xFFFF，低位在后，高位在前，结果与0x0000异或
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short CRC16_CCITT_FALSE(unsigned char *puchMsg, unsigned int usDataLen)
{
  unsigned short wCRCin = 0xFFFF;
  unsigned short wCPoly = 0x1021;
  unsigned char wChar = 0;
  int	i	=	0;
	
  while (usDataLen--) 	
  {
        wChar = *(puchMsg++);
        wCRCin ^= (wChar << 8);
        for(i = 0;i < 8;i++)
        {
          if(wCRCin & 0x8000)
            wCRCin = (wCRCin << 1) ^ wCPoly;
          else
            wCRCin = wCRCin << 1;
        }
  }
  return (wCRCin) ;
}
/*******************************************************************************
*函数名			:	CRC16_XMODEM
*功能描述		:	多项式x16+x12+x5+1（0x1021），初始值0x0000，低位在后，高位在前，结果与0x0000异或
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short CRC16_XMODEM(unsigned char *puchMsg, unsigned int usDataLen)
{
  unsigned short 	wCRCin 	= 0x0000;
  unsigned short 	wCPoly 	= 0x1021;
  unsigned char 	wChar 	= 0;
  int	i	=	0;
	
  while (usDataLen--) 	
  {
        wChar = *(puchMsg++);
        wCRCin ^= (wChar << 8);
        for(i = 0;i < 8;i++)
        {
          if(wCRCin & 0x8000)
            wCRCin = (wCRCin << 1) ^ wCPoly;
          else
            wCRCin = wCRCin << 1;
        }
  }
  return (wCRCin) ;
}
/*******************************************************************************
*函数名			:	CRC16_X25
*功能描述		:	多项式x16+x12+x5+1（0x1021），初始值0x0000，低位在前，高位在后，结果与0xFFFF异或
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short CRC16_X25(unsigned char *puchMsg, unsigned int usDataLen)
{
  unsigned short 	wCRCin 	= 0xFFFF;
  unsigned short 	wCPoly 	= 0x1021;
  unsigned char 	wChar 	= 0;
  int	i	=	0;
	
  while (usDataLen--) 	
  {
        wChar = *(puchMsg++);
        InvertUint8(&wChar,&wChar);
        wCRCin ^= (wChar << 8);
        for(i = 0;i < 8;i++)
        {
          if(wCRCin & 0x8000)
            wCRCin = (wCRCin << 1) ^ wCPoly;
          else
            wCRCin = wCRCin << 1;
        }
  }
  InvertUint16(&wCRCin,&wCRCin);
	return (wCRCin^0xFFFF) ;
//	wCRCin	=	wCRCin^0xFFFF;
//	*puchMsg++	=	wCRCin&0XFF;
//	*puchMsg++	=	wCRCin>>8&0XFF;
}
/*******************************************************************************
*函数名			:	CRC16_MODBUS
*功能描述		:	多项式x16+x15+x5+1（0x8005），初始值0xFFFF，低位在前，高位在后，结果与0x0000异或
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short CRC16_MODBUS(unsigned char *puchMsg, unsigned int usDataLen)
{
  unsigned short wCRCin = 0xFFFF;
  unsigned short wCPoly = 0x8005;
  unsigned char wChar 	= 0;
  int	i	=	0;
	
  while (usDataLen--) 	
  {
        wChar = *(puchMsg++);
        InvertUint8(&wChar,&wChar);
        wCRCin ^= (wChar << 8);
        for(i = 0;i < 8;i++)
        {
          if(wCRCin & 0x8000)
            wCRCin = (wCRCin << 1) ^ wCPoly;
          else
            wCRCin = wCRCin << 1;
        }
  }
  InvertUint16(&wCRCin,&wCRCin);
  return (wCRCin) ;
}
/*******************************************************************************
*函数名			:	CRC16_IBM
*功能描述		:	多项式x16+x15+x5+1（0x8005），初始值0x0000，低位在前，高位在后，结果与0x0000异或
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short CRC16_IBM(unsigned char *puchMsg, unsigned int usDataLen)
{
  unsigned short wCRCin = 0x0000;
  unsigned short wCPoly = 0x8005;
  unsigned char wChar 	= 0;
  int	i	=	0;
	
  while (usDataLen--) 	
  {
        wChar = *(puchMsg++);
        InvertUint8(&wChar,&wChar);
        wCRCin ^= (wChar << 8);
        for(i = 0;i < 8;i++)
        {
          if(wCRCin & 0x8000)
            wCRCin = (wCRCin << 1) ^ wCPoly;
          else
            wCRCin = wCRCin << 1;
        }
  }
  InvertUint16(&wCRCin,&wCRCin);
  return (wCRCin) ;
}
/*******************************************************************************
*函数名			:	CRC16_MAXIM
*功能描述		:	多项式x16+x15+x5+1（0x8005），初始值0x0000，低位在前，高位在后，结果与0xFFFF异或
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short CRC16_MAXIM(unsigned char *puchMsg, unsigned int usDataLen)
{
  unsigned short wCRCin = 0x0000;
  unsigned short wCPoly = 0x8005;
  unsigned char wChar 	= 0;
  int	i	=	0;
	
  while (usDataLen--) 	
  {
        wChar = *(puchMsg++);
        InvertUint8(&wChar,&wChar);
        wCRCin ^= (wChar << 8);
        for(i = 0;i < 8;i++)
        {
          if(wCRCin & 0x8000)
            wCRCin = (wCRCin << 1) ^ wCPoly;
          else
            wCRCin = wCRCin << 1;
        }
  }
  InvertUint16(&wCRCin,&wCRCin);
  return (wCRCin^0xFFFF) ;
}
/*******************************************************************************
*函数名			:	CRC16_USB
*功能描述		:	多项式x16+x15+x5+1（0x8005），初始值0xFFFF，低位在前，高位在后，结果与0xFFFF异或
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short CRC16_USB(unsigned char *puchMsg, unsigned int usDataLen)
{
  unsigned short wCRCin = 0xFFFF;
  unsigned short wCPoly = 0x8005;
  unsigned char wChar 	= 0;
  int i = 0;
	
  while (usDataLen--) 	
  {
        wChar = *(puchMsg++);
        InvertUint8(&wChar,&wChar);
        wCRCin ^= (wChar << 8);
        for(i = 0;i < 8;i++)
        {
          if(wCRCin & 0x8000)
            wCRCin = (wCRCin << 1) ^ wCPoly;
          else
            wCRCin = wCRCin << 1;
        }
  }
  InvertUint16(&wCRCin,&wCRCin);
  return (wCRCin^0xFFFF) ;
}
//------------------------------------------------------------------------------




static const unsigned char CRC8_8541_msb_tab[] =
{
    0x00,0x31,0x62,0x53,0xc4,0xf5,0xa6,0x97,0xb9,0x88,0xdb,0xea,0x7d,0x4c,0x1f,0x2e,
    0x43,0x72,0x21,0x10,0x87,0xb6,0xe5,0xd4,0xfa,0xcb,0x98,0xa9,0x3e,0x0f,0x5c,0x6d,
    0x86,0xb7,0xe4,0xd5,0x42,0x73,0x20,0x11,0x3f,0x0e,0x5d,0x6c,0xfb,0xca,0x99,0xa8,
    0xc5,0xf4,0xa7,0x96,0x01,0x30,0x63,0x52,0x7c,0x4d,0x1e,0x2f,0xb8,0x89,0xda,0xeb,
    0x3d,0x0c,0x5f,0x6e,0xf9,0xc8,0x9b,0xaa,0x84,0xb5,0xe6,0xd7,0x40,0x71,0x22,0x13,
    0x7e,0x4f,0x1c,0x2d,0xba,0x8b,0xd8,0xe9,0xc7,0xf6,0xa5,0x94,0x03,0x32,0x61,0x50,
    0xbb,0x8a,0xd9,0xe8,0x7f,0x4e,0x1d,0x2c,0x02,0x33,0x60,0x51,0xc6,0xf7,0xa4,0x95,
    0xf8,0xc9,0x9a,0xab,0x3c,0x0d,0x5e,0x6f,0x41,0x70,0x23,0x12,0x85,0xb4,0xe7,0xd6,
    0x7a,0x4b,0x18,0x29,0xbe,0x8f,0xdc,0xed,0xc3,0xf2,0xa1,0x90,0x07,0x36,0x65,0x54,
    0x39,0x08,0x5b,0x6a,0xfd,0xcc,0x9f,0xae,0x80,0xb1,0xe2,0xd3,0x44,0x75,0x26,0x17,
    0xfc,0xcd,0x9e,0xaf,0x38,0x09,0x5a,0x6b,0x45,0x74,0x27,0x16,0x81,0xb0,0xe3,0xd2,
    0xbf,0x8e,0xdd,0xec,0x7b,0x4a,0x19,0x28,0x06,0x37,0x64,0x55,0xc2,0xf3,0xa0,0x91,
    0x47,0x76,0x25,0x14,0x83,0xb2,0xe1,0xd0,0xfe,0xcf,0x9c,0xad,0x3a,0x0b,0x58,0x69,
    0x04,0x35,0x66,0x57,0xc0,0xf1,0xa2,0x93,0xbd,0x8c,0xdf,0xee,0x79,0x48,0x1b,0x2a,
    0xc1,0xf0,0xa3,0x92,0x05,0x34,0x67,0x56,0x78,0x49,0x1a,0x2b,0xbc,0x8d,0xde,0xef,
    0x82,0xb3,0xe0,0xd1,0x46,0x77,0x24,0x15,0x3b,0x0a,0x59,0x68,0xff,0xce,0x9d,0xac
};
/*******************************************************************************
*函数名			:	CRC8_8541_msb 高位在前
*功能描述		:	多项式：x8+x5+x4+1（二进制为：100110001）
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned char CRC8_8541_msb(unsigned char *puchMsg, unsigned int usDataLen)
{
  unsigned char m = 0;
	unsigned char crc 	= 0;	/* 计算的初始crc值 */  
	unsigned int i	=	0;
	goto tab_check;
	//---------------------------------------------计算法
	for(i=0;i<usDataLen;i++)
	{
		crc ^= puchMsg[i];  /* 每次先与需要计算的数据异或,计算完指向下一数据 */
		/* 数据往左移了8位，需要计算8次 */
		for (m=0; m<8; m--)   /* 判断最高位是否为1 */  
		{ 
			/* 最高位为1，不需要异或，往左移一位，然后与0x31异或 */
			/* 0x31(多项式：x8+x5+x4+1，100110001)，最高位不需要异或，直接去掉 */
			if (crc & 0x80)
				crc = (crc << 1) ^ 0x31;
			/* 最高位为0时，不需要异或，整体数据往左移一位 */
			else
				crc = (crc << 1);
		}
	}
  return (crc) ;
	//---------------------------------------------查表法
	tab_check:
	for(i=0;i<usDataLen;i++)
	{
		crc = CRC8_8541_msb_tab[crc ^ puchMsg[i]];
	}
	return (crc) ;
}
//------------------------------------------------------------------------------


const unsigned char CRC8_8541_lsb_tab[256]=
{
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
};
/*******************************************************************************
*函数名			:	CRC8_8541_lsb低位在前
*功能描述		:	多项式：x8+x5+x4+1（二进制为：100110001）
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned char CRC8_8541_lsb(unsigned char *puchMsg, unsigned int usDataLen)
{
  unsigned char m = 0;
	unsigned char crc 	= 0;	/* 计算的初始crc值 */  
	unsigned int i	=	0;
	goto tab_check;
	//---------------------------------------------计算法
	for(i=0;i<usDataLen;i++)
	{
		crc ^= puchMsg[i];  /* 每次先与需要计算的数据异或,计算完指向下一数据 */
		/* 数据往左移了8位，需要计算8次 */
		for (m=0; m<8; m--)   /* 判断最高位是否为1 */  
		{ 
			/* 最高位为1，不需要异或，往左移一位，然后与0x31异或 */
			/* 0x31(多项式：x8+x5+x4+1，100110001)，最高位不需要异或，直接去掉 */
			if (crc & 0x80)
				crc = (crc >> 1) ^ 0x8c;
			/* 最高位为0时，不需要异或，整体数据往左移一位 */
			else
				crc = (crc >> 1);
		}
	}
	return (crc) ;
	//---------------------------------------------查表法
	tab_check:
	for(i=0;i<usDataLen;i++)
	{
		crc = CRC8_8541_lsb_tab[crc ^ puchMsg[i]];
	}	
  return (crc) ;
}
