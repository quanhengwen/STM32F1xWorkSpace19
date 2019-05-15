#include "MFRC522.H"

#include "string.h"
#include "STM32_GPIO.H"

#define MAXRLEN 18

//unsigned char mfrc522_UID[5],mfrc522_Temp[4];

static mfrc522def* pMfrc522;


mfrc522_RegStsDef	mfrc522_RegSts;
//-----------------------------------------------static-hardware
#define mfrc522_set_rst_high	(pMfrc522->port.rst_port->BSRR=pMfrc522->port.rst_pin)
#define mfrc522_set_rst_low		(pMfrc522->port.rst_port->BRR=pMfrc522->port.rst_pin)
#define mfrc522_set_nss_high	(pMfrc522->port.nss_port->BSRR=pMfrc522->port.nss_pin)
#define mfrc522_set_nss_low		(pMfrc522->port.nss_port->BRR=pMfrc522->port.nss_pin)
#define mfrc522_set_clk_high	(pMfrc522->port.clk_port->BSRR=pMfrc522->port.clk_pin)
#define mfrc522_set_clk_low		(pMfrc522->port.clk_port->BRR=pMfrc522->port.clk_pin)
#define mfrc522_set_mosi_high	(pMfrc522->port.mosi_port->BSRR=pMfrc522->port.mosi_pin)
#define mfrc522_set_mosi_low	(pMfrc522->port.mosi_port->BRR=pMfrc522->port.mosi_pin)
//-----------------------------------------------static-hardware-miso
#define	mfrc522_get_miso			(pMfrc522->port.miso_port->IDR&pMfrc522->port.miso_pin)


/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_mfrc522_configuration(mfrc522def* pInfo)
{
	pMfrc522	=	pInfo;
	
	mfrc522_port_configuration(pMfrc522);
	
	mfrc522_set_reset();					//复位RC522
	mfrc522_set_AntennaOff();
	mfrc522_set_AntennaOn();			//开启天线发射
	M500PcdConfigISOType('A');
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
void api_mfrc522_self_test(mfrc522def* pInfo)
{
	unsigned char i = 0;
	unsigned char fifo[128]={0};
	//------------------------reset
	mfrc522_set_rst_low;
	mfrc522_delay_ms(5);
	mfrc522_set_rst_high;
	mfrc522_delay_ms(5);
	
	//------------------------Clear the internal buffer by writing 25 bytes of 00h and implement the Config command.
	mfrc522_set_clk_low;		//MF522_SCK = 0;
	mfrc522_set_nss_low;		//MF522_NSS = 0;
	for(i=0;i<25;i++)
	{
		mfrc522_write_byte(0x00);
	}
	mfrc522_set_nss_high;		//MF522_NSS = 1
	mfrc522_set_clk_high;		//MF522_SCK = 1;
	//------------------------Enable the self test by writing 09h to the AutoTestReg register
	mfrc522_write_data(AutoTestReg,0x09);
	//------------------------Write 00h to the FIFO buffer 64byte
	memset(fifo,0x00,128);
	mfrc522_write_buffer(FIFODataReg,fifo,DEF_FIFO_LENGTH);
	//------------------------Start the self test with the CalcCRC command
	mfrc522_write_data(AutoTestReg,0x09);
	//------------------------The self test is initiated
	mfrc522_delay_ms(10);
	//------------------------When the self test has completed, the FIFO buffer contains the following 64 bytes
	mfrc522_read_buffer(FIFODataReg,fifo,DEF_FIFO_LENGTH);
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
void api_mfrc522_Auto_Reader(mfrc522def* pInfo)
{
	
	
//  while(1)
//  {
//		
//    if(PcdRequest(0x52,mfrc522_Temp)==MI_OK)
//    {
//      if(PcdAnticoll(mfrc522_UID)==MI_OK)
//      { 
//				
//      }
//    }
//  } 
}
/*******************************************************************************
*函数名			:	mfrc522_PcdRequest
*功能描述		:	function
*输入				: req_code[IN]:寻卡方式
*                0x52 = 寻感应区内所有符合14443A标准的卡
*                0x26 = 寻未进入休眠状态的卡
*返回值			:	
                0x4400 = Mifare_UltraLight
                0x0400 = Mifare_One(S50)
                0x0200 = Mifare_One(S70)
                0x0800 = Mifare_Pro(X)
                0x4403 = Mifare_DESFire
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
mfrc522_card_typedef mfrc522_PcdRequest(unsigned char req_code)
{
	char status;	
	unsigned char i	=	0;
	unsigned int  unLen;
	unsigned char ucComMF522Buf[MAXRLEN];

	mfrc522_card_typedef card_type	=	Mifare_none; 	

	//-------------------------------------发送前清除FIFO---20190419添加测试查看数据
//	mfrc522_read_register(ComIrqReg);					//读中断寄存器---20190419添加测试查看数据
//	if(1==mfrc522_RegSts.ComIrqSts.RxIRq)			//接收器检测到有效数据
//	{
//		mfrc522_read_register(ControlReg);			//读取最后接收字节有效位个数
//		mfrc522_read_register(FIFOLevelReg);		//读取FIFO中数据个数
//		for (i=0; i<MAXRLEN; i++)
//		{
//			mfrc522_read_register(FIFODataReg);	//从FIFO缓存读取数据					
//		}
//		mfrc522_set_reset();
//		mfrc522_delay_ms(12);
//	}
	mfrc522_set_reset();



	
	mfrc522_del_Bit_Mask(Status2Reg,0x08);				//寄存器包含接收器和发送器和数据模式检测器的状态标志
	mfrc522_write_data(BitFramingReg,0x07);				//定义将发送数据的最后一个字节的位的数量
	mfrc522_set_Bit_Mask(TxControlReg,0x03);			//使能TX1、TX2输出，TX1、TX2输出信号将传递经发送数据调制的13.56MHz的能量载波信号。

	ucComMF522Buf[0] = req_code;

	status = mfrc522_PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);		//通过RC522和ISO14443卡通讯,读取数据

	if ((status == MI_OK) && (unLen == 0x10))		//读取到数据--寻卡成功
	{
		unsigned short temp	=	0;
		//--------------------转换卡类型高低位顺序
		temp=ucComMF522Buf[0];
		temp<<=8;
		temp|=ucComMF522Buf[1];		
		
		memcpy(&card_type,&temp,2);
	}
	else
	{   status = MI_ERR;   }

	return card_type;
}
/*******************************************************************************
*函数名			:	mfrc522_PcdRequest
*功能描述		:	function
*输入				: req_code[IN]:寻卡方式
*                0x52 = 寻感应区内所有符合14443A标准的卡
*                0x26 = 寻未进入休眠状态的卡
*返回值			:	
                0x4400 = Mifare_UltraLight
                0x0400 = Mifare_One(S50)
                0x0200 = Mifare_One(S70)
                0x0800 = Mifare_Pro(X)
                0x4403 = Mifare_DESFire
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
mfrc522_card_typedef mfrc522_PcdRequestbac(unsigned char req_code)
{
   
	char status;	
	unsigned int  unLen;
	unsigned char ucComMF522Buf[MAXRLEN];

	mfrc522_card_typedef card_type	=	Mifare_none; 	

	mfrc522_del_Bit_Mask(Status2Reg,0x08);				//寄存器包含接收器和发送器和数据模式检测器的状态标志
	mfrc522_write_data(BitFramingReg,0x07);				//定义将发送数据的最后一个字节的位的数量
	mfrc522_set_Bit_Mask(TxControlReg,0x03);			//使能TX1、TX2输出，TX1、TX2输出信号将传递经发送数据调制的13.56MHz的能量载波信号。

	ucComMF522Buf[0] = req_code;

	status = mfrc522_PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);		//通过RC522和ISO14443卡通讯,读取数据

	if ((status == MI_OK) && (unLen == 0x10))		//读取到数据--寻卡成功
	{
		unsigned short temp	=	0;
		//--------------------转换卡类型高低位顺序
		temp=ucComMF522Buf[0];
		temp<<=8;
		temp|=ucComMF522Buf[1];		
		
		memcpy(&card_type,&temp,2);
	}
	else
	{   status = MI_ERR;   }

	return card_type;
}
/////////////////////////////////////////////////////////////////////
//功    能：防冲撞--获取卡序列号
//参数说明: pSnr[OUT]:卡片序列号缓存地址，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////  
char mfrc522_PcdAnticoll(unsigned char *pSnr)
{
	char status;

	unsigned char i,snr_check=0;
	unsigned int  unLen;
	unsigned char ucComMF522Buf[MAXRLEN]; 


	mfrc522_del_Bit_Mask(Status2Reg,0x08);					//清除相关状态寄存器
	mfrc522_write_data(BitFramingReg,0x00);	//000b 表示最后一个字节的所有位都被发送
	mfrc522_del_Bit_Mask(CollReg,0x80);

	ucComMF522Buf[0] = PICC_ANTICOLL1;		//防冲撞
	ucComMF522Buf[1] = 0x20;

	status = mfrc522_PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);	//通过RC522和ISO14443卡通讯,读取数据

	if (status == MI_OK)
	{
		for (i=0; i<4; i++)
		{   
			*(pSnr+i)  = ucComMF522Buf[i];
			snr_check ^= ucComMF522Buf[i];
		}
		if (snr_check != ucComMF522Buf[i])
		{   status = MI_ERR;    }
	}    
	mfrc522_set_Bit_Mask(CollReg,0x80);
	return status;
}
/////////////////////////////////////////////////////////////////////
//功    能：通过RC522和ISO14443卡通讯
//参数说明：Command[IN]:RC522命令字
//          pInData[IN]:通过RC522发送到卡片的数据
//          InLenByte[IN]:发送数据的字节长度
//          pOutData[OUT]:接收到的卡片返回数据
//          *pOutLenBit[OUT]:返回数据的位长度
/////////////////////////////////////////////////////////////////////
char mfrc522_PcdComMF522(unsigned char Command,unsigned char *pInData,unsigned char InLenByte,unsigned char *pOutData,unsigned int  *pOutLenBit)
{
	char status = MI_ERR;
	unsigned char irqEn   = 0x00;
	unsigned char waitFor = 0x00;
	unsigned char lastBits;
	unsigned char n;
	unsigned int i;
	
	switch (Command)
	{
		case PCD_AUTHENT:			//验证密钥
			irqEn   = 0x12;
			waitFor = 0x10;
			break;
		case PCD_TRANSCEIVE:	//发送并接收数据
			irqEn   = 0x77;
			waitFor = 0x30;
			break;
		default:
		 break;
	}
	//mfrc522_check_status();
	
	//-------------------------------------发送设置
	mfrc522_write_register(ComIEnReg,irqEn|0x80);		//使能相关中断
	mfrc522_del_Bit_Mask(ComIrqReg,0x80);						//清除相关中断标志
	mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
	mfrc522_write_register(CommandReg,PCD_IDLE);		//取消当前命令
	mfrc522_set_Bit_Mask(FIFOLevelReg,0x80);				//清除当前缓存中的字节数
	//-------------------------------------写入FIFO
	for (i=0; i<InLenByte; i++)
	{
		mfrc522_write_data(FIFODataReg, pInData[i]);	//往FIFO缓存写入数据
	}
	//-------------------------------------开启发送
	mfrc522_write_register(CommandReg, Command);		//启动发送

	
	
	if (Command == PCD_TRANSCEIVE)
	{
		mfrc522_set_Bit_Mask(BitFramingReg,0x80);		//全部发送
	}
//	return 0;
	//-------------------------------------中断标识寄存器，检查有无接收到数据或超时
	//i = 600;	//根据时钟频率调整，操作M1卡最大等待时间25ms
	//i = 500;	//根据时钟频率调整，操作M1卡最大等待时间25ms	
	for(i=2500;i>0;i--) 
	{
		n = mfrc522_read_register(ComIrqReg);			//读取中断寄存器		
		memcpy(&mfrc522_RegSts.ComIrqSts,&n,1);
		
		if(1==mfrc522_RegSts.ComIrqSts.TimerIRq)	//定时时间到
		{
			break;
		}
		if(1==mfrc522_RegSts.ComIrqSts.IdleIRq)		//进入空闲状态
		{
			break;
		}
		if(PCD_TRANSCEIVE==Command)		//发送并接收数据
		{
			if(1==mfrc522_RegSts.ComIrqSts.RxIRq)	//接收器检测到有效数据
			{
				break;
			}
		}
		mfrc522_delay_us(10);
	}
	
	mfrc522_read_data(ControlReg);
	//-------------------------------------清除标识
	mfrc522_del_Bit_Mask(BitFramingReg,0x80);		//清除	
	//-------------------------------------检查接收状态并读取数据
	if (i!=0)
	{  
		//------------------读错误状态，检查有无错误
		mfrc522_read_register(ErrorReg);		//读错误码 判断标识0x1B
		if((1==mfrc522_RegSts.ErrorSts.ProtocolErr)		//SOF错误时该位置1
			||(1==mfrc522_RegSts.ErrorSts.ParityErr)		//奇偶校验错误时该位置1
			||(1==mfrc522_RegSts.ErrorSts.CollErr)			//检查出一个位冲突时该位置1
			||(1==mfrc522_RegSts.ErrorSts.BufferOvfl))	//尽管FIFO缓冲区已满，但主机或内部状态机(e.g. receiver)还试图向里面写数据时该位置1
		{
			status = MI_ERR;			
		}
		else
		{
			status = MI_OK;
			if (n & irqEn & 0x01)
			{	
				status = MI_NOTAGERR;		//未触发
			}
			if (Command == PCD_TRANSCEIVE)	//发送并接收数据命令
			{
				mfrc522_read_register(ControlReg);			//读取最后接收字节有效位个数
				mfrc522_read_register(FIFOLevelReg);		//读取FIFO中数据个数
				
				n = mfrc522_RegSts.FIFOLevelSts.FIFOLevel;				//表示FIFO 缓冲区中存储数据的字节数
				lastBits	=	mfrc522_RegSts.ControlSts.RxLastBits;	//表示最后接收字节的有效位的个数，如果此值为000B，则整个字节都是有效的
				
				if (lastBits)
				{
					*pOutLenBit = (n-1)*8 + lastBits;
				}
				else
				{
					*pOutLenBit = n*8;
				}
				if (n == 0)
				{
					n = 1;
				}
				if (n > MAXRLEN)		//12
				{
					n = MAXRLEN;
				}
				for (i=0; i<n; i++)
				{
					pOutData[i] = mfrc522_read_register(FIFODataReg);	//从FIFO缓存读取数据					
				}
			}
		}
	}
	//---------------------------
	mfrc522_set_Bit_Mask(ControlReg,0x80);   	// stop timer now
	mfrc522_write_data(CommandReg,PCD_IDLE);	//进入空闲模式
	return status;
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
static void mfrc522_check_status(void)
{
	// PAGE 0		控制和状态寄存器
	mfrc522_read_register(CommandReg);
	mfrc522_read_register(ComIEnReg);
	mfrc522_read_register(DivIEnReg);
	mfrc522_read_register(ComIrqReg);
	mfrc522_read_register(DivIrqReg);
	mfrc522_read_register(ErrorReg);
	mfrc522_read_register(Status1Reg);
	mfrc522_read_register(Status2Reg);
	mfrc522_read_register(FIFOLevelReg);
	mfrc522_read_register(WaterLevelReg);
	mfrc522_read_register(ControlReg);
	mfrc522_read_register(BitFramingReg);	
	mfrc522_read_register(CollReg);
	
	// PAGE 1  	通信寄存器
	mfrc522_read_register(ModeReg);
	mfrc522_read_register(TxModeReg);
	mfrc522_read_register(RxModeReg);
	mfrc522_read_register(TxControlReg);
	mfrc522_read_register(TxAutoReg);
	mfrc522_read_register(TxSelReg);
	mfrc522_read_register(RxSelReg);
	mfrc522_read_register(RxThresholdReg);
	mfrc522_read_register(DemodReg);
	mfrc522_read_register(MifareReg);
	mfrc522_read_register(SerialSpeedReg);
	
	// PAGE 2 	配置寄存器 
	mfrc522_read_register(CRCResultRegM);
	mfrc522_read_register(CRCResultRegL);
	mfrc522_read_register(ModWidthReg);
	mfrc522_read_register(RFCfgReg);
	mfrc522_read_register(GsNReg);
	mfrc522_read_register(CWGsCfgReg);
	mfrc522_read_register(ModGsCfgReg);
	mfrc522_read_register(TModeReg);
	mfrc522_read_register(TPrescalerReg);
	mfrc522_read_register(TReloadRegH);
	mfrc522_read_register(TReloadRegL);
	mfrc522_read_register(TCounterValueRegH);
	mfrc522_read_register(TCounterValueRegL);
	
	// PAGE 3  	Test寄存器
	mfrc522_read_register(TestSel1Reg);
	mfrc522_read_register(TestSel2Reg);
	mfrc522_read_register(TestPinEnReg);
	mfrc522_read_register(TestPinValueReg);
	mfrc522_read_register(TestBusReg);
	mfrc522_read_register(AutoTestReg);
	mfrc522_read_register(VersionReg);
	mfrc522_read_register(AnalogTestReg);
	mfrc522_read_register(TestDAC1Reg);
	mfrc522_read_register(TestDAC2Reg);
	mfrc522_read_register(TestADCReg);
}
/////////////////////////////////////////////////////////////////////
//功    能：寻卡
//参数说明: req_code[IN]:寻卡方式
//                0x52 = 寻感应区内所有符合14443A标准的卡
//                0x26 = 寻未进入休眠状态的卡
//          pTagType[OUT]：卡片类型代码
//                0x4400 = Mifare_UltraLight
//                0x0400 = Mifare_One(S50)
//                0x0200 = Mifare_One(S70)
//                0x0800 = Mifare_Pro(X)
//                0x4403 = Mifare_DESFire
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdRequest(unsigned char req_code,unsigned char *pTagType)
{
   char status;  
   unsigned int  unLen;
   unsigned char ucComMF522Buf[MAXRLEN]; 

   mfrc522_del_Bit_Mask(Status2Reg,0x08);				//寄存器包含接收器和发送器和数据模式检测器的状态标志
   mfrc522_write_data(BitFramingReg,0x07);			//定义将发送数据的最后一个字节的位的数量
   mfrc522_set_Bit_Mask(TxControlReg,0x03);			//TX1、TX2输出信号将传递经发送数据调制的13.56MHz的能量载波信号。
 
   ucComMF522Buf[0] = req_code;

   status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);		//通过RC522和ISO14443卡通讯,读取数据

   if ((status == MI_OK) && (unLen == 0x10))		//读取到数据--寻卡成功
   {    
       *pTagType     = ucComMF522Buf[0];
       *(pTagType+1) = ucComMF522Buf[1];
   }
   else
   {   status = MI_ERR;   }
   
   return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：防冲撞
//参数说明: pSnr[OUT]:卡片序列号缓存地址，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////  
char PcdAnticoll(unsigned char *pSnr)
{
	char status;

	unsigned char i,snr_check=0;
	unsigned int  unLen;
	unsigned char ucComMF522Buf[MAXRLEN]; 


	mfrc522_del_Bit_Mask(Status2Reg,0x08);					//清除相关状态寄存器
	mfrc522_write_data(BitFramingReg,0x00);	//000b 表示最后一个字节的所有位都被发送
	mfrc522_del_Bit_Mask(CollReg,0x80);

	ucComMF522Buf[0] = PICC_ANTICOLL1;		//防冲撞
	ucComMF522Buf[1] = 0x20;

	status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);	//通过RC522和ISO14443卡通讯,读取数据

	if (status == MI_OK)
	{
		for (i=0; i<4; i++)
		{   
			*(pSnr+i)  = ucComMF522Buf[i];
			snr_check ^= ucComMF522Buf[i];
		}
		if (snr_check != ucComMF522Buf[i])
		{   status = MI_ERR;    }
	}    
	mfrc522_set_Bit_Mask(CollReg,0x80);
	return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：选定卡片
//参数说明: pSnr[IN]:卡片序列号，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdSelect(unsigned char *pSnr)
{
    char status;
    unsigned char i;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i=0; i<4; i++)
    {
    	ucComMF522Buf[i+2] = *(pSnr+i);
    	ucComMF522Buf[6]  ^= *(pSnr+i);
    }
    CalulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);
  
    mfrc522_del_Bit_Mask(Status2Reg,0x08);

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);
    
    if ((status == MI_OK) && (unLen == 0x18))
    {   status = MI_OK;  }
    else
    {   status = MI_ERR;    }

    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：验证卡片密码
//参数说明: auth_mode[IN]: 密码验证模式
//                 0x60 = 验证A密钥
//                 0x61 = 验证B密钥 
//          addr[IN]：块地址
//          pKey[IN]：密码
//          pSnr[IN]：卡片序列号，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////               
char PcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+2] = *(pKey+i);   }
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+8] = *(pSnr+i);   }
 //   memcpy(&ucComMF522Buf[2], pKey, 6); 
 //   memcpy(&ucComMF522Buf[8], pSnr, 4); 
    
    status = PcdComMF522(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen);
    if ((status != MI_OK) || (!(mfrc522_read_data(Status2Reg) & 0x08)))
    {   status = MI_ERR;   }
    
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：读取M1卡一块数据
//参数说明: addr[IN]：块地址
//          pData[OUT]：读出的数据，16字节
//返    回: 成功返回MI_OK
///////////////////////////////////////////////////////////////////// 
char PcdRead(unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_READ;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
   
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
    if ((status == MI_OK) && (unLen == 0x90))
 //   {   memcpy(pData, ucComMF522Buf, 16);   }
    {
        for (i=0; i<16; i++)
        {    *(pData+i) = ucComMF522Buf[i];   }
    }
    else
    {   status = MI_ERR;   }
    
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：写数据到M1卡一块
//参数说明: addr[IN]：块地址
//          pData[IN]：写入的数据，16字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////                  
char PcdWrite(unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_WRITE;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
        
    if (status == MI_OK)
    {
        //memcpy(ucComMF522Buf, pData, 16);
        for (i=0; i<16; i++)
        {    ucComMF522Buf[i] = *(pData+i);   }
        CalulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);

        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen);
        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }
    
    return status;
}





/////////////////////////////////////////////////////////////////////
//用MF522计算CRC16函数
/////////////////////////////////////////////////////////////////////
void CalulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData)
{
    unsigned char i,n;
    mfrc522_del_Bit_Mask(DivIrqReg,0x04);
    mfrc522_write_data(CommandReg,PCD_IDLE);
    mfrc522_set_Bit_Mask(FIFOLevelReg,0x80);
    for (i=0; i<len; i++)
    {   mfrc522_write_data(FIFODataReg, *(pIndata+i));   }
    mfrc522_write_data(CommandReg, PCD_CALCCRC);
    i = 0xFF;
    do 
    {
        n = mfrc522_read_data(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));
    pOutData[0] = mfrc522_read_data(CRCResultRegL);
    pOutData[1] = mfrc522_read_data(CRCResultRegM);
}


//////////////////////////////////////////////////////////////////////
//设置RC632的工作方式 
//////////////////////////////////////////////////////////////////////
char M500PcdConfigISOType(unsigned char type)
{
   if (type == 'A')                     //ISO14443_A
   { 
       mfrc522_del_Bit_Mask(Status2Reg,0x08);

 /* 	mfrc522_write_byte(CommandReg,0x20);    //as default   
			mfrc522_write_byte(ComIEnReg,0x80);     //as default
			mfrc522_write_byte(DivlEnReg,0x0);      //as default
			mfrc522_write_byte(ComIrqReg,0x04);     //as default
			mfrc522_write_byte(DivIrqReg,0x0);      //as default
			mfrc522_write_byte(Status2Reg,0x0);//80    //trun off temperature sensor
			mfrc522_write_byte(WaterLevelReg,0x08); //as default
			mfrc522_write_byte(ControlReg,0x20);    //as default
			mfrc522_write_byte(CollReg,0x80);    //as default
*/
       mfrc522_write_data(ModeReg,0x3D);//3F
/*	   mfrc522_write_byte(TxModeReg,0x0);      //as default???
	   mfrc522_write_byte(RxModeReg,0x0);      //as default???
	   mfrc522_write_byte(TxControlReg,0x80);  //as default???

	   mfrc522_write_byte(TxSelReg,0x10);      //as default???
   */
       mfrc522_write_data(RxSelReg,0x86);//84
 //      mfrc522_write_byte(RxThresholdReg,0x84);//as default
 //      mfrc522_write_byte(DemodReg,0x4D);      //as default

 //      mfrc522_write_byte(ModWidthReg,0x13);//26
       mfrc522_write_data(RFCfgReg,0x7F);   //4F
	/*   mfrc522_write_byte(GsNReg,0x88);        //as default???
	   mfrc522_write_byte(CWGsCfgReg,0x20);    //as default???
       mfrc522_write_byte(ModGsCfgReg,0x20);   //as default???
*/
   	   mfrc522_write_data(TReloadRegL,30);//tmoLength);// TReloadVal = 'h6a =tmoLength(dec) 
	   mfrc522_write_data(TReloadRegH,0);
       mfrc522_write_data(TModeReg,0x8D);
	   mfrc522_write_data(TPrescalerReg,0x3E);
	   

  //     PcdSetTmo(106);
	    		mfrc522_delay_ms(10);
       mfrc522_set_AntennaOn();
   }
   else{ return -1; }
   
   return MI_OK;
}


 

/////////////////////////////////////////////////////////////////////
//功    能：通过RC522和ISO14443卡通讯
//参数说明：Command[IN]:RC522命令字
//          pInData[IN]:通过RC522发送到卡片的数据
//          InLenByte[IN]:发送数据的字节长度
//          pOutData[OUT]:接收到的卡片返回数据
//          *pOutLenBit[OUT]:返回数据的位长度
/////////////////////////////////////////////////////////////////////
char PcdComMF522(
									unsigned char Command, 
									unsigned char *pInData, 
									unsigned char InLenByte,
									unsigned char *pOutData, 
									unsigned int  *pOutLenBit)
{
	char status = MI_ERR;
	unsigned char irqEn   = 0x00;
	unsigned char waitFor = 0x00;
	unsigned char lastBits;
	unsigned char n;
	unsigned int i;
	
	switch (Command)
	{
		case PCD_AUTHENT:			//验证密钥
			irqEn   = 0x12;
			waitFor = 0x10;
			break;
		case PCD_TRANSCEIVE:	//发送并接收数据
			irqEn   = 0x77;
			waitFor = 0x30;
			break;
		default:
		 break;
	}
	//mfrc522_check_status();
	//-------------------------------------发送设置
	mfrc522_write_register(ComIEnReg,irqEn|0x80);		//使能相关中断
	mfrc522_del_Bit_Mask(ComIrqReg,0x80);						//清除相关中断标志
	mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
	mfrc522_write_register(CommandReg,PCD_IDLE);		//取消当前命令
	mfrc522_set_Bit_Mask(FIFOLevelReg,0x80);				//清除当前缓存中的字节数
	//-------------------------------------写入FIFO
	for (i=0; i<InLenByte; i++)
	{
		mfrc522_write_data(FIFODataReg, pInData[i]);	//往FIFO缓存写入数据
	}
	//-------------------------------------开启发送
	mfrc522_write_register(CommandReg, Command);		//启动发送


	if (Command == PCD_TRANSCEIVE)
	{
		mfrc522_set_Bit_Mask(BitFramingReg,0x80);		//全部发送
	}
	
	//-------------------------------------中断标识寄存器，检查有无接收到数据
  mfrc522_delay_ms(8);
//    i = 600;//根据时钟频率调整，操作M1卡最大等待时间25ms

	i = 500;	//根据时钟频率调整，操作M1卡最大等待时间25ms
//	do 
//	{
//		n = mfrc522_read_register(ComIrqReg);		//读取中断寄存器
//		mfrc522_delay_us(20);
//		i--;
//		memcpy(&mfrc522_RegSts.ComIrqSts,&n,1);
//	}
//	while ((i>0) && (1!=mfrc522_RegSts.ComIrqSts.TimerIRq) && !(n&waitFor));
////	while ((i!=0) && !(n&0x01) && !(n&waitFor));

	
	for(i=2500;i>0;i--) 
	{
		n = mfrc522_read_register(ComIrqReg);		//读取中断寄存器
		mfrc522_delay_us(10);
		memcpy(&mfrc522_RegSts.ComIrqSts,&n,1);
		
		if(1==mfrc522_RegSts.ComIrqSts.TimerIRq)	//定时时间到
		{
			break;
		}
		if(1==mfrc522_RegSts.ComIrqSts.IdleIRq)
		{
			break;
		}
		if(PCD_TRANSCEIVE==Command)
		{
			if((1==mfrc522_RegSts.ComIrqSts.IdleIRq)||(1==mfrc522_RegSts.ComIrqSts.RxIRq))
			{
				break;
			}
		}
	}
	mfrc522_delay_ms(2);
	mfrc522_read_data(ControlReg);
	mfrc522_delay_ms(2);
	//-------------------------------------清除标识
	mfrc522_del_Bit_Mask(BitFramingReg,0x80);		//清除	
	//-------------------------------------检查接收状态并读取数据
	if (i!=0)
	{    
		if(!(mfrc522_read_register(ErrorReg)&0x1B))		//读错误状态，检查有无错误
		{
			status = MI_OK;
			if (n & irqEn & 0x01)
			{	
				status = MI_NOTAGERR;
			}
			if (Command == PCD_TRANSCEIVE)	//发送并接收数据命令
			{
				mfrc522_delay_ms(3);
				n = mfrc522_read_register(FIFOLevelReg);	//读取FIFO中数据个数
				mfrc522_delay_ms(3);
				lastBits = mfrc522_read_register(ControlReg) & 0x07;		//读取最后接收字节有效位个数
				if (lastBits)
				{
					*pOutLenBit = (n-1)*8 + lastBits;
				}
				else
				{
					*pOutLenBit = n*8;
				}
				if (n == 0)
				{
					n = 1;
				}
				if (n > MAXRLEN)
				{
					n = MAXRLEN;
				}
				for (i=0; i<n; i++)
				{
					pOutData[i] = mfrc522_read_register(FIFODataReg);	//从FIFO缓存读取数据					
				}
			}
		}
		else
		{
			status = MI_ERR;
		}			
	}
	//---------------------------
	mfrc522_set_Bit_Mask(ControlReg,0x80);   	// stop timer now
	mfrc522_write_data(CommandReg,PCD_IDLE);	//进入空闲模式
	return status;
}



/////////////////////////////////////////////////////////////////////
//功    能：扣款和充值
//参数说明: dd_mode[IN]：命令字
//               0xC0 = 扣款
//               0xC1 = 充值
//          addr[IN]：钱包地址
//          pValue[IN]：4字节增(减)值，低位在前
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////                 
char PcdValue(unsigned char dd_mode,unsigned char addr,unsigned char *pValue)
{
    char status;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = dd_mode;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
        
    if (status == MI_OK)
    {
        memcpy(ucComMF522Buf, pValue, 4);
 //       for (i=0; i<16; i++)
 //       {    ucComMF522Buf[i] = *(pValue+i);   }
        CalulateCRC(ucComMF522Buf,4,&ucComMF522Buf[4]);
        unLen = 0;
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen);
        if (status != MI_ERR)
        {    status = MI_OK;    }
    }
    
    if (status == MI_OK)
    {
        ucComMF522Buf[0] = PICC_TRANSFER;
        ucComMF522Buf[1] = addr;
        CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]); 
   
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：备份钱包
//参数说明: sourceaddr[IN]：源地址
//          goaladdr[IN]：目标地址
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdBakValue(unsigned char sourceaddr, unsigned char goaladdr)
{
    char status;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_RESTORE;
    ucComMF522Buf[1] = sourceaddr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
    
    if (status == MI_OK)
    {
        ucComMF522Buf[0] = 0;
        ucComMF522Buf[1] = 0;
        ucComMF522Buf[2] = 0;
        ucComMF522Buf[3] = 0;
        CalulateCRC(ucComMF522Buf,4,&ucComMF522Buf[4]);
 
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen);
        if (status != MI_ERR)
        {    status = MI_OK;    }
    }
    
    if (status != MI_OK)
    {    return MI_ERR;   }
    
    ucComMF522Buf[0] = PICC_TRANSFER;
    ucComMF522Buf[1] = goaladdr;

    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }

    return status;
}

///////////////////////////////////////////////////////////////////////
// Delay us
///////////////////////////////////////////////////////////////////////
static void mfrc522_delay_us(unsigned short us)
{
	#include "STM32_SYSTICK.H"
	SysTick_DeleyuS(us);				//SysTick延时nmS
}
///////////////////////////////////////////////////////////////////////
// Delay ms
///////////////////////////////////////////////////////////////////////
static void mfrc522_delay_ms(unsigned short ms)
{
	#include "STM32_SYSTICK.H"
	SysTick_DeleymS(ms);				//SysTick延时nmS
}
//------------------------------------------------------------------












//-----------------------------------------------static-hardware
/*******************************************************************************
*函数名			:	function
*功能描述		:	配置读卡器接口
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void mfrc522_port_configuration(mfrc522def* pInfo)
{  
	
	GPIO_Configuration_OPP50	(pInfo->port.nss_port,pInfo->port.nss_pin);			//NSS
	GPIO_Configuration_OPP50	(pInfo->port.clk_port,pInfo->port.clk_pin);			//CLK
	GPIO_Configuration_OPP50	(pInfo->port.mosi_port,pInfo->port.mosi_pin);		//MOSI
	GPIO_Configuration_IPU		(pInfo->port.miso_port,pInfo->port.miso_pin);		//MISO
	
	GPIO_Configuration_OPP50	(pInfo->port.rst_port,pInfo->port.rst_pin);			//RESET
}
//------------------------------------------------------------------







//------------------------------------------------------------------


/*******************************************************************************
*函数名			:	function
*功能描述		:	写RC632寄存器
*输入				: Address[IN]:寄存器地址
							value[IN]:写入的值
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void mfrc522_write_byte(unsigned char value)
{
	unsigned char i=0;

	
	//-----------------------------发送地址
	for(i=8;i>0;i--)
	{
		mfrc522_delay_us(1);
		if(0x80	==	(value&0x80))
		{
		 mfrc522_set_mosi_high;	//mosi=1
		}
		else
		{
		 mfrc522_set_mosi_low;		//mosi=	0
		}
		mfrc522_delay_us(1);
		mfrc522_set_clk_high;		//MF522_SCK = 1;
		mfrc522_delay_us(1);
		mfrc522_set_clk_low;			//MF522_SCK = 0;
		value <<= 1;
	}
}

/*******************************************************************************
*函数名			:	function
*功能描述		:	写RC632寄存器
*输入				: Address[IN]:寄存器地址
							value[IN]:写入的值
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void mfrc522_write_data(unsigned char Address,unsigned char value)
{
	unsigned char ucAddr=0;

	mfrc522_set_clk_low;		//MF522_SCK = 0;
	mfrc522_set_nss_low;		//MF522_NSS = 0;
	
	mfrc522_delay_us(1);
	
	ucAddr = ((Address<<1)&0x7E);
	//ucAddr = (Address&0x7F);			//BIT7==0;write
	
	//-----------------------------发送地址--写数据
	mfrc522_write_byte(ucAddr);
	//-----------------------------发送数据
	mfrc522_write_byte(value);
	//-----------------------------结束
	mfrc522_delay_us(1);
	mfrc522_set_nss_high;		//MF522_NSS = 1
	mfrc522_set_clk_high;		//MF522_SCK = 1;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	写RC632寄存器
*输入				: Address[IN]:寄存器地址
							value[IN]:写入的值
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void mfrc522_write_buffer(unsigned char Address,unsigned char* tx_buffer,unsigned char len)
{
	unsigned char i, ucAddr;

	mfrc522_set_clk_low;		//MF522_SCK = 0;
	mfrc522_set_nss_low;		//MF522_NSS = 0;
	
	mfrc522_delay_us(1);
	
	ucAddr = ((Address<<1)&0x7E);
	//ucAddr = (Address&0x7F);			//BIT7==0;write
	
	//-----------------------------发送地址--写数据
	mfrc522_write_byte(ucAddr);
	//-----------------------------发送数据
	for(i=0;i<len;i++)
	{
		mfrc522_write_byte(tx_buffer[i]);
	}	
	//-----------------------------结束
	mfrc522_delay_us(1);
	mfrc522_set_nss_high;		//MF522_NSS = 1
	mfrc522_set_clk_high;		//MF522_SCK = 1;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	写RC632寄存器
*输入				: Address[IN]:寄存器地址
							value[IN]:写入的值
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void mfrc522_write_register(unsigned char Address,unsigned char value)
{
	mfrc522_write_data(Address,value);
}
/*******************************************************************************
*函数名			:	mfrc522_read_byte
*功能描述		:	读RC632寄存器
*输入				: Address[IN]:寄存器地址
*返回值			:	读出的值
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char mfrc522_read_byte(void)
{
	unsigned char i	=	0;
	unsigned char ucResult=0;
	//-----------------------------读取数据
	for(i=8;i>0;i--)
	{
		mfrc522_set_clk_high;		//MF522_SCK = 1;
		mfrc522_delay_us(1);
		ucResult <<= 1;
		if(mfrc522_get_miso)			//miso==1
		{
			ucResult|=0x01;
		}
		mfrc522_set_clk_low;			//MF522_SCK = 0;
		mfrc522_delay_us(1);
	}	
	return ucResult;
}


/*******************************************************************************
*函数名			:	mfrc522_read_byte
*功能描述		:	读RC632寄存器
*输入				: Address[IN]:寄存器地址
*返回值			:	读出的值
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char mfrc522_read_data(unsigned char Address)
{
	unsigned char ucAddr	=	0;
	unsigned char ucResult=0;

	mfrc522_set_clk_low;		//MF522_SCK = 0;
	mfrc522_set_nss_low;		//MF522_NSS = 0;
  mfrc522_delay_us(1);
	
	ucAddr = ((Address<<1)&0x7E)|0x80;
	
	//ucAddr = (Address|0x80);			//BIT7==1;read
	//-----------------------------发送地址
	mfrc522_write_byte(ucAddr);
	mfrc522_delay_us(1);
	//-----------------------------读取数据
	ucResult	=	mfrc522_read_byte();
	//-----------------------------结束
	mfrc522_delay_us(1);
	mfrc522_set_nss_high;		//MF522_NSS = 1
	mfrc522_set_clk_high;		//MF522_SCK = 1;
	
	return ucResult;
}
/*******************************************************************************
*函数名			:	mfrc522_read_byte
*功能描述		:	读RC632寄存器
*输入				: Address[IN]:寄存器地址
*返回值			:	读出的值
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char mfrc522_read_buffer(unsigned char Address,unsigned char* rx_buffer,unsigned char len)
{
	unsigned char i	=	0;
	unsigned char ucAddr	=	0;
	unsigned char ucResult=0;

	mfrc522_set_clk_low;		//MF522_SCK = 0;
	mfrc522_set_nss_low;		//MF522_NSS = 0;
  mfrc522_delay_us(1);
	
	ucAddr = ((Address<<1)&0x7E)|0x80;	
	//ucAddr = (Address|0x80);			//BIT7==1;read
	
	//-----------------------------发送地址
	mfrc522_write_byte(ucAddr);
	mfrc522_delay_us(1);
	//-----------------------------读取数据
	for(i=0;i<len;i++)
	{
		rx_buffer[i]=mfrc522_read_byte();
	}
	//-----------------------------结束
	mfrc522_delay_us(1);
	mfrc522_set_nss_high;		//MF522_NSS = 1
	mfrc522_set_clk_high;		//MF522_SCK = 1;
	
	return ucResult;
}
/*******************************************************************************
*函数名			:	mfrc522_read_byte
*功能描述		:	读RC632寄存器
*输入				: Address[IN]:寄存器地址
*返回值			:	读出的值
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char mfrc522_read_register(unsigned char Address)
{
	unsigned char value	=	0;
	unsigned char* status	=	NULL;
	value	=	mfrc522_read_data(Address);
	switch(Address)
	{
		// PAGE 0		控制和状态寄存器
		case CommandReg		:status	=	(unsigned char*)&mfrc522_RegSts.CommandSts;			break;
		case ComIEnReg		:status	=	(unsigned char*)&mfrc522_RegSts.ComIEnSts;			break;
		case DivIEnReg		:status	=	(unsigned char*)&mfrc522_RegSts.DivIEnSts;			break;
		case ComIrqReg		:status	=	(unsigned char*)&mfrc522_RegSts.ComIrqSts;			break;
		case DivIrqReg		:status	=	(unsigned char*)&mfrc522_RegSts.DivIrqSts;			break;
		case ErrorReg			:status	=	(unsigned char*)&mfrc522_RegSts.ErrorSts;				break;
		case Status1Reg		:status	=	(unsigned char*)&mfrc522_RegSts.Status1RegSts;	break;
		case Status2Reg		:status	=	(unsigned char*)&mfrc522_RegSts.Status2RegSts;	break;
		case FIFOLevelReg	:status	=	(unsigned char*)&mfrc522_RegSts.FIFOLevelSts;		break;
		case ControlReg		:status	=	(unsigned char*)&mfrc522_RegSts.ControlSts;			break;
		
		// PAGE 1  	通信寄存器 
		
		// PAGE 2 	配置寄存器
		case TReloadRegH					:status	=	(unsigned char*)&mfrc522_RegSts.TReloadVal_Hi;	break;
		case TReloadRegL					:status	=	(unsigned char*)&mfrc522_RegSts.TReloadVal_Lo;	break;
		case TCounterValueRegH		:status	=	(unsigned char*)&mfrc522_RegSts.TCounterVal_Hi;	break;
		case TCounterValueRegL		:status	=	(unsigned char*)&mfrc522_RegSts.TCounterVal_Lo;	break;
		
		// PAGE 3  	Test寄存器
		default :break;
	}
	if(NULL!=status)
		memcpy(status,&value,1);		//复制数据
	return value;
}
//------------------------------------------------------------------


/*******************************************************************************
*函数名			:	mfrc522_set_reset
*功能描述		:	RC522
*输入				: 
*返回值			:	成功返回MI_OK
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
char mfrc522_set_reset(void)
{
	mfrc522_set_rst_high;		//MF522_RST=1;
//	mfrc522_delay_us(200);
	mfrc522_set_rst_low;		//MF522_RST=0;
//	mfrc522_delay_us(200);
	mfrc522_set_rst_high;		//MF522_RST=1;
//	mfrc522_delay_us(200);
	
	mfrc522_write_register(CommandReg,PCD_RESETPHASE);	//复位mfrc522   
	mfrc522_write_register(ModeReg,0x3D);            		//和Mifare卡通讯，CRC初始值0x6363
	mfrc522_write_register(TReloadRegL,30); 						//TReloadReg 12位的定时器的装载值(高位)寄存器          
	mfrc522_write_register(TReloadRegH,0);
	mfrc522_write_register(TModeReg,0x8D);
	mfrc522_write_register(TPrescalerReg,0x3E);
	mfrc522_write_register(TxAutoReg,0x40);     
	return MI_OK;
}
/////////////////////////////////////////////////////////////////////
//开启天线  
//每次启动或关闭天线发射之间应至少有1ms的间隔
/////////////////////////////////////////////////////////////////////
void mfrc522_set_AntennaOn(void)
{
	unsigned char i;
	i = mfrc522_read_data(TxControlReg);
	if (!(i & 0x03))
	{
		mfrc522_set_Bit_Mask(TxControlReg, 0x03);
	}
}


/////////////////////////////////////////////////////////////////////
//关闭天线
/////////////////////////////////////////////////////////////////////
void mfrc522_set_AntennaOff(void)
{
	mfrc522_del_Bit_Mask(TxControlReg, 0x03);
}
/////////////////////////////////////////////////////////////////////
//功    能：命令卡片进入休眠状态
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char mfrc522_set_Halt(void)
{
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_HALT;
    ucComMF522Buf[1] = 0;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
		PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    return MI_OK;
}
//------------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	mfrc522_set_Bit_Mask
*功能描述		:	设置RC522寄存器标志位--按位设置
*输入				: reg:寄存器地址
*返回值			:	mask:置位值
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void mfrc522_set_Bit_Mask(unsigned char reg,unsigned char mask)  
{
    char tmp = 0x0;
    tmp = mfrc522_read_data(reg);
    mfrc522_write_data(reg,tmp | mask);  // set bit mask
}
/*******************************************************************************
*函数名			:	mfrc522_del_Bit_Mask
*功能描述		:	清RC522寄存器标志位--按位清除
*输入				: reg:寄存器地址
*返回值			:	mask:清位值
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void mfrc522_del_Bit_Mask(unsigned char reg,unsigned char mask)  
{
    char tmp = 0x0;
    tmp = mfrc522_read_data(reg);
    mfrc522_write_data(reg, tmp & ~mask);  // clear bit mask
}
//------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void mfrc522_test(void)
{
	unsigned char i	=	0;
	
//	mfrc522_del_Bit_Mask(Status2Reg,0x08);				//寄存器包含接收器和发送器和数据模式检测器的状态标志
//	mfrc522_write_data(BitFramingReg,0x07);				//定义将发送数据的最后一个字节的位的数量
//	mfrc522_set_Bit_Mask(TxControlReg,0x03);			//TX1、TX2输出信号将传递经发送数据调制的13.56MHz的能量载波信号。
	
//	mfrc522_read_register(TCounterValueRegH);
//	mfrc522_read_register(TCounterValueRegL);
	
	mfrc522_check_status();
	return;
//	mfrc522_read_register(TReloadRegH);							//读定时器装载值-高4位
//	mfrc522_read_register(TReloadRegL);							//读定时器装载值-低8位
	
	
	mfrc522_read_register(TCounterValueRegH);				//读定时器倒计数值-高4位
	mfrc522_read_register(TCounterValueRegL);				//读定时器倒计数值-低8位
	return;
	mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
	mfrc522_del_Bit_Mask(ComIrqReg,0x80);					//清除相关中断标志
	mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
	mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
	mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
	mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
	
	mfrc522_read_register(TCounterValueRegH);				//读定时器倒计数值-高4位
	mfrc522_read_register(TCounterValueRegL);				//读定时器倒计数值-低8位
	
	mfrc522_write_register(TCounterValueRegL,0);		//修改倒计数值
	mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
	mfrc522_read_register(TCounterValueRegL);				//读定时器倒计数值-低8位
	mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
	mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
	mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
	mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
	mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
	mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
	
	
	mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
	if(1==mfrc522_RegSts.ComIrqSts.TimerIRq)				//定时器中断
	{
		mfrc522_del_Bit_Mask(ComIrqReg,0x80);					//清除相关中断标志
		
		mfrc522_write_register(TCounterValueRegL,0);
		mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
		mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
		mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
		mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
		mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
		mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
		
		mfrc522_write_register(CommandReg, PCD_TRANSCEIVE);		//启动发送
		mfrc522_set_Bit_Mask(BitFramingReg,0x80);		//全部发送
		
//		unsigned char mfrc522_UID[5],mfrc522_Temp[4];
//		if(mfrc522_PcdRequest(PICC_REQALL)!=Mifare_none)	//寻卡
//		{
//			//寻卡成功
//			if(mfrc522_PcdAnticoll(mfrc522_UID)==MI_OK)
//			{ 
////				GPIO_Toggle(GPIOC,GPIO_Pin_13);
//			}
//		}
	
//		unsigned char irqEn   = 0x00;
//		
//		mfrc522_del_Bit_Mask(ComIrqReg,0x80);					//清除相关中断标志
//		
//		mfrc522_del_Bit_Mask(Status2Reg,0x08);				//寄存器包含接收器和发送器和数据模式检测器的状态标志
//		mfrc522_write_data(BitFramingReg,0x07);				//定义将发送数据的最后一个字节的位的数量
//		mfrc522_set_Bit_Mask(TxControlReg,0x03);			//使能TX1、TX2输出，TX1、TX2输出信号将传递经发送数据调制的13.56MHz的能量载波信号。
//		
//		
//		//-------------------------------------发送设置
//		mfrc522_write_register(ComIEnReg,irqEn|0x80);		//使能相关中断
//		mfrc522_del_Bit_Mask(ComIrqReg,0x80);						//清除相关中断标志
//		mfrc522_read_register(ComIrqReg);								//读中断寄存器---20190419添加测试查看数据
//		mfrc522_write_register(CommandReg,PCD_IDLE);		//取消当前命令
//		mfrc522_set_Bit_Mask(FIFOLevelReg,0x80);				//清除当前缓存中的字节数
		
//		for (i=0; i<1; i++)
//		{
//			mfrc522_write_data(FIFODataReg, 0x00);	//往FIFO缓存写入数据
//		}
		//-------------------------------------开启发送
		
//		mfrc522_write_register(CommandReg, PCD_TRANSCEIVE);		//启动发送
//		mfrc522_set_Bit_Mask(BitFramingReg,0x80);		//全部发送

		
	}
	
//	mfrc522_set_clk_low;		//MF522_SCK = 0;
//	mfrc522_set_nss_low;		//MF522_NSS = 0;
//	
//	mfrc522_delay_ms(1);
//	
//	for(i=0;i<8;i++)
//	{
//		mfrc522_write_byte(i);
//	}
//	
//	mfrc522_delay_ms(1);
//	//-----------------------------结束
//	mfrc522_set_nss_high;		//MF522_NSS = 1
//	mfrc522_set_clk_high;		//MF522_SCK = 1;
}
//------------------------------------------------------------------




