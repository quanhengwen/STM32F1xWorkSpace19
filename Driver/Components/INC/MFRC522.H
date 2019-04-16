#ifndef __MFRC522_H
#define __MFRC522_H

#include "STM32_SPI.H"
typedef struct	_mfrc522
{
	struct
	{
		//____________SPI_CS定义
		GPIO_TypeDef* 	nss_port;		//GPIOX
		unsigned short 	nss_pin;			//GPIO_Pin_x

		//____________SPI_CLK定义
		GPIO_TypeDef* 	clk_port;		//GPIOX
		unsigned short 	clk_pin;		//GPIO_Pin_x

		//____________SPI_MISO定义
		GPIO_TypeDef* 	miso_port;	//GPIOX
		unsigned short 	miso_pin;		//GPIO_Pin_x

		//____________SPI_MOSI定义
		GPIO_TypeDef* 	mosi_port;	//GPIOX
		unsigned short 	mosi_pin;		//GPIO_Pin_x
			
		//____________SPI_REST定义
		GPIO_TypeDef* 	rst_port;	//GPIOX
		unsigned short 	rst_pin;		//GPIO_Pin_x
	}port;
}mfrc522def;
//-----------------------------------------------API_
void api_mfrc522_configuration(mfrc522def* pInfo);
void api_mfrc522_self_test(mfrc522def* pInfo);


//-----------------------------------------------static-hardware
static void mfrc522_port_configuration(mfrc522def* pInfo);
//-----------------------------------------------static-hardware-rst
static void mfrc522_set_rst_high(mfrc522def* pInfo);
static void mfrc522_set_rst_low(mfrc522def* pInfo);
//-----------------------------------------------static-hardware-nss
static void mfrc522_set_nss_high(mfrc522def* pInfo);
static void mfrc522_set_nss_low(mfrc522def* pInfo);
//-----------------------------------------------static-hardware-clk
static void mfrc522_set_clk_high(mfrc522def* pInfo);
static void mfrc522_set_clk_low(mfrc522def* pInfo);
//-----------------------------------------------static-hardware-mosi
static void mfrc522_set_mosi_high(mfrc522def* pInfo);
static void mfrc522_set_mosi_low(mfrc522def* pInfo);
//-----------------------------------------------static-hardware-mosi
static unsigned short mfrc522_get_miso(mfrc522def* pInfo);

//-----------------------------------------------
static void mfrc522_write_byte(mfrc522def* pInfo,unsigned char value);
static void mfrc522_write_data(mfrc522def* pInfo,unsigned char Address,unsigned char value);
static void mfrc522_write_buffer(mfrc522def* pInfo,unsigned char Address,unsigned char* tx_buffer,unsigned char len);

static unsigned char mfrc522_read_byte(mfrc522def* pInfo);
static unsigned char mfrc522_read_data(mfrc522def* pInfo,unsigned char Address);
static unsigned char mfrc522_read_buffer(mfrc522def* pInfo,unsigned char Address,unsigned char* rx_buffer,unsigned char len);

//-----------------------------------------------
char mfrc522_set_reset(mfrc522def* pInfo);
void mfrc522_set_AntennaOn(mfrc522def* pInfo);
void mfrc522_set_AntennaOff(mfrc522def* pInfo);
char mfrc522_set_Halt(mfrc522def* pInfo);		//命令卡片进入休眠状态



//-----------------------------------------------
static void mfrc522_delay_us(unsigned short us);
static void mfrc522_delay_ms(unsigned short ms);
/////////////////////////////////////////////////////////////////////
//函数原型
/////////////////////////////////////////////////////////////////////


char PcdRequest(mfrc522def* pInfo,unsigned char req_code,unsigned char *pTagType); 		//寻卡  
char PcdAnticoll(mfrc522def* pInfo,unsigned char *pSnr);
char PcdSelect(mfrc522def* pInfo,unsigned char *pSnr);         
char PcdAuthState(mfrc522def* pInfo,unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr);     
char PcdRead(mfrc522def* pInfo,unsigned char addr,unsigned char *pData);     
char PcdWrite(mfrc522def* pInfo,unsigned char addr,unsigned char *pData);    
char PcdValue(mfrc522def* pInfo,unsigned char dd_mode,unsigned char addr,unsigned char *pValue);   
char PcdBakValue(mfrc522def* pInfo,unsigned char sourceaddr, unsigned char goaladdr);                                 

char PcdComMF522(mfrc522def* pInfo,
									unsigned char Command, 
                 unsigned char *pInData, 
                 unsigned char InLenByte,
                 unsigned char *pOutData, 
                 unsigned int  *pOutLenBit);		//通过RC522和ISO14443卡通讯
void CalulateCRC(mfrc522def* pInfo,unsigned char *pIndata,unsigned char len,unsigned char *pOutData);

 
void SetBitMask(mfrc522def* pInfo,unsigned char reg,unsigned char mask); 
void ClearBitMask(mfrc522def* pInfo,unsigned char reg,unsigned char mask); 
char M500PcdConfigISOType(mfrc522def* pInfo,unsigned char type);

void iccardcode();
//char PcdBakValue(unsigned char sourceaddr, unsigned char goaladdr);
//char PcdValue(mfrc522def* pInfo,unsigned char dd_mode,unsigned char addr,unsigned char *pValue);


								 
/////////////////////////////////////////////////////////////////////
//MF522命令字---PCD是接近式卡
/////////////////////////////////////////////////////////////////////
#define PCD_IDLE              0x00               //取消当前命令
#define PCD_AUTHENT           0x0E               //验证密钥/MIFARE 标准认证
#define PCD_RECEIVE           0x08               //接收数据/激活接收电路
#define PCD_TRANSMIT          0x04               //发送数据
#define PCD_TRANSCEIVE        0x0C               //从FIFO中发送数据到天线并在发送后自动开始接收
#define PCD_RESETPHASE        0x0F               //复位	/MF_CV520 的软复位
#define PCD_CALCCRC           0x03               //CRC计算

/////////////////////////////////////////////////////////////////////
//Mifare_One卡片命令字---PICC是接近式耦合设备
/////////////////////////////////////////////////////////////////////
#define PICC_REQIDL           0x26               //寻天线区内未进入休眠状态
#define PICC_REQALL           0x52               //寻天线区内全部卡
#define PICC_ANTICOLL1        0x93               //防冲撞
#define PICC_ANTICOLL2        0x95               //防冲撞
#define PICC_AUTHENT1A        0x60               //验证A密钥
#define PICC_AUTHENT1B        0x61               //验证B密钥
#define PICC_READ             0x30               //读块
#define PICC_WRITE            0xA0               //写块
#define PICC_DECREMENT        0xC0               //扣款
#define PICC_INCREMENT        0xC1               //充值
#define PICC_RESTORE          0xC2               //调块数据到缓冲区
#define PICC_TRANSFER         0xB0               //保存缓冲区中数据
#define PICC_HALT             0x50               //休眠

/////////////////////////////////////////////////////////////////////
//MF522 FIFO长度定义
/////////////////////////////////////////////////////////////////////
#define DEF_FIFO_LENGTH       64                 //FIFO size=64byte

/////////////////////////////////////////////////////////////////////
//MF522寄存器定义
/////////////////////////////////////////////////////////////////////
// PAGE 0		控制和状态寄存器
#define     RFU00                 0x00   	//保留 
#define     CommandReg            0x01   	//指令的启动和停止. 
#define     ComIEnReg             0x02  	//使能和禁用中断请求的控制。  
#define     DivlEnReg             0x03  	//使能和禁用中断请求的控制。  
#define     ComIrqReg             0x04    //中断请求。
#define     DivIrqReg             0x05		//中断请求。
#define     ErrorReg              0x06   	//Error 寄存器显示上一个指令执行的错误状态 
#define     Status1Reg            0x07   	//包含CRC，中断和FIFO 缓冲器的一些位的状态。 
#define     Status2Reg            0x08		//包含接收器，发送器和数据模式检测器的一些位的状态
#define     FIFODataReg           0x09		//64字节的FIFO 缓冲器的输入与输出.
#define     FIFOLevelReg          0x0A		//表示FIFO缓冲器中存储数据的字节数.
#define     WaterLevelReg         0x0B		//FIFO 缓冲器的溢出（上溢）和空（下溢）报警
#define     ControlReg            0x0C		//其它控制位。
#define     BitFramingReg         0x0D		//位方式帧的调整.
#define     CollReg               0x0E		//定义RF 接口上检测到的第一个位冲突
#define     RFU0F                 0x0F

// PAGE 1  	通信寄存器  
#define     RFU10                 0x10		//保留
#define     ModeReg               0x11		//发送和接收的通用模式的设置.
#define     TxModeReg             0x12		//数据发送时的波特率
#define     RxModeReg             0x13		//数据接收时的波特率.
#define     TxControlReg          0x14		//控制天线驱动器引脚TX1 和TX2的逻辑状态。
#define     TxAutoReg             0x15		//发送调制的设置
#define     TxSelReg              0x16		//选择模拟模块的内部信号源。
#define     RxSelReg              0x17		//内部接收器的设置。
#define     RxThresholdReg        0x18		//位解码器阀值的选择
#define     DemodReg              0x19		//定义解调器的设置
#define     RFU1A                 0x1A		//保留
#define     RFU1B                 0x1B		//保留
#define     MifareReg             0x1C		//MIFARE通信传输参数的设置
#define     RFU1D                 0x1D		//保留
#define     RFU1E                 0x1E		//保留
#define     SerialSpeedReg        0x1F

// PAGE 2 	配置寄存器   
#define     RFU20                 0x20  	//保留
#define     CRCResultRegM         0x21		//CRCResultReg (高位字节)寄存器
#define     CRCResultRegL         0x22		//CRCResultReg (低位字节)寄存器
#define     RFU23                 0x23		//保留
#define     ModWidthReg           0x24		//设定调制宽度
#define     RFU25                 0x25		//保留
#define     RFCfgReg              0x26		//接收器增益的配置
#define     GsNReg                0x27		//当驱动器开启时，定义天线驱动器TX1 和TX2为n-driver 时的电导率
#define     CWGsCfgReg            0x28		//定义p-driver 非调制时的输出电导率.
#define     ModGsCfgReg           0x29		//定义p-driver调制时的输出电导率
#define     TModeReg              0x2A		//这些寄存器定义定时器的设置
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C		//TReloadReg (高位)寄存器
#define     TReloadRegL           0x2D		//TReloadReg (低位)寄存器
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F

// PAGE 3  	Test寄存器
#define     RFU30                 0x30		//保留
#define     TestSel1Reg           0x31		//测试信号的配置
#define     TestSel2Reg           0x32		//测试信号的配置和PRBS控制.
#define     TestPinEnReg          0x33		//使能测试总线的输出驱动器。
#define     TestPinValueReg       0x34		//定义当测试引脚D1 至D7 用于I/O 时的电平值。
#define     TestBusReg            0x35		//显示内部测试总线的状态
#define     AutoTestReg           0x36		//数字自检的控制.
#define     VersionReg            0x37		//显示MF_CV520 的软件版本
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39  
#define     TestDAC2Reg           0x3A   	//定义TestDAC2 的测试值
#define     TestADCReg            0x3B   	//显示 ADC I 和 Q通道值
#define     RFU3C                 0x3C   
#define     RFU3D                 0x3D   
#define     RFU3E                 0x3E   
#define     RFU3F		  0x3F

/////////////////////////////////////////////////////////////////////
//和MF522通讯时返回的错误代码
/////////////////////////////////////////////////////////////////////
#define MI_OK                          0		//无错误
#define MI_NOTAGERR                    (-1)
#define MI_ERR                         (-2)



#endif
