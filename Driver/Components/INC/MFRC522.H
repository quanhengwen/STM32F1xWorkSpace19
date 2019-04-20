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

typedef enum _mfrc522_card_type		//卡片类型代码
{
	Mifare_none				=	0x0000,
	Mifare_UltraLight	=	0x4400,
	Mifare_One_S50		=	0x0400,
	Mifare_One_S70		=	0x0200,
	Mifare_Pro_X			=	0x0800,
	Mifare_DESFire		=	0x4403,
}mfrc522_card_typedef;

//-----------------------------------------------API_
void api_mfrc522_configuration(mfrc522def* pInfo);
void api_mfrc522_self_test(mfrc522def* pInfo);


//-----------------------------------------------static-hardware
static void mfrc522_port_configuration(mfrc522def* pInfo);
//-----------------------------------------------static-hardware-rst
//static void mfrc522_set_rst_high(void);
//static void mfrc522_set_rst_low(void);
//-----------------------------------------------static-hardware-nss
//static void mfrc522_set_nss_high(void);
//static void mfrc522_set_nss_low(void);
//-----------------------------------------------static-hardware-clk
//static void mfrc522_set_clk_high(void);
//static void mfrc522_set_clk_low(void);
//-----------------------------------------------static-hardware-mosi
//static void mfrc522_set_mosi_high(void);
//static void mfrc522_set_mosi_low(void);
//-----------------------------------------------static-hardware-mosi
//static unsigned short mfrc522_get_miso(void);


//-----------------------------------------------
static void mfrc522_write_byte(unsigned char value);
static void mfrc522_write_register(unsigned char Address,unsigned char value);
static void mfrc522_write_data(unsigned char Address,unsigned char value);
static void mfrc522_write_buffer(unsigned char Address,unsigned char* tx_buffer,unsigned char len);

static unsigned char mfrc522_read_byte(void);
static unsigned char mfrc522_read_register(unsigned char Address);
static unsigned char mfrc522_read_data(unsigned char Address);
static unsigned char mfrc522_read_buffer(unsigned char Address,unsigned char* rx_buffer,unsigned char len);

//-----------------------------------------------
char mfrc522_set_reset(void);
void mfrc522_set_AntennaOn(void);
void mfrc522_set_AntennaOff(void);
char mfrc522_set_Halt(void);		//命令卡片进入休眠状态
void mfrc522_set_Bit_Mask(unsigned char reg,unsigned char mask); 
void mfrc522_del_Bit_Mask(unsigned char reg,unsigned char mask); 


//-----------------------------------------------
mfrc522_card_typedef mfrc522_PcdRequest(unsigned char req_code);
char mfrc522_PcdComMF522(unsigned char Command,unsigned char *pInData,unsigned char InLenByte,unsigned char *pOutData,unsigned int  *pOutLenBit);		//通过RC522和ISO14443卡通讯
char mfrc522_PcdAnticoll(unsigned char *pSnr);

static void mfrc522_check_status(void);
//-----------------------------------------------
static void mfrc522_delay_us(unsigned short us);
static void mfrc522_delay_ms(unsigned short ms);
/////////////////////////////////////////////////////////////////////
//函数原型
/////////////////////////////////////////////////////////////////////


char PcdRequest(unsigned char req_code,unsigned char *pTagType); 		//寻卡  
char PcdAnticoll(unsigned char *pSnr);
char PcdSelect(unsigned char *pSnr);         
char PcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr);     
char PcdRead(unsigned char addr,unsigned char *pData);     
char PcdWrite(unsigned char addr,unsigned char *pData);    
char PcdValue(unsigned char dd_mode,unsigned char addr,unsigned char *pValue);   
char PcdBakValue(unsigned char sourceaddr, unsigned char goaladdr);                                 

char PcdComMF522(
									unsigned char Command, 
                 unsigned char *pInData, 
                 unsigned char InLenByte,
                 unsigned char *pOutData, 
                 unsigned int  *pOutLenBit);		//通过RC522和ISO14443卡通讯
void CalulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData);

 

char M500PcdConfigISOType(unsigned char type);




void mfrc522_test(void);

								 
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
#define     DivIEnReg             0x03  	//使能和禁用中断请求的控制。  
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
#define     TReloadRegH           0x2C		//TReloadReg 12位的定时器的装载值(高位)寄存器
#define     TReloadRegL           0x2D		//TReloadReg 12位的定时器的装载值(低位)寄存器
#define     TCounterValueRegH     0x2E		//12位的定时器的当前倒计数值(高位)寄存器
#define     TCounterValueRegL     0x2F		//12位的定时器的当前倒计数值(低位)寄存器

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
#define     RFU3F		  						0x3F


//------------指令寄存器0x01:复位值:20h,指令的启动和停止
typedef struct _CommandReg_bit
{	
	unsigned char	Command			:4;		//bit0-3	根据这些位的值激活相应的指令，对此寄存器进行读操作可得知正在运行的是哪些指令
	unsigned char	PowerDown		:1;		//bit4		1-启动软件掉电模式;0-MFCV520在掉电模式下置该位为0则立即进入唤醒步骤在此期间此位的值还为1；MFCV520 在初始化后该位为0；
	unsigned char	RcvOff			:1;		//bit5		1-关闭接收器的模拟部分
	unsigned char	reserved		:2;		//bit6-7
}CommandReg_bit_def;
//------------中断控制寄存器0x02:复位值:80h,使能和禁用中断请求的控制
typedef struct _ComIEnReg_bit	
{	
	unsigned char	TimerIEn		:1;		//bit0		允许定时器中断请求(TimerIRq bit) 送至IRQ引脚。
	unsigned char	ErrIEn			:1;		//bit1		允许错误中断请求(ErrIRq bit) 送至IRQ引脚。
	unsigned char	LoAlertIEn	:1;		//bit2		允许空（下溢）报警中断请求(LoAlertIRq bit) 送至IRQ引脚。
	unsigned char	HiAlertIEn	:1;		//bit3		允许溢出报警中断请求(HiAlertIRq bit)送至IRQ引脚。
	unsigned char	IdleIEn			:1;		//bit4		允许空闲中断请求(IdleIRq) 送至IRQ引脚。
	unsigned char	RxIEn				:1;		//bit5		允许接收中断请求(RxIRq) 送至IRQ引脚。
	unsigned char TxIEn				:1;		//bit6		允许发送中断请求(TxIRq) 送至IRQ引脚。
	unsigned char IRqInv			:1;		//bit7		1-IRQ 引脚的电平状态和Status1Reg 寄存器的IRq 位的状态相反。;0-IRQ引脚的电平状态和Status1Reg 寄存器的IRq 位的状态相等；和DivIEnReg 寄存器的IRqPushPull 位组合，缺省值为1，可确保IRQ输出为三态输出。	
}ComIEnReg_def;
//------------中断控制寄存器0x03:复位值:00h,使能和禁用中断请求的控制
typedef struct _DivIEnReg_bit
{	
	unsigned char	reserved0		:2;		//bit0-1	
	unsigned char	CRCIEn			:1;		//bit2		允许由DivIrqReg 寄存器的CRCIRq 位表示的CRC中断请求送至IRQ引脚。
	unsigned char	reserved1		:1;		//bit3		
	unsigned char	MfinActIEn	:1;		//bit4		允许MFIN中断请求送至IRQ引脚
	unsigned char	reserved2		:2;		//bit5-6	
	unsigned char	IRQPushPull	:1;		//bit7		1-IRQ输出为标准CMOS 输出。;0-IRQ输出为漏极开路输出。
}DivIEnReg_bit_def;
//------------中断请求寄存器0x04:复位值:14h,寄存器的所有位都可以由软件清
typedef struct _ComIrqReg_bit
{	
	unsigned char	TimerIRq		:1;		//bit0		寄存器TCounterValReg 中的定时值减至0时该位置位。
	unsigned char	ErrIRq			:1;		//bit1		ErrorReg 寄存器的任何error位被置位时该位置位
	unsigned char	LoAlertIRq	:1;		//bit2		FIFO缓冲区快为空时置位，当Status1Reg 寄存器的LoAlert 置位时该位置位和 LoAlert 位相反， LoAlertIRq 位保存着此事件并只能由此寄存器的Set1 位复位。
	unsigned char	HiAlertIRq	:1;		//bit3		FIFO缓冲区快溢出时置位，当Status1Reg 寄存器的HiAlert 置位时该位置位和 HiAlert 位相反， HiAlertIRq 位保存着此事件并只能由此寄存器的Set1 位复位。
	unsigned char	IdleIRq			:1;		//bit4		如果一个未知指令启动，则CommandReg 寄存器的Command[3:0]的值为空闲状态并且IdleIRq 位置1。微控器启动Idle指令并不会使IdleIRq 位置1。
	unsigned char	RxIRq				:1;		//bit5		接收器检测到有效数据的末位:如果RxModeReg寄存器的RxNoErr位置1,则当FIFO中的接收的数据有效时RxIRq 位置1
	unsigned char TxIRq				:1;		//bit6		数据的最后一位发送完成后立即置1
	unsigned char Set1				:1;		//bit7		1-置位ComIrqReg 寄存器中标记的位;0-清除InterruptIRq中标记的位	
}ComIrqReg_bit_def;
//------------中断请求寄存器0x05:复位值:x0h,寄存器的所有位都可以由软件清
typedef struct _DivIrqReg_bit
{	
	unsigned char	reserved0		:1;		//bit0-1		
	unsigned char	CRCIRq			:1;		//bit2		1-CalcCRC 指令处于运行状态且所有的数据都检查完毕
	unsigned char	reserved1		:1;		//bit3		
	unsigned char	MfinActIRq	:1;		//bit4		1-MFIN激活;当检测到信号的上升沿或者下降沿时置位此中断位
	unsigned char	reserved2		:1;		//bit5-6	如果一个未知指令启动，则CommandReg 寄存器的Command[3:0]的值为空闲状态并且IdleIRq 位置1。微控器启动Idle指令并不会使IdleIRq 位置1。
	unsigned char	Set2				:1;		//bit7		1-置位DivIrqReg 寄存器中的标记的位。;0-清零DivIrqReg 寄存器中的标记的位。
}DivIrqReg_bit_def;
//------------错误状态寄存器0x06:复位值:00h,显示上一个指令执行的错误状态。
typedef struct _ErrorReg_bit
{	
	unsigned char	ProtocolErr	:1;		//bit0		SOF错误时该位置1,在接收器启动阶段自动清零,只有在以106kbit/s波特率通信时有效,当MFAuthent指令执行期间，如果有接收到不正确的数据时则ProtocolErr位置1		
	unsigned char	ParityErr		:1;		//bit1		奇偶校验错误时该位置1,在接收器启动阶段自动清零,只有在以ISO/IEC 14443 A/MIFARE 协议的106k bit/s 波特率通信时有效
	unsigned char	CRCErr			:1;		//bit2		CRC 计算错误并且RxModeReg 寄存器的RxCRCEn 位置位时该位置1
	unsigned char	CollErr			:1;		//bit3		检查出一个位冲突时该位置1,在接收器的启动阶段自动清零,只有在以防冲突机制的106k bit/s 波特率通信时有效通常在以212k bit/s,424k bit/s, 848k bit/s 波特率通信时为0
	unsigned char	BufferOvfl	:1;		//bit4		尽管FIFO缓冲区已满，但主机或内部状态机(e.g. receiver)还试图向里面写数据时该位置1
	unsigned char	reserved		:1;		//bit5		
	unsigned char	TempErr			:1;		//bit6		内部温度传感器检测出超温时该位置1, 在此情况下天线驱动器自动关闭
	unsigned char	WrErr				:1;		//bit7		在MFAuthent 指令执行期间或在通过RF 接口发送和接收最后一位数据的期间内主机还在向FIFO 缓冲区里面写数据时该位置1
}ErrorReg_bit_def;
//------------状态1寄存器0x07:复位值:21h,包含CRC，中断和FIFO 缓冲器的一些位的状态
typedef struct _Status1Reg_bit
{	
	unsigned char	LoAlert			:1;		//bit0		FIFO 缓冲器中存储的数据的长度满足以下方程时该位置1:FIFO length = 4, WaterLevel = 4 → LoAlert = 1;FIFO length = 5, WaterLevel = 4 → LoAlert = 0	
	unsigned char	HiAlert			:1;		//bit1		FIFO缓冲器中存储的数据的长度满足以下方程时该位置1:FIFO length = 60, WaterLevel = 4 → HiAlert = 1;FIFO length = 59, WaterLevel = 4 → HiAlert = 0
	unsigned char	reserved0		:1;		//bit2		CRC 计算错误并且RxModeReg 寄存器的RxCRCEn 位置位时该位置1
	unsigned char	TRunning		:1;		//bit3		MF CV520 的定时器运行时该位置1，定时器将以寄存器TCounterValReg 中存储的定时值递减。
	unsigned char	IRq					:1;		//bit4		表示任何中断源的请求能否被接受还取决于中断使能位的设置: 参见寄存器ComIEnReg 和DivIEnReg
	unsigned char	CRCReady		:1;		//bit5		CRC计算完成时该位置1,只对执行了CalcCRC指令的CRC计算有效
	unsigned char	CRCOk				:1;		//bit6		CRC结果为0 时该位置1;数据的发送和接收时, CRCOk 的值不确定,表示CRC协处理器的状态, 在CRC校验期间值为0,当校验结果正确时该位置1。
	unsigned char	reserved1		:1;		//bit7		
}Status1Reg_bit_def;
//------------状态2寄存器0x08:复位值:00h,包含接收器，发送器和数据模式检测器的一些位的状态
typedef struct _Status2Reg_bit
{	
	unsigned char	ModemState	:3;		//bit0-2	表示发送器和接受器状态机的状态:
																	//				000-空闲
																	//				001-等待BitFramingReg 寄存器的StartSend位
																	//				010-TxWait:如果ModeReg寄存器的TxWaitRF位置1时则一直等待到RF场产生为止,TxWait的最短时间由TxWaitReg寄存器确定
																	//				011-发送
																	//				100-RxWait: 如果TModeReg寄存器的TxWaitRF位置1时则一直等待到RF 场产生为止RxWait 的最短时间由RxWaitReg 寄存器确定
																	//				101-数据等待
																	//				110-接收
	unsigned char	MFCrypto1On	:1;		//bit3		表示MIFARE Crypto1 单元打开，加密和卡通信时的所有数据只有成功执行MFAuthent指令后才可置该位为1,只对MIFARE 标准卡的读写有效此位可由软件清零		
	unsigned char	reserved0		:3;		//bit4-6		CRC 计算错误并且RxModeReg 寄存器的RxCRCEn 位置位时该位置1
	unsigned char	TempSensClear	:1;	//bit7		当温度低于125 °C极限报警值时该位置1 并纠正温度误差
}Status2Reg_bit_def;
//------------FIFO计数寄存器0x0A:复位值:00h,表示FIFO 缓冲器中存储数据的字节数.
typedef struct _FIFOLevelReg_bit
{	
	unsigned char	FIFOLevel		:7;		//bit0-6	表示FIFO 缓冲区中存储数据的字节数,向FIFODataReg 寄存器写数据时，FIFOLevel 的值会增加，反之从FIFODataReg 寄存器读数据时，FIFOLevel 的值会减小
	unsigned char	FlushBuffer	:1;		//bit7		该位置1时则立即清零FIFO缓冲区的读写指针和ErrorReg寄存器的BufferOvfl位,读取该位时返回值总为0
}FIFOLevelReg_bit_def;
//------------FIFO报警寄存器0x0B:复位值:08h,FIFO 缓冲器的溢出（上溢）和空（下溢）报警
typedef struct _WaterLevelReg_bit
{	
	unsigned char	WaterLevel	:6;		//bit0-5	定义一个报警界限值来表示FIFO缓冲区的溢出和空:
																	//				如果FIFO 缓冲器中剩余的空间少于或等于WaterLevel 中定义的值，则Status1Reg 寄存器的HiAlert 位置1。
																	//				如果FIFO缓冲器中数据所用的空间少于或等于WaterLevel中定义的值，则Status1Reg 寄存器的LoAlert 位置1。
	unsigned char	reserved		:2;		//bit6-7			
}WaterLevelReg_bit_def;
//------------其它控制位0x0C:复位值:10h,包含定时器的开启、关闭和接收有效位个数
typedef struct _ControlReg_bit
{	
	unsigned char	RxLastBits	:3;		//bit0-2	表示最后接收字节的有效位的个数，如果此值为000B，则整个字节都是有效的
	unsigned char	reserved		:3;		//bit3-5
	unsigned char	TStartNow		:1;		//bit6		定时器立即启动,读取该位时返回值为0
	unsigned char	TStopNow		:1;		//bit7		定时器立即停止,读取该位时返回值为0
}ControlReg_bit_def;



// PAGE 1  	通信寄存器

// PAGE 2 	配置寄存器

// PAGE 3  	Test寄存器

typedef struct _mfrc522_RegSts
{
	// PAGE 0		控制和状态寄存器
	CommandReg_bit_def		CommandSts;			//指令寄存器0x01:复位值:20h,指令的启动和停止
	ComIEnReg_def					ComIEnSts;			//中断控制寄存器0x02:复位值:80h,使能和禁用中断请求的控制
	DivIEnReg_bit_def			DivIEnSts;			//中断控制寄存器0x03:复位值:00h,使能和禁用中断请求的控制
	ComIrqReg_bit_def			ComIrqSts;			//中断请求寄存器0x04:复位值:14h,寄存器的所有位都可以由软件清
	DivIrqReg_bit_def			DivIrqSts;			//中断请求寄存器0x05:复位值:x0h,寄存器的所有位都可以由软件清
	ErrorReg_bit_def			ErrorSts;				//错误状态寄存器0x06:复位值:00h,显示上一个指令执行的错误状态。
	Status1Reg_bit_def		Status1RegSts;	//状态1寄存器0x07:复位值:21h,包含CRC，中断和FIFO 缓冲器的一些位的状态
	Status2Reg_bit_def		Status2RegSts;	//状态2寄存器0x08:复位值:00h,包含接收器，发送器和数据模式检测器的一些位的状态
	FIFOLevelReg_bit_def	FIFOLevelSts;		//FIFO计数寄存器0x0A:复位值:00h,表示FIFO 缓冲器中存储数据的字节数.
	ControlReg_bit_def		ControlSts;			//其它控制位0x0C:复位值:10h,包含定时器的开启、关闭和接收有效位个数
	
	// PAGE 1  	通信寄存器
	
	// PAGE 2 	配置寄存器
	unsigned char TReloadVal_Hi;					//TReloadReg 12位的定时器的装载值(高位)寄存器
	unsigned char	TReloadVal_Lo;					//TReloadReg 12位的定时器的装载值(低位)寄存器
	unsigned char TCounterVal_Hi;					//12位的定时器的当前倒计数值(高位)寄存器--高4位
	unsigned char	TCounterVal_Lo;					//12位的定时器的当前倒计数值(低位)寄存器--低8位
	
	// PAGE 3  	Test寄存器
	
}mfrc522_RegStsDef;
/////////////////////////////////////////////////////////////////////
//和MF522通讯时返回的错误代码
/////////////////////////////////////////////////////////////////////
#define MI_OK                          0		//无错误
#define MI_NOTAGERR                    (-1)
#define MI_ERR                         (-2)




#endif
