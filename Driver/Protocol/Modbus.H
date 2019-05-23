#ifndef __Modbus_H
#define __Modbus_H



typedef enum _Modbus_Cmd
{
	ReadCoils								=	0x01,		//读从机线圈寄存器，位操作，可读单个或者多个；
	ReadDiscreteInputs			=	0x02,		//读离散输入寄存器，位操作，可读单个或多个
	ReadHoldingRegisters		=	0x03,		//读保持寄存器，字节指令操作，可读单个或者多个
	ReadInputRegisters			=	0x04,		//读输入寄存器，字节指令操作，可读单个或者多个
	WriteSingleCoil					=	0x05,		//写单个线圈，位操作，只能写一个，写0xff00表示设置线圈状态为ON，写0x0000表示设置线圈状态为OFF
	WriteSingleRegister			=	0x06,		//写单个保持寄存器，字节指令操作，只能写一个
	ReadExceptionStatus			=	0x07,		//
	Diagnostics							=	0x08,		//诊断
	GetCommEventCounter			=	0x0B,		//
	GetCommEventLog					=	0x0C,		//
	WriteMultipleCoils			=	0x0F,		//写多个线圈寄存器。若数据区的某位值为“1”表示被请求的相应线圈状态为ON，若某位值为“0”，则为状态为OFF。
	WriteMultipleregisters	=	0x10,		//写多个保持寄存器，字节指令操作，可写多个
	ReportServerID					=	0x11,		//
	ReadFileRecord					=	0x14,		//
	WriteFileRecord					=	0x15,		//
	MaskWriteRegister				=	0x16,		//
}Modbus_Cmd_def;

#endif

