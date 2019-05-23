/**************************************************************************************************
*波特率：19200
*校验位：偶校验
*停止位：1
**************************************************************************************************/
#ifndef __BT459A_H_
#define __BT459A_H


#include 	"CRC.H"
#include "Modbus.H"

#define reg_zero					0x000b		//零点寄存器地址
#define reg_quantity			0x0030		//数量寄存器地址
#define	reg_weight				0x002c		//重量寄存器地址
#define	reg_counterpoise	0x0020		//标定砝码重量
#define	reg_range					0x0014		//满量程寄存器地址



unsigned char api_bt459a_get_frame_ReadQuantity(unsigned char addr,unsigned char* pbuffer);
unsigned char api_bt459a_get_frame_ReadWeight(unsigned char addr,unsigned char* pbuffer);
unsigned char api_bt459a_get_frame_SetQuantity(unsigned char addr,unsigned short Quantity,unsigned char* pbuffer);

#endif
/************************************** The End Of FILE **************************************/
