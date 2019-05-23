/**************************************************************************************************
*�����ʣ�19200
*У��λ��żУ��
*ֹͣλ��1
**************************************************************************************************/
#ifndef __BT459A_H_
#define __BT459A_H


#include 	"CRC.H"
#include "Modbus.H"

#define reg_zero					0x000b		//���Ĵ�����ַ
#define reg_quantity			0x0030		//�����Ĵ�����ַ
#define	reg_weight				0x002c		//�����Ĵ�����ַ
#define	reg_counterpoise	0x0020		//�궨��������
#define	reg_range					0x0014		//�����̼Ĵ�����ַ



unsigned char api_bt459a_get_frame_ReadQuantity(unsigned char addr,unsigned char* pbuffer);
unsigned char api_bt459a_get_frame_ReadWeight(unsigned char addr,unsigned char* pbuffer);
unsigned char api_bt459a_get_frame_SetQuantity(unsigned char addr,unsigned short Quantity,unsigned char* pbuffer);

#endif
/************************************** The End Of FILE **************************************/
