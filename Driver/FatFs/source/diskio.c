/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		  /* FatFs lower layer API */
#include "FatFsAPI.h"	  /* �Զ���API�ӿ�*/

/* Definitions of physical drive number for each drive */
#define DEV_RAM		0	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/
//====================����__weakռλ����START20180825
//-------------------disk_status
//__weak int RAM_disk_status(void)
//{
//  return 0;
//}
__weak int MMC_disk_status(void)
{
  return 0;
}
__weak int USB_disk_status(void)
{
  return 0;
}
//-------------------disk_initialize
//__weak int RAM_disk_initialize(void)
//{
//  return 0;
//}
__weak int MMC_disk_initialize(void)
{
  return 0;
}
__weak int USB_disk_initialize(void)
{
  return 0;
}
//-------------------disk_read
//__weak int RAM_disk_read(BYTE *buff,DWORD sector,UINT count)
//{
//  return 0;
//}
__weak int MMC_disk_read(BYTE *buff,DWORD sector,UINT count)
{
  return 0;
}
__weak int USB_disk_read(BYTE *buff,DWORD sector,UINT count)
{
  return 0;
}
//-------------------disk_write
//__weak int RAM_disk_write(const BYTE *buff,DWORD sector,UINT count)
//{
//  return 0;
//}
__weak int MMC_disk_write(const BYTE *buff,DWORD sector,UINT count)
{
  return 0;
}
__weak int USB_disk_write(const BYTE *buff,DWORD sector,UINT count)
{
  return 0;
}
//====================����__weakռλ����END
DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
	case DEV_RAM :
//		result = RAM_disk_status();
    
		// translate the reslut code here
    result = SD_disk_status();
		return stat;

	case DEV_MMC :
		result = MMC_disk_status();

		// translate the reslut code here

		return stat;

	case DEV_USB :
		result = USB_disk_status();

		// translate the reslut code here

		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv)
  {
	case DEV_RAM :
//		result = RAM_disk_initialize();
    

    // translate the reslut code here
    result = SD_disk_initialize();

		return stat;

	case DEV_MMC :
		result = MMC_disk_initialize();

		// translate the reslut code here

		return stat;

	case DEV_USB :
		result = USB_disk_initialize();

		// translate the reslut code here

		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
/*******************************************************************************
*������			:	Read Sector(s) 
*��������		:	������               
*����				: BYTE pdrv--Ҫ��ȡ�������������ţ����̺ţ�
              buff--��ȡ�����ݻ���
              sector--����ȡ����ʼ������
              count --����ȡ����������
*����ֵ			:	
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*Ӧ�þ���		: 
              
*ע��				:	wegam@sina.com
*******************************************************************************/
DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;
//	int result;

	switch (pdrv) {
	case DEV_RAM :
    // translate the reslut code here
//		result = RAM_disk_read(buff, sector, count);

		// translate the reslut code here
    res = SD_disk_read(buff, sector, count);

		return res;

	case DEV_MMC :
		// translate the arguments here

		res = MMC_disk_read(buff, sector, count);

		// translate the reslut code here

		return res;

	case DEV_USB :
		// translate the arguments here

		res = USB_disk_read(buff, sector, count);

		// translate the reslut code here

		return res;
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/
/*******************************************************************************
*������			:	Write Sector(s)
*��������		:	д������               
*����				: BYTE pdrv--Ҫд��������������ţ����̺ţ�
              buff--���ݻ���
              sector--��д�����ʼ������
              count --��д�����������
*����ֵ			:	
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*Ӧ�þ���		: 
              
*ע��				:	wegam@sina.com
*******************************************************************************/
DRESULT disk_write (
                    BYTE pdrv,			  /* Physical drive nmuber to identify the drive */
                    const BYTE *buff,	/* Data to be written */
                    DWORD sector,		  /* Start sector in LBA */
                    UINT count			  /* Number of sectors to write */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
	case DEV_RAM :
		// translate the arguments here

//		result = RAM_disk_write(buff, sector, count);

		// translate the reslut code here
    result = SD_disk_write(buff, sector, count);
		return res;

	case DEV_MMC :
		// translate the arguments here

		result = MMC_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;

	case DEV_USB :
		// translate the arguments here

		result = USB_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,  /* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff  /* Buffer to send/receive control data */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
	case DEV_RAM :

		// Process of the command for the RAM drive

		return res;

	case DEV_MMC :

		// Process of the command for the MMC/SD card

		return res;

	case DEV_USB :

		// Process of the command the USB drive

		return res;
	}

	return RES_PARERR;
}
