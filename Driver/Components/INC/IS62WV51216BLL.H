#ifndef __IS62WV51216BLL_H
#define __IS62WV51216BLL_H

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
	}port;
}mfrc522def;




#endif
