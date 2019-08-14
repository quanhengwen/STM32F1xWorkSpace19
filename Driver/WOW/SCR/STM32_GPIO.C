/******************************** User_library *********************************
* 文件名 	: STM32_SDCard.H
* 作者   	: wegam@sina.com
* 版本   	: V
* 日期   	: 2016/01/01
* 说明   	: 
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#include "STM32_GPIO.H"


#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_type.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_map.h"



//#include "STM32F10x_BitBand.H"
//#include "stm32f10x_gpio.h"



/*******************************************************************************
* 函数名		:	GPIO_ClockCmd	
* 功能描述	:	开启相关GPIO端口时钟	 
* 输入		:	GPIOx：GPIOA~GPIOG
						GPIO_Pin_x:GPIO_Pin_0~GPIO_Pin_15;GPIO_Pin_All---部分IO需要重定向接口
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_ClockConf(GPIO_TypeDef* GPIOx,						//GPIO端口			
										u16 GPIO_Pin_x									//GPIO引脚
										)		//开启相关GPIO时钟	 
{
	if(0	==	GPIOx)
	{
		return;
	}
	assert_param(IS_GPIO_ALL_PERIPH(GPIOx)); 
	switch (*(u32*)&GPIOx)
	{
		//********************GPIOA时钟使能********************	
		case GPIOA_BASE:
//			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			if(((GPIO_Pin_x&GPIO_Pin_13)==GPIO_Pin_13)||((GPIO_Pin_x&GPIO_Pin_14)==GPIO_Pin_14)||((GPIO_Pin_x&GPIO_Pin_15)==GPIO_Pin_15)||(GPIO_Pin_x==GPIO_Pin_All))
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
				//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);			//关闭SW功能
				GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);		//关闭JTAG,SW功能开启
			}
			else
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			break;
		//********************GPIOB时钟使能********************
		case GPIOB_BASE:
			if(((GPIO_Pin_x&GPIO_Pin_3)==GPIO_Pin_3)||((GPIO_Pin_x&GPIO_Pin_4)==GPIO_Pin_4)||(GPIO_Pin_x==GPIO_Pin_All))
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
				GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);				//关闭JTAG
			}
			else
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			break;
		//********************GPIOC时钟使能********************
		case GPIOC_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
			if(((GPIO_Pin_x&GPIO_Pin_14)==GPIO_Pin_14)||((GPIO_Pin_x&GPIO_Pin_15)==GPIO_Pin_15))
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_AFIO, ENABLE);
			break;
		//********************GPIOD时钟使能********************
		case GPIOD_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
			break;
		//********************GPIOE时钟使能********************
		case GPIOE_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
			break;
		//********************GPIOF时钟使能********************
		case GPIOF_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
			break;
		//********************GPIOG时钟使能********************
		case GPIOG_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
			break;
		
		default: break;
		
	}

}

/*******************************************************************************
* 函数名		:	GPIO_DeInitAll
* 功能描述	:	将所有的GPIO关闭----V20170605
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_DeInitAll(void)
{
	//1）GPIO结构体定义
	GPIO_InitTypeDef	GPIO_InitStructure;
	//2）初始化GPIO结构体
	GPIO_StructInit(&GPIO_InitStructure);			//GPIO_Pin_All，GPIO_Speed_2MHz，GPIO_Mode_IN_FLOATING
	//3）初始化GPIO为默认状态
	GPIO_Init(GPIOA,&GPIO_InitStructure);			//GPIOA
	GPIO_Init(GPIOB,&GPIO_InitStructure);			//GPIOB
	GPIO_Init(GPIOC,&GPIO_InitStructure);			//GPIOC
	GPIO_Init(GPIOD,&GPIO_InitStructure);			//GPIOD
	GPIO_Init(GPIOE,&GPIO_InitStructure);			//GPIOE
	GPIO_Init(GPIOF,&GPIO_InitStructure);			//GPIOF
	GPIO_Init(GPIOG,&GPIO_InitStructure);			//GPIOG
	//4）关闭GPIO时钟
	GPIO_DeInit(GPIOA);												//GPIOA
	GPIO_DeInit(GPIOB);												//GPIOB
	GPIO_DeInit(GPIOC);												//GPIOC
	GPIO_DeInit(GPIOD);												//GPIOD
	GPIO_DeInit(GPIOE);												//GPIOE
	GPIO_DeInit(GPIOF);												//GPIOF
	GPIO_DeInit(GPIOG);												//GPIOG
	//4）关闭AFIO
	GPIO_AFIODeInit();												//AFIO关闭	
}

/*******************************************************************************
* 函数名		:	GPIO_InitStructure_INA
* 功能描述	:	将GPIO相应管脚配置为模拟输入模式----V20170605
* 输入		:	GPIOx--GPIO端口，GPIO_Pin_n--GPIO管脚号
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_Configuration_INA(
														GPIO_TypeDef* GPIOx,		//GPIO端口,x=A/B/C/D/E/F/G
														u16 GPIO_Pin_n					//GPIO管脚号n=0~15/All
														)
{
	//1）GPIO结构体定义
	GPIO_InitTypeDef	GPIO_InitStructure;
	//2）初始化GPIO结构体
  GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_n;
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;		//输入模式下时钟配置无效
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AIN;			//模拟输入模式
	//3)根据GPIO端口及管脚号开启端口时钟及确定是否打开AFIO时钟
	GPIO_ClockConf(GPIOx,GPIO_Pin_n);
	//4）初始化GPIO
	GPIO_Init(GPIOx,&GPIO_InitStructure);			//GPIOA
}
/*******************************************************************************
* 函数名		:	GPIO_InitStructure_INF
* 功能描述	:	将GPIO相应管脚配置为浮空输入模式----V20170605
* 输入		:	GPIOx--GPIO端口，GPIO_Pin_n--GPIO管脚号
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_Configuration_INF(
														GPIO_TypeDef* GPIOx,							//GPIO端口,x=A/B/C/D/E/F/G
														u16 GPIO_Pin_n										//GPIO管脚号n=0~15/All
														)
{
	//1）GPIO结构体定义
	GPIO_InitTypeDef	GPIO_InitStructure;
	//2）初始化GPIO结构体
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_n;
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;					//输入模式下时钟配置无效
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_IN_FLOATING;		//浮空输入模式
	//3)根据GPIO端口及管脚号开启端口时钟及确定是否打开AFIO时钟
	GPIO_ClockConf(GPIOx,GPIO_Pin_n);
	//3）初始化GPIO
	GPIO_Init(GPIOx,&GPIO_InitStructure);			//GPIOA	
}

/*******************************************************************************
* 函数名		:	GPIO_Configuration_IPD
* 功能描述	:	将GPIO相应管脚配置为下拉输入模式----V20170605
* 输入		:	GPIOx--GPIO端口，GPIO_Pin_n--GPIO管脚号
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_Configuration_IPD(
														GPIO_TypeDef* GPIOx,							//GPIO端口,x=A/B/C/D/E/F/G
														u16 GPIO_Pin_n										//GPIO管脚号n=0~15/All
														)
{
	//1）GPIO结构体定义
	GPIO_InitTypeDef	GPIO_InitStructure;
	//2）初始化GPIO结构体
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_n;
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;					//输入模式下时钟配置无效
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_IPD;						//下拉输入模式
	//3)根据GPIO端口及管脚号开启端口时钟及确定是否打开AFIO时钟
	GPIO_ClockConf(GPIOx,GPIO_Pin_n);
	//3）初始化GPIO
	GPIO_Init(GPIOx,&GPIO_InitStructure);			//GPIOA	
}
/*******************************************************************************
* 函数名		:	GPIO_Configuration_IPU
* 功能描述	:	将GPIO相应管脚配置为上拉输入模式----V20170605
* 输入		:	GPIOx--GPIO端口，GPIO_Pin_n--GPIO管脚号
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_Configuration_IPU(
														GPIO_TypeDef* GPIOx,							//GPIO端口,x=A/B/C/D/E/F/G
														u16 GPIO_Pin_n										//GPIO管脚号n=0~15/All
														)
{
	//1）GPIO结构体定义
	GPIO_InitTypeDef	GPIO_InitStructure;
	//2）初始化GPIO结构体
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_n;
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;					//输入模式下时钟配置无效
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_IPU;						//上拉输入模式
	//3)根据GPIO端口及管脚号开启端口时钟及确定是否打开AFIO时钟
	GPIO_ClockConf(GPIOx,GPIO_Pin_n);
	//3）初始化GPIO
	GPIO_Init(GPIOx,&GPIO_InitStructure);			//GPIOA	
}


/*******************************************************************************
* 函数名		:	GPIO_Configuration_OOD2
* 功能描述	:	将GPIO相应管脚配置为OD(开漏)输出模式，最大速度2MHz----V20170605
* 输入		:	GPIOx--GPIO端口，GPIO_Pin_n--GPIO管脚号
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_Configuration_OOD2(
														GPIO_TypeDef* GPIOx,							//GPIO端口,x=A/B/C/D/E/F/G
														u16 GPIO_Pin_n										//GPIO管脚号n=0~15/All
														)
{
	//1）GPIO结构体定义
	GPIO_InitTypeDef	GPIO_InitStructure;
	//2）初始化GPIO结构体
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_n;
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;					//最大速度频率2MHz
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_Out_OD;					//OD(开漏)输出模式
	//3)根据GPIO端口及管脚号开启端口时钟及确定是否打开AFIO时钟
	GPIO_ClockConf(GPIOx,GPIO_Pin_n);
	//3）初始化GPIO
	GPIO_Init(GPIOx,&GPIO_InitStructure);			//GPIOA	
}
/*******************************************************************************
* 函数名		:	GPIO_Configuration_OOD2
* 功能描述	:	将GPIO相应管脚配置为OD(开漏)输出模式，最大速度10MHz----V20170605
* 输入		:	GPIOx--GPIO端口，GPIO_Pin_n--GPIO管脚号
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_Configuration_OOD10(
														GPIO_TypeDef* GPIOx,							//GPIO端口,x=A/B/C/D/E/F/G
														u16 GPIO_Pin_n										//GPIO管脚号n=0~15/All
														)
{
	//1）GPIO结构体定义
	GPIO_InitTypeDef	GPIO_InitStructure;
	//2）初始化GPIO结构体
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_n;
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_10MHz;					//最大速度频率10MHz
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_Out_OD;					//OD(开漏)输出模式
	//3)根据GPIO端口及管脚号开启端口时钟及确定是否打开AFIO时钟
	GPIO_ClockConf(GPIOx,GPIO_Pin_n);
	//3）初始化GPIO
	GPIO_Init(GPIOx,&GPIO_InitStructure);			//GPIOA	
}
/*******************************************************************************
* 函数名		:	GPIO_Configuration_OOD50
* 功能描述	:	将GPIO相应管脚配置为OD(开漏)输出模式，最大速度50MHz----V20170605
* 输入		:	GPIOx--GPIO端口，GPIO_Pin_n--GPIO管脚号
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_Configuration_OOD50(
														GPIO_TypeDef* GPIOx,							//GPIO端口,x=A/B/C/D/E/F/G
														u16 GPIO_Pin_n										//GPIO管脚号n=0~15/All
														)
{
	//1）GPIO结构体定义
	GPIO_InitTypeDef	GPIO_InitStructure;
	//2）初始化GPIO结构体
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_n;
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;					//最大速度频率50MHz
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_Out_OD;					//OD(开漏)输出模式
	//3)根据GPIO端口及管脚号开启端口时钟及确定是否打开AFIO时钟
	GPIO_ClockConf(GPIOx,GPIO_Pin_n);
	//3）初始化GPIO
	GPIO_Init(GPIOx,&GPIO_InitStructure);			//GPIOA	
}
/*******************************************************************************
* 函数名		:	GPIO_Configuration_OPP2
* 功能描述	:	将GPIO相应管脚配置为PP(推挽)输出模式，最大速度2MHz----V20170605
* 输入		:	GPIOx--GPIO端口，GPIO_Pin_n--GPIO管脚号
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_Configuration_OPP2(
														GPIO_TypeDef* GPIOx,							//GPIO端口,x=A/B/C/D/E/F/G
														u16 GPIO_Pin_n										//GPIO管脚号n=0~15/All
														)
{
	//1）GPIO结构体定义
	GPIO_InitTypeDef	GPIO_InitStructure;
	//2）初始化GPIO结构体
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_n;
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;					//最大速度频率2MHz
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_Out_PP;					//PP(推挽)输出模式
	//3)根据GPIO端口及管脚号开启端口时钟及确定是否打开AFIO时钟
	GPIO_ClockConf(GPIOx,GPIO_Pin_n);
	//3）初始化GPIO
	GPIO_Init(GPIOx,&GPIO_InitStructure);			//GPIOA	
}
/*******************************************************************************
* 函数名		:	GPIO_Configuration_OPP10
* 功能描述	:	将GPIO相应管脚配置为PP(推挽)输出模式，最大速度10MHz----V20170605
* 输入		:	GPIOx--GPIO端口，GPIO_Pin_n--GPIO管脚号
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_Configuration_OPP10(
														GPIO_TypeDef* GPIOx,							//GPIO端口,x=A/B/C/D/E/F/G
														u16 GPIO_Pin_n										//GPIO管脚号n=0~15/All
														)
{
	//1）GPIO结构体定义
	GPIO_InitTypeDef	GPIO_InitStructure;
	//2）初始化GPIO结构体
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_n;
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_10MHz;					//最大速度频率10MHz
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_Out_PP;					//PP(推挽)输出模式
	//3)根据GPIO端口及管脚号开启端口时钟及确定是否打开AFIO时钟
	GPIO_ClockConf(GPIOx,GPIO_Pin_n);
	//3）初始化GPIO
	GPIO_Init(GPIOx,&GPIO_InitStructure);			//GPIOA	
}
/*******************************************************************************
* 函数名		:	GPIO_Configuration_OPP50
* 功能描述	:	将GPIO相应管脚配置为PP(推挽)输出模式，最大速度10MHz----V20170605
* 输入		:	GPIOx--GPIO端口，GPIO_Pin_n--GPIO管脚号
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_Configuration_OPP50(
														GPIO_TypeDef* GPIOx,							//GPIO端口,x=A/B/C/D/E/F/G
														u16 GPIO_Pin_n										//GPIO管脚号n=0~15/All
														)
{
	//1）GPIO结构体定义
	GPIO_InitTypeDef	GPIO_InitStructure;
	//2）初始化GPIO结构体
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_n;
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;					//最大速度频率50MHz
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_Out_PP;					//PP(推挽)输出模式
	//3)根据GPIO端口及管脚号开启端口时钟及确定是否打开AFIO时钟
	GPIO_ClockConf(GPIOx,GPIO_Pin_n);
	//3）初始化GPIO
	GPIO_Init(GPIOx,&GPIO_InitStructure);			//GPIOA	
}

/*******************************************************************************
* 函数名		:	GPIO_Configuration_OPP50
* 功能描述	:	将GPIO相应管脚配置为PP(推挽)输出模式，最大速度10MHz----V20170605
* 输入		:	GPIOx--GPIO端口，GPIO_Pin_n--GPIO管脚号
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_RegConfiguration_OPP50(
														GPIO_TypeDef* GPIOx,							//GPIO端口,x=A/B/C/D/E/F/G
														u16 GPIO_Pin_n										//GPIO管脚号n=0~15/All
														)
{
u32 currentmode = 0x00, currentpin = 0x00, pinpos = 0x00, pos = 0x00;
  u32 tmpreg = 0x00, pinmask = 0x00;
	if(0	==	GPIOx)
	{
		return;
	}
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_MODE(GPIO_InitStruct->GPIO_Mode));
  assert_param(IS_GPIO_PIN(GPIO_InitStruct->GPIO_Pin));
	assert_param(IS_GET_GPIO_PIN(GPIO_InitStruct->GPIO_Pin));   
  
/*---------------------------- GPIO Mode Configuration -----------------------*/
    currentmode |= GPIO_Speed_50MHz;

/*---------------------------- GPIO CRL Configuration ------------------------*/
  /* Configure the eight low port pins */
  if ((GPIO_Pin_n & ((u32)0x00FF)) != 0x00)
  {
    tmpreg = GPIOx->CRL;

    for (pinpos = 0x00; pinpos < 0x08; pinpos++)
    {
      pos = ((u32)0x01) << pinpos;
      /* Get the port pins position */
      currentpin = (GPIO_Pin_n) & pos;

      if (currentpin == pos)
      {
        pos = pinpos << 2;
        /* Clear the corresponding low control register bits */
        pinmask = ((u32)0x0F) << pos;
        tmpreg &= ~pinmask;

        /* Write the mode configuration in the corresponding bits */
        tmpreg |= (currentmode << pos);
      }
    }
    GPIOx->CRL = tmpreg;
  }
/*---------------------------- GPIO CRH Configuration ------------------------*/
  /* Configure the eight high port pins */
  if (GPIO_Pin_n > 0x00FF)
  {
    tmpreg = GPIOx->CRH;
    for (pinpos = 0x00; pinpos < 0x08; pinpos++)
    {
      pos = (((u32)0x01) << (pinpos + 0x08));
      /* Get the port pins position */
      currentpin = ((GPIO_Pin_n) & pos);
      if (currentpin == pos)
      {
        pos = pinpos << 2;
        /* Clear the corresponding high control register bits */
        pinmask = ((u32)0x0F) << pos;
        tmpreg &= ~pinmask;

        /* Write the mode configuration in the corresponding bits */
        tmpreg |= (currentmode << pos);
      }
    }
    GPIOx->CRH = tmpreg;
  }
}
/*******************************************************************************
* 函数名		:	GPIO_Configuration_AOD2
* 功能描述	:	将GPIO相应管脚配置为AOD(复用开漏)输出模式，最大速度2MHz----V20170605
* 输入		:	GPIOx--GPIO端口，GPIO_Pin_n--GPIO管脚号
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_Configuration_AOD2(
														GPIO_TypeDef* GPIOx,							//GPIO端口,x=A/B/C/D/E/F/G
														u16 GPIO_Pin_n										//GPIO管脚号n=0~15/All
														)
{
	//1）GPIO结构体定义
	GPIO_InitTypeDef	GPIO_InitStructure;
	//2）初始化GPIO结构体
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_n;
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;					//最大速度频率2MHz
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF_OD;					//AOD(复用开漏)输出模式
	//3)根据GPIO端口及管脚号开启端口时钟及确定是否打开AFIO时钟
	GPIO_ClockConf(GPIOx,GPIO_Pin_n);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);				//打开复用时钟
	//3）初始化GPIO
	GPIO_Init(GPIOx,&GPIO_InitStructure);			//GPIOA	
}
/*******************************************************************************
* 函数名		:	GPIO_Configuration_AOD10
* 功能描述	:	将GPIO相应管脚配置为AOD(复用开漏)输出模式，最大速度10MHz----V20170605
* 输入		:	GPIOx--GPIO端口，GPIO_Pin_n--GPIO管脚号
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_Configuration_AOD10(
														GPIO_TypeDef* GPIOx,							//GPIO端口,x=A/B/C/D/E/F/G
														u16 GPIO_Pin_n										//GPIO管脚号n=0~15/All
														)
{
	//1）GPIO结构体定义
	GPIO_InitTypeDef	GPIO_InitStructure;
	//2）初始化GPIO结构体
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_n;
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_10MHz;					//最大速度频率10MHz
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF_OD;					//AOD(复用开漏)输出模式
	//3)根据GPIO端口及管脚号开启端口时钟及确定是否打开AFIO时钟
	GPIO_ClockConf(GPIOx,GPIO_Pin_n);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);				//打开复用时钟
	//3）初始化GPIO
	GPIO_Init(GPIOx,&GPIO_InitStructure);			//GPIOA	
}
/*******************************************************************************
* 函数名		:	GPIO_Configuration_AOD50
* 功能描述	:	将GPIO相应管脚配置为AOD(复用开漏)输出模式，最大速度50MHz----V20170605
* 输入		:	GPIOx--GPIO端口，GPIO_Pin_n--GPIO管脚号
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_Configuration_AOD50(
														GPIO_TypeDef* GPIOx,							//GPIO端口,x=A/B/C/D/E/F/G
														u16 GPIO_Pin_n										//GPIO管脚号n=0~15/All
														)
{
	//1）GPIO结构体定义
	GPIO_InitTypeDef	GPIO_InitStructure;
	//2）初始化GPIO结构体
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_n;
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;					//最大速度频率50MHz
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF_OD;					//AOD(复用开漏)输出模式
	//3)根据GPIO端口及管脚号开启端口时钟及确定是否打开AFIO时钟
	GPIO_ClockConf(GPIOx,GPIO_Pin_n);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);				//打开复用时钟
	//3）初始化GPIO
	GPIO_Init(GPIOx,&GPIO_InitStructure);			//GPIOA	
}
/*******************************************************************************
* 函数名		:	GPIO_Configuration_APP2
* 功能描述	:	将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度2MHz----V20170605
* 输入		:	GPIOx--GPIO端口，GPIO_Pin_n--GPIO管脚号
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_Configuration_APP2(
														GPIO_TypeDef* GPIOx,							//GPIO端口,x=A/B/C/D/E/F/G
														u16 GPIO_Pin_n										//GPIO管脚号n=0~15/All
														)
{
	//1）GPIO结构体定义
	GPIO_InitTypeDef	GPIO_InitStructure;
	//2）初始化GPIO结构体
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_n;
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;					//最大速度频率2MHz
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF_PP;					//APP(复用推挽)输出模式
	//3)根据GPIO端口及管脚号开启端口时钟及确定是否打开AFIO时钟
	GPIO_ClockConf(GPIOx,GPIO_Pin_n);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);				//打开复用时钟
	//3）初始化GPIO
	GPIO_Init(GPIOx,&GPIO_InitStructure);			//GPIOA	
}
/*******************************************************************************
* 函数名		:	GPIO_Configuration_APP10
* 功能描述	:	将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度10MHz----V20170605
* 输入		:	GPIOx--GPIO端口，GPIO_Pin_n--GPIO管脚号
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_Configuration_APP10(
														GPIO_TypeDef* GPIOx,							//GPIO端口,x=A/B/C/D/E/F/G
														u16 GPIO_Pin_n										//GPIO管脚号n=0~15/All
														)
{
	//1）GPIO结构体定义
	GPIO_InitTypeDef	GPIO_InitStructure;
	//2）初始化GPIO结构体
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_n;
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_10MHz;					//最大速度频率10MHz
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF_PP;					//APP(复用推挽)输出模式
	//3)根据GPIO端口及管脚号开启端口时钟及确定是否打开AFIO时钟
	GPIO_ClockConf(GPIOx,GPIO_Pin_n);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);				//打开复用时钟
	//3）初始化GPIO
	GPIO_Init(GPIOx,&GPIO_InitStructure);			//GPIOA	
}
/*******************************************************************************
* 函数名		:	GPIO_Configuration_APP50
* 功能描述	:	将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度50MHz----V20170605
* 输入		:	GPIOx--GPIO端口，GPIO_Pin_n--GPIO管脚号
* 输出		:
* 返回 		:
*******************************************************************************/
void GPIO_Configuration_APP50(
														GPIO_TypeDef* GPIOx,							//GPIO端口,x=A/B/C/D/E/F/G
														u16 GPIO_Pin_n										//GPIO管脚号n=0~15/All
														)
{
	//1）GPIO结构体定义
	GPIO_InitTypeDef	GPIO_InitStructure;
	//2）初始化GPIO结构体
	GPIO_InitStructure.GPIO_Pin  		= GPIO_Pin_n;
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;					//最大速度频率50MHz
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF_PP;					//APP(复用推挽)输出模式
	//3)根据GPIO端口及管脚号开启端口时钟及确定是否打开AFIO时钟
	GPIO_ClockConf(GPIOx,GPIO_Pin_n);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);				//打开复用时钟
	//3）初始化GPIO
	GPIO_Init(GPIOx,&GPIO_InitStructure);			//GPIOA	
}





/*******************************************************************************
* 函数名		:	GPIO_Configuration_APP50
* 功能描述	:	将GPIO相应管脚配置为APP(复用推挽)输出模式，最大速度50MHz----V20170605
* 输入		:	GPIOx--GPIO端口，GPIO_Pin_n--GPIO管脚号
* 输出		:
* 返回 		:
*******************************************************************************/
void api_gpio_toggle(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n)
{
	GPIOx->ODR = GPIOx->ODR^GPIO_Pin_n;
}
//------------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	API_GPIO_SET_ClockEN
*功能描述		:	打开相应GPIO时钟
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_gpio_set_rcc_enable(GPIO_TypeDef* GPIOx)
{
	switch (*(u32*)&GPIOx)
	{
		//********************GPIOA时钟使能********************	
		case GPIOA_BASE:
				RCC->APB2ENR |= RCC_APB2Periph_GPIOA;
				AFIO->MAPR	&=		0xFAFFFFFF;	//关闭JTAG,SW功能开启
				AFIO->MAPR	|=		0x02000000;	//关闭JTAG,SW功能开启
			break;
		//********************GPIOB时钟使能********************
		case GPIOB_BASE:			
				RCC->APB2ENR |= RCC_APB2Periph_GPIOB;
				AFIO->MAPR	&=		0xFAFFFFFF;	//关闭JTAG,SW功能开启
				AFIO->MAPR	|=		0x02000000;	//关闭JTAG,SW功能开启			
			break;
		//********************GPIOC时钟使能********************
		case GPIOC_BASE:
			RCC->APB2ENR |= RCC_APB2Periph_GPIOC;
			break;
		//********************GPIOD时钟使能********************
		case GPIOD_BASE:
			RCC->APB2ENR |= RCC_APB2Periph_GPIOD;
			break;
		//********************GPIOE时钟使能********************
		case GPIOE_BASE:
			RCC->APB2ENR |= RCC_APB2Periph_GPIOE;
			break;
		//********************GPIOF时钟使能********************
		case GPIOF_BASE:
			RCC->APB2ENR |= RCC_APB2Periph_GPIOF;
			break;
		//********************GPIOG时钟使能********************
		case GPIOG_BASE:
			RCC->APB2ENR |= RCC_APB2Periph_GPIOG;
			break;
		
		default: break;
	}
	RCC->APB2ENR |= RCC_APB2Periph_AFIO;
}
//------------------------------------------------------------------------------



//******************************************************************************
//													输出模式
//******************************************************************************

//=================================================将GPIO相应管脚配置为PP(推挽)输出模式-寄存器版本，最大速度50MHz----V20190805
/*******************************************************************************
*函数名			:	API_GPIO_SET_OPP50REGxx
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_gpio_set_OPP50_reg_pin_0(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFFFF0;
	GPIOx->CRL|=0x00000003;
}
void api_gpio_set_OPP50_reg_pin_1(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFFF0F;
	GPIOx->CRL|=0x00000030;
}
void api_gpio_set_OPP50_reg_pin_2(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFF0FF;
	GPIOx->CRL|=0x00000300;
}
void api_gpio_set_OPP50_reg_pin_3(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFF0FFF;
	GPIOx->CRL|=0x00003000;
}
void api_gpio_set_OPP50_reg_pin_4(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFF0FFFF;
	GPIOx->CRL|=0x00030000;
}
void api_gpio_set_OPP50_reg_pin_5(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFF0FFFFF;
	GPIOx->CRL|=0x00300000;
}
void api_gpio_set_OPP50_reg_pin_6(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xF0FFFFFF;
	GPIOx->CRL|=0x03000000;
}
void api_gpio_set_OPP50_reg_pin_7(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0x0FFFFFFF;
	GPIOx->CRL|=0x30000000;
}
void api_gpio_set_OPP50_reg_pin_8(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFFFF0;
	GPIOx->CRH|=0x00000003;
}
void api_gpio_set_OPP50_reg_pin_9(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFFF0F;
	GPIOx->CRH|=0x00000030;
}
void api_gpio_set_OPP50_reg_pin_10(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFF0FF;
	GPIOx->CRH|=0x00000300;
}
void api_gpio_set_OPP50_reg_pin_11(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFF0FFF;
	GPIOx->CRH|=0x00003000;
}
void api_gpio_set_OPP50_reg_pin_12(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFF0FFFF;
	GPIOx->CRH|=0x00030000;
}
void api_gpio_set_OPP50_reg_pin_13(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFF0FFFFF;
	GPIOx->CRH|=0x00300000;
}
void api_gpio_set_OPP50_reg_pin_14(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xF0FFFFFF;
	GPIOx->CRH|=0x03000000;
}
void api_gpio_set_OPP50_reg_pin_15(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0x0FFFFFFF;
	GPIOx->CRH|=0x30000000;
}
void api_gpio_set_OPP50_reg_pin_All(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0x00000000;
	GPIOx->CRL|=0x33333333;
	GPIOx->CRH&=0x00000000;
	GPIOx->CRH|=0x33333333;
}
void api_gpio_set_OPP50_reg_pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n)
{
	unsigned char loop=0;
	unsigned long cmp=GPIO_Pin_0;
	unsigned long DataAnd	=0x00000000;
	unsigned long DataOr 	=0x00000000;
	
	if(GPIO_Pin_n&0x00FF)
	{
		cmp=GPIO_Pin_0;
		DataAnd	=0x00000000;
		DataOr 	=0x00000000;
		for(loop=0;loop<8;loop++)
		{		
			if(cmp&GPIO_Pin_n)
			{
				DataAnd	|=	0x00000000<<(4*(loop));
				DataOr	|=	0x00000003<<(4*(loop));
			}
			else
			{
				DataAnd	|=	0x0000000F<<(4*(loop));
				DataOr	|=	0x00000000<<(4*(loop));
			}
			cmp<<=1;
		}
		GPIOx->CRL&=DataAnd;
		GPIOx->CRL|=DataOr;
	}
	
	if(GPIO_Pin_n&0xFF00)
	{
		cmp=GPIO_Pin_8;
		DataAnd	=0x00000000;
		DataOr 	=0x00000000;
		for(loop=0;loop<8;loop++)
		{		
			if(cmp&GPIO_Pin_n)
			{
				DataAnd	|=	0x00000000<<(4*(loop));
				DataOr	|=	0x00000003<<(4*(loop));
			}
			else
			{
				DataAnd	|=	0x0000000F<<(4*(loop));
				DataOr	|=	0x00000000<<(4*(loop));
			}
			cmp<<=1;
		}
		GPIOx->CRH&=DataAnd;
		GPIOx->CRH|=DataOr;
	}
}
//------------------------------------------------------------------------------

//=================================================将GPIO相应管脚配置为OD(开漏)输出模式-寄存器版本，最大速度50MHz----V20190805
/*******************************************************************************
*函数名			:	API_GPIO_Configuration_OOD50REGxx
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_gpio_set_OOD50_reg_pin_0(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFFFF0;
	GPIOx->CRL|=0x00000007;
}
void api_gpio_set_OOD50_reg_pin_1(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFFF0F;
	GPIOx->CRL|=0x00000070;
}
void api_gpio_set_OOD50_reg_pin_2(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFF0FF;
	GPIOx->CRL|=0x00000700;
}
void api_gpio_set_OOD50_reg_pin_3(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFF0FFF;
	GPIOx->CRL|=0x00007000;
}
void api_gpio_set_OOD50_reg_pin_4(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFF0FFFF;
	GPIOx->CRL|=0x00070000;
}
void api_gpio_set_OOD50_reg_pin_5(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFF0FFFFF;
	GPIOx->CRL|=0x00700000;
}
void api_gpio_set_OOD50_reg_pin_6(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xF0FFFFFF;
	GPIOx->CRL|=0x07000000;
}
void api_gpio_set_OOD50_reg_pin_7(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0x0FFFFFFF;
	GPIOx->CRL|=0x70000000;
}
void api_gpio_set_OOD50_reg_pin_8(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFFFF0;
	GPIOx->CRH|=0x00000007;
}
void api_gpio_set_OOD50_reg_pin_9(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFFF0F;
	GPIOx->CRH|=0x00000070;
}
void api_gpio_set_OOD50_reg_pin_10(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFF0FF;
	GPIOx->CRH|=0x00000700;
}
void api_gpio_set_OOD50_reg_pin_11(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFF0FFF;
	GPIOx->CRH|=0x00007000;
}
void api_gpio_set_OOD50_reg_pin_12(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFF0FFFF;
	GPIOx->CRH|=0x00070000;
}
void api_gpio_set_OOD50_reg_pin_13(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFF0FFFFF;
	GPIOx->CRH|=0x00700000;
}
void api_gpio_set_OOD50_reg_pin_14(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xF0FFFFFF;
	GPIOx->CRH|=0x07000000;
}
void api_gpio_set_OOD50_reg_pin_15(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0x0FFFFFFF;
	GPIOx->CRH|=0x70000000;
}
void api_gpio_set_OOD50_reg_pin_All(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0x00000000;
	GPIOx->CRL|=0x77777777;
	GPIOx->CRH&=0x00000000;
	GPIOx->CRH|=0x77777777;
}
void api_gpio_set_OOD50_reg_pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n)
{
	unsigned char loop=0;
	unsigned long cmp=GPIO_Pin_0;
	unsigned long DataAnd	=0x00000000;
	unsigned long DataOr 	=0x00000000;
	
	if(GPIO_Pin_n&0x00FF)
	{
		cmp=GPIO_Pin_0;
		DataAnd	=0x00000000;
		DataOr 	=0x00000000;
		for(loop=0;loop<8;loop++)
		{		
			if(cmp&GPIO_Pin_n)
			{
				DataAnd	|=	0x00000000<<(4*(loop));
				DataOr	|=	0x00000007<<(4*(loop));
			}
			else
			{
				DataAnd	|=	0x0000000F<<(4*(loop));
				DataOr	|=	0x00000000<<(4*(loop));
			}
			cmp<<=1;
		}
		GPIOx->CRL&=DataAnd;
		GPIOx->CRL|=DataOr;
	}
	
	if(GPIO_Pin_n&0xFF00)
	{
		cmp=GPIO_Pin_8;
		DataAnd	=0x00000000;
		DataOr 	=0x00000000;
		for(loop=0;loop<8;loop++)
		{		
			if(cmp&GPIO_Pin_n)
			{
				DataAnd	|=	0x00000000<<(4*(loop));
				DataOr	|=	0x00000007<<(4*(loop));
			}
			else
			{
				DataAnd	|=	0x0000000F<<(4*(loop));
				DataOr	|=	0x00000000<<(4*(loop));
			}
			cmp<<=1;
		}
		GPIOx->CRH&=DataAnd;
		GPIOx->CRH|=DataOr;
	}
}
//------------------------------------------------------------------------------

//=================================================将GPIO相应管脚配置为APP(复用推挽)输出模式-寄存器版本，最大速度50MHz----V20190805
/*******************************************************************************
*函数名			:	API_GPIO_SET_APP50REGxx
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_gpio_set_APP50_reg_pin_0(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFFFF0;
	GPIOx->CRL|=0x0000000B;
}
void api_gpio_set_APP50_reg_pin_1(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFFF0F;
	GPIOx->CRL|=0x000000B0;
}
void api_gpio_set_APP50_reg_pin_2(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFF0FF;
	GPIOx->CRL|=0x00000B00;
}
void api_gpio_set_APP50_reg_pin_3(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFF0FFF;
	GPIOx->CRL|=0x0000B000;
}
void api_gpio_set_APP50_reg_pin_4(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFF0FFFF;
	GPIOx->CRL|=0x000B0000;
}
void api_gpio_set_APP50_reg_pin_5(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFF0FFFFF;
	GPIOx->CRL|=0x00B00000;
}
void api_gpio_set_APP50_reg_pin_6(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xF0FFFFFF;
	GPIOx->CRL|=0x0B000000;
}
void api_gpio_set_APP50_reg_pin_7(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0x0FFFFFFF;
	GPIOx->CRL|=0xB0000000;
}
void api_gpio_set_APP50_reg_pin_8(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFFFF0;
	GPIOx->CRH|=0x0000000B;
}
void api_gpio_set_APP50_reg_pin_9(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFFF0F;
	GPIOx->CRH|=0x000000B0;
}
void api_gpio_set_APP50_reg_pin_10(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFF0FF;
	GPIOx->CRH|=0x00000B00;
}
void api_gpio_set_APP50_reg_pin_11(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFF0FFF;
	GPIOx->CRH|=0x0000B000;
}
void api_gpio_set_APP50_reg_pin_12(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFF0FFFF;
	GPIOx->CRH|=0x000B0000;
}
void api_gpio_set_APP50_reg_pin_13(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFF0FFFFF;
	GPIOx->CRH|=0x00B00000;
}
void api_gpio_set_APP50_reg_pin_14(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xF0FFFFFF;
	GPIOx->CRH|=0x0B000000;
}
void api_gpio_set_APP50_reg_pin_15(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0x0FFFFFFF;
	GPIOx->CRH|=0xB0000000;
}
void api_gpio_set_APP50_reg_pin_All(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0x00000000;
	GPIOx->CRL|=0xBBBBBBBB;
	GPIOx->CRH&=0x00000000;
	GPIOx->CRH|=0xBBBBBBBB;
}
void api_gpio_set_APP50_reg_pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n)
{
	unsigned char loop=0;
	unsigned long cmp=GPIO_Pin_0;
	unsigned long DataAnd	=0x00000000;
	unsigned long DataOr 	=0x00000000;
	
	if(GPIO_Pin_n&0x00FF)
	{
		cmp=GPIO_Pin_0;
		DataAnd	=0x00000000;
		DataOr 	=0x00000000;
		for(loop=0;loop<8;loop++)
		{		
			if(cmp&GPIO_Pin_n)
			{
				DataAnd	|=	0x00000000<<(4*(loop));
				DataOr	|=	0x0000000B<<(4*(loop));
			}
			else
			{
				DataAnd	|=	0x0000000F<<(4*(loop));
				DataOr	|=	0x00000000<<(4*(loop));
			}
			cmp<<=1;
		}
		GPIOx->CRL&=DataAnd;
		GPIOx->CRL|=DataOr;
	}
	
	if(GPIO_Pin_n&0xFF00)
	{
		cmp=GPIO_Pin_8;
		DataAnd	=0x00000000;
		DataOr 	=0x00000000;
		for(loop=0;loop<8;loop++)
		{		
			if(cmp&GPIO_Pin_n)
			{
				DataAnd	|=	0x00000000<<(4*(loop));
				DataOr	|=	0x0000000B<<(4*(loop));
			}
			else
			{
				DataAnd	|=	0x0000000F<<(4*(loop));
				DataOr	|=	0x00000000<<(4*(loop));
			}
			cmp<<=1;
		}
		GPIOx->CRH&=DataAnd;
		GPIOx->CRH|=DataOr;
	}
}
//------------------------------------------------------------------------------

//=================================================将GPIO相应管脚配置为AOD(复用开漏)输出模式-寄存器版本，最大速度50MHz----V20190805
/*******************************************************************************
*函数名			:	API_GPIO_SET_AOD50REGxx
*功能描述		:	将GPIO相应管脚配置为AOD(复用开漏)输出模式-寄存器版本
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_gpio_set_AOD50_reg_pin_0(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL|=0x0000000F;
}
void api_gpio_set_AOD50_reg_pin_1(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL|=0x000000F0;
}
void api_gpio_set_AOD50_reg_pin_2(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL|=0x00000F00;
}
void api_gpio_set_AOD50_reg_pin_3(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL|=0x0000F000;
}
void api_gpio_set_AOD50_reg_pin_4(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL|=0x000F0000;
}
void api_gpio_set_AOD50_reg_pin_5(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL|=0x00F00000;
}
void api_gpio_set_AOD50_reg_pin_6(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL|=0x0F000000;
}
void api_gpio_set_AOD50_reg_pin_7(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL|=0xF0000000;
}
void api_gpio_set_AOD50_reg_pin_8(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH|=0x0000000F;
}
void api_gpio_set_AOD50_reg_pin_9(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH|=0x000000F0;
}
void api_gpio_set_AOD50_reg_pin_10(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH|=0x00000F00;
}
void api_gpio_set_AOD50_reg_pin_11(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH|=0x0000F000;
}
void api_gpio_set_AOD50_reg_pin_12(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH|=0x000F0000;
}
void api_gpio_set_AOD50_reg_pin_13(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH|=0x00F00000;
}
void api_gpio_set_AOD50_reg_pin_14(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH|=0x0F000000;
}
void api_gpio_set_AOD50_reg_pin_15(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH|=0xF0000000;
}
void api_gpio_set_AOD50_reg_pin_All(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL|=0xFFFFFFFF;
	GPIOx->CRH|=0xFFFFFFFF;
}
void api_gpio_set_AOD50_reg_pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n)
{
	unsigned char loop=0;
	unsigned long cmp=GPIO_Pin_0;
	unsigned long DataOr 	=0x00000000;
	
	if(GPIO_Pin_n&0x00FF)
	{
		cmp=GPIO_Pin_0;
		DataOr 	=0x00000000;
		for(loop=0;loop<8;loop++)
		{		
			if(cmp&GPIO_Pin_n)
			{
				DataOr	|=	0x0000000F<<(4*(loop));
			}
			else
			{
				DataOr	|=	0x00000000<<(4*(loop));
			}
			cmp<<=1;
		}
		GPIOx->CRL|=DataOr;
	}
	
	if(GPIO_Pin_n&0xFF00)
	{
		cmp=GPIO_Pin_8;
		DataOr 	=0x00000000;
		for(loop=0;loop<8;loop++)
		{		
			if(cmp&GPIO_Pin_n)
			{
				DataOr	|=	0x0000000F<<(4*(loop));
			}
			else
			{
				DataOr	|=	0x00000000<<(4*(loop));
			}
			cmp<<=1;
		}
		GPIOx->CRH|=DataOr;
	}
}
//------------------------------------------------------------------------------



//******************************************************************************
//													输入模式
//******************************************************************************

//=================================================将GPIO相应管脚配置为INA(模拟)输入模式-寄存器版本----V20190805
/*******************************************************************************
*函数名			:	API_GPIO_SET_AOD50REGxx
*功能描述		:	将GPIO相应管脚配置为AOD(复用开漏)输出模式-寄存器版本
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
/*******************************************************************************
*函数名			:	API_GPIO_SET_APP50REGxx
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_gpio_set_INA_reg_pin_0(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFFFF0;
}
void api_gpio_set_INA_reg_pin_1(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFFF0F;
}
void api_gpio_set_INA_reg_pin_2(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFF0FF;
}
void api_gpio_set_INA_reg_pin_3(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFF0FFF;
}
void api_gpio_set_INA_reg_pin_4(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFF0FFFF;
}
void api_gpio_set_INA_reg_pin_5(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFF0FFFFF;
}
void api_gpio_set_INA_reg_pin_6(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xF0FFFFFF;
}
void api_gpio_set_INA_reg_pin_7(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0x0FFFFFFF;
}
void api_gpio_set_INA_reg_pin_8(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFFFF0;
}
void api_gpio_set_INA_reg_pin_9(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFFF0F;
}
void api_gpio_set_INA_reg_pin_10(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFF0FF;
}
void api_gpio_set_INA_reg_pin_11(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFF0FFF;
}
void api_gpio_set_INA_reg_pin_12(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFF0FFFF;
}
void api_gpio_set_INA_reg_pin_13(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFF0FFFFF;
}
void api_gpio_set_INA_reg_pin_14(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xF0FFFFFF;
}
void api_gpio_set_INA_reg_pin_15(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0x0FFFFFFF;
}
void api_gpio_set_INA_reg_pin_All(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0x00000000;
	GPIOx->CRH&=0x00000000;
}
void api_gpio_set_INA_reg_pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n)
{
	unsigned char loop=0;
	unsigned long cmp=GPIO_Pin_0;
	unsigned long DataAnd	=0x00000000;
	
	if(GPIO_Pin_n&0x00FF)
	{
		cmp=GPIO_Pin_0;
		DataAnd	=0x00000000;
		for(loop=0;loop<8;loop++)
		{		
			if(cmp&GPIO_Pin_n)
			{
				DataAnd	|=	0x00000000<<(4*(loop));
			}
			else
			{
				DataAnd	|=	0x0000000F<<(4*(loop));
			}
			cmp<<=1;
		}
		GPIOx->CRL&=DataAnd;
	}
	
	if(GPIO_Pin_n&0xFF00)
	{
		cmp=GPIO_Pin_8;
		DataAnd	=0x00000000;
		for(loop=0;loop<8;loop++)
		{		
			if(cmp&GPIO_Pin_n)
			{
				DataAnd	|=	0x00000000<<(4*(loop));
			}
			else
			{
				DataAnd	|=	0x0000000F<<(4*(loop));
			}
			cmp<<=1;
		}
		GPIOx->CRH&=DataAnd;
	}
}
//------------------------------------------------------------------------------

//=================================================将GPIO相应管脚配置为INF(浮空)输入模式-寄存器版本----V20190805
/*******************************************************************************
*函数名			:	API_GPIO_SET_AOD50REGxx
*功能描述		:	将GPIO相应管脚配置为AOD(复用开漏)输出模式-寄存器版本
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
/*******************************************************************************
*函数名			:	API_GPIO_SET_APP50REGxx
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_gpio_set_INF_reg_pin_0(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFFFF0;
	GPIOx->CRL|=0x00000004;
}
void api_gpio_set_INF_reg_pin_1(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFFF0F;
	GPIOx->CRL|=0x00000040;
}
void api_gpio_set_INF_reg_pin_2(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFF0FF;
	GPIOx->CRL|=0x00000400;
}
void api_gpio_set_INF_reg_pin_3(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFF0FFF;
	GPIOx->CRL|=0x00004000;
}
void api_gpio_set_INF_reg_pin_4(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFF0FFFF;
	GPIOx->CRL|=0x00040000;
}
void api_gpio_set_INF_reg_pin_5(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFF0FFFFF;
	GPIOx->CRL|=0x00400000;
}
void api_gpio_set_INF_reg_pin_6(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xF0FFFFFF;
	GPIOx->CRL|=0x04000000;
}
void api_gpio_set_INF_reg_pin_7(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0x0FFFFFFF;
	GPIOx->CRL|=0x40000000;
}
void api_gpio_set_INF_reg_pin_8(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFFFF0;
	GPIOx->CRH|=0x00000004;
}
void api_gpio_set_INF_reg_pin_9(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFFF0F;
	GPIOx->CRH|=0x00000040;
}
void api_gpio_set_INF_reg_pin_10(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFF0FF;
	GPIOx->CRH|=0x00000400;
}
void api_gpio_set_INF_reg_pin_11(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFF0FFF;
	GPIOx->CRH|=0x00004000;
}
void api_gpio_set_INF_reg_pin_12(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFF0FFFF;
	GPIOx->CRH|=0x00040000;
}
void api_gpio_set_INF_reg_pin_13(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFF0FFFFF;
	GPIOx->CRH|=0x00400000;
}
void api_gpio_set_INF_reg_pin_14(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xF0FFFFFF;
	GPIOx->CRH|=0x04000000;
}
void api_gpio_set_INF_reg_pin_15(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0x0FFFFFFF;
	GPIOx->CRH|=0x40000000;
}
void api_gpio_set_INF_reg_pin_All(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0x00000000;
	GPIOx->CRL|=0x44444444;
	GPIOx->CRH&=0x00000000;
	GPIOx->CRH|=0x44444444;
}
void api_gpio_set_INF_reg_pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n)
{
	unsigned char loop=0;
	unsigned long cmp=GPIO_Pin_0;
	unsigned long DataAnd	=0x00000000;
	unsigned long DataOr 	=0x00000000;
	if(GPIO_Pin_n&0x00FF)
	{
		cmp=GPIO_Pin_0;
		DataAnd	=0x00000000;
		DataOr 	=0x00000000;
		for(loop=0;loop<8;loop++)
		{		
			if(cmp&GPIO_Pin_n)
			{
				DataAnd	|=	0x00000000<<(4*(loop));
				DataOr	|=	0x00000004<<(4*(loop));
			}
			else
			{
				DataAnd	|=	0x0000000F<<(4*(loop));
				DataOr	|=	0x00000000<<(4*(loop));
			}
			cmp<<=1;
		}
		GPIOx->CRL&=DataAnd;
		GPIOx->CRL|=DataOr;
	}
	
	if(GPIO_Pin_n&0xFF00)
	{
		cmp=GPIO_Pin_8;
		DataAnd	=0x00000000;
		DataOr 	=0x00000000;
		for(loop=0;loop<8;loop++)
		{		
			if(cmp&GPIO_Pin_n)
			{
				DataAnd	|=	0x00000000<<(4*(loop));
				DataOr	|=	0x00000004<<(4*(loop));
			}
			else
			{
				DataAnd	|=	0x0000000F<<(4*(loop));
				DataOr	|=	0x00000000<<(4*(loop));
			}
			cmp<<=1;
		}
		GPIOx->CRH&=DataAnd;
		GPIOx->CRH|=DataOr;
	}
}
//------------------------------------------------------------------------------

//=================================================将GPIO相应管脚配置为IPU(上拉)输入模式-寄存器版本----V20190805
/*******************************************************************************
*函数名			:	API_GPIO_SET_AOD50REGxx
*功能描述		:	将GPIO相应管脚配置为AOD(复用开漏)输出模式-寄存器版本
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
/*******************************************************************************
*函数名			:	API_GPIO_SET_APP50REGxx
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_gpio_set_IPU_reg_pin_0(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFFFF0;
	GPIOx->CRL|=0x00000008;
	GPIOx->BSRR=GPIO_Pin_0;
}
void api_gpio_set_IPU_reg_pin_1(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFFF0F;
	GPIOx->CRL|=0x00000080;
	GPIOx->BSRR=GPIO_Pin_1;
}
void api_gpio_set_IPU_reg_pin_2(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFF0FF;
	GPIOx->CRL|=0x00000800;
	GPIOx->BSRR=GPIO_Pin_2;
}
void api_gpio_set_IPU_reg_pin_3(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFF0FFF;
	GPIOx->CRL|=0x00008000;
	GPIOx->BSRR=GPIO_Pin_3;
}
void api_gpio_set_IPU_reg_pin_4(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFF0FFFF;
	GPIOx->CRL|=0x00080000;
	GPIOx->BSRR=GPIO_Pin_4;
}
void api_gpio_set_IPU_reg_pin_5(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFF0FFFFF;
	GPIOx->CRL|=0x00800000;
	GPIOx->BSRR=GPIO_Pin_5;
}
void api_gpio_set_IPU_reg_pin_6(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xF0FFFFFF;
	GPIOx->CRL|=0x08000000;
	GPIOx->BSRR=GPIO_Pin_6;
}
void api_gpio_set_IPU_reg_pin_7(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0x0FFFFFFF;
	GPIOx->CRL|=0x80000000;
	GPIOx->BSRR=GPIO_Pin_7;
}
void api_gpio_set_IPU_reg_pin_8(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFFFF0;
	GPIOx->CRH|=0x00000008;
	GPIOx->BSRR=GPIO_Pin_8;
}
void api_gpio_set_IPU_reg_pin_9(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFFF0F;
	GPIOx->CRH|=0x00000080;
	GPIOx->BSRR=GPIO_Pin_9;
}
void api_gpio_set_IPU_reg_pin_10(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFF0FF;
	GPIOx->CRH|=0x00000800;
	GPIOx->BSRR=GPIO_Pin_10;
}
void api_gpio_set_IPU_reg_pin_11(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFF0FFF;
	GPIOx->CRH|=0x00008000;
	GPIOx->BSRR=GPIO_Pin_11;
}
void api_gpio_set_IPU_reg_pin_12(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFF0FFFF;
	GPIOx->CRH|=0x00080000;
	GPIOx->BSRR=GPIO_Pin_12;
}
void api_gpio_set_IPU_reg_pin_13(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFF0FFFFF;
	GPIOx->CRH|=0x00800000;
	GPIOx->BSRR=GPIO_Pin_13;
}
void api_gpio_set_IPU_reg_pin_14(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xF0FFFFFF;
	GPIOx->CRH|=0x08000000;
	GPIOx->BSRR=GPIO_Pin_14;
}
void api_gpio_set_IPU_reg_pin_15(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0x0FFFFFFF;
	GPIOx->CRH|=0x80000000;
	GPIOx->BSRR=GPIO_Pin_15;
}
void api_gpio_set_IPU_reg_pin_All(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0x00000000;
	GPIOx->CRL|=0x88888888;
	GPIOx->CRH&=0x00000000;
	GPIOx->CRH|=0x88888888;
	GPIOx->BSRR=GPIO_Pin_All;
}
void api_gpio_set_IPU_reg_pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n)
{
	unsigned char loop=0;
	unsigned long cmp=GPIO_Pin_0;
	unsigned long DataAnd		=0x00000000;
	unsigned long DataOr 		=0x00000000;
	unsigned short DataSet	=0x0000;
	if(GPIO_Pin_n&0x00FF)
	{
		cmp=GPIO_Pin_0;
		DataAnd	=0x00000000;
		DataOr 	=0x00000000;
		for(loop=0;loop<8;loop++)
		{	
			if(cmp&GPIO_Pin_n)
			{
				DataAnd	|=	0x00000000<<(4*(loop));
				DataOr	|=	0x00000008<<(4*(loop));
				DataSet|=0x0001<<loop;
			}
			else
			{
				DataAnd	|=	0x0000000F<<(4*(loop));
				DataOr	|=	0x00000000<<(4*(loop));
			}
			cmp<<=1;
		}
		GPIOx->CRL&=DataAnd;
		GPIOx->CRL|=DataOr;
	}	
	if(GPIO_Pin_n&0xFF00)
	{
		cmp=GPIO_Pin_8;
		DataAnd	=0x00000000;
		DataOr 	=0x00000000;
		for(loop=0;loop<8;loop++)
		{	
			if(cmp&GPIO_Pin_n)
			{
				DataAnd	|=	0x00000000<<(4*(loop));
				DataOr	|=	0x00000008<<(4*(loop));
				DataSet|=0x0100<<loop;
			}
			else
			{
				DataAnd	|=	0x0000000F<<(4*(loop));
				DataOr	|=	0x00000000<<(4*(loop));
			}
			cmp<<=1;
		}
		GPIOx->CRH&=DataAnd;
		GPIOx->CRH|=DataOr;		
	}
	GPIOx->BSRR=DataSet;
}
//------------------------------------------------------------------------------

//=================================================将GPIO相应管脚配置为IPD(下拉)输入模式-寄存器版本----V20190805
/*******************************************************************************
*函数名			:	API_GPIO_SET_AOD50REGxx
*功能描述		:	将GPIO相应管脚配置为AOD(复用开漏)输出模式-寄存器版本
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
/*******************************************************************************
*函数名			:	API_GPIO_SET_APP50REGxx
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_gpio_set_IPD_reg_pin_0(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFFFF0;
	GPIOx->CRL|=0x00000008;
	GPIOx->BRR=GPIO_Pin_0;
}
void api_gpio_set_IPD_reg_pin_1(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFFF0F;
	GPIOx->CRL|=0x00000080;
	GPIOx->BRR=GPIO_Pin_1;
}
void api_gpio_set_IPD_reg_pin_2(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFFF0FF;
	GPIOx->CRL|=0x00000800;
	GPIOx->BRR=GPIO_Pin_2;
}
void api_gpio_set_IPD_reg_pin_3(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFFF0FFF;
	GPIOx->CRL|=0x00008000;
	GPIOx->BRR=GPIO_Pin_3;
}
void api_gpio_set_IPD_reg_pin_4(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFFF0FFFF;
	GPIOx->CRL|=0x00080000;
	GPIOx->BRR=GPIO_Pin_4;
}
void api_gpio_set_IPD_reg_pin_5(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xFF0FFFFF;
	GPIOx->CRL|=0x00800000;
	GPIOx->BRR=GPIO_Pin_5;
}
void api_gpio_set_IPD_reg_pin_6(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0xF0FFFFFF;
	GPIOx->CRL|=0x08000000;
	GPIOx->BRR=GPIO_Pin_6;
}
void api_gpio_set_IPD_reg_pin_7(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0x0FFFFFFF;
	GPIOx->CRL|=0x80000000;
	GPIOx->BRR=GPIO_Pin_7;
}
void api_gpio_set_IPD_reg_pin_8(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFFFF0;
	GPIOx->CRH|=0x00000008;
	GPIOx->BRR=GPIO_Pin_8;
}
void api_gpio_set_IPD_reg_pin_9(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFFF0F;
	GPIOx->CRH|=0x00000080;
	GPIOx->BRR=GPIO_Pin_9;
}
void api_gpio_set_IPD_reg_pin_10(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFFF0FF;
	GPIOx->CRH|=0x00000800;
	GPIOx->BRR=GPIO_Pin_10;
}
void api_gpio_set_IPD_reg_pin_11(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFFF0FFF;
	GPIOx->CRH|=0x00008000;
	GPIOx->BRR=GPIO_Pin_11;
}
void api_gpio_set_IPD_reg_pin_12(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFFF0FFFF;
	GPIOx->CRH|=0x00080000;
	GPIOx->BRR=GPIO_Pin_12;
}
void api_gpio_set_IPD_reg_pin_13(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xFF0FFFFF;
	GPIOx->CRH|=0x00800000;
	GPIOx->BRR=GPIO_Pin_13;
}
void api_gpio_set_IPD_reg_pin_14(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0xF0FFFFFF;
	GPIOx->CRH|=0x08000000;
	GPIOx->BRR=GPIO_Pin_14;
}
void api_gpio_set_IPD_reg_pin_15(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRH&=0x0FFFFFFF;
	GPIOx->CRH|=0x80000000;
	GPIOx->BRR=GPIO_Pin_15;
}
void api_gpio_set_IPD_reg_pin_All(GPIO_TypeDef* GPIOx)
{
	GPIOx->CRL&=0x00000000;
	GPIOx->CRL|=0x88888888;
	GPIOx->CRH&=0x00000000;
	GPIOx->CRH|=0x88888888;
	GPIOx->BRR=GPIO_Pin_All;
}
void api_gpio_set_IPD_reg_pin_xx(GPIO_TypeDef* GPIOx,u16 GPIO_Pin_n)
{
	unsigned char loop=0;
	unsigned long cmp=GPIO_Pin_0;
	unsigned long DataAnd		=0x00000000;
	unsigned long DataOr 		=0x00000000;
	unsigned short DataSet	=0x0000;
	if(GPIO_Pin_n&0x00FF)
	{
		cmp=GPIO_Pin_0;
		DataAnd	=0x00000000;
		DataOr 	=0x00000000;
		for(loop=0;loop<8;loop++)
		{	
			if(cmp&GPIO_Pin_n)
			{
				DataAnd	|=	0x00000000<<(4*(loop));
				DataOr	|=	0x00000008<<(4*(loop));
				DataSet|=0x0001<<loop;
			}
			else
			{
				DataAnd	|=	0x0000000F<<(4*(loop));
				DataOr	|=	0x00000000<<(4*(loop));
			}
			cmp<<=1;
		}
		GPIOx->CRL&=DataAnd;
		GPIOx->CRL|=DataOr;
	}	
	if(GPIO_Pin_n&0xFF00)
	{
		cmp=GPIO_Pin_8;
		DataAnd	=0x00000000;
		DataOr 	=0x00000000;
		for(loop=0;loop<8;loop++)
		{	
			if(cmp&GPIO_Pin_n)
			{
				DataAnd	|=	0x00000000<<(4*(loop));
				DataOr	|=	0x00000008<<(4*(loop));
				DataSet|=0x0100<<loop;
			}
			else
			{
				DataAnd	|=	0x0000000F<<(4*(loop));
				DataOr	|=	0x00000000<<(4*(loop));
			}
			cmp<<=1;
		}
		GPIOx->CRH&=DataAnd;
		GPIOx->CRH|=DataOr;		
	}
	GPIOx->BRR=DataSet;
}
//------------------------------------------------------------------------------


