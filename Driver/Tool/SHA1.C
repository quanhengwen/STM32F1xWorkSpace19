/************************************ CRC ************************************
* 文件名 	: SHA1算法
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
/* Includes ------------------------------------------------------------------*/

#include 	"SHA1.H"
#include	"stdlib.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// Global Variables
static unsigned char sha1_Message[20];		//需要验证的消息These are the 20 bytes for the random message	sent to bq26100
static unsigned char sha1_Digest[20];			//bq62100反应的信息摘要These are the 20 bytes for the SHA1 response of the bq26100
static unsigned char sha1_Key[16];				//These are the 16 bytes for the secret key, should match with contents of valid bq26100 


// The functions used to implement the SHA1 as computed by the microcontroller were
// developed following the same guidance as described in the "How to Implement SHA-1/HMAC
// Authentication for bq26100" document. It can be found in www.ti.com by searching SLUA389.


// Global Variables for SHA1
static unsigned long sha1_Ws[80];			//Global Work schedule variable
static unsigned long sha1_A;
static unsigned long sha1_B;
static unsigned long sha1_C;
static unsigned long sha1_D;
static unsigned long sha1_E;
static unsigned long sha1_H[5];					//160-Bit SHA-1 Digest信息摘要
static unsigned long sha1_Random[5];		//The 16 bytes of random message for the bq26100 are contained here
					//for microcontroller to use in SHA1/HMAC
static unsigned long sha1_Digest_32[5];	//The result of the SHA1/HMAC obtained by the microcontroller is contained here
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/




/**********************************************************************/
/* 	unsigned long Rotl(unsigned long x, int n)					      */
/*																      */
/*	Description : 		This procedure is a rotate left n spaces of	  */
/*				  		32-bit word x.								  */
/* 	Arguments : 		x - word to be rotated						  */
/*						n - amount of spaces to rotated to the left   */
/*	Global Variables:	None   										  */
/*  Returns: 			Result of 32-bit word rotated n times         */
/**********************************************************************/
unsigned long sha1_Rotl(unsigned long x, int n)
{
	return (x<<n) |  (x>>(32-n));
}	



/**********************************************************************/
/* 	unsigned long W(int t)		                    			      */
/*																      */
/*	Description : 		This procedure determines the work schedule   */
/*				  		for W(16) through W(79)						  */
/* 	Arguments : 		t - index of work schedule 16 through 79	  */
/*	Global Variables:	Ws[]   										  */
/*  Returns: 			Work schedule value with index t              */
/**********************************************************************/
unsigned long sha1_W(int t)
{
	return sha1_Rotl(sha1_Ws[t-3] ^ sha1_Ws[t-8] ^ sha1_Ws[t-14] ^ sha1_Ws[t-16], 1);
}	

/**********************************************************************/
/* 	unsigned long K(int t)	 									      */
/*																      */
/*	Description : 		This procedure selects one of the K values 	  */
/*				  		depending on the index t.					  */
/* 	Arguments : 		t - index 									  */
/*	Global Variables:	None   										  */
/*  Returns: 			One of the 4 K values				          */
/**********************************************************************/
unsigned long sha1_K(int t)
{
	if (t<=19)
		return 0x5a827999;
	else if (t>=20 && t<=39)
		return 0x6ed9eba1;
	else if (t>=40 && t<=59)
		return 0x8f1bbcdc;
	else if (t>=60 && t<=79)
		return 0xca62c1d6;
	else
		return 0;		//Invalid value, not expected
}
	
/*******************************************************************************/
/* 	unsigned long f(unsigned long x, unsigned long y, unsigned long z, int t)  */
/*																               */
/*	Description : 		This procedure selects the ft(b,c,d) function based    */
/*				  		on SLUA389 and FIPS 180-2 document. 			           */
/* 	Arguments : 		x - b as seen in document						       */
/*						y - c as seen in document                              */
/*                      z - d as seed in document                              */
/*                      t - index											   */
/*	Global Variables:	None   										           */
/*  Returns: 			Result of ft function                                  */
/*******************************************************************************/
unsigned long sha1_f(unsigned long x, unsigned long y, unsigned long z, int t)
{
	if (t<=19)
		return (x & y) ^ ((~x) & z);
	else if (t>=20 && t<=39)
		return x ^ y ^ z;
	else if (t>=40 && t<=59)
		return (x & y) ^ (x & z) ^ (y & z);
	else if (t>=60 && t<=79)
		return x ^ y ^ z;
	else
		return 0;       //Invalid value, not expected
}



/**********************************************************************/
/* 	void Auth(void)					      							  */
/*																      */
/*	Description : 		This procedure computes the SHA1/HMAC as 	  */
/*				  		required by the bq26100						  */
/* 	Arguments : 		i - times that SHA1 is executing			  */
/*						t - index 0 through 79						  */
/*     					temp - Used to update working variables		  */
/*	Global Variables:	Random[], Message[], Key[], Ws[], H[],		  */
/*						A, B, C, D, E								  */
/*  Returns: 			Result of 32-bit word rotated n times         */
/**********************************************************************/ 
void sha1_Auth(void)
{
	int i;			//Used for doing two times the SHA1 as required by the bq26100
	int t;			//Used for the indexes 0 through 79
	unsigned long temp;		//Used as the temp variable during the loop in which the working
					//variables A, B, C, D and E are updated
	//-------------------------需要验证的消息为20字节/160位/占用5个32位缓存
	// The 20 bytes of random message that are given to the bq26100 are arranged in 32-bit words
	// so that the microcontroller can compute the SHA1/HMAC
	sha1_Random[0] = sha1_Message[0x10] + sha1_Message[0x11]*0x0100 + sha1_Message[0x12]*0x010000 + sha1_Message[0x13]*0x01000000;
	sha1_Random[1] = sha1_Message[0x0C] + sha1_Message[0x0D]*0x0100 + sha1_Message[0x0E]*0x010000 + sha1_Message[0x0F]*0x01000000;
	sha1_Random[2] = sha1_Message[0x08] + sha1_Message[0x09]*0x0100 + sha1_Message[0x0A]*0x010000 + sha1_Message[0x0B]*0x01000000;
	sha1_Random[3] = sha1_Message[0x04] + sha1_Message[0x05]*0x0100 + sha1_Message[0x06]*0x010000 + sha1_Message[0x07]*0x01000000;
	sha1_Random[4] = sha1_Message[0x00] + sha1_Message[0x01]*0x0100 + sha1_Message[0x02]*0x010000 + sha1_Message[0x03]*0x01000000;
	//-------------------------
	// The SHA1 is computed two times so that it complies with the bq26100 specification
	for (i=0; i<=1; i++)
	{
		//Work Schedule
		// The first four Working schedule variables Ws[0-3], are based on the key that is 
		// implied that the bq26100 contains.
		sha1_Ws[0] = sha1_Key[0x0C] + sha1_Key[0x0D]*0x0100 + sha1_Key[0x0E]*0x010000 + sha1_Key[0x0F]*0x01000000;
		sha1_Ws[1] = sha1_Key[0x08] + sha1_Key[0x09]*0x0100 + sha1_Key[0x0A]*0x010000 + sha1_Key[0x0B]*0x01000000;
		sha1_Ws[2] = sha1_Key[0x04] + sha1_Key[0x05]*0x0100 + sha1_Key[0x06]*0x010000 + sha1_Key[0x07]*0x01000000;
		sha1_Ws[3] = sha1_Key[0x00] + sha1_Key[0x01]*0x0100 + sha1_Key[0x02]*0x010000 + sha1_Key[0x03]*0x01000000;
		// On the first run of the SHA1 the random message is used 		
		if (i==0)
		{
			sha1_Ws[4] = sha1_Random[0];
			sha1_Ws[5] = sha1_Random[1];
			sha1_Ws[6] = sha1_Random[2];
			sha1_Ws[7] = sha1_Random[3];
			sha1_Ws[8] = sha1_Random[4];
		}
		// On the second run of the SHA1, H(Kd || M) is used		
		else
		{
			sha1_Ws[4] = sha1_H[0];
			sha1_Ws[5] = sha1_H[1];
			sha1_Ws[6] = sha1_H[2];
			sha1_Ws[7] = sha1_H[3];
			sha1_Ws[8] = sha1_H[4];
		}
		// The Work schedule variables Ws[9-15] remain the same regardless of which run of the SHA1.
		// These values are as required by bq26100.
		sha1_Ws[9] = 0x80000000;
		sha1_Ws[10] = 0x00000000;
		sha1_Ws[11] = 0x00000000;
		sha1_Ws[12] = 0x00000000;
		sha1_Ws[13] = 0x00000000;
		sha1_Ws[14] = 0x00000000;
		sha1_Ws[15] = 0x00000120;

		// The Work schedule variables Ws[16-79] are determined by the W(t) function	
		for (t = 16; t <= 79; t++)
			sha1_Ws[t]=sha1_W(t);
		// Working Variables, always start the same	regardless of which SHA1 run
		sha1_A = 0x67452301;
		sha1_B = 0xefcdab89;
		sha1_C = 0x98badcfe;
		sha1_D = 0x10325476;
		sha1_E = 0xc3d2e1f0;
		// Hash Values, always start the same regardless of what SHA1 run
		sha1_H[0] = sha1_A;
		sha1_H[1] = sha1_B;
		sha1_H[2] = sha1_C;
		sha1_H[3] = sha1_D;
		sha1_H[4] = sha1_E;
		// Loop to change working variables A, B, C, D and E
		// This is defined by FIPS 180-2 document
		for (t = 0; t <= 79; t++)
		{
			temp = sha1_Rotl(sha1_A,5) + sha1_f(sha1_B,sha1_C,sha1_D,t) + sha1_E + sha1_K(t) + sha1_Ws[t];
			sha1_E = sha1_D;
			sha1_D = sha1_C;
			sha1_C = sha1_Rotl(sha1_B,30);
			sha1_B = sha1_A;
			sha1_A = temp;
		}
		// 160-Bit SHA-1 Digest
		sha1_H[0] = sha1_A + sha1_H[0];
		sha1_H[1] = sha1_B + sha1_H[1];
		sha1_H[2] = sha1_C + sha1_H[2];
		sha1_H[3] = sha1_D + sha1_H[3];
		sha1_H[4] = sha1_E + sha1_H[4];
	}//End of for loop
}	//End sha1_Auth() function

/**********************************************************************/
/* 	int main(void)					      							  */
/*																      */
/*	Description : 		This is the main function. It calls the SDQ	  */
/*				  		communication and the SHA1 function. Results  */
/*						are displayed through LEDs.					  */
/* 	Arguments : 		*Value - generic variable for read functions  */
/*						i - used for repeat loops				      */
/*	Global Variables:	Message[], Key[], Digest[], Digest_32[]		  */
/*  Returns: 			None								          */
/**********************************************************************/
int example (void)
{
	unsigned char *Value;
	int i;
  	
//	// Select the key to be used here. In this example 0x3601FCFB12B87356C1548630FD3EA0D2 is used.  
//	Key[0] = 0xD2;
//	Key[1] = 0xA0;
//	Key[2] = 0x3E;
//	Key[3] = 0xFD;
//	Key[4] = 0x30;
//	Key[5] = 0x86;
//	Key[6] = 0x54;
//	Key[7] = 0xC1;
//	Key[8] = 0x56;
//	Key[9] = 0x73;
//	Key[10] = 0xB8;
//	Key[11] = 0x12;
//	Key[12] = 0xFB;
//	Key[13] = 0xFC;
//	Key[14] = 0x01;
//	Key[15] = 0x36;
//	
//	
//	/*	for (i=0; i<=15; i++){
//				Key[i] = 0x00;
//			}
//	*/ 
//	// Create a random message generator and insert here. In this example the message is fixed.  	
//	srand(0);
//	for (i=0; i<=19; i++)
//	{
//		Message[i] = rand()%256;
//	}

//	sha1_Auth();	 		//Perform SHA1 authentication through host
//  
//	// Send 160-bit Message to bq26100
//	SendReset();	   			
//	*Value = TestPresence(); 		//Determine if there is a SDQ compatible device connected
//	if (*Value==0x00)	//No device connected or functional
//	{	
//		GPIO_WriteBit(GPIO8, GPIO_Pin_0, Bit_RESET);
//		while (1)	//Blink red LED at a 0.5 sec on and 0.5 sec off rate
//		{				 		
//			GPIO_WriteBit(GPIO8, GPIO_Pin_6, Bit_RESET);  	//Red LED off
//			wait_us(1e6/2);  					  			//wait approximately 0.5 seconds
//			GPIO_WriteBit(GPIO8, GPIO_Pin_6, Bit_SET); 	  	//Red LED on
//			wait_us(1e6/2);									//wait approximately 0.5 seconds
//		}
//	}
//	// Skip ROM Command
//	WriteOneByte(0xCC);
//	// Write Message Command
//	WriteOneByte(0x22);	//Write Message Command
//	// Write Address
//	WriteOneByte(0x00);	//Address Low Byte
//	WriteOneByte(0x00);	//Address High Byte
//	// Write first byte of message, Message[0]
//	WriteOneByte(Message[0x00]);	
//	*Value = ReadOneByte();	//Read CRC of Message Command, Address and Data
//	//CRC are not being calculated by the microcontroller, the results given by
//	//bq26100 are being ignored
//	// Read Data to Verify Write
//	*Value = ReadOneByte();
//	// Write the remaining bytes of the message
//	for (i=1; i<=19; i++)
//	{
//		WriteOneByte(Message[i]);
//		*Value = ReadOneByte();		//Read CRC
//		*Value = ReadOneByte();		//Read Data
//	}


//	// Write Control to set AUTH bit to initiate Authentication by bq26100
//	SendReset();
//	*Value = TestPresence();
//	// Skip ROM Command
//	WriteOneByte(0xCC);
//	// Write Control Command
//	WriteOneByte(0x77);	//Write Control Command
//	// Write Address
//	WriteOneByte(0x00);	//Address Low Byte
//	WriteOneByte(0x00);	//Address High Byte
//	// Write Auth Bit of Control Register
//	WriteOneByte(0x01);	
//	*Value = ReadOneByte();	//Read CRC of Write Control Command, Address and Data
//	// Read Data to Verify Write
//	*Value = ReadOneByte();

//	
//	//Read Digest
//	SendReset();
//	*Value = TestPresence();
//	// Skip ROM Command
//	WriteOneByte(0xCC);
//	// Read Digest Command
//	WriteOneByte(0xDD);	//Read Digest Command
//	// Write Address
//	WriteOneByte(0x00);	//Address Low Byte
//	WriteOneByte(0x00);	//Address High Byte
//	*Value = ReadOneByte();	//Read CRC of Read Digest Command, Address
//	// Read Digest
//	for (i=0; i<=19; i++)
//	{
//		Digest[i] = ReadOneByte();
//	}
//	// Read CRC of Digest
//	*Value = ReadOneByte();

//	// The 20 bytes of the digest returned by the bq26100 is arranged in 32-bit words so that it
//	// can be compared with the results computed by the microcontroller
//	sha1_Digest_32[4] = Digest[0x00] + Digest[0x01]*0x0100 + Digest[0x02]*0x010000 + Digest[0x03]*0x01000000;
//	sha1_Digest_32[3] = Digest[0x04] + Digest[0x05]*0x0100 + Digest[0x06]*0x010000 + Digest[0x07]*0x01000000;
//	sha1_Digest_32[2] = Digest[0x08] + Digest[0x09]*0x0100 + Digest[0x0A]*0x010000 + Digest[0x0B]*0x01000000;
//	sha1_Digest_32[1] = Digest[0x0C] + Digest[0x0D]*0x0100 + Digest[0x0E]*0x010000 + Digest[0x0F]*0x01000000;
//	sha1_Digest_32[0] = Digest[0x10] + Digest[0x11]*0x0100 + Digest[0x12]*0x010000 + Digest[0x13]*0x01000000;

//	// The results given by microcontroller and bq26100 are compared
//	if (sha1_Digest_32[0] == H[0])
//	{
//		if (sha1_Digest_32[1] == H[1])
//		{
//			if (sha1_Digest_32[2] == H[2])
//			{
//				if (sha1_Digest_32[3] == H[3])
//				{
//					if (sha1_Digest_32[4] == H[4])
//					{ 	//If all values are same then Authentication is successful
//						GPIO_WriteBit(GPIO8, GPIO_Pin_0, Bit_SET);		//LED Green on
//						GPIO_WriteBit(GPIO8, GPIO_Pin_6, Bit_RESET); 	//LED Red off
//					}
//					else
//					{						//If any of the values do not match then authentication fails
//						GPIO_WriteBit(GPIO8, GPIO_Pin_6, Bit_SET); 		//LED Red on
//						GPIO_WriteBit(GPIO8, GPIO_Pin_0, Bit_RESET); 	//LED Green off
//					}
//				}
//				else
//				{
//					GPIO_WriteBit(GPIO8, GPIO_Pin_6, Bit_SET); 			//LED Red on
//					GPIO_WriteBit(GPIO8, GPIO_Pin_0, Bit_RESET);		//LED Green off
//				}
//			}			
//			else
//			{
//				GPIO_WriteBit(GPIO8, GPIO_Pin_6, Bit_SET); 				//LED Red on
//				GPIO_WriteBit(GPIO8, GPIO_Pin_0, Bit_RESET);			//LED Green off	
//			}
//		}
//		else
//		{
//			GPIO_WriteBit(GPIO8, GPIO_Pin_6, Bit_SET); 					//LED Red on
//			GPIO_WriteBit(GPIO8, GPIO_Pin_0, Bit_RESET);				//LED Green off
//		}
//	}
//	else
//	{
//		GPIO_WriteBit(GPIO8, GPIO_Pin_6, Bit_SET); 						//LED Red on
//		GPIO_WriteBit(GPIO8, GPIO_Pin_0, Bit_RESET);					//LED Green off
//	}
  
}
