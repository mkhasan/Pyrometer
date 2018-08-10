//**********************************************************************************************
//								BASIC INFORMATION ABOUT THIS PROGRAM
//***********************************************************************************************
//File name:	main.c
//Data:			22/05/2007
//Version:		01.00
//Company:		Melexis-Bulgaria(www.melexis.com)
//Details: 		Calculation of PEC when SMBus comunication with MLX90614 is done
//Author:		Dimo Petkov, dpv@melexis.com	
//*****************************************************************************************************************
//										HEADER FILES
//*****************************************************************************************************************
#include "main.h"


//*****************************************************************************************************************
//										MAIN PROGRAM
//*****************************************************************************************************************
void main(void)
{
	unsigned char 	arr[6];			// Memory for the sequence of bytes on which PEC is calculated
	unsigned char 	SlaveAddress; 	// Contains devive address
	unsigned char	command;	  	// Contains command byte
	unsigned char 	DataL;		  	// Contains data value low byte	
	unsigned char 	DataH;		  	// Contains data value high byte	
	unsigned char	PecReg;			// Defines memory for calculated byte
	
	
	arr[5]=SlaveAddress;			//
	arr[4]=command;					//
	arr[3]=SlaveAddress;			// Load the byte sequence  in the array arr, the first sent 
	arr[2]=DataL;					// byte is in arr[5] and so on. In arr[0] is written the zero  
	arr[1]=DataH;					// byte
	arr[0]=0;						//
						
							
	
	PecReg=PEC_calculation(arr);	//Calculate CRC

}/* End of main() */


	    
	    
	    
	
