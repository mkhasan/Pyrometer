//**********************************************************************************************
//								HEADER FILES
//**********************************************************************************************
#include"SMBus.h"
#include <delays.h>

//**********************************************************************************************
//							START CONDITION ON SMBus	
//**********************************************************************************************
//Name:			START_bit
//Function:		Generate START condition on SMBus
//Parameters:	No
//Return:		No
//Comments: 	Refer to "System Managment BUS(SMBus) specification Version 2.0"
//				or AN"SMBus communication with MLX90614" on the website www.melexis.com
//**********************************************************************************************
void START_bit(void)
{
	mSDA_HIGH();			// Set SDA line
	Delay10TCYx( TBUF );	// Wait a few microseconds
	mSCL_HIGH();			// Set SCL line
	Delay10TCYx( TBUF );	// Generate bus free time between Stop
							// and Start condition (Tbuf=4.7us min)
	mSDA_LOW();				// Clear SDA line
	Delay10TCYx( TBUF );	// Hold time after (Repeated) Start
							// Condition. After this period, the first clock is generated.
							//(Thd:sta=4.0us min)
	mSCL_LOW();				// Clear SCL line 
	Delay10TCYx( TBUF );	// Wait a few microseconds		
}
//*********************************************************************************************
//								STOP CONDITION ON SMBus	
//*********************************************************************************************
//Name:			STOPbit
//Function:		Generate STOP condition on SMBus
//Parameters:	No
//Return:		No
//Comments: 	Refer to "System Managment BUS(SMBus) specification Version 2.0"
//		    	or AN"SMBus communication with MLX90614" on the website www.melexis.com
//*********************************************************************************************	
void STOP_bit(void)
{
	mSCL_LOW();				// Clear SCL line
	Delay10TCYx( TBUF );	// Wait a few microseconds
	mSDA_LOW();				// Clear SDA line
	Delay10TCYx( TBUF );	// Wait a few microseconds
	mSCL_HIGH();			// Set SCL line
	Delay10TCYx( TBUF );	// Stop condition setup time(Tsu:sto=4.0us min)
	mSDA_HIGH();			// Set SDA line
}	
//*********************************************************************************************
//								TRANSMIT DATA ON SMBus	
//*********************************************************************************************
//Name:			TX_byte
//Function:		Send a byte on SMBus
//Parameters:	TX_buffer ( the byte which will be send on the SMBus )
//Return:		Ack_bit	  ( acknowledgment bit )
//Comments:  	Sends MSbit first
//*********************************************************************************************	
unsigned char TX_byte(unsigned char Tx_buffer)
{
	unsigned char	Bit_counter;
	unsigned char	Ack_bit;
	unsigned char	bit_out;

	for(Bit_counter=8; Bit_counter; Bit_counter--)
	{
		if(Tx_buffer&0x80) bit_out=1; // If the current bit of Tx_buffer is 1 set bit_out
		else			   bit_out=0; // else clear bit_out
		
		send_bit(bit_out);			  // Send the current bit on SDA
		Tx_buffer<<=1;				  // Get next bit for checking
	}
	
	Ack_bit=Receive_bit();			  // Get acknowledgment bit
			
	return	Ack_bit;
}// End of TX_bite()

//---------------------------------------------------------------------------------------------
void send_bit(unsigned char bit_out)
{
	if(bit_out==0) {mSDA_LOW();}	  
	else	 	   {mSDA_HIGH();}
	Nop();							//	
	Nop();							// Tsu:dat = 250ns minimum
	Nop();							//
	mSCL_HIGH();					// Set SCL line
	Delay10TCYx( HIGHLEV );			// High Level of Clock Pulse
	mSCL_LOW();						// Clear SCL line
	Delay10TCYx( LOWLEV );			// Low Level of Clock Pulse
//	mSDA_HIGH();				    // Master release SDA line ,
	return;
}//End of send_bit()
//---------------------------------------------------------------------------------------------

unsigned char Receive_bit(void)
{
	unsigned char Ack_bit;
	
	_SDA_IO=1;						// SDA-input
	mSCL_HIGH();					// Set SCL line
	Delay10TCYx( HIGHLEV );			// High Level of Clock Pulse
	if(_SDA)	Ack_bit=1;			// \ Read acknowledgment bit, save it in Ack_bit
	else		Ack_bit=0;			// /
	mSCL_LOW();						// Clear SCL line
	Delay10TCYx( LOWLEV );			// Low Level of Clock Pulse

	return	Ack_bit;
}//End of Receive_bit
//*********************************************************************************************
//									RECEIVE DATA ON SMBus	
//*********************************************************************************************
//Name:			RX_byte
//Function:		Receive a byte on SMBus
//Parameters:	ack_nack (ackowlegment bit) 
//Return:		RX_buffer(Received byte)
//Comments:  	MSbit is received first
//*********************************************************************************************
unsigned char RX_byte(unsigned char ack_nack)
{
	unsigned char 	RX_buffer;
	unsigned char	Bit_Counter;
	
	for(Bit_Counter=8; Bit_Counter; Bit_Counter--)
	{
		if(Receive_bit())						// Get a bit from the SDA line
		{
			RX_buffer <<= 1;					// If the bit is HIGH save 1  in RX_buffer
			RX_buffer |=0b00000001;
		}
		else
		{
			RX_buffer <<= 1;					// If the bit is LOW save 0 in RX_buffer 
			RX_buffer &=0b11111110;	
		}
	}
	
	send_bit(ack_nack);							// Sends acknowledgment bit
	
	return RX_buffer;
}
//----------------------------------------------------------------------------------------------
