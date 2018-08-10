//*HEADER FILES********************************************************************************
#include<p18F4320.h>
#include"main.h"

//*High and Low level of clock 
#define HIGHLEV	3
#define LOWLEV	1		


//*PROTOTYPES**********************************************************************************
void START_bit(void);
void STOP_bit(void);
unsigned char TX_byte(unsigned char Tx_buffer);
unsigned char RX_byte(unsigned char ack_nack);
void send_bit(unsigned char bit_out);
unsigned char Receive_bit(void);
//*EXTERNAL FUNCTION***************************************************************************
extern	void delay( unsigned long i);
