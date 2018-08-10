/* Uncomment this if push pull clock need to be used */
//#define PUSH_PULL

/*  If bootloader is not desired do next:
	1. Comment #define BOOTLOADER
	2. Remove from the project BLOAD_PM.asm file
	3. In c018i.c file replace "#pragma code _entry_scn=0x000200"  
						with   "#pragma code _entry_scn=0x000000" 
	4. Replace rm18F4320i.lkr file with 18F4320i.lkr
*/
#define BOOTLOADER
						
// Set bit in a variable
#define bit_set(var,bitno) ((var) |= 1 << (bitno))
// Clear bit in a variable
#define bit_clr(var,bitno) ((var) &= ~(1 << (bitno)))
// Test bit in a variable
#define testbit(data,bitno) ((data>>bitno)&0x01)

//SMBus control signals
#define _SCL_IO TRISCbits.TRISC3
#define _SDA_IO TRISCbits.TRISC4
#define _SCL    PORTCbits.RC3
#define _SDA    PORTCbits.RC4

#define mSDA_HIGH()	_SDA_IO=1;
#define mSDA_LOW()  _SDA=0;_SDA_IO=0;

#ifndef PUSH_PULL	
	#define mSCL_HIGH()	_SCL_IO=1;
#else
	#define mSCL_HIGH()	_SCL=1;_SCL_IO=0;
#endif

#define mSCL_LOW()  _SCL=0;_SCL_IO=0;

#define ACK	 0
#define	NACK 1

//MLX90614 constants
#define SA				0x00	// Slave address
#define DEFAULT_SA		0x5A	// Default Slave address
#define RAM_Access		0x00	// RAM access command
#define EEPROM_Access	0x20	// EEPROM access command
#define RAM_Tobj1		0x07	// To1 address in the eeprom

//Delay constants
#define DEL1SEC	 100000
#define DEL80ms	 7400
#define DEL200ms 18500
#define TBUF	 2
//*PROTOTYPES*****************************************************************************************************
void high_isr(void);
void low_isr(void);
void MCUinit(void);
unsigned int MemRead(unsigned char SlaveAddress,unsigned char command);
void SendRequest(void);
void DummyCommand(unsigned char byte);
float CalcTemp(unsigned int value);
//*EXTERNAL FUNCTIONS**********************************************************************************************
extern 	void START_bit(void);
extern	void STOP_bit(void);
extern	unsigned char TX_byte(unsigned char Tx_buffer);
extern	unsigned char RX_byte(unsigned char ack_nack);
extern	void delay( unsigned long i);
extern  unsigned char PEC_calculation(unsigned char pec[]);

