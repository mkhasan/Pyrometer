//**********************************************************************************************
//								BASIC INFORMATION ABOUT THIS PROGRAM
//***********************************************************************************************
//File name:	main.c
//Data:			19/09/2007
//Version:		01.00
//Company:		Melexis-Bulgaria(www.melexis.com)
//Details: 		SMBus comunication with MLX90614 using PIC18F4320 
//				Language C: MCC18 compiler
//				Fosc=11.0592MHz, Tcy=362ns
//Author:		Dimo Petkov( dpv@melexis.com)	
//*****************************************************************************************************************
//										HEADER FILES
//*****************************************************************************************************************
#include <p18F4320.h>
#include "main.h"

//*****************************************************************************************************************
//									CONFIGURATION BITS
//*****************************************************************************************************************
#pragma	config	OSC=HS,FSCM=OFF,IESO=ON
#pragma	config	PWRT=ON,BOR=OFF,BORV=42
#pragma	config	WDT=OFF
#pragma	config	MCLRE=ON,PBAD=DIG,CCP2MX=C1
#pragma	config	STVR=ON,LVP=OFF,DEBUG=OFF
#pragma	config	CP0=OFF,CP1=OFF,CP2=OFF,CP3=OFF
#pragma	config	CPB=ON,CPD=OFF
#pragma	config	WRT0=OFF,WRT1=OFF,WRT2=OFF,WRT3=OFF
#pragma	config	WRTC=OFF,WRTB=ON,WRTD=OFF
#pragma	config	EBTR0=OFF,EBTR1=OFF,EBTR2=OFF,EBTR3=OFF
#pragma	config	EBTRB=ON
//*****************************************************************************************************************
//							   			VECTORS REMAPING											                         */
//*****************************************************************************************************************
#ifdef BOOTLOADER

	#pragma code _HIGH_INTERRUPT_VECTOR = 0x000208
	void _high_ISR (void)
	{
    	_asm GOTO high_isr _endasm ;
	}
	#pragma code _LOW_INTERRUPT_VECTOR = 0x000218
	void _low_ISR (void)
	{
    	_asm GOTO low_isr _endasm ;;
	}		
	#pragma code
	
#endif
/***********************************************************************************************
*							   				GLOBAL VARIABLES								   *	
***********************************************************************************************/

/***********************************************************************************************
*								 				INTERRUPTS							   		   *	
***********************************************************************************************/

#pragma interrupt high_isr
void high_isr(void)
{
// User code...	
}


#pragma interruptlow low_isr
void low_isr(void)
{
// User code...  
}

#pragma code

//*****************************************************************************************************************
//										MAIN PROGRAM
//*****************************************************************************************************************	
void main(void)
{
	unsigned char 	SlaveAddress; 			//Contains device address
	unsigned char	command;	  			//Contains the access command
	unsigned int 	data;		  			//Contains data value	
	float			t;						//Contains calculated temperature in degrees Celsius	
		
	MCUinit();						  		//MCU initialization
	SlaveAddress=SA<<1;						//Set device address
	command=RAM_Access|RAM_Tobj1; 	        //Form RAM access command + RAM address 			
	
//	SendRequest();							//Switch to SMBus mode - this is need if module is in PWM mode only
//	DummyCommand(SlaveAddress);				//This is need if Request Command is sent even when the module is in SMBus mode
	delay(DEL200ms);						//Wait after POR,Tvalid=0.15s 
	
	while(1)
	{
		data=MemRead(SlaveAddress,command); //Read memory
		t=CalcTemp(data);					//Calculate temperature
		delay(DEL1SEC);						//Wait 1 second 
	}

}//End of main()

//*****************************************************************************************************************
//										FUNCTION'S DEFINITIONS
//*****************************************************************************************************************
void MCUinit(void)
{
//IO setting-up	
	ADCON1=0b00001111;		 //All chanels are digital I/O
	
//SMBus setting-up	
	mSDA_HIGH();		     //The bus is in idle state
	mSCL_HIGH();			 //SDA and SCL are in high level from pull up resitors
}//End of init()
	
//-----------------------------------------------------------------------------------------------------------------

unsigned int MemRead(unsigned char SlaveAddress,unsigned char command)
{
	unsigned int  data;				// Data storage (DataH:DataL)
	unsigned char Pec;				// PEC byte storage
	unsigned char DataL;			// Low data byte storage
	unsigned char DataH;			// High data byte storage
	unsigned char arr[6];			// Buffer for the sent bytes
	unsigned char PecReg;			// Calculated PEC byte storage
	unsigned char ErrorCounter;		// Defines the number of the attempts for communication with MLX90614
	
	ErrorCounter=0x00;				// Initialising of ErrorCounter
	
	do{
	repeat:
		STOP_bit();					//If slave send NACK stop comunication	
		--ErrorCounter;				//Pre-decrement ErrorCounter
		if(!ErrorCounter){			//ErrorCounter=0?
			break;					//Yes,go out from do-while{}
		}
		START_bit();				//Start condition
		
		if(TX_byte(SlaveAddress)){	//Send SlaveAddress
			goto	repeat;			//Repeat comunication again
		}
			
		if(TX_byte(command)){		//Send command	
			goto	repeat;			//Repeat comunication again
		}
		START_bit();				//Repeated Start condition
		
		if(TX_byte(SlaveAddress)){ //Send SlaveAddress
			goto	repeat;         //Repeat comunication again
		}
		
		DataL=RX_byte(ACK);			//Read low data,master must send ACK
		DataH=RX_byte(ACK); 		//Read high data,master must send ACK
		Pec=RX_byte(NACK);			//Read PEC byte, master must send NACK
		STOP_bit();					//Stop condition
		
		
		arr[5]=SlaveAddress;		//
		arr[4]=command;				//
		arr[3]=SlaveAddress;		//Load array arr 
		arr[2]=DataL;				//
		arr[1]=DataH;				//
		arr[0]=0;					//
		PecReg=PEC_calculation(arr);//Calculate CRC
	    	    
	}while(PecReg != Pec);		//If received and calculated CRC are equal go out from do-while{}
		
	*((unsigned char *)(&data))=DataL;	   // 
	*((unsigned char *)(&data)+1)=DataH ;  //data=DataH:DataL
	
	return data;							
}
//---------------------------------------------------------------------------------------------
void SendRequest(void)
{
	mSCL_LOW();			//SCL 1 ____________|<-----80ms------->|______________
	delay(DEL80ms);		// 	  0	            |__________________|
	mSCL_HIGH();
}
//---------------------------------------------------------------------------------------------
void DummyCommand(unsigned char byte)
{
	START_bit();		//Start condition
	TX_byte(byte);		//Send Slave Address or whatever,no need ACK checking
	STOP_bit();			//Stop condition
}
//---------------------------------------------------------------------------------------------
float CalcTemp(unsigned int value)
{
	float temp;
	
	temp=(value*0.02)-273.15;
	
	return temp;
}
