//*********************************************************************************************
//						CALCULATION PEC PACKET	
//*********************************************************************************************
//Name:			PEC_calculation
//Function:		Calculates the PEC of received bytes
//Parameters:	unsigned char pec[]			
//Return:		pec[0]-this byte contains calculated crc value
//Comments: 	Refer to "System Managment BUS(SMBus) specification Version 2.0" and
//				AN "SMBus comunication with MLX90614"
//*********************************************************************************************
unsigned char PEC_calculation(unsigned char pec[])
{
	unsigned char 	crc[6];
	unsigned char	BitPosition=47;
	unsigned char	shift;
	unsigned char	i;
	unsigned char	j;
	unsigned char	temp;

	do{
		/*Load pattern value 0x000000000107*/
		crc[5]=0;				
		crc[4]=0;
		crc[3]=0;
		crc[2]=0;
		crc[1]=0x01;
		crc[0]=0x07;
		
		/*Set maximum bit position at 47 ( six bytes byte5...byte0,MSbit=47)*/
		BitPosition=47;	
		
		/*Set shift position at 0*/		
		shift=0;
				
		/*Find first "1" in the transmited message beginning from the MSByte byte5*/
		i=5;					
		j=0;
		while((pec[i]&(0x80>>j))==0 && i>0){
			BitPosition--;
			if(j<7){
				j++;
			}
			else{
				j=0x00;
				i--;
			}
		}/*End of while */
		
		/*Get shift value for pattern value*/
		shift=BitPosition-8;	
		
		/*Shift pattern value */
		while(shift){
			for(i=5; i<0xFF; i--){
				if((crc[i-1]&0x80) && (i>0)){
					temp=1;
				}
				else{
					temp=0;
				}
				crc[i]<<=1;
				crc[i]+=temp;
			}/*End of for*/
			shift--;
		}/*End of while*/
		
		
		/*Exclusive OR between pec and crc*/		
		for(i=0; i<=5; i++){
			pec[i] ^=crc[i];
		}/*End of for*/
		
	}while(BitPosition>8);/*End of do-while*/
	
	return pec[0];
}/*End of PEC_calculation*/
