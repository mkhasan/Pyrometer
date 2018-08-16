

#include "crc.h"

#include <assert.h> 

int deg4 = 0;
int deg8 = 0;

uint32_t polynom4 = 0;
uint32_t polynom8 = 0;

static int GetDegree(uint32_t polynom);

int crcInit(uint32_t _polynom4, uint32_t _polynom8) {
  
  polynom4 = _polynom4;
  polynom8 = _polynom8;
  
  deg4 =  GetDegree(polynom4);
  deg8 = GetDegree(polynom8);
  
  if(deg4 != 4 || deg8 != 8)
    return -1;
  
  return 0;
  
}

uint8_t crcCalc(uint32_t const message, int const msgLen, uint32_t polynom)
{
    uint32_t  remainder;

    int deg = polynom == polynom4 ? deg4 : (polynom == polynom8 ? deg8 : GetDegree(polynom));
    
    assert(deg > 0 && deg <= msgLen);
    polynom <<= (msgLen-deg-1);	//0b110100;

    const uint32_t mask = 1 << (msgLen-1);
    //const int maxIt = msgLen;

    //printf("poly is %x \n", polynom);
    /*
     * Initially, the dividend is the remainder.
     *
     *
     */
    remainder = message;

    /*
     * For each bit position in the message....
     */
    for (uint8_t bit = msgLen; bit > 0; --bit)
    {
        /*
         * If the uppermost bit is a 1...
         */
        if (remainder & mask)
        {
            /*
             * XOR the previous remainder with the divisor.
             */
            remainder ^= polynom;
        }


        /*
         * Shift the next bit of the message into the remainder.
         */
        remainder = (remainder << 1);
        //printf("it %d: remainder is %x \n", msgLen-bit, remainder);
    }

    /*
     * Return only the relevant bits of the remainder as CRC.
     */

    const uint32_t remMask = ((1<<deg) - 1);
    return (uint8_t) ((remainder >> (msgLen-deg)) & remMask);

}   /* crcNaive() */

int GetDegree(uint32_t polynom) {
	int degree = -1;
	while(polynom) {
		polynom >>= 1;
		degree ++;
	}

	return degree;
}