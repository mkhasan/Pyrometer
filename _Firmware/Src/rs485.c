


#include "rs485.h"

#include "config.h"
#include "crc.h"


tUart data;
uint32_t addrData;
uint8_t addrCrc;                // from input data
uint8_t dataCrc;                // from output data

extern int readCallback;
extern int readyToReceive;
extern uint32_t 	pyroMeterData;		  			//Contains data value	

extern int dataReady;
extern int sendData;
extern UART_HandleTypeDef huart2;
extern int sentBufferEmpty;
extern int sendEvent;
extern int writeErrorCount;
extern int lastByte;
extern uint32_t myAddr;

uint32_t temp;

uint8_t cmaLen = RX1BUFFERSIZE;

uint8_t  ucpRx1Buffer  [RX1BUFFERSIZE]; // ??? ??? ??


extern int ucReceive_Event;

extern int reqReceived;




extern int ret;

static int idLen=0;

GPIO_PinState state;

int addr[ADDR_PIN_COUNT];



uint8_t cmdStr[266];

uint8_t cmdStrPrefix[256] = "GET VALUE ";

static uint8_t cmdLen = RX1BUFFERSIZE;

extern uint8_t myChar;

static uint8_t data_element;

static int CheckID(uint8_t *id);

static int IsValid(uint8_t value);

static int CheckCmd();

void UART_RxAgain(UART_HandleTypeDef *huart);

int requestOkay = 0;

void rs485_Init() {
  int i;
  data.Flag = 0;
  data.rx_point_head = data.rx_point_tail = data.tx_point_head = data.tx_point_tail = 0;

  for(i=0; i< RBUF_SIZE; i++) {
    data.RxBuf[i] = 0;
    data.TxBuf[i] = 0;
  }
  
  for(i=0; i<RX1BUFFERSIZE; i++)
    ucpRx1Buffer[i] = 'z';
    
  data.cmdIndex = 0;
  strcpy(cmdStr, cmdStrPrefix);
  strcat(cmdStr, "NNN");
  cmdLen = strlen(cmdStr);
  
  
  idLen = strlen(cmdStr) - strlen(cmdStrPrefix);
}


extern int transferErrorCount;
extern int writeErrorCount;
extern int readErrorCount;
extern uint32_t errCode[256];

void  RequestRecv() {
  
  
    HAL_GPIO_WritePin(RS485_ENABLE_PORT, RS485_RE_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS485_ENABLE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);

    
    
    uint8_t localData;
    int recvRet;
    if((recvRet=HAL_UART_Receive(&huart2,&localData,1, 0xFF)) == HAL_OK) {
      data.RxBuf[data.rx_point_head++] = localData;
      data.rx_point_head %= RBUF_SIZE;
    }
    else {
      errCode[transferErrorCount%256] = recvRet;
      transferErrorCount ++;
      readErrorCount ++;
    }
      
       
    
    
  
  
  
    
}

void ProcessInput() {

  
  if(data.rx_point_tail == data.rx_point_head)
    return;
    
  data_element = data.RxBuf[data.rx_point_tail++];
  data.rx_point_tail %= RBUF_SIZE;
  
  if(data.Flag == 0 && data_element != STX)
    return;
    
  if(data.Flag==1 && ((data_element == ETX) || (IsValid(data_element) && data.cmdIndex >= cmdLen))) {  
       
    if(data.cmdIndex == cmdLen)
      requestOkay = 1;
    
    data.Flag = 0;
    
  }
  
  else if(data.Flag==1) { 
    
    if(IsValid(data_element) && data.cmdIndex < cmdLen) {
      data.Flag = 1;
      data.CurrentCmd[data.cmdIndex++] = data_element;
    }
    else
      data.Flag = 0;
  }
  
  else if(data_element == STX) {  //1st Start flag
    data.Flag = 1;
    data.cmdIndex = 0;
  }

  if(requestOkay == 1) {
    
    if(CheckCmd() == 1) {
      dataReady = 1;
      requestOkay = 0;
    }
    
  }  
 

}    
    


int CheckID(uint8_t * idStr) {          // 1 means valid
  
  
  
uint32_t id = 0;
  
  /*
  uint32_t factor = 1;
  for(int i=0; i<idLen-1; i++)
    factor *= 10;
    
  
  for(i=0; i<idLen; i++) {
    if(!(idStr[i]>='0' && idStr[i] <= '9'))
      return 0;
    id += (idStr[i] - '0')*factor;
    factor /= 10;     
           
  }

  */
  
  
  if(idStr[2] != '0')
    return 0;
  addrData = idStr[0];
  
  addrCrc = crcCalc(addrData, 8, POLYNOM4);
  if (addrCrc != idStr[1])
    return 0;
  
  return (idStr[0] == myAddr);
  
  

  
  
  
   
}


 

int IsValid(uint8_t value) {           // 1 means valid 
  
  
  if(data.cmdIndex == 10)
    data.cmdIndex = 10;
  if(cmdStr[data.cmdIndex] == 'N')  
    return  1;
    
  
  
  if(!(value == ' ' || (value >= 'A' && value <= 'Z') || (value >= '0' && value <= '9' ))) 
    return 0;
  
  if(value != cmdStr[data.cmdIndex])
    return 0;
  
  return 1;
    
    

}

int CheckCmd() {
  
  int len = strlen(cmdStrPrefix);
  
  int i=0;
  for(i=0; i<len; i++)
    if(data.CurrentCmd[i] != cmdStrPrefix[i])
      return 0;
  
  return CheckID(&data.CurrentCmd[len]);
  
}

void SendData() {
  
  
  uint8_t *p;
  int i;
 

  for(i=0; i<10; i++)
    data.TxBuf[i] = '0';
  
  data.TxBuf[0] = DATA_KEY;
  data.TxBuf[1] = ' ';
  
  
  {    
    
    memcpy(&data.TxBuf[2], sendData, sizeof(sendData));
    
    uint32_t pDataH = (pyroMeterData & 0xFFFF) >> 8;
    uint32_t pDataL = (pyroMeterData & 0xFF);
    
    data.TxBuf[2] = sendData & 0xFF;
    data.TxBuf[3] = (sendData & 0xFF00) >> 8;
    data.TxBuf[4] = (sendData & 0xFF0000) >> 16;
    
    temp = 0;
    p = (uint8_t *) &temp;
    uint8_t tempAddr = (myAddr & 0xFF);
    temp |= (tempAddr << 24);
    temp |= ((pDataH & 0xFFF) << 12);
    temp |= (pDataL & 0xFFF);
    
    int index = 2+3;            // 3 for sendData
    //dataCrc = 
    for (i=0; i<4; i++)
      data.TxBuf[index+i] = p[i];
      
    
    dataCrc = crcCalc(temp, 32, POLYNOM8);
    data.TxBuf[9] = dataCrc;

    
    
    
    
    
    HAL_GPIO_WritePin(RS485_ENABLE_PORT, RS485_DE_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RS485_ENABLE_PORT, RS485_RE_PIN, GPIO_PIN_SET);
    
    
   
    sentBufferEmpty = 0;
    sendData ++;
    lastByte = -1*WRITE_DATA_LEN;

    
    
    if(HAL_UART_Transmit_IT(&huart2, data.TxBuf, 10)!= HAL_OK) {
      writeErrorCount ++;
      sentBufferEmpty = 1;
      lastByte = 0;
      sendData --;
      
      
    }
 
    
    
    

  
  }

}



    

uint32_t GetAddr() {
  
  //state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
  
  uint32_t ret=0;
    
  addr[0] = (HAL_GPIO_ReadPin(ADDR_LSB3_PORT, ADDR_PIN_0) == GPIO_PIN_SET);
  addr[1] = (HAL_GPIO_ReadPin(ADDR_LSB3_PORT, ADDR_PIN_1) == GPIO_PIN_SET);
  addr[2] = (HAL_GPIO_ReadPin(ADDR_LSB3_PORT, ADDR_PIN_2) == GPIO_PIN_SET);
  addr[3] = (HAL_GPIO_ReadPin(ADDR_MSB_PORT, ADDR_PIN_3) == GPIO_PIN_SET);

  
  int i;
  
  for(i=0, ret=0; i<ADDR_PIN_COUNT; i++)
    ret |= (addr[i] << i);
   
  return ret;
}
