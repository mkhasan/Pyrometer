
#include "stm32f1xx_hal.h"
#include "config.h"
#include "crc.h"

I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart2;              // PA2 and PA3


void SystemClock_Config(void);
void _Error_Handler(char * file, int line);
void MX_GPIO_Init(void);
void MX_USART2_UART_Init(void);


static void MX_I2C2_Init(void);

static void USART_ClearITPendingBit(UART_HandleTypeDef* USARTx, uint16_t USART_IT);
static int ret;



static int sec = 0;

static int tick = 0;

TIM_HandleTypeDef htim3;

static int timer2Tick = 0;
static int shift = 0;

uint32_t g_ADCValueDMA;

uint32_t g_ADCBuffer[ADC_BUFFER_LENGTH];
DMA_HandleTypeDef g_DmaHandle;
ADC_HandleTypeDef g_AdcHandle;

void MX_TIM3_Init(void);
void Display(int digitNo, char val);

void ConfigureADC();

int delay = 1000;

int turn=0;

uint32_t 	pyroMeterData;		  			//Contains data value	
float			tempCelcius;						//Contains calculated temperature in degrees Celsius	

uint8_t count = 0;

uint8_t success = 0;
uint8_t W_Success = 0;





#define EPROM_ADDR 0x02

#define SA				0x5A	// Slave address
#define DEFAULT_SA		0x5A	// Default Slave address
#define RAM_Access		0x00	// RAM access command
#define EEPROM_Access	0x20	// EEPROM access command
#define RAM_Tobj1		0x07	// To1 address in the eeprom




const uint8_t cmd = (0x20 | EPROM_ADDR);

//uint8_t command=RAM_Access|RAM_Tobj1;

const uint8_t slaveAddr = 0x5A << 1;// 0;//(0x5A << 1);

uint8_t buffer[10];
uint8_t recv[10];

//////////////////////////////////////
#define mSCL_HIGH()	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
#define mSCL_LOW()	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

#define mSDA_HIGH()	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
#define mSDA_LOW()	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

#define TBUF 2
#define HIGHLEV       3
#define LOWLEV        1


#define ACK     0
#define        NACK 1

#define DIGITS_AFTER_DEC 1

void send_bit(uint8_t bit_out);
uint8_t Receive_bit(void);

float CalcTemp(uint32_t value);

uint8_t GetDigits(float t, char *val);


static char val[4] = {'0', '0', '0', '0'};

static char val1[4];


uint32_t myAddr;


int sentBufferEmpty = 1;

int transferErrorCount = 0;
int writeErrorCount = 0;
int readErrorCount = 0;

int transferCallback = 0;
int readCallback = 0;

uint32_t errCode[256];

int lastByte = 0;

int sendData = 0;

int dataReady = 0;

uint8_t PEC_calculation(uint8_t pec[]);

void TestIt();

void SendRequest(void);
uint32_t MemRead(uint8_t SlaveAddress,uint8_t command);



void MCUinit(void)
{
//IO setting-up	
	//ADCON1=0b00001111;		 //All chanels are digital I/O
	
//SMBus setting-up	
	mSDA_HIGH();		     //The bus is in idle state
	mSCL_HIGH();			 //SDA and SCL are in high level from pull up resitors
}//End of init()


void Delay10TCYx(uint8_t n) {
  for(int i=0; i<10*n; i++) {
    asm("NOP");
  }
}

void Nop() {
  asm("NOP");
}



////////////////////////////////////////////////


uint32_t value = 0;

int sent = 0;


int main() {
  
  HAL_Init();
  
  

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

 
  if(crcInit(POLYNOM4, POLYNOM8) != 0)
    Error_Handler();
  
  MX_GPIO_Init();
  
  MX_USART2_UART_Init();
  
  
  MX_TIM3_Init();
  
  
  ///////////////////////////////

  uint8_t 	SlaveAddress; 			//Contains device address
  uint8_t	command;	  			//Contains the access command


          
  MCUinit();						  		//MCU initialization
  SlaveAddress=SA<<1;						//Set device address
  command=RAM_Access|RAM_Tobj1; 	        //Form RAM access command + RAM address 			

  //SendRequest();							//Switch to SMBus mode - this is need if module is in PWM mode only
  //DummyCommand(SlaveAddress);				//This is need if Request Command is sent even when the module is in SMBus mode
  //delay(DEL200ms);						//Wait after POR,Tvalid=0.15s 
  HAL_Delay(200);


  
  pyroMeterData = 0;
  
  HAL_TIM_Base_Start_IT(&htim3);                
  
  uint32_t temp = 0;
  
  
  /////////  UART2  /////////////
  
  rs485_Init();
  
  
  __HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
  __HAL_UART_DISABLE_IT(&huart2, UART_IT_TC);

 
  myAddr = GetAddr();
  
  ///////////////////////////////
  
    
  
  
  
  
  
  
  while(1) {
    
    
    
    if(timer2Tick >= 1000 ) {
      
      temp = value;
      val[0] = '0' + temp / 1000;  
      temp = temp % 1000;
      
      val[1] = '0' + temp/100;
      temp = temp % 100;
      
      val[2] = '0' + temp/10;
      temp = temp % 10;
      
      val[3] = '0' + temp;
      
      timer2Tick = 0;
      value = (value+1) % 10000;
      
      if(1) {
        pyroMeterData=MemRead(SlaveAddress,command); //Read memory
            
        tempCelcius=CalcTemp(pyroMeterData);					//Calculate temperature
      }
      
      GetDigits(tempCelcius, val);
            
            
      
            

    }
    
    
    /*
    if(shift >= 5) {            // should not be used here because serial comm make a delay of 100 ms 
                                // this delay destroy the display frequency...so I moved it to HAL_TIM_PeriodElapsedCallback
      Display(turn+1, val[turn]);
      turn = (turn+1)%4;
      shift = 0;
      
      
    }

    */
    
    
    
    if(dataReady == 0) {
      ProcessInput();                   //   parsing incoming bytes in the serial interface
    }
    else
      __NOP();

    if(sentBufferEmpty == 1 && dataReady == 1) {                // dataReady indicates that data has been prepared because of the most recent request to my address
        dataReady = 0;
        
        __HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
        HAL_Delay(100);
        SendData();                     // Sending current data if the request is to my addess
    //          HAL_Delay(10);
        sent ++;
        
    } else if (sentBufferEmpty && dataReady == 0 ) {
      
      
      __HAL_UART_DISABLE_IT(&huart2, UART_IT_TC);
      USART_ClearITPendingBit(&huart2, UART_IT_TC);
      
      RequestRecv();
    }    



    
    
  }
  return 0;
}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if ((ret=HAL_RCC_OscConfig(&RCC_OscInitStruct)) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if ((ret=HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }


  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}


void MX_GPIO_Init(void)
{


  
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* GPIO Ports Clock Enable */

  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_10  | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  

  
  __HAL_RCC_GPIOA_CLK_ENABLE();



  
  HAL_GPIO_WritePin(DIGIT_CTRL_PORT, DIG1_CTRL_PIN | DIG2_CTRL_PIN | DIG3_CTRL_PIN | DIG4_CTRL_PIN, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = DIG1_CTRL_PIN | DIG2_CTRL_PIN | DIG3_CTRL_PIN | DIG4_CTRL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DIGIT_CTRL_PORT, &GPIO_InitStruct);
  
  
  HAL_GPIO_WritePin(SEG_ABFG_PORT, SEG_B_PIN | SEG_F_PIN | SEG_A_PIN | SEG_G_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = SEG_B_PIN | SEG_F_PIN | SEG_A_PIN | SEG_G_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SEG_ABFG_PORT, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(SEG_CDEP_PORT, SEG_C_PIN | SEG_P_PIN | SEG_D_PIN | SEG_E_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = SEG_C_PIN | SEG_P_PIN | SEG_D_PIN | SEG_E_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SEG_CDEP_PORT, &GPIO_InitStruct);
  
  
  /////////////////  for rs485 address /////////////////
  
  GPIO_InitStruct.Pin = ADDR_PIN_2 | ADDR_PIN_1 | ADDR_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ADDR_LSB3_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = ADDR_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ADDR_MSB_PORT, &GPIO_InitStruct);
  
  
   
  //////////////////////////////////////////////////////
  
  HAL_GPIO_WritePin(RS485_ENABLE_PORT, RS485_RE_PIN | RS485_DE_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = RS485_RE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_ENABLE_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = RS485_DE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_ENABLE_PORT, &GPIO_InitStruct);
  
  


//#endif
  
}

static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 71;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==htim3.Instance) 
  {
    
    timer2Tick ++;
    shift ++;
    
    
    if(shift >= 5) {
    
      Display(turn+1, val[turn]);
      turn = (turn+1)%4;
      shift = 0;
      
      
    }



  }
  
  //test =100;
  
}



void Display(int digitNo, char val) {
  
  HAL_GPIO_WritePin(SEG_ABFG_PORT, SEG_B_PIN | SEG_F_PIN | SEG_A_PIN | SEG_G_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SEG_CDEP_PORT, SEG_C_PIN | SEG_P_PIN | SEG_D_PIN | SEG_E_PIN, GPIO_PIN_RESET);
  
  if(digitNo == 4-DIGITS_AFTER_DEC)
    HAL_GPIO_WritePin(SEG_CDEP_PORT, SEG_P_PIN, GPIO_PIN_SET);
  
  switch(val) {
    case '0':
      HAL_GPIO_WritePin(SEG_ABFG_PORT, SEG_B_PIN | SEG_F_PIN | SEG_A_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(SEG_CDEP_PORT, SEG_C_PIN | SEG_D_PIN | SEG_E_PIN, GPIO_PIN_SET);
      break;
      
    case '1':
      HAL_GPIO_WritePin(SEG_ABFG_PORT, SEG_B_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(SEG_CDEP_PORT, SEG_C_PIN, GPIO_PIN_SET);
      break;
      
    case '2':
      HAL_GPIO_WritePin(SEG_ABFG_PORT, SEG_A_PIN | SEG_B_PIN | SEG_G_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(SEG_CDEP_PORT, SEG_D_PIN | SEG_E_PIN, GPIO_PIN_SET);
      break;
      
    case '3':
      HAL_GPIO_WritePin(SEG_ABFG_PORT, SEG_A_PIN | SEG_B_PIN | SEG_G_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(SEG_CDEP_PORT, SEG_C_PIN | SEG_D_PIN, GPIO_PIN_SET);
      break;
      
    case '4':
      HAL_GPIO_WritePin(SEG_ABFG_PORT, SEG_B_PIN | SEG_F_PIN | SEG_G_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(SEG_CDEP_PORT, SEG_C_PIN, GPIO_PIN_SET);
      break;
      
    case '5':
      HAL_GPIO_WritePin(SEG_ABFG_PORT, SEG_A_PIN | SEG_F_PIN | SEG_G_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(SEG_CDEP_PORT, SEG_C_PIN | SEG_D_PIN, GPIO_PIN_SET);
      break;
      
    case '6':
      HAL_GPIO_WritePin(SEG_ABFG_PORT, SEG_A_PIN | SEG_F_PIN | SEG_G_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(SEG_CDEP_PORT, SEG_C_PIN | SEG_D_PIN | SEG_E_PIN, GPIO_PIN_SET);
      break;
      
    case '7':
      HAL_GPIO_WritePin(SEG_ABFG_PORT, SEG_A_PIN | SEG_F_PIN | SEG_B_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(SEG_CDEP_PORT, SEG_C_PIN, GPIO_PIN_SET);
      break;
    case '8':
      HAL_GPIO_WritePin(SEG_ABFG_PORT, SEG_B_PIN | SEG_F_PIN | SEG_A_PIN | SEG_G_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(SEG_CDEP_PORT, SEG_C_PIN | SEG_D_PIN | SEG_E_PIN, GPIO_PIN_SET);
      break;
    case '9':
      HAL_GPIO_WritePin(SEG_ABFG_PORT, SEG_B_PIN | SEG_F_PIN | SEG_A_PIN | SEG_G_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(SEG_CDEP_PORT, SEG_C_PIN | SEG_D_PIN, GPIO_PIN_SET);
      break;
      
      
  }
      
  HAL_GPIO_WritePin(DIGIT_CTRL_PORT, DIG1_CTRL_PIN | DIG2_CTRL_PIN | DIG3_CTRL_PIN | DIG4_CTRL_PIN, GPIO_PIN_SET);
  switch(digitNo) {
    case 1:
      HAL_GPIO_WritePin(DIGIT_CTRL_PORT, DIG1_CTRL_PIN, GPIO_PIN_RESET);
      break;
    case 2:
      HAL_GPIO_WritePin(DIGIT_CTRL_PORT, DIG2_CTRL_PIN, GPIO_PIN_RESET);
      break;
    case 3:
      HAL_GPIO_WritePin(DIGIT_CTRL_PORT, DIG3_CTRL_PIN, GPIO_PIN_RESET);
      break;
    case 4:
      HAL_GPIO_WritePin(DIGIT_CTRL_PORT, DIG4_CTRL_PIN, GPIO_PIN_RESET);
      break;
   }
 
}


void ConfigureADC()
{
    
 
  ADC_ChannelConfTypeDef adcChannel;

  g_AdcHandle.Instance = ADC1;

  g_AdcHandle.Init.ScanConvMode = ADC_SCAN_ENABLE;
  g_AdcHandle.Init.ContinuousConvMode = ENABLE;
  g_AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  g_AdcHandle.Init.NbrOfDiscConversion = 0;

  g_AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  g_AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  g_AdcHandle.Init.NbrOfConversion = 1;

  if (HAL_ADC_Init(&g_AdcHandle) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  adcChannel.Channel = ADC_CHANNEL_1;
  adcChannel.Rank = 1;
  adcChannel.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;//ADC_SAMPLETIME_55CYCLES_5;
  
  if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  
  
}





void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
  
  uint32_t sum=0;
  int i;
  for(i=0; i<ADC_BUFFER_LENGTH; i++) {
    sum += g_ADCBuffer[i];
  }
  g_ADCValueDMA = sum/ADC_BUFFER_LENGTH;
  
  //__NOP();
}

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000U;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0x0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

//#define SCL_PIN GPIO_PIN_10
//#define SDA_PIN GPIO_PIN_11

void HAL_GPIO_WRITE_ODR(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

void HAL_I2C_ClearBusyFlagErrata_2_14_7(I2C_HandleTypeDef *hi2c) {

    static uint8_t resetTried = 0;
    if (resetTried == 1) {
        return ;
    }
    uint32_t SDA_PIN = GPIO_PIN_11;
    uint32_t SCL_PIN = GPIO_PIN_10;
    GPIO_InitTypeDef GPIO_InitStruct;

    // 1
    __HAL_I2C_DISABLE(hi2c);

    __HAL_RCC_GPIOB_CLK_ENABLE();
    // 2
    GPIO_InitStruct.Pin = SDA_PIN|SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WRITE_ODR(GPIOB, SDA_PIN);
    HAL_GPIO_WRITE_ODR(GPIOB, SCL_PIN);

    // 3
    
    
    GPIO_PinState pinState;
    if (HAL_GPIO_ReadPin(GPIOB, SDA_PIN) == GPIO_PIN_RESET) {
        for(;;){}
    }
    if (HAL_GPIO_ReadPin(GPIOB, SCL_PIN) == GPIO_PIN_RESET) {
        for(;;){}
    }


    // 4
    GPIO_InitStruct.Pin = SDA_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_TogglePin(GPIOB, SDA_PIN);

    // 5
    if (HAL_GPIO_ReadPin(GPIOB, SDA_PIN) == GPIO_PIN_SET) {
        for(;;){}
    }

    // 6
    GPIO_InitStruct.Pin = SCL_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_TogglePin(GPIOB, SCL_PIN);

    // 7
    if (HAL_GPIO_ReadPin(GPIOB, SCL_PIN) == GPIO_PIN_SET) {
        for(;;){}
    }

    // 8
    GPIO_InitStruct.Pin = SDA_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WRITE_ODR(GPIOB, SDA_PIN);

    // 9
    if (HAL_GPIO_ReadPin(GPIOB, SDA_PIN) == GPIO_PIN_RESET) {
        //for(;;){}
    }

    // 10
    GPIO_InitStruct.Pin = SCL_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WRITE_ODR(GPIOB, SCL_PIN);

    // 11
    if (HAL_GPIO_ReadPin(GPIOB, SCL_PIN) == GPIO_PIN_RESET) {
        //for(;;){}
    }

    // 12
    GPIO_InitStruct.Pin = SDA_PIN|SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   // 13
   hi2c->Instance->CR1 |= I2C_CR1_SWRST;

   // 14
   hi2c->Instance->CR1 ^= I2C_CR1_SWRST;

   // 15
   __HAL_I2C_ENABLE(hi2c);

   resetTried = 1;
}

void HAL_GPIO_WRITE_ODR(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));

  GPIOx->ODR |= GPIO_Pin;
}



/** Pinout Configuration
*/


//////////////////////////////////////////////////
uint8_t ReadSDA() {
  
  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET)
    return 1;
  else 
    return 0;
}

void SendRequest(void);

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


uint8_t TX_byte(uint8_t Tx_buffer)
{
	uint8_t	Bit_counter;
	uint8_t	Ack_bit;
	uint8_t	bit_out;

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

void send_bit(uint8_t bit_out)
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


uint8_t Receive_bit(void)
{
	uint8_t Ack_bit;
	
	mSDA_HIGH();
	mSCL_HIGH();					// Set SCL line
	Delay10TCYx( HIGHLEV );			// High Level of Clock Pulse
	if(ReadSDA())	Ack_bit=1;			// \ Read acknowledgment bit, save it in Ack_bit
	else		Ack_bit=0;			// /
	mSCL_LOW();						// Clear SCL line
	Delay10TCYx( LOWLEV );			// Low Level of Clock Pulse

	return	Ack_bit;
}//End of Receive_bit

uint8_t RX_byte(uint8_t ack_nack)
{
	uint8_t 	RX_buffer;
	uint8_t	Bit_Counter;
	
	for(Bit_Counter=8; Bit_Counter; Bit_Counter--)
	{
		if(Receive_bit())						// Get a bit from the SDA line
		{
			RX_buffer <<= 1;					// If the bit is HIGH save 1  in RX_buffer
			RX_buffer |= 0x1;// 0b00000001;
		}
		else
		{
			RX_buffer <<= 1;					// If the bit is LOW save 0 in RX_buffer 
			RX_buffer &= 0xFE;//0b11111110;	
		}
	}
	
	send_bit(ack_nack);							// Sends acknowledgment bit
	
	return RX_buffer;
}


uint32_t MemRead(uint8_t SlaveAddress,uint8_t command)
{
	uint32_t  data=0;				// Data storage (DataH:DataL)
	uint8_t Pec;				// PEC byte storage
	uint8_t DataL;			// Low data byte storage
	uint8_t DataH;			// High data byte storage
	uint8_t arr[6];			// Buffer for the sent bytes
	uint8_t PecReg;			// Calculated PEC byte storage
	uint8_t ErrorCounter;		// Defines the number of the attempts for communication with MLX90614
	uint8_t addrR = SlaveAddress | 1;
	ErrorCounter=0x00;				// Initialising of ErrorCounter
	
        success = 1;
	do{
	repeat:
		STOP_bit();					//If slave send NACK stop comunication	
		--ErrorCounter;				//Pre-decrement ErrorCounter
		if(!ErrorCounter){			//ErrorCounter=0?
                        success = 0;
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
		
		if(TX_byte(addrR)){ //Send SlaveAddress
			goto	repeat;         //Repeat comunication again
		}
		
		DataL=RX_byte(ACK);			//Read low data,master must send ACK
		DataH=RX_byte(ACK); 		//Read high data,master must send ACK
		Pec=RX_byte(NACK);			//Read PEC byte, master must send NACK
		STOP_bit();					//Stop condition
		
		
		arr[5]=SlaveAddress;		//
		arr[4]=command;				//
		arr[3]=addrR;		//Load array arr 
		arr[2]=DataL;				//
		arr[1]=DataH;				//
		arr[0]=0;					//
		PecReg=PEC_calculation(arr);//Calculate CRC
	    	    
	}while(PecReg != Pec);		//If received and calculated CRC are equal go out from do-while{}
		
	*((uint8_t *)(&data))=DataL;	   // 
	*((uint8_t *)(&data)+1)=DataH ;  //data=DataH:DataL
	
	return data;							
}


uint32_t ReadFlags(uint8_t SlaveAddress)
{
	uint32_t  data;				// Data storage (DataH:DataL)
	uint8_t Pec;				// PEC byte storage
	uint8_t DataL;			// Low data byte storage
	uint8_t DataH;			// High data byte storage
	uint8_t arr[6];			// Buffer for the sent bytes
	uint8_t PecReg;			// Calculated PEC byte storage
	uint8_t ErrorCounter;		// Defines the number of the attempts for communication with MLX90614
	
	ErrorCounter=0x00;				// Initialising of ErrorCounter
        
        uint8_t command = 0xF0;
	
        success = 1;
	do{
	repeat:
		STOP_bit();					//If slave send NACK stop comunication	
		--ErrorCounter;				//Pre-decrement ErrorCounter
		if(!ErrorCounter){			//ErrorCounter=0?
                        success = 0;
			break;					//Yes,go out from do-while{}
		}
		START_bit();				//Start condition
		
		if(TX_byte(SlaveAddress)){	//Send SlaveAddress
			goto	repeat;			//Repeat comunication again
		}
			
		if(TX_byte(command)){		//Send command	
			goto	repeat;			//Repeat comunication again
		}
                
                /*
		START_bit();				//Repeated Start condition
		
		if(TX_byte(SlaveAddress)){ //Send SlaveAddress
			goto	repeat;         //Repeat comunication again
		}
		
                */
		DataL=RX_byte(ACK);			//Read low data,master must send ACK
		DataH=RX_byte(ACK); 		//Read high data,master must send ACK
		Pec=RX_byte(NACK);			//Read PEC byte, master must send NACK
		STOP_bit();					//Stop condition
		
		
		arr[5]=0;		//
		arr[4]=SlaveAddress;				//
		arr[3]=command;		//Load array arr 
		arr[2]=DataL;				//
		arr[1]=DataH;				//
		arr[0]=0;					//
		PecReg=PEC_calculation(arr);//Calculate CRC
	    	    
	}while(PecReg != Pec);		//If received and calculated CRC are equal go out from do-while{}
		
	*((uint8_t *)(&data))=DataL;	   // 
	*((uint8_t *)(&data)+1)=DataH ;  //data=DataH:DataL
	
	return data;							
}
uint8_t ErrorCounter=0x10;	// Initialising of ErrorCounter

uint8_t MemWrite(uint8_t SlaveAddress,uint8_t command, uint32_t data)
{
				// Data storage (DataH:DataL)
	uint8_t Pec;				// PEC byte storage
	uint8_t DataL = 0x02;//*((uint8_t *)(&data));			// Low data byte storage
	uint8_t DataH = 0x05;//*((uint8_t *)(&data)+1);			// High data byte storage
	uint8_t arr[6];			// Buffer for the sent bytes
	uint8_t PecReg;			// Calculated PEC byte storage
	//uint8_t ErrorCounter;		// Defines the number of the attempts for communication with MLX90614
        
        uint8_t flag = 1;
	
	//ErrorCounter=0x00;				// Initialising of ErrorCounter
        
        
        arr[5]=0;		//
        arr[4]=SlaveAddress;				//
        arr[3]=command;
        arr[2]=DataL;				//
        arr[1]=DataH;				//
        arr[0]=0;					//
        PecReg=PEC_calculation(arr);//Calculate CRC
                
	count = 0;
	do{
	repeat:
		STOP_bit();					//If slave send NACK stop comunication	
		--ErrorCounter;				//Pre-decrement ErrorCounter
		if(!ErrorCounter){			//ErrorCounter=0?
                        flag = 0;
			break;					//Yes,go out from do-while{}
		}
		START_bit();				//Start condition
		
		if(TX_byte(0xB4)){	//Send SlaveAddress
			goto	repeat;			//Repeat comunication again
		}
                
                count++;
			
		if(TX_byte(0x22)){		//Send command	
			goto	repeat;			//Repeat comunication again
		}
                
                count++;
                
		if(TX_byte(0x07)){		//Send command	
			goto	repeat;			//Repeat comunication again
		}
                
                count++;
                
		if(TX_byte(0xC8)){		//Send command	
			goto	repeat;			//Repeat comunication again
		}
                
                count++;
                
                    
		if(TX_byte(0x48)){		//Send command	
			goto	repeat;			//Repeat comunication again
		}
                
                               
                count++;

                
		STOP_bit();					//Stop condition
		
                break;
	}while(1);		//If received and calculated CRC are equal go out from do-while{}
		
	
	return flag;							
}

float CalcTemp(uint32_t value)
{
	float temp;
	
	temp=(value*0.02)-273.15;
	
	return temp;
}

uint8_t 	bitPosition=47;
uint8_t PEC_calculation(uint8_t pec[])
{
	uint8_t 	crc[6];
        uint8_t 	BitPosition=47;
	uint8_t 	shift;
	uint8_t 	i;
	uint8_t 	j;
	uint8_t 	temp;

	do{
		crc[5]=0;				/* Load CRC value 0x000000000107 */
		crc[4]=0;
		crc[3]=0;
		crc[2]=0;
		crc[1]=0x01;
		crc[0]=0x07;
		BitPosition=47;			/* Set maximum bit position at 47 */
		shift=0;
				
		//Find first 1 in the transmited message
		i=5;					/* Set highest index */
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
		
		shift=BitPosition-8;	/*Get shift value for crc value*/
		
                if(pec[5] == 0)
                  bitPosition = BitPosition;
		//Shift crc value 
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
		
		
		//Exclusive OR between pec and crc		
		for(i=0; i<=5; i++){
			pec[i] ^=crc[i];
		}/*End of for*/
	}while(BitPosition>8);/*End of do-while*/
	
	return pec[0];
}/*End of PEC_calculation*/


void SendRequest(void)
{
	mSCL_LOW();			//SCL 1 ____________|<-----80ms------->|______________
	HAL_Delay(80);		// 	  0	            |__________________|
	mSCL_HIGH();
}


void DummyCommand(uint8_t byte)
{
	START_bit();		//Start condition
	TX_byte(byte);		//Send Slave Address or whatever,no need ACK checking
	STOP_bit();			//Stop condition
}

///////////////////////////////////////0
static void HAL_I2C_ClearBusyFlagErrata_2_14_7(I2C_HandleTypeDef *hi2c);


void TestIt() {
  
  	uint8_t 	SlaveAddress; 			//Contains device address
	uint8_t	command;	  			//Contains the access command
	

		
	MCUinit();						  		//MCU initialization
	SlaveAddress=SA<<1;						//Set device address
	command=RAM_Access|RAM_Tobj1; 	        //Form RAM access command + RAM address 			
	
	//SendRequest();							//Switch to SMBus mode - this is need if module is in PWM mode only
	//DummyCommand(SlaveAddress);				//This is need if Request Command is sent even when the module is in SMBus mode
	//delay(DEL200ms);						//Wait after POR,Tvalid=0.15s 
	HAL_Delay(200);
        
        
        uint32_t value = 0xBE5B;
        pyroMeterData = 0;
        
	while(1)
	{
                               
                //W_Success=MemWrite(SlaveAddress,command, value);
                
                
                //while(1) {
                //}
                
		pyroMeterData=MemRead(SlaveAddress,command); //Read memory
                
                tempCelcius=CalcTemp(pyroMeterData);					//Calculate temperature
                
                
                HAL_Delay(1000);
                
                count ++;
                
                
                
	}
        
        while(1) {
          
        }
  
}


uint8_t GetDigits(float t, char *val) {
  
  if(t <= 0.0)
    return 0;
  
  uint32_t temp, value;
  
  uint32_t factor = 1;
  
  for (int i=0; i<DIGITS_AFTER_DEC; i++)
    factor *= 10;
  
  value = (uint32_t) factor*t;
  
  if(value > 9999)
    return 0;
  
  temp = value;
  val[0] = '0' + temp / 1000;  
  temp = temp % 1000;
  
  val[1] = '0' + temp/100;
  temp = temp % 100;
  
  val[2] = '0' + temp/10;
  temp = temp % 10;
  
  val[3] = '0' + temp;
    
}

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600; 
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  
  
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if ((ret=HAL_UART_Init(&huart2)) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  
  if(huart == &huart2)
  {
    

      //sentBufferEmpty = 0;
      transferCallback ++;
      
      sentBufferEmpty = 1;
      
    //__HAL_UART_DISABLE_IT(&huart1, UART_IT_TC);
  
      
    
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  
  
  if(sentBufferEmpty == 0)
    writeErrorCount ++;
  else
    readErrorCount ++;
  
  errCode[transferErrorCount%256] = huart->ErrorCode;
  transferErrorCount ++;
  sentBufferEmpty = 1;
  
  
  
  //UART_RxAgain(huart);
}



void USART_ClearITPendingBit(UART_HandleTypeDef* USARTx, uint16_t USART_IT)
{
  uint16_t bitpos = 0x00, itmask = 0x00;
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_CLEAR_IT(USART_IT));
  /* The CTS interrupt is not available for UART4 and UART5 */
  if (USART_IT == UART_IT_CTS)
  {
    assert_param(IS_USART_123_PERIPH(USARTx));
  }   
  
  bitpos = USART_IT >> 0x08;
  itmask = ((uint16_t)0x01 << (uint16_t)bitpos);
  USARTx->Instance->SR = (uint16_t)~itmask;
}
  
