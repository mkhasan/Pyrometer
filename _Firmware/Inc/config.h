#ifndef CONFIG_H
#define CONFIG_H

#define DIGIT_CTRL_PORT GPIOA
#define SEG_ABFG_PORT GPIOB
#define SEG_CDEP_PORT GPIOA         // P is used an alias of DP

/*  
#define DIG1_CTRL_PIN GPIO_PIN_3
#define DIG2_CTRL_PIN GPIO_PIN_5
#define DIG3_CTRL_PIN GPIO_PIN_4
#define DIG4_CTRL_PIN GPIO_PIN_6
*/

#define DIG1_CTRL_PIN GPIO_PIN_4
#define DIG2_CTRL_PIN GPIO_PIN_6
#define DIG3_CTRL_PIN GPIO_PIN_5
#define DIG4_CTRL_PIN GPIO_PIN_7


#define SEG_A_PIN GPIO_PIN_14
#define SEG_B_PIN GPIO_PIN_12
#define SEG_C_PIN GPIO_PIN_8
#define SEG_D_PIN GPIO_PIN_10
#define SEG_E_PIN GPIO_PIN_11
#define SEG_F_PIN GPIO_PIN_13
#define SEG_G_PIN GPIO_PIN_15
#define SEG_P_PIN GPIO_PIN_9

#define ADC_BUFFER_LENGTH 256

#define PYROMETER_PIN GPIO_PIN_2
#define PYROMETER_PORT GPIOA

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

#define ADDR_LSB3_PORT GPIOB
#define ADDR_MSB_PORT GPIOA

#define ADDR_PIN_3 GPIO_PIN_1
#define ADDR_PIN_2 GPIO_PIN_7
#define ADDR_PIN_1 GPIO_PIN_6
#define ADDR_PIN_0 GPIO_PIN_5

#define ADDR_PIN_COUNT 4

#define RS485_ENABLE_PORT GPIOB
#define RS485_RE_PIN GPIO_PIN_1
#define RS485_DE_PIN GPIO_PIN_0

#define WRITE_DATA_LEN 2


#endif