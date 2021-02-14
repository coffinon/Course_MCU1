#ifndef STM32F446XX_GPIO_H
#define STM32F446XX_GPIO_H

#include "stm32f446xx.h"

/*
 * GPIO Pin Config Structure
 */

typedef struct
{
	uint8_t GPIO_PinNumber;					// Pin number
	uint8_t GPIO_PinMode;					// Pin mode
	uint8_t GPIO_PinSpeed;					// Pin speed
	uint8_t GPIO_PinPuPdControl;			// Pull up / pull down control
	uint8_t GPIO_PinOpType;					//
	uint8_t GPIO_PinAltFunMode;				// Alternative function mode
}GPIO_PinConfig_t;


/*
 * GPIO Handle Structure
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;					// Base address of the GPIO port
	GPIO_PinConfig_t GPIO_PinConfig;		// Configuration of the GPIO port
}GPIO_Handle_t;


/*
 * @GPIO_PIN_MODE
 * GPIO Pin Possible Modes
 */

#define GPIO_MODE_IN						0x00
#define GPIO_MODE_OUT						0x01
#define GPIO_MODE_ALT						0x02
#define GPIO_MODE_ANALOG					0x03
#define GPIO_MODE_IT_RT						0x04
#define GPIO_MODE_IT_FT						0x05
#define GPIO_MODE_IT_RFT					0x06

/*
 * @GPIO_PIN_OUTPUT_TYPE
 * GPIO Pin Possible Output Types
 */

#define GPIO_OP_TYPE_PP						0x00
#define GPIO_OP_TYPE_OD						0x01

/*
 * @GPIO_PIN_SPEED
 * GPIO Pin Possible Speeds
 */

#define GPIO_SPEED_LOW						0x00
#define GPIO_SPEED_MEDIUM					0x01
#define GPIO_SPEED_FAST						0x02
#define GPIO_SPEED_HIGH						0x03

/*
 * @GPIO_PIN_PUPD
 * GPIO Pin Possible Pull Up / Pull Down Modes
 */

#define GPIO_PUPD_NONE						0x00
#define GPIO_PUPD_PU						0x01
#define GPIO_PUPD_PD						0x02

/*
 * @GPIO_PIN_NUMBER
 * GPIO Pin Possible Pin Numbers
 */

#define GPIO_PIN_0							0x00
#define GPIO_PIN_1							0x01
#define GPIO_PIN_2							0x02
#define GPIO_PIN_3							0x03
#define GPIO_PIN_4							0x04
#define GPIO_PIN_5							0x05
#define GPIO_PIN_6							0x06
#define GPIO_PIN_7							0x07
#define GPIO_PIN_8							0x08
#define GPIO_PIN_9							0x09
#define GPIO_PIN_10							0x0A
#define GPIO_PIN_11							0x0B
#define GPIO_PIN_12							0x0C
#define GPIO_PIN_13							0x0D
#define GPIO_PIN_14							0x0E
#define GPIO_PIN_15							0x0F


/*
 * ******************** GPIO Function Prototypes ********************
 */

/*
 * GPIO Peripheral Clock Setup
 */

void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIO, uint8_t state);

/*
 * GPIO Init And DeInit
 */

void GPIO_Init(GPIO_Handle_t *pGPIO_Handle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIO);

/*
 * GPIO Data Read And Write
 */

uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIO);
void GPIO_WritePin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber, uint8_t value);
void GPIO_WritePort(GPIO_RegDef_t *pGPIO, uint16_t value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber);

/*
 * GPIO IRQ Configuration and ISR Handling
 */

void GPIO_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t state);
void GPIO_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQ_Handler(uint8_t PinNumber);



#endif /* STM32F446XX_GPIO_H */
