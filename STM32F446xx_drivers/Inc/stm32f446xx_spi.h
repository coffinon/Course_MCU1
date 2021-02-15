#ifndef STM32F446XX_SPI_H
#define STM32F445XX_SPI_H

#include "stm32f446xx.h"

/*
 * GPIO Pin Config Structure
 */

typedef struct
{
	uint8_t SPI_DeviceMode;					// Pin number
	uint8_t SPI_BusConfig;					// Pin mode
	uint8_t SPI_SclkSpeed;					// Pin speed
	uint8_t SPI_DFF;						// Pull up / pull down control
	uint8_t SPI_CPOL;						// Clock polarity control
	uint8_t SPI_CPHA;						// Clock (phase) trigger control
	uint8_t SPI_SSM;						//
}SPI_Config_t;


/*
 * GPIO Handle Structure
 */

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
}SPI_Handle_t;

/*
 * ******************** SPI Function Prototypes ********************
 */

/*
 * SPI Peripheral Clock Setup
 */

void SPI_PeripheralClockControl(SPI_RegDef_t *pSPI, uint8_t state);

/*
 * SPI Init And DeInit
 */

void SPI_Init(SPI_Handle_t *pSPI_Handle);
void SPI_DeInit(SPI_RegDef_t *pSPI);

/*
 * SPI Send / Receive Data
 */

void SPI_SendData(SPI_RegDef_t *pSPI, uint8_t *pTxBuffer, uint32_t length);
void SPI_ReceiveData(SPI_RegDef_t *pSPI, uint8_t *pRxBuffer, uint32_t length);

/*
 * SPI IRQ Configuration and ISR Handling
 */

void SPI_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t state);
void SPI_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQ_Handler(SPI_Handle_t *pSPI_Handle);



#endif /* STM32F446XX_SPI_H */
