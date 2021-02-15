#ifndef STM32F446XX_H
#define STM32F446XX_H

/*
 * Includes
 */

#include <stdint.h>
#include <string.h>

/*
 * Generic Macros
 */

#define ENABLE							0x01
#define DISABLE							0x00
#define GPIO_PIN_SET					ENABLE
#define GPIO_PIN_RESET					DISABLE

/*
 * ARM Cortex-M4 Specific Addresses
 */

#define NVIC_ISER0						((volatile uint32_t *) 0xE000E100)
#define NVIC_ISER1						((volatile uint32_t *) 0xE000E104)
#define NVIC_ISER2						((volatile uint32_t *) 0xE000E108)
#define NVIC_ISER3						((volatile uint32_t *) 0xE000E10C)

#define NVIC_ICER0						((volatile uint32_t *) 0XE000E180)
#define NVIC_ICER1						((volatile uint32_t *) 0XE000E184)
#define NVIC_ICER2						((volatile uint32_t *) 0XE000E188)
#define NVIC_ICER3						((volatile uint32_t *) 0XE000E18C)

#define NVIC_IPR_BASE_ADDR				((volatile uint32_t *) 0xE000E400)

#define PRIORITY_BITS_NUMBER			0x04

/*
 * Memory Base Addresses
 */

#define ALIASMEM_BASE_ADDR				0x00000000U
#define FLASH_BASE_ADDR					0x08000000U
#define OPBYTES1_BASE_ADDR				0x1FFE0000U
#define SYSMEM_BASE_ADDR				0x1FFF0000U
#define OPBYTES2_BASE_ADDR				0x1FFFC000U
#define SRAM1_BASE_ADDR					0x20000000U
#define SRAM2_BASE_ADDR					0x2001C000U

/*
 * AHBx and APBx Bus Peripheral Base Addresses
 */

#define APB1_BASE_ADDR					0x40000000U
#define APB2_BASE_ADDR					0x40010000U
#define AHB1_BASE_ADDR					0x40020000U
#define AHB2_BASE_ADDR					0x50000000U
#define AHB3_BASE_ADDR					0x60000000U

/*
 * APB1 Peripherals Base Addresses
 */

#define SPI2_BASE_ADDR					( APB1_BASE_ADDR + 0x3800 )
#define SPI3_BASE_ADDR					( APB1_BASE_ADDR + 0x3C00 )

#define USART2_BASE_ADDR				( APB1_BASE_ADDR + 0x4400 )
#define USART3_BASE_ADDR				( APB1_BASE_ADDR + 0x4800 )

#define I2C1_BASE_ADDR					( APB1_BASE_ADDR + 0x5400 )
#define I2C2_BASE_ADDR					( APB1_BASE_ADDR + 0x5800 )
#define I2C3_BASE_ADDR					( APB1_BASE_ADDR + 0x5C00 )

/*
 * APB2 Peripherals Base Addresses
 */

#define USART1_BASE_ADDR				( APB2_BASE_ADDR + 0x1000 )
#define USART6_BASE_ADDR				( APB2_BASE_ADDR + 0x1400 )

#define SPI1_BASE_ADDR					( APB2_BASE_ADDR + 0x3000 )
#define SPI4_BASE_ADDR					( APB2_BASE_ADDR + 0x3400 )

#define SYSCFG_BASE_ADDR				( APB2_BASE_ADDR + 0x3800 )

#define EXTI_BASE_ADDR					( APB2_BASE_ADDR + 0x3C00 )

/*
 * AHB1 Peripherals Base Addresses
 */

#define GPIOA_BASE_ADDR					( AHB1_BASE_ADDR + 0x0000 )
#define GPIOB_BASE_ADDR					( AHB1_BASE_ADDR + 0x0400 )
#define GPIOC_BASE_ADDR					( AHB1_BASE_ADDR + 0x0800 )
#define GPIOD_BASE_ADDR					( AHB1_BASE_ADDR + 0x0C00 )
#define GPIOE_BASE_ADDR					( AHB1_BASE_ADDR + 0x1000 )
#define GPIOF_BASE_ADDR					( AHB1_BASE_ADDR + 0x1400 )
#define GPIOG_BASE_ADDR					( AHB1_BASE_ADDR + 0x1800 )
#define GPIOH_BASE_ADDR					( AHB1_BASE_ADDR + 0x1C00 )

#define RCC_BASE_ADDR					( AHB1_BASE_ADDR + 0x3800 )

/*
 ******************** Peripheral Register Definitions Structures ********************
 */

/*
 * RCC
 */

typedef struct
{
	volatile uint32_t CR;				// RCC clock control register
	volatile uint32_t PLLCFGR;			// RCC PLL configuration register
	volatile uint32_t CFGR;				// RCC clock configuration register
	volatile uint32_t CIR;				// RCC clock interrupt register
	volatile uint32_t AHB1RSTR;			// RCC AHB1 peripheral reset register
	volatile uint32_t AHB2RSTR;			// RCC AHB2 peripheral reset register
	volatile uint32_t AHB3RSTR;			// RCC AHB3 peripheral reset register
	volatile uint32_t RESERVED1;
	volatile uint32_t APB1RSTR;			// RCC APB1 peripheral reset register
	volatile uint32_t APB2RSTR;			// RCC APB2 peripheral reset register
	volatile uint32_t RESERVED2[2];
	volatile uint32_t AHB1ENR;			// RCC AHB1 peripheral clock enable register
	volatile uint32_t AHB2ENR;			// RCC AHB2 peripheral clock enable register
	volatile uint32_t AHB3ENR;			// RCC AHB3 peripheral clock enable register
	volatile uint32_t RESERVED3;
	volatile uint32_t APB1ENR;			// RCC APB1 peripheral clock enable register
	volatile uint32_t APB2ENR;			// RCC APB2 peripheral clock enable register
	volatile uint32_t RESERVED4[2];
	volatile uint32_t AHB1LPENR;		// RCC AHB1 peripheral clock enable in low power mode register
	volatile uint32_t AHB2LPENR;		// RCC AHB2 peripheral clock enable in low power mode register
	volatile uint32_t AHB3LPENR;		// RCC AHB3 peripheral clock enable in low power mode register
	volatile uint32_t RESERVED5;
	volatile uint32_t APB1LPENR;		// RCC APB1 peripheral clock enable in low power mode register
	volatile uint32_t APB2LPENR;		// RCC APB2 peripheral clock enable in low power mode register
	volatile uint32_t RESERVED6[2];
	volatile uint32_t BDCR;				// RCC Backup domain control register
	volatile uint32_t CSR;				// RCC clock control & status register
	volatile uint32_t RESERVED7[2];
	volatile uint32_t SSCGR;			// RCC spread spectrum clock generation register
	volatile uint32_t PLLI2SCFGR;		// RCC PLLI2S configuration register
	volatile uint32_t PLLSAICFGR;		// RCC PLL configuration register
	volatile uint32_t DCKCFGR;			// RCC dedicated clock configuration register
	volatile uint32_t CKGATENR;			// RCC clocks gated enable register
	volatile uint32_t DCKCFGR2;			// RCC dedicated clocks configuration register 2
}RCC_RegDef_t;

#define RCC								( ( RCC_RegDef_t * ) RCC_BASE_ADDR )

/*
 * GPIO
 */

typedef struct
{
	volatile uint32_t MODER;			// GPIO port mode register
	volatile uint32_t OTYPER;			// GPIO port output type register
	volatile uint32_t OSPEEDR;			// GPIO port output speed register
	volatile uint32_t PUPDR;			// GPIO port pull-up/pull-down register
	volatile uint32_t IDR;				// GPIO port input data register
	volatile uint32_t ODR;				// GPIO port output data register
	volatile uint32_t BSRR;				// GPIO port bit set/reset register
	volatile uint32_t LCKR;				// GPIO port configuration lock register
	volatile uint32_t AFR[2];			// GPIO alternate function register
}GPIO_RegDef_t;

#define GPIOA 							((GPIO_RegDef_t *) GPIOA_BASE_ADDR)
#define GPIOB 							((GPIO_RegDef_t *) GPIOB_BASE_ADDR)
#define GPIOC 							((GPIO_RegDef_t *) GPIOC_BASE_ADDR)
#define GPIOD 							((GPIO_RegDef_t *) GPIOD_BASE_ADDR)
#define GPIOE 							((GPIO_RegDef_t *) GPIOE_BASE_ADDR)
#define GPIOF 							((GPIO_RegDef_t *) GPIOF_BASE_ADDR)
#define GPIOG 							((GPIO_RegDef_t *) GPIOG_BASE_ADDR)
#define GPIOH 							((GPIO_RegDef_t *) GPIOH_BASE_ADDR)

/*
 * EXTI
 */

typedef struct
{
	volatile uint32_t IMR;				// Interrupt mask register
	volatile uint32_t EMR;				// Event mask register
	volatile uint32_t RTSR;				// Rising trigger selection register
	volatile uint32_t FTSR;				// Falling trigger selection register
	volatile uint32_t SWIER;			// Software interrupt event register
	volatile uint32_t PR;				// Pending register
}EXTI_RegDef_t;

#define EXTI							((EXTI_RegDef_t *) EXTI_BASE_ADDR)

/*
 * SYSCFG
 */

typedef struct
{
	volatile uint32_t MEMRMP;			// SYSCFG memory remap register
	volatile uint32_t PMC;				// SYSCFG peripheral mode configuration register
	volatile uint32_t EXTICR[4];		// SYSCFG external interrupt configuration registers
	volatile uint32_t CMPCR;			// Compensation cell control register
	volatile uint32_t CFGR;				// SYSCFG configuration register
}SYSCFG_RegDef_t;

#define SYSCFG							((SYSCFG_RegDef_t *) SYSCFG_BASE_ADDR)

/*
 * SPI
 */

typedef struct
{
	volatile uint16_t CR[2];			// SPI control registers
	volatile uint16_t SR;				// SPI status register
	volatile uint16_t DR;				// SPI data register
	volatile uint16_t CRCPR;			// SPI CRC polynomial register
	volatile uint16_t RXCRCR;			// SPI RX CRC register
	volatile uint16_t TXCRCR;			// SPI TX CRC register
	volatile uint16_t I2SCFGR;			// SPI_I2S configuration register
	volatile uint16_t I2SPR;			// SPI_I2S prescaler register
}SPI_RegDef_t;

#define SPI1							((SPI_RegDef_t *) SPI1_BASE_ADDR)
#define SPI2							((SPI_RegDef_t *) SPI2_BASE_ADDR)
#define SPI3							((SPI_RegDef_t *) SPI3_BASE_ADDR)
#define SPI4							((SPI_RegDef_t *) SPI4_BASE_ADDR)


/*
 * ******************** Macros ********************
 */

/*
 * GPIOx Clock Enable / Disable Macros
 */

#define GPIOA_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOF_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 5 ) )
#define GPIOG_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 6 ) )
#define GPIOH_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 7 ) )

#define GPIOA_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOF_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 5 ) )
#define GPIOG_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 6 ) )
#define GPIOH_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 7 ) )

/*
 * I2Cx Clock Enable / Disable Macros
 */

#define I2C1_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 21 ) )
#define I2C2_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 22 ) )
#define I2C3_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 23 ) )

#define I2C1_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 21 ) )
#define I2C2_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 22 ) )
#define I2C3_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 23 ) )

/*
 * SPIx Clock Enable / Disable Macros
 */

#define SPI1_PCLK_EN()					( RCC->APB2ENR |= ( 1 << 12 ) )
#define SPI2_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 14 ) )
#define SPI3_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 15 ) )
#define SPI4_PCLK_EN()					( RCC->APB2ENR |= ( 1 << 13 ) )

#define SPI1_PCLK_DI()					( RCC->APB2ENR &= ~( 1 << 12 ) )
#define SPI2_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 14 ) )
#define SPI3_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 15 ) )
#define SPI4_PCLK_DI()					( RCC->APB2ENR &= ~( 1 << 13 ) )

/*
 * USARTx Clock Enable / Disable Macros
 */

#define USART1_PCLK_EN()				( RCC->APB2ENR |= ( 1 << 4 ) )
#define USART2_PCLK_EN()				( RCC->APB1ENR |= ( 1 << 17 ) )
#define USART3_PCLK_EN()				( RCC->APB1ENR |= ( 1 << 18 ) )
#define USART4_PCLK_EN()				( RCC->APB1ENR |= ( 1 << 19 ) )
#define USART5_PCLK_EN()				( RCC->APB1ENR |= ( 1 << 20 ) )
#define USART6_PCLK_EN()				( RCC->APB2ENR |= ( 1 << 5 ) )

#define USART1_PCLK_DI()				( RCC->APB2ENR &= ~( 1 << 4 ) )
#define USART2_PCLK_DI()				( RCC->APB1ENR &= ~( 1 << 17 ) )
#define USART3_PCLK_DI()				( RCC->APB1ENR &= ~( 1 << 18 ) )
#define USART4_PCLK_DI()				( RCC->APB1ENR &= ~( 1 << 19 ) )
#define USART5_PCLK_DI()				( RCC->APB1ENR &= ~( 1 << 20 ) )
#define USART6_PCLK_DI()				( RCC->APB2ENR &= ~( 1 << 5 ) )

/*
 * SYSCFG Clock Enable / Disable Macros
 */

#define SYSCFG_PCLK_EN()				( RCC->APB2ENR |= ( 1 << 14 ) )

#define SYSCFG_PCLK_DI()				( RCC->APB2ENR &= ~( 1 << 14 ) )

/*
 * GPIOx Reset Registers Macros
 */

#define GPIOA_REG_RESET()				do{RCC->AHB1RSTR |=  (1 << 0); RCC->AHB1RSTR &= ~(1 << 0);}while(0)
#define GPIOB_REG_RESET()				do{RCC->AHB1RSTR |=  (1 << 1); RCC->AHB1RSTR &= ~(1 << 1);}while(0)
#define GPIOC_REG_RESET()				do{RCC->AHB1RSTR |=  (1 << 2); RCC->AHB1RSTR &= ~(1 << 2);}while(0)
#define GPIOD_REG_RESET()				do{RCC->AHB1RSTR |=  (1 << 3); RCC->AHB1RSTR &= ~(1 << 3);}while(0)
#define GPIOE_REG_RESET()				do{RCC->AHB1RSTR |=  (1 << 4); RCC->AHB1RSTR &= ~(1 << 4);}while(0)
#define GPIOF_REG_RESET()				do{RCC->AHB1RSTR |=  (1 << 5); RCC->AHB1RSTR &= ~(1 << 5);}while(0)
#define GPIOG_REG_RESET()				do{RCC->AHB1RSTR |=  (1 << 6); RCC->AHB1RSTR &= ~(1 << 6);}while(0)
#define GPIOH_REG_RESET()				do{RCC->AHB1RSTR |=  (1 << 7); RCC->AHB1RSTR &= ~(1 << 7);}while(0)

/*
 * SPIx Reset Registers Macros
 */

#define SPI1_REG_RESET()				do{RCC->APB2RSTR |= (1 << 12); RCC->APB2RSTR &= ~(1 << 12);}while(0)
#define SPI2_REG_RESET()				do{RCC->APB1RSTR |= (1 << 14); RCC->APB1RSTR &= ~(1 << 14);}while(0)
#define SPI3_REG_RESET()				do{RCC->APB1RSTR |= (1 << 15); RCC->APB1RSTR &= ~(1 << 15);}while(0)
#define SPI4_REG_RESET()				do{RCC->APB2RSTR |= (1 << 13); RCC->APB2RSTR &= ~(1 << 13);}while(0)


/*
 * Return Port Code Macro
 */

#define GET_GPIO_PORT(x)				(x == GPIOA) ? 0 : \
										(x == GPIOB) ? 1 : \
										(x == GPIOC) ? 2 : \
										(x == GPIOD) ? 3 : \
										(x == GPIOE) ? 4 : \
										(x == GPIOF) ? 5 : \
										(x == GPIOG) ? 6 : 7

/*
 * IRQ Numbers
 */

#define IRQ_EXTI0						0x06
#define IRQ_EXTI1						0x07
#define IRQ_EXTI2						0x08
#define IRQ_EXTI3						0x09
#define IRQ_EXTI4						0x0A
#define IRQ_EXTI9_5						0x17
#define IRQ_EXIT15_10					0x28

#define IRQ_PRI0						0x00
#define IRQ_PRI1						0x01
#define IRQ_PRI2						0x02
#define IRQ_PRI3						0x03
#define IRQ_PRI4						0x04
#define IRQ_PRI5						0x05
#define IRQ_PRI6						0x06
#define IRQ_PRI7						0x07
#define IRQ_PRI8						0x08
#define IRQ_PRI9						0x09
#define IRQ_PRI10						0x0A
#define IRQ_PRI11						0x0B
#define IRQ_PRI12						0x0C
#define IRQ_PRI13						0x0D
#define IRQ_PRI14						0x0E
#define IRQ_PRI15						0x0F



#endif /* STM32F446XX_H */
