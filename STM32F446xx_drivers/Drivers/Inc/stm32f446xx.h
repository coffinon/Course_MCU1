#ifndef STM32F446XX_H
#define STM32F446XX_H

/*
 * Includes
 */

#include <stdint.h>

/*
 * Generic Macros
 */

#define ENABLE							0x01
#define DISABLE							0x00
#define GPIO_PIN_SET					ENABLE
#define GPIO_PIN_RESET					DISABLE

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

#define TIM1_BASE_ADDR					( APB2_BASE_ADDR + 0x0000 )
#define TIM8_BASE_ADDR					( APB2_BASE_ADDR + 0x0400 )

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

#define RCC_BASE_ADDR					( AHB1_BASE_ADDR + 0x3000 )

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

#define RCC								( ( RCC_RefDef_t * ) RCC_BASE_ADDR )

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

#define GPIOA 							( ( GPIO_RegDef_t * ) GPIOA_BASE_ADDR )
#define GPIOB 							( ( GPIO_RegDef_t * ) GPIOB_BASE_ADDR )
#define GPIOC 							( ( GPIO_RegDef_t * ) GPIOC_BASE_ADDR )
#define GPIOD 							( ( GPIO_RegDef_t * ) GPIOD_BASE_ADDR )
#define GPIOE 							( ( GPIO_RegDef_t * ) GPIOE_BASE_ADDR )
#define GPIOF 							( ( GPIO_RegDef_t * ) GPIOF_BASE_ADDR )
#define GPIOG 							( ( GPIO_RegDef_t * ) GPIOG_BASE_ADDR )
#define GPIOH 							( ( GPIO_RegDef_t * ) GPIOH_BASE_ADDR )

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


#endif /* STM32F446XX_H */
