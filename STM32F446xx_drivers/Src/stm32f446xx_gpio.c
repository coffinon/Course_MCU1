#include "stm32f446xx_gpio.h"


/********************************************************************
 * 	@function_name			- GPIO_PeripheralClockControl
 *
 * 	@brief					- This function enables and disables peripheral clock for the given GPIO port
 *
 * 	@param[in]				- Base address of GPIO port
 * 	@param[in]				- State of the peripheral clock for GPIO port
 *
 * 	@return					- none
 *
 * 	@note					- none
 */
void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIO, uint8_t state)
{
	if(state == ENABLE)
	{
		if(pGPIO == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIO == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIO == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIO == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIO == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIO == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIO == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIO == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIO == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIO == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIO == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIO == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIO == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIO == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else
		{
			GPIOH_PCLK_EN();
		}
	}
}


/********************************************************************
 * 	@function_name			- GPIO_Init
 *
 * 	@brief					- This function initializes GPIO port
 *
 * 	@param[in]				- GPIO port handle
 *
 * 	@return					- none
 *
 * 	@note					- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle)
{
	// 1. Configure the GPIO mode for the given pin
	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		pGPIO_Handle->pGPIOx->MODER &= ~(0x03 << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->MODER |= (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	}
	else
	{
		// 1.1 Configure the EXTI trigger
		if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			EXTI->FTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			EXTI->RTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI->FTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 1.2 Configure the SYSCFG register
		uint8_t temp1 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t temp_port = GET_GPIO_PORT(pGPIO_Handle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= (temp_port << (4 * temp2));


		// 1.3 Enable EXTI interrupt on the given pin
		EXTI->IMR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// 2. Configure the GPIO output type for the given pin
	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT)
	{
		pGPIO_Handle->pGPIOx->OTYPER &= ~(0x01 << (pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->OTYPER |= (pGPIO_Handle->GPIO_PinConfig.GPIO_PinOpType << (pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	}

	// 3. Configure the GPIO output speed for the given pin
	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT)
	{
		pGPIO_Handle->pGPIOx->OSPEEDR &= ~(0x03 << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->OSPEEDR |= (pGPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	}

	// 4. Configure the GPIO pull up / pull down for the given pin
	pGPIO_Handle->pGPIOx->PUPDR &= ~(0x03 << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->PUPDR |= (pGPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));

	// 5. Configure the GPIO alternative functionality for the given pin
	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT)
	{
		uint8_t temp1, temp2;

		temp1 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIO_Handle->pGPIOx->AFR[temp1] &= ~(0x0F << (4 * temp2));
		pGPIO_Handle->pGPIOx->AFR[temp1] |= (pGPIO_Handle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}


/********************************************************************
 * 	@function_name			- GPIO_DeInit
 *
 * 	@brief					- This function deinitializes GPIO port
 *
 * 	@param[in]				- Base address of GPIO port
 *
 * 	@return					- none
 *
 * 	@note					- none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIO)
{
	if(pGPIO == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIO == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIO == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIO == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIO == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIO == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIO == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else
	{
		GPIOH_REG_RESET();
	}
}


/********************************************************************
 * 	@function_name			- GPIO_ReadPin
 *
 * 	@brief					- This function reads from the given pin
 *
 * 	@param[in]				- Base address of GPIO port
 * 	@param[in]				- Pin number
 *
 * 	@return					- uint8_t (0 or 1)
 *
 * 	@note					- none
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIO->IDR >> PinNumber) & 0x00000001);
	return value;
}


/********************************************************************
 * 	@function_name			- GPIO_ReadPort
 *
 * 	@brief					- This function reads from the given port
 *
 * 	@param[in]				- Base address of GPIO port
 *
 * 	@return					- uint16_t
 *
 * 	@note					- none
 */
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIO)
{
	uint16_t value;

	value = (uint16_t) pGPIO->IDR;
	return value;
}


/********************************************************************
 * 	@function_name			- GPIO_WritePin
 *
 * 	@brief					- This function writes to the given pin
 *
 * 	@param[in]				- Base address of GPIO port
 * 	@param[in]				- Pin number
 * 	@param[in]				- Value to write
 *
 * 	@return					- none
 *
 * 	@note					- none
 */
void GPIO_WritePin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		pGPIO->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIO->ODR &= ~(1 << PinNumber);
	}
}


/********************************************************************
 * 	@function_name			- GPIO_WritePort
 *
 * 	@brief					- This function writes to the given port
 *
 * 	@param[in]				- Base address of GPIO port
 * 	@param[in]				- Pin number
 *
 * 	@return					- none
 *
 * 	@note					- none
 */
void GPIO_WritePort(GPIO_RegDef_t *pGPIO, uint16_t value)
{
	pGPIO->ODR = value;
}


/********************************************************************
 * 	@function_name			- GPIO_TogglePin
 *
 * 	@brief					- This function toggles given pin
 *
 * 	@param[in]				- Base address of GPIO port
 * 	@param[in]				- Pin number
 *
 * 	@return					- none
 *
 * 	@note					- none
 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber)
{
	pGPIO->ODR ^= (1 << PinNumber);
}


/********************************************************************
 * 	@function_name			- GPIO_IRQ_Interrupt_Config
 *
 * 	@brief					- This function configures the IRQ
 *
 * 	@param[in]				- IRQ number
 * 	@param[in]				- State of the IRQ
 *
 * 	@return					- none
 *
 * 	@note					- none
 */
void GPIO_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t state)
{
	if(state == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 63)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber <= 95)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 63)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber <= 95)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}


/********************************************************************
 * 	@function_name			- GPIO_IRQ_Priority_Config
 *
 * 	@brief					- This function configures the IRQ priority level
 *
 *  @param[in]				- IRQ number
 * 	@param[in]				- Wanted priority level for the given IRQ Number
 *
 * 	@return					- none
 *
 * 	@note					- none
 */
void GPIO_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx, iprx_section, shift;

	iprx 			= IRQNumber / 4;
	iprx_section 	= IRQNumber % 4;
	shift			= (iprx_section * 8) + (8 - PRIORITY_BITS_NUMBER);

	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << shift);
}


/********************************************************************
 * 	@function_name			- GPIO_IRQ_Handler
 *
 * 	@brief					- This function handles the IRQ
 *
 * 	@param[in]				- Pin number on which IRQ appears
 *
 * 	@return					- none
 *
 * 	@note					- none
 */
void GPIO_IRQ_Handler(uint8_t PinNumber)
{
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}

