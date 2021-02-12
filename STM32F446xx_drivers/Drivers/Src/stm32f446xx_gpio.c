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
		switch(pGPIO)
		{
			GPIOA :
				GPIOA_PCLK_EN();
				break;
			GPIOB :
				GPIOB_PCLK_EN();
				break;
			GPIOC :
				GPIOC_PCLK_EN();
				break;
			GPIOD :
				GPIOD_PCLK_EN();
				break;
			GPIOE :
				GPIOE_PCLK_EN();
				break;
			GPIOF :
				GPIOF_PCLK_EN();
				break;
			GPIOG :
				GPIOG_PCLK_EN();
				break;
			GPIOH :
				GPIOH_PCLK_EN();
		}
	}
	else
	{
		switch(pGPIO)
		{
			GPIOA :
				GPIOA_PCLK_DI();
				break;
			GPIOB :
				GPIOB_PCLK_DI();
				break;
			GPIOC :
				GPIOC_PCLK_DI();
				break;
			GPIOD :
				GPIOD_PCLK_DI();
				break;
			GPIOE :
				GPIOE_PCLK_DI();
				break;
			GPIOF :
				GPIOF_PCLK_DI();
				break;
			GPIOG :
				GPIOG_PCLK_DI();
				break;
			GPIOH :
				GPIOH_PCLK_DI();
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
	uint32_t temp;

	// 1. Configure the GPIO mode for the given pin
	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		pGPIO_Handle->pGPIOx->MODER &= ~(0x03 << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->MODER |= (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	}
	else
	{
		// interrupt mode
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
	switch(pGPIO)
	{
		GPIOA :
			GPIOA_REG_RESET();
			break;
		GPIOB :
			GPIOB_REG_RESET();
			break;
		GPIOC :
			GPIOC_REG_RESET();
			break;
		GPIOD :
			GPIOD_REG_RESET();
			break;
		GPIOE :
			GPIOE_REG_RESET();
			break;
		GPIOF :
			GPIOF_REG_RESET();
			break;
		GPIOG :
			GPIOG_REG_RESET();
			break;
		GPIOH :
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
 * 	@return					- uint8_t
 *
 * 	@note					- none
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber)
{

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

}


/********************************************************************
 * 	@function_name			- GPIO_IRQ_Config
 *
 * 	@brief					- This function configures the IRQ
 *
 * 	@param[in]				- IRQ number
 * 	@param[in]				- IRQ priority level
 * 	@param[in]				- State of the IRQ
 *
 * 	@return					- none
 *
 * 	@note					- none
 */
void GPIO_IRQ_Config(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state)
{

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

}

