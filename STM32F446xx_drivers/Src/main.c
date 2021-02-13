#include "stm32f446xx.h"
#include "stm32f446xx_gpio.h"


void delay()
{
	for(uint32_t i = 0; i < 200000; i++);
}


int main(void)
{
	GPIO_Handle_t GPIO_Handle;

	GPIO_PeripheralClockControl(GPIOA, ENABLE);
	GPIO_PeripheralClockControl(GPIOC, ENABLE);

	// Push-Pull Configuration
	GPIO_Handle.pGPIOx 								= GPIOA;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_5;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinOpType		= GPIO_OP_TYPE_PP;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_SPEED_LOW;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PUPD_NONE;

	GPIO_Init(&GPIO_Handle);

	// Button Configuration
	GPIO_Handle.pGPIOx 								= GPIOC;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_13;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PUPD_NONE;

	GPIO_Init(&GPIO_Handle);

    while(1)
    {
    	if(GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
    	{
    		delay();
    		GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    	}
    }
}
