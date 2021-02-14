#include "stm32f446xx.h"
#include "stm32f446xx_gpio.h"


void delay()
{
	for(uint32_t i = 0; i < 200000; i++);
}


int main(void)
{
	GPIO_Handle_t GPIO_Handle;

	memset(&GPIO_Handle, 0, sizeof(GPIO_Handle));

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
	GPIO_Handle.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_IT_FT;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PUPD_NONE;

	GPIO_Init(&GPIO_Handle);

	GPIO_IRQ_Priority_Config(IRQ_EXIT15_10, IRQ_PRI15);
	GPIO_IRQ_Interrupt_Config(IRQ_EXIT15_10, ENABLE);

    while(1)
    {
    }
}


void EXTI15_10_IRQHandler(void)
{
	GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	delay();
	GPIO_IRQ_Handler(GPIO_PIN_13);
}
