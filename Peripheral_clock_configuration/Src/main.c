#include <stdint.h>

#define RCC_BASE_ADDR			0x40023800UL
#define ADC1_BASE_ADDR			0x40012000UL

#define RCC_APB2ENR_OFFSET		0x44UL
#define ADC1_CR1_OFFSET			0x04UL

#define RCC_APB2ENR_ADDR		(RCC_BASE_ADDR + RCC_APB2ENR_OFFSET)
#define ADC1_CR1_ADDR			(ADC1_BASE_ADDR + ADC1_CR1_OFFSET)

int main(void)
{
	uint32_t *pRcc_apb2enr 	= 	(uint32_t*) RCC_APB2ENR_ADDR;
	uint32_t *pAdc1_cr1 	= 	(uint32_t*) ADC1_CR1_ADDR;

	//1. Enable the peripheral clock for ADC1

	*pRcc_apb2enr |= (1 << 8);

	//2. Modify ADC1_CR1 register

	*pAdc1_cr1 |= (1 << 8);


	for(;;);
}
