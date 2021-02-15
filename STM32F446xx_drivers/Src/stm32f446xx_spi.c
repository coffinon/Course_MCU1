#include "stm32f446xx_spi.h"


void SPI_PeripheralClockControl(SPI_RegDef_t *pSPI, uint8_t state)
{
	if(state == ENABLE)
	{
		if(pSPI == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPI == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPI == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPI == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPI == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPI == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else
		{
			SPI4_PCLK_DI();
		}
	}
}



void SPI_Init(SPI_Handle_t *pSPI_Handle)
{

}


void SPI_DeInit(SPI_RegDef_t *pSPI)
{
	if(pSPI == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPI == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPI == SPI3)
	{
		SPI3_REG_RESET();
	}
	else
	{
		SPI4_REG_RESET();
	}
}









