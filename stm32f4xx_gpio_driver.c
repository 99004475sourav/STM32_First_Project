#include "stm32f4xx_gpio_driver.h"
#include <stdint.h>

//GPIO Peripheral Clock Setup
void GPIO_PeriClockControl (GPIO_RegDef_t *pGPIOx, uint16_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx==GPIOA)
			{
				GPIOA_PCLOCK_EN();
			}
		else if(pGPIOx==GPIOB)
			{
				GPIOB_PCLOCK_EN();
			}
		else if(pGPIOx==GPIOC)
			{
				GPIOC_PCLOCK_EN();
			}
		else if(pGPIOx==GPIOD)
			{
				GPIOD_PCLOCK_EN();
			}
		else if(pGPIOx==GPIOE)
			{
				GPIOE_PCLOCK_EN();
			}
	}

	else
	{
		if(pGPIOx==GPIOA)
			{
				GPIOA_PCLOCK_DIS();
			}
		else if(pGPIOx==GPIOB)
			{
				GPIOB_PCLOCK_DIS();
			}
		else if(pGPIOx==GPIOC)
			{
				GPIOC_PCLOCK_DIS();
			}
		else if(pGPIOx==GPIOD)
			{
				GPIOD_PCLOCK_DIS();
			}
		else if(pGPIOx==GPIOE)
			{
				GPIOE_PCLOCK_DIS();
			}
	}
}

//GPIO Init and DeInit
void GPIO_Init (GPIO_HANDLE_t *pGPIOHandle)
{
	uint32_t temp=0;

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);

	//Configure the Mode
	if(pGPIOHandle->GPIO_PIN_CONFIG.GPIO_MODE<= GPIO_MODE_ANALOG)
	{
		temp=(pGPIOHandle->GPIO_PIN_CONFIG.GPIO_MODE<< (2*pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2*pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER));
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp=0;
	}
	else
	{
		if(pGPIOHandle->GPIO_PIN_CONFIG.GPIO_MODE == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER);

		}
		else if(pGPIOHandle->GPIO_PIN_CONFIG.GPIO_MODE == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER);
			EXTI->RTSR |=  ( 1 << pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER);

		}
		else if(pGPIOHandle->GPIO_PIN_CONFIG.GPIO_MODE == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER);

		}

		uint8_t temp1= pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER/4;
		uint8_t temp2= pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER%4;
		uint8_t portcode= GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1]=portcode<<(temp2 * 4);

		EXTI->IMR|= 1<< pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER;
	}

	//Configure the SPEED
	temp=(pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_SPEED<< (2*pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp=0;

	//Configure the Out Type
	temp=(pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_OPTYPE<< (1*pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER));
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp=0;

	//Configure the Push or Pull
	temp=(pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_PUPDR<< (1*pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER));
	pGPIOHandle->pGPIOx->PUPDR&= ~(0x3 << (2*pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp=0;

	
	// configure the alt functionality
	if(pGPIOHandle->GPIO_PIN_CONFIG.GPIO_MODE == GPIO_MODE_ALT)
	{
		//configure the alt function registers.
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER / 8;
		temp2 = pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER  % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_ALTFUN<< ( 4 * temp2 ) );
	}
}

void GPIO_DeInit (GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_PCLOCK_DIS();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_PCLOCK_DIS();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_PCLOCK_DIS();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_PCLOCK_DIS();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_PCLOCK_DIS();
	}
}

//GPIO Data read and write
uint8_t GPIO_ReadFromInputPin (GPIO_RegDef_t *pGPIOx, uint16_t PinNumber)
{
	uint8_t value;
	value=(uint8_t)(pGPIOx->IDR>>PinNumber)&(0x00000001);
	return value;

}

uint16_t GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx)
{
	uint16_t temp;
	temp=(uint16_t)(pGPIOx->IDR);
	return temp;

}

void GPIO_WriteToOutputPin (GPIO_RegDef_t *pGPIOx, uint16_t PinNumber,uint16_t Value)
{
	if(Value == SET)
	{
		pGPIOx->ODR|=(1<<PinNumber);
	}
	else
	{
		pGPIOx->ODR&=~(1<<PinNumber);
	}

}

void GPIO_WriteToOutputPort (GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR=Value;
}

void GPIO_ToggleOutputPin (GPIO_RegDef_t *pGPIOx, uint16_t PinNumber)
{
	pGPIOx->ODR^=(1<<PinNumber);

}

//GPIO configuration and ISR Handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi==ENABLE)
	{
		if (IRQNumber<=31)
		{
			*NVIC_ISER0|=(1<<IRQNumber);
		}
		else if(IRQNumber>31 && IRQNumber <64)
		{
			*NVIC_ISER1|=(1<<(IRQNumber%32));
		}
		else if(IRQNumber>=64 && IRQNumber <96)
		{
			*NVIC_ISER2|=(1<<(IRQNumber%64));
		}		
	}

	else
	{
		if (IRQNumber<=31)
		{
			*NVIC_ICER0|=(1<<IRQNumber);
		}
		else if(IRQNumber>31 && IRQNumber <64)
		{
			*NVIC_ICER1|=(1<<(IRQNumber%32));
		}
		else if(IRQNumber>=64 && IRQNumber <96)
		{
			*NVIC_ICER2|=(1<<(IRQNumber%64));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;

	uint8_t shift_amount =(8 * iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx)|=(IRQPriority<<shift_amount);
}

void GPIO_IRQHandling (uint8_t PinNumber)
{
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR|=(1<<PinNumber);
	}
}
