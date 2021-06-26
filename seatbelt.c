/*
int_blink.c
*
*  Created on: 22-June-2020
*      Author: Sourav Sahoo
*/


#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"
#include<stdint.h>

int key=0;

void LED_ON(void)
{
    GPIO_WriteToOutputPin(GPIOD, 12 , SET);
    GPIO_WriteToOutputPin(GPIOD, 14 , SET);
}

void LED_OFF(void)
{
    GPIO_WriteToOutputPin(GPIOD, 12 , RESET);
    GPIO_WriteToOutputPin(GPIOD, 14 , RESET);
}


int idle(void)
{
    for(uint32_t i=0; i<60;i++)
    {
        for (uint32_t j=0; j<80000000; j++);
        GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NUMBER_12);
        GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NUMBER_14);
        
    }
    LED_OFF();
    return 0;
}

void delay2_5(void)
{
    for(uint32_t i=0; i<30;i++)
    {
        for (uint32_t j=0; j<80000000; j++);
        GPIO_WriteToOutputPin(GPIOD, 12 , SET);

    }
}

void delay_30(void)
{
    for(uint32_t i=0; i<30;i++)
    {
        for (uint32_t j=0; j<80000000; j++);
    }
}

void state_a()
{
    LED_ON();
    delay2_5();

}

void state_b()
{
    LED_OFF();
    delay_30();
}



int main(void)
{
	GPIO_HANDLE_t GpioLed1;
	GpioLed1.pGPIOx = GPIOD;
	GpioLed1.GPIO_PIN_CONFIG.GPIO_PIN_NUMBER = GPIO_PIN_NUMBER_12;
	GpioLed1.GPIO_PIN_CONFIG.GPIO_MODE = GPIO_MODE_OUT;
	GpioLed1.GPIO_PIN_CONFIG.GPIO_PIN_SPEED = GPIO_SPEED_LOW;
	GpioLed1.GPIO_PIN_CONFIG.GPIO_PIN_OPTYPE = GPIO_OP_TYP_PP;
	GpioLed1.GPIO_PIN_CONFIG.GPIO_PIN_PUPDR = GPIO_NO_PUPD;
    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&GpioLed1);

    GPIO_HANDLE_t GpioLed2;
	GpioLed2.pGPIOx = GPIOD;
	GpioLed2.GPIO_PIN_CONFIG.GPIO_PIN_NUMBER = GPIO_PIN_NUMBER_14;
	GpioLed2.GPIO_PIN_CONFIG.GPIO_MODE = GPIO_MODE_OUT;
	GpioLed2.GPIO_PIN_CONFIG.GPIO_PIN_SPEED = GPIO_SPEED_LOW;
	GpioLed2.GPIO_PIN_CONFIG.GPIO_PIN_OPTYPE = GPIO_OP_TYP_PP;
	GpioLed2.GPIO_PIN_CONFIG.GPIO_PIN_PUPDR = GPIO_NO_PUPD;
    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&GpioLed2);

    GPIO_HANDLE_t PushButton;
	PushButton.pGPIOx = GPIOA;
	PushButton.GPIO_PIN_CONFIG.GPIO_PIN_NUMBER = GPIO_PIN_NUMBER_0;
	PushButton.GPIO_PIN_CONFIG.GPIO_MODE = GPIO_MODE_IT_FT;
	PushButton.GPIO_PIN_CONFIG.GPIO_PIN_SPEED = GPIO_SPEED_VHIGH;
	PushButton.GPIO_PIN_CONFIG.GPIO_PIN_PUPDR = GPIO_PU;
	GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&PushButton);

    GPIO_IRQPriorityConfig(IRQ_NO_EXTI0,NVIC_IRQ_PRI0);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI0,ENABLE);

    while(1)
	{

        key=idle();
        if(key==0)
        {
            return 0;
        }

        else
        {
        state_a();
        state_b();
        return 0; 
        }        
    }
        
	return 0;

}

void EXTI0_IRQHandler(void)
{
    GPIO_IRQHandling(GPIO_PIN_NUMBER_0);
    state_a();
    state_b();

}