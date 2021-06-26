/*
 * stm32f407xx.h
 *
 *  Created on: Jun 18, 2021
 *      Author: sourav
 */

#ifndef STM32F4XX_H_
#define STM32F4XX_H_
#include<stdint.h>
/*base addresses of the memory components*/
#define FLASH_BASEADDR 	0x08000000U
#define SRAM_BASEADDR 	0x20000000U
#define SRAM1_BASEADDR 	0x20000000U
#define SRAM2_BASEADDR 	0x2001C000U
//fill up the

/*base addresses of the Bus System*/
#define AHB1_BASEADDR 	0x40020000U
#define APB1_BASEADDR 	0x40000000U
#define APB2_BASEADDR 	0x40010000U


/*base addresses of Peripherals hanging on to AHB1*/
#define GPIOA_BASEADDR 	(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR 	(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 	(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 	(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 	(AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 	(AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 	(AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 	(AHB1_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR 	(AHB1_BASEADDR + 0x2000)
#define RCC_BASEADDR	(AHB1_BASEADDR + 0x3800)

/*base addresses of Peripherals hanging on to APB1*/
#define TIM2_BASEADDR 	(APB1_BASEADDR + 0x0000)
#define TIM3_BASEADDR 	(APB1_BASEADDR + 0x0400)
#define TIM4_BASEADDR 	(APB1_BASEADDR + 0x0800)
#define TIM5_BASEADDR 	(APB1_BASEADDR + 0x0C00)
#define TIM6_BASEADDR 	(APB1_BASEADDR + 0x1000)
#define TIM7_BASEADDR 	(APB1_BASEADDR + 0x1400)
#define TIM12_BASEADDR 	(APB1_BASEADDR + 0x1800)
#define TIM13_BASEADDR 	(APB1_BASEADDR + 0x1C00)
#define TIM14_BASEADDR 	(APB1_BASEADDR + 0x2000)
#define RTC_BASEADDR 	(APB1_BASEADDR + 0x2800)
#define WWDG_BASEADDR 	(APB1_BASEADDR + 0x2C00)
#define IWDG_BASEADDR 	(APB1_BASEADDR + 0x3000)
#define I2S2_BASEADDR 	(APB1_BASEADDR + 0x3400)
#define SPI2_BASEADDR 	(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR 	(APB1_BASEADDR + 0x3C00)
#define I2S3ex_BASEADDR (APB1_BASEADDR + 0x4000)
#define USART2_BASEADDR (APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR (APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR 	(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR 	(APB1_BASEADDR + 0x5000)
#define I2C1_BASEADDR 	(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR 	(APB1_BASEADDR + 0x5800
#define I2C3_BASEADDR 	(APB1_BASEADDR + 0x5c00
#define CAN1_BASEADDR 	(APB1_BASEADDR + 0x6400
#define CAN2_BASEADDR 	(APB1_BASEADDR + 0x6800
#define PWR_BASEADDR 	(APB1_BASEADDR + 0x7000
#define DAC_BASEADDR 	(APB1_BASEADDR + 0x7400
#define UART7_BASEADDR 	(APB1_BASEADDR + 0x7800
#define UART8_BASEADDR 	(APB1_BASEADDR + 0x7C00

/*base addresses of Peripherals hanging on to APB2*/
#define TIM1_BASEADDR 	(APB2_BASEADDR + 0x0000)
#define TIM8_BASEADDR 	(APB2_BASEADDR + 0x0400)
#define USART1_BASEADDR (APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR (APB2_BASEADDR + 0x1400)
#define ADC_BASEADDR 	(APB2_BASEADDR + 0x2000)
#define SDIO_BASEADDR 	(APB2_BASEADDR + 0x2C00)
#define SPI1_BASEADDR 	(APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR 	(APB2_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR (APB2_BASEADDR + 0x3800)
#define EXTI_BASEADDR 	(APB2_BASEADDR + 0x3C00)
#define TIM9_BASEADDR 	(APB2_BASEADDR + 0x4000)
#define TIM10_BASEADDR 	(APB2_BASEADDR + 0x4400)
#define TIM11_BASEADDR 	(APB2_BASEADDR + 0x4800)
#define SPI5_BASEADDR 	(APB2_BASEADDR + 0x5000)
#define SPI6_BASEADDR 	(APB2_BASEADDR + 0x5400)
#define SAI1_BASEADDR 	(APB2_BASEADDR + 0x5800)
#define LCD_BASEADDR 	(APB2_BASEADDR + 0x6800)

/*typedef for GPIO*/
typedef struct
{
	uint32_t MODER;
	uint32_t OTYPER;
	uint32_t OSPEEDR;
	uint32_t PUPDR;
	uint32_t IDR;
	uint32_t ODR;
	uint32_t BSRRL;
	uint32_t BSRRH;
	uint32_t LCKR;
	uint32_t AFR[2];

} GPIO_RegDef_t;

/*typedef for RCC*/
typedef struct
{
	uint32_t CR;
	uint32_t PLLCFGR;
	uint32_t CFGR;
	uint32_t CIR;
	uint32_t AHB1RSTR;
	uint32_t AHB2RSTR;
	uint32_t AHB3RSTR;
	uint32_t RESERVED1;
	uint32_t APB1RSTR;
	uint32_t APB2RSTR;
	uint32_t RESERVED2[2];
	uint32_t AHB1ENR;
	uint32_t AHB2ENR;
	uint32_t AHB3ENR;
	uint32_t RESERVED3;
	uint32_t APB1ENR;
	uint32_t APB2ENR;
	uint32_t RESERVED4[2];
	uint32_t AHB1LPENR;
	uint32_t AHB2LPENR;
	uint32_t AHB3LPENR;
	uint32_t RESERVED5;
	uint32_t APB1LPENR;
	uint32_t APB2LPENR;
	uint32_t RESERVED6[2];
	uint32_t BDCR;
	uint32_t CSR;
	uint32_t RESERVED7[2];
	uint32_t SSCGR;
	uint32_t PLLI2SCFGR;
	uint32_t PLLSAICFGR;
	uint32_t DCKCFGR;
}RCC_RegDef_t;

typedef struct 
{
	uint32_t MEMPMR;
	uint32_t PMC;
	uint32_t EXTICR[4];
	uint32_t RESERVED[2];
	uint32_t CMPCRR;
	uint32_t RESERVED2[2];
	uint32_t CFGR;
} SYSCFG_RegDef_t;


typedef struct
{
	uint32_t IMR;
	uint32_t EMR;
	uint32_t RTSR;
	uint32_t FTSR;
	uint32_t SWIER;
	uint32_t PR;
}EXTI_RegDef_t;



// Macros for GPIO Port Defination
#define GPIOA		(GPIO_RegDef_t*) 	GPIOA_BASEADDR
#define GPIOB		(GPIO_RegDef_t*) 	GPIOB_BASEADDR
#define GPIOC		(GPIO_RegDef_t*) 	GPIOC_BASEADDR
#define GPIOD		(GPIO_RegDef_t*) 	GPIOD_BASEADDR
#define GPIOE		(GPIO_RegDef_t*) 	GPIOE_BASEADDR
#define GPIOF		(GPIO_RegDef_t*) 	GPIOF_BASEADDR
#define GPIOG		(GPIO_RegDef_t*) 	GPIOG_BASEADDR
#define GPIOH		(GPIO_RegDef_t*) 	GPIOH_BASEADDR
#define GPIOI		(GPIO_RegDef_t*) 	GPIOI_BASEADDR


//Clock Enable Macros for GPIO Peripherals
#define GPIOA_PCLOCK_EN()		RCC->AHB1ENR|=(1<<0)
#define GPIOB_PCLOCK_EN()		RCC->AHB1ENR|=(1<<1)
#define GPIOC_PCLOCK_EN()		RCC->AHB1ENR|=(1<<2)
#define GPIOD_PCLOCK_EN()		RCC->AHB1ENR|=(1<<3)
#define GPIOE_PCLOCK_EN()		RCC->AHB1ENR|=(1<<4)
#define GPIOF_PCLOCK_EN()		RCC->AHB1ENR|=(1<<5)
#define GPIOG_PCLOCK_EN()		RCC->AHB1ENR|=(1<<6)
#define GPIOH_PCLOCK_EN()		RCC->AHB1ENR|=(1<<7)
#define GPIOI_PCLOCK_EN()		RCC->AHB1ENR|=(1<<8)

//Clock Enable Macros for SPI Peripherals
#define SPI1_PCLOCK_EN()			RCC->APB2ENR|=(1<<12)
#define SPI2_PCLOCK_EN()			RCC->APB1ENR|=(1<<14)
#define SPI3_PCLOCK_EN()			RCC->APB1ENR|=(1<<15)
#define SPI4_PCLOCK_EN()			RCC->APB2ENR|=(1<<13)
#define SPI5_PCLOCK_EN()			RCC->APB2ENR|=(1<<20)
#define SPI6_PCLOCK_EN()			RCC->APB2ENR|=(1<<21)

//Clock Enable Macros for I2C Peripherals
#define I2C1_PCLOCK_EN()			RCC->APB1ENR|=(1<<21)
#define I2C2_PCLOCK_EN()			RCC->APB1ENR|=(1<<22)
#define I2C3_PCLOCK_EN()			RCC->APB1ENR|=(1<<23)

//Clock Disable Macros for GPIO Peripherals
#define GPIOA_PCLOCK_DIS()		RCC->AHB1ENR&=~(1<<0)
#define GPIOB_PCLOCK_DIS()		RCC->AHB1ENR&=~(1<<1)
#define GPIOC_PCLOCK_DIS()		RCC->AHB1ENR&=~(1<<2)
#define GPIOD_PCLOCK_DIS()		RCC->AHB1ENR&=~(1<<3)
#define GPIOE_PCLOCK_DIS()		RCC->AHB1ENR&=~(1<<4)
#define GPIOF_PCLOCK_DIS()		RCC->AHB1ENR&=~(1<<5)
#define GPIOG_PCLOCK_DIS()		RCC->AHB1ENR&=~(1<<6)
#define GPIOH_PCLOCK_DIS()		RCC->AHB1ENR&=~(1<<7)
#define GPIOI_PCLOCK_DIS()		RCC->AHB1ENR&=~(1<<8)

//Clock Disable Macros for SPI Peripherals
#define SPI1_PCLOCK_DIS()			RCC->RCC_APB2ENR&=~(1<<12)
#define SPI2_PCLOCK_DIS()			RCC->RCC_APB1ENR&=~(1<<14)
#define SPI3_PCLOCK_DIS()			RCC->RCC_APB1ENR&=~(1<<15)
#define SPI4_PCLOCK_DIS()			RCC->RCC_APB2ENR&=~(1<<13)
#define SPI5_PCLOCK_DIS()			RCC->RCC_APB2ENR&=~(1<<20)
#define SPI6_PCLOCK_DIS()			RCC->RCC_APB2ENR&=~(1<<21)

//Clock Disable Macros for I2C Peripherals
#define I2C1_PCLOCK_DIS()			RCC->RCC_APB1ENR&=~(1<<21)
#define I2C2_PCLOCK_DIS()			RCC->RCC_APB1ENR&=~(1<<22)
#define I2C3_PCLOCK_DIS()			RCC->RCC_APB1ENR&=~(1<<23)


//Clock Enable Macros for SYSCFG peripheral
#define SYSCFG_PCLK_EN() 			(RCC->APB2ENR |= (1 << 14))

//Clock Disable Macros for GPIOx peripherals
#define SYSCFG_PCLK_DI() 			(RCC->APB2ENR &= ~(1 << 14))

// NVIC ISERx register Addresses
#define NVIC_ISER0					(uint32_t*)0xE000E100
#define NVIC_ISER1					(uint32_t*)0xE000E104
#define NVIC_ISER2					(uint32_t*)0xE000E108
#define NVIC_ISER3					(uint32_t*)0xE000E10C

// NVIC ICERx register Addresses
#define NVIC_ICER0					(uint32_t*)0xE000E180
#define NVIC_ICER1					(uint32_t*)0xE000E184
#define NVIC_ICER2					(uint32_t*)0xE000E188
#define NVIC_ICER3					(uint32_t*)0xE000E18C

//Priority Register Address Calculation
#define NVIC_PR_BASE_ADDR 			((uint32_t*)0xE000E400)

//No of Priority Bits Implemented
#define NO_PR_BITS_IMPLEMENTED 		4

//Macro to return a code between 0 to 7 for given base address.
#define GPIO_BASEADDR_TO_CODE(x)	  ( (x == GPIOA) ?0:\
										(x == GPIOB) ?1:\
										(x == GPIOC) ?2:\
										(x == GPIOD) ?3:\
										(x == GPIOE) ?4:\
										(x == GPIOF) ?5:\
										(x == GPIOG) ?6:\
										(x == GPIOH) ?7: \
										(x == GPIOI) ?8:0)

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


// Macros for all the possible priority levels
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15
						
// Macros for RCC Defination
#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)


// Macros for EXTI Defination
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/**************************************************************************/

#define ENABLE 1
#define DISABLE 0
#define SET 1
#define RESET 0

#endif /* STM32F4XX_H_ */

