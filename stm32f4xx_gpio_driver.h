/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: 21-Jun-2021
 *      Author: soura_epxd
 */

#ifndef DRIVERS_INC_STM32F4XX_GPIO_DRIVER_H_
#define DRIVERS_INC_STM32F4XX_GPIO_DRIVER_H_
#include "stm32f4xx.h"
#include <stdint.h>

//Structure for GPIO Pin Registers
typedef struct{
	uint16_t GPIO_PIN_NUMBER;	//Pin 0 to 15
	uint16_t GPIO_MODE;			//4 options:- Input, Output, Alternate, Analog
	uint16_t GPIO_PIN_SPEED;	//4 Options:- 00:Low, 01:Medium, 10:High, 11:Very High
	uint16_t GPIO_PIN_PUPDR;	//4 options
	uint16_t GPIO_PIN_OPTYPE;	//2 options
	uint16_t GPIO_PIN_ALTFUN;	//4 Option
}GPIO_PIN_CONFIG_t;

// Handle Structure for GPIO PIN
typedef struct{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PIN_CONFIG_t GPIO_PIN_CONFIG;
}GPIO_HANDLE_t;

//GPIO Pin Number
#define GPIO_PIN_NUMBER_0		0
#define GPIO_PIN_NUMBER_1		1
#define GPIO_PIN_NUMBER_2		2
#define GPIO_PIN_NUMBER_3		3
#define GPIO_PIN_NUMBER_4		4
#define GPIO_PIN_NUMBER_5		5
#define GPIO_PIN_NUMBER_6		6
#define GPIO_PIN_NUMBER_7		7
#define GPIO_PIN_NUMBER_8		8
#define GPIO_PIN_NUMBER_9		9
#define GPIO_PIN_NUMBER_10		10
#define GPIO_PIN_NUMBER_11		11
#define GPIO_PIN_NUMBER_12		12
#define GPIO_PIN_NUMBER_13		13
#define GPIO_PIN_NUMBER_14		14
#define GPIO_PIN_NUMBER_15		15

//GPIO Pin Mode
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALT			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6


//GPIO Pin Speed
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_HIGH			2
#define GPIO_SPEED_VHIGH		3

//GPIO Pin Pull up Pull Down
#define GPIO_NO_PUPD			0
#define GPIO_PU					1
#define GPIO_PD					2

//GPIO Pin Output Type
#define GPIO_OP_TYP_PP			0
#define GPIO_OP_TYP_OD			1

//GPIO Peripheral Clock Setup
void GPIO_PeriClockControl (GPIO_RegDef_t *pGPIOx, uint16_t EnorDi);

//GPIO Init and DeInit
void GPIO_Init (GPIO_HANDLE_t *pGPIOHandle);
void GPIO_DeInit (GPIO_RegDef_t *pGPIOx);

//GPIO Data read and write
uint8_t GPIO_ReadFromInputPin (GPIO_RegDef_t *pGPIOx, uint16_t PinNumber);
uint16_t GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin (GPIO_RegDef_t *pGPIOx, uint16_t PinNumber,uint16_t Value);
void GPIO_WriteToOutputPort (GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin (GPIO_RegDef_t *pGPIOx, uint16_t PinNumber);

//GPIO configuration and ISR Handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling (uint8_t PinNumber);



#endif /* DRIVERS_INC_STM32F4XX_GPIO_DRIVER_H_ */
