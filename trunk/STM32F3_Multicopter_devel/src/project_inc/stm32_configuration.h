/*
 * stm32_configuration.h
 *
 *  Created on: Nov 16, 2012
 *      Author: franz
 */

#ifndef STM32_CONFIGURATION_H_
#define STM32_CONFIGURATION_H_

#include <stm32f30x.h>
#include <stm32f30x_conf.h>
#include <helperFunctions.h>

#define PRESCALEVALUE 6

void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void USART_Configuration(void);
void SPI_Configuration(void);
void I2C_Configuration(void);
void TIM2_PWM_Configuration(void);
void TIM3_PWM_Configuration(void);
void TIM4_Configuration(void);
void ADC_Configuration(void);
void DMA_Configuration(uint8_t *buffer);
EXTI_InitTypeDef *getEXTI_Config(void);
void EXTI_Configuration(void);
#endif /* STM32_CONFIGURATION_H_ */
