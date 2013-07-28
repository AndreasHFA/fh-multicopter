/**
  ******************************************************************************
  * @file    Project/STM32F30x_StdPeriph_Templates/stm32f30x_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-October-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f30x_it.h"
#include <remote_control.h>

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

///**
//  * @brief  This function handles SVCall exception.
//  * @param  None
//  * @retval None
//  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

///**
//  * @brief  This function handles PendSVC exception.
//  * @param  None
//  * @retval None
//  */
//void PendSV_Handler(void)
//{
//}

///**
//  * @brief  This function handles SysTick Handler.
//  * @param  None
//  * @retval None
//  */
//void SysTick_Handler(void)
//{
//}

// this is the interrupt request handler (IRQ) for ALL USART1 interrupts
/*USART Reception*/
void USART2_IRQHandler(void)
{
	static unsigned char rxBuffer[5];
	static uint8_t i = 0;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		/* Read one byte from the receive data register */
		rxBuffer[i] = USART_ReceiveData(USART2);
		if(i >= 4)
		{
			//xQueueSendFromISR( usartRxQueue, &msgPointer, &xYieldRequired );
			CalcRXValues(&rxBuffer[0], 55);
			i = 0;
		}
		else
		{
			if(rxBuffer[0] == 'A')
			{
				i++;
			}
			else
			{
				i = 0;
				rxBuffer[0] = 0;
				rxBuffer[1] = 0;
				rxBuffer[2] = 0;
				rxBuffer[3] = 0;
				rxBuffer[4] = 0;
			}
		}
	}
	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	USART_ClearFlag(USART2, USART_IT_RXNE);
}

void disableEXTI(uint32_t EXTINr)
{
	getEXTI_Config()->EXTI_Line = EXTINr;
	getEXTI_Config()->EXTI_LineCmd = DISABLE;
	EXTI_Init(getEXTI_Config());
}

void enableEXTI(uint32_t EXTINr)
{
	getEXTI_Config()->EXTI_Line = EXTINr;
	getEXTI_Config()->EXTI_LineCmd = ENABLE;
	EXTI_Init(getEXTI_Config());
}

/* Interupt Handlers */
void EXTI15_10_IRQHandler(void)
{
	static uint8_t timNr = 0;
	static uint8_t timStage = 0;
	static unsigned char rxBuffer[6];

	switch(timNr)
	{
	case 0:
		if (EXTI_GetITStatus(EXTI_Line12) == SET)
		{
			if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12) == 1)
			{
				timStage += 1;
				TIM_SetCounter(TIM4, 0);
			}
			else if (timStage == (timNr+1))
			{
				timNr += 1;
				/* add an offset to zero the GAS-value */
				rxBuffer[2] = TIM_GetCounter(TIM4);
				//GAS = TIM_GetCounter(TIM4);
				disableEXTI(EXTI_Line12);
				enableEXTI(EXTI_Line13);
			}
		}
		break;
	case  1:
		if (EXTI_GetITStatus(EXTI_Line13) == SET)
		{
			if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13) == 1)
			{
				timStage += 1;
				TIM_SetCounter(TIM4, 0);
			}
			else if (timStage == (timNr+1))
			{
				timNr += 1;
				rxBuffer[3] = TIM_GetCounter(TIM4);
				disableEXTI(EXTI_Line13);
				enableEXTI(EXTI_Line14);
			}
		}
		break;
	case  2:
		if (EXTI_GetITStatus(EXTI_Line14) == SET)
		{
			if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_14) == 1)
			{
				timStage += 1;
				TIM_SetCounter(TIM4, 0);
			}
			else if (timStage == (timNr+1))
			{
				timNr += 1;
				rxBuffer[4] = TIM_GetCounter(TIM4);
				disableEXTI(EXTI_Line14);
				enableEXTI(EXTI_Line15);
			}
		}
		break;
	case  3:
		if (EXTI_GetITStatus(EXTI_Line15) == SET)
		{
			if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_15) == 1)
			{
				timStage += 1;
				TIM_SetCounter(TIM4, 0);
			}
			else if (timStage == (timNr+1))
			{
				timNr += 1;
				rxBuffer[5] = TIM_GetCounter(TIM4);
				disableEXTI(EXTI_Line15);
				enableEXTI(EXTI_Line12);
			}
		}
		break;
	default:
		timNr = 0;
		timStage = 0;
		TIM_SetCounter(TIM4, 0);
		CalcRXValues(&rxBuffer[0], 75);
		break;
	}
	EXTI_ClearITPendingBit(EXTI_Line12);
	EXTI_ClearITPendingBit(EXTI_Line13);
	EXTI_ClearITPendingBit(EXTI_Line14);
	EXTI_ClearITPendingBit(EXTI_Line15);
}

/******************************************************************************/
/*                 STM32F30x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f30x.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
