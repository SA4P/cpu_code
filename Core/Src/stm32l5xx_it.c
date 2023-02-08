/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l5xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l5xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L5xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l5xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */


// USART2 IRQ handler
void USART2_IRQHandler(){
//	tx_rbuf.

	if(USART2->CR1 & USART_CR1_TXEIE_Msk && USART2->ISR & USART_ISR_TXE_Msk){			// Case: TD empty interrupt enabled AND TD empty flag is set


		// Case 1: Have data to send ==> Send Populate TDR with one byte
		byte_t next_byte;
		if(!rb_consume_one(&us2_tx_rbuf, &next_byte)){
			USART2->CR1 ^= USART_CR1_TXEIE;											// Couldn't receive anything from tx_rbuf ==> Buffer empty ==> Have consumed everything we could have consumed ==> Disable interrupt until buffer has been filled with some data
			#ifdef PROFILING_MACRO_AUTH_TIME
//				if(profiling_ctr_usart2 % 3 == 0){
//					PROFILING_SET_COUNTER_E(2);
//				}else if(profiling_ctr_usart2 % 5 == 0){
//					PROFILING_SET_COUNTER_E(6);
//				}else{
//					__NOP();
//				}
			#endif

			return;
		}
		USART2->TDR = next_byte;

		return;
	}

	if(USART2->CR1 & USART_CR1_RXNEIE_Msk && USART2->ISR & USART_ISR_RXNE_Msk){		// Case: RD not empty interrupt enabled AND RD not empty flag is set
		uint8_t rx_byte = USART2->RDR & 0xFF;											// 8 bit data ==> Mask RDR (Receive Data Register) with 0xFF.
		if(!rb_produce_one(&us2_rx_rbuf, rx_byte)){
			// Could not put into buffer because it was full ==> Assuming there is no other bug, this means that the buffer was not cleared fast enough!
			while(1){
				__NOP();
			}
		}
	}

}

// LPUART1 IRQ handler
void LPUART1_IRQHandler(){
//	tx_rbuf.

	if(LPUART1->CR1 & USART_CR1_TXEIE_Msk && LPUART1->ISR & USART_ISR_TXE_Msk){			// Case: TD empty interrupt enabled AND TD empty flag is set


		// Case 1: Have data to send ==> Send Populate TDR with one byte
		byte_t next_byte;
		if(!rb_consume_one(&lp1_tx_rbuf, &next_byte)){
			LPUART1->CR1 ^= USART_CR1_TXEIE;											// Couldn't receive anything from tx_rbuf ==> Buffer empty ==> Have consumed everything we could have consumed ==> Disable interrupt until buffer has been filled with some data

			#ifdef PROFILING_MACRO_AUTH_TIME
//				if(profiling_ctr_lpuart % 4 == 0){										// profiling_ctr_lpuart is set to 4 by the send task just before sending out the dummy message used in the pairing procedure.
//					PROFILING_SET_COUNTER_E(4);
//				}else{
//					__NOP();
//				}
			#endif

			return;
		}
		LPUART1->TDR = next_byte;

		return;
	}

	if(LPUART1->CR1 & USART_CR1_RXNEIE_Msk && LPUART1->ISR & USART_ISR_RXNE_Msk){		// Case: RD not empty interrupt enabled AND RD not empty flag is set
		uint8_t rx_byte = LPUART1->RDR & 0xFF;											// 8 bit data ==> Mask RDR (Receive Data Register) with 0xFF.
		if(!rb_produce_one(&lp1_rx_rbuf, rx_byte)){
			// Could not put into buffer because it was full ==> Assuming there is no other bug, this means that the buffer was not cleared fast enough!
			while(1){
				__NOP();
			}
		}
	}

}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
