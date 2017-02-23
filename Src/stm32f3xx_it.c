/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* USER CODE BEGIN 0 */
#define NEXT 9
#define STEP_0 0
#define STEP_1 1
#define STEP_2 2
#define STEP_3 3
#define STEP_4 4
#define STEP_5 5
#define	I2CRECEIVEDBUFFERSIZE 8

extern uint8_t compState;
extern uint16_t commutationTimerCounter;
extern uint16_t commutationTimerCounterArray[6];
extern uint8_t commutationStepCounter;
extern void commutationPattern(uint8_t step);
extern uint8_t waitForCommutation;
extern uint16_t commutationTimerCounterAverage;
extern uint16_t commutationTimerOffset;
extern uint16_t counterTIM3Values;

extern uint8_t	rx_bytes_counter;
extern uint8_t	command_ready;
extern uint8_t	data_ready;
extern uint8_t rx_bytes[10];
extern uint8_t receiveBuffer[I2CRECEIVEDBUFFERSIZE];
extern uint16_t oc5Value;
extern uint16_t setPWM;
extern uint8_t pwmState;
extern int16_t compWindowOffset;
extern DAC_HandleTypeDef hdac;

extern uint8_t startUp;
extern uint8_t waitForCommutation;

uint8_t systickDivider = 0;
uint16_t oc5ValueOld = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan;
extern COMP_HandleTypeDef hcomp1;
extern COMP_HandleTypeDef hcomp2;
extern COMP_HandleTypeDef hcomp3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

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
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
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
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
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
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
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
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles ADC1 and ADC2 interrupts.
*/
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
* @brief This function handles USB low priority or CAN_RX0 interrupts.
*/
void USB_LP_CAN_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_LP_CAN_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN_RX0_IRQn 1 */
}

/**
* @brief This function handles TIM1 capture compare interrupt.
*/
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	if((__HAL_TIM_GET_FLAG(&htim3,TIM_FLAG_CC1)== SET) && waitForCommutation == 1)
	{
		commutationPattern(NEXT);
	}
	
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles COMP1, COMP2 and COMP3 interrupts through EXTI lines 21, 22 and 29.
*/
void COMP1_2_3_IRQHandler(void)
{
  /* USER CODE BEGIN COMP1_2_3_IRQn 0 */
if(__HAL_COMP_COMP1_EXTI_GET_FLAG() && waitForCommutation == 0)
{			
	HAL_COMP_Stop_IT(&hcomp1);			
	commutationTimerCounterArray[5] = __HAL_TIM_GetCounter(&htim3);
	commutationTimerCounterAverage = (commutationTimerCounterArray[0]+commutationTimerCounterArray[1]+commutationTimerCounterArray[2]+commutationTimerCounterArray[3]+commutationTimerCounterArray[4]+commutationTimerCounterArray[5])/12; // /12
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,commutationTimerCounterAverage+commutationTimerOffset);			
	__HAL_TIM_SetCounter(&htim3,0);
	for(uint8_t i=0; i<5; i++)
	{
		commutationTimerCounterArray[i] = commutationTimerCounterArray[i+1];
	}	
	waitForCommutation = 1;	
} 
else if(__HAL_COMP_COMP2_EXTI_GET_FLAG() && waitForCommutation == 0)
{		
	HAL_COMP_Stop_IT(&hcomp2);			
	commutationTimerCounterArray[5] = __HAL_TIM_GetCounter(&htim3);
	commutationTimerCounterAverage = (commutationTimerCounterArray[0]+commutationTimerCounterArray[1]+commutationTimerCounterArray[2]+commutationTimerCounterArray[3]+commutationTimerCounterArray[4]+commutationTimerCounterArray[5])/12; 
    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,commutationTimerCounterAverage+commutationTimerOffset);
	__HAL_TIM_SetCounter(&htim3,0);		
	for(uint8_t i=0; i<5; i++)
	{
		commutationTimerCounterArray[i] = commutationTimerCounterArray[i+1];
	}		
	waitForCommutation = 1;
}
/*
Once in this cycle we check if the speed is above or below a certain level (allow some hysteresis) to see if the comparator crossing-methode shouls change (pwm high or pwm low)
*/

else if(__HAL_COMP_COMP3_EXTI_GET_FLAG() && waitForCommutation == 0)
{		
	HAL_COMP_Stop_IT(&hcomp3);
	commutationTimerCounterArray[5] = __HAL_TIM_GetCounter(&htim3);	
	commutationTimerCounterAverage = (commutationTimerCounterArray[0]+commutationTimerCounterArray[1]+commutationTimerCounterArray[2]+commutationTimerCounterArray[3]+commutationTimerCounterArray[4]+commutationTimerCounterArray[5])/12; 
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,commutationTimerCounterAverage+commutationTimerOffset);		
	__HAL_TIM_SetCounter(&htim3,0);	
	for(uint8_t i=0; i<5; i++)
	{
		commutationTimerCounterArray[i] = commutationTimerCounterArray[i+1];
	}	
	waitForCommutation = 1;

	if(setPWM > 300 && pwmState == 0) //300 == 50 PWM
	{
	  GPIO_InitTypeDef GPIO_InitStruct;
	  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);	
		 
	  HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_5);
	  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,1900+oc5Value); //4095 would be max. (3.3 V) 1241 = 1.0 V (at 6V * 20/120) or 1200 with 4S LiPo (16V)
	  
	  TIM_OC_InitTypeDef sConfigOC5A;
	  sConfigOC5A.OCMode = TIM_OCMODE_PWM1;
	  compWindowOffset = -80;
	  sConfigOC5A.OCFastMode = TIM_OCFAST_DISABLE; //DISABLE
	  sConfigOC5A.OCIdleState = TIM_OCIDLESTATE_RESET;
	  sConfigOC5A.OCPolarity = TIM_OCPOLARITY_LOW; //LOW for PWM high detection
	  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC5A, TIM_CHANNEL_5);		
	  TIM1->CCR5 = setPWM + compWindowOffset;
	  pwmState = 1;
	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_5);	  
	}
	
	if(setPWM > 300 && pwmState == 1 && oc5Value != oc5ValueOld)
	{
		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,1900+oc5Value);
		oc5ValueOld = oc5Value;
	}

	if(setPWM <= 250 && pwmState == 1) //ca. 40 PWM
	{
	  GPIO_InitTypeDef GPIO_InitStruct;
	  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_5);
	  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0); //4095 would be max. (3.3 V) 1241 = 1.0 V (bei 6V * 20/120)
	  
	  TIM_OC_InitTypeDef sConfigOC5B;
	  sConfigOC5B.OCMode = TIM_OCMODE_PWM1;
	  compWindowOffset = 300;
	  sConfigOC5B.OCFastMode = TIM_OCFAST_DISABLE;
	  sConfigOC5B.OCIdleState = TIM_OCIDLESTATE_RESET;
	  sConfigOC5B.OCPolarity = TIM_OCPOLARITY_HIGH; //HIGH for PWM low detection
	  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC5B, TIM_CHANNEL_5);
	  TIM1->CCR5 = setPWM + compWindowOffset;
	  pwmState = 0;
	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_5);
	}
}

  /* USER CODE END COMP1_2_3_IRQn 0 */
  HAL_COMP_IRQHandler(&hcomp1);
  HAL_COMP_IRQHandler(&hcomp2);
  HAL_COMP_IRQHandler(&hcomp3);
  /* USER CODE BEGIN COMP1_2_3_IRQn 1 */

  /* USER CODE END COMP1_2_3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
