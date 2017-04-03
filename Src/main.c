/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#define NEXT 9
#define STEP_0 0
#define STEP_1 1
#define STEP_2 2
#define STEP_3 3
#define STEP_4 4
#define STEP_5 5
#define	I2CRECEIVEDBUFFERSIZE 8
#define COMPDELAY for(uint32_t i = 0; i < 250; i++) //170 with 3S PWM low
#define CANMOTORADDRESS 0x233
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;
COMP_HandleTypeDef hcomp3;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile int commutationTimerCounter = 2000;
volatile uint16_t commutationTimerCounterArray[6] = { 20000, 20000, 2000, 20000, 20000, 20000 };
volatile uint8_t commutationStepCounter;
volatile uint8_t waitForCommutation = 1;
volatile uint16_t myADCvalue;
volatile uint16_t commutationTimerCounterAverage;
volatile uint16_t counterTIM3Values;
volatile int16_t commutationTimerOffset;

volatile uint8_t rx_bytes_counter;
volatile uint8_t command_ready;
volatile uint8_t data_ready;
volatile uint8_t rx_bytes[10];
volatile uint8_t receiveBuffer[I2CRECEIVEDBUFFERSIZE];
volatile int16_t newPWM = 100;
volatile int16_t setPWM = 100;
volatile int16_t compWindowOffset = 300; //this opens a comparator window to block overshooting signals from triggering (adjust to Vbat)
volatile uint16_t oc5Value = 128;  //preset offset-value (for fine tuning) of the DAC voltage output as the reference voltage for the compartors (about 0V or Vbat/2)
volatile uint8_t pwmState = 0; //indicates whether a zero crossing (pwm low) or a Vbat/2 crossing (pwm high) will be detected; 0 = PWM low detection, 1 = PWM high detection
volatile uint8_t bTransferRequest = 0;
volatile uint8_t motorGotStarted = 0;

static CanRxMsgTypeDef myRxMessage;

TIM_OC_InitTypeDef sConfigOC;
GPIO_InitTypeDef GPIO_InitStructCom;

void commutationPattern(uint8_t step);
void commutateNow_0(void);
void commutateNow_1(void);
void commutateNow_2(void);
void commutateNow_3(void);
void commutateNow_4(void);
void commutateNow_5(void);
void startMotor(void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
static void MX_COMP3_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void commutationPattern(uint8_t step) {
	//too fast changes of the speed / pwm setting can cause out of sync. so allow only max pwm steps per cycle
	if (newPWM < (setPWM - 2)) {
		setPWM -= 1;
		TIM1->CCR1 = setPWM;
		TIM1->CCR5 = setPWM + compWindowOffset;
	} else if (newPWM > (setPWM + 2)) {
		setPWM += 1;
		TIM1->CCR1 = setPWM;
		TIM1->CCR5 = setPWM + compWindowOffset;
	} else {
		setPWM = newPWM;
	}
	//waitForCommutation means that a comparator event has been detected the commutation can happen after the 30deg delay (timer 3)
	if (step == NEXT && waitForCommutation == 1) {
		if (commutationStepCounter < STEP_5)
			commutationStepCounter++;
		else
			commutationStepCounter = STEP_0;

		switch (commutationStepCounter) {
		case STEP_0:
			commutateNow_0();
			break;
		case STEP_1:
			commutateNow_1();
			break;
		case STEP_2:
			commutateNow_2();
			break;
		case STEP_3:
			commutateNow_3();
			break;
		case STEP_4:
			commutateNow_4();
			break;
		case STEP_5:
			commutateNow_5();
			break;
		}
	} else {
		waitForCommutation = 0;
		switch (step) {
		case STEP_0:
			commutateNow_0();
			break;
		case STEP_1:
			commutateNow_1();
			break;
		case STEP_2:
			commutateNow_2();
			break;
		case STEP_3:
			commutateNow_3();
			break;
		case STEP_4:
			commutateNow_4();
			break;
		case STEP_5:
			commutateNow_5();
			break;
		}
	}
}

void commutateNow_0() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); //set PB11 to 0 to not inhibit phase B; float
	GPIO_InitStructCom.Pin = GPIO_PIN_2;			//set PB2 to output to allow GND on phase C
	GPIO_InitStructCom.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructCom.Pull = GPIO_PULLDOWN;
	GPIO_InitStructCom.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructCom);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); //set PB2 to 0 to set phase C to GND
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //set PB12 to 1 to inhibit phase C
	COMPDELAY
	;
	waitForCommutation = 0;
	HAL_COMP_Start_IT(&hcomp2);
}

void commutateNow_1() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); //set PB10 to 0 to not inhibit phase A; float
	GPIO_InitStructCom.Pin = GPIO_PIN_1;			//set PB1 to input to allow PWM on phase B
	GPIO_InitStructCom.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructCom.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructCom);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); //set PB11 to 1 to inhibit phase B
	COMPDELAY
	;
	waitForCommutation = 0;
	HAL_COMP_Start_IT(&hcomp1);
}

void commutateNow_2() {
	GPIO_InitStructCom.Pin = GPIO_PIN_0;			//set PB0 to output to allow GND on phase A
	GPIO_InitStructCom.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructCom.Pull = GPIO_PULLDOWN;
	GPIO_InitStructCom.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructCom);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //set PB0 to 0 to set phase A to GND
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); //set PB10 to 1 to inhibit phase A
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //set PB12 to 0 to not inhibit phase C; float
	COMPDELAY
	;
	waitForCommutation = 0;
	HAL_COMP_Start_IT(&hcomp3);
}

void commutateNow_3() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); //set PB11 to 0 to not inhibit phase B; float
	GPIO_InitStructCom.Pin = GPIO_PIN_2;			//set PB2 to input to allow PWM on phase C
	GPIO_InitStructCom.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructCom.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructCom);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //set PB12 to 1 to inhibit phase C
	COMPDELAY
	;
	waitForCommutation = 0;
	HAL_COMP_Start_IT(&hcomp2);
}

void commutateNow_4() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); //set PB10 to 0 to not inhibit phase A; float
	GPIO_InitStructCom.Pin = GPIO_PIN_1;			//set PB1 to output to allow GND on phase B
	GPIO_InitStructCom.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructCom.Pull = GPIO_PULLDOWN;
	GPIO_InitStructCom.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructCom);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); //set PB1 to 0 to set phase B to GND
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); //set PB11 to 1 to inhibit phase B
	COMPDELAY
	;
	waitForCommutation = 0;
	HAL_COMP_Start_IT(&hcomp1);
}

void commutateNow_5() {
	GPIO_InitStructCom.Pin = GPIO_PIN_0;			//set PB0 to input to allow PWM on phase A
	GPIO_InitStructCom.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructCom.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructCom);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); //set PB10 to 1 to inhibit phase A
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //set PB12 to 0 to not inhibit phase C; float
	COMPDELAY
	;
	waitForCommutation = 0;
	HAL_COMP_Start_IT(&hcomp3);
}

/*
The CAN bus receives commands and data from the flight management system:
Typically the protocol looks like e.g. this: #pwm2/lsb msb (8 bytes)
The command parser selects the command and the associated data
*/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan) {
	if ((hcan->pRxMsg->StdId == CANMOTORADDRESS) && (hcan->pRxMsg->IDE == CAN_ID_STD) && (hcan->pRxMsg->DLC == 8)) {
		rx_bytes[0] = hcan->pRxMsg->Data[0];
		rx_bytes[1] = hcan->pRxMsg->Data[1];
		rx_bytes[2] = hcan->pRxMsg->Data[2];
		rx_bytes[3] = hcan->pRxMsg->Data[3];
		rx_bytes[4] = hcan->pRxMsg->Data[4];
		rx_bytes[5] = hcan->pRxMsg->Data[5];
		rx_bytes[6] = hcan->pRxMsg->Data[6];
		rx_bytes[7] = hcan->pRxMsg->Data[7];

		rx_bytes_counter = 0;

		for (uint8_t i = 0; i < I2CRECEIVEDBUFFERSIZE; i++) {
			//rx_bytes[rx_bytes_counter] = receiveBuffer[rx_bytes_counter];
			rx_bytes_counter++;

			if (rx_bytes[rx_bytes_counter - 1] == '#') {
				rx_bytes_counter = 1;
				command_ready = 0;
				data_ready = 0;
			}

			if (command_ready == 1) {
				if ((rx_bytes[4] == '1') && (rx_bytes_counter == 7))
					data_ready = 1;
				if ((rx_bytes[4] == '2') && (rx_bytes_counter == 8))
					data_ready = 1;
			}

			if (rx_bytes[rx_bytes_counter - 1] == '/')
				command_ready = 1;

			if (data_ready == 1) {
				if ((rx_bytes[1] == 'p') && (rx_bytes[2] == 'w') && (rx_bytes[3] == 'm')) {  //speed
					if (motorGotStarted == 2) {
						newPWM = (rx_bytes[6] + rx_bytes[7] * 256) / 16;
					}
					command_ready = 0;
					data_ready = 0;
					rx_bytes_counter = 0;
				}

				if ((rx_bytes[1] == 'o') && (rx_bytes[2] == 'c') && (rx_bytes[3] == 'o')) {  //adjust oc5value
					oc5Value = rx_bytes[6] * 1;
					command_ready = 0;
					data_ready = 0;
					rx_bytes_counter = 0;
				}

				if ((rx_bytes[1] == 's') && (rx_bytes[2] == 't') && (rx_bytes[3] == 'a')) {  //start the motor
					newPWM = setPWM = 100;
					motorGotStarted = 1;
					command_ready = 0;
					data_ready = 0;
					rx_bytes_counter = 0;
				}

				if ((rx_bytes[1] == 's') && (rx_bytes[2] == 't') && (rx_bytes[3] == 'o')) {  //stop the motor
					newPWM = setPWM = 0;
					TIM1->CCR1 = setPWM;
					TIM1->CCR5 = setPWM + compWindowOffset;
					motorGotStarted = 0;
					command_ready = 0;
					data_ready = 0;
					rx_bytes_counter = 0;
				}
			}
		}
	}
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);

	/* Receive */
	if (HAL_CAN_Receive_IT(hcan, CAN_FIFO0) != HAL_OK) {
		/* Reception Error */
		Error_Handler();
	}
}

void CAN_Config(void) {
	CAN_FilterConfTypeDef sFilterConfig;

	//#### Configure the CAN Filter ###########################################*/
	sFilterConfig.FilterNumber = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = CANMOTORADDRESS;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}
}

/*
This forces the motor to turn; adjust to your own start up procedure (needs some patience ;-)).
Not the coolest way to do this. Suggestions?
*/
void startMotor(void) {
	newPWM = setPWM = 250;
	compWindowOffset = 300;
	TIM1->CCR1 = setPWM;
	TIM1->CCR5 = setPWM + compWindowOffset;

	commutateNow_0();
	HAL_Delay(150);
	commutationStepCounter = STEP_0;
	waitForCommutation = 1;

	for (uint16_t i = 0; i < 1; i++) {
		commutationPattern(NEXT);
		waitForCommutation = 1;
		HAL_Delay(150);
	}
	HAL_Delay(150);
	motorGotStarted = 2;

}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_COMP1_Init();
  MX_COMP2_Init();
  MX_COMP3_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_CAN_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  //does not to be IT
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_5);
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0); //4095 would be max. (3.3 V)
	newPWM = setPWM = TIM1->CCR1 = TIM1->CCR5 = 0;
	CAN_Config();
	hcan.pRxMsg = &myRxMessage;
	HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
	motorGotStarted = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if (motorGotStarted == 1) {
			startMotor();
		}
	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_CC4;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the ADC multi-mode 
    */
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_2TQ;
  hcan.Init.BS1 = CAN_BS1_2TQ;
  hcan.Init.BS2 = CAN_BS2_2TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

}

/* COMP1 init function */
static void MX_COMP1_Init(void)
{

  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH1;
  hcomp1.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp1.Init.Output = COMP_OUTPUT_NONE;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRCE_TIM1OC5;
  hcomp1.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* COMP2 init function */
static void MX_COMP2_Init(void)
{

  hcomp2.Instance = COMP2;
  hcomp2.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH1;
  hcomp2.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO2;
  hcomp2.Init.Output = COMP_OUTPUT_NONE;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRCE_TIM1OC5;
  hcomp2.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* COMP3 init function */
static void MX_COMP3_Init(void)
{

  hcomp3.Instance = COMP3;
  hcomp3.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH1;
  hcomp3.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp3.Init.Output = COMP_OUTPUT_NONE;
  hcomp3.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp3.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp3.Init.BlankingSrce = COMP_BLANKINGSRCE_TIM1OC5;
  hcomp3.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp3.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp3.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
  if (HAL_COMP_Init(&hcomp3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim1.Init.Period = 1600;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.Pulse = 120;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_5) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
