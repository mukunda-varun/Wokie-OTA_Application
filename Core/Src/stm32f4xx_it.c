/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_it.h"
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
extern volatile uint8_t gu8_State;
extern volatile uint8_t no_tooth ;
extern volatile uint32_t gu32_T1 ;
extern volatile uint32_t gu32_T2 ;
extern volatile uint32_t gu32_Ticks;
extern volatile uint32_t gu16_TIM2_OVC;
extern volatile float gu32_Freq;
extern volatile float speed_ic ;

extern uint8_t startProcess, abortProcess;

extern uint8_t displayValueHex;
count segmentCounterVar;
uint32_t debugCnt;
uint32_t androidDebugMessageCnt;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_adc1;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim2;

/* USER CODE BEGIN EV */
extern void Send_Android_Debug_Packet(void);
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Pre-fetch fault, memory access fault.
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
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

	/*if(HAL_GPIO_ReadPin(INDUCTION_SPARE_GPIO_Port,INDUCTION_SPARE_Pin)==GPIO_PIN_SET)
	{
	   HAL_GPIO_WritePin(INDUCTION_K1_GPIO_Port,INDUCTION_K1_Pin, RESET);
	}
	else
	{   HAL_GPIO_WritePin(INDUCTION_K1_GPIO_Port,INDUCTION_K1_Pin, SET);

	}*/
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
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
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
//  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if interrupt source is channel 1
//{
//	if(gu8_State == IC_IDLE)
//	{
//		gu32_T1 = TIM3->CCR1;
//		gu16_TIM2_OVC = 0;
//		gu8_State = IC_DONE;
//	}
//	else if(gu8_State == IC_DONE)
//	{
//		gu32_T2 = TIM3->CCR1;
//		gu32_Ticks = (gu32_T2 + (gu16_TIM2_OVC * (0xFFFF + 1))) - gu32_T1;
////		tick_arr[cnt] = gu32_Ticks;
////		cnt++;
////		if(cnt >= 20)
////		{
////			cnt = 0;
////		}
//		gu8_State = IC_IDLE;
//	}
//}

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_DAC_IRQHandler(&hdac);
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
  if(debugCnt++ >= 200)
  {
	  HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
	  debugCnt = 0;
  }

#if BS84C12A_DRIVE_ENABLE == 1
	if(HAL_GPIO_ReadPin(A_SEG_GPIO_Port, A_SEG_Pin) == (uint8_t)RESET)
		{
			segmentCounterVar.segmentACnt++;
			if(segmentCounterVar.segmentACnt >= 40)
			{
				displayValueHex &= ~(1 << 0);
				segmentCounterVar.segmentACnt = 0;
			}
		}
		else
		{
			displayValueHex |= (1 << 0);
			segmentCounterVar.segmentACnt = 0;
		}
		if(HAL_GPIO_ReadPin(B_SEG_GPIO_Port, B_SEG_Pin) == (uint8_t)RESET)
		{
			segmentCounterVar.segmentBCnt++;
			if(segmentCounterVar.segmentBCnt >= 40)
			{
				displayValueHex &= ~(1 << 1);
				segmentCounterVar.segmentBCnt = 0;
			}
		}
		else
		{
			segmentCounterVar.segmentBCnt = 0;
			displayValueHex |= (1 << 1);
		}
		if(HAL_GPIO_ReadPin(C_SEG_GPIO_Port, C_SEG_Pin) == (uint8_t)RESET)
		{
			segmentCounterVar.segmentCCnt++;
			if(segmentCounterVar.segmentCCnt >= 40)
			{
				displayValueHex &= ~(1 << 2);
				segmentCounterVar.segmentCCnt = 0;
			}
		}
		else
		{
			segmentCounterVar.segmentCCnt = 0;
			displayValueHex |= (1 << 2);
		}
		if(HAL_GPIO_ReadPin(D_SEG_GPIO_Port, D_SEG_Pin) == (uint8_t)RESET)
		{
			segmentCounterVar.segmentDtempCnt++;
			if(segmentCounterVar.segmentDtempCnt >= 40)
			{
				segmentCounterVar.segmentDtempvalue=0;
				segmentCounterVar.segmentDtempCnt = 0;
			}
		}
		else
		{
			segmentCounterVar.segmentDtempCnt = 0;
			segmentCounterVar.segmentDtempvalue=1;
		}



		if(segmentCounterVar.segmentDtempvalue)
		{
			segmentCounterVar.segmentDCnt++;
			if(segmentCounterVar.segmentDCnt >= 400)
			{

				displayValueHex |= (1 << 3);
				segmentCounterVar.segmentDCnt = 0;
			}
		}
		else
		{
			segmentCounterVar.segmentDCnt = 0;
			displayValueHex &= ~(1 << 3);
		}

		if(HAL_GPIO_ReadPin(E_SEG_GPIO_Port, E_SEG_Pin) == (uint8_t)RESET)
		{
			segmentCounterVar.segmentECnt++;
			if(segmentCounterVar.segmentECnt >= 40)
			{
				displayValueHex &= ~(1 << 4);
				segmentCounterVar.segmentECnt = 0;
			}
		}
		else
		{
			segmentCounterVar.segmentECnt = 0;
			displayValueHex |= (1 << 4);
		}
		if(HAL_GPIO_ReadPin(F_SEG_GPIO_Port, F_SEG_Pin) == (uint8_t)RESET)
		{
			segmentCounterVar.segmentFCnt++;
			if(segmentCounterVar.segmentFCnt >= 40)
			{
				displayValueHex &= ~(1 << 5);
				segmentCounterVar.segmentFCnt = 0;
			}
		}
		else
		{
			segmentCounterVar.segmentFCnt = 0;
			displayValueHex |= (1 << 5);
		}
		if(HAL_GPIO_ReadPin(G_SEG_GPIO_Port, G_SEG_Pin) == (uint8_t)RESET)
		{
			segmentCounterVar.segmentGCnt++;
			if(segmentCounterVar.segmentGCnt >= 40)
			{
				displayValueHex &= ~(1 << 6);
				segmentCounterVar.segmentGCnt = 0;
			}
		}
		else
		{
			segmentCounterVar.segmentGCnt = 0;
			displayValueHex |= (1 << 6);
		}
		displayValueHex &= ~(1 << 7);
	#if 0
	if(HAL_GPIO_ReadPin(A_SEG_GPIO_Port, A_SEG_Pin) != (uint8_t)RESET)
	{
		segmentCounterVar.segmentACnt++;
		if(segmentCounterVar.segmentACnt >= 5)
		{
			displayValueHex |= (1 << 0);
			segmentCounterVar.segmentACnt = 0;
		}
	}
	else
	{
		displayValueHex &= ~(1 << 0);
		segmentCounterVar.segmentACnt = 0;
	}
	if(HAL_GPIO_ReadPin(B_SEG_GPIO_Port, B_SEG_Pin) != (uint8_t)RESET)
	{
		segmentCounterVar.segmentBCnt++;
		if(segmentCounterVar.segmentBCnt >= 5)
		{
			displayValueHex |= (1 << 1);
			segmentCounterVar.segmentBCnt = 0;
		}
	}
	else
	{
		segmentCounterVar.segmentBCnt = 0;
		displayValueHex &= ~(1 << 1);
	}
	if(HAL_GPIO_ReadPin(C_SEG_GPIO_Port, C_SEG_Pin) != (uint8_t)RESET)
	{
		segmentCounterVar.segmentCCnt++;
		if(segmentCounterVar.segmentCCnt >= 5)
		{
			displayValueHex |= (1 << 2);
			segmentCounterVar.segmentCCnt = 0;
		}
	}
	else
	{
		segmentCounterVar.segmentCCnt = 0;
		displayValueHex &= ~(1 << 2);
	}
	/*if(HAL_GPIO_ReadPin(D_SEG_GPIO_Port, D_SEG_Pin) != (uint8_t)RESET)
	{
		segmentCounterVar.segmentDCnt++;
		if(segmentCounterVar.segmentDCnt >= 5)
		{
			displayValueHex |= (1 << 3);
			segmentCounterVar.segmentDCnt = 0;
		}
	}
	else
	{
		segmentCounterVar.segmentDCnt = 0;
		displayValueHex &= ~(1 << 3);
	}*/
	if(HAL_GPIO_ReadPin(E_SEG_GPIO_Port, E_SEG_Pin) != (uint8_t)RESET)
	{
		segmentCounterVar.segmentECnt++;
		if(segmentCounterVar.segmentECnt >= 5)
		{
			displayValueHex |= (1 << 4);
			segmentCounterVar.segmentECnt = 0;
		}
	}
	else
	{
		segmentCounterVar.segmentECnt = 0;
		displayValueHex &= ~(1 << 4);
	}
	if(HAL_GPIO_ReadPin(F_SEG_GPIO_Port, F_SEG_Pin) != (uint8_t)RESET)
	{
		segmentCounterVar.segmentFCnt++;
		if(segmentCounterVar.segmentFCnt >= 5)
		{
			displayValueHex |= (1 << 5);
			segmentCounterVar.segmentFCnt = 0;
		}
	}
	else
	{
		segmentCounterVar.segmentFCnt = 0;
		displayValueHex &= ~(1 << 5);
	}
	if(HAL_GPIO_ReadPin(G_SEG_GPIO_Port, G_SEG_Pin) != (uint8_t)RESET)
	{
		segmentCounterVar.segmentGCnt++;
		if(segmentCounterVar.segmentGCnt >= 5)
		{
			displayValueHex |= (1 << 6);
			segmentCounterVar.segmentGCnt = 0;
		}
	}
	else
	{
		segmentCounterVar.segmentGCnt = 0;
		displayValueHex &= ~(1 << 6);
	}
	displayValueHex &= ~(1 << 7);
	#endif
#endif
  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
