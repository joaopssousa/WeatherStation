/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : stm32f4xx_hal_msp.c
 * Description        : This file provides code for the MSP Initialization
 *                      and de-Initialization codes.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "hw.h"
#include "timeServer.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief ADC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (hadc->Instance == ADC1) {
		/* USER CODE BEGIN ADC1_MspInit 0 */

		/* USER CODE END ADC1_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_ADC1_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**ADC1 GPIO Configuration
		 PA0-WKUP     ------> ADC1_IN0
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_4;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		/* USER CODE BEGIN ADC1_MspInit 1 */

		/* USER CODE END ADC1_MspInit 1 */
	} else if (hadc->Instance == ADC2) {
		/* USER CODE BEGIN ADC2_MspInit 0 */

		/* USER CODE END ADC2_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_ADC2_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**ADC2 GPIO Configuration
		 PA7     ------> ADC2_IN7		// MIK
		 */
		GPIO_InitStruct.Pin = BIRUTA_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(BIRUTA_GPIO_Port, &GPIO_InitStruct);

		/* USER CODE BEGIN ADC2_MspInit 1 */

		/* USER CODE END ADC2_MspInit 1 */
	}

}

/**
 * @brief ADC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		/* USER CODE BEGIN ADC1_MspDeInit 0 */

		/* USER CODE END ADC1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_ADC1_CLK_DISABLE();

		/**ADC1 GPIO Configuration
		 PA0-WKUP     ------> ADC1_IN0
		 */
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0);

		/* USER CODE BEGIN ADC1_MspDeInit 1 */

		/* USER CODE END ADC1_MspDeInit 1 */
	} else if (hadc->Instance == ADC2) {
		/* USER CODE BEGIN ADC2_MspDeInit 0 */

		/* USER CODE END ADC2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_ADC2_CLK_DISABLE();

		/**ADC2 GPIO Configuration
		 PA7     ------> ADC2_IN7			// MIK
		 */
		HAL_GPIO_DeInit(BIRUTA_GPIO_Port, BIRUTA_Pin);		// MIK

		/* USER CODE BEGIN ADC2_MspDeInit 1 */

		/* USER CODE END ADC2_MspDeInit 1 */
	}

}

/**
 * @brief I2C MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (hi2c->Instance == I2C1) {
		/* USER CODE BEGIN I2C1_MspInit 0 */

		/* USER CODE END I2C1_MspInit 0 */

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**I2C1 GPIO Configuration
		 PB6     ------> I2C1_SCL
		 PB7     ------> I2C1_SDA
		 */
		//GPIO_InitStruct.Pin = SCL_BME280_Pin|SDA_BME280_Pin;
		GPIO_InitStruct.Pin = SCL_BME280_Pin | SDA_BME280_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_I2C1_CLK_ENABLE();
		/* USER CODE BEGIN I2C1_MspInit 1 */

		/* USER CODE END I2C1_MspInit 1 */
	}

}

/**
 * @brief I2C MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == I2C1) {
		/* USER CODE BEGIN I2C1_MspDeInit 0 */

		/* USER CODE END I2C1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_I2C1_CLK_DISABLE();

		/**I2C1 GPIO Configuration
		 PB6     ------> I2C1_SCL
		 PB7     ------> I2C1_SDA
		 */
		HAL_GPIO_DeInit(SCL_BME280_GPIO_Port, SCL_BME280_Pin);

		HAL_GPIO_DeInit(SDA_BME280_GPIO_Port, SDA_BME280_Pin);

		/* USER CODE BEGIN I2C1_MspDeInit 1 */

		/* USER CODE END I2C1_MspDeInit 1 */
	}

}

/**
 * @brief RTC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hrtc: RTC handle pointer
 * @retval None
 */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc) {
	if (hrtc->Instance == RTC) {
		/* USER CODE BEGIN RTC_MspDeInit 0 */

		/* USER CODE END RTC_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_RTC_DISABLE();
		/* USER CODE BEGIN RTC_MspDeInit 1 */

		/* USER CODE END RTC_MspDeInit 1 */
	}

}

/**
 * @brief SPI MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (hspi->Instance == SPI2) {
		/* USER CODE BEGIN SPI2_MspInit 0 */

		/* USER CODE END SPI2_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_SPI2_CLK_ENABLE();

		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**SPI2 GPIO Configuration
		 PC2     ------> SPI2_MISO
		 PC3     ------> SPI2_MOSI
		 PB13     ------> SPI2_SCK
		 */
		GPIO_InitStruct.Pin = LORA_MISO_Pin | LORA_MOSI_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = LORA_SCK_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
		HAL_GPIO_Init(LORA_SCK_GPIO_Port, &GPIO_InitStruct);

		/* USER CODE BEGIN SPI2_MspInit 1 */

		/* USER CODE END SPI2_MspInit 1 */
	}

}

/**
 * @brief SPI MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI2) {
		/* USER CODE BEGIN SPI2_MspDeInit 0 */

		/* USER CODE END SPI2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_SPI2_CLK_DISABLE();

		/**SPI2 GPIO Configuration
		 PC2     ------> SPI2_MISO
		 PC3     ------> SPI2_MOSI
		 PB13     ------> SPI2_SCK
		 */
		HAL_GPIO_DeInit(GPIOC, LORA_MISO_Pin | LORA_MOSI_Pin);

		HAL_GPIO_DeInit(LORA_SCK_GPIO_Port, LORA_SCK_Pin);

		/* USER CODE BEGIN SPI2_MspDeInit 1 */

		/* USER CODE END SPI2_MspDeInit 1 */
	}

}

/**
 * @brief TIM_OC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_oc: TIM_OC handle pointer
 * @retval None
 */
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim_oc) {
	if (htim_oc->Instance == TIM4) {
		/* USER CODE BEGIN TIM4_MspInit 0 */

		/* USER CODE END TIM4_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_TIM4_CLK_ENABLE();
		/* TIM4 interrupt Init */
		HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM4_IRQn);
		/* USER CODE BEGIN TIM4_MspInit 1 */

		/* USER CODE END TIM4_MspInit 1 */
	}

}

/**
 * @brief TIM_OC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_oc: TIM_OC handle pointer
 * @retval None
 */
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim_oc) {
	if (htim_oc->Instance == TIM4) {
		/* USER CODE BEGIN TIM4_MspDeInit 0 */

		/* USER CODE END TIM4_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM4_CLK_DISABLE();

		/* TIM4 interrupt DeInit */
		HAL_NVIC_DisableIRQ(TIM4_IRQn);
		/* USER CODE BEGIN TIM4_MspDeInit 1 */

		/* USER CODE END TIM4_MspDeInit 1 */
	}

}

/**
 * @brief This function configures the source of the time base.
 * @brief  don't enable systick
 * @param TickPriority: Tick interrupt priority.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
	/* Return function status */
	return HAL_OK;
}

/**
 * @brief This function provides delay (in ms)
 * @param Delay: specifies the delay time length, in milliseconds.
 * @retval None
 */
void HAL_Delay(__IO uint32_t Delay) {
	HW_RTC_DelayMs(Delay); /* based on RTC */
}

/**
 * @brief  Initializes the MSP.
 * @retval None
 */
void HAL_MspInit(void) {
	/* Enable Power Clock */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	/* Ensure that MSI is wake-up system clock */
	//__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);
	/* Configure all IOs in analog input              */
	/* Except PA143 and PA14 (SWCLK and SWD) for debug*/
	/* PA13 and PA14 are configured in debug_init     */
	HW_GpioInit();
}

/**
 * @brief RTC MSP Initialization
 *        This function configures the hardware resources used in this example:
 *           - Peripheral's clock enable
 * @param hrtc: RTC handle pointer
 * @note  Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to select
 *        the RTC clock source; in this case the Backup domain will be reset in
 *        order to modify the RTC Clock source, as consequence RTC registers (including
 *        the backup registers) and RCC_CSR register are set to their reset values.
 * @retval None
 */
void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc) {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct = {0};

	  __HAL_RCC_BKPSRAM_CLK_ENABLE();
	   // additional code for Backup RAM enable

	   HAL_PWR_EnableBkUpAccess();
	   // additional code for Backup RAM enable

	   HAL_PWREx_EnableBkUpReg();
	   // additional code for Backup RAM enable

	  /*##-1- Configue the RTC clock soucre ######################################*/
	  /* -a- Enable LSE Oscillator */
	  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /* -b- Select LSI as RTC clock source */
	  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV8;
	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /*##-2- Enable the RTC peripheral Clock ####################################*/
	  /* Enable RTC Clock */
	  __HAL_RCC_RTC_ENABLE();//RTCCLK

	  /*##-3- Configure the NVIC for RTC Alarm ###################################*/
	  HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0x0, 0);
	  HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
}

/**
 * @brief  Alarm A callback.
 * @param  hrtc: RTC handle
 * @retval None
 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
	TimerIrqHandler();
}

/**
 * @brief  EXTI line detection callbacks.
 * @param  GPIO_Pin: Specifies the pins connected to the EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	HW_GPIO_IrqHandler(GPIO_Pin);
}

/**
 * @brief  Gets IRQ number as a function of the GPIO_Pin.
 * @param  GPIO_Pin: Specifies the pins connected to the EXTI line.
 * @retval IRQ number
 */
IRQn_Type MSP_GetIRQn(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case GPIO_PIN_0:
		return EXTI0_IRQn;
	case GPIO_PIN_1:
		return EXTI1_IRQn;
	case GPIO_PIN_2:
		return EXTI2_IRQn;
	case GPIO_PIN_3:
		return EXTI3_IRQn;
	case GPIO_PIN_4:
		return EXTI4_IRQn;
	case GPIO_PIN_5:
	case GPIO_PIN_6:
	case GPIO_PIN_7:
	case GPIO_PIN_8:
	case GPIO_PIN_9:
		return EXTI9_5_IRQn;
	case GPIO_PIN_10:
	case GPIO_PIN_11:
	case GPIO_PIN_12:
	case GPIO_PIN_13:
	case GPIO_PIN_14:
	case GPIO_PIN_15:
	default:
		return EXTI15_10_IRQn;
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
