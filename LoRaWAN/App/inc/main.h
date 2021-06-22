/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "app_sfu.h"
#include "app_hw.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_BUTTON_Pin GPIO_PIN_3
#define USER_BUTTON_GPIO_Port GPIOE
#define LORA_MISO_Pin GPIO_PIN_2
#define LORA_MISO_GPIO_Port GPIOC
#define LORA_MOSI_Pin GPIO_PIN_3
#define LORA_MOSI_GPIO_Port GPIOC
#define RFID_TX_Pin GPIO_PIN_2
#define RFID_TX_GPIO_Port GPIOA
#define RFID_RX_Pin GPIO_PIN_3
#define RFID_RX_GPIO_Port GPIOA
#define LORA_DIO5_Pin GPIO_PIN_5
#define LORA_DIO5_GPIO_Port GPIOC
#define LORA_RESET_Pin GPIO_PIN_0
#define LORA_RESET_GPIO_Port GPIOB
#define LORA_DIO2_Pin GPIO_PIN_7
#define LORA_DIO2_GPIO_Port GPIOE
#define LORA_DIO1_Pin GPIO_PIN_8
#define LORA_DIO1_GPIO_Port GPIOE
#define LORA_DIO0_Pin GPIO_PIN_9
#define LORA_DIO0_GPIO_Port GPIOE
#define LORA_DIO4_Pin GPIO_PIN_10
#define LORA_DIO4_GPIO_Port GPIOE
#define LORA_DIO3_Pin GPIO_PIN_11
#define LORA_DIO3_GPIO_Port GPIOE
#define DEBUG_TX_Pin GPIO_PIN_10
#define DEBUG_TX_GPIO_Port GPIOB
#define DEBUG_RX_Pin GPIO_PIN_11
#define DEBUG_RX_GPIO_Port GPIOB
#define LORA_NSS_Pin GPIO_PIN_1
#define LORA_NSS_GPIO_Port GPIOB
#define LORA_SCK_Pin GPIO_PIN_13
#define LORA_SCK_GPIO_Port GPIOB
//#define MEM_SCL_Pin GPIO_PIN_6
//#define MEM_SCL_GPIO_Port GPIOB
//#define MEM_SDA_Pin GPIO_PIN_7
//#define MEM_SDA_GPIO_Port GPIOB
#define MEM_WP_Pin GPIO_PIN_8
#define MEM_WP_GPIO_Port GPIOB

#define BIRUTA_Pin GPIO_PIN_7
#define BIRUTA_GPIO_Port GPIOA

#define ANEMOMETRO_Pin GPIO_PIN_14
#define ANEMOMETRO_GPIO_Port GPIOE
#define ANEMOMETRO_EXTI_IRQn EXTI0_IRQn

#define PLUVIOMETRO_Pin GPIO_PIN_15
#define PLUVIOMETRO_GPIO_Port GPIOE
#define PLUVIOMETRO_EXTI_IRQn EXTI2_IRQn

#define SCL_BME280_Pin GPIO_PIN_6
#define SCL_BME280_GPIO_Port GPIOB

#define SDA_BME280_Pin GPIO_PIN_7
#define SDA_BME280_GPIO_Port GPIOB

#define LED_Pin GPIO_PIN_5			// Depois substituir por SPI3
#define LED_GPIO_Port GPIOB

#define BLE_BRK_Pin GPIO_PIN_8
#define BLE_BRK_GPIO_Port GPIOA
#define BLE_TX_Pin GPIO_PIN_9
#define BLE_TX_GPIO_Port GPIOA
#define BLE_RX_Pin GPIO_PIN_10
#define BLE_RX_GPIO_Port GPIOA
#define BLE_STATE_Pin GPIO_PIN_11
#define BLE_STATE_GPIO_Port GPIOA

#define SD_DET_CARD_Pin GPIO_PIN_7
#define SD_DET_CARD_GPIO_Port GPIOC



/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
