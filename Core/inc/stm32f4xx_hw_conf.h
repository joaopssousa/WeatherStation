/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: contains hardaware configuration Macros and Constants

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
/**
  ******************************************************************************
  * @file    stm32f4xx_hw_conf.h
  * @author  MCD Application Team
  * @brief   contains hardaware configuration Macros and Constants for stm32l4
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONF_F4_H__
#define __HW_CONF_F4_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

//#define RADIO_DIO_4
//#define RADIO_DIO_5

#include "main.h"

/* LORA I/O definition */

#ifdef USE_SX126X_DVK

#define RADIO_RESET_PORT                          GPIOA
#define RADIO_RESET_PIN                           GPIO_PIN_0

#define RADIO_MOSI_PORT                           GPIOA
#define RADIO_MOSI_PIN                            GPIO_PIN_7

#define RADIO_MISO_PORT                           GPIOA
#define RADIO_MISO_PIN                            GPIO_PIN_6

#define RADIO_SCLK_PORT                           GPIOA
#define RADIO_SCLK_PIN                            GPIO_PIN_5

#define RADIO_NSS_PORT                            GPIOA
#define RADIO_NSS_PIN                             GPIO_PIN_8

#define RADIO_BUSY_PORT                           GPIOB
#define RADIO_BUSY_PIN                            GPIO_PIN_3

#define RADIO_DIO_0_PORT                          GPIOA
#define RADIO_DIO_0_PIN                           GPIO_PIN_10

#define RADIO_DIO_1_PORT                          GPIOB
#define RADIO_DIO_1_PIN                           GPIO_PIN_4

#define RADIO_DIO_2_PORT                          GPIOB
#define RADIO_DIO_2_PIN                           GPIO_PIN_5

#define RADIO_DIO_3_PORT                          GPIOB
#define RADIO_DIO_3_PIN                           GPIO_PIN_4

#define RADIO_ANT_SWITCH_POWER_PORT               GPIOA
#define RADIO_ANT_SWITCH_POWER_PIN                GPIO_PIN_9

#define DEVICE_SEL_PORT                           GPIOA
#define DEVICE_SEL_PIN                            GPIO_PIN_4

#define RADIO_LEDTX_PORT                           GPIOC
#define RADIO_LEDTX_PIN                            GPIO_PIN_1

#define RADIO_LEDRX_PORT                           GPIOC
#define RADIO_LEDRX_PIN                            GPIO_PIN_0

#else


#define RADIO_RESET_PORT                          LORA_RESET_GPIO_Port
#define RADIO_RESET_PIN                           LORA_RESET_Pin

#define RADIO_MOSI_PORT                           LORA_MOSI_GPIO_Port
#define RADIO_MOSI_PIN                            LORA_MOSI_Pin

#define RADIO_MISO_PORT                           LORA_MISO_GPIO_Port
#define RADIO_MISO_PIN                            LORA_MISO_Pin

#define RADIO_SCLK_PORT                           LORA_SCK_GPIO_Port
#define RADIO_SCLK_PIN                            LORA_SCK_Pin

#define RADIO_NSS_PORT                            LORA_NSS_GPIO_Port
#define RADIO_NSS_PIN                             LORA_NSS_Pin

#define RADIO_DIO_0_PORT                          LORA_DIO0_GPIO_Port
#define RADIO_DIO_0_PIN                           LORA_DIO0_Pin

#define RADIO_DIO_1_PORT                          LORA_DIO1_GPIO_Port
#define RADIO_DIO_1_PIN                           LORA_DIO1_Pin

#define RADIO_DIO_2_PORT                          LORA_DIO2_GPIO_Port
#define RADIO_DIO_2_PIN                           LORA_DIO2_Pin

#define RADIO_DIO_3_PORT                          LORA_DIO3_GPIO_Port
#define RADIO_DIO_3_PIN                           LORA_DIO3_Pin

#ifdef RADIO_DIO_4
#define RADIO_DIO_4_PORT                          LORA_DIO4_GPIO_Port
#define RADIO_DIO_4_PIN                           LORA_DIO4_Pin
#endif

#ifdef RADIO_DIO_5
#define RADIO_DIO_5_PORT                          LORA_DIO5_GPIO_Port
#define RADIO_DIO_5_PIN                           LORA_DIO5_Pin
#endif



#define RADIO_ANT_SWITCH_PORT                     GPIOC
#define RADIO_ANT_SWITCH_PIN                      GPIO_PIN_1

#endif

/*  SPI MACRO redefinition */
#define SPI_RADIO						SPI2

#define SPI_CLK_ENABLE()                __HAL_RCC_SPI2_CLK_ENABLE()

#define SPI2_AF                          GPIO_AF5_SPI2

/* ADC MACRO redefinition */

#define ADC_READ_CHANNEL                 ADC_CHANNEL_1
#define ADCCLK_ENABLE()                 __HAL_RCC_ADC1_CLK_ENABLE() ;
#define ADCCLK_DISABLE()                __HAL_RCC_ADC1_CLK_DISABLE() ;

/* --------------------------- RTC HW definition -------------------------------- */

#define RTC_OUTPUT       DBG_RTC_OUTPUT

/* --------------------------- USART HW definition -------------------------------*/


#define USARTx                           USART3
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART3_CLK_ENABLE();//__USART3_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __USART3_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART3_RELEASE_RESET()


#define USARTx_TX_PIN                  DEBUG_TX_Pin
#define USARTx_TX_GPIO_PORT            DEBUG_TX_GPIO_Port
#define USARTx_TX_AF                   GPIO_AF7_USART3
#define USARTx_RX_PIN                  DEBUG_RX_Pin
#define USARTx_RX_GPIO_PORT            DEBUG_RX_GPIO_Port
#define USARTx_RX_AF                   GPIO_AF7_USART3

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART3_IRQn
#define USARTx_IRQHandler                USART3_IRQHandler

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL             DMA1_Stream4

/* Definition for USARTx's DMA Request */
#define USARTx_TX_DMA_REQUEST             DMA_CHANNEL_7

/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA1_Stream4_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA1_Stream4_IRQHandler

#define USARTx_Priority 0
#define USARTx_DMA_Priority 0



/* --------------------------- DEBUG redefinition -------------------------------*/

#define __HAL_RCC_DBGMCU_CLK_ENABLE()
#define __HAL_RCC_DBGMCU_CLK_DISABLE()

#define LED_Toggle( x )
#define LED_On( x )
#define LED_Off( x )

#ifdef __cplusplus
}
#endif

#endif /* __HW_CONF_F4_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
