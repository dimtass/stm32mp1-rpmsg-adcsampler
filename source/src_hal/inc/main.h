/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    OpenAMP/OpenAMP_TTY_echo/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
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
#include "stm32mp1xx_hal.h"
#include "stm32mp15xx_disco_stpmic1.h"
#include "openamp.h"
#include "lock_resource.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "openamp_log.h"
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
void Error_Handler(char * file, int line);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DEFAULT_IRQ_PRIO      1U

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
/* User can use this section to tailor ADCx instance under use and associated
   resources */

/* ## Definition of ADC related resources ################################### */
/* Definition of ADCx clock resources */
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC12_CLK_ENABLE()
#define ADCx_FORCE_RESET()              __HAL_RCC_ADC12_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC12_RELEASE_RESET()

/* Definition of ADCx channels 
* A0 : PF14 -> ADC2_IN6
* A1 : PF13 -> ADC2_IN2
* A2 : ANA0 -> ADC1_IN0
* A3 : ANA1 -> ADC1_IN1
* A4 : PC3  -> ADC1_IN13
*/

/* Definition of ADCx NVIC resources */

/* Definition of ADCx channels pins */
#define ADCx_CHANNEL_AC_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOF_CLK_ENABLE()
#define ADCx_CHANNEL_CT1_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()
#define ADCx_CHANNEL_CT2_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define ADCx_CHANNEL_CT3_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define ADCx_CHANNEL_CT4_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()
/* Definition of ADCx DMA resources */
#define ADCx_DMA_CLK_ENABLE()           __HAL_RCC_DMA2_CLK_ENABLE()
#define ADCx_DMAMUX_CLK_ENABLE()        __HAL_RCC_DMAMUX_CLK_ENABLE()

enum en_emontx_channel {
  EMONTX_CH_AC,
  EMONTX_CH_CT1,
  EMONTX_CH_CT2,
  EMONTX_CH_CT3,
  EMONTX_CH_CT4,
  EMONTX_CH_NUM = EMONTX_CH_CT4
};

#define NUMBER_OF_ADCS 2

struct adc_dev_t {
  ADC_TypeDef         *adc;
  uint8_t             adc_irqn;
  void (*adc_irq_handler)(void);
  DMA_Stream_TypeDef  *dma_stream;
  uint8_t             dma_stream_irqn;
  void (*stream_irq_handler)(void);
};

struct adc_channel_t {
  ADC_TypeDef         *adc;
  uint32_t            channel;
  GPIO_TypeDef        *port;
  uint16_t            pin;
};

extern struct adc_channel_t adc_channels[EMONTX_CH_NUM];
extern struct adc_dev_t adc_dev[NUMBER_OF_ADCS];

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
