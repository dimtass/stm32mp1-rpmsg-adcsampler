/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    OpenAMP/OpenAMP_TTY_echo/Src/stm32mp1xx_hal_msp.c
  * @author  MCD Application Team
  * @brief   This file provides code for the MSP Initialization 
  *          and de-Initialization codes.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
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
static void DMA_Init_Channel(ADC_HandleTypeDef *hadc, uint8_t channel);
/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief IPCC MSP Initialization
* This function configures the hardware resources used in this example
* @param hipcc: IPCC handle pointer
* @retval None
*/
void HAL_IPCC_MspInit(IPCC_HandleTypeDef* hipcc)
{

  if(hipcc->Instance==IPCC)
  {
  /* USER CODE BEGIN IPCC_MspInit 0 */

  /* USER CODE END IPCC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_IPCC_CLK_ENABLE();
  /* IPCC interrupt Init */
    HAL_NVIC_SetPriority(IPCC_RX1_IRQn, DEFAULT_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(IPCC_RX1_IRQn);
  /* USER CODE BEGIN IPCC_MspInit 1 */

  /* USER CODE END IPCC_MspInit 1 */
}

}

/**
* @brief IPCC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hipcc: IPCC handle pointer
* @retval None
*/

void HAL_IPCC_MspDeInit(IPCC_HandleTypeDef* hipcc)
{

  if(hipcc->Instance==IPCC)
  {
  /* USER CODE BEGIN IPCC_MspDeInit 0 */

  /* USER CODE END IPCC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_IPCC_CLK_DISABLE();

    /* IPCC interrupt DeInit */
    HAL_NVIC_DisableIRQ(IPCC_RX1_IRQn);
  /* USER CODE BEGIN IPCC_MspDeInit 1 */

  /* USER CODE END IPCC_MspDeInit 1 */
  }

}


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */

    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, DEFAULT_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */

    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }

}

/**
  * @brief ADC MSP initialization
  *        This function configures the hardware resources used in this example:
  *          - Enable clock of ADC peripheral
  *          - Configure the GPIO associated to the peripheral channels
  *          - Configure the DMA associated to the peripheral
  *          - Configure the NVIC associated to the peripheral interruptions
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  RCC_PeriphCLKInitTypeDef  PeriphClkInit;
  
  log_info("HAL_ADC_MspInit\n");

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable clock of GPIO associated to the peripheral channels */
  ADCx_CHANNEL_AC_GPIO_CLK_ENABLE();
  ADCx_CHANNEL_CT1_GPIO_CLK_ENABLE();
  ADCx_CHANNEL_CT2_GPIO_CLK_ENABLE();
  ADCx_CHANNEL_CT3_GPIO_CLK_ENABLE();
  ADCx_CHANNEL_CT4_GPIO_CLK_ENABLE();
  /* Enable clock of ADCx peripheral (core clock) */
  ADCx_CLK_ENABLE();

  
  /* Configure ADCx clock prescaler */
  /* Caution: On STM32MP15, ADC clock frequency max is 36MHz (refer to device   */
  /*          datasheet).                                                     */
  /*          Therefore, ADC clock prescaler must be configured in function   */
  /*          of ADC clock source frequency to remain below this maximum      */
  /*          frequency.                                                      */
  if(IS_ENGINEERING_BOOT_MODE())
  {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PER;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
  }

  /* Enable clock of DMA associated to the peripheral */
  ADCx_DMA_CLK_ENABLE();
  ADCx_DMAMUX_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  // GPIO_InitTypeDef          GPIO_InitStruct;
  // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // for (int i=0; i<EMONTX_CH_NUM; i++) {
  //   if (adc_channels[i].adc == hadc->Instance) {
  //     PERIPH_LOCK(adc_channels[i].port);
  //     GPIO_InitStruct.Pin = adc_channels[i].pin;
  //     HAL_GPIO_Init(adc_channels[i].port, &GPIO_InitStruct);
  //     PERIPH_UNLOCK(adc_channels[i].port);
  //   }
  // }

  /*##-3- Configure the DMA ##################################################*/
  for (int i=0; i<2; i++) {
    if (adc_dev[i].adc == hadc->Instance) {
      DMA_Init_Channel(hadc, i);
    }
  }
 
}

static inline void DMA_Init_Channel(ADC_HandleTypeDef *hadc, uint8_t channel)
{ 
  static DMA_HandleTypeDef  DmaHandle;

  // log_info("Initialize ADC%d\n", channel);
  /* Configure DMA parameters */
  DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;   /* Transfer from ADC by half-word to match with ADC configuration: ADC resolution 10 or 12 bits */
  DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;   /* Transfer to memory by half-word to match with buffer variable type: half-word */
  DmaHandle.Init.Mode                = DMA_CIRCULAR;              /* DMA in circular mode to match with ADC configuration: DMA continuous requests */
  DmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;
  
  struct adc_dev_t * adc = &adc_dev[channel];

  DmaHandle.Instance = adc->dma_stream;
  if (adc->adc == ADC1) {
    return;
    log_info("Initialize ADC%d\n", channel);
    DmaHandle.Init.Request = DMA_REQUEST_ADC1;
  }
  else {
    log_info("Initialize ADC%d\n", channel);
    DmaHandle.Init.Request = DMA_REQUEST_ADC2;
  }
  HAL_DMA_DeInit(&DmaHandle);
  HAL_DMA_Init(&DmaHandle);

  /* Associate the initialized DMA handle to the ADC handle */
  __HAL_LINKDMA(hadc, DMA_Handle, DmaHandle);
  
  /*##-4- Configure the NVIC #################################################*/
  /* NVIC configuration for ADC interrupt */
  /* Priority: high-priority */
  HAL_NVIC_SetPriority(adc->adc_irqn, DEFAULT_IRQ_PRIO, 0);
  HAL_NVIC_EnableIRQ(adc->adc_irqn);
  
  /* NVIC configuration for DMA interrupt (transfer completion or error) */
  /* Priority: high-priority */
  HAL_NVIC_SetPriority(adc->dma_stream_irqn, (DEFAULT_IRQ_PRIO + 1U), 0);
  HAL_NVIC_EnableIRQ(adc->dma_stream_irqn);
  log_info("DMA_Init_Channel: %d\n", channel);
}

/**
  * @brief ADC MSP de-initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable clock of ADC peripheral
  *          - Revert GPIO associated to the peripheral channels to their default state
  *          - Revert DMA associated to the peripheral to its default state
  *          - Revert NVIC associated to the peripheral interruptions to its default state
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  /*##-1- Reset peripherals ##################################################*/
  ADCx_FORCE_RESET();
  ADCx_RELEASE_RESET();


  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* De-initialize GPIO pin of the selected ADC channel */
  // for (int i=0; i<EMONTX_CH_NUM; i++) {
  //   PERIPH_LOCK(adc_channels[i].port);  
  //   HAL_GPIO_DeInit(adc_channels[i].port, adc_channels[i].pin);
  //   PERIPH_UNLOCK(adc_channels[i].port); 
  // }

  /*##-3- Disable the DMA ####################################################*/
  /* De-Initialize the DMA associated to the peripheral */
  if(hadc->DMA_Handle != NULL)
  {
    HAL_DMA_DeInit(hadc->DMA_Handle);
  }

  /*##-4- Disable the NVIC ###################################################*/
  for (int i=0; i<2; i++) {
    /* Disable the NVIC configuration for ADC interrupt */
    HAL_NVIC_DisableIRQ(adc_dev[i].adc_irqn);
    /* Disable the NVIC configuration for DMA interrupt */
    HAL_NVIC_DisableIRQ(adc_dev[i].dma_stream_irqn);
  }
  
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
