/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    OpenAMP/OpenAMP_TTY_echo/Inc/main.c
  * @author  MCD Application Team
  * @brief   Main program body.
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
#include "stm32mp15xx_disco.h"
#include "stm32mp1xx_it.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BUFFER_SIZE RPMSG_BUFFER_SIZE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IPCC_HandleTypeDef  hipcc;
TIM_HandleTypeDef   htim2;
ADC_HandleTypeDef   hadc1;
ADC_HandleTypeDef   hadc2;

#define ADC_VALUE_BUFFER_SIZE  2
struct emontx_values_t {
  __IO  uint16_t  AC[ADC_VALUE_BUFFER_SIZE];
  __IO  uint16_t  CT1[ADC_VALUE_BUFFER_SIZE];
  __IO  uint16_t  CT2[ADC_VALUE_BUFFER_SIZE];
  __IO  uint16_t  CT3[ADC_VALUE_BUFFER_SIZE];
  __IO  uint16_t  CT4[ADC_VALUE_BUFFER_SIZE];
};
struct emontx_values_t emontx_values;

#define ADC1_BUFFER_SIZE  (ADC_VALUE_BUFFER_SIZE * 3)
#define ADC2_BUFFER_SIZE  (ADC_VALUE_BUFFER_SIZE * 4)
uint16_t adc1_values[ADC1_BUFFER_SIZE];  // 2 channels on ADC1
uint16_t adc2_values[ADC2_BUFFER_SIZE];  // 3 channels on ADC2


struct adc_channel_t adc_channels[EMONTX_CH_NUM] = {
  [EMONTX_CH_AC] = {
    .adc = ADC2,
    .channel = ADC_CHANNEL_6,
    .port = GPIOF,
    .pin = GPIO_PIN_14,
  },
  [EMONTX_CH_CT1] = {
    .adc = ADC2,
    .channel = ADC_CHANNEL_2,
    .port = GPIOF,
    .pin = GPIO_PIN_13,
  },
  [EMONTX_CH_CT2] = {
    .adc = ADC2,
    .channel = ADC_CHANNEL_0,
    .port = GPIOA,
    .pin = GPIO_PIN_0,
  },
  [EMONTX_CH_CT3] = {
    .adc = ADC2,
    .channel = ADC_CHANNEL_1,
    .port = GPIOA,
    .pin = GPIO_PIN_1,
  },
  // [EMONTX_CH_CT4] = {
  //   .adc = ADC1,
  //   .channel = ADC_CHANNEL_13,
  //   .port = GPIOC,
  //   .pin = GPIO_PIN_3,
  // },
};


struct adc_dev_t adc_dev[NUMBER_OF_ADCS] = {
  [0] = {
    .adc = ADC2,
    .adc_irqn = ADC2_IRQn,
    .adc_irq_handler = &ADC2_IRQHandler,
    .dma_stream = DMA2_Stream1,
    .dma_stream_irqn = DMA2_Stream1_IRQn,
    .stream_irq_handler = &DMA2_Stream1_IRQHandler,
  },
  [1] = {
    .adc = ADC1,
    .adc_irqn = ADC1_IRQn,
    .adc_irq_handler = &ADC1_IRQHandler,
    .dma_stream = DMA2_Stream2,
    .dma_stream_irqn = DMA2_Stream2_IRQn,
    .stream_irq_handler = &DMA2_Stream2_IRQHandler,
  },
};

/* Variable to report status of DMA transfer of ADC group regular conversions */
/*  0: DMA transfer is not completed                                          */
/*  1: DMA transfer is completed                                              */
/*  2: DMA transfer has not yet been started yet (initial state)              */
__IO   uint8_t ubDmaTransferStatus = 2; /* Variable set into DMA interruption callback */


enum { PREAMBLE=0xABCD, };
#pragma pack(1)
struct packet {
  uint16_t preamble;
  uint16_t length;
  uint16_t crc16;
};

typedef void (*VIRT_UART_RxCpltCallback)(VIRT_UART_HandleTypeDef *huart);

void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart);
void VIRT_UART1_RxCpltCallback(VIRT_UART_HandleTypeDef *huart);


// uint8_t VirtUart0ChannelBuffRx[MAX_BUFFER_SIZE];
uint8_t VirtUart0ChannelBuffTx[MAX_BUFFER_SIZE];
// uint8_t VirtUart1ChannelBuffRx[MAX_BUFFER_SIZE];
uint8_t VirtUart1ChannelBuffTx[MAX_BUFFER_SIZE];

struct virt_uart {
  VIRT_UART_HandleTypeDef huart;
  __IO FlagStatus rx_status;
  __IO uint8_t *rx_buffer;
  __IO uint16_t rx_size;
  __IO FlagStatus tx_status;
  __IO uint8_t *tx_buffer;
  __IO uint16_t tx_size;
  VIRT_UART_RxCpltCallback cbk;
};

struct virt_uart virt_uart0 = {
  .rx_status = RESET,
  .rx_buffer = NULL,
  .rx_size = 0,
  .tx_status = RESET,
  .tx_buffer = VirtUart0ChannelBuffTx,
  .tx_size = 0,
  .cbk = VIRT_UART0_RxCpltCallback,
};
__IO uint16_t virt_uart0_expected_nbytes = 0;

struct virt_uart virt_uart1 = {
  .rx_status = RESET,
  .rx_buffer = NULL,
  .rx_size = 0,
  .tx_status = RESET,
  .tx_buffer = VirtUart1ChannelBuffTx,
  .tx_size = 0,
  .cbk = VIRT_UART1_RxCpltCallback,
};
__IO uint16_t virt_uart1_expected_nbytes = 0;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_IPCC_Init(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void Configure_ADC(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initialize the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
    if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure the system clock */
    SystemClock_Config();
  }
  /* USER CODE END Init */

  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();

  if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure PMIC */
    BSP_PMIC_Init();
    BSP_PMIC_InitRegulators();

    /* Configure VREFBUF */
    __HAL_RCC_VREF_CLK_ENABLE();
    HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE);
    HAL_SYSCFG_EnableVREFBUF();
  }

  /* IPCC initialisation */
   MX_IPCC_Init();
  /* OpenAmp initialisation ---------------------------------*/
  MX_OPENAMP_Init(RPMSG_REMOTE, NULL);
  MX_GPIO_Init();
  MX_TIM2_Init();

  COM_InitTypeDef uart_init;
  uart_init.BaudRate = 115200;
  uart_init.Parity = UART_PARITY_NONE;
  uart_init.StopBits = UART_STOPBITS_1;
  uart_init.WordLength = UART_WORDLENGTH_8B;
  uart_init.HwFlowCtl = UART_HWCONTROL_NONE;
  BSP_COM_Init(COM2, &uart_init);
  BSP_COM_SelectLogPort(COM2);

  log_info("Cortex-M4 boot successful with STM32Cube FW version: v%ld.%ld.%ld \r\n",
                                            ((HAL_GetHalVersion() >> 24) & 0x000000FF),
                                            ((HAL_GetHalVersion() >> 16) & 0x000000FF),
                                            ((HAL_GetHalVersion() >> 8) & 0x000000FF));

  log_info("MAX_BUFFER_SIZE: %d\n", MAX_BUFFER_SIZE);
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
  /*
   * Create Virtual UART device
   * defined by a rpmsg channel attached to the remote device
   */
  log_info("Virtual UART0 OpenAMP-rpmsg channel creation\r\n");
  if (VIRT_UART_Init(&virt_uart0.huart) != VIRT_UART_OK) {
    log_err("VIRT_UART_Init UART0 failed.\r\n");
    Error_Handler(__FILE__, __LINE__);
  }

  log_info("Virtual UART1 OpenAMP-rpmsg channel creation\r\n");
  if (VIRT_UART_Init(&virt_uart1.huart) != VIRT_UART_OK) {
    log_err("VIRT_UART_Init UART1 failed.\r\n");
    Error_Handler(__FILE__, __LINE__);
  }

  /*Need to register callback for message reception by channels*/
  if(VIRT_UART_RegisterCallback(&virt_uart0.huart, VIRT_UART_RXCPLT_CB_ID, virt_uart0.cbk) != VIRT_UART_OK)
  {
   Error_Handler(__FILE__, __LINE__);
  }
  if(VIRT_UART_RegisterCallback(&virt_uart1.huart, VIRT_UART_RXCPLT_CB_ID, virt_uart1.cbk) != VIRT_UART_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

  for (int i = 0; i < ADC_VALUE_BUFFER_SIZE; i++)
  {
    emontx_values.AC[i] = 0;
    emontx_values.CT1[i] = 0;
    emontx_values.CT2[i] = 0;
    emontx_values.CT3[i] = 0;
    emontx_values.CT4[i] = 0;
  }


  /* Enable GPIOA clock */
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* Configure PH.6 pin as output */
  GPIO_InitTypeDef   GPIO_InitStruct;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  PERIPH_LOCK(GPIOH);
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_6, GPIO_PIN_SET);
  PERIPH_UNLOCK(GPIOH);

 /* Configure ADC */
  /* Note: This function configures the ADC but does not enable it.           */
  /*       Only ADC internal voltage regulator is enabled by function         */
  /*       "HAL_ADC_Init()".                                                  */
  /*       To activate ADC (ADC enable and ADC conversion start), use         */
  /*       function "HAL_ADC_Start_xxx()".                                    */
  /*       This is intended to optimize power consumption:                    */
  /*       1. ADC configuration can be done once at the beginning             */
  /*          (ADC disabled, minimal power consumption)                       */
  /*       2. ADC enable (higher power consumption) can be done just before   */
  /*          ADC conversions needed.                                         */
  /*          Then, possible to perform successive ADC activation and         */
  /*          deactivation without having to set again ADC configuration.     */
  Configure_ADC();
  
  /* Run the ADC linear calibration in single-ended mode */
  if (HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler(__FILE__, __LINE__);
  }
  if (HAL_ADCEx_Calibration_Start(&hadc2,ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler(__FILE__, __LINE__);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*## Enable Timer ########################################################*/
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    /* Counter enable error */
    Error_Handler(__FILE__, __LINE__);
  }
  
  /*## Start ADC conversions ###############################################*/
  /* Start ADC group regular conversion with DMA */
  // if (HAL_ADC_Start_DMA(&hadc1,
  //                       (uint32_t *)adc1_values,
  //                       ADC1_BUFFER_SIZE
  //                      ) != HAL_OK)
  // {
  //   /* ADC conversion start error */
  //   Error_Handler(__FILE__, __LINE__);
  // }
  if (HAL_ADC_Start_DMA(&hadc2,
                        (uint32_t *)adc2_values,
                        ADC2_BUFFER_SIZE
                       ) != HAL_OK)
  {
    /* ADC conversion start error */
    Error_Handler(__FILE__, __LINE__);
  }
  log_info("Started ADC/DMA\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    OPENAMP_check_for_message();

    /* USER CODE END WHILE */
    if (virt_uart0.tx_status) {
      virt_uart0.tx_status = RESET;
      VIRT_UART_Transmit(&virt_uart0.huart, (uint8_t*) virt_uart0.tx_buffer, virt_uart0.tx_size);
    }

    if (virt_uart1.tx_status) {
      virt_uart1.tx_status = RESET;
      VIRT_UART_Transmit(&virt_uart1.huart, (uint8_t*) virt_uart1.tx_buffer, virt_uart1.tx_size);
    }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_DIG;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSIDivValue = RCC_HSI_DIV1;

  /**PLL1 Config
  */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL12SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 81;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 1;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLFRACV = 0x800;
  RCC_OscInitStruct.PLL.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
  RCC_OscInitStruct.PLL.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

    /**PLL2 Config
    */
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL2.PLLSource = RCC_PLL12SOURCE_HSE;
  RCC_OscInitStruct.PLL2.PLLM = 3;
  RCC_OscInitStruct.PLL2.PLLN = 66;
  RCC_OscInitStruct.PLL2.PLLP = 2;
  RCC_OscInitStruct.PLL2.PLLQ = 1;
  RCC_OscInitStruct.PLL2.PLLR = 1;
  RCC_OscInitStruct.PLL2.PLLFRACV = 0x1400;
  RCC_OscInitStruct.PLL2.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL2.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
  RCC_OscInitStruct.PLL2.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

    /**PLL3 Config
    */
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL3.PLLSource = RCC_PLL3SOURCE_HSE;
  RCC_OscInitStruct.PLL3.PLLM = 2;
  RCC_OscInitStruct.PLL3.PLLN = 34;
  RCC_OscInitStruct.PLL3.PLLP = 2;
  RCC_OscInitStruct.PLL3.PLLQ = 17;
  RCC_OscInitStruct.PLL3.PLLR = 37;
  RCC_OscInitStruct.PLL3.PLLRGE = RCC_PLL3IFRANGE_1;
  RCC_OscInitStruct.PLL3.PLLFRACV = 0x1A04;
  RCC_OscInitStruct.PLL3.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL3.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
  RCC_OscInitStruct.PLL3.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

    /**PLL4 Config
    */
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL4.PLLSource = RCC_PLL4SOURCE_HSE;
  RCC_OscInitStruct.PLL4.PLLM = 4;
  RCC_OscInitStruct.PLL4.PLLN = 99;
  RCC_OscInitStruct.PLL4.PLLP = 6;
  RCC_OscInitStruct.PLL4.PLLQ = 8;
  RCC_OscInitStruct.PLL4.PLLR = 8;
  RCC_OscInitStruct.PLL4.PLLRGE = RCC_PLL4IFRANGE_0;
  RCC_OscInitStruct.PLL4.PLLFRACV = 0;
  RCC_OscInitStruct.PLL4.PLLMODE = RCC_PLL_INTEGER;
  RCC_OscInitStruct.PLL4.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
  RCC_OscInitStruct.PLL4.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
  Error_Handler(__FILE__, __LINE__);
  }
  /**RCC Clock Config
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_ACLK
                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                |RCC_CLOCKTYPE_PCLK3|RCC_CLOCKTYPE_PCLK4
                |RCC_CLOCKTYPE_PCLK5|RCC_CLOCKTYPE_MPU;
  RCC_ClkInitStruct.MPUInit.MPU_Clock = RCC_MPUSOURCE_PLL1;
  RCC_ClkInitStruct.MPUInit.MPU_Div = RCC_MPU_DIV2;
  RCC_ClkInitStruct.AXISSInit.AXI_Clock = RCC_AXISSOURCE_PLL2;
  RCC_ClkInitStruct.AXISSInit.AXI_Div = RCC_AXI_DIV1;
  RCC_ClkInitStruct.MCUInit.MCU_Clock = RCC_MCUSSOURCE_PLL3;
  RCC_ClkInitStruct.MCUInit.MCU_Div = RCC_MCU_DIV1;
  RCC_ClkInitStruct.APB4_Div = RCC_APB4_DIV2;
  RCC_ClkInitStruct.APB5_Div = RCC_APB5_DIV4;
  RCC_ClkInitStruct.APB1_Div = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2_Div = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB3_Div = RCC_APB3_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
  Error_Handler(__FILE__, __LINE__);
  }

  /**Set the HSE division factor for RTC clock
  */
  __HAL_RCC_RTC_HSEDIV(24);
}

/**
  * @brief IPPC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
     Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 97999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

}

static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart)
{

  /* copy received msg in a variable to sent it back to master processor in main infinite loop*/
  uint16_t recv_size = huart->RxXferSize < MAX_BUFFER_SIZE? huart->RxXferSize : MAX_BUFFER_SIZE-1;

  struct packet* in = (struct packet*) &huart->pRxBuffPtr[0];
  if (in->preamble == PREAMBLE) {
    in->preamble = 0;
    virt_uart0_expected_nbytes = in->length;
    log_info("length: %d\n", virt_uart0_expected_nbytes);                        
  }

  virt_uart0.rx_size += recv_size;
  log_info("UART0: %d/%d\n", virt_uart0.rx_size, virt_uart0_expected_nbytes);
  if (virt_uart0.rx_size >= virt_uart0_expected_nbytes) {
    virt_uart0.rx_size = 0;
    virt_uart0.tx_buffer[0] = virt_uart0_expected_nbytes & 0xff;
    virt_uart0.tx_buffer[1] = (virt_uart0_expected_nbytes >> 8) & 0xff;
    log_info("UART0 resp: %d\n", virt_uart0_expected_nbytes);
    virt_uart0_expected_nbytes = 0;
    virt_uart0.tx_size = 2;
    virt_uart0.tx_status = SET;
    // huart->RxXferSize = 0;
  }
}

void VIRT_UART1_RxCpltCallback(VIRT_UART_HandleTypeDef *huart)
{

  /* copy received msg in a variable to sent it back to master processor in main infinite loop*/
  uint16_t recv_size = huart->RxXferSize < MAX_BUFFER_SIZE? huart->RxXferSize : MAX_BUFFER_SIZE-1;

  struct packet* in = (struct packet*) &huart->pRxBuffPtr[0];
  if (in->preamble == PREAMBLE) {
    
  }

  virt_uart1.rx_size += recv_size;
  log_info("UART0: %d/%d\n", recv_size, virt_uart1.rx_size);
  if (virt_uart1.rx_size >= 512) {
    virt_uart1.tx_buffer[1] = (virt_uart1.rx_size >> 8) & 0xff;
    virt_uart1.tx_buffer[0] = virt_uart1.rx_size & 0xff;
    virt_uart1.rx_size = 0;
    virt_uart1.tx_size = 2;
    virt_uart1.tx_status = SET;
  }
}


/**
  * @brief  Configure ADC (ADC instance: ADCx) and GPIO used by ADC channels.
  *         Configuration of GPIO:
  *           - Pin:                    PA.04 (on this STM32 device, ADC2 channel 16 is mapped on this GPIO)
  *           - Mode:                   analog
  *         Configuration of ADC:
  *         - Common to several ADC:
  *           - Conversion clock:       Synchronous from PCLK
  *           - Internal path:          None                         (default configuration from reset state)
  *         - Multimode
  *           Feature not used: all parameters let to default configuration from reset state
  *           - Mode                    Independent                  (default configuration from reset state)
  *           - DMA transfer:           Disabled                     (default configuration from reset state)
  *           - Delay sampling phases   1 ADC clock cycle            (default configuration from reset state)
  *         - ADC instance
  *           - Resolution:             12 bits                      (default configuration from reset state)
  *           - Data alignment:         right aligned                (default configuration from reset state)
  *           - Low power mode:         disabled                     (default configuration from reset state)
  *           - Offset:                 none                         (default configuration from reset state)
  *         - Group regular
  *           - Trigger source:         SW start
  *           - Trigger edge:           not applicable with SW start
  *           - Continuous mode:        single conversion            (default configuration from reset state)
  *           - DMA transfer:           enabled, unlimited requests
  *           - Overrun:                data overwritten
  *           - Sequencer length:       disabled: 1 rank             (default configuration from reset state)
  *           - Sequencer discont:      disabled: sequence done in 1 scan (default configuration from reset state)
  *           - Sequencer rank 1:       ADCx ADCx_CHANNELa
  *         - Group injected
  *           Feature not used: all parameters let to default configuration from reset state
  *           - Trigger source:         SW start                     (default configuration from reset state)
  *           - Trigger edge:           not applicable with SW start
  *           - Auto injection:         disabled                     (default configuration from reset state)
  *           - Contexts queue:         disabled                     (default configuration from reset state)
  *           - Sequencer length:       disabled: 1 rank             (default configuration from reset state)
  *           - Sequencer discont:      disabled: sequence done in 1 scan (default configuration from reset state)
  *           - Sequencer rank 1:       first channel available      (default configuration from reset state)
  *         - Channel
  *           - Sampling time:          ADCx ADCx_CHANNELa set to sampling time 160.5 ADC clock cycles (on this STM32 serie, sampling time is channel wise)
  *           - Differential mode:      single ended                 (default configuration from reset state)
  *         - Analog watchdog
  *           Feature not used: all parameters let to default configuration from reset state
  *           - AWD number:             1
  *           - Monitored channels:     none                         (default configuration from reset state)
  *           - Threshold high:         0x000                        (default configuration from reset state)
  *           - Threshold low:          0xFFF                        (default configuration from reset state)
  *         - Oversampling
  *           Feature not used: all parameters let to default configuration from reset state
  *           - Scope:                  none                         (default configuration from reset state)
  *           - Discontinuous mode:     disabled                     (default configuration from reset state)
  *           - Ratio:                  2                            (default configuration from reset state)
  *           - Shift:                  none                         (default configuration from reset state)
  *         - Interruptions
  *           None: with HAL driver, ADC interruptions are set using
  *           function "HAL_ADC_start_xxx()".
  * @note   Using HAL driver, configuration of GPIO used by ADC channels,
  *         NVIC and clock source at top level (RCC)
  *         are not implemented into this function,
  *         must be implemented into function "HAL_ADC_MspInit()".
  * @param  None
  * @retval None
  */
__STATIC_INLINE void Configure_ADC(void)
{
  ADC_ChannelConfTypeDef   sConfig;
  
  /*## Configuration of ADC ##################################################*/
  
  /*## Configuration of ADC hierarchical scope: ##############################*/
  /*## common to several ADC, ADC instance, ADC group regular  ###############*/
  
  log_info("Starting ADC configuration...\n");
  /* Set ADC instance of HAL ADC handle hadc */
  hadc1.Instance = ADC1;
  
  /* Configuration of HAL ADC handle init structure:                          */
  /* parameters of scope ADC instance and ADC group regular.                  */
  /* Note: On this STM32 serie, ADC group regular sequencer is                */
  /*       fully configurable: sequencer length and each rank                 */
  /*       affectation to a channel are configurable.                         */
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;              /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait      = DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
  hadc1.Init.NbrOfConversion       = 3;                             /* Parameter discarded because sequencer is disabled */
  hadc1.Init.DiscontinuousConvMode = ENABLE;                       /* Parameter discarded because sequencer is disabled */
  hadc1.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
  hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIG_T2_TRGO;      /* Trig of conversion start done by external event */
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING; /* Parameter discarded because trig of conversion by software start (no external event) */
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode      = DISABLE;

  // if (HAL_ADC_DeInit(&hadc1) != HAL_OK)
  // {
  //   /* ADC Deinitialization error */
  //   Error_Handler(__FILE__, __LINE__);
  // }
  // if (HAL_ADC_Init(&hadc1) != HAL_OK)
  // {
  //   /* ADC initialization error */
  //   Error_Handler(__FILE__, __LINE__);
  // }

  memcpy(&hadc2, &hadc1, sizeof(ADC_HandleTypeDef));
  hadc2.Instance = ADC2;
  hadc2.Init.NbrOfConversion = 4;
  if (HAL_ADC_DeInit(&hadc2) != HAL_OK)
  {
    /* ADC Deinitialization error */
    Error_Handler(__FILE__, __LINE__);
  }
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    /* ADC initialization error */
    Error_Handler(__FILE__, __LINE__);
  }
  
  /*## Configuration of ADC hierarchical scope: ##############################*/
  /*## ADC group injected and channels mapped on group injected ##############*/
  
  /* Note: ADC group injected not used and not configured in this example.    */
  /*       Refer to other ADC examples using this feature.                    */
  /* Note: Call of the functions below are commented because they are         */
  /*       useless in this example:                                           */
  /*       setting corresponding to default configuration from reset state.   */
  
  
  /*## Configuration of ADC hierarchical scope: ##############################*/
  /*## channels mapped on group regular         ##############################*/
  
  /* Configuration of channel on ADCx regular group on sequencer rank 1 */
  /* Note: On this STM32 serie, ADC group regular sequencer is                */
  /*       fully configurable: sequencer length and each rank                 */
  /*       affectation to a channel are configurable.                         */
  /* Note: Considering IT occurring after each ADC conversion                 */
  /*       (IT by ADC group regular end of unitary conversion),               */
  /*       select sampling time and ADC clock with sufficient                 */
  /*       duration to not create an overhead situation in IRQHandler.        */
  sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;  /* ADC channel sampling time */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* ADC channel differential mode */
  sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* ADC channel affected to offset number */
  sConfig.Offset       = 0;                           /* Parameter discarded because offset correction is disabled */
  
  sConfig.Rank         = ADC_REGULAR_RANK_1;          /* ADC group regular rank in which is mapped the selected ADC channel */
  sConfig.Channel      = adc_channels[EMONTX_CH_AC].channel;               /* ADC channel selection */
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
      /* Channel Configuration Error */
      Error_Handler(__FILE__, __LINE__);
  }
  sConfig.Rank         = ADC_REGULAR_RANK_2;          /* ADC group regular rank in which is mapped the selected ADC channel */
  sConfig.Channel      = adc_channels[EMONTX_CH_CT1].channel;               /* ADC channel selection */
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
      /* Channel Configuration Error */
      Error_Handler(__FILE__, __LINE__);
  }
  sConfig.Rank         = ADC_REGULAR_RANK_3;          /* ADC group regular rank in which is mapped the selected ADC channel */
  sConfig.Channel      = adc_channels[EMONTX_CH_CT2].channel;               /* ADC channel selection */
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
      /* Channel Configuration Error */
      Error_Handler(__FILE__, __LINE__);
  }
  sConfig.Rank         = ADC_REGULAR_RANK_4;          /* ADC group regular rank in which is mapped the selected ADC channel */
  sConfig.Channel      = adc_channels[EMONTX_CH_CT3].channel;               /* ADC channel selection */
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
      /* Channel Configuration Error */
      Error_Handler(__FILE__, __LINE__);
  }
  // sConfig.Rank         = ADC_REGULAR_RANK_3;          /* ADC group regular rank in which is mapped the selected ADC channel */
  // sConfig.Channel      = adc_channels[EMONTX_CH_CT4].channel;               /* ADC channel selection */
  // if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  // {
  //     /* Channel Configuration Error */
  //     Error_Handler(__FILE__, __LINE__);
  // }
  log_info("ADC configuration done...\n");
  
  /*## Configuration of ADC hierarchical scope: multimode ####################*/
  /* Note: ADC multimode not used and not configured in this example.         */
  /*       Refer to other ADC examples using this feature.                    */
  
  
  /*## Configuration of ADC transversal scope: analog watchdog ###############*/
  
  /* Note: ADC analog watchdog not used and not configured in this example.   */
  /*       Refer to other ADC examples using this feature.                    */
  
  
  /*## Configuration of ADC transversal scope: oversampling ##################*/
  
  /* Note: ADC oversampling not used and not configured in this example.      */
  /*       Refer to other ADC examples using this feature.                    */
  
}

  
  
/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  hadc: ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  
  /* Update status variable of DMA transfer */
  ubDmaTransferStatus = 1;
  
  /* Set LED depending on DMA transfer status */
  /* - Turn-on if DMA transfer is completed */
  /* - Turn-off if DMA transfer is not completed */
  BSP_LED_On(LED7);

  if (hadc->Instance == ADC1) {
    sprintf((char*)virt_uart0.tx_buffer, "ADC[1.2]:%d,%d,%d,%d\n",
      adc2_values[0], adc2_values[1], adc2_values[2], adc2_values[3]);
    printf((char*)virt_uart0.tx_buffer);
  }
  else if (hadc->Instance == ADC2) {
    sprintf((char*)virt_uart0.tx_buffer, "ADC[2.2]:%d,%d,%d,%d\n",
      adc2_values[0], adc2_values[1], adc2_values[2], adc2_values[3]);
    printf((char*)virt_uart0.tx_buffer);
  }

  virt_uart0.rx_size = 0;
  virt_uart0_expected_nbytes = 0;
  virt_uart0.tx_size = strlen((char*)virt_uart0.tx_buffer);
  virt_uart0.tx_status = SET;
}

/**
  * @brief  Conversion DMA half-transfer callback in non blocking mode 
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  
  /* Update status variable of DMA transfer */
  ubDmaTransferStatus = 0;
  
  /* Set LED depending on DMA transfer status */
  /* - Turn-on if DMA transfer is completed */
  /* - Turn-off if DMA transfer is not completed */
  BSP_LED_Off(LED7);

  if (hadc->Instance == ADC1) {
    sprintf((char*)virt_uart0.tx_buffer, "ADC[1.1]:%d,%d,%d,%d\n",
      adc2_values[0], adc2_values[1], adc2_values[2], adc2_values[3]);
    printf((char*)virt_uart0.tx_buffer);
  }
  else if (hadc->Instance == ADC2) {
    sprintf((char*)virt_uart0.tx_buffer, "ADC[2.1]:%d,%d,%d,%d\n",
      adc2_values[0], adc2_values[1], adc2_values[2], adc2_values[3]);
    printf((char*)virt_uart0.tx_buffer);
  }

  virt_uart0.rx_size = 0;
  virt_uart0_expected_nbytes = 0;
  virt_uart0.tx_size = strlen((char*)virt_uart0.tx_buffer);
  virt_uart0.tx_status = SET;
}

/**
  * @brief  ADC error callback in non blocking mode
  *        (ADC conversion with interruption or transfer by DMA)
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  /* In case of ADC error, call main error handler */
  Error_Handler(__FILE__, __LINE__);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  printf("Error_Handler: %s:%d", file, line);
  while(1)
  {
    /* Toggle LED7 */
    BSP_LED_Off(LED7);
    HAL_Delay(800);
    BSP_LED_On(LED7);
    HAL_Delay(10);
    BSP_LED_Off(LED7);
    HAL_Delay(180);
    BSP_LED_On(LED7);
    HAL_Delay(10);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  log_err("OOOps: file %s, line %d\r\n", __FILE__, __LINE__);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
