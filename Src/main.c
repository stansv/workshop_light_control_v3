/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <sys/types.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
/**
  * @brief  Retargets the C library printf function to the USART.
  */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define PCA9635_SWRST 0x03 << 1
#define PCA9635_ADDR 0x20 << 1
#define NUM_CHANNELS 5

uint16_t phase_shift_values[NUM_CHANNELS+1] = {0, 256/1, 256/2, 256/3, 256/4, 256/5};
GPIO_TypeDef *channels_port_mapping[NUM_CHANNELS] = {GPIOB, GPIOB, GPIOB, GPIOB, GPIOA};
uint16_t channels_pin_mapping[NUM_CHANNELS] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_8};
uint8_t pca9635_output_mapping[NUM_CHANNELS] = {0x02, 0x03, 0x04, 0x05, 0x06};
volatile uint8_t off = 0;
volatile uint8_t off_bounce_count = 0;
uint16_t remembered_cnt = 0;

// GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin

void i2cscan() {
  HAL_Delay(1000);
  uint8_t i, ret;

  /*-[ I2C Bus Scanning ]-*/
  printf("Starting I2C Scanning: \r\n");
  for(i=1; i<128; i++) {
    ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
    if (ret != HAL_OK) /* No ACK Received At That Address */
    {
      printf(" - ");
    }
    else if(ret == HAL_OK)
    {
      printf("0x%X", i);
    }
  }
  printf("Done! \r\n\r\n");
  /*--[ Scanning Done ]--*/
}

void pca_swrst() {
  uint8_t mode1[2] = {0xA5, 0x5A}; //  magic byte 1, magic byte 2
  HAL_I2C_Master_Transmit (&hi2c1, PCA9635_SWRST, mode1, 2, 5);
  uint8_t magic3 = 0x06;
  HAL_I2C_Master_Transmit (&hi2c1, 0x00, &magic3, 2, 5);

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  uint8_t channels_on_state[NUM_CHANNELS] = {0,0,0,0,0};
  uint16_t brightness_setting_state = 0;
  float current_brightness_setting = 0;
  uint8_t blink = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  TIM2->CNT = 0;
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  i2cscan();
  pca_swrst();

  uint8_t mode1[2] = {0x00, 0b10000001};
  HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, mode1, 2, 5);

  uint8_t mode2[2] = {0x01, 0b00000110};
  HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, mode2, 2, 5);

  uint8_t ledout0[2] = {0x14, 0b10101010};
  HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, ledout0, 2, 5);
  uint8_t ledout1[2] = {0x15, 0b10101010};
  HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, ledout1, 2, 5);
  uint8_t ledout2[2] = {0x16, 0b10101010};
  HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, ledout2, 2, 5);
  uint8_t ledout3[2] = {0x17, 0b10101010};
  HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, ledout3, 2, 5);

  uint8_t pwm0[2] = {0x02, 0b10000000};
  HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, pwm0, 2, 5);
  uint8_t pwm1[2] = {0x03, 0b10000000};
  HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, pwm1, 2, 5);
  uint8_t pwm2[2] = {0x04, 0b10000000};
  HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, pwm2, 2, 5);
  uint8_t pwm3[2] = {0x05, 0b10000000};
  HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, pwm3, 2, 5);
  uint8_t pwm4[2] = {0x06, 0b10000000};
  HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, pwm4, 2, 5);



  // setup PCA9635: enable change on ACK

  // disable sleep and allcall addr
  // uint8_t mode1[2] = {0x01, 0b10000000};
  // HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, &mode1, 2, 5);
  // HAL_Delay(1);
  //
  // // ???
  // uint8_t mode2[2] = {0x01, 0b00001100};
  // HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, &mode2, 2, 5);
  //
  // // uint8_t grppwm[2] = {0x12, 0xC8};
  // // HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, &grppwm, 2, 5);
  // //
  // // uint8_t grpfreq[2] = {0x13, 0xC8};
  // // HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, &grpfreq, 2, 5);
  //
  // uint8_t ledout0[2] = {0x14, 0xFF};
  // HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, &ledout0, 2, 5);
  // uint8_t ledout1[2] = {0x15, 0xFF};
  // HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, &ledout1, 2, 5);
  // uint8_t ledout2[2] = {0x16, 0xFF};
  // HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, &ledout2, 2, 5);
  // uint8_t ledout3[2] = {0x17, 0xFF};
  // HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, &ledout3, 2, 5);
  //
  // uint8_t pwm0[2] = {0x02, 0xC8};
  // HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, &pwm0, 2, 5);
  //
  printf("Setup DONE!\r\n");
  i2cscan();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    if(! blink) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    }
    blink++;
    if(off_bounce_count > 0) {
      off_bounce_count--;
    }

    uint8_t state_changed = 0;

    if (off) {
      if(remembered_cnt && TIM2->CNT > 0) {
        off = 0;
        remembered_cnt = 0;
      }
      if (!remembered_cnt) {
        remembered_cnt = TIM2->CNT;
        TIM2->CNT = 0;
      }
    } else if (remembered_cnt) {
      TIM2->CNT = remembered_cnt;
      remembered_cnt = 0;
    }

    uint16_t brightness_setting = TIM2->CNT;
    if(brightness_setting > 32767) {
      TIM2->CNT = 0;
      brightness_setting = 0;
    } else if(brightness_setting > 64) {
      TIM2->CNT = 64;
      brightness_setting = 64;
    }

    // https://alexgyver.ru/lessons/led-crt/
    // out = max * ((val / max) ^ gamma)

    float real_brightness_setting = 750.0f * pow((float)(brightness_setting << 6) / 4096.0f, 2.0f);

    if(current_brightness_setting < real_brightness_setting) {
      if(real_brightness_setting - current_brightness_setting > 0.5f) {
        current_brightness_setting += (real_brightness_setting - current_brightness_setting) * 0.1f;
      } else {
        current_brightness_setting = real_brightness_setting;
      }
    } else if(current_brightness_setting > real_brightness_setting) {
      if(current_brightness_setting - real_brightness_setting > 0.5f) {
        current_brightness_setting -= (current_brightness_setting - real_brightness_setting) * 0.1f;
      } else {
        current_brightness_setting = real_brightness_setting;
      }
    }

    brightness_setting = (uint16_t)current_brightness_setting;

    if(brightness_setting_state != brightness_setting) {
      state_changed = 1;
      brightness_setting_state = brightness_setting;
      printf("brightness_setting = %d\r\n", brightness_setting);
    }

    uint8_t num_channels_on = 0;

    for(uint8_t i = 0; i < NUM_CHANNELS; i++) {
      GPIO_PinState on = !HAL_GPIO_ReadPin(channels_port_mapping[i], channels_pin_mapping[i]);
      if(channels_on_state[i] != on) {
        state_changed = 1;
        channels_on_state[i] = on;
      }
      if(on) {
        num_channels_on++;
      }
    }

    if(state_changed) {
      uint16_t phase_shift = phase_shift_values[num_channels_on];
      uint16_t from = 0;

      for(uint8_t i = 0; i < NUM_CHANNELS; i++) {
        uint8_t led_on_l = 0, led_on_h = 0, led_off_l = 0, led_off_h = 0b00010000;
        if(channels_on_state[i] && brightness_setting > 0) {
          uint16_t to = from + brightness_setting;

          if(to >= 4096) {
            to -= 4096;
          }
          led_on_l = (uint8_t)(from >> 0) & 0xFF;
          led_on_h = (uint8_t)(from >> 8) & 0xFF;
          led_off_l = (uint8_t)(to >> 0) & 0xFF;
          led_off_h = (uint8_t)(to >> 8) & 0xFF;
          printf("ch %d on: %d off: %d --- %02x %02x %02x %02x %02x\r\n", i, from, to, pca9635_output_mapping[i], led_on_l, led_on_h, led_off_l, led_off_h);
          from += phase_shift;
        }
        else {
          printf("ch %d off\r\n", i);
        }

        uint8_t buffer[5] = {pca9635_output_mapping[i], led_on_l, led_on_h, led_off_l, led_off_h};
        HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, buffer, 5, 5);
      }
    }

    HAL_Delay(1);

    /* USER CODE END WHILE */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 128;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */

  /*Configure GPIO pins : PA2 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// EXTI Line9 External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_2 && off_bounce_count == 0) {
    off = !off;
    off_bounce_count--;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */