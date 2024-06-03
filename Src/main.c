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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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

// #define PCA9635_SWRST 0x03 << 1
// #define PCA9635_ADDR 0x20 << 1
#define NUM_CHANNELS 5

GPIO_TypeDef *channels_port_mapping[NUM_CHANNELS] = {GPIOB, GPIOB, GPIOB, GPIOB, GPIOA};
uint16_t channels_pin_mapping[NUM_CHANNELS] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_8};
//uint8_t pca9635_output_mapping[NUM_CHANNELS] = {0x02, 0x03, 0x04, 0x05, 0x06};
uint32_t *tim_ccr_mapping[NUM_CHANNELS] = {&TIM3->CCR1, &TIM3->CCR2, &TIM3->CCR3, &TIM3->CCR4, &TIM1->CCR4};

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

// void pca9635_swrst() {
//   uint8_t mode1[2] = {0xA5, 0x5A}; //  magic byte 1, magic byte 2
//   HAL_I2C_Master_Transmit (&hi2c1, PCA9635_SWRST, mode1, 2, 5);
//   uint8_t magic3 = 0x06;
//   HAL_I2C_Master_Transmit (&hi2c1, 0x00, &magic3, 2, 5);
//
// }
//
// void pca9635_init() {
//     // https://www.nxp.com/docs/en/data-sheet/PCA9635.pdf, page 10-13
//     uint8_t mode1[2] = {0x00, 0b10000001};
//     HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, mode1, 2, 5);
//
//     uint8_t mode2[2] = {0x01, 0b00000110};
//     HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, mode2, 2, 5);
//
//     uint8_t ledout0[2] = {0x14, 0b10101010};
//     HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, ledout0, 2, 5);
//     uint8_t ledout1[2] = {0x15, 0b10101010};
//     HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, ledout1, 2, 5);
//     uint8_t ledout2[2] = {0x16, 0b10101010};
//     HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, ledout2, 2, 5);
//     uint8_t ledout3[2] = {0x17, 0b10101010};
//     HAL_I2C_Master_Transmit (&hi2c1, PCA9635_ADDR, ledout3, 2, 5);
// }

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
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  TIM2->CNT = 0;
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  // pca9635_swrst();
  // pca9635_init();
  //
  // i2cscan();

  TIM1->CCR4 = 0;
  TIM3->CCR1 = 0;
  TIM3->CCR2 = 0;
  TIM3->CCR3 = 0;
  TIM3->CCR4 = 0;

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  printf("Setup DONE!\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
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
    float real_brightness_setting = 50.0f * pow((float)(brightness_setting << 6) / 4096.0f, 2.0f);

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

    for(uint8_t i = 0; i < NUM_CHANNELS; i++) {
      GPIO_PinState on = !HAL_GPIO_ReadPin(channels_port_mapping[i], channels_pin_mapping[i]);
      if(channels_on_state[i] != on) {
        state_changed = 1;
        channels_on_state[i] = on;
      }
    }

    if(state_changed) {
      for(uint8_t i = 0; i < NUM_CHANNELS; i++) {
        if(channels_on_state[i] && brightness_setting > 0) {
          printf("ch %d duty cycle: %d\r\n", i, brightness_setting);
          *tim_ccr_mapping[i] = brightness_setting;
        }
        else {
          printf("ch %d off\r\n", i);
          *tim_ccr_mapping[i] = 0;
        }


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
