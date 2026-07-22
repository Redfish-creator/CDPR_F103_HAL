/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task.h"
#include "vision.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef APP_VISION_DEBUG_TEST
#define APP_VISION_DEBUG_TEST 1   /* 1=只测试视觉串口; 0=恢复正式比赛流程 */
#endif

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if APP_VISION_DEBUG_TEST
static void VisionDebug_PrintFixed10(float v)
{
    int32_t scaled = (v >= 0.0f) ? (int32_t)(v * 10.0f + 0.5f)
                                 : (int32_t)(v * 10.0f - 0.5f);
    if (scaled < 0) {
        printf("-");
        scaled = -scaled;
    }
    printf("%ld.%ld", (long)(scaled / 10), (long)(scaled % 10));
}

static void VisionDebug_PrintCoord(const char *tag, uint32_t count,
                                   float x, float y, uint32_t age_ms)
{
    printf("[%s] cnt=%lu x=", tag, (unsigned long)count);
    VisionDebug_PrintFixed10(x);
    printf(" y=");
    VisionDebug_PrintFixed10(y);
    printf(" age=%lums\r\n", (unsigned long)age_ms);
}

static void VisionDebug_Banner(void)
{
    printf("\r\n=== VISION DEBUG MODE ===\r\n");
    printf("PC debug: USART1 PA9/PA10, 115200 8N1\r\n");
    printf("MaixCAM : USART2 PA2(TX)->CAM_RX, PA3(RX)<-CAM_TX, 115200 8N1\r\n");
    printf("Expect  : $HB,n# $LASER,x,y# $FIRE,x,y# $CIRCLE,x,y# $TILT,x,y#\r\n");
    printf("Notice  : motor/task/menu are disabled in this firmware\r\n\r\n");
}

static void VisionDebug_Task(void)
{
    static uint32_t last_stat_ms = 0;
    static uint32_t last_laser_count = 0;
    static uint32_t last_fire_count = 0;
    static uint32_t last_circle_count = 0;
    static uint32_t last_tilt_count = 0;
    static uint32_t last_hb_count = 0;

    uint32_t now = HAL_GetTick();

    if (g_vision.hb_count != last_hb_count) {
        last_hb_count = g_vision.hb_count;
        printf("[HB] cnt=%lu value=%d age=%lums, echoed to USART2\r\n",
               (unsigned long)g_vision.hb_count,
               g_vision.hb_value,
               (unsigned long)(now - g_vision.last_rx_ms));
    }

    if (g_vision.laser_count != last_laser_count) {
        last_laser_count = g_vision.laser_count;
        g_vision.laser_fresh = 0;
        VisionDebug_PrintCoord("LASER", g_vision.laser_count,
                               g_vision.laser_x, g_vision.laser_y,
                               now - g_vision.laser_ms);
    }

    if (g_vision.fire_count != last_fire_count) {
        last_fire_count = g_vision.fire_count;
        g_vision.fire_fresh = 0;
        VisionDebug_PrintCoord("FIRE", g_vision.fire_count,
                               g_vision.fire_x, g_vision.fire_y,
                               now - g_vision.fire_ms);
    }

    if (g_vision.circle_count != last_circle_count) {
        last_circle_count = g_vision.circle_count;
        g_vision.circle_fresh = 0;
        VisionDebug_PrintCoord("CIRCLE", g_vision.circle_count,
                               g_vision.circle_x, g_vision.circle_y,
                               now - g_vision.circle_ms);
    }

    if (g_vision.tilt_count != last_tilt_count) {
        last_tilt_count = g_vision.tilt_count;
        g_vision.tilt_fresh = 0;
        VisionDebug_PrintCoord("TILT", g_vision.tilt_count,
                               g_vision.tilt_x, g_vision.tilt_y,
                               now - g_vision.tilt_ms);
    }

    if ((now - last_stat_ms) >= 1000U) {
        last_stat_ms = now;
        printf("[STAT] online=%u ", (unsigned int)vision_is_online(1000U));
        if (g_vision.last_rx_ms == 0U) {
            printf("age=never ");
        } else {
            printf("age=%lums ", (unsigned long)(now - g_vision.last_rx_ms));
        }
        printf("rx=%lu frame=%lu bad=%lu unknown=%lu hb=%lu laser=%lu fire=%lu circle=%lu tilt=%lu\r\n",
               (unsigned long)g_vision.rx_byte_count,
               (unsigned long)g_vision.frame_count,
               (unsigned long)g_vision.bad_frame_count,
               (unsigned long)g_vision.unknown_count,
               (unsigned long)g_vision.hb_count,
               (unsigned long)g_vision.laser_count,
               (unsigned long)g_vision.fire_count,
               (unsigned long)g_vision.circle_count,
               (unsigned long)g_vision.tilt_count);
    }
}
#endif

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  vision_init();      /* 要用视觉才加; 需 USART2+DMA 已初始化 */
#if APP_VISION_DEBUG_TEST
  VisionDebug_Banner();
#else
  Task_Init();        /* 先显示 HOME; 视觉回中心成功后再进入菜单 */
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if APP_VISION_DEBUG_TEST
    vision_task();      /* 收到 $HB,n# 时原样回发给 MaixCAM */
    VisionDebug_Task();
    HAL_Delay(5);
#else
    Task_Loop();
    vision_task();      /* 视觉心跳回发($HB), 现在用视觉, 必须开 */
#endif
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
    return ch;
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
