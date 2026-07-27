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
#include "motor_test.h"
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef APP_VISION_DEBUG_TEST
#define APP_VISION_DEBUG_TEST 0   /* 1=只测试视觉串口; 调完视觉后改回0恢复任务流程 */
#endif
#ifndef APP_MOTOR_DEBUG_TEST
#define APP_MOTOR_DEBUG_TEST 0    /* 1=串口命令触发的低速电机测试 */
#endif
#ifndef APP_VISION_DEBUG_AUTO_TOTAL
#define APP_VISION_DEBUG_AUTO_TOTAL 0 /* 1=调试时自动请求 total; 默认用KEY0手动切换, 避免误锚定 */
#endif
#if APP_VISION_DEBUG_TEST && APP_MOTOR_DEBUG_TEST
#error "Select only one debug test mode"
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

static uint32_t VisionDebug_Age(uint32_t now, uint32_t event_ms)
{
    return (event_ms <= now) ? (now - event_ms) : 0U;
}

static uint32_t VisionDebug_TimeDiff(uint32_t a, uint32_t b)
{
    return (a >= b) ? (a - b) : (b - a);
}

static const char *VisionDebug_ErrorName(vision_parse_error_t reason)
{
    switch (reason) {
    case VISION_PARSE_ERROR_EMPTY:          return "empty frame";
    case VISION_PARSE_ERROR_NO_START:       return "'#' without '$'";
    case VISION_PARSE_ERROR_RESTARTED:      return "new '$' before '#'";
    case VISION_PARSE_ERROR_TOO_LONG:       return "frame too long";
    case VISION_PARSE_ERROR_MISSING_FIELD:  return "missing field";
    case VISION_PARSE_ERROR_INVALID_NUMBER: return "invalid number";
    default:                                return "unknown parse error";
    }
}

static const char *VisionDebug_CircleName(float x, float y)
{
    const float tol = 3.0f;

    if (fabsf(x - 0.0f) <= tol && fabsf(y - 0.0f) <= tol) return "CENTER";
    if (fabsf(x + 20.0f) <= tol && fabsf(y - 20.0f) <= tol) return "TOP_LEFT";
    if (fabsf(x - 20.0f) <= tol && fabsf(y - 20.0f) <= tol) return "TOP_RIGHT";
    if (fabsf(x + 20.0f) <= tol && fabsf(y + 20.0f) <= tol) return "BOTTOM_LEFT";
    if (fabsf(x - 20.0f) <= tol && fabsf(y + 20.0f) <= tol) return "BOTTOM_RIGHT";
    return "UNKNOWN";
}

static void VisionDebug_PrintGlobalPair(uint32_t now)
{
    const uint32_t pair_max_ms = 250U;
    uint32_t pair_dt;

    if (g_vision.laser_count == 0U || g_vision.circle_count == 0U) {
        return;
    }

    pair_dt = VisionDebug_TimeDiff(g_vision.laser_ms, g_vision.circle_ms);
    if (pair_dt > pair_max_ms) {
        printf("[PAIR WAIT] laser/circle time gap=%lums, need <=%lums\r\n",
               (unsigned long)pair_dt, (unsigned long)pair_max_ms);
        return;
    }

    printf("[GLOBAL PAIR] circle=%s ref=(",
           VisionDebug_CircleName(g_vision.circle_x, g_vision.circle_y));
    VisionDebug_PrintFixed10(g_vision.circle_x);
    printf(",");
    VisionDebug_PrintFixed10(g_vision.circle_y);
    printf(") laser=(");
    VisionDebug_PrintFixed10(g_vision.laser_x);
    printf(",");
    VisionDebug_PrintFixed10(g_vision.laser_y);
    printf(") err=(");
    VisionDebug_PrintFixed10(g_vision.laser_x - g_vision.circle_x);
    printf(",");
    VisionDebug_PrintFixed10(g_vision.laser_y - g_vision.circle_y);
    printf(") pair_dt=%lums age=%lums\r\n",
           (unsigned long)pair_dt,
           (unsigned long)VisionDebug_Age(now, g_vision.last_valid_ms));
}

static void VisionDebug_SendTotalCommand(uint32_t now)
{
#if APP_VISION_DEBUG_AUTO_TOTAL
    static uint32_t last_cmd_ms = 0;

    if (last_cmd_ms == 0U || (now - last_cmd_ms) >= 3000U) {
        uint8_t ok = vision_send_mode("total");
        last_cmd_ms = now;
        printf("[CMD] &mode,total# %s\r\n", (ok != 0U) ? "sent" : "send_fail");
    }
#else
    (void)now;
#endif
}

static uint8_t VisionDebug_ButtonEdge(GPIO_TypeDef *port, uint16_t pin, uint8_t *last)
{
    uint8_t raw = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1U : 0U;
    uint8_t edge = (raw != 0U && *last == 0U) ? 1U : 0U;
    *last = raw;
    return edge;
}

static void VisionDebug_Banner(void)
{
    printf("\r\n=== VISION DEBUG MODE ===\r\n");
    printf("PC debug: USART1 PA9/PA10, 115200 8N1\r\n");
    printf("MaixCAM : USART2 PA2(TX)->CAM_RX, PA3(RX)<-CAM_TX, 115200 8N1\r\n");
    printf("Expect  : $HB,n# $LASER,x,y# $HOME,dx,dy# $FIRE,x,y# $CIRCLE,x,y# $TILT,x,y#\r\n");
    printf("Mode   : power-up MaixCAM stays initial; press KEY0 near center circle to send &mode,total#\r\n");
    printf("Result  : [LINK OK]=connected, [NO RX]=wiring/baud, [DATA ERROR]=bad packet\r\n");
    printf("Notice  : motor/task/menu are disabled in this firmware\r\n\r\n");
}

static void VisionDebug_Task(void)
{
    static uint32_t last_stat_ms = 0;
    static uint32_t last_laser_count = 0;
    static uint32_t last_home_count = 0;
    static uint32_t last_fire_count = 0;
    static uint32_t last_circle_count = 0;
    static uint32_t last_tilt_count = 0;
    static uint32_t last_hb_count = 0;
    static uint32_t last_bad_count = 0;
    static uint32_t last_unknown_count = 0;
    static uint32_t last_uart_error_count = 0;
    static uint32_t last_pair_laser_count = 0;
    static uint32_t last_pair_circle_count = 0;
    static uint8_t last_key0 = 0U;

    uint32_t now = HAL_GetTick();

    VisionDebug_SendTotalCommand(now);

    if (VisionDebug_ButtonEdge(GPIOC, GPIO_PIN_5, &last_key0) != 0U) {
        uint8_t ok = vision_send_mode("total");
        printf("[CMD] KEY0 -> &mode,total# %s\r\n", (ok != 0U) ? "sent" : "send_fail");
    }

    if (g_vision.hb_count != last_hb_count) {
        last_hb_count = g_vision.hb_count;
        printf("[HB] cnt=%lu value=%d age=%lums, echoed to USART2\r\n",
               (unsigned long)g_vision.hb_count,
               g_vision.hb_value,
               (unsigned long)VisionDebug_Age(now, g_vision.last_rx_ms));
    }

    if (g_vision.laser_count != last_laser_count) {
        last_laser_count = g_vision.laser_count;
        g_vision.laser_fresh = 0;
        VisionDebug_PrintCoord("LASER", g_vision.laser_count,
                               g_vision.laser_x, g_vision.laser_y,
                               VisionDebug_Age(now, g_vision.laser_ms));
    }

    if (g_vision.home_count != last_home_count) {
        last_home_count = g_vision.home_count;
        g_vision.home_fresh = 0;
        VisionDebug_PrintCoord("HOME", g_vision.home_count,
                               g_vision.home_x, g_vision.home_y,
                               VisionDebug_Age(now, g_vision.home_ms));
    }

    if (g_vision.fire_count != last_fire_count) {
        last_fire_count = g_vision.fire_count;
        g_vision.fire_fresh = 0;
        VisionDebug_PrintCoord("FIRE", g_vision.fire_count,
                               g_vision.fire_x, g_vision.fire_y,
                               VisionDebug_Age(now, g_vision.fire_ms));
    }

    if (g_vision.circle_count != last_circle_count) {
        last_circle_count = g_vision.circle_count;
        g_vision.circle_fresh = 0;
        VisionDebug_PrintCoord("CIRCLE", g_vision.circle_count,
                               g_vision.circle_x, g_vision.circle_y,
                               VisionDebug_Age(now, g_vision.circle_ms));
    }

    if (g_vision.laser_count != last_pair_laser_count ||
        g_vision.circle_count != last_pair_circle_count) {
        last_pair_laser_count = g_vision.laser_count;
        last_pair_circle_count = g_vision.circle_count;
        VisionDebug_PrintGlobalPair(now);
    }

    if (g_vision.tilt_count != last_tilt_count) {
        last_tilt_count = g_vision.tilt_count;
        g_vision.tilt_fresh = 0;
        VisionDebug_PrintCoord("TILT", g_vision.tilt_count,
                               g_vision.tilt_x, g_vision.tilt_y,
                               VisionDebug_Age(now, g_vision.tilt_ms));
    }

    if (g_vision.bad_frame_count != last_bad_count) {
        last_bad_count = g_vision.bad_frame_count;
        printf("[DATA ERROR] bad=%lu reason=%s data=\"%s\"\r\n",
               (unsigned long)g_vision.bad_frame_count,
               VisionDebug_ErrorName(g_vision.last_bad_reason),
               g_vision.last_bad_frame);
    }

    if (g_vision.unknown_count != last_unknown_count) {
        last_unknown_count = g_vision.unknown_count;
        printf("[DATA ERROR] unknown_tag=%lu data=\"%s\"\r\n",
               (unsigned long)g_vision.unknown_count,
               g_vision.last_unknown_frame);
    }

    if (g_vision.uart_error_count != last_uart_error_count) {
        last_uart_error_count = g_vision.uart_error_count;
        printf("[UART2 ERROR] count=%lu HAL_error=0x%08lX\r\n",
               (unsigned long)g_vision.uart_error_count,
               (unsigned long)g_vision.last_uart_error);
    }

    if ((now - last_stat_ms) >= 1000U) {
        last_stat_ms = now;

        if (!g_vision.dma_started) {
            printf("[LINK ERROR] USART2 DMA start failed, HAL_status=%u\r\n",
                   (unsigned int)g_vision.dma_start_status);
        } else if (g_vision.rx_byte_count == 0U) {
            printf("[NO RX] USART2 received 0 bytes; check CAM_TX->PA3, GND and 115200 8N1\r\n");
        } else if (g_vision.valid_frame_count == 0U) {
            if (g_vision.frame_count == 0U) {
                printf("[LINK PARTIAL] bytes are arriving, but no complete $...# frame\r\n");
            } else {
                printf("[LINK PARTIAL] complete frames received, but none passed protocol validation\r\n");
            }
        } else if (vision_is_online(1500U)) {
            printf("[LINK OK] valid vision data is active\r\n");
        } else {
            printf("[LINK TIMEOUT] bytes may arrive, but no valid frame for %lums\r\n",
                   (unsigned long)(now - g_vision.last_valid_ms));
        }

        printf("[STAT] dma=%u rx=%lu frame=%lu valid=%lu bad=%lu unknown=%lu uart_err=%lu "
               "hb=%lu laser=%lu home=%lu fire=%lu circle=%lu tilt=%lu "
               "byte_age=%lu valid_age=%lu\r\n",
               (unsigned int)g_vision.dma_started,
               (unsigned long)g_vision.rx_byte_count,
               (unsigned long)g_vision.frame_count,
               (unsigned long)g_vision.valid_frame_count,
               (unsigned long)g_vision.bad_frame_count,
               (unsigned long)g_vision.unknown_count,
               (unsigned long)g_vision.uart_error_count,
               (unsigned long)g_vision.hb_count,
               (unsigned long)g_vision.laser_count,
               (unsigned long)g_vision.home_count,
               (unsigned long)g_vision.fire_count,
               (unsigned long)g_vision.circle_count,
               (unsigned long)g_vision.tilt_count,
               (unsigned long)VisionDebug_Age(now, g_vision.last_byte_ms),
               (unsigned long)VisionDebug_Age(now, g_vision.last_valid_ms));

        if (g_vision.home_count != 0U && g_vision.laser_count == 0U) {
            printf("[MODE CHECK] Only HOME is arriving: MaixCAM is still initial. Put laser near center circle, then press KEY0 for total.\r\n");
        }
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
  uint32_t reset_flags = RCC->CSR;
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
  setvbuf(stdout, NULL, _IONBF, 0);
  printf("\r\n[BOOT] CDPR backup-current app start Q2-CAL-JOG-v12-Q3-TXDMA1-MENU3\r\n");
  printf("RESET csr=0x%08lX pin=%u por=%u sw=%u iwdg=%u wwdg=%u lpwr=%u\r\n",
         (unsigned long)reset_flags,
         (unsigned int)((reset_flags & RCC_CSR_PINRSTF) != 0U),
         (unsigned int)((reset_flags & RCC_CSR_PORRSTF) != 0U),
         (unsigned int)((reset_flags & RCC_CSR_SFTRSTF) != 0U),
         (unsigned int)((reset_flags & RCC_CSR_IWDGRSTF) != 0U),
         (unsigned int)((reset_flags & RCC_CSR_WWDGRSTF) != 0U),
         (unsigned int)((reset_flags & RCC_CSR_LPWRRSTF) != 0U));
  __HAL_RCC_CLEAR_RESET_FLAGS();
  vision_init();      /* 要用视觉才加; 需 USART2+DMA 已初始化 */
#if APP_MOTOR_DEBUG_TEST
  MotorTest_Init();
#elif APP_VISION_DEBUG_TEST
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
#if APP_MOTOR_DEBUG_TEST
      vision_task();
      MotorTest_Loop();
#elif APP_VISION_DEBUG_TEST
      vision_task(); /* 收到 $HB,n# 时原样回发给 MaixCAM */
      VisionDebug_Task();
      HAL_Delay(5);
#else
      Task_Loop();
      vision_task(); /* 视觉心跳回发($HB), 现在用视觉, 必须开 */
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
int __io_putchar(int ch)
{
    uint8_t byte = (uint8_t)ch;
    return (HAL_UART_Transmit(&huart1, &byte, 1, 10) == HAL_OK) ? ch : EOF;
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
