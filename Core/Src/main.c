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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu_decode.h"
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
#define RX_BUF_SIZE 1024     // ★ 512 -> 1024，1 ms 轮询下留足余量
uint8_t rx_buf[RX_BUF_SIZE]; // DMA 循环写入的缓冲区
uint16_t rx_read_pos = 0;    // ★ 已解码到的位置（上次解码结束处）

volatile uint32_t g_rx_lap = 0;     // ★ 环形缓冲走满一圈的次数
static uint32_t s_total_w_prev = 0; // ★ 上次轮询的绝对写入位置，用于发现溢出

imu_data_t g_imu = {};            // 最新解出的一帧 IMU 数据
volatile uint8_t g_imu_ready = 0; // 新帧标志（1 = 有）
// LED状态
uint8_t LED_R_state = 0;
uint8_t LED_G_state = 1;
uint8_t LED_B_state = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2); // 开启定时器2
  HAL_TIM_Base_Start_IT(&htim3); // 开启定时器3
  // 开启PWM输出
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  // 启动接收
  if (HAL_UART_Receive_DMA(&huart1, rx_buf, RX_BUF_SIZE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
imu_data_t imu_data = {};
uint16_t rx_ok_n = 0;

/* ★ 循环模式下 DMA 每走满一整圈触发一次 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    g_rx_lap++;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    rx_ok_n = get_receive_ok_count();
    clear_receive_count();  
    if (LED_R_state == 0){
      HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
      LED_R_state = 1;
    }
    else{
      HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
      LED_R_state = 0;
    }
    if (LED_G_state == 0)
    {
      HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
      LED_G_state = 1;
    }
    else
    {
      HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
      LED_G_state = 0;
    }
  }
  // 定时器3中断回调函数：每 1ms 取一次 DMA 新收到的字节，拼帧并解码
  if (htim->Instance == TIM3)
  {

    uint32_t lap;
    uint16_t wpos;
    uint32_t total_w;

    /* 1) 取 DMA 写指针；和 lap 一起读，避免刚好绕圈时读数不一致 */
    do
    {
      lap = g_rx_lap;
      wpos = imu_dma_write_pos((uint16_t)__HAL_DMA_GET_COUNTER(&hdma_usart1_rx),
                               RX_BUF_SIZE);
    } while (lap != g_rx_lap);

    /* 2) 溢出检测：两轮之间 DMA 前进超过一整圈，说明未读数据已被覆盖 */
    total_w = lap * (uint32_t)RX_BUF_SIZE + wpos;
    if ((total_w - s_total_w_prev) > (uint32_t)RX_BUF_SIZE)
    {
      g_imu_rx_stat.overrun++;
      rx_read_pos = wpos; /* 旧数据不可信，从当前写指针重新同步 */
    }
    s_total_w_prev = total_w;

    /* 3) 只解码 [rx_read_pos, wpos) 这段新数据，窗口内所有整帧一次解完 */
    if (imu_decode_poll(rx_buf, RX_BUF_SIZE, &rx_read_pos, wpos, &imu_data) > 0)
    {
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,
                            (uint16_t)((imu_data.rpy[0] + 180.0f) * 100.0f));
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,
                            (uint16_t)((imu_data.rpy[1] + 180.0f) * 100.0f));
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,
                            (uint16_t)((imu_data.rpy[2] + 180.0f) * 100.0f));
    }
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
#ifdef USE_FULL_ASSERT
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
