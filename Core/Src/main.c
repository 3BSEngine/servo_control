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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float current_angle; // 当前累积的角度（度）
  int16_t total_turns; // 累计圈数
  uint8_t is_ok;       // 读取是否成功标志：1成功，0失败
} Servo_Feedback_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void Servo_Write_Command(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t param_len);
void Servo_Set_Multi_Turn_Position(uint8_t id, float target_angle_deg, uint32_t time_ms, uint16_t power_mw);
Servo_Feedback_t Servo_Read_Multi_Turn_Position(uint8_t id);
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
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET); // 关闭发送通道
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET); // 关闭接收通道
  Servo_Set_Multi_Turn_Position(0, -360.0f, 3000, 0);
  HAL_Delay(3500); // 等待旋转完成

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Servo_Feedback_t servo1_data;
  while (1)
  {
    /* USER CODE END WHILE */
    servo1_data = Servo_Read_Multi_Turn_Position(0);

    HAL_Delay(200);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pins : PE7 PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  新协议通用发送函数
  * @param  id: 舵机ID
  * @param  cmd: 指令编号（如 0x0D）
  * @param  params: 参数数组指针
  * @param  param_len: 参数长度
  */
void Servo_Write_Command(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t param_len)
{
  uint8_t buf[32];

  uint8_t length = param_len + 1;

  buf[0] = 0x12;     // 固定标识高字节
  buf[1] = 0x4c;     // 固定标识低字节
  buf[2] = cmd;      // 指令编号
  buf[3] = length;   // 长度
  buf[4] = id;       // 舵机 ID

  // 装载参数
  for (uint8_t i = 0; i < param_len; i++) {
    buf[5 + i] = params[i];
  }

  // 计算校验和：∑(Byte[0..14]) % 256，即除了最后一字节校验和以外的所有字节求和
  uint32_t sum = 0;
  uint8_t total_len = param_len + 5; // 帧头2 + cmd1 + len1 + id1 + params_len
  for (uint8_t i = 0; i < total_len; i++) {
    sum += buf[i];
  }
  buf[total_len] = (uint8_t)(sum % 256); // 写入校验码

  // 硬件控制：切换为发送状态
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);   // 关接收
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET); // 开发送

  // 发送数据包
  HAL_UART_Transmit(&huart6, buf, total_len + 1, 100);

  // 死等发送完成
  while(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) == RESET);

  // 延时等电平物理飘完
  for(volatile uint32_t i = 0; i < 200; i++);

  // 恢复为默认接收状态
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);   // 关发送
}

/**
  * @brief  控制舵机多圈旋转到指定角度
  * @param  id: 舵机ID
  * @param  target_angle_deg: 目标角度（支持正负、多圈，例如 720.0 度，传参 720.0）
  * @param  time_ms: 运动时间
  * @param  power_mw: 执行功率（传 0 时按照最大执行功率运行）
  */
void Servo_Set_Multi_Turn_Position(uint8_t id, float target_angle_deg, uint32_t time_ms, uint16_t power_mw)
{
  uint8_t params[10]; // position(4) + time(4) + power(2) = 10 字节

  // 1. 转换角度单位：图纸要求单位为 0.1°，int32_t 类型
  int32_t pos_val = (int32_t)(target_angle_deg * 10.0f);

  // 拆分到 params [0~3] (低字节在前，小端模式)
  params[0] = (uint8_t)(pos_val & 0xFF);
  params[1] = (uint8_t)((pos_val >> 8) & 0xFF);
  params[2] = (uint8_t)((pos_val >> 16) & 0xFF);
  params[3] = (uint8_t)((pos_val >> 24) & 0xFF);

  // 2. 转换时间：uint32_t 类型，拆分到 params [4~7]
  params[4] = (uint8_t)(time_ms & 0xFF);
  params[5] = (uint8_t)((time_ms >> 8) & 0xFF);
  params[6] = (uint8_t)((time_ms >> 16) & 0xFF);
  params[7] = (uint8_t)((time_ms >> 24) & 0xFF);

  // 3. 转换功率：uint16_t 类型，拆分到 params [8~9]
  params[8] = (uint8_t)(power_mw & 0xFF);
  params[9] = (uint8_t)((power_mw >> 8) & 0xFF);

  // 调用发送：参数长度为 10
  Servo_Write_Command(id, 0x0D, params, 10);
}

/**
  * @brief  读取舵机当前的多圈角度信息
  * @param  id: 舵机ID
  * @retval 包含角度和圈数的结构体
  */
Servo_Feedback_t Servo_Read_Multi_Turn_Position(uint8_t id)
{
  Servo_Feedback_t result = {0.0f, 0, 0};
  uint8_t send_buf[6];
  uint8_t recv_buf[12] = {0}; // 回包总长度 12 字节（帧头2 + cmd1 + len1 + id1 + pos4 + turn2 + checksum1）

  // 1. 组装读取包：指令0x10，参数长0，Length = 1(id自身)
  send_buf[0] = 0x12;
  send_buf[1] = 0x4c;
  send_buf[2] = 0x10; // cmd_id
  send_buf[3] = 0x01; // length
  send_buf[4] = id;   // servo_id
  send_buf[5] = (uint8_t)((0x12 + 0x4c + 0x10 + 0x01 + id) % 256); // 校验和

  // 2. 硬件控制切换并发送
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_UART_Transmit(&huart6, send_buf, 6, 10);

  while(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) == RESET);
  for(volatile uint32_t i = 0; i < 200; i++);

  // 3. 切换到接收状态并收取 12 字节
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);

  if (HAL_UART_Receive(&huart6, recv_buf, 12, 50) == HAL_OK)
  {
    // 4. 校验响应包
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 11; i++) {
      sum += recv_buf[i];
    }

    // 核对响应帧头 (0x05 0x1C) 以及校验和
    if (recv_buf[0] == 0x05 && recv_buf[1] == 0x1C && recv_buf[11] == (uint8_t)(sum % 256))
    {
      // 5. 解析数据 (小端模式组合)
      int32_t raw_position = (int32_t)(recv_buf[5] | (recv_buf[6] << 8) | (recv_buf[7] << 16) | (recv_buf[8] << 24));
      int16_t raw_turns = (int16_t)(recv_buf[9] | (recv_buf[10] << 8));

      // 转化为可读的浮点角度度数 (图纸说明：需除以10.0)
      result.current_angle = (float)raw_position / 10.0f;
      result.total_turns = raw_turns;
      result.is_ok = 1; // 成功
      return result;
    }
  }

  return result; // 失败返回全0
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
