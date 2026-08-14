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
#include "3bsd_kinematics.h"
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
/* 齿轮减速传动比 (1 : 8.25)，写在私有宏定义区 */
#define GEAR_REDUCTION_RATIO   8.25
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* 定义变量存储解算出的三个舵机/轴承目标角度 */
double target_w1 = 0.0;
double target_w2 = 0.0;
double target_w3 = 0.0;
/* 定义变量存储经传动比换算后，实际发给舵机的控制角度 */
float servo_cmd_angle0 = 0.0f;
float servo_cmd_angle1 = 0.0f;
float servo_cmd_angle2 = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void Servo_Write_Command(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t param_len);
void Servo_Set_Multi_Turn_Position(uint8_t id, float target_angle_deg, uint32_t time_ms, uint16_t power_mw);
Servo_Feedback_t Servo_Read_Multi_Turn_Position(uint8_t id);
void Servo_Sync_Set_Multi_Turn_Position_3CH(float angle0, float angle1, float angle2, uint32_t time_ms, uint16_t power_mw);
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
  // 测试参数：截面倾角 24度，期望下偏 60度，侧偏 10度
  double input_dN = 0.0;
  double input_dNy = 0.0;
  double param_theta = 26.25;
  //Servo_Set_Multi_Turn_Position(2, 0.0f, 3000, 0);
  //HAL_Delay(3500); // 等待旋转完成
  // 2. 调用逆运动学解算算法
  if (solveInverseKinematics(input_dN, input_dNy, param_theta, &target_w1, &target_w2, &target_w3))
  {
    // 3. 核心计算：考虑 1:8.25 传动比，换算为舵机实际目标角度
    // 假设：舵机ID 0 控制第1级轴承 (w1), ID 1 控制第2级 (w2), ID 2 控制第3级 (w3)
    servo_cmd_angle0 = (float)(target_w1 * GEAR_REDUCTION_RATIO);
    servo_cmd_angle1 = (float)(target_w2 * GEAR_REDUCTION_RATIO);
    servo_cmd_angle2 = (float)(target_w3 * GEAR_REDUCTION_RATIO);

    // 4. 三通道同步下发控制指令给总线舵机（3000ms内运动到位）
    Servo_Sync_Set_Multi_Turn_Position_3CH(servo_cmd_angle0, servo_cmd_angle2, servo_cmd_angle1, 3000, 0);
  }
  HAL_Delay(3500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Servo_Feedback_t servos_data[3];
  while (1)
  {
    /* USER CODE END WHILE */
    // 依次轮询读取 ID 0, 1, 2
    for (uint8_t id = 0; id < 3; id++)
    {
      servos_data[id] = Servo_Read_Multi_Turn_Position(id);

      // 读完一个舵机后，稍微延时 5ms 腾出总线空闲，避免发包太密产生冲突
      HAL_Delay(5);
    }

    // 整个大循环周期延时（可根据业务需求调整）
    HAL_Delay(100);
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

/**
  * @brief  同步控制三个舵机（ID:0, 1, 2）的多圈旋转角度（基于时间）
  * @param  angle0/1/2: 目标角度（支持正负、多圈，例如 720.0 度，传参 720.0）
  * @param  time_ms: 运动时间（三个舵机统一运动时间）
  * @param  power_mw: 执行功率（传 0 时按照最大执行功率运行）
  */
void Servo_Sync_Set_Multi_Turn_Position_3CH(float angle0, float angle1, float angle2, uint32_t time_ms, uint16_t power_mw)
{
  uint8_t buf[64]; // 36字节数据，开辟64字节缓冲区
  float angles[3] = {angle0, angle1, angle2};

  // 1. 外层固定协议头
  buf[0] = 0x12;     // 固定标识
  buf[1] = 0x4c;     // 固定标识
  buf[2] = 0x19;     // 同步写指令 0x19

  // 【精准计算】：内层11字节 * 3个舵机 + 3 = 36 字节
  buf[3] = 36;

  // 2. 内层配置（严格对齐 0x0D 简易多圈指令）
  buf[4] = 0x0D;     // 内层核心指令：简易多圈角度控制 (0x0D)
  buf[5] = 11;       // 内层单机长度：严格填 11
  buf[6] = 0x03;     // 舵机个数：3个

  // 3. 循环装载 3 个舵机的数据，从 buf[7] 开始
  uint8_t base_idx = 7;

  for (uint8_t id = 0; id < 3; id++)
  {
    buf[base_idx++] = id; // 写入舵机 ID（0, 1, 2）

    // position: 目标位置（4 字节 int32_t，单位0.1°，小端模式）
    int32_t pos_val = (int32_t)(angles[id] * 10.0f);
    buf[base_idx++] = (uint8_t)(pos_val & 0xFF);
    buf[base_idx++] = (uint8_t)((pos_val >> 8) & 0xFF);
    buf[base_idx++] = (uint8_t)((pos_val >> 16) & 0xFF);
    buf[base_idx++] = (uint8_t)((pos_val >> 24) & 0xFF);

    // time: 运动时间（4 字节 uint32_t，单位ms，小端模式）
    buf[base_idx++] = (uint8_t)(time_ms & 0xFF);
    buf[base_idx++] = (uint8_t)((time_ms >> 8) & 0xFF);
    buf[base_idx++] = (uint8_t)((time_ms >> 16) & 0xFF);
    buf[base_idx++] = (uint8_t)((time_ms >> 24) & 0xFF);

    // power: 执行功率（2 字节 uint16_t，小端模式）
    buf[base_idx++] = (uint8_t)(power_mw & 0xFF);
    buf[base_idx++] = (uint8_t)((power_mw >> 8) & 0xFF);
  }

  // 4. 计算整体校验和
  uint32_t sum = 0;
  for (uint8_t i = 0; i < base_idx; i++) {
    sum += buf[i];
  }
  buf[base_idx] = (uint8_t)(sum % 256); // 写入最后一字节校验码

  // 5. 硬件切换为发送状态
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);

  // 6. 串口发送（总长 base_idx + 1 字节）
  HAL_UART_Transmit(&huart6, buf, base_idx + 1, 200);

  // 等待硬件发送完毕
  while(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) == RESET);
  for(volatile uint32_t i = 0; i < 200; i++);

  // 恢复默认接收状态
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET); // 👈 加上这一行：开接收
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
