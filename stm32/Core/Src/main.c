/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 智能救援车底盘控�? (ROS2 工业�? PID 闭环完整�?)
  * @note           : 包含严格�? 20ms 控制任务、PID 闭环、原子操作防死锁
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_PPR 1320.0f  // 编码器单圈�?�脉冲数 (线数*减�?�比*4)
#define CTRL_DT_S   0.02f    // 控制周期 20ms (0.02�?)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// ==================== 1. 通信与状态变�? ====================
uint8_t rx_buffer[10];
uint8_t rx_byte;
uint8_t rx_idx = 0;

volatile int16_t target_vx = 0;
volatile int16_t target_vy = 0;
volatile int16_t target_w  = 0;

volatile uint32_t last_cmd_time = 0; // 看门狗安全时间戳
uint32_t last_ctrl_time = 0;         // 20ms 任务调度时间�?
uint32_t last_upload_time = 0;       // 编码器上传调度时间戳

uint8_t tx_buffer[12];

// ==================== 2. PID 与电机闭环结构体 ====================
typedef struct {
    float Kp, Ki, Kd;
    float err, last_err, integral;
    float max_integral, max_out;
} PID_TypeDef;

typedef struct {
    PID_TypeDef pid;
    float target_rpm;   // 目标转�??
    float current_rpm;  // 真实反馈转�??
    int16_t pwm_out;    // 输出占空�? (-100 ~ 100)
    int16_t enc_delta;  // 20ms 内的原始脉冲增量
} Motor_TypeDef;

// 数组映射: 0:左前(LF), 1:右前(RF), 2:左后(LR), 3:右后(RR)
Motor_TypeDef motors[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
// ==================== 闭环控制核心函数声明 ====================
void PID_Init(void);
void Control_Task_20ms(void);
void Kinematics_Update(void);
void Encoder_Update(void);
void PID_Update(void);
float PID_Calc(PID_TypeDef *pid, float target, float measure);
void Chassis_SetPWM(int speed_lf, int speed_lr, int speed_rf, int speed_rr);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ==================== 控制系统完整实现 ====================

// 1. PID 参数初始�? (上电执行�?�?)
void PID_Init(void) {
    for (int i = 0; i < 4; i++) {
        motors[i].pid.Kp = 1.2f;
        motors[i].pid.Ki = 0.1f;
        motors[i].pid.Kd = 0.02f;
        motors[i].pid.max_out = 100.0f;     // �?�? PWM 输出限制
        motors[i].pid.max_integral = 50.0f; // 积分抗饱和限�?

        motors[i].pid.integral = 0.0f;
        motors[i].pid.last_err = 0.0f;
        motors[i].target_rpm = 0.0f;
        motors[i].current_rpm = 0.0f;
        motors[i].pwm_out = 0;
    }
}

// 2. 运动学�?�解：将 ROS 目标速度转化为四轮目�? RPM
void Kinematics_Update(void) {
    // 保护：原子操作读取串口更新的全局变量，防止中断数据撕�?
    __disable_irq();
    int16_t vx = target_vx;
    int16_t vy = target_vy;
    int16_t w  = target_w;
    __enable_irq();

    // 500ms 看门狗：断联自动停车
    if (HAL_GetTick() - last_cmd_time > 500) {
        vx = 0; vy = 0; w = 0;
    }

    // 麦克纳姆轮标准�?�解 (注意对应你的实际安装极�??)
    motors[0].target_rpm = vx + vy - w; // 0: LF
    motors[1].target_rpm = vx - vy + w; // 1: RF
    motors[2].target_rpm = vx - vy - w; // 2: LR
    motors[3].target_rpm = vx + vy + w; // 3: RR
}

// 3. 编码器测速：读取脉冲并转换为真实 RPM
void Encoder_Update(void) {
    // 读取原始脉冲
    motors[0].enc_delta = (int16_t)__HAL_TIM_GET_COUNTER(&htim1); // LF
    motors[1].enc_delta = (int16_t)__HAL_TIM_GET_COUNTER(&htim2); // RF
    motors[2].enc_delta = (int16_t)__HAL_TIM_GET_COUNTER(&htim5); // LR
    motors[3].enc_delta = (int16_t)__HAL_TIM_GET_COUNTER(&htim8); // RR

    // 立即清空硬件计数器，为下�?�? 20ms 周期准备
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    __HAL_TIM_SET_COUNTER(&htim8, 0);

    // 转化为真�? RPM�?(脉冲�? / 总分辨率) * (1 / 0.02�?) * 60�?
    float rpm_coeff = (1.0f / CTRL_DT_S) * 60.0f / ENCODER_PPR;
    for(int i = 0; i < 4; i++) {
        motors[i].current_rpm = motors[i].enc_delta * rpm_coeff;
    }
}

// 4. PID 计算与更�?
void PID_Update(void) {
    for (int i = 0; i < 4; i++) {
        motors[i].pwm_out = (int16_t)PID_Calc(&motors[i].pid, motors[i].target_rpm, motors[i].current_rpm);
    }
}

// 5. 组装 20ms 完整控制任务
void Control_Task_20ms(void) {
    Kinematics_Update();  // 步骤 A: 刷新四轮 target_rpm
    Encoder_Update();     // 步骤 B: 刷新四轮 current_rpm
    PID_Update();         // 步骤 C: 计算四轮 pwm_out

    // 步骤 D: 物理输出给电机驱�? (注意参数传�?�顺�? LF, LR, RF, RR)
    Chassis_SetPWM(motors[0].pwm_out, motors[2].pwm_out, motors[1].pwm_out, motors[3].pwm_out);
}

// 6. PID 核心算式实现
float PID_Calc(PID_TypeDef *pid, float target, float measure) {
    pid->err = target - measure;

    pid->integral += pid->err;
    if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    else if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

    float out = (pid->Kp * pid->err) + (pid->Ki * pid->integral) + (pid->Kd * (pid->err - pid->last_err));
    pid->last_err = pid->err;

    if (out > pid->max_out) out = pid->max_out;
    else if (out < -pid->max_out) out = -pid->max_out;

    return out;
}

// 7. AT8236 底层 PWM 映射
void Chassis_SetPWM(int speed_lf, int speed_lr, int speed_rf, int speed_rr) {
    uint32_t arr_val = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1; // 修正 ARR 获取

    if(speed_lf > 100) speed_lf = 100; else if(speed_lf < -100) speed_lf = -100;
    if(speed_lr > 100) speed_lr = 100; else if(speed_lr < -100) speed_lr = -100;
    if(speed_rf > 100) speed_rf = 100; else if(speed_rf < -100) speed_rf = -100;
    if(speed_rr > 100) speed_rr = 100; else if(speed_rr < -100) speed_rr = -100;

    uint32_t pwm_lf = (abs(speed_lf) * arr_val) / 100;
    uint32_t pwm_lr = (abs(speed_lr) * arr_val) / 100;
    uint32_t pwm_rf = (abs(speed_rf) * arr_val) / 100;
    uint32_t pwm_rr = (abs(speed_rr) * arr_val) / 100;

    // 左前 (TIM4 CH1/2) - PD12/PD13
    if (speed_lf > 0) { __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_lf); __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0); }
    else if (speed_lf < 0) { __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0); __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm_lf); }
    else { __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0); __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0); }

    // 左后 (TIM4 CH3/4) - PD14/PD15
    if (speed_lr > 0) { __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_lr); __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0); }
    else if (speed_lr < 0) { __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm_lr); }
    else { __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0); }

    // 右前 (TIM3 CH1/2) - PA6/PA7
    if (speed_rf > 0) { __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_rf); }
    else if (speed_rf < 0) { __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_rf); __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); }
    else { __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); }

    // 右后 (TIM3 CH3/4) - PB0/PB1
    if (speed_rr > 0) { __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_rr); }
    else if (speed_rr < 0) { __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_rr); __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0); }
    else { __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0); }
}
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  // 1. 初始�? PID 参数结构�?
  PID_Init();

  // 2. 启动 8 �? PWM
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  Chassis_SetPWM(0, 0, 0, 0);

  // 3. 启动 4 个编码器
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

  // 4. 暴力清空串口错误标志，防止刚上电就死�?
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  __HAL_UART_CLEAR_NEFLAG(&huart1);
  __HAL_UART_CLEAR_FEFLAG(&huart1);
  __HAL_UART_CLEAR_PEFLAG(&huart1);

  // 5. �?�? 1 字节接收中断
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  // 记录启动时间�?
  last_ctrl_time = HAL_GetTick();
  last_upload_time = HAL_GetTick();
  last_cmd_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t current_time = HAL_GetTick();

      // ==============================================================
      // 核心任务：严�? 20ms 闭环调度任务 (频率 50Hz)
      // ==============================================================
      if (current_time - last_ctrl_time >= 20) {
          last_ctrl_time = current_time;

          Control_Task_20ms(); // 触发全套闭环解算：目�?->测�??->PID->PWM
      }

      // ==============================================================
      // ROS 上传任务�?20ms 频率上传编码器原始脉�?
      // ==============================================================
      if (current_time - last_upload_time >= 20) {
          last_upload_time = current_time;

          tx_buffer[0] = 0xAA;
          tx_buffer[1] = 0x66;
          // 直接使用�? Encoder_Update 中缓存下来的真实增量�?
          tx_buffer[2] = (motors[0].enc_delta >> 8) & 0xFF;
          tx_buffer[3] = motors[0].enc_delta & 0xFF;
          tx_buffer[4] = (motors[1].enc_delta >> 8) & 0xFF;
          tx_buffer[5] = motors[1].enc_delta & 0xFF;
          tx_buffer[6] = (motors[2].enc_delta >> 8) & 0xFF;
          tx_buffer[7] = motors[2].enc_delta & 0xFF;
          tx_buffer[8] = (motors[3].enc_delta >> 8) & 0xFF;
          tx_buffer[9] = motors[3].enc_delta & 0xFF;

          uint8_t sum = 0;
          for (int i = 2; i <= 9; i++) sum += tx_buffer[i];

          tx_buffer[10] = sum;
          tx_buffer[11] = 0x0D;

          HAL_UART_Transmit(&huart1, tx_buffer, 12, 10);
      }

      // ==============================================================
      // 串口防聋守护：如果由于静�?/干扰引发错误标志，强制重启中�?
      // ==============================================================
      if (huart1.RxState != HAL_UART_STATE_BUSY_RX) {
          __HAL_UART_CLEAR_OREFLAG(&huart1);
          __HAL_UART_CLEAR_NEFLAG(&huart1);
          __HAL_UART_CLEAR_FEFLAG(&huart1);
          huart1.RxState = HAL_UART_STATE_READY;
          huart1.Lock = HAL_UNLOCKED;
          HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
      }
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
// ==============================================================
// 串口中断解析机制：滑动窗口解决脏数据及错位问�?
// ==============================================================
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {

        // 新数据从尾部推入
        rx_buffer[9] = rx_byte;

        // 校验 10 字节协议�?
        if(rx_buffer[0] == 0xAA && rx_buffer[1] == 0x55 && rx_buffer[9] == 0x0D) {
            uint8_t sum = 0;
            for(int i = 2; i <= 7; i++) sum += rx_buffer[i];

            if(sum == rx_buffer[8]) {
                target_vx = (int16_t)((rx_buffer[2] << 8) | rx_buffer[3]);
                target_vy = (int16_t)((rx_buffer[4] << 8) | rx_buffer[5]);
                target_w  = (int16_t)((rx_buffer[6] << 8) | rx_buffer[7]);

                last_cmd_time = HAL_GetTick(); // 更新通信安全看门�?

                // 清空缓冲区，防止同一有效包被重复识别
                for(int i = 0; i < 10; i++) {
                    rx_buffer[i] = 0;
                }
            } else {
                // 校验失败，整体前移一格丢弃脏数据
                for(int i = 0; i < 9; i++) rx_buffer[i] = rx_buffer[i+1];
            }
        } else {
            // 包头尾未对齐，整体前移一格滑�?
            for(int i = 0; i < 9; i++) rx_buffer[i] = rx_buffer[i+1];
        }

        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

// 错误中断兜底回调
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        HAL_UART_AbortReceive_IT(huart);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

