/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Mã nguồn điều khiển AMR / Differential Drive Robot (Tối ưu hóa)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050/mpu6050.h"
#include "encoder.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float alpha_d;
} PID_Config_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARR_MAX_L       4199
#define PID_LIMIT_L     ARR_MAX_L

#define ARR_MAX_R       2099
#define PID_LIMIT_R     ARR_MAX_R

#define OFFSET_PWM_L    320.0f
#define OFFSET_PWM_R    90.0f   

#define Ts              0.01f   // Chu kỳ lấy mẫu 10ms
#define ARR_MIN         10
#define Base_Link       0.31548f
#define WHEEL_RADIUS    0.05f
#define DEG_TO_RAD      0.0174532925f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t buzzer_done_flag = 0;

/*=========================== MPU6050 PARAMETERS =============================*/
extern volatile float Final_Gyro_Z;
extern signed char Is_Calibrated;
extern float Robot_Yaw;
volatile signed char mpu_data_ready_flag = 0;

/*=========================== ENCODER PARAMETERS =============================*/
Encoder_t LeftEncoder;
Encoder_t RightEncoder;

float left_rad_raw, left_vel_raw;
float left_rad_filt = 0.0f, left_vel_filt = 0.0f;
float alpha_vel_left = 0.3f;

float right_rad_raw, right_vel_raw;
float right_rad_filt = 0.0f, right_vel_filt = 0.0f;
float alpha_vel_right = 0.3f;

/*=========================== PI VELOCITY PARAMETERS =========================*/
volatile uint8_t pid_trigger = 0;

// Motor Trái
float Kp_L = 260.0f; 
float Ki_L = 3800.0f; 
float Kb_L = 0.0f;

float error_sat_p_L = 0.0f;
float error_sat_L = 0.0f;

float ui_p_L = 0.0f;

volatile int16_t motor_L_output = 0;
volatile float setpoint_L = 0.0f;

// Motor Phải
float Kp_R = 145.0f; 
float Ki_R = 3500.0f;
float Kb_R = 0.0f;

float error_sat_p_R = 0.0f;
float error_sat_R = 0.0f;

float ui_p_R = 0.0f;

volatile int16_t motor_R_output = 0;
volatile float setpoint_R = 0.0f;

/*=========================== PID ANGLE PARAMETERS ===========================*/
float Kp_angle = 0.1f; 
float Ki_angle = 0.0f; 
float Kd_angle = 0.0f; 
float Kb_angle = 0.0f;

float alpha_d_angle = 0.15f;
float angle_error = 0.0f; 
float angle_last_error = 0.0f; 
float angle_integral = 0.0f;

const float PID_LIMIT_ANGLE = 20.0f;
float error_sat_p_angle = 0.0f;
float last_ud_f_angle = 0.0f;

/*=========================== UART & ODOMETRY ===============================*/
// Rx UART1 (Control Command)
uint8_t rxFrame[10];
uint8_t uart_flag = 0;
float linear_x = 0.0f, angular_z = 0.0f;
volatile uint16_t uart_timeout_counter = 0;

// Tx UART6 (Odometry Data)
uint8_t txFrame[14];
float x = 0.0f, y = 0.0f;
float omega = 0.0f, theta = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART6_UART_Init(void);

/* USER CODE BEGIN PFP */
void encoder_process(void);
void PI_vel(float rad_L, float rad_R);
float PID_angle(float setpoint, float feedback, float dt);
void Set_Motor(int16_t left_output, int16_t right_output);
void AMR_Teleop_Process(void);
void Update_Odometry(void);
void UART6_Send_Odom(float x_pos, float y_pos, float theta_pos);
void Read_DMP(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ========================================================================== */
/*                          INTERRUPT CALLBACKS                               */
/* ========================================================================== */

// Ngắt EXTI MPU6050 (Data Ready)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU_INT_Pin)
    {
        mpu_data_ready_flag = 1;
    }
}

// Ngắt Timer 5 chu kỳ 10ms (Control Loop Trigger)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        pid_trigger = 1;
    }
}

// Ngắt Nhận UART DMA Idle (Lấy lệnh điều khiển)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxFrame, 10);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
        uart_flag = 1;

        if (rxFrame[0] == 0xAA && rxFrame[9] == 0x0D)
        {
            uint8_t *p;

            p = (uint8_t *)&linear_x;
            p[0] = rxFrame[1]; p[1] = rxFrame[2]; p[2] = rxFrame[3]; p[3] = rxFrame[4];

            p = (uint8_t *)&angular_z;
            p[0] = rxFrame[5]; p[1] = rxFrame[6]; p[2] = rxFrame[7]; p[3] = rxFrame[8];

            uart_timeout_counter = 0;
        }
    }
}

/* ========================================================================== */
/*                       ROBOT CONTROL & PROCESSING                           */
/* ========================================================================== */

// Tính vận tốc bánh xe từ xung Encoder
void encoder_process(void)
{
    Encoder_Update(&LeftEncoder);
    Encoder_Update(&RightEncoder);

    left_rad_raw  = LeftEncoder.speed_rad_s;
    left_vel_raw  = LeftEncoder.velocity_m_s;
    left_rad_filt = (alpha_vel_left * left_rad_raw) + ((1.0f - alpha_vel_left) * left_rad_filt);
    left_vel_filt = (alpha_vel_left * left_vel_raw) + ((1.0f - alpha_vel_left) * left_vel_filt);

    right_rad_raw  = RightEncoder.speed_rad_s;
    right_vel_raw  = RightEncoder.velocity_m_s;
    right_rad_filt = (alpha_vel_right * right_rad_raw) + ((1.0f - alpha_vel_right) * right_rad_filt);
    right_vel_filt = (alpha_vel_right * right_vel_raw) + ((1.0f - alpha_vel_right) * right_vel_filt);
}

// Bộ điều khiển PI Vận tốc động cơ
void PI_vel(float rad_L, float rad_R)
{
    static uint8_t was_running_L = 0;
    static uint8_t was_running_R = 0;

    // ------------------ 1. BÁNH TRÁI ------------------
    if (fabsf(setpoint_L) > 0.05f)
    {
        if (!was_running_L)
        {
            ui_p_L = 0.0f;
            error_sat_p_L = 0.0f;
            was_running_L = 1;
        }

        float offset_L = (setpoint_L > 0.0f) ? OFFSET_PWM_L : -OFFSET_PWM_L;
        float error_L = setpoint_L - rad_L;
        float up_L = Kp_L * error_L;
        float ui_L = ui_p_L + Ts * (Ki_L * error_L + Kb_L * error_sat_p_L);

        float out_L_raw = offset_L + up_L + ui_L;

        if (out_L_raw > PID_LIMIT_L) {
            motor_L_output = PID_LIMIT_L;
            error_sat_L = PID_LIMIT_L - out_L_raw;
        } else if (out_L_raw < -PID_LIMIT_L) {
            motor_L_output = -PID_LIMIT_L;
            error_sat_L = -PID_LIMIT_L - out_L_raw;
        } else {
            motor_L_output = (int16_t)out_L_raw;
            error_sat_L = 0.0f;
        }

        ui_p_L = ui_L;
        error_sat_p_L = error_sat_L;
    }
    else
    {
        motor_L_output = 0;
        ui_p_L = 0.0f;
        error_sat_p_L = 0.0f;
        was_running_L = 0;
    }

    // ------------------ 2. BÁNH PHẢI ------------------
    if (fabsf(setpoint_R) > 0.05f)
    {
        if (!was_running_R)
        {
            ui_p_R = 0.0f;
            error_sat_p_R = 0.0f;
            was_running_R = 1;
        }

        float offset_R = (setpoint_R > 0.0f) ? OFFSET_PWM_R : -OFFSET_PWM_R;
        float error_R = setpoint_R - rad_R;
        float up_R = Kp_R * error_R;
        float ui_R = ui_p_R + Ts * (Ki_R * error_R + Kb_R * error_sat_p_R);

        float out_R_raw = offset_R + up_R + ui_R;

        if (out_R_raw > PID_LIMIT_R) {
            motor_R_output = PID_LIMIT_R;
            error_sat_R = PID_LIMIT_R - out_R_raw;
        } else if (out_R_raw < -PID_LIMIT_R) {
            motor_R_output = -PID_LIMIT_R;
            error_sat_R = -PID_LIMIT_R - out_R_raw;
        } else {
            motor_R_output = (int16_t)out_R_raw;
            error_sat_R = 0.0f;
        }

        ui_p_R = ui_R;
        error_sat_p_R = error_sat_R;
    }
    else
    {
        motor_R_output = 0;
        ui_p_R = 0.0f;
        error_sat_p_R = 0.0f;
        was_running_R = 0;
    }
}

// Bộ điều khiển PID Góc Hướng
//float PID_angle(float setpoint, float feedback, float dt)
//{
//    angle_error = setpoint - feedback;

//    while (angle_error > 180.0f)  angle_error -= 360.0f;
//    while (angle_error < -180.0f) angle_error += 360.0f;

//    float up_angle = Kp_angle * angle_error;
//    float ui_angle = angle_integral + dt * (Ki_angle * angle_error + Kb_angle * error_sat_p_angle);
//    float ud_angle_raw = Kd_angle * (angle_error - angle_last_error) / dt;

//    float ud_angle_filtered = (alpha_d_angle * ud_angle_raw) + ((1.0f - alpha_d_angle) * last_ud_f_angle);
//    last_ud_f_angle = ud_angle_filtered;

//    float out_angle_raw = up_angle + ui_angle + ud_angle_filtered;
//    float angle_output = 0.0f;
//    float error_sat_angle = 0.0f;

//    if (out_angle_raw > PID_LIMIT_ANGLE) {
//        angle_output = PID_LIMIT_ANGLE;
//        error_sat_angle = PID_LIMIT_ANGLE - out_angle_raw;
//    }
//    else if (out_angle_raw < -PID_LIMIT_ANGLE) {
//        angle_output = -PID_LIMIT_ANGLE;
//        error_sat_angle = -PID_LIMIT_ANGLE - out_angle_raw;
//    }
//    else {
//        angle_output = out_angle_raw;
//        error_sat_angle = 0.0f;
//    }

//    angle_integral = ui_angle;
//    error_sat_p_angle = error_sat_angle;
//    angle_last_error = angle_error;

//    return angle_output;
//}

// Xuất PWM tới Cầu H Động Cơ
void Set_Motor(int16_t left_output, int16_t right_output)
{
    // Động cơ Trái (TIM8 CH3/CH4)
    if (left_output >= 0) {
        if (left_output > ARR_MAX_L) left_output = ARR_MAX_L;
        TIM8->CCR3 = ARR_MIN;
        TIM8->CCR4 = left_output;
    } else {
        left_output = -left_output;
        if (left_output > ARR_MAX_L) left_output = ARR_MAX_L;
        TIM8->CCR3 = left_output;
        TIM8->CCR4 = ARR_MIN;
    }

    // Động cơ Phải (TIM4 CH3/CH4)
    if (right_output >= 0) {
        if (right_output > ARR_MAX_R) right_output = ARR_MAX_R;
        TIM4->CCR3 = right_output;
        TIM4->CCR4 = ARR_MIN;
    } else {
        right_output = -right_output;
        if (right_output > ARR_MAX_R) right_output = ARR_MAX_R;
        TIM4->CCR3 = ARR_MIN;
        TIM4->CCR4 = right_output;
    }
}

// Xử lý Teleop (Chuyển đổi v_robot, omega -> v_wheel)
void AMR_Teleop_Process(void)
{
    // Kiểm tra Timeout An toàn (Mất kết nối quá 200ms -> Dừng)
    uart_timeout_counter += 10;
    if (uart_timeout_counter > 200)
    {
        linear_x  = 0.0f;
        angular_z = 0.0f;
    }

    // Kinematics Differential Drive
    float v_left_m_s  = linear_x - (angular_z * Base_Link * 0.5f);
    float v_right_m_s = linear_x + (angular_z * Base_Link * 0.5f);

    setpoint_L = v_left_m_s / WHEEL_RADIUS;
    setpoint_R = v_right_m_s / WHEEL_RADIUS;
}

// Cập nhật vị trí Odometry (X, Y, Theta)
void Update_Odometry(void)
{
    omega = Final_Gyro_Z * DEG_TO_RAD;
    theta = Robot_Yaw * DEG_TO_RAD;

    float v_robot = (left_vel_filt + right_vel_filt) / 2.0f;

    x += v_robot * cosf(theta) * Ts;
    y += v_robot * sinf(theta) * Ts;

    // Chuẩn hóa theta trong khoảng [-PI, PI]
    while (theta >  3.14159265f) theta -= 2.0f * 3.14159265f;
    while (theta < -3.14159265f) theta += 2.0f * 3.14159265f;
}

// Gửi Odometry qua UART6
void UART6_Send_Odom(float x_pos, float y_pos, float theta_pos)
{
    txFrame[0] = 0xAA;
    *(float*)&txFrame[1] = x_pos;
    *(float*)&txFrame[5] = y_pos;
    *(float*)&txFrame[9] = theta_pos;
    txFrame[13] = 0x0D;

    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart6, txFrame, sizeof(txFrame), 10);
    if (status != HAL_OK) {
        GPIOA->ODR |= (1 << 2); // Báo lỗi truyền UART
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */
  GPIOA->ODR &=~ (1 << 2);   // Turn ON Power LED
  GPIOE->ODR &=~ (1 << 1);   // Enable MPU6050

  MPU6050_initialize(); 
  DMP_Init();           

  // Start PWM Motor Channels
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

  // Start Encoders
  Encoder_Init(&LeftEncoder, TIM2);
  Encoder_Init(&RightEncoder, TIM3);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  // Start Control Timer (10ms Interrupt)
  HAL_TIM_Base_Start_IT(&htim5);

  // Initial Anti-windup Gain
  Kb_L = Ki_L / Kp_L;
  Kb_R = Ki_R / Kp_R;
  Kb_angle = Ki_angle / Kp_angle;

  // Start DMA Receive for Teleop Control
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxFrame, 10);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 1. Xử lý dữ liệu DMP MPU6050 */
    if (mpu_data_ready_flag == 1)
    {
        mpu_data_ready_flag = 0;
        Read_DMP(); 
    }

    /* 2. Báo hiệu Buzz khi kết thúc Calibration */
    if (Is_Calibrated == 1 && buzzer_done_flag == 0)
    {
        buzzer_done_flag = 1;
        for (int i = 0; i < 3; i++) {   
            GPIOE->ODR |= (1 << 2); 
            HAL_Delay(50);             
            GPIOE->ODR &=~ (1 << 2); 
            HAL_Delay(50);
        }
    }

    /* 3. Khi chưa Calib xong -> Khóa xe */
    if (Is_Calibrated == 0) 
    {
        setpoint_L = 0.0f; 
        setpoint_R = 0.0f;
        motor_L_output = 0; 
        motor_R_output = 0;
        Set_Motor(0, 0);

        if (pid_trigger == 1) {
            pid_trigger = 0;
            encoder_process(); 
        }
        continue; 
    }

    /* 4. Vòng lặp điều khiển chính (Chạy định kỳ mỗi 10ms) */
    if (pid_trigger == 1)
    {
        pid_trigger = 0;

        encoder_process();
        Update_Odometry();
        AMR_Teleop_Process();

        PI_vel(left_rad_filt, right_rad_filt);
        Set_Motor(motor_L_output, motor_R_output);

        // Gửi Odometry định kỳ mỗi 20ms (chu kỳ 2 lần pid_trigger)
        static uint8_t send_odom_counter = 0;
        if (++send_odom_counter >= 2) {
            send_odom_counter = 0;
            UART6_Send_Odom(x, y, theta); 
        }
    }
  }
  /* USER CODE END 3 */
}

/* System Initialization Functions below remain unchanged */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

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
}

static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM4_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4199;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim4);
}

static void MX_TIM5_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 8399;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 99;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM8_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 8399;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim8);
}

static void MX_USART1_UART_Init(void)
{
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
}

static void MX_USART6_UART_Init(void)
{
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
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, BUZZER_Pin|MPU_Enable_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = BUZZER_Pin|MPU_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MPU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU_INT_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif