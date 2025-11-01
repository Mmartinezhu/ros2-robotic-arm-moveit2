/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "as5600.h"
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static int32_t last_cnt[3]    = {0,0,0};   // cuenta anterior (para D)
static uint8_t settle_ctr[3]  = {0,0,0};   // ciclos asentado
void error_loop(){}
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

//osThreadId_t defaultTaskHandle;


typedef struct {
    int32_t target_encoder_count;
    bool moving;
    float current_angle;
    float target_angle;
    uint32_t counts_per_revolution;
    TIM_HandleTypeDef* encoder_tim;
} DCMotorController;

DCMotorController dc_motors[3];



void simple_motor_test(void);
void update_dc_motors(void);
void init_sensors(void);
void stop(void);
void init_dc_motors(void);
void start_dc_move_to_angle(uint8_t motor_id, float angle);
void update_encoder_angles(void);
int get_motor_direction(uint8_t motor_id);
void test_motors_manual(void);
void move_to_left(uint8_t motor_id);
void move_to_right(uint8_t motor_id);
void stop_motor(uint8_t motor_id);
void stop_dc_motor(uint8_t motor_id);
void move_joint_dc_base(float target_angle);
void move_joint_dc_1(float target_angle);
void move_joint_dc_2(float target_angle);
void move_joint_dc_3(float target_angle);
void move_servo(int id, float target_angle);
void publish_angle(void);
void motor_angles_callback(const void *msgin);




#define COUNTS_PER_REV_MOTOR1 352000
#define COUNTS_PER_REV_MOTOR2 8000
#define COUNTS_PER_REV_MOTOR3 30000
#define MAX_PWM 2200
#define ENCODER_TOLERANCE 2
#define KP 8.0f
#define KD 12.0f
#define PWM_MIN 500
#define VEL_TOLERANCE 8
#define SETTLE_CYCLES 12

rosidl_runtime_c__String joint_names[5];
double joint_positions[5];

int32_t angle_to_encoder_counts(float angle);
float encoder_counts_to_angle(int32_t counts);


rcl_publisher_t encoder_angles_publisher;
std_msgs__msg__Float32MultiArray encoder_angles_msg;


rcl_subscription_t joint_sub;
trajectory_msgs__msg__JointTrajectory joint_msg;


uint32_t counter = 0;
float encoder_dc1 = 0.0f;
float encoder_dc2 = 0.0f;
float encoder_dc3 = 0.0f;
float current_angle_dc  = 0;
float target_angle_dc =0;
float last_servo1_angle = 0.0f;
float last_servo2_angle = 0.0f;
float encoder_data[3] = {0.0f, 0.0f, 0.0f};
float target_joint_angles[5] = {0, 0, 0, 0, 0};
float current_joint_angles[5] = {0, 0, 0, 0, 0};
float encoder1_angle = 0;
float encoder2_angle = 0;
float encoder3_angle = 0;

bool  position_reached = true;
bool new_target_available = false;

uint32_t total_pulses = 0;
uint32_t last_encoder_count = 0;


AS5600_TypeDef sensor1;
AS5600_TypeDef sensor2;
AS5600_TypeDef sensor3;



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  /* PWM motores */
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1); // Motor de la base der
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2); // Motor de la base izq
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3); // Motor joint 1 der
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4); // Motor joint 1 izq
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // Motor joint 2 der
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); // Motor joint 2 izq

  /* Duty inicial 0 */
  __HAL_TIM_SET_COMPARE(&htim5,  TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim5,  TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim5,  TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim5,  TIM_CHANNEL_4, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);


  init_dc_motors();

  init_sensors();




  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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

/* USER CODE BEGIN 4 */
static inline uint32_t clamp_pwm(int32_t mag) {
  if (mag < 0) mag = -mag;
  if (mag > 0 && mag < PWM_MIN) mag = PWM_MIN;  // vence fricción
  if (mag > MAX_PWM) mag = MAX_PWM;
  return (uint32_t)mag;
}

static void motor_set_pwm(uint8_t i, int32_t u)
{
  uint32_t mag = clamp_pwm(u);

  switch(i) {
    case 0: /* Base: TIM5_CH1/CH2 */
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, (u>0)? mag:0);  // RPWM
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, (u<0)? mag:0);  // LPWM
      break;

    case 1: /* Motor1: TIM5_CH3 / TIM5_CH4 */
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, (u<0)? mag:0); // derecha -> TIM10
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, (u>0)? mag:0); // izquierda -> TIM11
      break;

    case 2: /* Motor2: TIM8_CH1/CH2 */
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (u>0)? mag:0);  // RPWM
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (u<0)? mag:0);  // LPWM
      break;
  }
}

/* Delta de encoder con wrap (TIM4 es 16b; TIM2/TIM5 32b) */
static inline int32_t enc_delta(TIM_HandleTypeDef* tim, uint32_t curr, uint32_t last)
{
  if (tim->Instance == TIM4) {
    return (int32_t)( (int16_t)((uint16_t)curr - (uint16_t)last) );
  } else {
    return (int32_t)( (int32_t)((uint32_t)curr - (uint32_t)last) );
  }
}

void reset_encoder_position(void)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  total_pulses = 0;
  last_encoder_count = 0;
  current_angle_dc = 0.0f;
}

void joint_callback(const void * msgin) {
  const trajectory_msgs__msg__JointTrajectory * msg = (const trajectory_msgs__msg__JointTrajectory *) msgin;
  if (msg->points.size == 0) return;

  trajectory_msgs__msg__JointTrajectoryPoint point = msg->points.data[0];
  if (point.positions.size >= 5) {
    target_joint_angles[0] = point.positions.data[0];
    target_joint_angles[1] = point.positions.data[1];
    target_joint_angles[2] = point.positions.data[2];
    target_joint_angles[3] = point.positions.data[3];
    target_joint_angles[4] = point.positions.data[4];
    new_target_available = true;
  } else {
    for (int i = 0; i < 5; i++) {
      if (i < point.positions.size) target_joint_angles[i] = point.positions.data[i];
      else                          target_joint_angles[i] = 0.0f;
    }
    new_target_available = true;
  }
}

void init_dc_motors(void)
{
  /* Motor0 Base */
  dc_motors[0].target_encoder_count = 0;
  dc_motors[0].moving = false;
  dc_motors[0].current_angle = 0.0f;
  dc_motors[0].target_angle = 0.0f;
  dc_motors[0].counts_per_revolution = COUNTS_PER_REV_MOTOR1;
  dc_motors[0].encoder_tim = &htim2;

  /* Motor1 Articulación 1 */
  dc_motors[1].target_encoder_count = 0;
  dc_motors[1].moving = false;
  dc_motors[1].current_angle = 0.0f;
  dc_motors[1].target_angle = 0.0f;
  dc_motors[1].counts_per_revolution = COUNTS_PER_REV_MOTOR2;
  dc_motors[1].encoder_tim = &htim3;

  /* Motor2 Articulación 2 */
  dc_motors[2].target_encoder_count = 0;
  dc_motors[2].moving = false;
  dc_motors[2].current_angle = 0.0f;
  dc_motors[2].target_angle = 0.0f;
  dc_motors[2].counts_per_revolution = COUNTS_PER_REV_MOTOR3;
  dc_motors[2].encoder_tim = &htim4;
}

void move_to_left(uint8_t motor_id)
{
  switch(motor_id) {
    case 0:
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_PWM);
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
      break;
    case 1:
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_PWM);
      break;
    case 2:
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, MAX_PWM);
      break;
  }
}

void move_to_right(uint8_t motor_id)
{
  switch(motor_id) {
    case 0:
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_PWM);
      break;
    case 1:
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_PWM);
      break;
    case 2:
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, MAX_PWM);
      break;
  }
}

void stop_motor(uint8_t motor_id){
	  dc_motors[motor_id].moving = false;
	  switch(motor_id) {
	    case 0:
	      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
	      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
	      break;
	    case 1:
	      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
	      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
	      break;
	    case 2:
	      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
	      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
	      break;
	  }
}

void stop_dc_motor(uint8_t motor_id)
{
  stop_motor(motor_id);
  dc_motors[motor_id].moving = false;
}

void start_dc_move_to_angle(uint8_t motor_id, float angle_deg)
{
  if (motor_id >= 3) return;

  DCMotorController *m = &dc_motors[motor_id];
  TIM_HandleTypeDef *tim = m->encoder_tim;

  __HAL_TIM_SET_COUNTER(tim, 0);

  last_cnt[motor_id]   = 0;
  settle_ctr[motor_id] = 0;

  m->target_encoder_count = (int32_t)lroundf((angle_deg * m->counts_per_revolution) / 360.0f);

  m->moving        = true;
  m->target_angle  = angle_deg;
  m->current_angle = 0.0f;
}

void update_dc_motors(void)
{
  for(int i = 0; i < 3; i++) {
    if(!dc_motors[i].moving) continue;

    TIM_HandleTypeDef* tim = dc_motors[i].encoder_tim;
    uint32_t curr_u = __HAL_TIM_GET_COUNTER(tim);
    int32_t curr = (int32_t)curr_u;

    int32_t e = dc_motors[i].target_encoder_count - curr;
    int32_t d = enc_delta(tim, curr_u, (uint32_t)last_cnt[i]);

    /* u = Kp*e - Kd*d  (si cnt sube acercándose a target, d>0 y reduce u) */
    float uf = KP * (float)e - KD * (float)d;
    int32_t u = (int32_t)uf;


    if ( (abs(e) <= ENCODER_TOLERANCE) && (abs(d) <= VEL_TOLERANCE) ) {
      if (++settle_ctr[i] >= SETTLE_CYCLES) {
        stop_motor(i);

        __HAL_TIM_SET_COUNTER(tim, 0);
        last_cnt[i] = 0;

        dc_motors[i].moving = false;
        dc_motors[i].current_angle = 0.0f;
        dc_motors[i].target_encoder_count = 0;
        continue;
      }
    } else {
      settle_ctr[i] = 0;
    }

    motor_set_pwm(i, u);
    last_cnt[i] = curr;
  }
}

void update_encoder_angles(void)
{
  /* Dejo tu mapeo tal cual */
  encoder_dc1 = (__HAL_TIM_GET_COUNTER(&htim5) * 360.0f) / COUNTS_PER_REV_MOTOR2;
  encoder_dc2 = (__HAL_TIM_GET_COUNTER(&htim2) * 360.0f) / COUNTS_PER_REV_MOTOR3;
  encoder_dc3 = (__HAL_TIM_GET_COUNTER(&htim4) * 360.0f) / COUNTS_PER_REV_MOTOR1;

  dc_motors[0].current_angle = encoder_dc1;
  dc_motors[1].current_angle = encoder_dc2;
  dc_motors[2].current_angle = encoder_dc3;
}

void move_joint_dc_1(float target_angle) { start_dc_move_to_angle(0, target_angle); }
void move_joint_dc_2(float target_angle) { start_dc_move_to_angle(1, target_angle); }
void move_joint_dc_3(float target_angle) { start_dc_move_to_angle(2, target_angle); }

void move_servo(int id, float angle)

{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;


    uint32_t channel;
    if (id == 3) {
        channel = TIM_CHANNEL_1;
    } else {
        channel = TIM_CHANNEL_2;
    }
	uint32_t pulse = 50 + (angle * 500) / 180;
	uint32_t pwm_value = 2000 - pulse;
	__HAL_TIM_SET_COMPARE(&htim9, channel, pwm_value);

}


void motor_angles_callback(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;

  if (msg->data.size >= 3) {

    start_dc_move_to_angle(1, (float)msg->data.data[0]);  // Motor DC 1
    start_dc_move_to_angle(0, (float)msg->data.data[1]);  // Motor DC 2
    start_dc_move_to_angle(2, (float)msg->data.data[2]);  // Motor DC 3
    move_servo(3,(float)msg->data.data[3]);				  // Servo motor 1
    move_servo(4,(float)msg->data.data[4]);               // Servo motor 2
  }

}

void init_sensors(void){
	  sensor1.i2cHandle = &hi2c1;
	  sensor1.i2cAddr = AS5600_SLAVE_ADDRESS;
	  sensor1.confRegister[0] = 0x00;
	  sensor1.confRegister[1] = 0x00;

	  sensor2.i2cHandle = &hi2c2;
	  sensor2.i2cAddr = AS5600_SLAVE_ADDRESS;
	  sensor2.confRegister[0] = 0x00;
	  sensor2.confRegister[1] = 0x00;

	  sensor3.i2cHandle = &hi2c3;
	  sensor3.i2cAddr = AS5600_SLAVE_ADDRESS;
	  sensor3.confRegister[0] = 0x00;
	  sensor3.confRegister[1] = 0x00;

	  AS5600_Init(&sensor1);
	  AS5600_GetAngle(&sensor1, &encoder1_angle);


	  AS5600_Init(&sensor2);
	  AS5600_Init(&sensor3);


}


void publish_angle()
{
    AS5600_GetAngle(&sensor1, &encoder1_angle);
    AS5600_GetAngle(&sensor2, &encoder2_angle);
    AS5600_GetAngle(&sensor3, &encoder3_angle);

    encoder_angles_msg.data.data[0] = encoder1_angle;
    encoder_angles_msg.data.data[1] = encoder2_angle;
    encoder_angles_msg.data.data[2] = encoder3_angle;
    encoder_angles_msg.data.size = 3;

    RCSOFTCHECK(rcl_publish(&encoder_angles_publisher, &encoder_angles_msg, NULL));
}


void StartDefaultTask(void *argument)
{
  rcl_node_t node;
  rclc_executor_t executor;

  rcl_subscription_t motor_angles_sub;
  std_msgs__msg__Int32MultiArray motor_angles_msg;

  std_msgs__msg__Float32MultiArray__init(&encoder_angles_msg);
  encoder_angles_msg.data.data = (float*)malloc(sizeof(float) * 3);
  encoder_angles_msg.data.capacity = 3;
  encoder_angles_msg.data.size = 3;

  // Buffers para los datos
  int32_t motor_angles_buffer[5] = {0, 0, 0, 0, 0};
  float encoder_data_buffer[3] = {0.0f, 0.0f, 0.0f};

  // Inicializar mensajes
  std_msgs__msg__Float32MultiArray__init(&encoder_angles_msg);

  // Configurar mensaje de ángulos de motores (entrada)
  motor_angles_msg.data.data = motor_angles_buffer;
  motor_angles_msg.data.capacity = 5;
  motor_angles_msg.data.size = 5;

  // Configurar mensaje de ángulos de encoders (salida)
  encoder_angles_msg.data.data = encoder_data_buffer;
  encoder_angles_msg.data.capacity = 3;
  encoder_angles_msg.data.size = 3;

  rmw_uros_set_custom_transport(
    true,
    (void *) &huart1,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read
  );

  rcl_allocator_t freeRTOS_allocator = rcutils_get_default_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  // Esperar conexión con el agente
  while (rmw_uros_ping_agent(100, 1) != RCL_RET_OK) {
    osDelay(100);
  }

  rclc_support_t support;
  RCCHECK(rclc_support_init(&support, 0, NULL, &freeRTOS_allocator));
  RCCHECK(rclc_node_init_default(&node, "motor_control_node", "", &support));
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &freeRTOS_allocator));

  RCCHECK(rclc_subscription_init_default(
    &motor_angles_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/motor_angles"
  ));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &motor_angles_sub,
    &motor_angles_msg,
    &motor_angles_callback,
    ON_NEW_DATA
  ));

  // Publicador para enviar ángulos actuales de los encoders
  RCCHECK(rclc_publisher_init_default(
    &encoder_angles_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/encoder_angles"
  ));


  osDelay(100);

  for(;;)
  {
    // Procesar mensajes entrantes
    rclc_executor_spin_some(&executor, 10);
    // Actualizar lecturas de encoders
    update_encoder_angles();
    // Actualizar control de motores DC
    update_dc_motors();
    publish_angle();
    osDelay(1);
  }
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
