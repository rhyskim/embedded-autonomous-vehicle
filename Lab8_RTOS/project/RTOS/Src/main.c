/**
  ******************************************************************************
  * @file    TIM/TIM_PWMOutput/Src/main.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    26-February-2014
  * @brief   This sample code shows how to use STM32F4xx TIM HAL API to generate
  * 4 signals in PWM.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "cmsis_os.h"

/* Private typedef -----------------------------------------------------------*/
#define  PERIOD_VALUE       0xFFFF  /* Period Value  */

/* Timer handler declaration */
TIM_HandleTypeDef    TimHandle1, TimHandle2, TimHandle3, TimHandle4;
TIM_IC_InitTypeDef     sICConfig;
   
/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig1, sConfig2, sConfig3;

/* Counter Prescaler value */
uint32_t uwPrescalerValue = 0;

volatile int32_t motorInterrupt1 = 0;
volatile int32_t motorInterrupt2 = 0;

uint8_t encoder_right = READY ;
uint8_t encoder_left  = READY ;
    
 /* Captured Values */
uint32_t               uwIC2Value1 = 0;
uint32_t               uwIC2Value2 = 0;
uint32_t               uwDiffCapture1 = 0;
   
uint32_t               uwIC2Value3 = 0;
uint32_t               uwIC2Value4 = 0;
uint32_t               uwDiffCapture2 = 0;

uint32_t               uwIC2Value5 = 0;
uint32_t               uwIC2Value6= 0;
uint32_t               uwDiffCapture3 = 0;

/* ADC handler declaration */
ADC_HandleTypeDef    AdcHandle1, AdcHandle2, AdcHandle3;
ADC_ChannelConfTypeDef adcConfig1, adcConfig2, adcConfig3;

/* Variable used to get converted value */
__IO uint32_t uhADCxRight;
__IO uint32_t uhADCxLeft;   

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
void Motor_Forward(void);
void Motor_Left(void);
void Motor_Right(void);
void Motor_Stop(void);
static void EXTILine_Config(void);
static void Error_Handler(void);
/* Private functions ---------------------------------------------------------*/

extern UART_HandleTypeDef UartHandle1, UartHandle2;

#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
   
PUTCHAR_PROTOTYPE
{
   HAL_UART_Transmit(&UartHandle1, (uint8_t *)&ch, 1, 0xFFFF); 
   return ch;
}

/********************************* FreeRTOS Maze & Auto-Correction Tasks ************************************/

/* 전역 하드웨어 상태 플래그 */
volatile uint8_t obs_forward = 0;
volatile uint8_t obs_left    = 0;
volatile uint8_t obs_right   = 0;

volatile uint8_t is_turning  = 0; 

/* [미세 보정 기능] 적외선 45도 센서 기반 제어 변수 */
volatile int32_t ir_error = 0;
volatile int32_t ir_prev_error = 0;
volatile int32_t pd_steering_output = 0;
volatile uint32_t dist_forward_cm = 999;
volatile uint32_t dist_left_cm = 999;
volatile uint32_t dist_right_cm = 999;
volatile int32_t last_valid_error = 0; 

/* 절대 방위각 구조체 및 변수 정의 */
typedef enum {
    DIR_NORTH = 0, 
    DIR_EAST  = 1, 
    DIR_SOUTH = 2, 
    DIR_WEST  = 3  
} Direction;

volatile Direction current_dir = DIR_NORTH; 

int get_direction_score(Direction dir)
{
   if(dir == DIR_NORTH) return 3;
   if(dir == DIR_EAST || dir == DIR_WEST) return 2;
   return 1;
}

/* 하드웨어 튜닝 파라미터 */
#define KP 4              
#define KD 8               
#define BASE_PULSE 19000   
#define ULTRASONIC_FRONT_TH 15
#define ULTRASONIC_SIDE_TH 25    
#define IR_VALID_MIN 220
#define PWM_MIN 11000
#define PWM_MAX 20000
#define SENSOR_CONFIRM_SAMPLES 3
#define SENSOR_CONFIRM_DELAY_MS 70
#define FRONT_CORNER_CAUTION_TH 20
#define SIDE_CORNER_DANGER_TH 15
#define CORNER_BASE_PULSE 11000
#define CORNER_STEER_ERROR 420
#define SINGLE_IR_STEER_ERROR 250
#define TURN_90_TICKS   900   
#define TURN_180_TICKS  1800  
#define TURN_BASE_PULSE 15000 
#define CENTER_TICKS    600   
#define IR_DEADZONE     100   

// [NEW] 동적 장애물(로봇청소기) 판단용 파라미터
#define IR_DELTA_TH 200         // 로봇청소기가 지나갈 때 발생하는 IR 센서 최소 변화량 절대값 (환경에 맞춰 150~300 튜닝)
#define DYNAMIC_SCAN_COUNT 50   // 약 500ms 동안 서서 청소기가 움직이는지 째려보는 횟수
#define DASH_TIME_MS 5000       // 로봇청소기 피하고 급발진해서 탈출할 시간 (ms)

static int32_t Abs32(int32_t v) { return (v < 0) ? -v : v; }

static void Reset_Encoder_Counts(void) {
   motorInterrupt1 = 0;
   motorInterrupt2 = 0;
}

// 틱 기반 제자리 회전 함수
static void Execute_Tick_Turn(uint8_t turn_dir) {
   Reset_Encoder_Counts();
   
   TIM8->CCR1 = TURN_BASE_PULSE; TIM8->CCR2 = TURN_BASE_PULSE;
   TIM4->CCR1 = TURN_BASE_PULSE; TIM4->CCR2 = TURN_BASE_PULSE;
   
   if(turn_dir == 1 || turn_dir == 3) Motor_Right();
   else Motor_Left();

   int32_t target_ticks = (turn_dir == 3) ? TURN_180_TICKS : TURN_90_TICKS;

   while(1) {
      int32_t dr = Abs32(motorInterrupt1);
      int32_t dl = Abs32(motorInterrupt2);
      if ((dr + dl) / 2 >= target_ticks) break;
      osDelay(5);
   }
   Motor_Stop();
   osDelay(150);
}

/**
  * @brief  정면 및 좌우 90도 초음파 센서 기반 벽면 스캔 태스크 (우선순위 2)
  */
void Detect_obstacle(void * argument) {
   osDelay(200); 
   printf("\r\n[SYS] Ultrasonic Scanner Initialized");

   for(;;)
   {
      osDelay(60); 
      
      if (is_turning) {
         dist_forward_cm = 0; dist_right_cm = 0; dist_left_cm = 0;
         obs_forward = 1; obs_right = 1; obs_left = 1;
         continue;
      }
      
      dist_forward_cm = uwDiffCapture2 / 58;
      dist_right_cm   = uwDiffCapture1 / 58;
      dist_left_cm    = uwDiffCapture3 / 58;

      obs_forward = (dist_forward_cm < ULTRASONIC_FRONT_TH) ? 1 : 0;
      obs_right   = (dist_right_cm   < ULTRASONIC_SIDE_TH)  ? 1 : 0;
      obs_left    = (dist_left_cm    < ULTRASONIC_SIDE_TH)  ? 1 : 0;
   }
}   

static void Read_Obstacle_Snapshot(uint8_t *front_blocked, uint8_t *left_blocked, uint8_t *right_blocked)
{
   int i;
   int front_count = 0;
   int left_count = 0;
   int right_count = 0;

   for(i = 0; i < SENSOR_CONFIRM_SAMPLES; i++)
   {
      osDelay(SENSOR_CONFIRM_DELAY_MS);

      if(obs_forward) front_count++;
      if(obs_left)    left_count++;
      if(obs_right)   right_count++;
   }

   *front_blocked = (front_count >= 2) ? 1 : 0;
   *left_blocked  = (left_count >= 2)  ? 1 : 0;
   *right_blocked = (right_count >= 2) ? 1 : 0;
}

/**
  * @brief  우벽 타기 및 방위각 기반 경로 탐색 (우선순위 1)
  */
void Motor_control(void * argument) {
   osDelay(500); 
   printf("\r\n[SYS] Forward-First Azimuth Navigation Initialized");

   // 동적 장애물 감지용 로컬 고성능 변수
   uint32_t last_ir_left = 0;
   uint32_t last_ir_right = 0;
   
   // [버그 수정 2] 안전 타이머: 출발 직후 사람 손이나 초기 떨림을 청소기로 착각하지 않도록 10초 쿨타임 부여
   uint32_t maze_start_time = osKernelSysTick();

   for(;;)
   {
      // 1. 앞이 열려 있으면 직진 모드 실행
      if(obs_forward == 0)
      {
         // ------------------------------------------------------------------
         // [우선순위 최상위] T자형 통로 전용 북쪽 샛길 감지 시스템
         // ------------------------------------------------------------------
         uint8_t found_north_fork = 0;
         uint8_t fork_turn_dir = 0; 

         if (current_dir == DIR_EAST && dist_left_cm > 35 && dist_right_cm < 25) {
            found_north_fork = 1;
            fork_turn_dir = 2; 
         }
         else if (current_dir == DIR_WEST && dist_right_cm > 35 && dist_left_cm < 25) {
            found_north_fork = 1;
            fork_turn_dir = 1; 
         }

         if (found_north_fork) {
            osDelay(40); 
            
            if ((fork_turn_dir == 2 && (uwDiffCapture3 / 58) > 35 && (uwDiffCapture1 / 58) < 25) || 
                (fork_turn_dir == 1 && (uwDiffCapture1 / 58) > 35 && (uwDiffCapture3 / 58) < 25)) 
            {
               printf("\r\n[FORK] Strict T-Junction North Path Verified!");
               is_turning = 1; 

               Reset_Encoder_Counts();
               TIM8->CCR1 = BASE_PULSE; TIM8->CCR2 = BASE_PULSE;
               TIM4->CCR1 = BASE_PULSE; TIM4->CCR2 = BASE_PULSE;
               Motor_Forward();

               while (1) {
                  int32_t dr = Abs32(motorInterrupt1);
                  int32_t dl = Abs32(motorInterrupt2);
                  if ((dr + dl) / 2 >= CENTER_TICKS) break;
                  osDelay(5);
               }
               Motor_Stop();
               osDelay(150);

               Execute_Tick_Turn(fork_turn_dir);
               current_dir = DIR_NORTH; 

               for(int i = 0; i < 20; i++)
               {
                  HAL_ADC_Start(&AdcHandle1); uhADCxLeft = HAL_ADC_GetValue(&AdcHandle1);
                  HAL_ADC_Start(&AdcHandle2); uhADCxRight = HAL_ADC_GetValue(&AdcHandle2);
                  ir_error = (int32_t)uhADCxLeft - (int32_t)uhADCxRight;
                  pd_steering_output = KP * ir_error;

                  TIM8->CCR1 = BASE_PULSE + pd_steering_output;
                  TIM8->CCR2 = BASE_PULSE + pd_steering_output;
                  TIM4->CCR1 = BASE_PULSE - pd_steering_output;
                  TIM4->CCR2 = BASE_PULSE - pd_steering_output;

                  Motor_Forward();
                  osDelay(10);
               }
               Motor_Stop();
               osDelay(100);

               is_turning = 0; 
               ir_prev_error = 0; last_valid_error = 0;
               continue; 
            }
         }

         // ------------------------------------------------------------------
         // 기본 정규 주행 및 라인 트레이싱 (PD 엔진)
         // ------------------------------------------------------------------
         uint32_t front_cm = dist_forward_cm;
         uint32_t left_cm = dist_left_cm;
         uint32_t right_cm = dist_right_cm;

         int32_t drive_base = BASE_PULSE;
         int32_t forced_error = 0;
         uint8_t corner_guard = 0;

         HAL_ADC_Start(&AdcHandle1);
         HAL_ADC_PollForConversion(&AdcHandle1, 0xFF);
         uhADCxLeft = HAL_ADC_GetValue(&AdcHandle1);

         HAL_ADC_Start(&AdcHandle2);
         HAL_ADC_PollForConversion(&AdcHandle2, 0xFF);
         uhADCxRight = HAL_ADC_GetValue(&AdcHandle2);

         if(left_cm < SIDE_CORNER_DANGER_TH && right_cm >= SIDE_CORNER_DANGER_TH)
         {
            forced_error = CORNER_STEER_ERROR;      
            corner_guard = 1;
            drive_base = CORNER_BASE_PULSE;
         }
         else if(right_cm < SIDE_CORNER_DANGER_TH && left_cm >= SIDE_CORNER_DANGER_TH)
         {
            forced_error = -CORNER_STEER_ERROR;     
            corner_guard = 1;
            drive_base = CORNER_BASE_PULSE;
         }
         else if(front_cm < FRONT_CORNER_CAUTION_TH)
         {
            drive_base = CORNER_BASE_PULSE;

            if(left_cm < right_cm)
            {
               forced_error = CORNER_STEER_ERROR;
               corner_guard = 1;
            }
            else if(right_cm < left_cm)
            {
               forced_error = -CORNER_STEER_ERROR;
               corner_guard = 1;
            }
         }

         if(corner_guard) 
         {
            ir_error = forced_error;
         }
         else if(uhADCxLeft >= IR_VALID_MIN && uhADCxRight >= IR_VALID_MIN) 
         {
            int32_t current_error = (int32_t)uhADCxLeft - (int32_t)uhADCxRight;
            if(Abs32(current_error) < IR_DEADZONE) {
               ir_error = 0;
            } else {
               ir_error = current_error;
               last_valid_error = ir_error;
            }
         }
         else 
         {
            ir_error = 0;
            ir_prev_error = 0; 
         }

         pd_steering_output = (KP * ir_error) + (KD * (ir_error - ir_prev_error));
         ir_prev_error = ir_error;

         int32_t left_motor_pulse  = drive_base + pd_steering_output;
         int32_t right_motor_pulse = drive_base - pd_steering_output;

         if(left_motor_pulse > PWM_MAX)  left_motor_pulse = PWM_MAX;
         if(left_motor_pulse < PWM_MIN)  left_motor_pulse = PWM_MIN;
         if(right_motor_pulse > PWM_MAX) right_motor_pulse = PWM_MAX;
         if(right_motor_pulse < PWM_MIN) right_motor_pulse = PWM_MIN;

         TIM8->CCR1 = left_motor_pulse;
         TIM8->CCR2 = left_motor_pulse;
         TIM4->CCR1 = right_motor_pulse;
         TIM4->CCR2 = right_motor_pulse;

         Motor_Forward();
         osDelay(15);
      }
      
      // 2. 앞이 막혀 있을 때 (정면 초음파 감지 이벤트)
      else
      {
         // ------------------------------------------------------------------
         // [NEW] 실시간 IR 변화율 기반 로봇청소기(동적 장애물) 판독 엔진
         // ------------------------------------------------------------------
         Motor_Stop(); // 일단 긴급 제동
         
         // 초기 기준값 샘플링 전 센서 안정화 대기
         osDelay(150); 
         HAL_ADC_Start(&AdcHandle1); uhADCxLeft = HAL_ADC_GetValue(&AdcHandle1);
         HAL_ADC_Start(&AdcHandle2); uhADCxRight = HAL_ADC_GetValue(&AdcHandle2);
         last_ir_left = uhADCxLeft;
         last_ir_right = uhADCxRight;

         uint8_t is_dynamic_obstacle = 0;

         // [버그 수정 2] 출발 후 10초(10000ms)가 지나야만 청소기 감지 엔진 가동!
         if (osKernelSysTick() - maze_start_time > 10000) 
         {
             // 약 500ms(10ms * 50회) 동안 정지 상태에서 센서 변화율 미분 추적
             for(int scan = 0; scan < DYNAMIC_SCAN_COUNT; scan++)
             {
                osDelay(10);
                HAL_ADC_Start(&AdcHandle1); uhADCxLeft = HAL_ADC_GetValue(&AdcHandle1);
                HAL_ADC_Start(&AdcHandle2); uhADCxRight = HAL_ADC_GetValue(&AdcHandle2);

                int32_t delta_left  = Abs32((int32_t)uhADCxLeft - (int32_t)last_ir_left);
                int32_t delta_right = Abs32((int32_t)uhADCxRight - (int32_t)last_ir_right);

                // 가만히 서있는데 45도 IR 데이터가 요동치면 동적 장애물!
                if(delta_left > IR_DELTA_TH || delta_right > IR_DELTA_TH)
                {
                   is_dynamic_obstacle = 1;
                   break; 
                }

                last_ir_left = uhADCxLeft;
                last_ir_right = uhADCxRight;
             }
         }

         // 로봇청소기로 확정된 경우 급발진 회피
         if(is_dynamic_obstacle)
         {
            printf("\r\n[ROBOT CLEANER] Moving Obstacle Confirmed!");
            is_turning = 1; 

            Motor_Stop();
            printf("\r\n[DODGE] Waiting for Cleaner to Pass Away...");

            uint32_t escape_timeout = osKernelSysTick() + 4000;
            while(osKernelSysTick() < escape_timeout)
            {
               osDelay(30);
               uint32_t live_front_cm = uwDiffCapture2 / 58;

               HAL_ADC_Start(&AdcHandle1); uhADCxLeft = HAL_ADC_GetValue(&AdcHandle1);
               HAL_ADC_Start(&AdcHandle2); uhADCxRight = HAL_ADC_GetValue(&AdcHandle2);

               if(live_front_cm > 25 && uhADCxLeft < 400 && uhADCxRight < 400)
               {
                  osDelay(80); 
                  break;
               }
            }

            printf("\r\n[DODGE] DASH!!!");
            TIM8->CCR1 = PWM_MAX; TIM8->CCR2 = PWM_MAX;
            TIM4->CCR1 = PWM_MAX; TIM4->CCR2 = PWM_MAX;
            Motor_Forward();

            osDelay(DASH_TIME_MS); 

            Motor_Stop();
            osDelay(100);

            is_turning = 0; 
            ir_prev_error = 0; last_valid_error = 0;
            continue; 
         }

         // ------------------------------------------------------------------
         // [기존 코드 유지] 정적 장애물(벽)로 판단되었을 때의 회전 탐색 엔진
         // ------------------------------------------------------------------
         uint8_t front_blocked;
         uint8_t left_blocked;
         uint8_t right_blocked;

         Direction right_dir;
         Direction left_dir;

         int right_score;
         int left_score;

         is_turning = 0; 
         Motor_Stop();
         osDelay(200); // 이미 센서를 확인했으므로 스냅샷 전 가볍게 안정화 마진만 부여

         Read_Obstacle_Snapshot(&front_blocked, &left_blocked, &right_blocked);

         if(front_blocked == 0)
         {
            ir_prev_error = 0;
            last_valid_error = 0;
            continue;
         }

         is_turning = 1; 

         right_dir = (Direction)((current_dir + 1) % 4);
         left_dir  = (Direction)((current_dir + 3) % 4);

         right_score = (right_blocked == 0) ? get_direction_score(right_dir) : -1;
         left_score  = (left_blocked == 0)  ? get_direction_score(left_dir)  : -1;

         printf("\r\n[ROUTE] Snapshot F:%d L:%d R:%d / LS:%d RS:%d",
                front_blocked, left_blocked, right_blocked, left_score, right_score);

         if(right_blocked == 0 && left_blocked == 1)
         {
            printf("\r\n[ROUTE] Open Right -> Turn Right");
            Execute_Tick_Turn(1);
            current_dir = right_dir;
         }
         else if(left_blocked == 0 && right_blocked == 1)
         {
            printf("\r\n[ROUTE] Open Left -> Turn Left");
            Execute_Tick_Turn(2);
            current_dir = left_dir;
         }
         else if(left_blocked == 0 && right_blocked == 0)
         {
            if(right_score >= left_score)
            {
               printf("\r\n[ROUTE] Both Open (Right Prefer) -> Turn Right");
               Execute_Tick_Turn(1);
               current_dir = right_dir;
            }
            else
            {
               printf("\r\n[ROUTE] Both Open (Left Prefer) -> Turn Left");
               Execute_Tick_Turn(2);
               current_dir = left_dir;
            }
         }
         else
         {
            printf("\r\n[ROUTE] Dead End -> U-Turn");
            Execute_Tick_Turn(3);
            current_dir = (Direction)((current_dir + 2) % 4);
         }

         // [버그 수정 1] 정렬 시 초고속 BASE_PULSE가 아닌 저속 CORNER_BASE_PULSE 사용!
         printf("\r\n[ALIGN] Secondary Fine Tuning...");
         for(int i = 0; i < 20; i++)
         {
            HAL_ADC_Start(&AdcHandle1); uhADCxLeft = HAL_ADC_GetValue(&AdcHandle1);
            HAL_ADC_Start(&AdcHandle2); uhADCxRight = HAL_ADC_GetValue(&AdcHandle2);

            ir_error = (int32_t)uhADCxLeft - (int32_t)uhADCxRight;
            pd_steering_output = KP * ir_error;

            TIM8->CCR1 = CORNER_BASE_PULSE + pd_steering_output;
            TIM8->CCR2 = CORNER_BASE_PULSE + pd_steering_output;
            TIM4->CCR1 = CORNER_BASE_PULSE - pd_steering_output;
            TIM4->CCR2 = CORNER_BASE_PULSE - pd_steering_output;

            Motor_Forward();
            osDelay(10);
         }

         Motor_Stop();
         osDelay(100);

         is_turning = 0;
         ir_prev_error = 0; last_valid_error = 0;

         printf("\r\n[SYS] Turn Completed. Direction: %d", current_dir);
      }
   }
}

/***************************************************************************/
int main(void)
{
   GPIO_InitTypeDef  GPIO_InitStruct;
   
   HAL_Init();
   SystemClock_Config();   
   BSP_COM1_Init();
   
   uwPrescalerValue = (SystemCoreClock/2)/1000000;
   
   __GPIOB_CLK_ENABLE();
      
   GPIO_InitStruct.Pin = GPIO_PIN_2;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
      
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); 
   
   sConfig1.OCMode     = TIM_OCMODE_PWM1;
   sConfig1.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfig1.OCFastMode = TIM_OCFAST_DISABLE;
   sConfig1.Pulse = 20000;
   
   TimHandle1.Instance = TIM8;
   TimHandle1.Init.Prescaler     = uwPrescalerValue;
   TimHandle1.Init.Period        = 20000; 
   TimHandle1.Init.ClockDivision = 0;
   TimHandle1.Init.CounterMode   = TIM_COUNTERMODE_UP;
   HAL_TIM_PWM_Init(&TimHandle1);
   
   HAL_TIM_PWM_ConfigChannel(&TimHandle1, &sConfig1, TIM_CHANNEL_1);
   HAL_TIM_PWM_ConfigChannel(&TimHandle1, &sConfig1, TIM_CHANNEL_2);
      
   sConfig2.OCMode     = TIM_OCMODE_PWM1;
   sConfig2.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfig2.OCFastMode = TIM_OCFAST_DISABLE;
   sConfig2.Pulse = 20000;
   
   TimHandle2.Instance = TIM4; 
   TimHandle2.Init.Prescaler     = uwPrescalerValue;
   TimHandle2.Init.Period        = 20000;
   TimHandle2.Init.ClockDivision = 0;
   TimHandle2.Init.CounterMode   = TIM_COUNTERMODE_UP;
   HAL_TIM_PWM_Init(&TimHandle2);

   HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig2, TIM_CHANNEL_1);
   HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig2, TIM_CHANNEL_2);

   EXTILine_Config();
    
   uwPrescalerValue = ((SystemCoreClock / 2) / 1000000) - 1;   
    
   TimHandle3.Instance = TIM3;
   TimHandle3.Init.Period        = 0xFFFF;
   TimHandle3.Init.Prescaler     = uwPrescalerValue;
   TimHandle3.Init.ClockDivision = 0;
   TimHandle3.Init.CounterMode   = TIM_COUNTERMODE_UP; 
   
   if(HAL_TIM_IC_Init(&TimHandle3) != HAL_OK){ Error_Handler();}

   sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
   sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
   sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
   sICConfig.ICFilter    = 0;   
   
   HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_1);
   HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_2);
   HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_3);
   HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_4);

   HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_2) ;
   HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_3) ;
   HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_4) ;

   uwPrescalerValue = (SystemCoreClock / 2 / 131099) - 1;
      
   TimHandle4.Instance = TIM10;
   TimHandle4.Init.Prescaler     = uwPrescalerValue;
   TimHandle4.Init.Period        = 0xFFFF;
   TimHandle4.Init.ClockDivision = 0;
   TimHandle4.Init.CounterMode   = TIM_COUNTERMODE_UP;
   HAL_TIM_PWM_Init(&TimHandle4);
   
   sConfig3.OCMode     = TIM_OCMODE_PWM1;
   sConfig3.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfig3.OCFastMode = TIM_OCFAST_DISABLE;
   sConfig3.Pulse = 2;
   HAL_TIM_PWM_ConfigChannel(&TimHandle4, &sConfig3, TIM_CHANNEL_1);
  
   HAL_TIM_PWM_Start(&TimHandle4, TIM_CHANNEL_1);
    
   AdcHandle1.Instance          = ADC3;   
   AdcHandle1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
   AdcHandle1.Init.Resolution = ADC_RESOLUTION12b;
   AdcHandle1.Init.ScanConvMode = DISABLE;
   AdcHandle1.Init.ContinuousConvMode = DISABLE;
   AdcHandle1.Init.DiscontinuousConvMode = DISABLE;
   AdcHandle1.Init.NbrOfDiscConversion = 0;  
   AdcHandle1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
   AdcHandle1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
   AdcHandle1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
   AdcHandle1.Init.NbrOfConversion = 1;
   AdcHandle1.Init.DMAContinuousRequests = DISABLE;
   AdcHandle1.Init.EOCSelection = DISABLE;

   HAL_ADC_Init(&AdcHandle1);

   adcConfig1.Channel = ADC_CHANNEL_11; 
   adcConfig1.Rank = 1;
   adcConfig1.SamplingTime = ADC_SAMPLETIME_480CYCLES; 
   adcConfig1.Offset = 0;
   HAL_ADC_ConfigChannel(&AdcHandle1, &adcConfig1);
      
   AdcHandle2.Instance          = ADC2;   
   AdcHandle2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
   AdcHandle2.Init.Resolution = ADC_RESOLUTION12b;
   AdcHandle2.Init.ScanConvMode = DISABLE;
   AdcHandle2.Init.ContinuousConvMode = DISABLE;
   AdcHandle2.Init.DiscontinuousConvMode = DISABLE;
   AdcHandle2.Init.NbrOfDiscConversion = 0;  
   AdcHandle2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
   AdcHandle2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
   AdcHandle2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
   AdcHandle2.Init.NbrOfConversion = 1;
   AdcHandle2.Init.DMAContinuousRequests = DISABLE;
   AdcHandle2.Init.EOCSelection = DISABLE;

   HAL_ADC_Init(&AdcHandle2);

   adcConfig2.Channel = ADC_CHANNEL_14;
   adcConfig2.Rank = 1;
   adcConfig2.SamplingTime = ADC_SAMPLETIME_480CYCLES;
   adcConfig2.Offset = 0;   
   HAL_ADC_ConfigChannel(&AdcHandle2, &adcConfig2);
   
   AdcHandle3.Instance          = ADC1;   
   AdcHandle3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
   AdcHandle3.Init.Resolution = ADC_RESOLUTION12b;
   AdcHandle3.Init.ScanConvMode = DISABLE;
   AdcHandle3.Init.ContinuousConvMode = DISABLE;
   AdcHandle3.Init.DiscontinuousConvMode = DISABLE;
   AdcHandle3.Init.NbrOfDiscConversion = 0;  
   AdcHandle3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
   AdcHandle3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
   AdcHandle3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
   AdcHandle3.Init.NbrOfConversion = 1;
   AdcHandle3.Init.DMAContinuousRequests = DISABLE;
   AdcHandle3.Init.EOCSelection = DISABLE;

   HAL_ADC_Init(&AdcHandle3);

   adcConfig3.Channel = ADC_CHANNEL_15;
   adcConfig3.Rank = 1;
   adcConfig3.SamplingTime = ADC_SAMPLETIME_480CYCLES;
   adcConfig3.Offset = 0;   
   HAL_ADC_ConfigChannel(&AdcHandle3, &adcConfig3);
       
   xTaskCreate(Detect_obstacle, (signed char *)"Detect", 128, NULL, 2, NULL);
   xTaskCreate(Motor_control,(signed char *)"Motor", 128, NULL, 1, NULL);

   vTaskStartScheduler();
  
   while(1)
   {
   }
}

void Motor_Forward(void)
{
   HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_2);
}

void Motor_Left(void)
{
   HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_2);
}

void Motor_Right(void)
{
   HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_1);
}

void Motor_Stop(void)
{
   HAL_TIM_PWM_Stop(&TimHandle1, TIM_CHANNEL_1);
   HAL_TIM_PWM_Stop(&TimHandle1, TIM_CHANNEL_2);            
   HAL_TIM_PWM_Stop(&TimHandle2, TIM_CHANNEL_1);
   HAL_TIM_PWM_Stop(&TimHandle2, TIM_CHANNEL_2);
}

static void SystemClock_Config(void)
{
   RCC_ClkInitTypeDef RCC_ClkInitStruct;
   RCC_OscInitTypeDef RCC_OscInitStruct;

   __PWR_CLK_ENABLE();
   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
   RCC_OscInitStruct.PLL.PLLM = 25;
   RCC_OscInitStruct.PLL.PLLN = 360;
   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
   RCC_OscInitStruct.PLL.PLLQ = 7;
   HAL_RCC_OscConfig(&RCC_OscInitStruct);

   HAL_PWREx_ActivateOverDrive();

   RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
   HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
   while (1)
   {
   }
}
#endif

 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
   switch(GPIO_Pin)
   {
      case GPIO_PIN_15 :
         encoder_right = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3);
         if(encoder_right == 0)
         {
            motorInterrupt1++;
            encoder_right = READY;
         }
         else if(encoder_right == 1)
         {
            motorInterrupt1--;
            encoder_right = READY;
         }
         break;
      
      case GPIO_PIN_4 :
         encoder_left = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);
         if(encoder_left == 0)
         {
            motorInterrupt2++;            
            encoder_left = READY;
         }      
         else if(encoder_left == 1)
         {
            motorInterrupt2--;
            encoder_left = READY;
         }   
         break;
   }
 }

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
   if(htim->Instance == TIM3)
   {
      if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
      {
         if((TIM3->CCER & TIM_CCER_CC2P) == 0)
         {
            uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            TIM3->CCER |= TIM_CCER_CC2P;
         }
         else if((TIM3->CCER & TIM_CCER_CC2P) == TIM_CCER_CC2P)
         {
            uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); 
            
            if (uwIC2Value2 > uwIC2Value1)
            {
                  uwDiffCapture1 = (uwIC2Value2 - uwIC2Value1); 
            }
            else if (uwIC2Value2 < uwIC2Value1)
            {
                  uwDiffCapture1 = ((0xFFFF - uwIC2Value1) + uwIC2Value2); 
            }
            else
            {
                  uwDiffCapture1 = 0;
            }
               
            TIM3->CCER &= ~TIM_CCER_CC2P;
         }
      }

      if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
      {
         if((TIM3->CCER & TIM_CCER_CC3P) == 0)
         {
            uwIC2Value3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            TIM3->CCER |= TIM_CCER_CC3P;
         }
         else if((TIM3->CCER & TIM_CCER_CC3P) == TIM_CCER_CC3P)
         {
            uwIC2Value4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); 
            
            if (uwIC2Value4 > uwIC2Value3)
            {
               uwDiffCapture2 = (uwIC2Value4 - uwIC2Value3); 
            }
            else if (uwIC2Value4 < uwIC2Value3)
            {
               uwDiffCapture2 = ((0xFFFF - uwIC2Value3) + uwIC2Value4); 
            }
            else
            {
               uwDiffCapture2 = 0;
            }
               
            TIM3->CCER &= ~TIM_CCER_CC3P;
         }      
      }
      
      if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
      {
         if((TIM3->CCER & TIM_CCER_CC4P) == 0)
         {
            uwIC2Value5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            TIM3->CCER |= TIM_CCER_CC4P;
         }
         else if((TIM3->CCER & TIM_CCER_CC4P) == TIM_CCER_CC4P)
         {
            uwIC2Value6 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); 
            
            if (uwIC2Value6 > uwIC2Value5)
            {
               uwDiffCapture3 = (uwIC2Value6 - uwIC2Value5); 
            }
            else if (uwIC2Value6 < uwIC2Value5)
            {
               uwDiffCapture3 = ((0xFFFF - uwIC2Value5) + uwIC2Value6); 
            }
            else
            {
               uwDiffCapture3 = 0;
            }
               
            TIM3->CCER &= ~TIM_CCER_CC4P;
         }
      }
   }
}

static void Error_Handler(void)
{
    BSP_LED_On(LED3);
    while(1)
    {
    }
}

static void EXTILine_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  __GPIOA_CLK_ENABLE();
  
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_15 ;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
   
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
      
 __GPIOB_CLK_ENABLE();

  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);   
   
  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}