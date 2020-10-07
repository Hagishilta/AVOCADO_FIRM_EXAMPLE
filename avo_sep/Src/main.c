/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>

#include "st2_driver/st2_driver.h"
#include "st5_driver/st5_driver.h"
#include "test/test.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



// Check Dispenser Type
#define TEST 0
#define SAUCE 1
#define SCREW 0
#define ROTARY 0




// START IO
//uint8_t relay;
GPIO_TypeDef* inputPort[] = {GPIOD, GPIOD, GPIOB, GPIOB, GPIOB, GPIOB, GPIOD, GPIOD};
uint16_t inputPin[] = {GPIO_PIN_4, GPIO_PIN_6, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_6, GPIO_PIN_5, GPIO_PIN_7, GPIO_PIN_5};

//uint8_t sensor;
GPIO_TypeDef* outputPort[] = {GPIOD, GPIOD, GPIOC, GPIOA, GPIOC, GPIOC, GPIOD, GPIOD};
uint16_t outputPin[] = {GPIO_PIN_1, GPIO_PIN_0, GPIO_PIN_11, GPIO_PIN_15, GPIO_PIN_10, GPIO_PIN_12, GPIO_PIN_2, GPIO_PIN_3};

uint8_t inputval[8];
// END IO



// START Screw
#define SCREW_SHUTTER_AMOUNT 1200
#define SCREW_WEIGHT_1 200
#define SCREW_WEIGHT_BELOW 30

bool screw_shutter_home_sensor(){
  if(HAL_GPIO_ReadPin(inputPort[2], inputPin[2]) == GPIO_PIN_SET){
    return true;
  }
  else{
    return false;
  }
}

bool screw_bowl_sensor(){
  if(HAL_GPIO_ReadPin(inputPort[3], inputPin[3]) == GPIO_PIN_SET){
    return false;
  }
  else{
    return true;
  }
}

bool screw_button_manual(){
  if(HAL_GPIO_ReadPin(inputPort[0], inputPin[0]) == GPIO_PIN_SET){
    return false;
  }
  else{
    return true;
  }
}

bool screw_button_mode(){
  if(HAL_GPIO_ReadPin(inputPort[1], inputPin[1]) == GPIO_PIN_SET){
    return true;
  }
  else{
    return false;
  }
}
// END Screw



// START Rotary
#define ROTARY_AMOUNT 700
#define ROTARY_WEIGHT 9

bool rotary_bowl_sensor(){
  if(HAL_GPIO_ReadPin(inputPort[3], inputPin[3]) == GPIO_PIN_SET){
    return false;
  }
  else{
    return true;
  }
}

bool rotary_button_manual(){
  if(HAL_GPIO_ReadPin(inputPort[1], inputPin[1]) == GPIO_PIN_SET){
    return false;
  }
  else{
    return true;
  }
}

bool rotary_button_mode(){
  if(HAL_GPIO_ReadPin(inputPort[0], inputPin[0]) == GPIO_PIN_SET){
    return true;
  }
  else{
    return false;
  }
}
// END Rotary



// START Sauce
#define POSITION_HALF 100
#define POSITION_NORMAL 200
#define POSITION_ONEHALF 300
#define POSITION_MAXIMUM 2500
#define POSITION_DISASSEMBLE 9800

void sauce_three_way_valve(bool n){
  if(n){
    HAL_GPIO_WritePin(outputPort[0], outputPin[0], GPIO_PIN_SET);
  }
  else{
    HAL_GPIO_WritePin(outputPort[0], outputPin[0], GPIO_PIN_RESET);
  }
}

void sauce_anti_drop_valve(bool n){
  if(n){
    HAL_GPIO_WritePin(outputPort[1], outputPin[1], GPIO_PIN_SET);
  }
  else{
    HAL_GPIO_WritePin(outputPort[1], outputPin[1], GPIO_PIN_RESET);
  }
}

bool sauce_piston_home_sensor(){
  if(HAL_GPIO_ReadPin(inputPort[3], inputPin[3]) == GPIO_PIN_SET){
    return false;
  }
  else{
    return true;
  }
}

bool sauce_bowl_sensor(){
  if(HAL_GPIO_ReadPin(inputPort[4], inputPin[4]) == GPIO_PIN_SET){
    return false;
  }
  else{
    return true;
  }
}

bool sauce_button_manual(){
  if(HAL_GPIO_ReadPin(inputPort[0], inputPin[0]) == GPIO_PIN_SET){
    return false;
  }
  else{
    return true;
  }
}

bool sauce_button_mode_1(){
  if(HAL_GPIO_ReadPin(inputPort[1], inputPin[1]) == GPIO_PIN_SET){
    return true;
  }
  else{
    return false;
  }
}

bool sauce_button_mode_2(){
  if(HAL_GPIO_ReadPin(inputPort[2], inputPin[2]) == GPIO_PIN_SET){
    return true;
  }
  else{
    return false;
  }
}
// END Sauce



// START Common
void common_power_alert_led(bool n){
  if(n){
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
  }
  else{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
  }
}

// Unavailable
void common_buzzer_sound(bool n){
  if(n){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  }
  else{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  }
}
// END Common





// START CAS Load Cell
typedef enum _CAS_HEADER1{
 CAS_OVERLOAD,
 CAS_UNDERLOAD,
 CAS_STABLE,
 CAS_UNSTABLE
}CAS_HEADER1;

typedef enum _CAS_HEADER2{
 CAS_NET_WEIGHT,
 CAS_GROSS_WEIGHT
}CAS_HEADER2;

typedef enum _CAS_UNIT{
  CAS_UNIT_Kg,
  CAS_UINT_t,
  CAS_UINT_g
}CAS_UNIT;

typedef struct _CAS_DATA{
  CAS_HEADER1 header1;
  CAS_HEADER2 header2;
  float data;
  CAS_UNIT unit;
}CAS_DATA;

#define CAS_RX_BUFFSIZE 100
uint8_t m_casRxBuffer[CAS_RX_BUFFSIZE];

CAS_DATA m_casData;

#define ci(i) (((i) + CAS_RX_BUFFSIZE) % CAS_RX_BUFFSIZE)
bool parse_casData(){
  
  static uint8_t temp[7];
  static CAS_DATA casData;
  
  uint8_t i, j;
  bool state = false;
  
  int32_t weight_hi, weight_lo;
  bool is_neg, is_decimal;
  
  char* x = (char*)&temp;
  
  // 1. Find frame between CR(0x0D) LF(0x0A)
  for(i = 0; i < CAS_RX_BUFFSIZE; i++){
    if(m_casRxBuffer[i] == 0x0D){
      if(i == CAS_RX_BUFFSIZE - 1){
        if(m_casRxBuffer[0] == 0x0A){
          state = true;
          break;
        }
      }else{
        if(m_casRxBuffer[i + 1] == 0x0A){
          state = true;
          break;
        }
      }
    }
  }
  
  if(!state) return false;
  
  // 2. Check data frame briefly
  i = ci(i - 16);
  if(m_casRxBuffer[ci(i + 2)] != ',') return false;
  if(m_casRxBuffer[ci(i + 5)] != ',') return false;
  
  // 3. Check Data Header1
  temp[0] = m_casRxBuffer[i];
  temp[1] = m_casRxBuffer[ci(i + 1)];
  
  
  if(!strncmp(x, "OL", 2)) casData.header1 = CAS_OVERLOAD;
  else if(!strncmp(x, "UL", 2)) casData.header1 = CAS_UNDERLOAD;
  else if(!strncmp(x, "ST", 2)) casData.header1 = CAS_STABLE;
  else if(!strncmp(x, "US", 2)) casData.header1 = CAS_UNSTABLE;
  else return false;
  
  // 4. Check Data Header2
  temp[0] = m_casRxBuffer[ci(i + 3)];
  temp[1] = m_casRxBuffer[ci(i + 4)];
  
  if(!strncmp(x, "NT", 2)) casData.header2 = CAS_NET_WEIGHT;
  else if(!strncmp(x, "GS", 2)) casData.header2 = CAS_GROSS_WEIGHT;
  else return false;
  
  // 5. Check Weight Data
  for(j = 0; j < 8; j++) temp[j] = m_casRxBuffer[ci(i + 6 + j)];
  
  is_neg = false;
  is_decimal = false;
  switch(temp[0]){
  case '+' : break;
  case '-' : is_neg = true; break;
  case ' ' : break;
  case '.' : is_decimal = true; break;
  default : return false;
  }
  
  weight_hi = 0;
  weight_lo = 0;
  for(j = 1; j < 8; j++){
    if(temp[j] >= '0' && temp[j] <= '9'){
      if(is_decimal) weight_lo = weight_lo * 10 + (temp[j] - '0');
      else weight_hi = weight_hi * 10 + (temp[j] - '0');
    }else if(temp[j] == '.'){
      is_decimal = true;
    }else{
      return false;
    }
  }
  
  casData.data = (float)weight_lo;
  while(casData.data > 1.0f) casData.data /= 10.0f;
  casData.data += (float)weight_hi;
  if(is_neg) casData.data *= -1.0f;
  
  // 6. Check units
  temp[0] = m_casRxBuffer[ci(i + 14)];
  temp[1] = m_casRxBuffer[ci(i + 15)];
  if(!strncmp(x, "kg", 2)) casData.unit = CAS_UNIT_Kg;
  else if(!strncmp(x, "-t", 2)) casData.unit = CAS_UINT_t;
  else if(!strncmp(x, "-g", 2)) casData.unit = CAS_UINT_g;
  else return false;
  
  // 7. Copy data & erase frame from buffer
  for(j = 0; j < 18; j++) m_casRxBuffer[ci(i + j)] = 0x00;
  memcpy(&m_casData, &casData, sizeof(CAS_DATA));
  
  return true;
  
}
// END CAS Load Cell



// START Motor - DC
#define MOTOR_SPEED_2_SAUCE 800
#define MOTOR_SPEED_5_SAUCE 800
#define MOTOR_SPEED_2_SCREW 700
#define MOTOR_SPEED_5_SCREW 700
#define MOTOR_SPEED_2_ROTARY 700
#define MOTOR_SPEED_5_ROTARY 300
#define MOTOR_FREQUENCY_2 20000
#define MOTOR_FREQUENCY_5 20000

float dc_speed_demand; // [-100, 100]
float dc_speed_command;
int16_t pwm_duty;

// Motor - 2phase, 5phase
#define ENABLE_GATE     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)
#define DISABLE_GATE    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)

#define ENABLE_DC       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)
#define DISABLE_DC      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)

#define ENABLE_ST2      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define DISABLE_ST2     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)

#define EANBLE_ST5      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET)
#define DISABLE_ST5     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET)

float st2_pwmDuty[4];
float st5_pwmDuty[5];

// END Motor



// START Serial Communication with Raspberry Pi
uint8_t txBuffer[2];
uint8_t rxBuffer;

// END Serial Communication with Raspberry Pi



// START Callback every 20kHz (= 20000Hz)
// Mode : STOP/DISASSEMBLE(0) AUTO(1) MANUAL(2)
uint8_t current_mode_test;

//uint8_t current_mode_sauce;
//uint8_t current_mode_screw;
//uint8_t current_mode_rotary;        // AUTO START
uint8_t button_mode_sauce[2];
uint8_t button_mode_screw;
uint8_t button_mode_rotary;
uint8_t current_state_sauce;    // DISASSEMBLE(0) AUTO(1) LOADING(2) LOADING_DONE(3) DISPENSING(4) MANUAL(5) MANUAL_DISPENSING(6)
uint8_t current_state_screw;    // READY/DISPENSING_DONE(0) AUTO(1) LOADING(2) LOADING_DONE(3) DISPENSING(4) MANUAL(5) MANUAL_DISPENSING(6)
uint8_t current_state_rotary;   // (READY/DISASSEMBLE)(0) AUTO(1) DISPENSING(2) MANUAL(3) MANUAL_DISPENSING(4)

uint8_t cnt_2hz;
uint8_t cnt_pi_communication;
uint16_t my_cnt;

// local variables
uint8_t cnt_rotary_manual;
uint8_t cnt_screw_manual;
uint8_t cnt_sauce_manual;

uint8_t cnt_rotary_bowl_sensor;
uint8_t cnt_screw_shutter_home_sensor;
uint8_t cnt_screw_bowl_sensor;
uint8_t cnt_sauce_bowl_sensor;
uint8_t cnt_sauce_piston_home_sensor;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim == &htim2){
    // CAS load cell
    parse_casData();
    
    // Stepping motor - 2phase
    ST2_Loop();
    
    ST2_getPWM(st2_pwmDuty);
    
    TIM4->CCR1 = (uint16_t)(st2_pwmDuty[0] * TIM4->ARR);
    TIM4->CCR3 = (uint16_t)(st2_pwmDuty[1] * TIM4->ARR);
    TIM4->CCR4 = (uint16_t)(st2_pwmDuty[2] * TIM4->ARR);
    TIM4->CCR2 = (uint16_t)(st2_pwmDuty[3] * TIM4->ARR);
    
    // Stepping motor - 5phase
    ST5_Loop();
    
    ST5_getPWM(st5_pwmDuty);
    
    TIM8->CCR4 = (uint16_t)(st5_pwmDuty[0] * TIM8->ARR);
    TIM8->CCR3 = (uint16_t)(st5_pwmDuty[1] * TIM8->ARR);
    TIM8->CCR2 = (uint16_t)(st5_pwmDuty[2] * TIM8->ARR);
    TIM1->CCR2 = (uint16_t)(st5_pwmDuty[3] * TIM1->ARR);
    TIM1->CCR1 = (uint16_t)(st5_pwmDuty[4] * TIM1->ARR);
    
    
    /* Button input method example
    //if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) x++;
    //else x = 0;
    
    //if(x > 100) button_mode[0] = 1;
    //else button_mode[0] = 0;
    */
    
    
    if(TEST == 1){
      // do nothing.
    }
    
    else if(SAUCE == 1){
      // mode switch
      if(sauce_button_mode_1()){
        if(button_mode_sauce[0] >= 255){
          button_mode_sauce[0] = 1;
        }
        button_mode_sauce[0]++;
      }
      else{
        button_mode_sauce[0] = 0;
      }
      
      if(sauce_button_mode_2()){
        if(button_mode_sauce[1] >= 255){
          button_mode_sauce[1] = 1;
        }
        button_mode_sauce[1]++;
      }
      else{
        button_mode_sauce[1] = 0;
      }
      
      // manual dispensing button
      if(sauce_button_manual()){
        if(cnt_sauce_manual >= 255){
          cnt_sauce_manual = 1;
        }
        cnt_sauce_manual++;
      }
      else{
        cnt_sauce_manual = 0;
      }
      
      // sauce piston home sensor
      if(sauce_piston_home_sensor()){
        if(cnt_sauce_piston_home_sensor >= 255){
          cnt_sauce_piston_home_sensor = 1;
        }
        cnt_sauce_piston_home_sensor++;
      }
      else{
        cnt_sauce_piston_home_sensor = 0;
      }
      
      // If not permissive, don't start : Check bowl sensor
      if(sauce_bowl_sensor()){
        if(cnt_sauce_bowl_sensor >= 255){
          cnt_sauce_bowl_sensor = 1;
        }
        cnt_sauce_bowl_sensor++;
      }
      else{
        cnt_sauce_bowl_sensor = 0;
      }
      
      // State : DISASSEMBLE(0) AUTO (1) LOADING(2) LOADING_DONE(3) DISPENSING(4) MANUAL(5) MANUAL_DISPENSING(6)
      if(button_mode_sauce[0] > 0 && button_mode_sauce[1] > 0 && (current_state_sauce == 1 || current_state_sauce == 5)){
        // DISASSEMBLE(0)
        current_state_sauce = 0;
      }
      else if(button_mode_sauce[0] > 0 && button_mode_sauce[1] <= 0 && current_state_sauce == 0){
        // AUTO(1)
        current_state_sauce = 1;
      }
      else if(button_mode_sauce[0] <= 0 && button_mode_sauce[1] > 0){
        if(cnt_sauce_bowl_sensor > 0 && cnt_sauce_manual > 0){
          // MANUAL_DISPENSING(6)
          current_state_sauce = 6;
        }
        else{
          // MANUAL(5)
          current_state_sauce = 5;
        }
      }
      
      
    }
    
    else if(SCREW == 1){
      // mode switch
      if(screw_button_mode()){
        if(button_mode_screw >= 255){
          button_mode_screw = 1;
        }
        button_mode_screw++;
      }
      else{
        button_mode_screw = 0;
      }
      
      // manual dispensing button
      if(screw_button_manual()){
        if(cnt_screw_manual >= 255){
          cnt_screw_manual = 1;
        }
        cnt_screw_manual++;
      }
      else{
        cnt_screw_manual = 0;
      }
      
      // screw shutter home sensor
      if(screw_shutter_home_sensor()){
        if(cnt_screw_shutter_home_sensor >= 255){
          cnt_screw_shutter_home_sensor = 1;
        }
        cnt_screw_shutter_home_sensor++;
      }
      else{
        cnt_screw_shutter_home_sensor = 0;
      }
      
      // If not permissive, don't start : Check bowl sensor
      if(screw_bowl_sensor()){
        if(cnt_screw_bowl_sensor >= 255){
          cnt_screw_bowl_sensor = 1;
        }
        cnt_screw_bowl_sensor++;
      }
      else{
        cnt_screw_bowl_sensor = 0;
      }
      
      // State : READY/DISPENSING_DONE(0) AUTO (1) LOADING(2) LOADING_DONE(3) DISPENSING(4) MANUAL(5) MANUAL_DISPENSING(6)
      if(button_mode_screw > 0 && (current_state_screw == 0 || current_state_screw == 5)){
        // AUTO(1)
        current_state_screw = 1;
      }
      else if(button_mode_screw <= 0){
        if(cnt_screw_bowl_sensor > 0 && cnt_screw_manual > 0){
          // MANUAL_DISPENSING(6)
          current_state_screw = 6;
        }
        else{
          // MANUAL(5)
          current_state_screw = 5;
        }
      }

    
    }
    
    else if(ROTARY == 1){
      // mode switch
      if(rotary_button_mode()){
        if(button_mode_rotary >= 255){
          button_mode_rotary = 1;
        }
        button_mode_rotary++;
      }
      else{
        button_mode_rotary = 0;
      }
      
      // manual dispensing button
      if(rotary_button_manual()){
        if(cnt_rotary_manual >= 255){
          cnt_rotary_manual = 1;
        }
        cnt_rotary_manual++;
      }
      else{
        cnt_rotary_manual = 0;
      }

      // If not permissive, don't start : Check bowl sensor
      if(rotary_bowl_sensor()){
        if(cnt_rotary_bowl_sensor >= 255){
          cnt_rotary_bowl_sensor = 1;
        }
        cnt_rotary_bowl_sensor++;
      }
      else{
        cnt_rotary_bowl_sensor = 0;
      }
      
      // State : (STOP/DISASSEMBLE)(0) AUTO(1) DISPENSING(2) MANUAL(3) MANUAL_DISPENSING(4)
      if(button_mode_rotary > 0 && (current_state_rotary == 0 || current_state_rotary == 3)){
        // AUTO(1)
        current_state_rotary = 1;
      }
      else if(button_mode_rotary <= 0){
        if(cnt_rotary_manual > 0 && cnt_rotary_bowl_sensor > 0){
          // MANUAL_DISPENSING(3)
          current_state_rotary = 4;
        }
        else{
          // MANUAL(2)
          current_state_rotary = 3;
        }
      }
      
    }

    // 2Hz communication with raspberry pi
    cnt_pi_communication++;
    if(cnt_pi_communication > 100){
      cnt_2hz++;
      if(cnt_2hz > 100){
        if(SAUCE == 1){
          txBuffer[0] = current_state_sauce + '0';
        }
        else if(SCREW == 1){
          txBuffer[0] = current_state_screw + '0';
        }
        else if(ROTARY == 1){
          txBuffer[0] = current_state_rotary + '0';
        }
        txBuffer[1] = '\n';
        HAL_UART_Transmit(&huart4, txBuffer, sizeof(txBuffer), 1000);
        cnt_2hz = 0;
      }
      cnt_pi_communication = 0;
    }
    my_cnt = TIM2->CNT;
    
  }
}
// END Callback


// local variables
uint8_t my_variables;
float rotary_initial_weight;
int32_t screw_initial_position;
float screw_initial_weight;
uint8_t screw_loading_amount;
bool screw_loading_repeat;

uint8_t sauce_loading_amount;
int32_t sauce_initial_position;

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
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_UART_Receive_DMA(&huart2, m_casRxBuffer, CAS_RX_BUFFSIZE);

  DISABLE_GATE;
  DISABLE_DC;
  DISABLE_ST2;
  DISABLE_ST5;
  
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);    // DC-
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);    // DC+
  
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);     // ST2 A+
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);     // ST2 B+
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);     // ST2 A-
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);     // ST2 B-
  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);     // ST5 E
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);     // ST5 D
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);     // ST5 C
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);     // ST5 B
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);     // ST5 A
  
  ST2_Init(MOTOR_FREQUENCY_2);
  ST5_Init(MOTOR_FREQUENCY_5);
  
  ENABLE_GATE;
  ENABLE_DC;
  ENABLE_ST2;
  EANBLE_ST5;
  
  HAL_TIM_Base_Start_IT(&htim2); // 20kHz Interrupt
  
//  my_func(1);
  
  // TODO : Check the rotate direction
  if(SAUCE == 1){
    st2.speed = MOTOR_SPEED_2_SAUCE;
    st5.speed = MOTOR_SPEED_5_SAUCE;
    current_state_sauce = 0;    // Set initial state
    
    // If not permissive, Ready position.
    while(current_state_sauce != 1){
      // wait until auto mode
      HAL_Delay(10);
    }
    st2.input_cnt = 100000;    // START homing
    while(cnt_sauce_piston_home_sensor <= 10){
      // wait
      // HAL_Delay(10);
    }
    // STOP when piston is home
    st2.speed = 0;
    HAL_Delay(1);
    st2.input_cnt = st2.current_cnt;
    HAL_Delay(1);
    st2.speed = MOTOR_SPEED_2_SAUCE;
    sauce_initial_position = st2.current_cnt;
  }
  else if(SCREW == 1){
    st2.speed = MOTOR_SPEED_2_SCREW;
    st5.speed = MOTOR_SPEED_5_SCREW;
    current_state_screw = 0;    // Set initial state
    st2.input_cnt = -100000;    // START homing
    while(cnt_screw_shutter_home_sensor <= 10){
      // wait
      //HAL_Delay(10);
    }
    // STOP when shutter is home
    st2.speed = 0;
    HAL_Delay(1);
    st2.input_cnt = st2.current_cnt;
    HAL_Delay(1);
    st2.speed = MOTOR_SPEED_2_SCREW;
    screw_initial_position = st2.current_cnt;     // Assume st2.current_cnt < 0
  }
  else if(ROTARY == 1){
    st2.speed = MOTOR_SPEED_2_ROTARY;
    st5.speed = MOTOR_SPEED_5_ROTARY;
    current_state_rotary = 0;   // Set initial state
  }
  
  common_power_alert_led(true);
  
  /*
  // IO TEST CODE : Check functionality
  uint8_t i, j;
  for(j=0; j<8; ++j){
      HAL_GPIO_WritePin(outputPort[j], outputPin[j], GPIO_PIN_SET);
    }
  for(j=0; j<8; ++j){
      HAL_GPIO_WritePin(outputPort[j], outputPin[j], GPIO_PIN_RESET);
    }
  
  for(j=0; j<8; ++j){
      HAL_GPIO_WritePin(outputPort[j], outputPin[j], GPIO_PIN_SET);
    }
*/
  
  /*
  bool flag = true;
  while(1){
    // Solenoid TEST
    power_alert_led(flag);
    //three_way_valve(flag);
    //anti_drop_valve(flag);
    
    flag = !flag;
  }*/


 
  /*
  // Double check AUTO(1) mode
  if(current_mode == 1){
    // 3 way valve OPEN
    three_way_valve(true);
    
    // anti drop valve CLOSE
    anti_drop_valve(false);
    
    // piston cylinder CLOSE (HOME)
    // st2.input_cnt = 10000;
  }
  else{
    // Error
    while(1){
      HAL_Delay(1000);
    }
  }
*/
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t i, j;
    
    if(TEST == 1){
//      HAL_UART_Receive(&huart4, &rxBuffer, 1, 1000);
//      txBuffer[0] = '3';
      txBuffer[0] = 4 + '0';
      txBuffer[1] = '\n';
      //HAL_UART_Transmit(&huart4, "1\n", sizeof("1\n"), 1000);
      HAL_UART_Transmit(&huart4, txBuffer, sizeof(txBuffer), 1000);
      HAL_UART_Receive(&huart4, &rxBuffer, 2, 1000);
    }
    
    
    else if(SAUCE == 1){
      // DISASSEMBLE(0)
      if(current_state_sauce == 0){
        // piston CLOSE
        st2.input_cnt = sauce_initial_position;
        while(st2.input_cnt != st2.current_cnt){
          HAL_Delay(10);
          if(current_state_sauce != 0){
            break;
          }
        }
        
        HAL_Delay(500); // need enough delay
        
        // 3way valve open
        sauce_three_way_valve(true);
        
        // anti drop valve CLOSE
        sauce_anti_drop_valve(false);
        
        // piston OPEN
        st2.input_cnt = sauce_initial_position - POSITION_DISASSEMBLE;
        while(st2.input_cnt != st2.current_cnt){
          HAL_Delay(10);
          if(current_state_sauce != 0){
            break;
          }
        }
        while(current_state_sauce == 0){
          HAL_Delay(10);
        }
      }
      // AUTO(1)
      else if(current_state_sauce == 1){
        // AUTO (From KMS)
        if(HAL_UART_Receive(&huart4, &rxBuffer, 1, 1000) == HAL_OK){
          if(rxBuffer == '1'){
            // HALF loading
            sauce_loading_amount = 1;
          }
          else if(rxBuffer == '2'){
            // NORMAL loading
            // Piston Cylinder OPEN
            sauce_loading_amount = 2;
          }
          else if(rxBuffer == '3'){
            // ONEHALF loading
            sauce_loading_amount = 3;
          }
          else{
            // ERROR
            sauce_loading_amount = 0;
          }
          
          if(sauce_loading_amount > 0){
            // If not permissive, Ready position.
            while(1){
              if(current_state_sauce == 1){
                break;
              }
              HAL_Delay(50);
            }
            
            current_state_sauce = 2;
          }
          
        }
      }
      // LOADING(2)
      else if(current_state_sauce == 2){
        HAL_Delay(500); // wait another 0.5 second
        
        if(sauce_loading_amount == 1){
          st2.input_cnt = sauce_initial_position - POSITION_HALF;
        }
        else if(sauce_loading_amount == 2){
          st2.input_cnt = sauce_initial_position - POSITION_NORMAL;
        }
        else if(sauce_loading_amount == 3){
          st2.input_cnt = sauce_initial_position - POSITION_ONEHALF;
        }
        
        while(st2.input_cnt != st2.current_cnt){
          HAL_Delay(10);
        }
        current_state_sauce = 3;
      }
      // LOADING_DONE(3)
      else if(current_state_sauce == 3){
        // Trigger from KMS
        if(HAL_UART_Receive(&huart4, &rxBuffer, 1, 1000) == HAL_OK){
          if(rxBuffer == 'q'){
            // If not permissive, Hold & Buzzer ON & Alert LED ON.
            bool flag = true;
            while(current_state_sauce == 3 && !sauce_bowl_sensor()){
              common_power_alert_led(flag);
              common_buzzer_sound(flag);      // buzzer NOT working now
              flag = !flag;
              HAL_Delay(500);
            }
            // RESET LED & BUZZER
            common_power_alert_led(true);
            common_buzzer_sound(false);
            
            current_state_sauce = 4;
          }
        }
      }
      // DISPENSING(4)
      else if(current_state_sauce == 4){
        sauce_three_way_valve(false);
        HAL_Delay(500);
        sauce_anti_drop_valve(true);
        HAL_Delay(500);
        
        st2.input_cnt = sauce_initial_position;
        while(st2.input_cnt != st2.current_cnt){
          HAL_Delay(10);
        }
        HAL_Delay(500);
        sauce_anti_drop_valve(false);
        HAL_Delay(50);
        sauce_three_way_valve(true);
        
        current_state_sauce = 1;        // AUTO ready position
      }
      // MANUAL(5)
      else if(current_state_sauce == 5){
        // ready position
        
        // 3 way valve OPEN
        sauce_three_way_valve(true);
        
        // anti drop valve CLOSE
        sauce_anti_drop_valve(false);
        
        // piston CLOSE
        st2.input_cnt = sauce_initial_position;
        while(st2.input_cnt != st2.current_cnt){
          HAL_Delay(10);
        }
      }
      // MANUAL_DISPENSING(6)
      else if(current_state_sauce == 6){
        while(current_state_sauce == 6 && !sauce_bowl_sensor()){
          HAL_Delay(10);
        }
        
        // piston FULL OPEN
        st2.input_cnt = sauce_initial_position - POSITION_MAXIMUM;
        while(st2.input_cnt != st2.current_cnt){
          HAL_Delay(10);
        }
        
        // 3 way valve CLOSE
        sauce_three_way_valve(false);
        
        // anti drop valve OPEN
        sauce_anti_drop_valve(true);
        
        // piston CLOSE
        st2.input_cnt = sauce_initial_position;
        while(st2.input_cnt != st2.current_cnt){
          HAL_Delay(10);
        }
        
        // anti drop valve CLOSE
        sauce_anti_drop_valve(false);
        
        // 3 way valve OPEN
        sauce_three_way_valve(true);
        
      }
      else{
        // Error : Unavailable Current State
      }
      
    }   // END of SAUCE

    else if(SCREW == 1){
      // State : READY/DISPENSING_DONE(0) AUTO (1) LOADING(2) LOADING_DONE(3) DISPENSING(4) MANUAL(5) MANUAL_DISPENSING(6)
      // READY/DISPENSING_DONE(0)
      if(current_state_screw == 0){
        // do nothing.
      }
      // AUTO (1)
      else if(current_state_screw == 1){
        // Trigger from KMS
        if(HAL_UART_Receive(&huart4, &rxBuffer, 1, 1000) == HAL_OK){
          // HALF, NORMAL, ONEHALF
          if(rxBuffer == '1'){
            // HALF
            screw_loading_amount = 1;
            screw_loading_repeat = false;
          }
          else if(rxBuffer == '2'){
            // NORMAL
            screw_loading_amount = 2;
            screw_loading_repeat = false;
          }
          else if(rxBuffer == '3'){
            // ONEHALF
            screw_loading_amount = 2;
            screw_loading_repeat = true;
          }
          else{
            screw_loading_amount = 0;
          }
          if(screw_loading_amount > 0){
            // If not permissive, Ready position.
            while(current_state_screw == 1 && !screw_shutter_home_sensor()){
              HAL_Delay(100);
            }
            
            // Bowl waiting delay
            HAL_Delay(1000);
            
            current_state_screw = 2;
          }
        }
      }
      // LOADING(2)
      else if(current_state_screw == 2){
        HAL_Delay(1000);        // wait before start
        if(screw_loading_amount == 1){
          // HALF
          // SCRREW DISPENSING START
          screw_initial_weight = m_casData.data;  // m_casData.data >= 0
          st5.input_cnt = st5.input_cnt + 10000000;
          while(m_casData.data < screw_initial_weight + SCREW_WEIGHT_1/2){
            if(current_state_screw != 2){
              break;
            }
            HAL_Delay(20);
          }
          st5.speed = 0;
          HAL_Delay(10);
          st5.input_cnt = st5.current_cnt;
          HAL_Delay(10);
          st5.speed = MOTOR_SPEED_5_ROTARY;
          // SCRREW DISPENSING END
        }
        else if(screw_loading_amount == 2){
          // NORMAL
          // SCRREW DISPENSING START
          screw_initial_weight = m_casData.data;  // m_casData.data >= 0
          st5.input_cnt = st5.input_cnt + 10000000;
          while(m_casData.data < screw_initial_weight + SCREW_WEIGHT_1){
            if(current_state_screw != 2){
              break;
            }
            HAL_Delay(20);            
          }
          st5.speed = 0;
          HAL_Delay(10);
          st5.input_cnt = st5.current_cnt;
          HAL_Delay(10);
          st5.speed = MOTOR_SPEED_5_ROTARY;
          // SCRREW DISPENSING END
        }
        else{
          // ERROR
        }
        
        current_state_screw = 3;
      }
      // LOADING_DONE(3)
      else if(current_state_screw == 3){
        // Trigger from KMS
        if(HAL_UART_Receive(&huart4, &rxBuffer, 1, 1000) == HAL_OK){
          if(rxBuffer == 'q'){
            // If not permissive, Hold & Buzzer ON & Alert LED ON.
            bool flag = true;
            while(!screw_bowl_sensor()){
              common_power_alert_led(flag);
              common_buzzer_sound(flag);      // buzzer NOT working now
              flag = !flag;
              HAL_Delay(500);
            }
            // RESET LED & BUZZER
            common_power_alert_led(true);
            common_buzzer_sound(false);
            
            current_state_screw = 4;
          }
        }
      }
      // DISPENSING(4)
      else if(current_state_screw == 4){
        // OPEN shutter
        st2.input_cnt = screw_initial_position + SCREW_SHUTTER_AMOUNT;
        while(st2.input_cnt != st2.current_cnt){
          HAL_Delay(10);
        }
        
        while(m_casData.data > SCREW_WEIGHT_BELOW){
          HAL_Delay(20);
        }
        HAL_Delay(2000);    // Stay 2 more seconds
        screw_loading_amount = 0;
        
        // CLOSE shutter
        st2.input_cnt = screw_initial_position;
        while(st2.input_cnt != st2.current_cnt){
          HAL_Delay(10);        // Why do we need this...?
        }
        
        if(screw_loading_repeat){
          // 2nd loading
          screw_loading_repeat = false;
          screw_loading_amount = 1;
          HAL_Delay(100);
          current_state_screw = 2;
        }
        else{
          current_state_screw = 1;
        }
      }
      // MANUAL(5)
      else if(current_state_screw == 5){
        // STOP screw
        if(st5.input_cnt != st5.current_cnt){
          st5.speed = 0;
          HAL_Delay(10);
          st5.input_cnt = st5.current_cnt;
          HAL_Delay(10);
          st5.speed = MOTOR_SPEED_5_SCREW;
          st5.current_cnt = screw_initial_position;
        }
        
        // CLOSE shutter
        st2.input_cnt = screw_initial_position;
        while(st2.input_cnt != st2.current_cnt){
          HAL_Delay(10);        // Why do we need this...?
        }
      }
      // MANUAL_DISPENSING(6)
      else if(current_state_screw == 6){
        // OPEN shutter
        st2.input_cnt = screw_initial_position + SCREW_SHUTTER_AMOUNT;
        while(st2.input_cnt != st2.current_cnt){
          HAL_Delay(10);
        }
        
        // RUN screw
        // TODO : FIX integer limit bug
        st5.input_cnt = st5.input_cnt + SCREW_SHUTTER_AMOUNT;
//        if(st5.input_cnt < 2147483648){
//          st5.input_cnt = st5.input_cnt + SCREW_SHUTTER_AMOUNT;
//        }
//        else{
//          st5.current_cnt = screw_initial_position + SCREW_SHUTTER_AMOUNT;
//        }
      }
      
    }   // END of SCREW
    
    else if(ROTARY == 1){
      // Mode : AUTO(1)
      if(current_state_rotary == 1){
        if(HAL_UART_Receive(&huart4, &rxBuffer, 1, 1000) == HAL_OK){
          if(rxBuffer == '0'){
            //If not permissive, Hold & Buzzer ON & Alert LED ON.
            bool flag = true;
            while(current_state_rotary == 1 && !rotary_bowl_sensor()){
              common_power_alert_led(flag);
              common_buzzer_sound(flag);      // buzzer NOT working now
              flag = !flag;
              HAL_Delay(500);
            }
            
            // Bowl waiting delay
            HAL_Delay(1000);
            
            current_state_rotary = 2;
            HAL_Delay(100);
          }
          else if(rxBuffer == '1'){
            // do nothing.
          }
        }
      }
      // Mode : DISPENSING(2)
      else if(current_state_rotary == 2){
        // ROTARY DISPENSING START
        rotary_initial_weight = m_casData.data;  // m_casData.data < 0
        while((m_casData.data + ROTARY_WEIGHT) >= rotary_initial_weight){
          // clockwise rotation
          st2.input_cnt = st2.input_cnt + ROTARY_AMOUNT;
          while(st2.input_cnt != st2.current_cnt){
            HAL_Delay(50);
          }
          // counter-clockwise rotation
          st2.input_cnt = st2.input_cnt - ROTARY_AMOUNT;
          while(st2.input_cnt != st2.current_cnt){
            HAL_Delay(50);
          }
          if(current_state_rotary != 2){
            break;
          }
        }
        // ROTARY DISPENSING END
        
        current_state_rotary = 1;
      }
      // Mode : MANUAL(3)
      else if(current_state_rotary == 3){
        // ROTARY STOP
        st2.speed = 0;
        HAL_Delay(50);
        st2.input_cnt = st2.current_cnt;
        HAL_Delay(50);
        st2.speed = MOTOR_SPEED_2_ROTARY;
      }
      // Mode : MANUAL_DISPENSING(4)
      else if(current_state_rotary == 4){
        st2.input_cnt = st2.input_cnt + 5 * ROTARY_AMOUNT;
      }
      
        
    }   // END of ROTARY
    
   
    
    /* CAS TEST
    if(parse_casData()){
      if(m_casData.header1 == CAS_STABLE){
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);     // LED ON
      }
      else{
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);   // LED OFF
      }
    }
    */
    
    /* DC MOTOR TEST
    if(m_casData.data <= -10){
      dc_speed_demand = 0.0f;
      while(1){
        dc_speed_demand = dc_speed_demand > 100.0f ? 100.0f : (dc_speed_demand < -100.0f ? -100.0f : dc_speed_demand);
        dc_speed_command = dc_speed_command * 0.99f + dc_speed_demand * 0.01f;
        
        pwm_duty = (uint16_t)((float)htim12.Instance->ARR * dc_speed_command / 100.0f);
        if(pwm_duty < 0){
          htim12.Instance->CCR1 = 0;
          htim12.Instance->CCR2 = -pwm_duty;
        }else{
          htim12.Instance->CCR2 = 0;
          htim12.Instance->CCR1 = pwm_duty;
        }
        
        HAL_Delay(3);
        if(-10 <= pwm_duty && pwm_duty <= 10){
          break;
        }
      }
    }
    else{
      dc_speed_demand = 100.0f;
      while(1){
        dc_speed_demand = dc_speed_demand > 100.0f ? 100.0f : (dc_speed_demand < -100.0f ? -100.0f : dc_speed_demand);
        dc_speed_command = dc_speed_command * 0.99f + dc_speed_demand * 0.01f;
        
        pwm_duty = (uint16_t)((float)htim12.Instance->ARR * dc_speed_command / 100.0f);
        if(pwm_duty < 0){
          htim12.Instance->CCR1 = 0;
          htim12.Instance->CCR2 = -pwm_duty;
        }else{
          htim12.Instance->CCR2 = 0;
          htim12.Instance->CCR1 = pwm_duty;
        }
        
        HAL_Delay(3);
        if(pwm_duty >= 4490){
          break;
        }
      }
    }
    */
    
    /* Board test code 
    if(HAL_UART_Receive(&huart4, &rxBuffer, 1, 1000) == HAL_OK){
      if(rxBuffer == '0'){
        st5.input_cnt = 1000;
      }
      else if(rxBuffer == '1'){
        st5.input_cnt = 0;
      }
      else if(rxBuffer == '2'){
        st2.input_cnt = 1000;
      }
      else if(rxBuffer == '3'){
        st2.input_cnt = 0;
      }
      else if(rxBuffer == '4'){
        dc_speed_demand = 100.0f;
        while(1){
          dc_speed_demand = dc_speed_demand > 100.0f ? 100.0f : (dc_speed_demand < -100.0f ? -100.0f : dc_speed_demand);
          dc_speed_command = dc_speed_command * 0.99f + dc_speed_demand * 0.01f;
          
          pwm_duty = (uint16_t)((float)htim12.Instance->ARR * dc_speed_command / 100.0f);
          if(pwm_duty < 0){
            htim12.Instance->CCR1 = 0;
            htim12.Instance->CCR2 = -pwm_duty;
          }else{
            htim12.Instance->CCR2 = 0;
            htim12.Instance->CCR1 = pwm_duty;
          }
          
          HAL_Delay(3);
          if(pwm_duty >= 4490){
            break;
          }
        }
      }
      else if(rxBuffer == '5'){
        dc_speed_demand = 0.0f;
        while(1){
          dc_speed_demand = dc_speed_demand > 100.0f ? 100.0f : (dc_speed_demand < -100.0f ? -100.0f : dc_speed_demand);
          dc_speed_command = dc_speed_command * 0.99f + dc_speed_demand * 0.01f;
          
          pwm_duty = (uint16_t)((float)htim12.Instance->ARR * dc_speed_command / 100.0f);
          if(pwm_duty < 0){
            htim12.Instance->CCR1 = 0;
            htim12.Instance->CCR2 = -pwm_duty;
          }else{
            htim12.Instance->CCR2 = 0;
            htim12.Instance->CCR1 = pwm_duty;
          }
          
          HAL_Delay(3);
          if(-10 <= pwm_duty && pwm_duty <= 10){
            break;
          }
        }
      }
      else if(rxBuffer == '6'){
        dc_speed_demand = -100.0f;
        while(1){
          dc_speed_demand = dc_speed_demand > 100.0f ? 100.0f : (dc_speed_demand < -100.0f ? -100.0f : dc_speed_demand);
          dc_speed_command = dc_speed_command * 0.99f + dc_speed_demand * 0.01f;
          
          pwm_duty = (uint16_t)((float)htim12.Instance->ARR * dc_speed_command / 100.0f);
          if(pwm_duty < 0){
            htim12.Instance->CCR1 = 0;
            htim12.Instance->CCR2 = -pwm_duty;
          }else{
            htim12.Instance->CCR2 = 0;
            htim12.Instance->CCR1 = pwm_duty;
          }
          
          HAL_Delay(3);
          if(pwm_duty <= -4490){
            break;
          }
        }
      }
      else if(rxBuffer == '7'){
        for(j=0; j<8; ++j){
          HAL_GPIO_WritePin(outputPort[j], outputPin[j], GPIO_PIN_SET);
          HAL_Delay(200);
          //hal_gpio_writepin(gpioe, gpio_pin_0, gpio_pin_set); // led
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   // BUZZER
        }
      }
      else if(rxBuffer == '8'){
        for(j=0; j<8; ++j){
          HAL_GPIO_WritePin(outputPort[j], outputPin[j], GPIO_PIN_RESET);
          HAL_Delay(200);
          //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET); // LED
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);   // BUZZER
        }
      }
      else if(rxBuffer == '9'){
        
      }
    }
    
    */
    
    /* NO DC motor at this time.
    HAL_Delay(50);
    if(){
    }
    dc_speed_demand = 0.0f;
      while(1){
        dc_speed_demand = dc_speed_demand > 100.0f ? 100.0f : (dc_speed_demand < -100.0f ? -100.0f : dc_speed_demand);
        dc_speed_command = dc_speed_command * 0.99f + dc_speed_demand * 0.01f;
        
        pwm_duty = (uint16_t)((float)htim12.Instance->ARR * dc_speed_command / 100.0f);
        if(pwm_duty < 0){
          htim12.Instance->CCR1 = 0;
          htim12.Instance->CCR2 = -pwm_duty;
        }else{
          htim12.Instance->CCR2 = 0;
          htim12.Instance->CCR1 = pwm_duty;
        }
        
        HAL_Delay(3);
        if(-10 <= pwm_duty && pwm_duty <= 10){
          break;
        }
      }
*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  
  /* Test while loop
  while (1){
    uint8_t i, j;
    parse_casData();
    if(HAL_UART_Receive(&huart4, &rxBuffer, 1, 1000) == HAL_OK){
      if(rxBuffer == '0'){
        
      }
      else if(rxBuffer == '1'){
        break;
      }
      else if(rxBuffer == '2'){
        
      }
      else if(rxBuffer == '3'){
      }
      else if(rxBuffer == '4'){
      }
      else if(rxBuffer == '5'){
        
      }
      else if(rxBuffer == '6'){
        
      }
      else if(rxBuffer == 0x61){
        for(j=0; j<8; ++j){
          HAL_GPIO_WritePin(outputPort[j], outputPin[j], GPIO_PIN_SET);
          HAL_Delay(200);
          //hal_gpio_writepin(gpioe, gpio_pin_0, gpio_pin_set); // led
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   // BUZZER
        }
      }
      else if(rxBuffer == 0x62){
        for(j=0; j<8; ++j){
          HAL_GPIO_WritePin(outputPort[j], outputPin[j], GPIO_PIN_RESET);
          HAL_Delay(200);
          //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET); // LED
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);   // BUZZER
        }
      }
      else if(rxBuffer == '9'){
        
      }
    }
  }
  */
  
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 4499;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 4499;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
