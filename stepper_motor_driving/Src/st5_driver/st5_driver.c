#include "st5_driver.h"
#include "st5_hardware.h"

ST5_DRIVER st5;

static uint8_t step5Value[10][5] = {
  {1, 1, 0, 0, 0},
  {0, 1, 0, 0, 0},
  {0, 1, 1, 0, 0},
  {0, 0, 1, 0, 0},
  {0, 0, 1, 1, 0},
  {0, 0, 0, 1, 0},
  {0, 0, 0, 1, 1},
  {0, 0, 0, 0, 1},
  {1, 0, 0, 0, 1},
  {1, 0, 0, 0, 0},
};

static uint8_t st5_step = 0;
static float st5_dT;
static float st5_currentCnt_float;
static float st5_duty;
static float st5_speed;

void ST5_goNext(bool dir){
  if(dir) st5_step += 1;
  else st5_step += 9;
  st5_step %= 10;
}

void ST5_interPolate(int32_t speed){
  
  uint8_t i;
  float x1, x2;
  float y1, y2;
  
  if(speed < 0) speed = -speed;
  
  for(i = 0; i < TABLE_LEN; i++){
    if(speed < motor_table[i][0]) break;
  }
  
  // Interpolate
  if(i == 0){
    st5_duty = motor_table[0][1];
    return;
  }else if(i >= TABLE_LEN - 1){
    st5_duty = motor_table[TABLE_LEN - 1][1];
    return;
  }
  
  x1 = motor_table[i-1][0];
  x2 = motor_table[i][0];
  y1 = motor_table[i-1][1];
  y2 = motor_table[i][1];
  
  st5_duty = (y2 - y1) / (x2 - x1) * (speed - x1) + y1;
}

void ST5_Init(uint16_t freq){
  st5_dT = 1.0f / (float)freq;
  st5_currentCnt_float = 0.0f;
}

void ST5_Loop(){ // Should be called periodically
  
  static int32_t cnt_prev;
  
  st5_speed = ((float)st5.input_cnt - st5_currentCnt_float) / st5_dT;
  
  // speed validation
  if(st5.speed < 0) st5.speed = -st5.speed;
  if(motor_table[TABLE_LEN - 1][0] < st5.speed) st5.speed = (int32_t)motor_table[TABLE_LEN - 1][0];
  
  
  if(st5_speed > st5.speed) st5_speed = st5.speed;
  else if(st5_speed < -st5.speed) st5_speed = -st5.speed;
  
  st5_currentCnt_float += st5_speed * st5_dT;
  cnt_prev = st5.current_cnt;
  st5.current_cnt = (int32_t)st5_currentCnt_float;
  
  if(cnt_prev != st5.current_cnt) ST5_goNext(st5_speed >= 0.0f);
}

void ST5_getPWM(float pwmDuty[5]){
  
  uint8_t* pinValue = step5Value[st5_step];
  
  ST5_interPolate(st5_speed); // Calculate proper duty
  
  pwmDuty[0] = pinValue[0] ? st5_duty : 0; // A
  pwmDuty[1] = pinValue[1] ? st5_duty : 0; // B
  pwmDuty[2] = pinValue[2] ? st5_duty : 0; // C
  pwmDuty[3] = pinValue[3] ? st5_duty : 0; // D
  pwmDuty[4] = pinValue[4] ? st5_duty : 0; // E
}