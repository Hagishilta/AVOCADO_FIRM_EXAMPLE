#include "st2_driver.h"

ST2_DRIVER st2;

static uint8_t step2Value[4][4] = {
  {1, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 1},
  {1, 0, 0, 1}
};

static uint8_t st2_step = 0;
volatile static uint16_t st2_pwr;
static float st2_dT;
static float st2_currentCnt_float;
static float st2_max_duty;

void ST2_goNext(bool dir){
  if(dir) st2_step += 1;
  else st2_step += 3;
  st2_step %= 4;
}

void ST2_Init(uint16_t freq, float max_duty){
  st2_dT = 1.0f / (float)freq;
  st2_currentCnt_float = 0.0f;
  
  if(max_duty > 1.0f) max_duty = 1.0f;
  else if(max_duty < 0.0f) max_duty = 0.0f;
  
  st2_max_duty = max_duty;
}

void ST2_Loop(){ // Should be called periodically
  
  float d_speed;
  static int32_t cnt_prev;
  
  d_speed = ((float)st2.input_cnt - st2_currentCnt_float) / st2_dT;
  
  if(st2.speed < 0) st2.speed = -st2.speed;
  
  if(d_speed > st2.speed) d_speed = st2.speed;
  else if(d_speed < -st2.speed) d_speed = -st2.speed;
  
  st2_currentCnt_float += d_speed * st2_dT;
  cnt_prev = st2.current_cnt;
  st2.current_cnt = (int32_t)st2_currentCnt_float;
  
  if(cnt_prev != st2.current_cnt) ST2_goNext(d_speed >= 0.0f);
}

void ST2_getPWM(float pwmDuty[4]){
  
  uint8_t* pinValue = step2Value[st2_step];
  
  pwmDuty[0] = pinValue[0] ? st2_max_duty : 0; // A+
  pwmDuty[1] = pinValue[1] ? st2_max_duty : 0; // B+
  pwmDuty[2] = pinValue[2] ? st2_max_duty : 0; // A-
  pwmDuty[3] = pinValue[3] ? st2_max_duty : 0; // B-
}