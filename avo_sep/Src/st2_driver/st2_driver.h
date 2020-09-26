#ifndef __ST2_DRIVER_H__
#define __ST2_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>

typedef struct _ST2_DRIVER{
  int32_t current_cnt;
  int32_t input_cnt;
  int32_t speed;
}ST2_DRIVER;

extern ST2_DRIVER st2;

extern void ST2_Init(uint16_t freq);
extern void ST2_Loop();
extern void ST2_getPWM(float pwmDuty[4]);
#endif