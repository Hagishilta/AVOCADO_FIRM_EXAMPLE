#ifndef __ST5_HARDWARE_H__
#define __ST5_HARDWARE_H__


// A140K-M599-G5
#define TABLE_LEN       7
static float motor_table[TABLE_LEN][2] = {
  // {SPEED, DUTY}
  {0, 0.12},
  {140, 0.15},
  {200, 0.18},
  {280, 0.22},
  {400, 0.3},
  {550, 0.45},
  {700, 0.55}
};

#endif