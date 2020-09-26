#ifndef __ST2_HARDWARE_H__
#define __ST2_hARDWARE_H__

// NK245-01AT
#define TABLE_LEN       7
static float motor_table[TABLE_LEN][2] = {
  // {SPEED, DUTY}
  {0, 0.05},
  {200, 0.08},
  {400, 0.1},
  {450, 0.12},
  {550, 0.13},
  {600, 0.14},
  {780, 0.2}
};

#endif