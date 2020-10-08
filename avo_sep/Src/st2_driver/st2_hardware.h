#ifndef __ST2_HARDWARE_H__
#define __ST2_hARDWARE_H__

// NK245-01AT
#define TABLE_LEN       10


// ROTARY & SCREW
static float motor_table[TABLE_LEN][2] = {
  // {SPEED, DUTY}
  {0, 0.05},
  {200, 0.08},
  {400, 0.1},
  {450, 0.12},
  {550, 0.13},
  {600, 0.14},
  {780, 0.2},
  {1000, 0.3},
  {1500, 0.4},
  {2000, 0.55},
};

/*
// SAUCE
static float motor_table[TABLE_LEN][2] = {
  // {SPEED, DUTY}
  {0, 0.2},
  {200, 0.32},
  {400, 0.4},
  {450, 0.48},
  {550, 0.52},
  {600, 0.56},
  {780, 0.8},
  {1000, 1.0},
  {1500, 1.6},
  {2000, 2.0},
};
*/

#endif