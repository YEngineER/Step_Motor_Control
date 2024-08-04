#ifndef _STEPPER_CONT_
#define _STEPPER_CONT_

#include <Arduino.h>
#define X_STEP_BIT 5 // Uno Digital Pin 2
#define Y_STEP_BIT 6 // Uno Digital Pin 3
#define Z_STEP_BIT 7 // Uno Digital Pin 4

#define X_DIRECTION_BIT 2 // Uno Digital Pin 5
#define Y_DIRECTION_BIT 3 // Uno Digital Pin 6
#define Z_DIRECTION_BIT 4 // Uno Digital Pin 7

#define Limit_X       9
#define Limit_Y       10

#define PULLEY_DIA_MM       6.0f
#define STEPPER_STEP_DEG    1.8f
#define STEP_PER_MM         ((PULLEY_DIA_MM/2.0f)*(STEPPER_STEP_DEG/180.0f)*M_PI)

#define DELAY_CAL_ORIGIN_MS   2

void init_GPIO();

void move_step(int32_t step_dx, int32_t step_dy);
void pulseX();
void pulseY();
void Cal_Origin();

#endif