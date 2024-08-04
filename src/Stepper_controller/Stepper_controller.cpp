#include <Arduino.h>
#include <Stepper_controller/Stepper_controller.h>

void init_GPIO(){
  pinMode(X_STEP_BIT, OUTPUT);
  pinMode(Y_STEP_BIT, OUTPUT);
  pinMode(Z_STEP_BIT, OUTPUT);

  pinMode(X_DIRECTION_BIT, OUTPUT);
  pinMode(Y_DIRECTION_BIT, OUTPUT);
  pinMode(Z_DIRECTION_BIT, OUTPUT);

  digitalWrite(Y_DIRECTION_BIT, HIGH);
  digitalWrite(Z_DIRECTION_BIT, LOW);

  pinMode(Limit_X, INPUT_PULLUP);
  pinMode(Limit_Y, INPUT_PULLUP);
}

void move_step(int32_t step_dx, int32_t step_dy)
{
  digitalWrite(X_DIRECTION_BIT, step_dx > 0);
  digitalWrite(Y_DIRECTION_BIT, step_dy > 0);
  digitalWrite(Z_DIRECTION_BIT, !(step_dy > 0));

  /* -- Absolute dx and dy -- */
  uint32_t step_dx_to_move = step_dx > 0 ? step_dx : -step_dx;
  uint32_t step_dy_to_move = step_dy > 0 ? step_dy : -step_dy;

  boolean dir = step_dx_to_move > step_dy_to_move;

  uint8_t unit_dx;
  uint8_t unit_dy;
  uint32_t hyp;
  while ((step_dx_to_move != 0) || (step_dy_to_move != 0))
  {
    hyp = sqrt(step_dx_to_move * step_dx_to_move + step_dy_to_move * step_dy_to_move);
    unit_dx = roundf((float)step_dx_to_move / (float)hyp);
    unit_dy = roundf((float)step_dy_to_move / (float)hyp);

    if (unit_dx)
    {
      pulseX();
      step_dx_to_move--;
    }
    if (unit_dy)
    {
      pulseY();
      step_dy_to_move--;
    }
  }
}

void pulseX() {
  delayMicroseconds(600);
  digitalWrite(X_STEP_BIT, HIGH);
  delayMicroseconds(300);
  digitalWrite(X_STEP_BIT, LOW);
}
void pulseY() {
  delayMicroseconds(600);
  PORTD |= 0b11000000;
  delayMicroseconds(300);
  PORTD &= 0b00111111;
}

void Cal_Origin(){
  uint32_t max_Step_X = 0;
  uint32_t max_Step_Y = 0;

  digitalWrite(X_DIRECTION_BIT, HIGH);
  while (digitalRead(Limit_X)) {
    pulseX();
    delay(DELAY_CAL_ORIGIN_MS);
  }
  max_Step_X = 0;

  digitalWrite(X_DIRECTION_BIT, LOW);
  while (digitalRead(Limit_X)) {
    pulseX();
    max_Step_X++;
    delay(DELAY_CAL_ORIGIN_MS);
  }

  digitalWrite(Y_DIRECTION_BIT, HIGH);
  while (digitalRead(Limit_Y)) {
    pulseY();
    delay(DELAY_CAL_ORIGIN_MS);
  }
  max_Step_Y = 0;

  digitalWrite(Y_DIRECTION_BIT, LOW);
  while (digitalRead(Limit_Y)) {
    pulseY();
    max_Step_Y++;
    delay(DELAY_CAL_ORIGIN_MS);
  }

  Serial.print("dx : ");
  Serial.print(max_Step_X);
  Serial.print(", dy : ");
  Serial.println(max_Step_Y); 
}