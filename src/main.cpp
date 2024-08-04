#include <Arduino.h>
#include <Stepper_controller/Stepper_controller.h>
#include <AD9833.h>

typedef enum{
  Await_CMD,
  Execute_Move_up,
  Execute_Move_down,
  Execute_Move_Left,
  Execute_Move_Right,
  Execute_Move_Origin,
  Execute_Set_Origin,
  Execute_Cal_Origin,
  Execute_maxX
}Control_FSM;


void readCMD();
void readTermination();
void Probe_Tune_Test();

Control_FSM cnt_fsm = Await_CMD;

void setup() {
  init_GPIO();

  // for (uint16_t i = 0; i < 500; i++)
  // {
  //   // digitalWrite(Y_STEP_BIT, HIGH);
  //   // digitalWrite(Z_STEP_BIT, HIGH);
  //   // delay(2);
  //   // digitalWrite(Y_STEP_BIT, LOW);
  //   // digitalWrite(Z_STEP_BIT, LOW);
  //   // delay(2);
  //   PORTD ^= 0b11000000;
  //   // delay(2);
  //   delayMicroseconds(500);
  //   PORTD ^= 0b11000000;
  //   // delay(2);
  //   delayMicroseconds(500);
  // }
  // delay(1000);

  // digitalWrite(X_DIRECTION_BIT, HIGH);
  // for (uint16_t i = 0; i < 600; i++)
  // {
  //   digitalWrite(X_STEP_BIT, HIGH);
  //   // delay(2);
  //   delayMicroseconds(500);
  //   digitalWrite(X_STEP_BIT, LOW);
  //   // delay(2);
  //   delayMicroseconds(500);
  // }
  // delay(1000);

  // digitalWrite(Y_DIRECTION_BIT, LOW);
  // digitalWrite(Z_DIRECTION_BIT, HIGH);
  // for (uint16_t i = 0; i < 500; i++)
  // {
  //   // digitalWrite(Y_STEP_BIT, HIGH);
  //   // digitalWrite(Z_STEP_BIT, HIGH);
  //   // delay(2);
  //   // digitalWrite(Y_STEP_BIT, LOW);
  //   // digitalWrite(Z_STEP_BIT, LOW);
  //   // delay(2);
  //   PORTD ^= 0b11000000;
  //   // delay(2);
  //   delayMicroseconds(500);
  //   PORTD ^= 0b11000000;
  //   // delay(2);
  //   delayMicroseconds(500);
  // }
  // delay(1000);
  // digitalWrite(X_DIRECTION_BIT, LOW);
  // for (uint16_t i = 0; i < 600; i++)
  // {
  //   digitalWrite(X_STEP_BIT, HIGH);
  //   // delay(2);
  //   delayMicroseconds(500);
  //   digitalWrite(X_STEP_BIT, LOW);
  //   // delay(2);
  //   delayMicroseconds(500);
  // }
  // move_step(500,0);
  // delay(1000);
  // move_step(0,500);
  // delay(1000);
  // move_step(-500,-500);
  Serial.begin(115200);
}

void loop() {
  uint16_t step_x = 0;
  uint16_t step_y = 0;
  // Probe_Tune_Test();
  switch (cnt_fsm){
  case Await_CMD :
    readCMD();
    break;
  case Execute_Move_up :
    digitalWrite(Y_DIRECTION_BIT, HIGH);
    digitalWrite(Z_DIRECTION_BIT, LOW);
    pulseY();
    if(digitalRead(Limit_Y) == LOW)
      for(uint8_t i = 0;i < 40;i++){
        digitalWrite(Y_DIRECTION_BIT, LOW);
        digitalWrite(Z_DIRECTION_BIT, HIGH);
        pulseY();
      }
    cnt_fsm = Await_CMD;
    break;
  case Execute_Move_down :
    digitalWrite(Y_DIRECTION_BIT, LOW);
    digitalWrite(Z_DIRECTION_BIT, HIGH);
    pulseY();
    if(digitalRead(Limit_Y) == LOW)
      for(uint8_t i = 0;i < 40;i++){
        digitalWrite(Y_DIRECTION_BIT, HIGH);
        digitalWrite(Z_DIRECTION_BIT, LOW);
        pulseY();
      }
    cnt_fsm = Await_CMD;
    break;
  case Execute_Move_Left :
    digitalWrite(X_DIRECTION_BIT, LOW);
    pulseX();
    if(digitalRead(Limit_X) == LOW)
      for(uint8_t i = 0;i < 40;i++){
        digitalWrite(X_DIRECTION_BIT, HIGH);
        pulseX();
      }
    cnt_fsm = Await_CMD;
    break;
  case Execute_Move_Right :
    digitalWrite(X_DIRECTION_BIT, HIGH);
    pulseX();
    if(digitalRead(Limit_X) == LOW)
      for(uint8_t i = 0;i < 40;i++){
        digitalWrite(X_DIRECTION_BIT, LOW);
        pulseX();
      }
    cnt_fsm = Await_CMD;
    break;
  case Execute_Move_Origin :
    readTermination();
    break;
  case Execute_Set_Origin :
    readTermination();
    break;
  case Execute_Cal_Origin :
    readTermination();
    break;
  case Execute_maxX:
  /* goto max +X*/
    while (digitalRead(Limit_X) == HIGH) {
      digitalWrite(X_DIRECTION_BIT, HIGH);
      pulseX();
    }
    step_x = 0;
    for(uint8_t i = 0;i < 40;i++){
      digitalWrite(X_DIRECTION_BIT, LOW);
      pulseX();
    }
  /* goto max -X*/
    while(digitalRead(Limit_X) == HIGH){
      digitalWrite(X_DIRECTION_BIT, LOW);
      pulseX();
      step_x++;
    }
    for(uint8_t i = 0;i < 40;i++){
      digitalWrite(X_DIRECTION_BIT, HIGH);
      pulseX();
    }

    /* goto max +Y */
    while(digitalRead(Limit_Y) == HIGH){
      digitalWrite(Y_DIRECTION_BIT, HIGH);
      digitalWrite(Z_DIRECTION_BIT, LOW);
      pulseY();
    }
    step_y = 0;
    for(uint8_t i = 0;i < 80;i++){
      digitalWrite(Y_DIRECTION_BIT, LOW);
      digitalWrite(Z_DIRECTION_BIT, HIGH);
      pulseY();
    }

    while (digitalRead(Limit_Y) == HIGH){
      digitalWrite(Y_DIRECTION_BIT, LOW);
      digitalWrite(Z_DIRECTION_BIT, HIGH);
      pulseY();
      step_y++;
    }
    for(uint8_t i = 0;i < 80;i++){
      digitalWrite(Y_DIRECTION_BIT, HIGH);
      digitalWrite(Z_DIRECTION_BIT, LOW);
      pulseY();
    }

    for(uint16_t i = 0;i < step_x/2;i++){
      digitalWrite(X_DIRECTION_BIT, HIGH);
      pulseX();
    }
    for(uint16_t i = 0;i < step_y/2;i++){
      digitalWrite(Y_DIRECTION_BIT, HIGH);
      digitalWrite(Z_DIRECTION_BIT, LOW);
      pulseY();
    }

    Serial.println(step_x);
    Serial.println(step_y);

    cnt_fsm = Await_CMD;
    break;
  default:
    cnt_fsm = Await_CMD;
    break;
  }
  // int16_t mov_dx = analogRead(A6) - 512;
  // int16_t mov_dy = analogRead(A7) - 512;

  // int32_t distance_move = hypotf(mov_dx,mov_dy);

  // int32_t move_to_dx = 0;
  // int32_t move_to_dy = 0;
  // if(!(mov_dx < 712 && mov_dx > 312)){
  //   move_to_dx = (mov_dx - 512) / 64;
  // }

  // if(!(mov_dy > 712 && mov_dy < 312)){
  //   move_to_dy = (mov_dy - 512) / 64;
  // }
  // if(distance_move > 300){
  //   move_to_dx = mov_dx / 128;
  //   move_to_dy = mov_dy / 128;
  // }

  // Serial.print("dx : ");
  // Serial.print(move_to_dx);
  // Serial.print(", dy : ");
  // Serial.print(move_to_dy);
  // Serial.println();
  // move_step(move_to_dx,move_to_dy);
}

void Probe_Tune_Test(){
  /* goto max +X*/
    while (digitalRead(Limit_X) == HIGH) {
      digitalWrite(X_DIRECTION_BIT, HIGH);
      pulseX();
      delayMicroseconds(800);
    }
    for(uint8_t i = 0;i < 40;i++){
      digitalWrite(X_DIRECTION_BIT, LOW);
      pulseX();
    }
  /* goto max -X*/
    while(digitalRead(Limit_X) == HIGH){
      digitalWrite(X_DIRECTION_BIT, LOW);
      pulseX();
      delayMicroseconds(800);
    }
    for(uint8_t i = 0;i < 40;i++){
      digitalWrite(X_DIRECTION_BIT, HIGH);
      pulseX();
    }
    delay(500);
}

void readCMD(){
  // check if data is available
  if (Serial.available() > 0) {
    // read the incoming string:
    String incomingString = Serial.readStringUntil('\n');
    if(incomingString == "mx+") cnt_fsm = Execute_Move_Right;
    if(incomingString == "mx-") cnt_fsm = Execute_Move_Left;
    if(incomingString == "my+") cnt_fsm = Execute_Move_up;
    if(incomingString == "my-") cnt_fsm = Execute_Move_down;
    if(incomingString == "center") cnt_fsm = Execute_maxX;
  }
}

void readTermination(){
  if (Serial.available() > 0){
    // read the incoming string:
    String incomingString = Serial.readStringUntil('\n');
    if(incomingString == "cancel") cnt_fsm = Await_CMD;
  }
}