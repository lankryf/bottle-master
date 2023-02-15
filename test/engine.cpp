#include <Arduino.h>
#include <GyverStepper.h>
//GStepper<STEPPER4WIRE> stepper(2048, 5, 3, 4, 2);
GStepper<STEPPER2WIRE> stepper(200, 26, 27);
int target = 0;
void setup() {
  Serial.begin(115200);

  // режим следования к целевй позиции
  stepper.setRunMode(KEEP_SPEED);

  // установка макс. скорости в шагах/сек
  stepper.setMaxSpeed(10000);

  stepper.setSpeed(20);

  // установка ускорения в шагах/сек/сек
  stepper.setAcceleration(500);
}

void loop() {
  stepper.tick();
  
}