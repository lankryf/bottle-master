#include <Arduino.h>
#include "ConfigPorts.h"
#include "GyverTM1637.h"
#include "GyverEncoder.h"
#include <GyverStepper.h>
#include <max6675.h>

GStepper<STEPPER2WIRE> stepper(200, stepperDIR, stepperSTEP);
GyverTM1637 disp(CLKDIS, DIO);
Encoder enc1(CLK, DT, SW);


unsigned int values[2] = {1, 1};
int temperature = 0;
unsigned int status = 0;
uint32_t timer, timer2;
unsigned int setting = false;


MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

void setup() {
  Serial.begin(9600);
  enc1.setType(TYPE1);
  enc1.setFastTimeout(40);
  disp.clear();
  disp.brightness(3);


  // engine
  stepper.setRunMode(KEEP_SPEED);
  stepper.setMaxSpeed(1700);
  stepper.setSpeed(200);
  
  values[1] = 20;
}


void loop() {
 
  enc1.tick();
  stepper.tick();
  

  if (enc1.isRight()) values[status]++;

  if (enc1.isLeft()) values[status]--;

  if (enc1.isPress()){
    status = !status;
  }
  stepper.tick();
  values[0] = constrain(values[0], 120, 250);
  values[1] = constrain(values[1], 1, 100);



  if (enc1.isTurn() || enc1.isRelease()) {
    disp.displayInt(values[status]* ( 1 - 2*status ));
    timer = millis();
    setting = true;
  }
  
  stepper.tick();
if (setting==0){
  stepper.tick();
  if ( millis() - timer2 >= 1000) {
        stepper.tick();
    timer2 = millis();
    temperature = thermocouple.readCelsius(); 
    stepper.tick();
    disp.displayInt(temperature);
  }
}

  stepper.tick();
  if (setting && millis() - timer >= 3000) {
      timer = millis();
      stepper.setSpeed(10 * values[1]);
      setting = false;
  }
  stepper.tick();

}