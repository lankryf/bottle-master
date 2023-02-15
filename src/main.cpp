#include <Arduino.h>
#include "ConfigPorts.h"
#include "GyverTM1637.h"
#include "GyverEncoder.h"
#include <GyverStepper.h>
#include <max6675.h>
#include <GyverPID2.h>

// tune
#include "PIDtuner.h"
PIDtuner2 tuner;


const int freq = 25000; 
const int ledChannel = 0; 
const int resolution = 8;

GStepper<STEPPER2WIRE> stepper(200, stepperDIR, stepperSTEP);
GyverTM1637 disp(CLKDIS, DIO);
Encoder enc1(CLK, DT, SW);
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
hw_timer_t *Bottler = NULL;
GyverPID regulator(5, 0.2, 15, 500);



unsigned int values[2] = {1, 1};
int temperature = 0;
unsigned int status = 0;
uint32_t timer, timer2;
unsigned int setting = false;


void IRAM_ATTR onTimer(){
  // stepper.setSpeed(10 * values[1]);
  stepper.tick();
}


void setup() {
  Serial.begin(9600);
  enc1.setType(TYPE1);
  enc1.setFastTimeout(40);
  disp.clear();
  disp.brightness(3);
  pinMode(TCO , OUTPUT);

  // engine
  stepper.setRunMode(KEEP_SPEED);
  stepper.setMaxSpeed(1700);
  stepper.setSpeed(200);
  values[1] = 20;
  Bottler = timerBegin(0, 80, true);
  timerAttachInterrupt(Bottler, &onTimer, true);
  timerAlarmWrite(Bottler, 800, true);
  timerAlarmEnable(Bottler);
  
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(TCO, ledChannel);

  regulator.setDirection(NORMAL);
  regulator.setpoint = 220;
  regulator.setLimits(0, 255);

  tuner.setParameters(NORMAL, 0, 30, 500, 2, 500);
  
}



void loop() {
  
  enc1.tick();
  // stepper.tick();

  if (enc1.isRight()) values[status]++;

  if (enc1.isLeft()) values[status]--;

  if (enc1.isPress()){
    status = !status;
  }
  values[0] = constrain(values[0], 120, 250);
  values[1] = constrain(values[1], 1, 100);



  if (enc1.isTurn() || enc1.isRelease()) {
    disp.displayInt(values[status]* ( 1 - 2*status ));
    timer = millis();
    setting = true;
  }
  

if (setting==0){
  if ( millis() - timer2 >= 100) {




    timer2 = millis();
    temperature = thermocouple.readCelsius(); 
    regulator.input = temperature;
    // Serial.println(regulator.getResult());
    
    disp.displayInt(temperature);

    // TUNE
    tuner.setInput(temperature);
    tuner.compute();
    ledcWrite(ledChannel, tuner.getOutput());
    // tuner.compute();
    tuner.debugText();
  }
}

  if (setting && millis() - timer >= 3000) {
      timer = millis();

      //engine
      timerAlarmDisable(Bottler);
      stepper.setSpeed(10 * values[1]);
      timerAlarmEnable(Bottler);

      // temperature
      regulator.setpoint = values[0];
      

      setting = false;
  }
}