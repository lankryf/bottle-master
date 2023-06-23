#include <Arduino.h>
#include "ConfigPorts.h"
#include "GyverTM1637.h"
#include "GyverEncoder.h"
#include <GyverStepper.h>
#include <max6675.h>
#include <EEPROM.h>

#define INIT_ADDR 90
#define INIT_KEY 50


const int freq = 25000;
const int ledChannel = 0;
const int resolution = 8;

GStepper<STEPPER2WIRE> stepper(1200, stepperDIR, stepperSTEP);
GyverTM1637 disp(CLKDIS, DIO);
Encoder enc1(CLK, DT, SW);
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
hw_timer_t *Bottler = NULL;

unsigned int values[2] = {1, 1};
float temperature = 0;
unsigned int status = 0;
uint32_t timer, timer2, timer3;
unsigned int setting = false;
bool work = false;

void IRAM_ATTR onTimer()
{
  stepper.tick();
}

int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  float err = setpoint - input;
  static float integral = 0, prevErr = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErr) / dt;
  prevErr = err;
  return constrain(err * kp + integral + D * kd, minOut, maxOut);
}

void setup()
{
  Serial.begin(9600);

  // eeprom
  EEPROM.begin(100);
  if (EEPROM.read(INIT_ADDR) != INIT_KEY) {
    EEPROM.write(INIT_ADDR, INIT_KEY);

    EEPROM.put(0, 180);
    EEPROM.commit();
    EEPROM.put(10, 20);
    EEPROM.commit();
  }

  EEPROM.get(0, values[0]);
  EEPROM.get(10, values[1]);


  enc1.setType(TYPE1);
  enc1.setFastTimeout(40);
  disp.clear();
  disp.brightness(3);
  pinMode(TCO, OUTPUT);

  // engine setting
  int eepromTemp, eepromEngine;

  EEPROM.get(0, eepromTemp);
  EEPROM.get(1, eepromEngine);


  stepper.setRunMode(KEEP_SPEED);
  stepper.setMaxSpeed(1700);
  stepper.setSpeed(0);
  Bottler = timerBegin(0, 80, true);
  timerAttachInterrupt(Bottler, &onTimer, true);
  timerAlarmWrite(Bottler, 800, true);
  timerAlarmEnable(Bottler);



  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(TCO, ledChannel);

  // indicator
  pinMode(workIndicator, OUTPUT);

}

void loop()
{

  enc1.tick();


  if (enc1.isRight())
    values[status]++;

  if (enc1.isLeft())
    values[status]--;

  if (enc1.isHolded())
  {
    status = !status;
    disp.displayInt(values[status] * (1 - 2 * status));
    Serial.print('\r', values[status] * (1 - 2 * status));
  }
  if (enc1.isRelease()){
    work = !work;
    if (!work){
      stepper.setSpeed(0);
      digitalWrite(workIndicator, LOW);
    } else {
      stepper.setSpeed(10 * values[1]);
      digitalWrite(workIndicator, HIGH);
    }
  }

  values[0] = constrain(values[0], 50, 350);
  values[1] = constrain(values[1], 1, 100);

  if (millis() - timer3 >= 200)
  {
    timer3 = millis();

    temperature = thermocouple.readCelsius();
    int PIDout = computePID(temperature, values[0], 22.0, 0.5, 10.0, 0.2, 0, 255);
    ledcWrite(ledChannel, PIDout);

    Serial.print(temperature);
    Serial.print(" ");
    Serial.print(values[0]);
    Serial.print(" ");
    Serial.println(PIDout);
  }

  if (enc1.isTurn() || enc1.isRelease())
  {
    disp.displayInt(values[status] * (1 - 2 * status));
    timer = millis();
    setting = true;
  }


  if (setting == 0 && millis() - timer2 >= 1000)
  {
    timer2 = millis();
    disp.displayInt(temperature);
  }



  if (setting && millis() - timer >= 3000)
  {
    timer = millis();
    EEPROM.put(0, values[0]);
    EEPROM.commit();
    EEPROM.put(10, values[1]);
    EEPROM.commit();
    setting = false;
  }
}