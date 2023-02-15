#include <Arduino.h>
#include "ConfigPorts.h"
#include "GyverEncoder.h"
#include "GyverTM1637.h"

class DataControl
{
private:
    Encoder *enc;
    GyverTM1637 *disp;

    unsigned int values[2] = {1, 1};
    unsigned int status = 0;


    void init() {
        enc->setType(TYPE1);
        enc->setFastTimeout(40);
        disp->clear();
        disp->brightness(3);
    }


public:
    DataControl() {
        enc = new Encoder(CLK, DT, SW);
        disp = new GyverTM1637(CLKDIS, DIO);
        init();
    }


    void getEncoder() {
        enc->tick();

        if (enc->isRight()) values[status]++;

        if (enc->isLeft()) values[status]--;

        if (enc->isPress()) status = !status;

        values[0] = constrain(values[0], 120, 250);
        values[1] = constrain(values[1], 1, 100);
    }


    void displayIfTurn() {
        if (enc->isTurn() || enc->isRelease()) displayData();
    }


    void displayData() {
        disp->displayInt(values[status] * ( 1 - 2 * status ));
    }


    ~DataControl(){
        delete enc;
        delete disp;
    }
};