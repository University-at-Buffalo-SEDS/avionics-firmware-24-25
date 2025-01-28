//Credit to Rob Tillaart for the DRV8825 driver
//https://github.com/RobTillaart/DRV8825

//TODO:
//Write switch case for doing each valve
//The switch case will be a letter for each valve
//Find drivers for

#include "DRV8825.h"

DRV8825 stepper;
const int DIRECTION_PIN = 9; //PA1
const int STEP_PIN = 8; //PA0



void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("DRV8825_LIB_VERSION: ");
  Serial.println(DRV8825_LIB_VERSION);

  //write code to display letter assignments to user
  //i.e. type A for motor, C for whatever

  stepper.begin(DIRECTION_PIN, STEP_PIN);

}


void loop()
{
    int action = 0;
    switch (action) {
        case 0: //Stepper Motor, the fuel actuator
            //whatever
            break;
        case 1: //Linear Actuator, P
            //whatever
            break;
        case 3: //Linear Actuator, N
            //whatever
            break;
        case 4: //Logic High, N2O Valve
            //whatever
            break;
        case 5: //Logic High, N2 Valve
            //whatever
            break;
        default:
            //nothing
            break;
    }

    stepper.setDirection(DRV8825_CLOCK_WISE);
    stepper.setDirection(DRV8825_COUNTERCLOCK_WISE);
    stepper.step();

}


// -- END OF FILE --
