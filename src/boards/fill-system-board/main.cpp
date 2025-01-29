//Credit to Rob Tillaart for the DRV8825 driver
//https://github.com/RobTillaart/DRV8825

//TODO:
//Write switch case for doing each valve
//The switch case will be a letter for each valve
//Other than the DRV8825, all valves and actuators
//only need a logic high from the STM32 to actuate

//This doesn't need to be made with RTOS right?

#include "DRV8825.h"
#include <Arduino.h>

#include "FillSystemSTM32Pinouts.hpp"

DRV8825 stepper;



void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("DRV8825_LIB_VERSION: ");
  Serial.println(DRV8825_LIB_VERSION);

  //write code to display letter assignments to user
  //i.e. type A for motor, C for whatever

  stepper.begin(STEPPER_DIRECTION_PIN, STEPPER_CONTROL_PIN);

}


void loop()
{
    int action = 0;
    switch (action) {
        case 0: //Stepper Motor, the fuel actuator
            stepper.setDirection(DRV8825_CLOCK_WISE);
            stepper.setDirection(DRV8825_COUNTERCLOCK_WISE);
            stepper.step();
            break;
        //LINEAR ACTUATOR HAS A SPECIFIC IMPLEMENTATION: ASK JUSTIN ;)
        //Basically, the way the H-Bridge works means that there are 4 inputs
        //Those being: P_FWD, P_BCKWD, N_FWD, N_BCKWD
        //To work the bridge, one must only "close" two inputs
        //Each input is essentialy a switch, and their function will 
        //determine the motor direction
        //In this case: here are the possible variants with direction
        // P_FWD    P_BCKWD     N_FWD   N_BCKWD Direction
        // open     close       open    close   forward
        // close    open        close   open    backward
        //You may notice that there could be more variations, like
        //if P_FWD and P_BCKWD are both closed
        //In short, DO NOT DO THIS
        //Closing a backwards and forwards input with easily and swiftly
        //fry the H-Bridge, killing it before it could lead a fruitful life
        //So just follow the table and everything is hunky dory
        //Source: https://www.modularcircuits.com/blog/articles/h-bridge-secrets/h-bridges-the-basics/
        case 1: //Linear Actuator, P and N gate
            //whatever
            break;
        case 2: //Valve Driver, N2O Valve
            //whatever
            break;
        case 3: //Valve Driver, N2 Valve
            //whatever
            break;
        default:
            //nothing
            break;
    }


}


// -- END OF FILE --
