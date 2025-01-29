//Credit to Rob Tillaart for the DRV8825 driver
//https://github.com/RobTillaart/DRV8825

//TODO:
//Write switch case for doing each valve
//The switch case will be a letter for each valve
//Find drivers for lin act

//This doesn't need to be made with RTOS right?

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
    char action = 0;
    switch (action) {
        case 0: //Stepper Motor, the fuel actuator
            //whatever
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
        // close    open        close    open   backward
        //You may notice that there could be more variations, like
        //if P_FWD and P_BCKWD are both closed
        //In short, DO NOT DO THIS
        //Closing a backwards and forwards input with easily and swiftly
        //fry the H-Bridge, killing it before it could lead a fruitful life
        //So just follow the table and everything is hunky dory
        //Source: https://www.modularcircuits.com/blog/articles/h-bridge-secrets/h-bridges-the-basics/
        case 1: //Linear Actuator, P
            //whatever
            break;
        case 3: //Linear Actuator, N
            //whatever
            break;
        case 4: //Valve Driver, N2O Valve
            //whatever
            break;
        case 5: //Valve Driver, N2 Valve
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
