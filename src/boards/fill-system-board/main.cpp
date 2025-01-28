//Credit to Rob Tillaart for the DRV8825 driver
//https://github.com/RobTillaart/DRV8825


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

  stepper.begin(DIRECTION_PIN, STEP_PIN);

}


void loop()
{

    stepper.setDirection(DRV8825_CLOCK_WISE);
    stepper.setDirection(DRV8825_COUNTERCLOCK_WISE);
    stepper.step();

}


// -- END OF FILE --
