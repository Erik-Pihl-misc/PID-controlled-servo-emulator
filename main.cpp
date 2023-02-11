/********************************************************************************
* main.cpp: Emulator for PID controlled servo with two TOF sensors to read
*           the relative angle. The servo target angle is set to 90 degrees 
*           (center), with minimum and maximum angle set to 30 and 150 degrees 
*           (maximum angle 60 degrees to the left and right respectively). 
*           The minimum and maximum values of the TOF sensors are set to 0
*           and 1023 respectively. 
*
*           The servo is run continuously, where the used gets to enter values 
*           for left and right TOF sensor. The servo angle is then regulated to 
*           keep the servo heading towards the target. The target angle and 
*           current servo angle is printed in the terminal along with mapped
*           input value read from the sensors.
********************************************************************************/
#include "servo.hpp"

/********************************************************************************
* main: Initates a new servo and runs it continuously.
********************************************************************************/
int main(void)
{
   servo servo1(90, 30, 150, 0, 1023);
   
   while (1)
   {
      servo1.run();
   }

   return 0;
}