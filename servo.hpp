/********************************************************************************
* servo.hpp: Includes drives for PID controlled servos with TOF (Time Of Flight)
*            sensors to read the relative angle, followed by regulating the
*            servo angle towards the target.
********************************************************************************/
#ifndef SERVO_HPP_
#define SERVO_HPP_

/* Include directives: */
#include "pid_controller.hpp"
#include "tof_sensor.hpp"

/********************************************************************************
* servo: Struct for implementation of PID controlled servos.
*        Two TOF (Time Of Flight) sensors are used to read the relative 
*        angle of the servo. The PID controller regulates the servo angle
*        according to the sensor input to steer towards specified target angle.
********************************************************************************/
struct servo
{
   pid_controller pid;      /* PID controller for regulating the servo angle. */
   tof_sensor left_sensor;  /* Left TOF sensor, indicates relative distance to the left. */
   tof_sensor right_sensor; /* Right TOF sensor, indicates relative distance to the right.  */


   /********************************************************************************
   * servo: Initiates servo with specified parameters. As default, the target
   *        angle is set to 90 degrees (center), with a range of 0 - 180 degrees,
   *        where 0 means all the way to the left and 180 means all the way to
   *        the right. The TOF sensor boundry values is set to 0 as minimum and
   *        1023 as maximum. Finally the PID parameters are set to default values
   *        suitable for most applications.
   *        
   *        - target_angle: Target angle for servo.
   *        - angle_min   : Minimum servo angle (default = 0, i.e full left).
   *        - angle_max   : Maximum servo angle (default = 180, i.e full right).
   *        - input_min   : Minimum input value for sensors (default = 0).
   *        - input_max   : Maximum input value for sensors (default = 1023).
   *        - kp          : Proportional constant for PID controller (default = 1).
   *        - ki          : Integrate constant for PID controller (default = 0.01).
   *        - kd          : Derivate constant for PID controller (default = 0.1).
   ********************************************************************************/
   servo(const double target_angle = 90,
         const double angle_min = 0,
         const double angle_max = 180,
         const double input_min = 0,
         const double input_max = 1023,
         const double kp = 1,
         const double ki = 0.01,
         const double kd = 0.1)
   {
      init(target_angle, angle_min, angle_max, input_min, input_max, kp, ki, kd);
      return;
   }

   /********************************************************************************
   * init: Initiates servo with specified parameters. As default, the target
   *       angle is set to 90 degrees (center), with a range of 0 - 180 degrees,
   *       where 0 means all the way to the left and 180 means all the way to
   *       the right. The TOF sensor boundry values is set to 0 as minimum and
   *       1023 as maximum. Finally the PID parameters are set to default values
   *       suitable for most applications.
   *
   *       - target_angle: Target angle for servo.
   *       - angle_min   : Minimum servo angle (default = 0, i.e full left).
   *       - angle_max   : Maximum servo angle (default = 180, i.e full right).
   *       - input_min   : Minimum input value for sensors (default = 0).
   *       - input_max   : Maximum input value for sensors (default = 1023).
   *       - kp          : Proportional constant for PID controller (default = 1).
   *       - ki          : Integrate constant for PID controller (default = 0.01).
   *       - kd          : Derivate constant for PID controller (default = 0.1).
   ********************************************************************************/
   void init(const double target_angle = 90,
             const double angle_min = 0,
             const double angle_max = 180,
             const double input_min = 0,
             const double input_max = 1023,
             const double kp = 1,
             const double ki = 0.01,
             const double kd = 0.1)
   {
      pid.init(target_angle, angle_min, angle_max, kp, ki, kd);
      left_sensor.init(input_min, input_max);
      right_sensor.init(input_min, input_max);
      return;
   }

   /********************************************************************************
   * target: Returns the target angle of the servo.
   ********************************************************************************/
   double target(void) const
   {
      return pid.target;
   }

   /********************************************************************************
   * output: Returns the current output angle of the servo.
   ********************************************************************************/
   double output(void) const
   {
      return pid.output;
   }

   /********************************************************************************
   * input_range: Returns the range of the input values, i.e. the difference
   *              between specified max and min values.
   ********************************************************************************/
   double input_range(void) const
   {
      return left_sensor.input_range();
   }

   /********************************************************************************
   * input_difference: Returns the difference between the input signals of left 
   *                   and right TOF sensor.
   *
   *                   For example, if the left sensor reads 500 while the right 
   *                   sensor reads 700, the difference between the input signals 
   *                   is 500 - 700 = -200, which is returned.
   ********************************************************************************/
   double input_difference(void) const
   {
      return left_sensor.val - right_sensor.val;
   }

   /********************************************************************************
   * input_mapped: Returns relative input measured between values of left and
   *               right TOF sensor, mapped to scale with the servo angle and
   *               centered to the target.
   ********************************************************************************/
   double input_mapped(void) const
   {
      return input_ratio() * (pid.target * 2);
   }

   /********************************************************************************
   * input_ratio: Returns the ratio of the difference between the input values
   *              for left and right sensor as a number between 0 to 1. 
   * 
   *              For example, assume the input min and max are set to 0 and 1023 
   *              respectively and the left sensor reads 500 while the right sensor 
   *              reads 700. Then the difference between the input signals is 
   *              500 - 700 = -200 while the input range is 1023 - 0 = 1023.
   * 
   *              To scale the difference from a value between [-1023, 1023] to
   *              [0, 1023], the input range is added and the sum is divided by 2. 
   *              Therefore, the scaled input is (-200 + 1023) / 2 = 411.5.
   *              
   *              Finally the ratio of the scaled input is calculated by dividing
   *              the scaled input value with the input range. Therefore, the
   *              input ratio is 411.5 / 1023 = 0.4, i.e 40 % of max. 
   *              This ratio is returned after calculation.
   ********************************************************************************/
   double input_ratio(void) const
   {
      const auto scaled_input = (input_difference() + input_range()) / 2.0;
      return scaled_input / input_range();
   }

   /********************************************************************************
   * print: Prints target value, input, output and last measured error for servo,
   *        along with servo angle relative to the target. The output is printed
   *        in the terminal with one decimal as default.
   *
   *        - ostream     : Reference to output stream used (default = std::cout).
   *        - num_decimals: Number of printed decimals per parameter (default = 1).
   ********************************************************************************/
   void print(std::ostream& ostream = std::cout,
              const int num_decimals = 1) const
   {
      ostream << std::fixed << std::setprecision(num_decimals);
      ostream << "--------------------------------------------------------------------------------\n";
      ostream << "Target servo angle:\t\t" << target() << "\n";
      ostream << "Mapped input value:\t\t" << input_mapped() << "\n";
      ostream << "Current servo angle:\t\t" << output() << "\n\n";
      print_relative_angle();
      ostream << "--------------------------------------------------------------------------------\n\n";
      return;
   }

   /********************************************************************************
   * print_relative_angle: Prints the servo angle relative to target. As default,
   *                       the angle is printed in the terminal with one decimal.
   * 
   *                       - ostream     : Reference to output stream used 
   *                                       (default = std::cout).
   *                       - num_decimals: Number of printed decimals per parameter 
   *                                       (default = 1).
   ********************************************************************************/
   void print_relative_angle(std::ostream& ostream = std::cout,
                             const int num_decimals = 1) const
   {
      ostream << std::fixed << std::setprecision(num_decimals);

      if (output() < target())
      {
         ostream << "The servo is angled " << target() - output()
                 << " degrees to the left of target!\n";
      }
      else if (output() > target())
      {
         ostream << "The servo is angled " << output() - target()
                 << " degrees to the right of target!\n";
      }
      else
      {
         ostream << "The servo is angled right at target!\n";
      }
      return;
   }

   /********************************************************************************
   * run: Reads input values for left and right TOF sensor, regulates the servo
   *      angle according to the input and prints the result in the terminal.
   ********************************************************************************/
   void run(void)
   {
      std::cout << "Enter input for left sensor:\n";
      left_sensor.read_from_terminal();
      std::cout << "Enter input for right sensor:\n";
      right_sensor.read_from_terminal();

      pid.regulate(input_mapped());
      print();
      return;
   }

};

#endif /* SERVO_HPP_ */