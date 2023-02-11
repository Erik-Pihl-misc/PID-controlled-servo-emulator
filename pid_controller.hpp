/********************************************************************************
* pid_controller.hpp: Contains drivers for user friendly PID controllers.
********************************************************************************/
#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

/* Include directives: */
#include <iostream>
#include <iomanip>

/********************************************************************************
* pid_controller: Struct for implementation of PID controllers with adjustable
*                 PID parameters, minimum and maximum output values etc.
********************************************************************************/
struct pid_controller
{
   double target     = 0; /* Desired output value. */
   double output     = 0; /* Real output value. */
   double input      = 0; /* Input value from sensor (used for printing only). */
   double kp         = 0; /* Proportional constant. */
   double ki         = 0; /* Integrate constant. */
   double kd         = 0; /* Derivate constant */
   double integrate  = 0; /* Integral value, multiplied with ki when setting new output. */
   double derivate   = 0; /* Delta value, multiplied with kd when setting new output. */
   double last_error = 0; /* Last measured error. */
   double output_min = 0; /* Minimum output value. */;
   double output_max = 0; /* Maximum output value. */

   /********************************************************************************
   * pid_controller: Default constructor, creates empty pid controller.
   ********************************************************************************/
   pid_controller(void) { }

   /********************************************************************************
   * pid_controller: Initiates PID controller with specified parameters.
   * 
   *                 - target    : Desired output value.
   *                 - output_min: Minimum output value (default = 0).
   *                 - output_max: Maximum output value (default = 180).
   *                 - kp        : Proportional constant (default = 1.0).
   *                 - ki        : Integrate constant (default = 0.01).
   *                 - kd        : Derivate constant (default = 0.1).
   ********************************************************************************/
   pid_controller(const double target,
                  const double output_min = 0,
                  const double output_max = 180,
                  const double kp = 1.0,
                  const double ki = 0.01,
                  const double kd = 0.1)
   {
      this->init(target, output_min, output_max, kp, ki, kd);
      return;
   }

   /********************************************************************************
   * init: Initiates PID controller with specified parameters.
   *
   *       - target    : Desired output value.
   *       - output_min: Minimum output value (default = 0).
   *       - output_max: Maximum output value (default = 180).
   *       - kp        : Proportional constant (default = 1.0).
   *       - ki        : Integrate constant (default = 0.01).
   *       - kd        : Derivate constant (default = 0.1).
   ********************************************************************************/
   void init(const double target,
             const double output_min = 0,
             const double output_max = 180,
             const double kp = 1.0,
             const double ki = 0.01,
             const double kd = 0.1)
   {
      this->target = target;
      this->output_min = output_min;
      this->output_max = output_max;
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
      return;
   }

   /********************************************************************************
   * regulate: Regulates output value of PID controller on the basis of new input. 
   *         
   *           - new_input: New input value of PID controller.
   ********************************************************************************/
   void regulate(const double new_input)
   {
      const auto error = target - new_input;
      input = new_input;
      integrate += error;
      derivate = error - last_error;
      output = target + kp * error + ki * integrate + kd * derivate;
      last_error = error;
      check_output();
      return;
   }

   /********************************************************************************
   * check_output: Checks if the PID controller output value is within specified
   *               minimum and maximum output value. If the output value is
   *               out of this range, the value is set to nearest boundary.
   ********************************************************************************/
   void check_output(void)
   {
      if (output < output_min)
      {
         output = output_min;
      }
      else if (output > output_max)
      {
         output = output_max;
      }
      return;
   }

   /********************************************************************************
   * print: Prints target value, input, output and last measured error for PID
   *        controller. The output is printed in the terminal with one decimal
   *        as default.
   * 
   *        - ostream     : Reference to output stream used (default = std::cout).
   *        - num_decimals: Number of printed decimals per parameter (default = 1).
   ********************************************************************************/
   void print(std::ostream& ostream = std::cout,
              const int num_decimals = 1)
   {
      ostream << std::fixed << std::setprecision(num_decimals);
      ostream << "--------------------------------------------------------------------------------\n";
      ostream << "Target:\t\t" << target << "\n";
      ostream << "Input:\t\t" << input << "\n";
      ostream << "Output:\t\t" << output << "\n";
      ostream << "Last error:\t" << last_error << "\n";
      ostream << "--------------------------------------------------------------------------------\n\n";
      return;
   }
};

#endif /* PID_CONTROLLER_HPP_ */