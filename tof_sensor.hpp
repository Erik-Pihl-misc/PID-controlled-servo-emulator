/********************************************************************************
* tof_sensor.hpp: Include drivers for TOF (Time Of Flight) sensors, where the
*                 input values are entered from the terminal. As default, the
*                 min and max sensor values are set to 0 and 1023 respectively.
********************************************************************************/
#ifndef TOF_SENSOR_HPP_
#define TOF_SENSOR_HPP_

/* Include directives: */
#include <iostream>
#include "input.hpp"

/********************************************************************************
* tof_sensor: Struct for implementation of TOF sensors with adjustable min and
*             max values.
********************************************************************************/
struct tof_sensor
{
   static constexpr auto DEFAULT_MIN = 0.0;    /* Default minimum sensor value. */
   static constexpr auto DEFAULT_MAX = 1023.0; /* Default maximum sensor value. */
   double min = DEFAULT_MIN;                   /* Minimum sensor value. */
   double max = DEFAULT_MAX;                   /* Maximum sensor value. */
   double val  = 0;                            /* Input sensor value. */

   /********************************************************************************
   * tof_sensor: Default constructor, initiates TOF sensor with default parameters.
   ********************************************************************************/
   tof_sensor(void) { }

   /********************************************************************************
   * tof_sensor: Initiates TOF sensor with specified minimum and maximum sensor
   *             values, provided that the maximum value is higher than the 
   *             minimum value.
   * 
   *             - sensor_min: Minimum sensor value.
   *             - sensor_max: Maximum sensor value.
   ********************************************************************************/
   tof_sensor(const double sensor_min, 
              const double sensor_max)
   {
      init(sensor_min, sensor_max);
      return;
   }

   /********************************************************************************
   * init: Initiates TOF sensor with specified minimum and maximum sensor values,
   *       provided that the maximum value is higher than the minimum value.
   *
   *       - sensor_min: Minimum sensor value.
   *       - sensor_max: Maximum sensor value.
   ********************************************************************************/
   void init(const double sensor_min,
             const double sensor_max)
   {
      min = sensor_min >= 0 ? sensor_min : 0;
      max = sensor_max > sensor_min ? sensor_max : 1023;
      return;
   }

   /********************************************************************************
   * read_from_terminal: Reads new sensor value between specified minimum and
   *                     maximum from the terminal.
   ********************************************************************************/
   void read_from_terminal(void)
   {
      val = input::get_double();
      check_sensor_value();
      return;
   }

   /********************************************************************************
   * check_sensor_value: Checks if the specified sensor value is within set minumum
   *                     and maximum. If the sensor value is out of this range, 
   *                     the value is set to nearest boundary. 
   ********************************************************************************/
   void check_sensor_value(void)
   {
      if (val < min)
      {
         val = min;
      }
      else if (val > max)
      {
         val = max;
      }
      return;
   }

   /********************************************************************************
    * input_range: Returns the range of the input values, i.e. the difference
    *              between specified max and min values.
    ********************************************************************************/
   double input_range(void) const
   {
      return max - min;
   }
};

#endif /* TOF_SENSOR_HPP_ */