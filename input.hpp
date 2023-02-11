/********************************************************************************
* input.hpp: Contains miscellaneous input functions to read data such as strings, 
*            integers and floating point numbers from the terminal.
********************************************************************************/
#ifndef INPUT_HPP_
#define INPUT_HPP_

/* Include directives: */
#include <iostream>
#include <string>
#include <sstream>

/********************************************************************************
* input: Namespace containing miscellaneous input functions.
********************************************************************************/
namespace input
{
   /********************************************************************************
   * readline: Reads a line from the terminal and stores in referenced string.
   *           As default, a new line is printed to generate space between the
   *           entered line and next input/output.
   * 
   *           - s    : Reference to string for storage of entered content.
   *           - space: Characters to print after entered line (default = "\n").
   ********************************************************************************/
   void readline(std::string& s,
                 const char* space = "\n")
   {
      std::getline(std::cin, s);
      if (space) std::cout << space;
      return;
   }

   /********************************************************************************
   * get_integer: Returns an integer of specified data type read from the terminal.
   *              As default, a new line is printed to generate space between the
   *              entered line and next input/output.
   * 
   *              - space: Characters to print after entered line (default = "\n").
   ********************************************************************************/
   template<class T = int>
   T get_integer(const char* space = "\n")
   {
      std::string s;

      while (1)
      {
         readline(s, space);

         try
         {
            return static_cast<T>(stoi(s));
         }
         catch (std::invalid_argument&)
         {
            std::cout << "Invalid argument, try again!\n\n";
         }
      }
   }

   /********************************************************************************
   * get_double: Returns a floating point number read from the terminal. Supported
   *             decimal points are comma (',') and dot ('.'). As default, a new 
   *             line is printed to generate space between the entered line and 
   *             next input/output.
   *
   *              - space: Characters to print after entered line (default = "\n").
   ********************************************************************************/
   double get_double(void)
   {
      std::string s;

      while (1)
      {
         readline(s);

         for (auto& i : s)
         {
            if (i == ',') i = '.';
         }

         try
         {
            return stod(s);
         }
         catch (std::invalid_argument&)
         {
            std::cout << "Invalid argument, try again!\n\n";
         }
      }
   }

   /********************************************************************************
   * read: Returns a value of specified data type read from the terminal. 
   *       As default, a new line is printed to generate space between the entered 
   *       line and next input/output.
   *
   *       - space: Characters to print after entered line (default = "\n").
   ********************************************************************************/
   template<class T = int>
   T read(const char* space = "\n")
   {
      T val{};
      std::string s;
      readline(s, space);
      std::stringstream stream(s);
      stream >> val;
      return val;
   }
}

#endif /* INPUT_HPP_ */