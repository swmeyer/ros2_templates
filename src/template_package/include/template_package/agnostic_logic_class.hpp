/** 
 * Author: Stephanie Meyer swmeyer16@gmail.com 7 March 2022
 * Brief: example header for a ros-agnostic class template. This is where the logic for your solution/application gets declared. 
 *              You can think of the hpp, the header, as the blueprint for a class, where the class logic is built up or "defined"
 *              in the accompanying cpp, or source file, which can be found in the src/ folder and should share the same name as the hpp
 * File: agnostic_logic_class.hpp
 */

#ifndef TEMPLATE_HPP
#define TEMPLATE_HPP

// -------------------------------
#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/float32_multi_array.hpp>
// -------------------------------

namespace templates
{
    class ExampleClass
    {
        public:

            typedef struct
            {
                int x_max; //[m] maximum x bound (example setting)
            } Settings;
            
            /**
             * @function    ExampleClass
             * @brief       constructor for example class
             * @param       settings - settings to use in this class
             */
            ExampleClass(const Settings& settings);
            
            /**
             * @function    ~ExampleClass
             * @brief       destructor for example class
             * @param       none
             */
            ~ExampleClass();


            void updateSettings(const Settings& settings) { this->settings = settings; } //Fully defined function for updating the settings member


            //Optional functions:

            //--------- Getters/Setters ----------/

            //Example "getter" function - simply returns or fetches a private member variable
            int getBeispiel() { return beispiel; }

            //Example "setter" function - simply sets a private member variable to a given value
            void setBeispiel(const int& beispiel) { this->beispiel = beispiel; }


            //--------- Default Parameter and Overloaded Function Examples ------/

            /**
             * @function    exampleDefaultParameterFunction
             * @brief       the final parameters in the list of funciton params can be set to a value in the function declaration,
             *              and if the user doesn't send in values for these parameters, the default values will be used
             * @param       int1 - example normal parameter
             * @param       int2 - example parameter with default-value
             * @return      void
             */
            void exampleDefaultParameterFunction(int& int1, const int& int2 = 0);

            /**
             * @function    exampleOverloadedFunction
             * @brief       example that shows off function overloading - using the same function name with different parameters
             * @param       input_string - example string parameter
             * @return      void
             */
            void exampleOverloadedFunction();
            void exampleOverloadedFunction(std::string& input_string);

        
        private:
            //Variables:
            bool do_prediction;     // Example optional behavior flag

            int beispiel;           // [m] - Example class-level variable with units of meters

            Settings settings;      // Instantiation of the class settings struct


            //Methods (Class Functions):

            /**
             * @function    templateFunction
             * @brief       describe the function
             * @param       pt1 - example parameter
             * @return      void
             */
            void templateFunction(const int& pt1);

            /**
             * @function    isInTolerance
             * @brief       checks if the given two numbers are within the given tolerance of each other
             * @param       num1 - first number to compare
             * @param       num2 - second number to compare
             * @param       tolerance - acceptable difference between the two
             * @return      bool - true if the numbers are in tolerance, 
             *                     false otherwise
             */
            bool isInTolerance(const double& num1, const double& num2, const double& tolerance);
    

    }; //end class ExampleClass


} //end namepsace templates

#endif // TEMPLATE_HPP
