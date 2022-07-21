/** 
 * Author: Stephanie Meyer swmeyer16@gmail.com 7 March 2022
 * Brief: example source file for a ros-agnostic class template. This is where the logic for your solution/application gets defined. 
 *              This code essentially building out the contents of the blueprints in the accopanying hpp, or header file, which has the 
 *              same name as this cpp and can be found in the include/ folder
 * File: agnostic_logic_class.cpp
 */

// -------------------------------
#include "template_package/agnostic_logic_class.hpp"
// -------------------------------

namespace templates
{            
    /**
     * @function    ExampleClass
     * @brief       constructor for example class
     * @param       settings - settings to use in this class
     */
    ExampleClass::ExampleClass(const Settings& settings)
    {
        this->settings = settings;
    }
    
    /**
     * @function    ~ExampleClass
     * @brief       destructor for example class
     * @param       none
     */
    ExampleClass::~ExampleClass()
    {

    }


    //Optional functions:

    //--------- Default Parameter and Overloaded Function Examples ------/

    /**
     * @function    exampleDefaultParameterFunction
     * @brief       the final parameters in the list of funciton params can be set to a value in the function declaration,
     *              and if the user doesn't send in values for these parameters, the default values will be used
     * @param       int1 - example normal parameter
     * @param       int2 - example parameter with default-value
     * @return      void
     */
    void ExampleClass::exampleDefaultParameterFunction(int& int1, const int& int2)
    {
        std::cout << "Int1: " << int1 << ", int2: " << int2 << "\n";
    }

    /**
     * @function    exampleOverloadedFunction
     * @brief       example that shows off function overloading - using the same function name with different parameters
     * @param       input_string - example string parameter
     * @return      void
     */
    void ExampleClass::exampleOverloadedFunction()
    {   
        std::string msg = "Hello!";
        this->exampleOverloadedFunction(msg);
    }
    void ExampleClass::exampleOverloadedFunction(std::string& input_string)
    {
        std::cout << "Template class says " << input_string << "\n";
    }

    //Methods (Class Functions):

    /**
     * @function    templateFunction
     * @brief       describe the function
     * @param       pt1 - example parameter
     * @return      void
     */
    void ExampleClass::templateFunction(const int& pt1)
    {
        //Do something here :D
    }

    /**
     * @function    isInTolerance
     * @brief       checks if the given two numbers are within the given tolerance of each other
     * @param       num1 - first number to compare
     * @param       num2 - second number to compare
     * @param       tolerance - acceptable difference between the two
     * @return      bool - true if the numbers are in tolerance, 
     *                     false otherwise
     */
    bool ExampleClass::isInTolerance(const double& num1, const double& num2, const double& tolerance)
    {
        if (fabs(num1 - num2) > tolerance) return false;

        return true;
    }


} //end namepsace templates