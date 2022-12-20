/** 
 * Author: Stephanie Meyer swmeyer16@gmail.com 7 March 2022
 * Brief: example ROS2 wrapper/node template. 
 *        This source code contains the main function, or the start of this c++ executable. 
 *        It does not have an accompanying hpp or header file, as it is our standard to keep the ros wrapper for each
 *        node simple and consistent, and to instantiate the node as a class that is declared and defined all in the cpp file.
 *        All ros-specific functionality, such as subscriptions, publishers, parameter handling, and so on, should be handled in this wrapper.
 *        This node will have instances of whatever ros-agnostic logic classes it needs, and will call functions within those classes to 
 *        execute the desired behavor for this application. 
 * File: ros_node.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/float32_multi_array.hpp>

#include "template_package/agnostic_logic_class.hpp"

#include "rcl_interfaces/msg/set_parameters_result.hpp"
// --------------------------

class TemplateNode : public rclcpp::Node
{
    public:
        TemplateNode() :
            Node("template_node")
        {
            //Define callback group(s)
            // rclcpp::callback_group::CallbackGroup::SharedPtr my_callback_group1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::CallbackGroup::SharedPtr my_callback_group1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


            rclcpp::SubscriptionOptions callback_options1;
            callback_options1.callback_group = my_callback_group1;


            // rclcpp::callback_group::CallbackGroup::SharedPtr my_callback_group2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::CallbackGroup::SharedPtr my_callback_group2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


            rclcpp::SubscriptionOptions callback_options2;
            callback_options2.callback_group = my_callback_group2;


            //Declare Parameters
            this->declare_parameter<std::string>("input_topic", "/input");
            this->declare_parameter<std::string>("output_topic1", "/output");
            this->declare_parameter<std::string>("output_topic2", "/output_stream");
            this->declare_parameter<bool>("publish", false);
            this->declare_parameter<int>("x_max", 0);
            this->declare_parameter<int>("timer_ms", 50);

            //Get Parameters
            std::string input_topic, output_topic1, output_topic2;
            this->get_parameter<std::string>("input_topic", input_topic);
            this->get_parameter<std::string>("output_topic1", output_topic1);
            this->get_parameter<std::string>("output_topic2", output_topic2);
            this->get_parameter<bool>("publish", this->publish);
            this->get_parameter<int>("x_max", this->settings.x_max);

                
            //Publishers and Subscribers
            this->msg_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(input_topic, 1, std::bind(&TemplateNode::exampleCallback, this, std::placeholders::_1), callback_options1);
            
            this->msg_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(output_topic1, 1);
            this->msg_stream_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(output_topic2, 1);

            // Parameter Type Protection and Updater
            this->parameter_updater = this->add_on_set_parameters_callback(std::bind(&TemplateNode::parametersCallback, this, std::placeholders::_1));

            
            //Class Instantiation
            this->template_class = new templates::ExampleClass(this->settings);

            this->timer = this->create_wall_timer(
                    std::chrono::milliseconds(this->get_parameter("timer_ms").as_int()), std::bind(&TemplateNode::timerCallback, this));

        }

        ~TemplateNode()
        {
            delete template_class;
        }

    private:
        //Variables
        bool publish;

        templates::ExampleClass* template_class;
        templates::ExampleClass::Settings settings;

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr msg_sub;

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr msg_pub;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr msg_stream_pub;

        OnSetParametersCallbackHandle::SharedPtr parameter_updater;


        rclcpp::TimerBase::SharedPtr timer; 


        //Methods

        void updateParameters()
        {
            //NOTE: if the parameter changed on the parameter server, this->settings will hold the new values!

            this->template_class->updateSettings(this->settings); //Send the new settings into the class
        }

        //Handles type protection on updated parameters!
        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
        {
            rcl_interfaces::msg::SetParametersResult result; 
            result.successful = true ; 
            result.reason = "No Checks" ; 
            
            for(const rclcpp::Parameter &param : parameters)
            {
      
                if(param.get_name() == "publish")
                {
                    if(param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
                    {
                        this->publish = param.as_bool();
                        result.successful = true; 
                        result.reason = "success ;)";

                    } else
                    {
                        result.successful = false ; 
                        result.reason = "Incorrect Type" ; 
                    }
                } else if (param.get_name() == "x_max")
                {
                    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                    {
                        this->settings.x_max == param.as_int();
                        result.successful = true; 
                        result.reason = "success ;)";
                    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
                    {
                        this->settings.x_max == static_cast<int>(param.as_double());
                        result.successful = true; 
                        result.reason = "success ;)";
                    } else
                    {
                        result.successful = false ; 
                        result.reason = "Incorrect Type" ; 
                    }
                }
            }
            return result; 
        }

        void exampleCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
            // RCLCPP_INFO(this->get_logger(), "Received input message");
            //DO something here!
            updateParameters();

            //ask the class to perform some functionality:
            this->template_class->exampleOverloadedFunction();
            std::string output_string("This is an overload string!\n");
            this->template_class->exampleOverloadedFunction(output_string);
            template_class->setBeispiel(0);

            if (this->publish)
            {
                std_msgs::msg::Float32MultiArray msg_copy;
                msg_copy.data = msg->data;
                msg_pub->publish(msg_copy); //use a publisher to publish output
            }
        }

        void timerCallback()
        {

            // RCLCPP_INFO(this->get_logger(), "Stepped into timer callback");

            //This function will be called at a set rate, defined by the "timer_ms" param used to create the timer

            updateParameters();

            if (this->publish)
            {
                //Make a new instance of the message type!
                std_msgs::msg::Float32MultiArray new_msg;

                //Make some values for the message! This message type is an array (or vector) or floats
                std::vector<float> data_vector;
                data_vector.push_back(100);
                data_vector.push_back(200);

                //Set the data member of the msg to be equal to the new vector that we've made
                new_msg.data = data_vector;

                //Send the message out to the topic that's attached to the msg_stream_pub publisher!
                this->msg_stream_pub->publish(new_msg);
            }
        }


}; //end class TemplateNode

int main(int argc, char* argv[])
{   
    rclcpp::init(argc, argv);

    // This makes a multi-threaded node with simultaneous callback execution

    rclcpp::Node::SharedPtr node_handle_1 = std::make_shared<TemplateNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_handle_1);
    executor.spin();

    rclcpp::shutdown();
    return(0);
}