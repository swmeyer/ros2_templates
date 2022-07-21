/** 
 * Author: Stephanie Meyer swmeyer16@gmail.com 7 March 2022
 * Brief: example ROS2 wrapper/node template
 * File: template_node.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/float32_multi_array.hpp>

#include "template_package/template.hpp"
// --------------------------

namespace templates
{

class TemplateNode : public rclcpp::Node
{
    public:
        TemplateNode() :
            Node("template_node")
        {
            //main content

            //Declare Parameters
            this->declare_parameter<std::string>("input_topic", "/input");
            this->declare_parameter<std::string>("output_topic", "/output");
            this->declare_parameter<bool>("publish", false);
            this->declare_parameter<int>("x_max", 0);

            //Get Parameters
            std::string input_topic, output_topic;
            this->get_parameter<std::string>("input_topic", input_topic);
            this->get_parameter<std::string>("output_topic", output_topic);
            this->get_parameter<bool>("publish", this->publish);
            this->get_parameter<int>("x_max", this->settings.x_max);

            //Class Instantiation
            this->template_class = new examples::ExampleClass(this->settings);
                
            //Publishers and Subscribers
            msg_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(input_topic, 1, std::bind(&TemplateNode::exampleCallback, this, std::placeholders::_1));
            
            msg_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(output_topic, 1);
        
        }

        ~TemplateNode()
        {
            delete template_class;
        }

    private:
        //Variables
        bool publish;

        examples::ExampleClass* template_class;
        examples::ExampleClass::Settings settings;

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr msg_sub;

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr msg_pub;


        //Methods

        void updateParameters()
        {
            //Get Parameters
            this->declare_parameter<bool>("publish", false);
            this->get_parameter<int>("x_max", this->settings.x_max); //Collect a new parameter value off the ros param server

            this->template_class->updateSettings(this->settings); //Send the new settings into the class
        }

        void exampleCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
            //DO something here!
            updateParameters();

            template_class->setBeispiel(0); //ask the class to perform some functionality
            if (this->publish)
            {
                msg_pub->publish(*msg); //use a publisher to publish output
            }
        }


}; //end class TemplateNode

}//end namespace templates

int main(int argc, char* argv[])
{   
    rclcpp::init(argc, argv);

    // //This makes a single-threaded node with blocking callbacks
    // rclcpp::spin(std::make_shared<templates::TemplateNode>());

    // This makes a multi-threaded node with simultaneous callback execution
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(std::make_shared<templates::TemplateNode>());
    executor.spin();


    rclcpp::shutdown();
    return(0);
}
