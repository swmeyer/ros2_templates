/** 
 * Author: Stephanie Meyer swmeyer16@gmail.com 19 Jul 2022
 * Brief: example interface with the ros2 foxy bag api to read messages from bag directly into a ros node
 * File: bag_read_node.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"

#include <rosbag2_cpp/readers/sequential_reader.hpp>
// #include <rosbag2_cpp/typesupport_helpers.hpp>
// #include "rosidl_runtime_cpp/message_type_support_decl.hpp"

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>


//Messages
    #include "std_msgs/msg/float32.hpp"
    #include "sensor_msgs/msg/point_cloud2.hpp"
//

// --------------------------

class TemplateNode : public rclcpp::Node
{
    public:
        TemplateNode() :
            Node("template_node")
        {
            //main content

            //Declare Parameters
            this->declare_parameter<std::string>("bagfile_path", "test_bag");
            this->declare_parameter<std::string>("bagfile", "../rosbag2_test_data");
            this->declare_parameter<std::string>("topics_to_record", "");

            //Get Parameters
            std::string bagfile_path, bagfile;
            this->get_parameter<std::string>("bagfile_path", bagfile_path);
            this->get_parameter<std::string>("bagfile", bagfile);
            this->get_parameter<std::string>("topics_to_record", this->topics_to_record);

            this->reader = new rosbag2_cpp::readers::SequentialReader();

            boost::filesystem::path bagfile_path_, bagfile_, full_bagpath;
            bagfile_path_ = bagfile_path;
            bagfile_ = bagfile;
            full_bagpath = bagfile_path_/bagfile_;

            this->storage_options.uri = full_bagpath.string();
            this->storage_options.storage_id = "sqlite3";

            this->converter_options.input_serialization_format = "cdr";
            this->converter_options.output_serialization_format = "cdr";
            
            //clear outfiles:
            std::ofstream file("output.txt");
            file.close();
            file.open("output2.txt");
            file.close();

            this->readBag();

        }

        ~TemplateNode()
        {
            delete reader;
        }

    private:
        //Variables
        std::string topics_to_record;
        
        //Objects
        rosbag2_cpp::readers::SequentialReader* reader;
        rosbag2_cpp::StorageOptions storage_options{};
        rosbag2_cpp::ConverterOptions converter_options{};


        // Methods

        bool processMessage(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialized_msg, const std::string& topic_type)
        {
            std::ofstream file2("output2.txt", std::ios_base::app);
            file2.close();

            if (this->topics_to_record.find(serialized_msg->topic_name) != std::string::npos)
            {

                file2.open("output2.txt", std::ios_base::app);
                file2 << "Recording topic " << serialized_msg->topic_name << "\n";
                file2.close();
            } else
            {
                file2.open("output2.txt", std::ios_base::app);
                file2 << "Skipping topic " <<serialized_msg->topic_name << "\n";
                file2.close();
                return false;
            }

            // deserialization and conversion to ros message

            auto ros_message = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
            ros_message->time_stamp = 0;
            ros_message->message = nullptr;
            ros_message->allocator = rcutils_get_default_allocator();

            rosbag2_cpp::SerializationFormatConverterFactory factory;
            std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer> cdr_deserializer;
            cdr_deserializer = factory.load_deserializer("cdr");

            // ros message data
            if (serialized_msg->topic_name == "/joystick/brake_cmd_override")
            {
                const rosidl_message_type_support_t * type_support = rosidl_typesupport_cpp::get_message_type_support_handle<std_msgs::msg::Float32>();
                std_msgs::msg::Float32 msg;
                ros_message->message = &msg;
                cdr_deserializer->deserialize(serialized_msg, type_support, ros_message);

                file2.open("output2.txt", std::ios_base::app);
                file2 << "   " << msg.data << "\n";
                file2.close();

                //NOTE! We can also publish msg out on a topic, or process it directly in an algorithm node

            } else if (serialized_msg->topic_name == "/joystick/accelerator_cmd_max")
            {
                const rosidl_message_type_support_t * type_support = rosidl_typesupport_cpp::get_message_type_support_handle<std_msgs::msg::Float32>();
                std_msgs::msg::Float32 msg;
                ros_message->message = &msg;
                cdr_deserializer->deserialize(serialized_msg, type_support, ros_message);

                file2.open("output2.txt", std::ios_base::app);
                file2 << "   " << msg.data << "\n";
                file2.close();

            } else if (serialized_msg->topic_name == "/luminar_left_points")
            {
                const rosidl_message_type_support_t * type_support = rosidl_typesupport_cpp::get_message_type_support_handle<sensor_msgs::msg::PointCloud2>();
                sensor_msgs::msg::PointCloud2 msg;
                ros_message->message = &msg;
                cdr_deserializer->deserialize(serialized_msg, type_support, ros_message);

                file2.open("output2.txt", std::ios_base::app);
                file2 << "   " << msg.height << "\n";
                file2 << "   " << msg.width << "\n";
                // There's more pointcloud2 members... print them out at will, or work with the pointclouds directly in code!
                file2.close();
            }

            return true;
        }

        void readBag()
        {
            this->reader->open(this->storage_options, this->converter_options);
            
            std::ofstream file("output.txt", std::ios_base::app);
            file << "Reading bag " << this->storage_options.uri << "\n";
            file.close();
            std::vector<rosbag2_storage::TopicMetadata> topics = this->reader->get_all_topics_and_types();
            
            file.open("output.txt", std::ios_base::app);
            file << "Found " << topics.size() << " topics\n";
            file.close();
            std::map<std::string, std::string> topics_map;

            // about metadata
            for (rosbag2_storage::TopicMetadata topic:topics)
            {
                file.open("output.txt", std::ios_base::app);
                file << "meta name: " << topic.name << std::endl;
                file << "meta type: " << topic.type << std::endl;
                file.close();
 
                topics_map.insert(std::pair<std::string, std::string>(topic.name, topic.type));
            }
            
            // read and deserialize "serialized data"
            while (reader->has_next()){
                file.open("output.txt", std::ios_base::app);
                file << "\nReading messages\n\n";
                file.close();

                // serialized data
                std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialized_message = reader->read_next();
                std::string topic_type = topics_map[serialized_message->topic_name];

                file.open("output.txt", std::ios_base::app);
                file << "     Topic: " << serialized_message->topic_name << "\n";
                file << "     Stamp: " << serialized_message->time_stamp << "\n";
                file << "     Type:  " << topic_type << "\n"; 
                file.close();

                if (this->processMessage(serialized_message, topic_type))
                {
                    file.open("output.txt", std::ios_base::app);
                    file << "    successs\n";
                    file.close();
                } else
                {
                    file.open("output.txt", std::ios_base::app);
                    file << "    skipped\n";
                    file.close();
                }
            }

            file.open("output.txt", std::ios_base::app);
            file << "\nFinished processing ros bag \n";
            file.close();
        }

}; //end class TemplateNode

int main(int argc, char* argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemplateNode>());
    rclcpp::shutdown();
    return(0);
}
