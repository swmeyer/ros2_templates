/** 
 * Author: Stephanie Meyer swmeyer16@gmail.com 19 Jul 2022
 * Brief: example interface with the ros2 foxy bag api to read messages from bag directly into a ros node
 * File: bag_read_node.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

// #include "rcutils/time.h"

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
// #include "rosidl_runtime_cpp/message_type_support_decl.hpp"

#include <iostream>
#include <fstream>


//Messages
    #include "std_msgs/msg/float32.hpp"
    #include "sensor_msgs/msg/point_cloud2.hpp"
//

// --------------------------

class BagWriteNode : public rclcpp::Node
{
    public:
        BagWriteNode() :
            Node("bagwrite_node")
        {
            //main content

            //Declare Parameters
            this->declare_parameter<std::string>("bagfile_in", "../rosbag2_test_data");
            this->declare_parameter<std::string>("bagfile_out", "../rosbag2_write");
            this->declare_parameter<std::string>("topics_to_write", "");

            //Get Parameters
            std::string bagfile_in, bagfile_out;
            this->get_parameter<std::string>("bagfile_in", bagfile_in);
            this->get_parameter<std::string>("bagfile_out", bagfile_out);
            this->get_parameter<std::string>("topics_to_write", this->topics_to_write);


            //Setup reader
            this->reader = new rosbag2_cpp::readers::SequentialReader();

            RCLCPP_INFO(this->get_logger(), "Reading from bag: %s", bagfile_in.c_str());
            this->storage_options_read.uri = bagfile_in;
            this->storage_options_read.storage_id = "sqlite3";

            this->converter_options.input_serialization_format = "cdr";
            this->converter_options.output_serialization_format = "cdr";


            //Setup writer
            this->writer = new rosbag2_cpp::writers::SequentialWriter();

            RCLCPP_INFO(this->get_logger(), "Writing to bag: %s", bagfile_out.c_str());
            this->storage_options_write.uri = bagfile_out;
            this->storage_options_write.storage_id = "sqlite3";
            this->writer->open(this->storage_options_write, this->converter_options);

            
            //clear outfiles:
            std::ofstream file("output.txt");
            file.close();
            file.open("output2.txt");
            file.close();

            this->readBag();

        }

        ~BagWriteNode()
        {
            delete reader;
            delete writer;
        }

    private:
        //Variables
        std::string topics_to_write;
        
        //Objects
        rosbag2_cpp::readers::SequentialReader* reader;
        // rosbag2_cpp::StorageOptions storage_options{};
        rosbag2_storage::StorageOptions storage_options_read{};
        rosbag2_cpp::ConverterOptions converter_options{};


        rosbag2_cpp::writers::SequentialWriter* writer;
        rosbag2_storage::StorageOptions storage_options_write{};


        // Methods

        bool processMessage(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialized_msg, const std::string& topic_type)
        {
            std::ofstream file2("output2.txt", std::ios_base::app);
            file2.close();

            //Make a new bag message, if we didn't already have one:
            // rosbag2_storage::SerializedBagMessage::SharedPtr bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            //rcutils_ret_t ret = rcutils_system_time_now(&bag_message->time_stamp);
            // if (ret != RCL_RET_OK) 
            // {
            //     FAIL() << "couldn't assign time rosbag message";
            // }

            // //Make new topic info, if we didn't already have it:
            // rosbag2_storage::TopicMetadata topic_metadata;
            // topic_metadata.name = "/my/test/topic";
            // topic_metadata.type = "test_msgs/msg/BasicTypes";
            // topic_metadata.serialization_format = "cdr";
            // writer.create_topic(topic_metadata);
        
            // bag_message->topic_name = topic_metadata.name;
            // bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
            // &serialized_msg.get_rcl_serialized_message(), [](rcutils_uint8_array_t * /* data */) {});


            if (this->topics_to_write.find(serialized_msg->topic_name) != std::string::npos)
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

            if (serialized_msg->topic_name == "/joystick/brake_cmd_override")
            {

                //Write to csv file:
                std_msgs::msg::Float32 msg;
                rclcpp::Serialization<std_msgs::msg::Float32> serialization;
                rclcpp::SerializedMessage extracted_serialized_msg(*serialized_msg->serialized_data);
                serialization.deserialize_message(&extracted_serialized_msg, &msg);

                file2.open("output2.txt", std::ios_base::app);
                file2 << "   " << msg.data << "\n";
                file2.close();


                //write to bag:
                // serialized_msg->topic_name  = "/joystick/brake_cmd"; //optionally rename the topic!
                // this->writer->write(serialized_msg);

            }

            return true;
        }

        void readBag()
        {
            this->reader->open(this->storage_options_read, this->converter_options);
            
            std::ofstream file("output.txt", std::ios_base::app);
            file << "Reading bag " << this->storage_options_read.uri << "\n";
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
    rclcpp::spin(std::make_shared<BagWriteNode>());
    rclcpp::shutdown();
    return(0);
}
