#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

class publisherCpp : public rclcpp::Node  //class definition
{
    public:
        publisherCpp() : Node("publisher_node"), counter_(0)
        {
            // Create publisher for String messages on "chatter" topic with queue size 10
            publisher_ = this->create_publisher<std_msgs::msg::String>("chatter",10);
            RCLCPP_INFO(this->get_logger(), "Publishing_Started");

            //std::bind(&class_name::callback_function)
            timer_ = this->create_wall_timer(
                                             std::chrono::seconds(1),
                                             std::bind(&publisherCpp::timerCallback, this));                               
        }
    private:
        void timerCallback()
        {  
            // Step 1: Create empty message (auto Let compiler infer the type automatically)
            auto message = std_msgs::msg::String();

            // Step 2: Fill the data field
            message.data = "Message#" + std::to_string(counter_);

            // 3. PUBLISH the message to all subscibers 
            publisher_->publish(message);

            // 4. Log publication
            RCLCPP_INFO(this->get_logger(), "hello %d",counter_);
            counter_++;
        }
        //==================these two_lines_refers_to_Class_member===============//
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
            rclcpp::TimerBase::SharedPtr timer_;
        //=======================================================================//
            int counter_ ;
};
int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<publisherCpp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}