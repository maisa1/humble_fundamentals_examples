#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

class pubsubCpp : public rclcpp::Node  //class definition
{
    public:
        pubsubCpp() : Node("pubsub_node"), counter_(0)
        {
            // Create publisher for String messages on "chatter" topic with queue size 10
            publisher_ = this->create_publisher<std_msgs::msg::String>("chatter",10);

            subscriber_ = this->create_subscription<std_msgs::msg::String>("chatter",10,
                                                                        std::bind(&pubsubCpp::dataCallback,this,
                                                                        std::placeholders::_1));
                                                         
            RCLCPP_INFO(this->get_logger(), "Publishing_Started");

            timer_ = this->create_wall_timer(
                                             std::chrono::seconds(1),
                                             std::bind(&pubsubCpp::timerCallback, this));                                                                    
        }
    private:
        void timerCallback()
        {  
            auto message = std_msgs::msg::String();
            message.data = "Message#" + std::to_string(counter_);
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "hello %d",counter_);
            counter_++;
        }

        void dataCallback(const std_msgs::msg::String::SharedPtr message)
        {
            RCLCPP_INFO(this->get_logger(), "Received: %s", message->data.c_str());
           
        }
        //==================these two_lines_refers_to_Class_member===============//
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
            rclcpp::TimerBase::SharedPtr timer_;
        //=======================================================================//
            int counter_ ;
};
int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pubsubCpp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}