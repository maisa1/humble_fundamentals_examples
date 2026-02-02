#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class subscriberCpp : public rclcpp::Node  //class definition
{
    public:
        subscriberCpp() : Node("subscriber_node"), counter_(0)
        {

            // ROS2: "Ah! Put the message in the 1st parameter slot (_1)"
            subscriber_ = this->create_subscription<std_msgs::msg::String>("chatter",10,
                                                                        std::bind(&subscriberCpp::dataCallback,
                                                                        this,std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Subscriber started - listening on /chatter");                          
        }
    private:
        void dataCallback(const std_msgs::msg::String::SharedPtr msg)
        {  
            RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
            counter_++;
        }
        //==================this line_refers_to_Class_member===============//
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
            int counter_ ;
};
int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<subscriberCpp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}