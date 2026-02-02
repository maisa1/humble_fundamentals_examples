#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include <chrono>

class pubsubCpp : public rclcpp::Node  //class definition
{
    public:
        pubsubCpp() : Node("number_counter"), counter_(0)
        {
            this->declare_parameter("publish_queue_size", 10);
            this->declare_parameter("subscribe_queue_size", 10);

            int publish_queue_size = this->get_parameter("publish_queue_size").as_int();
            int subscribe_queue_size = this->get_parameter("subscribe_queue_size").as_int();

            // Create publisher for String messages on "chatter" topic with queue size 10
            publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count",publish_queue_size);

            subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number",subscribe_queue_size,
                                                                        std::bind(&pubsubCpp::dataCallback,this,
                                                                        std::placeholders::_1));                                                                                                                                          
        }
    private:
        void dataCallback(const example_interfaces::msg::Int64::SharedPtr message)
        {
            auto count_msg = example_interfaces::msg::Int64();
            counter_ += message->data;
            count_msg.data = counter_;
            publisher_->publish(count_msg);
            RCLCPP_INFO(this->get_logger(), "Received: %ld", message->data);
           
                }      
        //==================these two_lines_refers_to_Class_member===============//
            rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
            rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;            
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