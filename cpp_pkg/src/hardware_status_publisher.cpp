#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"
#include <chrono>

class publisherCpp : public rclcpp::Node  //class definition
{
    public:
        publisherCpp() : Node("hardware_publisher_status")
        {
            // Create publisher for String messages on "chatter" topic with queue size 10
            publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status",10);
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
            auto message = my_robot_interfaces::msg::HardwareStatus();

            message.temperture = 52.57;
            message.are_motors_ready = false;
            message.debug_message = "Motors are too hot";
            publisher_->publish(message);
        }
        //==================these two_lines_refers_to_Class_member===============//
            rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr publisher_;
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