#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"
#include <chrono>

class pubsubCpp : public rclcpp::Node  //class definition
{
    public:
        pubsubCpp() : Node("number_counter"), counter_(0)
        {
            // Create publisher for String messages on "chatter" topic with queue size 10
            publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count",10);

            subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number",10,
                                                                        std::bind(&pubsubCpp::dataCallback,this,
                                                                        std::placeholders::_1));

            server_ = this->create_service<example_interfaces::srv::SetBool>("reset_counter", 
                std::bind(&pubsubCpp::callbackSetbool,this,
                std::placeholders::_1,std::placeholders::_2));                                                                                                                                            
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
        void callbackSetbool(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                             const example_interfaces::srv::SetBool::Response::SharedPtr response)
        {
            
            // Check if the client wants to reset (data == true)
            if (request->data) {
            // Reset counter to 0
                counter_ = 0;  
            // Set response fields
                response->success = true;
                response->message = "Counter successfully reset to zero";
            
                RCLCPP_INFO(this->get_logger(), "Service called: Counter reset to 0");
        } else {
            // Client sent false, so don't reset
            response->success = false;
            response->message = "Counter not reset - request data was false";
            
            RCLCPP_INFO(this->get_logger(), 
                       "Service called but data was false. Counter remains: %ld", 
                       counter_);
        }
            

        }       
        //==================these two_lines_refers_to_Class_member===============//
            rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
            rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
            rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;            
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