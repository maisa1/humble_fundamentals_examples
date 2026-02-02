#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <chrono>


class AddTwoIntsClientNode : public rclcpp::Node  //class definition
{
    public:
        AddTwoIntsClientNode() : Node("add_two_ints_client")
        {
            client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
               

            RCLCPP_INFO(this->get_logger(),"client started") ;                                                              
        }
            void callbackAddTwoInts(int a ,int b)
        {

            while (!client_->wait_for_service(std::chrono::seconds(1)))
            {
            RCLCPP_WARN(this->get_logger(),"waiting for the server....");

            }

            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            request->a = a;
            request->b = b;
            client_->async_send_request(
                request,std::bind(&AddTwoIntsClientNode::AddTwoIntsResponsecallback, this,std::placeholders::_1));
        }
    private:
        void AddTwoIntsResponsecallback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
        {

            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "sum: %d", (int)response->sum);
        }


        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;

};
int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();
    node->callbackAddTwoInts(10,5);
    node->callbackAddTwoInts(10,2);
    node->callbackAddTwoInts(3,5);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}