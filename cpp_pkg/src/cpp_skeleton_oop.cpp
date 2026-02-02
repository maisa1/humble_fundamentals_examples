#include "rclcpp/rclcpp.hpp"

class skeleton_cpp : public rclcpp::Node
{
    public:
        skeleton_cpp() : Node("cpp_test_oop"), counter_(0)
        {
            RCLCPP_INFO(this->get_logger(), "Hello cpp oop");
            timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                             std::bind(&skeleton_cpp::timerCallback, this));
        }
    private:

        void timerCallback()
        {
            RCLCPP_INFO(this->get_logger(), "hello %d",counter_);
            counter_++;
        }
            rclcpp::TimerBase::SharedPtr timer_;
            int counter_ ;

};

int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<skeleton_cpp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}