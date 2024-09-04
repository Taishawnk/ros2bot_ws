#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node
{
public:
    SimplePublisher() : Node("simple_publisher"), counter_(0)
    {
        pub_ = create_publisher<std_msgs::msg::String>("chatter", 10); //created chatter topic
        timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this));
        RCLCPP_INFO(get_logger(), "Publishing at 1Hz");
    }

    void timerCallback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, ROS2 " + std::to_string(counter_++);
        pub_->publish(message);
    }

private:
    unsigned int counter_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}