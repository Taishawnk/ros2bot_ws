#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node
{
public:
    SimpleSubscriber() : Node("SimpleSubscriber")
    {
        sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&SimpleSubscriber::msgCallback, this, _1)); // inherited from Node class
    }

    void msgCallback(const std_msgs::msg::String &msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // initialize ROS 2
    auto node = std::make_shared<SimpleSubscriber>();
    rclcpp::spin(node); // spin until node is shut down
    rclcpp::shutdown(); // shutdown ROS 2 node when main function exits
    return 0;
}