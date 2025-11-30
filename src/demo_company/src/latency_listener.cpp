#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;

class LatencyListener : public rclcpp::Node
{
public:
  LatencyListener()
  : Node("latency_listener")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Header>(
      "latency_test", 10, std::bind(&LatencyListener::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::Header::SharedPtr msg) const
  {
    rclcpp::Time now = this->now();
    rclcpp::Time msg_time = msg->stamp;
    rclcpp::Duration latency = now - msg_time;
    
    RCLCPP_INFO(this->get_logger(), "Received at: %f, Latency: %f µs", 
      now.seconds(), latency.nanoseconds() / 1000.0);
  }
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LatencyListener>());
  rclcpp::shutdown();
  return 0;
}
