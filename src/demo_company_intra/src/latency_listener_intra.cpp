#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/header.hpp"

namespace demo_company_intra
{

class LatencyListener : public rclcpp::Node
{
public:
  explicit LatencyListener(const rclcpp::NodeOptions & options)
  : Node("latency_listener", options)
  {
    subscription_ = this->create_subscription<std_msgs::msg::Header>(
      "latency_test", 10, std::bind(&LatencyListener::topic_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Listener initialized with intra-process communication");
  }

private:
  void topic_callback(std_msgs::msg::Header::UniquePtr msg)
  {
    rclcpp::Time now = this->now();
    rclcpp::Time msg_time = msg->stamp;
    rclcpp::Duration latency = now - msg_time;
    
    RCLCPP_INFO(this->get_logger(), "Received at: %f, Latency: %.3f µs", 
                now.seconds(), latency.nanoseconds() / 1000.0);
  }
  
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr subscription_;
};

}  // namespace demo_company_intra

RCLCPP_COMPONENTS_REGISTER_NODE(demo_company_intra::LatencyListener)
