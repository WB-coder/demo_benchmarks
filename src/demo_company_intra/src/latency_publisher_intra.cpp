#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

namespace demo_company_intra
{

class LatencyPublisher : public rclcpp::Node
{
public:
  explicit LatencyPublisher(const rclcpp::NodeOptions & options)
  : Node("latency_publisher", options)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Header>("latency_test", 10);
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&LatencyPublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Publisher initialized with intra-process communication");
  }

private:
  void timer_callback()
  {
    auto message = std::make_unique<std_msgs::msg::Header>();
    message->stamp = this->now();
    message->frame_id = "latency_test_frame";
    RCLCPP_INFO(this->get_logger(), "Publishing at: %d.%09d", 
                message->stamp.sec, message->stamp.nanosec);
    publisher_->publish(std::move(message));
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher_;
};

}  // namespace demo_company_intra

RCLCPP_COMPONENTS_REGISTER_NODE(demo_company_intra::LatencyPublisher)
