#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

class LatencyPublisher : public rclcpp::Node
{
public:
  LatencyPublisher()
  : Node("latency_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Header>("latency_test", 10);
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&LatencyPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Header();
    message.stamp = this->now();
    message.frame_id = "latency_test_frame";
    RCLCPP_INFO(this->get_logger(), "Publishing at: %d.%d", message.stamp.sec, message.stamp.nanosec);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LatencyPublisher>());
  rclcpp::shutdown();
  return 0;
}
