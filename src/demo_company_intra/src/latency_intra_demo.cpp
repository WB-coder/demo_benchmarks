#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

// Publisher Node
class LatencyPublisher : public rclcpp::Node
{
public:
  LatencyPublisher()
  : Node("latency_publisher", rclcpp::NodeOptions().use_intra_process_comms(true))
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

// Subscriber Node
class LatencyListener : public rclcpp::Node
{
public:
  LatencyListener()
  : Node("latency_listener", rclcpp::NodeOptions().use_intra_process_comms(true))
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Create both nodes with intra-process communication enabled
  auto publisher = std::make_shared<LatencyPublisher>();
  auto listener = std::make_shared<LatencyListener>();
  
  // Use a single-threaded executor to spin both nodes
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher);
  executor.add_node(listener);
  
  RCLCPP_INFO(rclcpp::get_logger("main"), 
              "Starting intra-process latency demo (both nodes in same process)");
  
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
