#include <algorithm>
#include <cinttypes>
#include <chrono>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "helloworld_msgs/msg/hello.hpp"

#include <cstdio>

using namespace std::chrono_literals;

class ExamplePublisher : public rclcpp::Node
{
public:
  explicit ExamplePublisher() : Node("helloworld_publisher" /* Name of the node */)
  {
    // This callback will be called every seconds.
    auto publish_message = [this]() -> void
    {
      msg_->userid++;
      std::unique_ptr<helloworld_msgs::msg::Hello> msg = std::make_unique<helloworld_msgs::msg::Hello>(*msg_);

      // Print log.
      RCLCPP_INFO(this->get_logger(), "Sending message:");
      RCLCPP_INFO(this->get_logger(), "  userid: %" PRIu32, msg->userid);
      RCLCPP_INFO(this->get_logger(), "  message: %s", msg->message.c_str());

      // Publish message.
      pub_->publish(std::move(*msg));
    };

    // Set QoS of the Publisher.
    rclcpp::QoS qos(rclcpp::KeepLast(10));

    // Create initial message
    msg_ = std::make_unique<helloworld_msgs::msg::Hello>();
    msg_->userid = 0;
    msg_->message = strdup("Hello!");

    // Create Publisher.
    pub_ = this->create_publisher<helloworld_msgs::msg::Hello>("helloworld", qos);
    timer_ = this->create_wall_timer(1s, publish_message);
  }

private:
  std::unique_ptr<helloworld_msgs::msg::Hello> msg_;
  rclcpp::Publisher<helloworld_msgs::msg::Hello>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  // Initialize any global resources needed by the middleware and the client library.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Create a node.
  auto node = std::make_shared<ExamplePublisher>();

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
