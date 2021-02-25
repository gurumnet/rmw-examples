#include <cinttypes>
#include <iomanip>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "helloworld_msgs/msg/hello.hpp"

class ExampleSubscriber : public rclcpp::Node
{
public:
  explicit ExampleSubscriber() : Node("helloworld_subscriber" /* Name of the node */)
  {
    // This callback will be called when the subscriber receives message.
    auto callback = [this](const helloworld_msgs::msg::Hello::SharedPtr msg) -> void
    {
      // Print received message.
      RCLCPP_INFO(this->get_logger(), "Received message:");
      RCLCPP_INFO(this->get_logger(), "  userid: %" PRIu32, msg->userid);
      RCLCPP_INFO(this->get_logger(), "  message: %s", msg->message.c_str());
    };

    // Set QoS of the subscription.
    rclcpp::QoS qos(rclcpp::KeepLast(10));

    // Create subscription.
    sub_ = create_subscription<helloworld_msgs::msg::Hello>("helloworld", qos, callback);
  }

private:
  rclcpp::Subscription<helloworld_msgs::msg::Hello>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  // Initialize any global resources needed by the middleware and the client library.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Create a node.
  auto node = std::make_shared<ExampleSubscriber>();

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
