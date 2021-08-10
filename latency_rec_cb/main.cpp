#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class StringSubscriber : public rclcpp::Node
{
public:
  StringSubscriber()
  : Node("string_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "String",
      10,
      [this](std_msgs::msg::String::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received a message");
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StringSubscriber>());
  rclcpp::shutdown();
  return 0;
}
