#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class LatencyRec : public rclcpp::Node
{
public:
  LatencyRec()
  : Node("LatencyRec")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(0)).best_effort().durability_volatile();
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "pkg_send",
      qos,
      [this](std_msgs::msg::String::UniquePtr msg) {
        this->pub_->publish(*msg);
        //RCLCPP_INFO(this->get_logger(), "Received : %i bytes", msg->data.size());
      });

    pub_ = this->create_publisher<std_msgs::msg::String>(
      "pkg_reply",
      qos);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LatencyRec>());
  rclcpp::shutdown();
  return 0;
}
