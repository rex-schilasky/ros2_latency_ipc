#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class StringPublisher : public rclcpp::Node
{
public:
  StringPublisher()
  : Node("string_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("String", 10);
    message_ = std_msgs::msg::String();
    message_.data = "Hello, world! ";
    for(auto i = 0; i < 14; ++i) message_.data += message_.data;
    
      auto timer_callback =
      [this]() -> void {
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_.data.c_str());
        this->publisher_->publish(this->message_);
      };
      
    timer_ = this->create_wall_timer(100ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr                         timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  publisher_;
  std_msgs::msg::String                                message_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StringPublisher>());
  rclcpp::shutdown();
  return 0;
}
