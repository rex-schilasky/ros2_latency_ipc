#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class LatencyRec : public rclcpp::Node
{
public:
  LatencyRec(bool log_it)
    : Node("LatencyRec"),
    log_it_(log_it)
  {
    // prepare timestamp array to avoid allocations
    latency_array_.reserve(10000);

    // define qos
    auto qos = rclcpp::QoS(rclcpp::KeepLast(0)).best_effort().durability_volatile();

    // create publisher for topic 'pong'
    pub_ = this->create_publisher<std_msgs::msg::String>("pong", qos);

    // create subscriber for topic 'pkg_snd'
    sub_ = create_subscription<std_msgs::msg::String>("ping", qos, std::bind(&LatencyRec::OnReceive, this, std::placeholders::_1));
  }

  void OnReceive(std_msgs::msg::String::UniquePtr msg)
  {
    // take receive time
    auto rec_time = get_microseconds();

    // read send time
    auto snd_time = *reinterpret_cast<long long*>(&msg->data[0]);

    // final message ? :-)
    if (snd_time == 42)
    {
      std::cout << "----------------------------------------"                 << std::endl;
      std::cout << "Messages received       : " << latency_array_.size()      << std::endl;
      if (!latency_array_.empty())
      {
        long long sum_time = std::accumulate(latency_array_.begin(), latency_array_.end(), 0LL);
        long long avg_time = sum_time / latency_array_.size();
        std::cout << "Message size            : " << rec_size_/1024 << " kB"  << std::endl;
        std::cout << "Message average latency : " << avg_time << " us"        << std::endl;
      }
      std::cout << "----------------------------------------"                 << std::endl;

      // shutdown here
      rclcpp::shutdown();
    }
    else
    {
      // calculate latency
      auto latency = rec_time - snd_time;

      // store latency for later experiment evaluation
      latency_array_.push_back(latency);

      // store new send time into msg
      *reinterpret_cast<long long*>(&msg->data[0]) = get_microseconds();

      // and publish back
      pub_->publish(*msg);

      // store receive time
      rec_size_ = msg->data.size();

      // log it
      if (log_it_) RCLCPP_INFO(get_logger(), "Received : %li bytes", msg->data.size());
      if (log_it_) RCLCPP_INFO(get_logger(), "Latency  : %lli us\n", latency);
    }
  }

private:
  long long get_microseconds()
  {
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    return(std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

  std::vector<long long>                                 latency_array_;

  size_t                                                 rec_size_ = 0;
  bool                                                   log_it_ = false;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LatencyRec>(false));
  rclcpp::shutdown();
  return 0;
}
