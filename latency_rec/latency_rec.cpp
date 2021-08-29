#include <rclcpp/rclcpp.hpp>
#include <rcutils/cmdline_parser.h>
#include <std_msgs/msg/string.hpp>

#include "latency_log.h"

class LatencyRec : public rclcpp::Node
{
public:
  LatencyRec(int delay, std::string& log_file)
    : Node("LatencyRec"), delay_(delay), log_file_(log_file)
  {
    // prepare timestamp array to avoid allocations
    latency_array_.reserve(10000);

    // define qos
    auto qos = rclcpp::QoS(rclcpp::KeepLast(0)).best_effort().durability_volatile();

    // create subscriber for topic 'ping'
    sub_ = create_subscription<std_msgs::msg::String>("ping", qos, std::bind(&LatencyRec::OnReceive, this, std::placeholders::_1));
  }

  void OnReceive(std_msgs::msg::String::UniquePtr msg)
  {
    // take receive time
    long long rec_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    // read send time
    auto snd_time = *reinterpret_cast<long long*>(&msg->data[0]);

    // final message ? :-)
    if (snd_time == 42)
    {
      // evaluate all
      evaluate(latency_array_, rec_size_, warmups_, log_file_);

      // log all latencies into file
      log2file(latency_array_, rec_size_, log_file_);

      // reset latency array and receive size
      latency_array_.clear();
      rec_size_ = 0;
    }
    else
    {
      // update latency and size
      auto latency = rec_time - snd_time;
      latency_array_.push_back(latency);
      rec_size_ = msg->data.size();
      // delay callback
      if (delay_ > 0) std::this_thread::sleep_for(std::chrono::milliseconds(delay_));
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::vector<long long>                                 latency_array_;
  const size_t                                           warmups_  = 10;
  size_t                                                 rec_size_ = 0;
  size_t                                                 delay_    = 0;
  std::string                                            log_file_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  int delay(100); // callback process delay in ms
  {
    char* cli_option = rcutils_cli_get_option(argv, argv + argc, "-d");
    if (cli_option) delay = std::atoi(cli_option);
  }
  std::string log_file; // base file name to export results
  {
    char* cli_option = rcutils_cli_get_option(argv, argv + argc, "-l");
    if (cli_option) log_file = cli_option;
  }

  rclcpp::spin(std::make_shared<LatencyRec>(delay, log_file));
  rclcpp::shutdown();

  return 0;
}
