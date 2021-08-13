#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/string.hpp"

class LatencySnd : public rclcpp::Node
{
public:
  LatencySnd(int runs, int snd_size, int delay, bool log_it)
    : Node("LatencySnd"),
    runs_(runs),
    snd_size_(snd_size),
    delay_(delay),
    log_it_(log_it)
  {
    // log test
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << " RUNS       : " << runs_                                           << std::endl;
    std::cout << " SEND SIZE  : " << snd_size_ << " kB"                              << std::endl;
    std::cout << " SEND DELAY : " << delay_    << " ms"                              << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    // create message string
    msg_ = std_msgs::msg::String();
    snd_size_ *= 1024;
    msg_.data.resize(snd_size_);

    // prepare timestamp array to avoid allocations
    latency_array_.reserve(runs_);

    // define qos
    auto qos = rclcpp::QoS(rclcpp::KeepLast(0)).best_effort().durability_volatile();

    // create publisher for topic 'pkg_send'
    pub_ = create_publisher<std_msgs::msg::String>("pkg_send", qos);

    // create subscriber for topic 'pkg_reply'
    sub_ = create_subscription<std_msgs::msg::String>("pkg_reply", qos, std::bind(&LatencySnd::OnReceive, this, std::placeholders::_1));

    // create and start timer for publishing with 100 ms
    auto timer_delay = std::chrono::milliseconds(delay);
    timer_ = create_wall_timer(timer_delay, std::bind(&LatencySnd::OnPublish, this));
  }

  void OnPublish()
  {
    // store publication time
    last_snd_time_ = get_microseconds();
    *reinterpret_cast<long long*>(&msg_.data[0]) = last_snd_time_;

    // check for termination
    if (snd_pkgs_ < runs_)
    {
      // and publish the message
      pub_->publish(msg_);
      snd_pkgs_++;

      // log it
      if(log_it_) RCLCPP_INFO(get_logger(), "Sent     : %i bytes", msg_.data.size());
    }
    else
    {
      // send final message
      *reinterpret_cast<long long*>(&msg_.data[0]) = 42;
      pub_->publish(msg_);

      // stop timer
      timer_->cancel();
      // destroy subscriber
      sub_ = nullptr;

      // calculate roundtrip time over all received messages
      if (!latency_array_.empty())
      {
        long long sum_time = std::accumulate(latency_array_.begin(), latency_array_.end(), 0LL);
        long long avg_time = sum_time / latency_array_.size();
        std::cout << std::endl;
        std::cout << "-----------------------------------------------------------------" << std::endl;
        std::cout << "Messages sent                      : " << snd_pkgs_                << std::endl;
        std::cout << "Messages received                  : " << latency_array_.size()    << std::endl;
        std::cout << "Message average latency            : " << avg_time << " us"        << std::endl;
        std::cout << "-----------------------------------------------------------------" << std::endl;
      }

      // shutdown here
      rclcpp::shutdown();
    }
  }

  void OnReceive(std_msgs::msg::String::UniquePtr msg)
  {
    // calculate latency
    auto snd_time    = *reinterpret_cast<long long*>(&msg->data[0]);
    auto rec_time    = get_microseconds();
    auto rec_latency = rec_time - snd_time;

    // store delta time for later experiment evaluation
    latency_array_.push_back(rec_latency);
    
    // log it
    if (log_it_) RCLCPP_INFO(get_logger(), "Received : %i bytes", msg->data.size());
    if (log_it_) RCLCPP_INFO(get_logger(), "Latency  : %i us\n", rec_latency);
  }

private:
  long long get_microseconds()
  {
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    return(std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count());
  }

  rclcpp::TimerBase::SharedPtr                           timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std_msgs::msg::String                                  msg_;

  std::vector<long long>                                 latency_array_;
  long long                                              last_snd_time_ = 0;
  size_t                                                 snd_pkgs_ = 0;

  size_t                                                 runs_ = 0;
  size_t                                                 snd_size_ = 0;
  size_t                                                 delay_ = 0;
  bool                                                   log_it_ = false;
};

int main(int argc, char* argv[])
{
  // initialize
  rclcpp::init(argc, argv);

  // paramter 'runs'
  int runs(100);
  {
    char* cli_option = rcutils_cli_get_option(argv, argv + argc, "-r");
    if (cli_option) runs = std::atoi(cli_option);
  }
  // parameter 'size' in kB
  int snd_size(1);
  {
    char* cli_option = rcutils_cli_get_option(argv, argv + argc, "-s");
    if (cli_option) snd_size = std::atoi(cli_option);
  }
  // parameter 'delay' in ms
  int delay(100);
  {
    char* cli_option = rcutils_cli_get_option(argv, argv + argc, "-d");
    if (cli_option) delay = std::atoi(cli_option);
  }

  // spin it
  rclcpp::spin(std::make_shared<LatencySnd>(runs, snd_size, delay, false));

  // shutdown
  rclcpp::shutdown();

  return 0;
}
