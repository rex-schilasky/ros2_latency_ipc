#include <rclcpp/rclcpp.hpp>
#include <rcutils/cmdline_parser.h>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class LatencySnd : public rclcpp::Node
{
public:
  LatencySnd(int runs, int snd_size, int delay)
    : Node("LatencySnd"), runs_(runs), snd_size_(snd_size), delay_(delay)
  {
    // log test
    std::cout << "----------------------------------------"         << std::endl;
    std::cout << "Runs                    : " << runs_              << std::endl;
    std::cout << "Message size            : " << snd_size_ << " kB" << std::endl;
    std::cout << "Message delay           : " << delay_    << " ms" << std::endl;

    // create message string
    msg_ = std_msgs::msg::String();
    snd_size_ *= 1024;
    msg_.data.resize(snd_size_);

    // define qos
    auto qos = rclcpp::QoS(rclcpp::KeepLast(0)).best_effort().durability_volatile();

    // create publisher for topic 'ping'
    pub_ = create_publisher<std_msgs::msg::String>("ping", qos);

    // finally create and start timer for publishing
    auto timer_delay = std::chrono::milliseconds(delay);
    timer_ = create_wall_timer(timer_delay, std::bind(&LatencySnd::OnPublish, this));
  }

  void OnPublish()
  {
    // check for termination
    if (snd_pkgs_ < runs_ + warmups_)
    {
      // store send time into string (bad style for sure)
      long long snd_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      *reinterpret_cast<long long*>(&msg_.data[0]) = snd_time;

      // and publish the message
      pub_->publish(msg_);
      snd_pkgs_++;
    }
    else
    {
      // stop timer
      timer_->cancel();

      // send final messages to trigger latency_rec to summarize
      *reinterpret_cast<long long*>(&msg_.data[0]) = 42;
      pub_->publish(msg_);

      std::cout << "Messages sent           : " << snd_pkgs_ - warmups_ << std::endl;
      std::cout << "----------------------------------------"           << std::endl;

      // shutdown here
      rclcpp::shutdown();
    }
  }

private:
  rclcpp::TimerBase::SharedPtr                        timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std_msgs::msg::String                               msg_;
  size_t                                              snd_pkgs_ = 0;
  size_t                                              runs_     = 0;
  const size_t                                        warmups_  = 10;
  size_t                                              snd_size_ = 0;
  size_t                                              delay_    = 0;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  int runs(100); // number of publications
  {
    char* cli_option = rcutils_cli_get_option(argv, argv + argc, "-r");
    if (cli_option) runs = std::atoi(cli_option);
  }
  int snd_size(64); // message size in kB
  {
    char* cli_option = rcutils_cli_get_option(argv, argv + argc, "-s");
    if (cli_option) snd_size = std::atoi(cli_option);
  }
  int delay(100); // delay between two publications in ms
  {
    char* cli_option = rcutils_cli_get_option(argv, argv + argc, "-d");
    if (cli_option) delay = std::atoi(cli_option);
  }

  rclcpp::spin(std::make_shared<LatencySnd>(runs, snd_size, delay));
  rclcpp::shutdown();

  return 0;
}
