#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class LatencySnd : public rclcpp::Node
{
public:
  LatencySnd()
    : Node("LatencySnd")
  {
    // parameter
    snd_size_ = 1024; // kB
    runs_     = 200;

    // log test
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << " LATENCY / THROUGHPUT TEST"                                        << std::endl;
    std::cout << " RUNS  : " << runs_                                                << std::endl;
    std::cout << " SIZE  : " << snd_size_ << " kB"                                   << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    // create payload
    msg_ = std_msgs::msg::String();
    msg_.data = "Hello, world! ";
    snd_size_ *= 1024;
    while (msg_.data.size() < snd_size_) msg_.data += msg_.data;
    msg_.data.resize(snd_size_);

    // prepare timestamp array
    diff_array_.reserve(runs_);
    snd_time_ = 0;
    snd_pkgs_ = 0;

    // create publisher
    pub_ = this->create_publisher<std_msgs::msg::String>(
      "pkg_send",
      10);

    // create subscriber
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "pkg_reply",
      10,
      [this](std_msgs::msg::String::UniquePtr msg) {
        this->diff_array_.push_back(GetMicroSeconds() - this->snd_time_);
        //RCLCPP_INFO(this->get_logger(), "Received : %i bytes\n", msg->data.size());
      });

    auto timer_callback =
      [this]() -> void {
      // store send time
      this->snd_time_ = GetMicroSeconds();
      // send message
      this->pub_->publish(this->msg_);
      //RCLCPP_INFO(this->get_logger(), "Sent     : %i bytes", msg_.data.size());

      // check for termination
      if (this->snd_pkgs_ >= this->runs_)
      {
        // stop timer
        timer_->cancel();
        // destroy subscriber
        sub_ = nullptr;

        // calculate roundtrip time over all received messages
        long long sum_time = std::accumulate(this->diff_array_.begin(), this->diff_array_.end(), 0LL);
        long long avg_time = sum_time / this->diff_array_.size();
        std::cout << std::endl;
        std::cout << "-----------------------------------------------------------------" << std::endl;
        std::cout << "Messages sent                      : " << this->snd_pkgs_          << std::endl;
        std::cout << "Messages received                  : " << this->diff_array_.size() << std::endl;
        std::cout << "Sum time                           : " << sum_time/1000 << " ms"   << std::endl;
        std::cout << "Message average roundtrip time     : " << avg_time   << " us"      << std::endl;
        std::cout << "Message average roundtrip time / 2 : " << avg_time/2 << " us"      << std::endl;
        std::cout << "-----------------------------------------------------------------" << std::endl;
      }
      this->snd_pkgs_++;
    };
    timer_ = this->create_wall_timer(100ms, timer_callback);
  }

private:
  long long GetMicroSeconds()
  {
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    return(std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count());
  }

  rclcpp::TimerBase::SharedPtr                           timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std_msgs::msg::String                                  msg_;

  int                                                    runs_;
  int                                                    snd_size_;

  std::vector<long long>                                 diff_array_;
  long long                                              snd_time_;
  int                                                    snd_pkgs_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LatencySnd>());
  rclcpp::shutdown();
  return 0;
}
