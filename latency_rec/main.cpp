#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class LatencyRec : public rclcpp::Node
{
public:
  LatencyRec(bool log_it)
    : Node("LatencyRec"), log_it_(log_it)
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
    auto rec_time = get_microseconds();

    // read send time
    auto snd_time = *reinterpret_cast<long long*>(&msg->data[0]);

    // final message ? :-)
    if (snd_time == 42)
    {
      // do evaluation
      evaluate(latency_array_, rec_size_, warmups_);

      // reset latency array and receive size
      latency_array_.clear();
      rec_size_ = 0;
    }
    else
    {
      // calculate latency and store it into latency array
      auto latency = rec_time - snd_time;
      latency_array_.push_back(latency);

      // store receive size
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

  void evaluate(std::vector<long long>& lat_arr, size_t rec_size, size_t warmups)
  {
    // remove warmup runs
    if (lat_arr.size() >= warmups)
    {
      lat_arr.erase(lat_arr.begin(), lat_arr.begin() + warmups);
    }

    // evaluate all
    size_t sum_msg = lat_arr.size();
    std::cout << "--------------------------------------------" << std::endl;
    std::cout << "Messages received             : " << sum_msg  << std::endl;
    if (sum_msg > warmups)
    {
      long long sum_time = std::accumulate(lat_arr.begin(), lat_arr.end(), 0LL);
      long long avg_time = sum_time / sum_msg;
      auto      min_it = std::min_element(lat_arr.begin(), lat_arr.end());
      auto      max_it = std::max_element(lat_arr.begin(), lat_arr.end());
      size_t    min_pos = min_it - lat_arr.begin();
      size_t    max_pos = max_it - lat_arr.begin();
      long long min_time = *min_it;
      long long max_time = *max_it;
      std::cout << "Message size received         : " << rec_size / 1024 << " kB"        << std::endl;
      std::cout << "Message average latency       : " << avg_time << " us"               << std::endl;
      std::cout << "Message min latency           : " << min_time << " us @ " << min_pos << std::endl;
      std::cout << "Message max latency           : " << max_time << " us @ " << max_pos << std::endl;
      std::cout << "Throughput                    : " << static_cast<int>(((rec_size * sum_msg) / 1024.0) / (sum_time / 1000.0 / 1000.0))          << " kB/s"  << std::endl;
      std::cout << "                              : " << static_cast<int>(((rec_size * sum_msg) / 1024.0 / 1024.0) / (sum_time / 1000.0 / 1000.0)) << " MB/s"  << std::endl;
      std::cout << "                              : " << static_cast<int>(sum_msg / (sum_time / 1000.0 / 1000.0))                                  << " Msg/s" << std::endl;
    }
    std::cout << "--------------------------------------------" << std::endl;
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::vector<long long>                                 latency_array_;
  const size_t                                           warmups_  = 10;
  size_t                                                 rec_size_ = 0;
  bool                                                   log_it_   = false;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LatencyRec>(false));
  rclcpp::shutdown();
  return 0;
}
