#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <palletrone_interfaces/msg/cmd.hpp>

static constexpr int RATE_HZ = 400;

class PositionCmd : public rclcpp::Node {
public:
  PositionCmd() : rclcpp::Node("position_cmd")
  {
    using palletrone_interfaces::msg::Cmd;

    pub_cmd_ = this->create_publisher<Cmd>("/cmd", 10);

    auto period_ms = std::chrono::milliseconds(1000 / (RATE_HZ > 0 ? RATE_HZ : 1));
    t0_ = std::chrono::steady_clock::now();
    timer_ = this->create_wall_timer(period_ms,std::bind(&PositionCmd::onTick, this));

  }

private:
  void onTick()
  {
    using palletrone_interfaces::msg::Cmd;
    Cmd msg;
    // double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
    // double w = 2.0 * M_PI / 40.0;
    // double s = std::sin(w*t), c = std::cos(w*t);
    // double x = 2.0 * s;
    // double y = 1.0 * s * c;
    // double z = 5.0 + 0.5 * std::sin(w*t + M_PI/2.0);
    // msg.pos_cmd[0] = static_cast<float>(x);
    // msg.pos_cmd[1] = static_cast<float>(y);
    // msg.pos_cmd[2] = static_cast<float>(z);
    msg.pos_cmd[0] = 0.0; // override x for testing
    msg.pos_cmd[1] = 0.0; // override y for testing
    msg.pos_cmd[2] = 1.5; // override z for testing

    msg.att_cmd[0] = 15.0*M_PI/180.0; // roll
    msg.att_cmd[1] = 0.0; // pitch
    msg.att_cmd[2] = 0.0; // yaw

    pub_cmd_->publish(msg);
  }

  rclcpp::Publisher<palletrone_interfaces::msg::Cmd>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point t0_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionCmd>());
  rclcpp::shutdown();
  return 0;
}
