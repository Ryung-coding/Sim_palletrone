#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <palletrone_interfaces/msg/cmd.hpp>

static constexpr int RATE_HZ = 400;
static constexpr double HOVER_SEC  = 7.0;   
static constexpr double HOVER_ALT  = 2.0;  
static constexpr double TILT_ANGLE = 25.0 * M_PI / 180.0;

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

    double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();

    if(t < HOVER_SEC) {
      msg.pos_cmd[0] = 0.0;
      msg.pos_cmd[1] = 0.0;
      msg.pos_cmd[2] = HOVER_ALT;
      msg.att_cmd[0] = 0.0;
      msg.att_cmd[1] = 0.0;
      msg.att_cmd[2] = 0.0;
    } else {
      double tm = t - HOVER_SEC;
      double w = 2.0 * M_PI / 5.0;
      double s = std::sin(w * tm), c = std::cos(w * tm);

      msg.pos_cmd[0] = 2.0 * std::sin(TILT_ANGLE) * s;
      msg.pos_cmd[1] = 0.0;
      msg.pos_cmd[2] = HOVER_ALT + 2.0 * std::sin(TILT_ANGLE) * c;

      msg.att_cmd[0] =  TILT_ANGLE * c;
      msg.att_cmd[1] =  0.0;
      msg.att_cmd[2] = -TILT_ANGLE * s;
    }

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
