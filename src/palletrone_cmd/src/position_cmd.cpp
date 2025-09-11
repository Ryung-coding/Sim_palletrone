#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <palletrone_interfaces/msg/cmd.hpp>

static constexpr double X_CMD = 0.0;
static constexpr double Y_CMD = 0.0;
static constexpr double Z_CMD = 5.0;
static constexpr int    RATE_HZ = 400;

class PositionCmd : public rclcpp::Node {
public:
  PositionCmd() : rclcpp::Node("position_cmd")
  {
    using palletrone_interfaces::msg::Cmd;

    pub_cmd_ = this->create_publisher<Cmd>("/cmd", 10);

    auto period_ms = std::chrono::milliseconds(1000 / (RATE_HZ > 0 ? RATE_HZ : 1));
    timer_ = this->create_wall_timer(
      period_ms,
      std::bind(&PositionCmd::onTick, this));

    RCLCPP_INFO(this->get_logger(), "position_cmd up (/%s @ %d Hz)",
                pub_cmd_->get_topic_name(), RATE_HZ);
  }

private:
  void onTick()
  {
    using palletrone_interfaces::msg::Cmd;
    Cmd msg;
    msg.pos_cmd[0] = static_cast<float>(X_CMD);
    msg.pos_cmd[1] = static_cast<float>(Y_CMD);
    msg.pos_cmd[2] = static_cast<float>(Z_CMD);

    pub_cmd_->publish(msg);
  }

  rclcpp::Publisher<palletrone_interfaces::msg::Cmd>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionCmd>());
  rclcpp::shutdown();
  return 0;
}
