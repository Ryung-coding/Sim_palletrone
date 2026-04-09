#include <rclcpp/rclcpp.hpp>
#include <palletrone_interfaces/msg/cmd.hpp>
#include <palletrone_interfaces/msg/palletrone_state.hpp>
#include <palletrone_interfaces/msg/wrench.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>

class WrenchController : public rclcpp::Node
{
public:
  WrenchController() : rclcpp::Node("wrench_controller")
  {
    const double KP_POS[3] = {10.0, 10.0, 1.00};
    const double KI_POS[3] = {0.0, 0.0, 0.0};
    const double KD_POS[3] = {5.00, 5.00, 1.50};
    const double I_MIN_POS = -5.0, I_MAX_POS = 5;

    const double KP_ATT[3] = {3.00, 3.00, 3.00};
    const double KI_ATT[3] = {0.0, 0.0, 0.0};
    const double KD_ATT[3] = {0.80, 0.80, 0.80};
    const double I_MIN_ATT = -1.0, I_MAX_ATT = 1.0;

    auto init_pid = [](double kp, double ki, double kd, double i_min, double i_max) -> std::function<double(double,double,double,double)>
    {
      double iacc = 0.0;
      return [=](double ref, double cur, double dcur, double dt) mutable
      {
        if (dt <= 0.0) dt = 1e-3;
        const double e = ref - cur;
        const double de = -dcur;
        iacc += ki * e * dt;
        iacc = std::clamp(iacc, i_min, i_max);
        double u = kp*e + iacc + kd*de;
        return u;
      };
    };

    pid_pos_[0] = init_pid(KP_POS[0], KI_POS[0], KD_POS[0], I_MIN_POS, I_MAX_POS);
    pid_pos_[1] = init_pid(KP_POS[1], KI_POS[1], KD_POS[1], I_MIN_POS, I_MAX_POS );
    pid_pos_[2] = init_pid(KP_POS[2], KI_POS[2], KD_POS[2], I_MIN_POS, I_MAX_POS);

    pid_att_[0] = init_pid(KP_ATT[0], KI_ATT[0], KD_ATT[0], I_MIN_ATT, I_MAX_ATT);
    pid_att_[1] = init_pid(KP_ATT[1], KI_ATT[1], KD_ATT[1], I_MIN_ATT, I_MAX_ATT);
    pid_att_[2] = init_pid(KP_ATT[2], KI_ATT[2], KD_ATT[2], I_MIN_ATT, I_MAX_ATT);

    sub_cmd_ = this->create_subscription<palletrone_interfaces::msg::Cmd>("/cmd", 10, std::bind(&WrenchController::onCmd, this, std::placeholders::_1));
    sub_state_ = this->create_subscription<palletrone_interfaces::msg::PalletroneState>("/palletrone_state", 10, std::bind(&WrenchController::onState, this, std::placeholders::_1));
    pub_wrench_ = this->create_publisher<palletrone_interfaces::msg::Wrench>("/wrench", 10);

    pos_cmd_.setZero();
    att_cmd_.setZero();
    last_time_ = this->now();
  }

private:
  void onCmd(const palletrone_interfaces::msg::Cmd::SharedPtr msg)
  {
    pos_cmd_ << static_cast<double>(msg->pos_cmd[0]), static_cast<double>(msg->pos_cmd[1]), static_cast<double>(msg->pos_cmd[2]); 
    att_cmd_ << static_cast<double>(msg->att_cmd[0]), static_cast<double>(msg->att_cmd[1]), static_cast<double>(msg->att_cmd[2]); 
    have_cmd_ = true; 
    tryPublish();
  }

  void onState(const palletrone_interfaces::msg::PalletroneState::SharedPtr msg)
  {
    pos_ << static_cast<double>(msg->pos[0]), static_cast<double>(msg->pos[1]), static_cast<double>(msg->pos[2]); 
    vel_ << static_cast<double>(msg->vel[0]), static_cast<double>(msg->vel[1]), static_cast<double>(msg->vel[2]); 
    rpy_ << static_cast<double>(msg->rpy[0]), static_cast<double>(msg->rpy[1]), static_cast<double>(msg->rpy[2]); 
    w_body_ << static_cast<double>(msg->w_rpy[0]), static_cast<double>(msg->w_rpy[1]), static_cast<double>(msg->w_rpy[2]); 
    
    have_state_ = true; tryPublish();
  }

  void tryPublish()
  {
    if (!have_state_) return;

    const rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (!(dt > 0.0) || dt > 0.2) dt = 1.0/400.0;

    const Eigen::Vector3d pos_ref = have_cmd_ ? pos_cmd_ : Eigen::Vector3d::Zero();
    Eigen::Vector3d position_pid; 
    position_pid.x() = pid_pos_[0](pos_ref.x(), pos_.x(), vel_.x(), dt); 
    position_pid.y() = pid_pos_[1](pos_ref.y(), pos_.y(), vel_.y(), dt); 
    position_pid.z() = pid_pos_[2](pos_ref.z(), pos_.z(), vel_.z(), dt);

    Eigen::Vector3d F_world; 
    F_world.x() = position_pid.x(); 
    F_world.y() = position_pid.y(); 
    F_world.z() = position_pid.z() + mass_ * grav_;

    const double r = rpy_.x(), p = rpy_.y(), y = rpy_.z();
    const double sr = std::sin(r), cr = std::cos(r), sp = std::sin(p), cp = std::cos(p), sy = std::sin(y), cy = std::cos(y);

    Eigen::Matrix3d R_WB;
    R_WB <<  cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr,
             sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr,
               -sp,             cp*sr,             cp*cr;

    const Eigen::Vector3d F_body = R_WB.transpose() * F_world;

    const Eigen::Vector3d att_ref = have_cmd_ ? att_cmd_ : Eigen::Vector3d::Zero();
    const double y_err = std::atan2(std::sin(att_ref.z() - rpy_.z()), std::cos(att_ref.z() - rpy_.z()));
    const double y_ref_equiv = rpy_.z() + y_err;

    Eigen::Vector3d M_body; 
    M_body.x() = pid_att_[0](att_ref.x(), rpy_.x(), w_body_.x(), dt); 
    M_body.y() = pid_att_[1](att_ref.y(), rpy_.y(), w_body_.y(), dt); 
    M_body.z() = pid_att_[2](y_ref_equiv, rpy_.z(), w_body_.z(), dt);

    palletrone_interfaces::msg::Wrench w; 
    w.moment[0] = static_cast<float>(M_body(0)); 
    w.moment[1] = static_cast<float>(M_body(1)); 
    w.moment[2] = static_cast<float>(M_body(2)); 
    w.force[0] = static_cast<float>(F_body(0)); 
    w.force[1] = static_cast<float>(F_body(1)); 
    w.force[2] = static_cast<float>(F_body(2)); 
    
    pub_wrench_->publish(w);
  }

  rclcpp::Subscription<palletrone_interfaces::msg::Cmd>::SharedPtr sub_cmd_;
  rclcpp::Subscription<palletrone_interfaces::msg::PalletroneState>::SharedPtr sub_state_;
  rclcpp::Publisher<palletrone_interfaces::msg::Wrench>::SharedPtr pub_wrench_;
  rclcpp::Time last_time_;

  Eigen::Vector3d pos_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d att_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d w_body_{Eigen::Vector3d::Zero()};

  std::function<double(double,double,double,double)> pid_pos_[3];
  std::function<double(double,double,double,double)> pid_att_[3];

  const double mass_{4.8}, grav_{9.806};
  bool have_state_{false}, have_cmd_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchController>());
  rclcpp::shutdown();
  return 0;
}
