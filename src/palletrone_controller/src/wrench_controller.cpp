#include <rclcpp/rclcpp.hpp>
#include <palletrone_interfaces/msg/cmd.hpp>
#include <palletrone_interfaces/msg/palletrone_state.hpp>
#include <palletrone_interfaces/msg/wrench.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

class WrenchController : public rclcpp::Node 
{
public:
  WrenchController() : rclcpp::Node("wrench_controller")
  {
    mass_ = 4.0;   // [kg]
    grav_ = 9.81;  // [m/s^2]

    constexpr double KP_POS = 0.1, KI_POS = 1.0, KD_POS = 1.3;
    constexpr double KP_ATT_RP = 0.5, KI_ATT_RP = 0.0, KD_ATT_RP = 0.2; 
    constexpr double KP_ATT_YAW = 0.5,  KI_ATT_YAW = 0.0, KD_ATT_YAW = 0.3;

    for (int i=0;i<3;++i) 
    {
      pid_pos_[i].setGains(KP_POS, KI_POS, KD_POS);
      pid_pos_[i].setILimits(-5.0, 5.0);
      pid_pos_[i].setOutLimits(-200.0, 200.0);
    }

    pid_att_[0].setGains(KP_ATT_RP, KI_ATT_RP, KD_ATT_RP);
    pid_att_[1].setGains(KP_ATT_RP, KI_ATT_RP, KD_ATT_RP);
    pid_att_[2].setGains(KP_ATT_YAW, KI_ATT_YAW, KD_ATT_YAW);

    for (int i=0;i<3;++i) 
    {
      pid_att_[i].setILimits(-1.0, 1.0);
      pid_att_[i].setOutLimits(-5.0, 5.0);
    }

    sub_cmd_ = this->create_subscription<palletrone_interfaces::msg::Cmd>("/cmd", 10, std::bind(&WrenchController::onCmd, this, std::placeholders::_1));
    sub_state_ = this->create_subscription<palletrone_interfaces::msg::PalletroneState>("/palletrone_state", 10, std::bind(&WrenchController::onState, this, std::placeholders::_1));
    
    pub_wrench_ = this->create_publisher<palletrone_interfaces::msg::Wrench>("/wrench", 10);

    pos_cmd_.setZero();
    last_time_ = this->now();
  }

private:
  struct PID 
  {
    double kp{0}, ki{0}, kd{0};
    double iacc{0};
    double i_min{-1e9}, i_max{1e9};
    double out_min{-1e9}, out_max{1e9};

    void setGains(double p, double i, double d){ kp=p; ki=i; kd=d; }
    void setILimits(double mn, double mx){ i_min=mn; i_max=mx; }
    void setOutLimits(double mn, double mx){ out_min=mn; out_max=mx; }

    inline double step(double e, double de, double dt)
    {
      if (dt <= 0.0) dt = 1e-3;

      iacc += ki * e * dt;
      iacc = std::clamp(iacc, i_min, i_max);
      double u = kp*e + iacc + kd*de;

      return std::clamp(u, out_min, out_max);
    }

    inline void reset(){ iacc = 0.0; }
  };

  static inline double wrapPi(double a)
  {
    while (a >  M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
  }

  static Eigen::Matrix3d Rx(double r)
  {
    const double s=std::sin(r), c=std::cos(r);
    Eigen::Matrix3d R; R << 1,0,0,  0,c,-s,  0,s,c; return R;
  }

  static Eigen::Matrix3d Ry(double p)
  {
    const double s=std::sin(p), c=std::cos(p);
    Eigen::Matrix3d R; R << c,0,s,  0,1,0,  -s,0,c; return R;
  }

  static Eigen::Matrix3d Rz(double y)
  {
    const double s=std::sin(y), c=std::cos(y);
    Eigen::Matrix3d R; R << c,-s,0,  s,c,0,  0,0,1; return R;
  }

  static Eigen::Matrix3d R_BW_from_rpy(double r, double p, double y)
  {
    const Eigen::Matrix3d R_WB = Rz(y) * Ry(p) * Rx(r);
    return R_WB.transpose();
  }

  void onCmd(const palletrone_interfaces::msg::Cmd::SharedPtr msg) 
  {
    pos_cmd_ << static_cast<double>(msg->pos_cmd[0]), static_cast<double>(msg->pos_cmd[1]), static_cast<double>(msg->pos_cmd[2]);
    have_cmd_ = true;

    tryPublish();
  }

  void onState(const palletrone_interfaces::msg::PalletroneState::SharedPtr msg) 
  {
    pos_ << static_cast<double>(msg->pos[0]),
            static_cast<double>(msg->pos[1]),
            static_cast<double>(msg->pos[2]);
    
    vel_ << static_cast<double>(msg->vel[0]),
            static_cast<double>(msg->vel[1]),
            static_cast<double>(msg->vel[2]);
    
    rpy_ << static_cast<double>(msg->rpy[0]),
            static_cast<double>(msg->rpy[1]),
            static_cast<double>(msg->rpy[2]);

    w_body_ << static_cast<double>(msg->w_rpy[0]),
               static_cast<double>(msg->w_rpy[1]),
               static_cast<double>(msg->w_rpy[2]);

    have_state_ = true;

    tryPublish();
  }

  void tryPublish() 
  {
    if (!have_state_) return;

    const rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (!(dt > 0.0) || dt > 0.2) dt = 1.0/400.0;

    const Eigen::Vector3d p_ref = have_cmd_ ? pos_cmd_ : Eigen::Vector3d::Zero();
    const Eigen::Vector3d e_p   = p_ref - pos_;
    const Eigen::Vector3d de_p  = -vel_;

    Eigen::Vector3d F_world_pid(0,0,0);

    for (int i=0;i<3;++i) F_world_pid(i) = pid_pos_[i].step(e_p(i), de_p(i), dt);

    Eigen::Vector3d F_world;
    F_world.x() = F_world_pid.x();
    F_world.y() = F_world_pid.y();
    F_world.z() = F_world_pid.z() + mass_ * grav_;

    const double r = rpy_(0), p = rpy_(1), y = rpy_(2);
    const Eigen::Matrix3d R_BW = R_BW_from_rpy(r, p, y);
    const Eigen::Vector3d F_body = R_BW * F_world;

    const Eigen::Vector3d e_att( -r, -p, wrapPi(-y) );
    const Eigen::Vector3d de_att = -w_body_;

    Eigen::Vector3d M_body(0,0,0);
    for (int i=0;i<3;++i) M_body(i) = pid_att_[i].step(e_att(i), de_att(i), dt);

    palletrone_interfaces::msg::Wrench w;
    w.moment[0] = static_cast<float>(M_body(0));
    w.moment[1] = static_cast<float>(M_body(1));
    w.moment[2] = static_cast<float>(M_body(2));
    w.force[0]  = static_cast<float>(F_body(0));
    w.force[1]  = static_cast<float>(F_body(1));
    w.force[2]  = static_cast<float>(F_body(2));
    pub_wrench_->publish(w);
  }


  rclcpp::Subscription<palletrone_interfaces::msg::Cmd>::SharedPtr sub_cmd_;
  rclcpp::Subscription<palletrone_interfaces::msg::PalletroneState>::SharedPtr sub_state_;
  rclcpp::Publisher<palletrone_interfaces::msg::Wrench>::SharedPtr pub_wrench_;
  rclcpp::Time last_time_;

  Eigen::Vector3d pos_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d w_body_{Eigen::Vector3d::Zero()};

  struct PID pid_pos_[3];
  struct PID pid_att_[3];

  double mass_{4.0};
  double grav_{9.81};

  bool have_state_{false};
  bool have_cmd_{false};

};

int main(int argc, char** argv) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchController>());
  rclcpp::shutdown();
  return 0;
}
