#include <rclcpp/rclcpp.hpp>
#include <palletrone_interfaces/msg/wrench.hpp>
#include <palletrone_interfaces/msg/input.hpp>
#include <palletrone_interfaces/msg/palletrone_state.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

class AllocatorController : public rclcpp::Node 
{
public:
  static constexpr double inv_sqrt2   = 1.0 / 1.4142135623730951;
  static constexpr size_t buffer_size = 10;

  static constexpr double lpf_alpha = 0.01;   // LPF for tauz bias
  static constexpr double zeta = 0.02;        // reaction torque coeff [Nm/N]
  static constexpr double r = 0.148492;
  static constexpr double r_z = 0.075;

  AllocatorController() : rclcpp::Node("allocator_controller")
  {
    sub_wrench_ = this->create_subscription<palletrone_interfaces::msg::Wrench>("/wrench", 10, std::bind(&AllocatorController::onWrench, this, std::placeholders::_1));
    sub_state_ = this->create_subscription<palletrone_interfaces::msg::PalletroneState>("/palletrone_state", 10, std::bind(&AllocatorController::onState, this, std::placeholders::_1));

    pub_input_ = this->create_publisher<palletrone_interfaces::msg::Input>("/input", 10);

    dt_buffer_.assign(buffer_size, 0.01);
    dt_sum_ = 0.01 * static_cast<double>(buffer_size);

    Pc_.setZero();
  }

private:
  void onState(const palletrone_interfaces::msg::PalletroneState::SharedPtr msg) 
  {
    C2_mea_(0) = static_cast<double>(msg->servo[0]);
    C2_mea_(1) = static_cast<double>(msg->servo[1]);
    C2_mea_(2) = static_cast<double>(msg->servo[2]);
    C2_mea_(3) = static_cast<double>(msg->servo[3]);
  }

  void onWrench(const palletrone_interfaces::msg::Wrench::SharedPtr msg)
  {
    rclcpp::Time current_callback_time = this->now();
    double dt = (last_callback_time_.nanoseconds() > 0) ? (current_callback_time - last_callback_time_).seconds() : 0.01;
    last_callback_time_ = current_callback_time;
    if (dt <= 0.0 || dt > 0.2) dt = 0.01;

    dt_sum_ = dt_sum_ - dt_buffer_[buffer_index_] + dt;
    dt_buffer_[buffer_index_] = dt;
    buffer_index_ = (buffer_index_ + 1) % buffer_size;
    double avg_dt = dt_sum_ / static_cast<double>(buffer_size);
    filtered_frequency_ = 1.0 / std::max(avg_dt, 1e-4);

    Eigen::Matrix<double,6,1> Wrench;
    Wrench << static_cast<double>(msg->moment[0]), static_cast<double>(msg->moment[1]), static_cast<double>(msg->moment[2]),
               static_cast<double>(msg->force[0]),  static_cast<double>(msg->force[1]),  static_cast<double>(msg->force[2]);

    tauz_bar_ = lpf_alpha * Wrench(2) + (1.0 - lpf_alpha) * tauz_bar_;
    double tauz_r     = Wrench(2) - tauz_bar_;
    double tauz_r_sat = std::clamp(tauz_r, -2.0, 2.0); // yaw reaction torque limit [Nm]
    double tauz_t     = tauz_bar_ + (tauz_r - tauz_r_sat);

    Eigen::Vector4d B1(Wrench(0), Wrench(1), tauz_r_sat, Wrench(5));
    Eigen::Matrix4d A1 = calc_A1(C2_mea_);
    {
      Eigen::FullPivLU<Eigen::Matrix4d> lu_1(A1);
      if (lu_1.isInvertible()) C1_ = lu_1.solve(B1);
      else C1_ = (A1.transpose()*A1 + 1e-8*Eigen::Matrix4d::Identity()).ldlt().solve(A1.transpose()*B1);
    }

    Eigen::Vector4d B2(Wrench(3), Wrench(4), tauz_t, 0.0);
    Eigen::Matrix4d A2 = calc_A2(C1_, C2_mea_);
    {
      Eigen::FullPivLU<Eigen::Matrix4d> lu_2(A2);
      if (lu_2.isInvertible()) C2_des_ = lu_2.solve(B2);
      else C2_des_ = (A2.transpose()*A2 + 1e-8*Eigen::Matrix4d::Identity()).ldlt().solve(A2.transpose()*B2);
    }

    palletrone_interfaces::msg::Input out;
    
    double motor_speed[4];
    motor_speed[0] = std::sqrt(std::max(0.0, C1_(0)/zeta));
    motor_speed[1] = std::sqrt(std::max(0.0, C1_(1)/zeta));
    motor_speed[2] = std::sqrt(std::max(0.0, C1_(2)/zeta));
    motor_speed[3] = std::sqrt(std::max(0.0, C1_(3)/zeta));

    out.u[0] = motor_speed[0]; out.u[1] = motor_speed[1]; out.u[2] = motor_speed[2]; out.u[3] = motor_speed[3];
    out.u[4] = C2_des_(0); out.u[5] = C2_des_(1); out.u[6] = C2_des_(2); out.u[7] = C2_des_(3);
    pub_input_->publish(out);
  }

  Eigen::Matrix4d calc_A1(const Eigen::Vector4d& C2) 
  {
    Eigen::Matrix4d A1;

    double s1 = std::sin(C2(0)); double s2 = std::sin(C2(1));
    double s3 = std::sin(C2(2)); double s4 = std::sin(C2(3));
    double c1 = std::cos(C2(0)); double c2 = std::cos(C2(1));
    double c3 = std::cos(C2(2)); double c4 = std::cos(C2(3));

    double pcx = Pc_(0), pcy = Pc_(1), pcz = Pc_(2);

    A1(0,0) = inv_sqrt2 * ( zeta +  r_z - pcz) * s1 + ( +r - pcy) * c1;
    A1(0,1) = inv_sqrt2 * (-zeta -  r_z + pcz) * s2 + ( +r - pcy) * c2;
    A1(0,2) = inv_sqrt2 * (-zeta -  r_z + pcz) * s3 + ( -r - pcy) * c3;
    A1(0,3) = inv_sqrt2 * ( zeta +  r_z - pcz) * s4 + ( -r - pcy) * c4;

    A1(1,0) = inv_sqrt2 * (-zeta +  r_z - pcz) * s1 + ( -r + pcx) * c1;
    A1(1,1) = inv_sqrt2 * (-zeta +  r_z - pcz) * s2 + ( +r + pcx) * c2;
    A1(1,2) = inv_sqrt2 * ( zeta -  r_z + pcz) * s3 + ( +r + pcx) * c3;
    A1(1,3) = inv_sqrt2 * ( zeta -  r_z + pcz) * s4 + ( -r + pcx) * c4;

    A1(2,0) =  zeta * c1;
    A1(2,1) = -zeta * c2;
    A1(2,2) =  zeta * c3;
    A1(2,3) = -zeta * c4;

    A1(3,0) = c1;
    A1(3,1) = c2;
    A1(3,2) = c3;
    A1(3,3) = c4;

    return A1;
  }

  Eigen::Matrix4d calc_A2(const Eigen::Vector4d& C1, const Eigen::Vector4d& C2) 
  {
    Eigen::Matrix4d A2;

    double pcx = Pc_(0), pcy = Pc_(1);
    double s1 = std::sin(C2(0)); double s2 = std::sin(C2(1));
    double s3 = std::sin(C2(2)); double s4 = std::sin(C2(3));

    double f1 = C1(0), f2 = C1(1), f3 = C1(2), f4 = C1(3);

    A2(0,0) =  inv_sqrt2 * f1;
    A2(0,1) =  inv_sqrt2 * f2;
    A2(0,2) = -inv_sqrt2 * f3;
    A2(0,3) = -inv_sqrt2 * f4;

    A2(1,0) = -inv_sqrt2 * f1;
    A2(1,1) =  inv_sqrt2 * f2;
    A2(1,2) =  inv_sqrt2 * f3;
    A2(1,3) = -inv_sqrt2 * f4;

    A2(2,0) = inv_sqrt2 * ( +pcx + pcy) * s1 + inv_sqrt2 * ( -(+r) - (+r) ) * f1;
    A2(2,1) = inv_sqrt2 * ( -pcx + pcy) * s2 + inv_sqrt2 * (  (-r) - (+r) ) * f2;
    A2(2,2) = inv_sqrt2 * ( -pcx - pcy) * s3 + inv_sqrt2 * (  (-r) + (-r) ) * f3;
    A2(2,3) = inv_sqrt2 * ( +pcx - pcy) * s4 + inv_sqrt2 * ( -(+r) + (-r) ) * f4;

    A2(3,0) = +inv_sqrt2 * ( -(+r) - (+r) ) * f1;
    A2(3,1) = -inv_sqrt2 * ( +(-r) - (+r) ) * f2;
    A2(3,2) = +inv_sqrt2 * ( +(-r) + (-r) ) * f3;
    A2(3,3) = -inv_sqrt2 * ( -(+r) + (-r) ) * f4;

    return A2;
  }

  rclcpp::Subscription<palletrone_interfaces::msg::Wrench>::SharedPtr          sub_wrench_;
  rclcpp::Subscription<palletrone_interfaces::msg::PalletroneState>::SharedPtr sub_state_;
  rclcpp::Publisher<palletrone_interfaces::msg::Input>::SharedPtr              pub_input_;

  rclcpp::Time last_callback_time_;
  std::vector<double> dt_buffer_;
  size_t buffer_index_{0};
  double dt_sum_{0.0};
  double filtered_frequency_{0.0};

  double tauz_bar_{0.0};
  Eigen::Vector4d C1_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d C2_mea_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d C2_des_{Eigen::Vector4d::Zero()};
  Eigen::Vector3d Pc_{Eigen::Vector3d::Zero()};
};

int main(int argc, char** argv) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AllocatorController>());
  rclcpp::shutdown();
  return 0;
}
