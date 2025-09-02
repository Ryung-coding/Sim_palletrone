#include "controller_geom.hpp"

#include <iostream>
#include <vector>

ControllerGeom::ControllerGeom()
 : state_(new fdcl::state_t()),
  command_(new fdcl::command_t()),
  fdcl_controller_(state_, command_),
  zeta_(Eigen::Vector4d( zeta, -zeta, zeta, -zeta )),
  q_B0_(Eigen::Vector4d( 0.25*M_PI, 0.75*M_PI, -0.75*M_PI, -0.25*M_PI )) {

  DH_params_ <<
    //   a      alpha     d   theta0
       0.120,   0.0,     0.0,  0.0,   // B->0
       0.134,   M_PI/2,  0.0,  0.0,   // 0->1
       0.115,   0.0,     0.0,  0.0,   // 1->2
       0.110,   0.0,     0.0,  0.0,   // 2->3
       0.024,   M_PI/2,  0.0,  0.0,   // 3->4
       0.104,   0.0,     0.0,  0.0;   // 4->5

  command_->xd << 0.0, 0.0, 0.0;
  command_->xd_dot.setZero();
  command_->xd_2dot.setZero();
  command_->xd_3dot.setZero();
  command_->xd_4dot.setZero();

  command_->b1d << 1.0, 0.0, 0.0;
  command_->b1d_dot.setZero();
  command_->b1d_ddot.setZero();
}

void ControllerGeom::state_update(const ControlInput& in) {
  // 1) Desired pose
  command_->xd << in.pos_des[0], -in.pos_des[1], -in.pos_des[2];
  command_->xd_dot.setZero();
  command_->xd_2dot.setZero();
  command_->xd_3dot.setZero();
  command_->xd_4dot.setZero();

  const double yaw = in.heading_yaw;
  command_->b1d << std::cos(yaw), -std::sin(yaw), 0.0;
  command_->b1d_dot.setZero();
  command_->b1d_ddot.setZero();

  // 2) Sensors
  state_->x << in.pos[0], -in.pos[1], -in.pos[2];
  state_->v << in.vel[0], -in.vel[1], -in.vel[2];
  state_->a << in.acc[0], -in.acc[1], -in.acc[2];
  
  const double w = in.quat_wxyz[0];
  const double x = in.quat_wxyz[1];
  const double y = in.quat_wxyz[2];
  const double z = in.quat_wxyz[3];
  Eigen::Matrix<double,3,3> R = quat_to_R(w, x, y, z);
  state_->R = R;
  state_->W << in.gyro[0], -in.gyro[1], -in.gyro[2];

  // 4) Inertia
  state_->J << in.inertia[0], in.inertia[1], in.inertia[2],
               in.inertia[3], in.inertia[4], in.inertia[5],
               in.inertia[6], in.inertia[7], in.inertia[8];

  for (int i = 0; i < 4; ++i){for (int j=0; j < 5; ++j) arm_pos_[i][j] = in.arm_pos[i][j];}
}

void ControllerGeom::compute_controller(double wrench[4], double pwm_out[4]) {
  // 1) Flight controller (Geometry control in SE(3))
  double Fz_out = 0.0;
  Eigen::Vector3d M_out = Eigen::Vector3d::Zero();

  fdcl_controller_.position_control();
  fdcl_controller_.output_fM(Fz_out, M_out);

  Eigen::Vector4d Wrench;
  Wrench << M_out(0), -M_out(1), -M_out(2), Fz_out;

  // 2) Control Allocation
  Eigen::Matrix<double,4,12> A1; A1.setZero();
  Eigen::Matrix<double,12,4> A2; A2.setZero();

  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  std::array<Eigen::Matrix4d,4> T_a;

  // FK for each arm
  for (int arm = 0; arm < 4; ++arm) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i <= 5; ++i) {
      const double a     = DH_params_(i,0);
      const double alpha = DH_params_(i,1);
      const double d     = DH_params_(i,2);
      const double th0   = DH_params_(i,3);
      const double q     = (i == 0) ? q_B0_(arm) : arm_pos_[arm][i-1];
      T *= compute_DH(a, alpha, d, th0 + q);
    }
    T_a[arm] = T;

    const Eigen::Vector3d r = T.block<3,1>(0,3);
    const Eigen::Matrix3d S = skew(r);
    const Eigen::Matrix3d M = S + zeta_(arm) * I3;

    A1.block<3,3>(0, 3*arm) = M;
    A1(3, 3*arm + 2) = 1.0;  // z-component to sum Fz
    A2.block<3,1>(3*arm, arm) = T.block<3,1>(0,0);
  }

  // get thrust
  const Eigen::Matrix4d A = A1 * A2;
  Eigen::Vector4d F;
  Eigen::FullPivLU<Eigen::Matrix4d> lu(A);
  if (lu.isInvertible()) {F = lu.solve(Wrench);}
  else {
    const double lambda2 = 1e-8;
    F = (A.transpose()*A + lambda2*Eigen::Matrix4d::Identity()).ldlt().solve(A.transpose()*Wrench);
  }

  // thrust -> pwm
  Eigen::Vector4d pwm;
  for (int i = 0; i < 4; ++i) {
    const double val = std::max(0.0, (F(i) - pwm_beta_) / pwm_alpha_);
    pwm(i) = std::sqrt(val);
    pwm(i) = std::clamp(pwm(i), 0.0, 1.0);
  }

  // 5) copy out
  pwm_out[0] = pwm(0);
  pwm_out[1] = pwm(1);
  pwm_out[2] = pwm(2);
  pwm_out[3] = pwm(3);
  wrench[0] = Wrench(0);
  wrench[1] = Wrench(1);
  wrench[2] = Wrench(2);
  wrench[3] = Wrench(3);
}

ControllerGeom::~ControllerGeom() {
  delete state_;
  delete command_;
}


// -------------------- C API (ctypes) --------------------

extern "C" {

controller_handle_t controller_create() {
  return reinterpret_cast<controller_handle_t>(new ControllerGeom());
}

void controller_destroy(controller_handle_t h) {
  auto* p = reinterpret_cast<ControllerGeom*>(h);
  delete p;
}

void controller_state_update(controller_handle_t h,
                           const double pos_des3[3],
                           double heading_yaw,
                           const double pos3[3],
                           const double vel3[3],
                           const double acc3[3],
                           const double quat_wxyz4[4],
                           const double gyro3[3],
                           const double inertia9[9],
                           const double arm_pos20[20]) {
  auto* p = reinterpret_cast<ControllerGeom*>(h);
  ControlInput in{};

  in.pos_des[0] = pos_des3[0];
  in.pos_des[1] = pos_des3[1];
  in.pos_des[2] = pos_des3[2];
  in.heading_yaw = heading_yaw;

  in.pos[0] = pos3[0]; in.pos[1] = pos3[1]; in.pos[2] = pos3[2];
  in.vel[0] = vel3[0]; in.vel[1] = vel3[1]; in.vel[2] = vel3[2];
  in.acc[0] = acc3[0]; in.acc[1] = acc3[1]; in.acc[2] = acc3[2];

  in.quat_wxyz[0] = quat_wxyz4[0];
  in.quat_wxyz[1] = quat_wxyz4[1];
  in.quat_wxyz[2] = quat_wxyz4[2];
  in.quat_wxyz[3] = quat_wxyz4[3];

  in.gyro[0] = gyro3[0]; in.gyro[1] = gyro3[1]; in.gyro[2] = gyro3[2];

  for (int i = 0; i < 9; ++i) in.inertia[i] = inertia9[i];
  for (int a=0; a<4; ++a) for (int j=0; j<5; ++j) {in.arm_pos[a][j] = arm_pos20[a*5 + j];}

  p->state_update(in);
}

void controller_compute_pwm(controller_handle_t h,
                           double wrench[4],
                           double pwm_out[4]) {
  auto* p = reinterpret_cast<ControllerGeom*>(h);
  p->compute_controller(wrench, pwm_out);
}

} // extern "C"