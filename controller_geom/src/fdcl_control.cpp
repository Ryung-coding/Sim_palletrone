#include "fdcl/control.hpp"

fdcl::control::control(
  fdcl::state_t *&state_,
  fdcl::command_t *&command_) : 
  state(state_), command(command_)
{
  // init uninitialized parameters
  e1 << 1.0, 0.0, 0.0;
  e2 << 0.0, 1.0, 0.0;
  e3 << 0.0, 0.0, 1.0;

  // load parameters from the config file
  fdcl::control::load_config();
};

fdcl::control::control(void){
  // This should not happen as this leads to uninitialized pointers.
  std::cout << "CTRL: control class is initiated without required parameters."
            << "\n\tThis might lead to undefined behavoirs."
            << "\n\tCalling the destructor .."
            << std::endl;
    this->~control();
}

fdcl::control::~control(void){};

void fdcl::control::position_control(void){
  // translational error functions
  eX = state->x - command->xd;     // position error - eq (11)
  eV = state->v - command->xd_dot; // velocity error - eq (12)

  // if norm(eX) exceeds limit, scale it back
  double eX_norm = eX.norm();
  if(eX_norm > eX_norm_max_) {eX = eX * (eX_norm_max_ / eX_norm);}

  // position integral terms
  eIX.integrate(c1 * eX + eV, dt); // eq (13)
  double sat_sigma = 20.0/kIX;
  saturate(eIX.error, -sat_sigma, sat_sigma);

  // force 'f' along negative b3-axis - eq (14)
  // this term equals to R.e3
  Vector3 A = -kX*eX - kV*eV - kIX*eIX.error - m*g*e3 + m*command->xd_2dot;

  Vector3 b3 = state->R * e3;
  Vector3 b3_dot = state->R * hat(state->W) * e3; // eq (22)
  f_total = -A.dot(b3);

  // intermediate terms for rotational errors
  Vector3 ea = g * e3 - f_total / m * b3 - command->xd_2dot;
  Vector3 A_dot = -kX * eV - kV * ea + m * command->xd_3dot;

  double fdot = -A_dot.dot(b3) - A.dot(b3_dot);
  Vector3 eb = -fdot / m * b3 - f_total / m * b3_dot - command->xd_3dot;
  Vector3 A_ddot = -kX * ea - kV * eb + m * command->xd_4dot;

  Vector3 b3c, b3c_dot, b3c_ddot;
  deriv_unit_vector(-A, -A_dot, -A_ddot, b3c, b3c_dot, b3c_ddot);

  Vector3 A2 = -hat(command->b1d) * b3c;
  Vector3 A2_dot = -hat(command->b1d_dot) * b3c - hat(command->b1d) * b3c_dot;
  Vector3 A2_ddot = -hat(command->b1d_ddot)*b3c - 2.0*hat(command->b1d_dot)*b3c_dot - hat(command->b1d)*b3c_ddot;

  Vector3 b2c, b2c_dot, b2c_ddot;
  deriv_unit_vector(A2, A2_dot, A2_ddot, b2c, b2c_dot, b2c_ddot);

  Vector3 b1c = hat(b2c) * b3c;
  Vector3 b1c_dot = hat(b2c_dot) * b3c + hat(b2c) * b3c_dot;
  Vector3 b1c_ddot = hat(b2c_ddot)*b3c + 2.0*hat(b2c_dot)*b3c_dot + hat(b2c)*b3c_ddot;

  Matrix3 Rddot, Rdddot;

  command->Rd << b1c, b2c, b3c;
  Rddot << b1c_dot, b2c_dot, b3c_dot;
  Rdddot << b1c_ddot, b2c_ddot, b3c_ddot;

  command->Wd = vee(command->Rd.transpose()*Rddot);
  command->Wd_dot = vee(command->Rd.transpose()*Rdddot - hat(command->Wd)*hat(command->Wd));

  // roll / pitch
  command->b3d = b3c;
  command->b3d_dot = b3c_dot;
  command->b3d_ddot = b3c_ddot;

  // yaw
  command->b1c = b1c;
  command->wc3 = e3.dot(state->R.transpose() * command->Rd * command->Wd);
  command->wc3_dot = (e3).dot(state->R.transpose()*command->Rd * command->Wd_dot) - e3.dot(hat(state->W)*state->R.transpose()*command->Rd*command->Wd);

  // This is set through the config file.
  if (use_decoupled_yaw){this->attitude_control_decoupled_yaw();}
  else {this->attitude_control();}
}

void fdcl::control::attitude_control(void){
  Matrix3 RdtR = command->Rd.transpose() * state->R;
  eR = 0.5 * vee(RdtR - RdtR.transpose());
  eW = state->W - state->R.transpose() * command->Rd * command->Wd;

  eIR.integrate(eW + c2 * eR, dt);

  M = - kR * eR \
      - kW * eW \
      - kI * eIR.error \
      + hat(state->R.transpose() * command->Rd * command->Wd) * state->J * \
            state->R.transpose() * command->Rd * command->Wd \
      + state->J * state->R.transpose() * command->Rd * command->Wd_dot;
}

void fdcl::control::attitude_control_decoupled_yaw(void){
  b1 = state->R * e1;
  b2 = state->R * e2;
  b3 = state->R * e3;

  double ky = kR(2, 2);
  double kwy = kW(2, 2);

  // roll/pitch angular velocity vector
  Vector3 W_12 = state->W(0) * b1 + state->W(1) * b2;
  b3_dot = hat(W_12) * b3; // eq (26)

  Vector3 W_12d = hat(command->b3d) * command->b3d_dot;
  Vector3 W_12d_dot = hat(command->b3d) * command->b3d_ddot;

  Vector3 eb = hat(command->b3d) * b3;           // eq (27)
  Vector3 ew = W_12 + hat(b3) * hat(b3) * W_12d; // eq (28)

  // yaw
  double ey = -b2.dot(command->b1c);
  double ewy = state->W(2) - command->wc3;

  // attitude integral terms
  Vector3 eI = ew + c2 * eb;

  eI1.integrate(eI.dot(b1), dt); // b1 axis - eq (29)
  eI2.integrate(eI.dot(b2), dt); // b2 axis - eq (30)
  eIy.integrate(ewy + c3 * ey, dt);

  // control moment for the roll/pitch dynamics - eq (31)
  Vector3 tau;
  tau = -kR(0, 0) * eb \
        - kW(0, 0) * ew \
        - state->J(0, 0) * b3.transpose() * W_12d * b3_dot \
        - state->J(0, 0) * hat(b3) * hat(b3) * W_12d_dot;

  tau += -kI * eI1.error * b1 - kI * eI2.error * b2;

  double M1, M2, M3;

  // control moment around b1 axis - roll - eq (24)
  M1 = b1.transpose() * tau + state->J(2, 2) * state->W(2) * state->W(1);

  // control moment around b2 axis - pitch - eq (24)
  M2 = b2.transpose() * tau - state->J(2, 2) * state->W(2) * state->W(0);

  // control moment around b3 axis - yaw - eq (52)
  M3 = -ky * ey - kwy * ewy + state->J(2, 2) * command->wc3_dot;
  M3 += -kyI * eIy.error;

  M << M1, M2, M3;

  // for saving:
  Matrix3 RdtR = command->Rd.transpose() * state->R;
  eR = 0.5 * vee(RdtR - RdtR.transpose());
  eIR.error << eI1.error, eI2.error, eIy.error;
  eW = state->W - state->R.transpose() * command->Rd * command->Wd;
}

void fdcl::control::output_fM(double &f, Vector3 &M){
  f = this->f_total;
  M = this->M;
}

void fdcl::control::output_debug(Vector3 &X){
  X = eIX.error;
  // X = eV;
}

void fdcl::control::integral_reset(){
  eIR.set_zero();
  eI1.set_zero();
  eI2.set_zero();
  eIy.set_zero();
  eIX.set_zero();
}

void fdcl::control::load_config(void){
  // controller_param.h
  auto cp = controller_param::getControlParameters();
  auto ip = controller_param::getIntegralParameters();
  auto up = controller_param::getUAVParameters();

  use_decoupled_yaw = cp.use_decoupled_yaw;
  
  kX.setZero();
  kX(0,0) = cp.kX[0];
  kX(1,1) = cp.kX[1];
  kX(2,2) = cp.kX[2];
  kV.setZero();
  kV(0,0) = cp.kV[0];
  kV(1,1) = cp.kV[1];
  kV(2,2) = cp.kV[2];

  kIX = ip.kIX;
  ki = ip.ki;
  kIR = ip.kIR;
  kI  = ip.kI;
  kyI = ip.kyI;
  c1  = ip.c1; c2  = ip.c2; c3  = ip.c3;
  
  kR.setZero();
  kR(0,0) = cp.kR[0];
  kR(1,1) = cp.kR[1];
  kR(2,2) = cp.kR[2];

  kW.setZero();
  kW(0,0) = cp.kW[0];
  kW(1,1) = cp.kW[1];
  kW(2,2) = cp.kW[2];

  this->m = up.m;
  this->g = up.g;
}