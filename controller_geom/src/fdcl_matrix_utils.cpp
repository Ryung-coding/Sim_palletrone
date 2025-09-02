#include "fdcl/matrix_utils.hpp"

Matrix3 hat(const Vector3 v){
  Matrix3 V;
  V.setZero();

  V(2,1) = v(0);
  V(1,2) = -V(2, 1);
  V(0,2) = v(1);
  V(2,0) = -V(0, 2);
  V(1,0) = v(2);
  V(0,1) = -V(1, 0);

  return V;
}

Vector3 vee(const Matrix3 V){
  Vector3 v;
  Matrix3 E;

  v.setZero();
  E = V + V.transpose();
    
  if(E.norm() > 1.e-6){std::cout << "VEE: E.norm() = " << E.norm() << std::endl;}

  v(0) = V(2, 1);
  v(1) = V(0, 2);
  v(2) = V(1, 0);

  return  v;
}

void saturate(Vector3 &x, const double x_min, const double x_max){
  for (int i = 0; i < 3; i++){
    if (x(i) > x_max){x(i) = x_max;}
    else if (x(i) < x_min){x(i) = x_min;}
  }
}

void deriv_unit_vector( \
    const Vector3 &A, const Vector3 &A_dot, const Vector3 &A_ddot, \
    Vector3 &q, Vector3 &q_dot, Vector3 &q_ddot){

  double nA = A.norm();
  double nA3 = pow(nA, 3);
  double nA5 = pow(nA, 5);

  q = A / nA;
  q_dot = A_dot / nA - A * A.dot(A_dot) / nA3;

  q_ddot = A_ddot / nA \
           - A_dot / nA3 * (2 * A.dot(A_dot)) \
           - A / nA3 * (A_dot.dot(A_dot) + A.dot(A_ddot)) \
           + 3 * A / nA5 * pow(A.dot(A_dot), 2);
}