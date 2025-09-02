#ifndef FDCL_MATRIX_UTILS_HPP
#define FDCL_MATRIX_UTILS_HPP

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "common_types.hpp"

Matrix3 hat(const Vector3 v);
Vector3 vee(const Matrix3 V);

void saturate(Vector3 &x, const double x_min, const double x_max);
void deriv_unit_vector(\
    const Vector3 &A, const Vector3 &A_dot, const Vector3 &A_ddot, \
    Vector3 &q, Vector3 &q_dot, Vector3 &q_ddot
);

#endif