#pragma once

#include <motion3d/transforms/axis_angle_.hpp>
#include <motion3d/transforms/dual_quaternion_.hpp>
#include <motion3d/transforms/euler_.hpp>
#include <motion3d/transforms/matrix_.hpp>
#include <motion3d/transforms/quaternion_.hpp>

namespace motion3d
{

// AxisAngleTransform
AxisAngleTransform& AxisAngleTransform::inverse_()
{
  auto inv = this->inverse();
  this->translation_ = inv.translation_;
  this->angle_axis_ = inv.angle_axis_;
  return *this;
}

AxisAngleTransform AxisAngleTransform::inverse() const
{
  return this->asType<QuaternionTransform>().inverse_().asType<AxisAngleTransform>();
}

// DualQuaternionTransform
DualQuaternionTransform& DualQuaternionTransform::inverse_()
{
  dq_.quatConjugate_();
  return *this;
}

DualQuaternionTransform DualQuaternionTransform::inverse() const
{
  return DualQuaternionTransform(dq_.quatConjugate(), unsafe_);
}

// EulerTransform
EulerTransform& EulerTransform::inverse_()
{
  auto inv = this->inverse();
  this->translation_ = inv.translation_;
  this->ai_ = inv.ai_;
  this->aj_ = inv.aj_;
  this->ak_ = inv.ak_;
  this->axes_ = inv.axes_;
  return *this;
}

EulerTransform EulerTransform::inverse() const
{
  return {this->asType<MatrixTransform>().inverse_(), this->axes_};
}

// MatrixTransform
MatrixTransform& MatrixTransform::inverse_()
{
  matrix_.block<3, 3>(0, 0).transposeInPlace();
  matrix_.block<3, 1>(0, 3) = - matrix_.block<3, 3>(0, 0) * matrix_.block<3, 1>(0, 3);
  return *this;
}

MatrixTransform MatrixTransform::inverse() const
{
  Eigen::Matrix<double, 4, 4> new_matrix = Eigen::Matrix<double, 4, 4>::Identity();
  new_matrix.block<3, 3>(0, 0) = matrix_.block<3, 3>(0, 0).transpose();
  new_matrix.block<3, 1>(0, 3) = - new_matrix.block<3, 3>(0, 0) * matrix_.block<3, 1>(0, 3);
  return MatrixTransform(new_matrix, unsafe_);
}

// QuaternionTransform
QuaternionTransform& QuaternionTransform::inverse_()
{
  Quaternion<double> q_copy(this->quaternion_);
  this->quaternion_.conjugate_();

  Quaternion<double> tq;
  tq.w() = 0.0;
  tq.vec() = translation_;
  this->translation_ = -(this->quaternion_ * tq * q_copy).vec();

  return *this;
}

QuaternionTransform QuaternionTransform::inverse() const
{
  Quaternion<double> qi = quaternion_.conjugate();

  Quaternion<double> tq;
  tq.w() = 0.0;
  tq.vec() = translation_;
  Quaternion<double> tqi = qi * tq * quaternion_;

  return {-tqi.vec(), qi, unsafe_};
}

} // namespace motion3d
