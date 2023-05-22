#pragma once

#include <motion3d/common/math.hpp>
#include <motion3d/transforms/axis_angle_.hpp>
#include <motion3d/transforms/dual_quaternion_.hpp>
#include <motion3d/transforms/euler_.hpp>
#include <motion3d/transforms/matrix_.hpp>
#include <motion3d/transforms/quaternion_.hpp>

namespace motion3d
{

// AxisAngleTransform
double AxisAngleTransform::rotationNorm() const
{
  return std::abs(normalizeAngle(angle_axis_.angle()));
}

double AxisAngleTransform::translationNorm() const
{
  return translation_.norm();
}

// DualQuaternionTransform
double DualQuaternionTransform::rotationNorm() const
{
  return dq_.rotationNorm();
}

double DualQuaternionTransform::translationNorm() const
{
  return dq_.translationNorm();
}

// EulerTransform
double EulerTransform::rotationNorm() const
{
  return this->asType<QuaternionTransform>().rotationNorm();
}

double EulerTransform::translationNorm() const
{
  return translation_.norm();
}

// MatrixTransform
double MatrixTransform::rotationNorm() const
{
  return this->asType<AxisAngleTransform>().rotationNorm();
}

double MatrixTransform::translationNorm() const
{
  return matrix_.block<3, 1>(0, 3).norm();
}

// QuaternionTransform
double QuaternionTransform::rotationNorm() const
{
  return quaternion_.rotationNorm();
}

double QuaternionTransform::translationNorm() const
{
  return translation_.norm();
}

} // namespace motion3d