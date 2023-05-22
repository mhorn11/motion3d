#pragma once

#include <motion3d/transforms/axis_angle_.hpp>
#include <motion3d/transforms/dual_quaternion_.hpp>
#include <motion3d/transforms/euler_.hpp>
#include <motion3d/transforms/matrix_.hpp>
#include <motion3d/transforms/quaternion_.hpp>

namespace motion3d
{

EulerTransform::EulerTransform(
  const AxisAngleTransform &obj,
  const EulerAxes axes)
  : EulerTransform(obj.asType<MatrixTransform>(), axes)
{
}

EulerTransform::EulerTransform(
  const DualQuaternionTransform &obj,
  const EulerAxes axes)
  : EulerTransform(obj.asType<MatrixTransform>(), axes)
{
}

EulerTransform::EulerTransform(
  const EulerTransform &obj,
  const EulerAxes axes)
  : TransformBase(obj.unsafe_)
  , translation_(obj.translation_)
  , ai_(0.0)
  , aj_(0.0)
  , ak_(0.0)
  , axes_(axes)
{
  Eigen::Matrix<double, 3, 3> matrix = euler::eulerToMatrix(obj.ai_, obj.aj_, obj.ak_, obj.axes_);
  std::tie(ai_, aj_, ak_) = euler::matrixToEuler(matrix, axes);
}

EulerTransform::EulerTransform(
  const MatrixTransform &obj,
  const EulerAxes axes)
  : TransformBase(obj.isUnsafe())
  , translation_(obj.getTranslation())
  , ai_(0.0)
  , aj_(0.0)
  , ak_(0.0)
  , axes_(axes)
{
  std::tie(ai_, aj_, ak_) = euler::matrixToEuler(obj.getRotationMatrix(), axes);
}

EulerTransform::EulerTransform(
  const QuaternionTransform &obj,
  const EulerAxes axes)
  : EulerTransform(obj.asType<MatrixTransform>(), axes)
{
}

} // namespace motion3d
