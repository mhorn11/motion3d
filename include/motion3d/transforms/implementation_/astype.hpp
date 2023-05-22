#pragma once

#include <motion3d/transforms/axis_angle_.hpp>
#include <motion3d/transforms/dual_quaternion_.hpp>
#include <motion3d/transforms/euler_.hpp>
#include <motion3d/transforms/matrix_.hpp>
#include <motion3d/transforms/quaternion_.hpp>
#include <motion3d/utils/templating.hpp>

namespace motion3d
{

// TransformBase
template <class Derived>
TransformInterface::Ptr TransformBase<Derived>::asType(const TransformType &type) const
{
  switch (type)
  {
    case internal::traits<AxisAngleTransform>::kTType:
      return std::make_shared<AxisAngleTransform>(derived().template asType<AxisAngleTransform>());
    case internal::traits<DualQuaternionTransform>::kTType:
      return std::make_shared<DualQuaternionTransform>(derived().template asType<DualQuaternionTransform>());
    case internal::traits<EulerTransform>::kTType:
      return std::make_shared<EulerTransform>(derived().template asType<EulerTransform>());
    case internal::traits<MatrixTransform>::kTType:
      return std::make_shared<MatrixTransform>(derived().template asType<MatrixTransform>());
    case internal::traits<QuaternionTransform>::kTType:
      return std::make_shared<QuaternionTransform>(derived().template asType<QuaternionTransform>());
  }
  throw std::runtime_error("Transform type not supported");
}

// AxisAngleTransform
template<class T>
T AxisAngleTransform::asType() const
{
  if constexpr (std::is_same<AxisAngleTransform, T>::value)
  {
    return AxisAngleTransform(*this);
  }

  else if constexpr (std::is_same<DualQuaternionTransform, T>::value)
  {
    return this->asType<QuaternionTransform>().asType<DualQuaternionTransform>();
  }

  else if constexpr (std::is_same<EulerTransform, T>::value)
  {
    return this->asType<MatrixTransform>().asType<EulerTransform>();
  }

  else if constexpr (std::is_same<MatrixTransform, T>::value)
  {
    Eigen::Matrix<double, 4, 4> matrix;
    matrix.setIdentity();
    matrix.block<3, 1>(0, 3) = this->translation_;
    matrix.block<3, 3>(0, 0) = this->angle_axis_.toRotationMatrix();
    return MatrixTransform(matrix, this->unsafe_);
  }

  else if constexpr (std::is_same<QuaternionTransform, T>::value)
  {
    return QuaternionTransform(this->translation_, Quaterniond(this->angle_axis_), this->unsafe_);
  }

  else
  {
    static_assert(always_false<T>, "Transform type not supported");
  }
}

// DualQuaternionTransform
template<class T>
T DualQuaternionTransform::asType() const
{
  if constexpr (std::is_same<AxisAngleTransform, T>::value)
  {
    return this->asType<QuaternionTransform>().asType<AxisAngleTransform>();
  }

  else if constexpr (std::is_same<DualQuaternionTransform, T>::value)
  {
    return DualQuaternionTransform(*this);
  }

  else if constexpr (std::is_same<EulerTransform, T>::value)
  {
    return this->asType<QuaternionTransform>().asType<EulerTransform>();
  }

  else if constexpr (std::is_same<MatrixTransform, T>::value)
  {
    return this->asType<QuaternionTransform>().asType<MatrixTransform>();
  }

  else if constexpr (std::is_same<QuaternionTransform, T>::value)
  {
    return QuaternionTransform(this->dq_.getTranslationQuaternion().vec(), this->dq_.real(), this->unsafe_);
  }

  else
  {
    static_assert(always_false<T>, "Transform type not supported");
  }
}

// EulerTransform
template<class T>
T EulerTransform::asType() const
{
  if constexpr (std::is_same<AxisAngleTransform, T>::value)
  {
    return this->asType<QuaternionTransform>().asType<AxisAngleTransform>();
  }

  else if constexpr (std::is_same<DualQuaternionTransform, T>::value)
  {
    return this->asType<QuaternionTransform>().asType<DualQuaternionTransform>();
  }

  else if constexpr (std::is_same<EulerTransform, T>::value)
  {
    return EulerTransform(*this);
  }

  else if constexpr (std::is_same<MatrixTransform, T>::value)
  {
    Eigen::Matrix<double, 4, 4> matrix;
    matrix.setIdentity();
    matrix.block<3, 1>(0, 3) = this->translation_;
    matrix.block<3, 3>(0, 0) = euler::eulerToMatrix(this->ai_, this->aj_, this->ak_, this->axes_);
    return MatrixTransform(matrix, this->unsafe_);
  }

  else if constexpr (std::is_same<QuaternionTransform, T>::value)
  {
    Quaternion<double> quaternion = euler::eulerToQuaternion(this->ai_, this->aj_, this->ak_, this->axes_);
    return QuaternionTransform(this->translation_, quaternion, this->unsafe_);
  }

  else
  {
    static_assert(always_false<T>, "Transform type not supported");
  }
}

// MatrixTransform
template<class T>
T MatrixTransform::asType() const
{
  if constexpr (std::is_same<AxisAngleTransform, T>::value)
  {
    return AxisAngleTransform(this->matrix_.block<3, 1>(0, 3), 
                              Eigen::AngleAxisd(this->matrix_.block<3, 3>(0, 0)), 
                              this->unsafe_);
  }

  else if constexpr (std::is_same<DualQuaternionTransform, T>::value)
  {
    return this->asType<QuaternionTransform>().asType<DualQuaternionTransform>();
  }

  else if constexpr (std::is_same<EulerTransform, T>::value)
  {
    double ai = 0;
    double aj = 0;
    double ak = 0;
    Eigen::Matrix<double, 3, 3> rotation = this->matrix_.block<3, 3>(0, 0);
    std::tie(ai, aj, ak) = euler::matrixToEuler(rotation, kEulerAxesDefault);
    return EulerTransform(this->matrix_.block<3, 1>(0, 3), ai, aj, ak, kEulerAxesDefault, this->unsafe_);
  }

  else if constexpr (std::is_same<MatrixTransform, T>::value)
  {
    return MatrixTransform(*this);
  }

  else if constexpr (std::is_same<QuaternionTransform, T>::value)
  {
    return QuaternionTransform(this->matrix_.block<3, 1>(0, 3), 
                               Quaterniond::FromRotationMatrix(this->matrix_.block<3, 3>(0, 0)),
                               this->unsafe_);
  }

  else
  {
    static_assert(always_false<T>, "Transform type not supported");
  }
}


// QuaternionTransform
template<class T>
T QuaternionTransform::asType() const
{
  if constexpr (std::is_same<AxisAngleTransform, T>::value)
  {
    return AxisAngleTransform(this->translation_, this->quaternion_.toAxisAngle(), this->unsafe_);
  }

  else if constexpr (std::is_same<DualQuaternionTransform, T>::value)
  {
    DualQuaternion<double> dq(this->translation_, this->quaternion_);
    return DualQuaternionTransform(dq, this->unsafe_);
  }

  else if constexpr (std::is_same<EulerTransform, T>::value)
  {
    return this->asType<MatrixTransform>().asType<EulerTransform>();
  }

  else if constexpr (std::is_same<MatrixTransform, T>::value)
  {
    Eigen::Matrix<double, 4, 4> matrix;
    matrix.setIdentity();
    matrix.block<3, 1>(0, 3) = this->translation_;
    matrix.block<3, 3>(0, 0) = this->quaternion_.toRotationMatrix();
    return MatrixTransform(matrix, this->unsafe_);
  }

  else if constexpr (std::is_same<QuaternionTransform, T>::value)
  {
    return QuaternionTransform(*this);
  }

  else
  {
    static_assert(always_false<T>, "Transform type not supported");
  }
}

} // namespace motion3d
