#pragma once

#include <motion3d/transforms/base.hpp>
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
TransformInterface::Ptr TransformBase<Derived>::applyPre_(const TransformInterface &other)
{
  switch (other.getType())
  {
    case internal::traits<AxisAngleTransform>::kTType:
      derived().template applyPre_(dynamic_cast<const AxisAngleTransform&>(other)); break;
    case internal::traits<DualQuaternionTransform>::kTType:
      derived().template applyPre_(dynamic_cast<const DualQuaternionTransform&>(other)); break;
    case internal::traits<EulerTransform>::kTType:
      derived().template applyPre_(dynamic_cast<const EulerTransform&>(other)); break;
    case internal::traits<MatrixTransform>::kTType:
      derived().template applyPre_(dynamic_cast<const MatrixTransform&>(other)); break;
    case internal::traits<QuaternionTransform>::kTType:
      derived().template applyPre_(dynamic_cast<const QuaternionTransform&>(other)); break;
    default:
      throw std::runtime_error("Transformation type not implemented");
  }
  return this->shared_from_this();
}

template <class Derived>
TransformInterface::Ptr TransformBase<Derived>::applyPre(const TransformInterface &other) const
{
  switch (other.getType())
  {
    case internal::traits<AxisAngleTransform>::kTType:
      return std::make_shared<Derived>(derived().template applyPre(dynamic_cast<const AxisAngleTransform&>(other)));
    case internal::traits<DualQuaternionTransform>::kTType:
      return std::make_shared<Derived>(derived().template applyPre(dynamic_cast<const DualQuaternionTransform&>(other)));
    case internal::traits<EulerTransform>::kTType:
      return std::make_shared<Derived>(derived().template applyPre(dynamic_cast<const EulerTransform&>(other)));
    case internal::traits<MatrixTransform>::kTType:
      return std::make_shared<Derived>(derived().template applyPre(dynamic_cast<const MatrixTransform&>(other)));
    case internal::traits<QuaternionTransform>::kTType:
      return std::make_shared<Derived>(derived().template applyPre(dynamic_cast<const QuaternionTransform&>(other)));
  }
  throw std::runtime_error("Transformation type not implemented");
}

template <class Derived>
TransformInterface::Ptr TransformBase<Derived>::applyPost_(const TransformInterface &other)
{
  switch (other.getType())
  {
    case internal::traits<AxisAngleTransform>::kTType:
      derived().template applyPost_(dynamic_cast<const AxisAngleTransform&>(other)); break;
    case internal::traits<DualQuaternionTransform>::kTType:
      derived().template applyPost_(dynamic_cast<const DualQuaternionTransform&>(other)); break;
    case internal::traits<EulerTransform>::kTType:
      derived().template applyPost_(dynamic_cast<const EulerTransform&>(other)); break;
    case internal::traits<MatrixTransform>::kTType:
      derived().template applyPost_(dynamic_cast<const MatrixTransform&>(other)); break;
    case internal::traits<QuaternionTransform>::kTType:
      derived().template applyPost_(dynamic_cast<const QuaternionTransform&>(other)); break;
    default:
      throw std::runtime_error("Transformation type not implemented");
  }
  return this->shared_from_this();
}

template <class Derived>
TransformInterface::Ptr TransformBase<Derived>::applyPost(const TransformInterface &other) const
{
  switch (other.getType())
  {
    case internal::traits<AxisAngleTransform>::kTType:
      return std::make_shared<Derived>(derived().template applyPost(dynamic_cast<const AxisAngleTransform&>(other)));
    case internal::traits<DualQuaternionTransform>::kTType:
      return std::make_shared<Derived>(derived().template applyPost(dynamic_cast<const DualQuaternionTransform&>(other)));
    case internal::traits<EulerTransform>::kTType:
      return std::make_shared<Derived>(derived().template applyPost(dynamic_cast<const EulerTransform&>(other)));
    case internal::traits<MatrixTransform>::kTType:
      return std::make_shared<Derived>(derived().template applyPost(dynamic_cast<const MatrixTransform&>(other)));
    case internal::traits<QuaternionTransform>::kTType:
      return std::make_shared<Derived>(derived().template applyPost(dynamic_cast<const QuaternionTransform&>(other)));
  }
  throw std::runtime_error("Transformation type not implemented");
}

// AxisAngleTransform
template<class T>
AxisAngleTransform& AxisAngleTransform::applyPre_(const T &other)
{
  *this = std::move(this->applyPre(other));
  return *this;
}

template<class T>
AxisAngleTransform AxisAngleTransform::applyPre(const T &other) const
{
  return (this->asType<QuaternionTransform>().template applyPre_(other)).template asType<AxisAngleTransform>();
}

template<class T>
AxisAngleTransform& AxisAngleTransform::applyPost_(const T &other)
{
  *this = std::move(this->applyPost(other));
  return *this;
}

template<class T>
AxisAngleTransform AxisAngleTransform::applyPost(const T &other) const
{
  return (this->asType<QuaternionTransform>().template applyPost_(other)).template asType<AxisAngleTransform>();
}

// DualQuaternionTransform
template<class T>
DualQuaternionTransform& DualQuaternionTransform::applyPre_(const T &other)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
  {
    return this->applyPre_(*other->template asType<DualQuaternionTransform>());
  }
  else if constexpr (std::is_same<DualQuaternionTransform, T>::value)
  {
    this->dq_ = other.getDualQuaternion() * this->dq_;
    this->unsafe_ = this->unsafe_ || other.isUnsafe();
    return *this;
  }
  else if constexpr (std::is_base_of<TransformInterface, T>::value)
  {
    return this->applyPre_(other.template asType<DualQuaternionTransform>());
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
DualQuaternionTransform DualQuaternionTransform::applyPre(const T &other) const
{
  return DualQuaternionTransform(*this).template applyPre_(other);
}

template<class T>
DualQuaternionTransform& DualQuaternionTransform::applyPost_(const T &other)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
  {
    return this->applyPost_(*other->template asType<DualQuaternionTransform>());
  }
  else if constexpr (std::is_same<DualQuaternionTransform, T>::value)
  {
    this->dq_ *= other.getDualQuaternion();
    this->unsafe_ = this->unsafe_ || other.isUnsafe();
    return *this;
  }
  else if constexpr (std::is_base_of<TransformInterface, T>::value)
  {
    return this->applyPost_(other.template asType<DualQuaternionTransform>());
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
DualQuaternionTransform DualQuaternionTransform::applyPost(const T &other) const
{
  return DualQuaternionTransform(*this).template applyPost_(other);
}

// EulerTransform
template<class T>
EulerTransform& EulerTransform::applyPre_(const T &other)
{
  *this = std::move(this->applyPre(other));
  return *this;
}

template<class T>
EulerTransform EulerTransform::applyPre(const T &other) const
{
  return EulerTransform(this->asType<MatrixTransform>().template applyPre_(other), this->axes_);
}

template<class T>
EulerTransform& EulerTransform::applyPost_(const T &other)
{
  *this = std::move(this->applyPost(other));
  return *this;
}

template<class T>
EulerTransform EulerTransform::applyPost(const T &other) const
{
  return EulerTransform(this->asType<MatrixTransform>().template applyPost_(other), this->axes_);
}

// MatrixTransform
template<class T>
MatrixTransform& MatrixTransform::applyPre_(const T &other)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
  {
    return this->applyPre_(*other->template asType<MatrixTransform>());
  }
  else if constexpr (std::is_same<MatrixTransform, T>::value)
  {
    this->matrix_ = other.getMatrix() * this->matrix_;
    this->unsafe_ = this->unsafe_ || other.isUnsafe();
    return *this;
  }
  else if constexpr (std::is_base_of<TransformInterface, T>::value)
  {
    return this->applyPre_(other.template asType<MatrixTransform>());
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
MatrixTransform MatrixTransform::applyPre(const T &other) const
{
  return MatrixTransform(*this).template applyPre_(other);
}

template<class T>
MatrixTransform& MatrixTransform::applyPost_(const T &other)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
  {
    return this->applyPost_(*other->template asType<MatrixTransform>());
  }
  else if constexpr (std::is_same<MatrixTransform, T>::value)
  {
    this->matrix_ *= other.getMatrix();
    this->unsafe_ = this->unsafe_ || other.isUnsafe();
    return *this;
  }
  else if constexpr (std::is_base_of<TransformInterface, T>::value)
  {
    return this->applyPost_(other.template asType<MatrixTransform>());
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
MatrixTransform MatrixTransform::applyPost(const T &other) const
{
  return MatrixTransform(*this).template applyPost_(other);
}

// QuaternionTransform
template<class T>
QuaternionTransform& QuaternionTransform::applyPre_(const T &other)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
  {
    return this->applyPre_(*other->template asType<QuaternionTransform>());
  }
  else if constexpr (std::is_same<QuaternionTransform, T>::value)
  {
    this->translation_ = other.getTranslation() + other.getQuaternion().transformPoint(this->translation_);
    this->quaternion_ = other.getQuaternion() * this->quaternion_;
    this->unsafe_ = this->unsafe_ || other.isUnsafe();
    return *this;
  }
  else if constexpr (std::is_base_of<TransformInterface, T>::value)
  {
    return this->applyPre_(other.template asType<QuaternionTransform>());
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
QuaternionTransform QuaternionTransform::applyPre(const T &other) const
{
  return QuaternionTransform(*this).template applyPre_(other);
}

template<class T>
QuaternionTransform& QuaternionTransform::applyPost_(const T &other)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
  {
    return this->applyPost_(*other->template asType<QuaternionTransform>());
  }
  else if constexpr (std::is_same<QuaternionTransform, T>::value)
  {
    this->translation_ += this->quaternion_.transformPoint(other.getTranslation());
    this->quaternion_ *= other.getQuaternion();
    this->unsafe_ = this->unsafe_ || other.isUnsafe();
    return *this;
  }
  else if constexpr (std::is_base_of<TransformInterface, T>::value)
  {
    return this->applyPost_(other.template asType<QuaternionTransform>());
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
QuaternionTransform QuaternionTransform::applyPost(const T &other) const
{
  return QuaternionTransform(*this).template applyPost_(other);
}

} // namespace motion3d