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

// multiplication copy operators
template<class T>
TransformInterface::Ptr operator*(const TransformInterface::ConstPtr &t1, const T &t2)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value ||
    std::is_same<TransformInterface::ConstPtr, T>::value ||
    std::is_base_of<TransformInterface, T>::value)
  {
    return t1->applyPost(t2);
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
AxisAngleTransform operator*(const AxisAngleTransform &t1, const T &t2)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value ||
    std::is_same<TransformInterface::ConstPtr, T>::value ||
    std::is_base_of<TransformInterface, T>::value)
  {
    return t1.applyPost(t2);
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
DualQuaternionTransform operator*(const DualQuaternionTransform &t1, const T &t2)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value ||
    std::is_same<TransformInterface::ConstPtr, T>::value ||
    std::is_base_of<TransformInterface, T>::value)
  {
    return t1.applyPost(t2);
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
EulerTransform operator*(const EulerTransform &t1, const T &t2)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value ||
    std::is_same<TransformInterface::ConstPtr, T>::value ||
    std::is_base_of<TransformInterface, T>::value)
  {
    return t1.applyPost(t2);
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
MatrixTransform operator*(const MatrixTransform &t1, const T &t2)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value ||
    std::is_same<TransformInterface::ConstPtr, T>::value ||
    std::is_base_of<TransformInterface, T>::value)
  {
    return t1.applyPost(t2);
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
QuaternionTransform operator*(const QuaternionTransform &t1, const T &t2)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value ||
    std::is_same<TransformInterface::ConstPtr, T>::value ||
    std::is_base_of<TransformInterface, T>::value)
  {
    return t1.applyPost(t2);
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

// multiplication inplace operators
template<class T>
AxisAngleTransform& AxisAngleTransform::operator*=(const T &other)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value ||
                std::is_same<TransformInterface::ConstPtr, T>::value ||
                std::is_base_of<TransformInterface, T>::value)
  {
    return this->applyPost_(other);
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
DualQuaternionTransform& DualQuaternionTransform::operator*=(const T &other)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value ||
                std::is_same<TransformInterface::ConstPtr, T>::value ||
                std::is_base_of<TransformInterface, T>::value)
  {
    return this->applyPost_(other);
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
EulerTransform& EulerTransform::operator*=(const T &other)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value ||
                std::is_same<TransformInterface::ConstPtr, T>::value ||
                std::is_base_of<TransformInterface, T>::value)
  {
    return this->applyPost_(other);
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
MatrixTransform& MatrixTransform::operator*=(const T &other)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value ||
                std::is_same<TransformInterface::ConstPtr, T>::value ||
                std::is_base_of<TransformInterface, T>::value)
  {
    return this->applyPost_(other);
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
QuaternionTransform& QuaternionTransform::operator*=(const T &other)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value ||
                std::is_same<TransformInterface::ConstPtr, T>::value ||
                std::is_base_of<TransformInterface, T>::value)
  {
    return this->applyPost_(other);
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

// division copy operators
template<class T>
TransformInterface::Ptr operator/(const TransformInterface::ConstPtr &t1, const T &t2)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
  {
    return t1->applyPost(t2->inverse());
  }
  else if constexpr (std::is_base_of<TransformInterface, T>::value)
  {
    return t1->applyPost(t2.inverse());
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
AxisAngleTransform operator/(const AxisAngleTransform &t1, const T &t2)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
  {
    return t1.applyPost(t2->inverse());
  }
  else if constexpr (std::is_base_of<TransformInterface, T>::value)
  {
    return t1.applyPost(t2.inverse());
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
DualQuaternionTransform operator/(const DualQuaternionTransform &t1, const T &t2)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
  {
    return t1.applyPost(t2->inverse());
  }
  else if constexpr (std::is_base_of<TransformInterface, T>::value)
  {
    return t1.applyPost(t2.inverse());
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
EulerTransform operator/(const EulerTransform &t1, const T &t2)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
  {
    return t1.applyPost(t2->inverse());
  }
  else if constexpr (std::is_base_of<TransformInterface, T>::value)
  {
    return t1.applyPost(t2.inverse());
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
MatrixTransform operator/(const MatrixTransform &t1, const T &t2)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
  {
    return t1.applyPost(t2->inverse());
  }
  else if constexpr (std::is_base_of<TransformInterface, T>::value)
  {
    return t1.applyPost(t2.inverse());
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
QuaternionTransform operator/(const QuaternionTransform &t1, const T &t2)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
  {
    return t1.applyPost(t2->inverse());
  }
  else if constexpr (std::is_base_of<TransformInterface, T>::value)
  {
    return t1.applyPost(t2.inverse());
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

// division inplace operators
template<class T>
AxisAngleTransform& AxisAngleTransform::operator/=(const T &other)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
  {
    return this->applyPost_(other->inverse());
  }
  else if constexpr (std::is_base_of<TransformInterface, T>::value)
  {
    return this->applyPost_(other.inverse());
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
DualQuaternionTransform& DualQuaternionTransform::operator/=(const T &other)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
  {
    return this->applyPost_(other->inverse());
  }
  else if constexpr (std::is_base_of<TransformInterface, T>::value)
  {
    return this->applyPost_(other.inverse());
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
EulerTransform& EulerTransform::operator/=(const T &other)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
  {
    return this->applyPost_(other->inverse());
  }
  else if constexpr (std::is_base_of<TransformInterface, T>::value)
  {
    return this->applyPost_(other.inverse());
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
MatrixTransform& MatrixTransform::operator/=(const T &other)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
  {
    return this->applyPost_(other->inverse());
  }
  else if constexpr (std::is_base_of<TransformInterface, T>::value)
  {
    return this->applyPost_(other.inverse());
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

template<class T>
QuaternionTransform& QuaternionTransform::operator/=(const T &other)
{
  if constexpr (std::is_same<TransformInterface::Ptr, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
  {
    return this->applyPost_(other->inverse());
  }
  else if constexpr (std::is_base_of<TransformInterface, T>::value)
  {
    return this->applyPost_(other.inverse());
  }
  else
  {
    static_assert(always_false<T>, "Input must be a transform");
  }
}

// stream operators
std::ostream& operator<<(std::ostream& os, const TransformInterface::Ptr& ptr)
{
  return ptr->stream(os);
}

std::ostream& operator<<(std::ostream& os, const TransformInterface::ConstPtr& ptr)
{
  return ptr->stream(os);
}

template <class Derived>
std::ostream& operator<<(std::ostream& os, const TransformBase<Derived>& obj)
{
  return obj.stream(os);
}

} // namespace motion3d
