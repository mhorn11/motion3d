#pragma once

#include <motion3d/transforms/base.hpp>
#include <motion3d/transforms/dual_quaternion_.hpp>

namespace motion3d
{

template<class Derived>
bool TransformBase<Derived>::isEqual(const TransformInterface::ConstPtr &other, const double eps) const
{
  // invalid handling
  if (!this->isValid() || !other->isValid())
  {
    return false;
  }

  // get normalized dual quaternions
  DualQuaternion dq1 = TransformInterface::template asType<DualQuaternionTransform>()->getDualQuaternion();
  DualQuaternion dq2 = other->asType<DualQuaternionTransform>()->getDualQuaternion();

  // check equality
  return dq1.isEqual(dq2, eps) || dq1.isEqual(-dq2, eps);
}

template<class Derived>
template<class Other>
bool TransformBase<Derived>::isEqual(const TransformBase<Other> &other, const double eps) const
{
  // invalid handling
  if (!this->isValid() || !other.isValid())
  {
    return false;
  }

  // get normalized dual quaternions
  DualQuaternion dq1 = TransformInterface::template asType<DualQuaternionTransform>()->getDualQuaternion();
  DualQuaternion dq2 = other.template asType<DualQuaternionTransform>()->getDualQuaternion();

  // check equality
  return dq1.isEqual(dq2, eps) || dq1.isEqual(-dq2, eps);
}

} // namespace motion3d
