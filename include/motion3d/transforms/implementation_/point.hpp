#pragma once

#include <motion3d/transforms/axis_angle_.hpp>
#include <motion3d/transforms/dual_quaternion_.hpp>
#include <motion3d/transforms/euler_.hpp>
#include <motion3d/transforms/matrix_.hpp>
#include <motion3d/transforms/quaternion_.hpp>

namespace motion3d
{

// TransformBase
template <class Derived>
Eigen::Matrix<double, 3, 1> TransformBase<Derived>::transformPoint(
    const Eigen::Ref<const Eigen::Matrix<double, 3, 1>> &point) const
{
  return derived().transformPoint(point);
}

template <class Derived>
Eigen::Matrix<double, 3, Eigen::Dynamic> TransformBase<Derived>::transformCloud(
    const Eigen::Ref<const Eigen::Matrix<double, 3, Eigen::Dynamic>> &cloud) const
{
  return derived().transformCloud(cloud);
}

// AxisAngleTransform
template<typename Derived>
Eigen::Matrix<double, 3, 1> AxisAngleTransform::transformPoint(const Eigen::MatrixBase<Derived> &point) const
{
  return this->asType<QuaternionTransform>().transformPoint(point);
}

template<typename Derived>
Eigen::Matrix<double, 3, Derived::ColsAtCompileTime> AxisAngleTransform::transformCloud(const Eigen::MatrixBase<Derived> &cloud) const
{
  return this->asType<MatrixTransform>().transformCloud(cloud);
}

// DualQuaternionTransform
template<typename Derived>
Eigen::Matrix<double, 3, 1> DualQuaternionTransform::transformPoint(const Eigen::MatrixBase<Derived> &point) const
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 1);
  return this->dq_.transformPoint(point);
}

template<typename Derived>
Eigen::Matrix<double, 3, Derived::ColsAtCompileTime> DualQuaternionTransform::transformCloud(const Eigen::MatrixBase<Derived> &cloud) const
{
  return this->asType<MatrixTransform>().transformCloud(cloud);
}

// EulerTransform
template<typename Derived>
Eigen::Matrix<double, 3, 1> EulerTransform::transformPoint(const Eigen::MatrixBase<Derived> &point) const
{
  return this->asType<MatrixTransform>().transformPoint(point);
}

template<typename Derived>
Eigen::Matrix<double, 3, Derived::ColsAtCompileTime> EulerTransform::transformCloud(const Eigen::MatrixBase<Derived> &cloud) const
{
  return this->asType<MatrixTransform>().transformCloud(cloud);
}

// MatrixTransform
template<typename Derived>
Eigen::Matrix<double, 3, 1> MatrixTransform::transformPoint(const Eigen::MatrixBase<Derived> &point) const
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 1);
  return this->getRotationMatrix() * point + this->getTranslation();
}

template<typename Derived>
Eigen::Matrix<double, 3, Derived::ColsAtCompileTime> MatrixTransform::transformCloud(const Eigen::MatrixBase<Derived> &cloud) const
{
  EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  if constexpr (Derived::ColsAtCompileTime == Eigen::Dynamic)
  {
    Eigen::Matrix<double, 4, Eigen::Dynamic> cloud_hom;
    cloud_hom.resize(4, cloud.cols());
    cloud_hom << cloud, Eigen::Matrix<double, 1, Eigen::Dynamic>::Constant(1, cloud.cols(), 1.0);
    Eigen::Matrix<double, 4, Eigen::Dynamic> cloud_new_hom = this->matrix_ * cloud_hom;
    return cloud_new_hom.template topRows<3>();
  }
  else
  {
    Eigen::Matrix<double, 4, Derived::ColsAtCompileTime> cloud_hom;
    cloud_hom << cloud, Eigen::Matrix<double, 1, Derived::ColsAtCompileTime>::Constant(1.0);
    Eigen::Matrix<double, 4, Derived::ColsAtCompileTime> cloud_new_hom = this->matrix_ * cloud_hom;
    return cloud_new_hom.template topRows<3>();
  }
}

// QuaternionTransform
template<typename Derived>
Eigen::Matrix<double, 3, 1> QuaternionTransform::transformPoint(const Eigen::MatrixBase<Derived> &point) const
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 1);
  return this->quaternion_.transformPoint(point) + this->getTranslation();
}

template<typename Derived>
Eigen::Matrix<double, 3, Derived::ColsAtCompileTime> QuaternionTransform::transformCloud(const Eigen::MatrixBase<Derived> &cloud) const
{
  return this->asType<MatrixTransform>().transformCloud(cloud);
}

} // namespace motion3d