#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <tuple>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <motion3d/utils/exceptions.hpp>
#include <motion3d/utils/templating.hpp>

namespace motion3d
{

constexpr double kDefaultEps = 1e-6;

/** Exception thrown in case a mathematical operation fails or is undefined. */
struct MathException : public MessageException
{
  explicit MathException(std::string msg) : MessageException(std::move(msg)) {}
};

/** Inplace variant of normalizeAngle(). */
template<typename _Scalar>
void normalizeAngle_(_Scalar &x)
{
  x = std::fmod(x + M_PI, 2.0 * M_PI);  // NOLINT(*-magic-numbers)
  if (x < 0.0)
  {
    x += 2.0 * M_PI;  // NOLINT(*-magic-numbers)
  }
  x -= M_PI;
}

/** Normalizes angle <I>x</I> to range \f$[-\pi,\pi)\f$. */
template<typename _Scalar>
_Scalar normalizeAngle(_Scalar x)
{
  normalizeAngle_(x);
  return x;
}

/** \returns the inverse factor for normalizing an axis to the upper hemisphere. */
template<typename Derived>
typename Derived::Scalar getAxisNormalizationFactorInverse(const Eigen::MatrixBase<Derived> &v, bool unit_norm)
{
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);

  // base factor
  typename Derived::Scalar factor = unit_norm ? v.norm() : 1.0;

  // get length
  std::size_t v_len = v.size();

  // check hemisphere
  for (size_t i = 0; i < v_len; ++i)
  {
    if (v(i) > 0.0)
    {
      return factor;
    }
    if (v(i) < 0.0)
    {
      return -factor;
    }
  }

  return factor;
}

/** Inplace variant of normalizeRotationMatrix(). */
template<typename Derived>
void normalizeRotationMatrix_(Eigen::MatrixBase<Derived> &m)
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);

  auto det = m.determinant();
  if (std::abs(det) < kDefaultEps)
  {
    m.setIdentity();
  }
  else
  {
    // unit determinant
    m *= std::cbrt(1.0 / std::abs(det));

    // orthogonalize and make determinant positive
    auto svd = m.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    if (det > 0)
    {
      m = svd.matrixU() * svd.matrixV().transpose();
    }
    else
    {
      Eigen::Matrix<typename Derived::Scalar, 3, 1> mirror;
      mirror << 1.0, 1.0, -1.0;
      m = svd.matrixU() * mirror.asDiagonal() * svd.matrixV().transpose();
    }
  }
}

/** Normalizes matrix <I>m</I> to a valid \f$\mathrm{SO(3)}\f$ rotation matrix. */
template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> normalizeRotationMatrix(const Eigen::MatrixBase<Derived> &m)
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);
  Eigen::Matrix<typename Derived::Scalar, 3, 3> m_copy(m);
  normalizeRotationMatrix_(m_copy);
  return m_copy;
}

/** Decomposes a 3x3 matrix into rotations, zooms and shears.
  * \note The implementation is based on <CODE>transforms3d.affines.decomposeA44</CODE>:
  *       https://github.com/matthew-brett/transforms3d/blob/master/transforms3d/affines.py#L10
  */
template<typename Derived>
std::tuple<Eigen::Matrix<typename Derived::Scalar, 3, 3>, Eigen::Matrix<typename Derived::Scalar, 3, 1>, Eigen::Matrix<typename Derived::Scalar, 3, 1>> 
  decomposeRZS(const Eigen::MatrixBase<Derived> &m)
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);

  // compute scales and shears
  Eigen::Matrix<typename Derived::Scalar, 3, 1> m0 = m.template block<3, 1>(0, 0);
  Eigen::Matrix<typename Derived::Scalar, 3, 1> m1 = m.template block<3, 1>(0, 1);
  Eigen::Matrix<typename Derived::Scalar, 3, 1> m2 = m.template block<3, 1>(0, 2);

  // extract x scale and normalize
  auto sx = m0.norm();
  m0 /= sx;

  // orthogonalize m1 with respect to m0
  auto sx_sxy = m0.dot(m1);
  m1 -= sx_sxy * m0;

  // extract y scale and normalize
  auto sy = m1.norm();
  m1 /= sy;
  auto sxy = sx_sxy / sx;

  // orthogonalize m2 with respect to m0 and m1
  auto sx_sxz = m0.dot(m2);
  auto sy_syz = m1.dot(m2);
  m2 -= (sx_sxz * m0 + sy_syz * m1);

  // extract z scale and normalize
  auto sz = m2.norm();
  m2 /= sz;
  auto sxz = sx_sxz / sx;
  auto syz = sy_syz / sy;

  // reconstruct rotation matrix, ensure positive determinant
  Eigen::Matrix<typename Derived::Scalar, 3, 3> rmat;
  rmat.template block<3, 1>(0, 0) = m0;
  rmat.template block<3, 1>(0, 1) = m1;
  rmat.template block<3, 1>(0, 2) = m2;
  if (rmat.determinant() < 0)
  {
    sx *= -1.0;
    rmat.template block<3, 1>(0, 0) *= -1.0;
  }

  // zooms and shears
  Eigen::Matrix<typename Derived::Scalar, 3, 1> zooms;
  zooms << sx, sy, sz;
  Eigen::Matrix<typename Derived::Scalar, 3, 1> shears;
  shears << sxy, sxz, syz;

  return std::make_tuple(rmat, zooms, shears);
}

} // namespace motion3d
