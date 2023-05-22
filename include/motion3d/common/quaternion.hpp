#pragma once

#include <Eigen/Geometry>

#include <motion3d/common/math.hpp>
#include <motion3d/utils/templating.hpp>

namespace motion3d
{

constexpr int kQuaternionDim = 4;

// forward definitions for templated friend functions
template<class _Scalar> class Quaternion;
template<class _Scalar> std::ostream& operator<<(std::ostream& os, const Quaternion<_Scalar>& q);

/** \brief The quaternion class used for performing computations involving quaternions, 
  *        including but not limited to 3D rotations.
  * \tparam _Scalar the scalar type, i.e., the type of the coefficients

  * This class represents a quaternion \f$\mathrm{q}=w+xi+yj+zk\f$, extending the implementation of <CODE>Eigen::Quaternion</CODE>.
  * It can also denoted vectorized as \f$\mathrm{vec(q)}=[w \ x \ y \ z]^\mathrm{T}\f$ or in scalar-vector notation as \f$\mathrm{q}=\{ w,\boldsymbol{v} \}\f$
  * with \f$\boldsymbol{v}=[x \ y \ z]^\mathrm{T}\f$.
  * 
  * A unit quaternion satifies \f$||\mathrm{r}|| = 1\f$.
  * Unit quaternions can be used to represent spatial rotations.
  *
  * \warning Operations interpreting the quaternion as rotation have undefined behavior if the quaternion is not normalized.
  * \warning The original <CODE>coeffs()</CODE> implemented in <CODE>Eigen::Quaternion</CODE> are in order \f$[x \ y \ z \ w]^\mathrm{T}\f$.
  * \sa https://eigen.tuxfamily.org/dox/classEigen_1_1QuaternionBase.html
  */
template<typename _Scalar>
class Quaternion : public Eigen::Quaternion<_Scalar>
{
 public:
  using AngleAxisType = typename Eigen::Quaternion<_Scalar>::AngleAxisType;
  using Matrix3 = typename Eigen::Quaternion<_Scalar>::Matrix3;
  using Vector3 = typename Eigen::Quaternion<_Scalar>::Vector3;

  /** Default constructor leaving the quaternion uninitialized. */
  Quaternion() : Eigen::Quaternion<_Scalar>() {}

  /** Constructs and initializes a quaternion from the angle-axis <I>aa</I>. */
  explicit Quaternion(const AngleAxisType &aa)
    : Eigen::Quaternion<_Scalar>(aa) {}

  /** Constructs and initializes a quaternion from the axis-angle representation with the <I>angle</I> in <I>rad</I>. */
  template<class Derived>
  Quaternion(const _Scalar &angle, const Eigen::MatrixBase<Derived> &axis)
    : Quaternion(AngleAxisType(angle, axis)) {}

  /** Constructs and initializes the quaternion from its four coefficients <I>w</I>, <I>x</I>, <I>y</I> and <I>z</I>. */
  Quaternion(const _Scalar &w, const _Scalar &x, const _Scalar &y, const _Scalar &z)
    : Eigen::Quaternion<_Scalar>(w, x, y, z) {}

  /** Explicit copy constructor from Eigen quaternion with scalar conversion. */
  template<typename OtherScalar, int OtherOptions>
  explicit Quaternion(const Eigen::Quaternion<OtherScalar, OtherOptions> &other)
    : Eigen::Quaternion<_Scalar>(other) {}  

  // NOLINTBEGIN(google-explicit-constructor)
  /** Copy constructor from Eigen quaternion. */
  template<class Derived>
  Quaternion(const Eigen::QuaternionBase<Derived> &other)
    : Eigen::Quaternion<_Scalar>(other) {}

  /** Move constructor from Eigen quaternion. */
  Quaternion(Eigen::Quaternion<_Scalar> &&other)
    : Eigen::Quaternion<_Scalar>(other) {}
  // NOLINTEND(google-explicit-constructor)

  /** Move assignment operator from Eigen quaternion. */
  Quaternion<_Scalar>& operator=(Eigen::Quaternion<_Scalar> &&other) {
    Eigen::Quaternion<_Scalar>::operator=(other);
    return *this;
  }

 private:
  /** Constructs and initializes a quaternion from either:
    * \li a rotation matrix expression,
    * \li a 4D vector expression representing quaternion coefficients in the order \f$[x \ y \ z \ w]^\mathrm{T}\f$.
    *
    * \warning The coefficient order in this method is different from the usual order \f$[w \ x \ y \ z]^\mathrm{T}\f$.
    */
  template<typename Derived>
  explicit Quaternion(const Eigen::MatrixBase<Derived> &other)
    : Eigen::Quaternion<_Scalar>(other) {}

  /** Constructs and initialize a quaternion from the array data in the order \f$[x \ y \ z \ w]^\mathrm{T}\f$.
    * \warning The coefficient order in this method is different from the usual order \f$[w \ x \ y \ z]^\mathrm{T}\f$.
    */
  template<typename Derived>
  explicit Quaternion(const _Scalar *data)
    : Eigen::Quaternion<_Scalar>(data) {}

 public:
  /** \returns a quaternion filled with zeros. */
  static Quaternion<_Scalar> Zero();

  /** \returns an identity quaternion \f$ \{ 1, \boldsymbol{0} \} \f$. */
  static Quaternion<_Scalar> Identity();

  /** \returns a quaternion from an std vector with the order \f$[w \ x \ y \ z]^\mathrm{T}\f$. */
  static Quaternion<_Scalar> FromVector(const std::vector<_Scalar> &data);

  /** \returns a quaternion from an Eigen vector with the order \f$[w \ x \ y \ z]^\mathrm{T}\f$. */
  template<typename Derived>
  static Quaternion<_Scalar> FromVector(const Eigen::DenseBase<Derived> &data);
  
  /** \returns a quaternion converted from a rotation matrix expression. */
  template<class Derived>
  static Quaternion<_Scalar> FromRotationMatrix(const Eigen::MatrixBase<Derived> &matrix);

  /** Converts <CODE>*this</CODE> to std vector with the order \f$[w \ x \ y \ z]^\mathrm{T}\f$. */
  std::vector<_Scalar> toVector() const;

  /** Converts <CODE>*this</CODE> to Eigen vector with the order \f$[w \ x \ y \ z]^\mathrm{T}\f$. */
  Eigen::Matrix<_Scalar, kQuaternionDim, 1> toEigenVector() const;

  /** Converts the left quaternion of a quaternion multiplication to a 4x4 matrix
    * for representing the multiplication as matrix-vector product
    * \f$\text{vec}(\mathrm{q}_a \mathrm{q}_b) = \boldsymbol{Q}_a^{+} \text{vec}(\mathrm{q}_b)\f$.
    * 
    * The following two lines yield identical results for <CODE>Quaterniond</CODE> <CODE>q1</CODE> and <CODE>q2</CODE>:
    * \code{.cpp}
    *   Eigen::Matrix<double, 4, 1> v1 = (q1 * q2).toEigenVector();
    *   Eigen::Matrix<double, 4, 1> v2 = q1.toPositiveMatrix() * q2.toEigenVector();
    * \endcode
    */
  Eigen::Matrix<_Scalar, kQuaternionDim, kQuaternionDim> toPositiveMatrix() const;

  /** Converts the right quaternion of a quaternion multiplication to a 4x4 matrix
    * for representing the multiplication as matrix-vector product
    * \f$\text{vec}(\mathrm{q}_a \mathrm{q}_b) = \boldsymbol{Q}_b^{-} \text{vec}(\mathrm{q}_a)\f$.
    * 
    * The following two lines yield identical results for <CODE>Quaterniond</CODE> <CODE>q1</CODE> and <CODE>q2</CODE>:
    * \code{.cpp}
    *   Eigen::Matrix<double, 4, 1> v1 = (q1 * q2).toEigenVector();
    *   Eigen::Matrix<double, 4, 1> v2 = q2.toNegativeMatrix() * q1.toEigenVector();
    * \endcode
    */
  Eigen::Matrix<_Scalar, kQuaternionDim, kQuaternionDim> toNegativeMatrix() const;

  /** Converts <CODE>*this</CODE> to axis-angle representation. */
  AngleAxisType toAxisAngle() const { return AngleAxisType(*this); }

  /** \returns the rotation norm in <I>rad</I>. */
  _Scalar rotationNorm() const;

  /** Inplace variant of conjugate(). */
  Quaternion<_Scalar>& conjugate_();

  /** \returns the conjugate quaternion \f$\mathrm{q}^{*} = w-xi-yj-zk\f$. */
  Quaternion<_Scalar> conjugate() const;

  /** Inplace variant of inverse(). */
  Quaternion<_Scalar>& inverse_();

  /** \returns the inverse quaternion so \f$\mathrm{q} \mathrm{q}^{-1} = \{ 1, \boldsymbol{0} \} \f$.
    * \throws MathException if the norm of the quaternion is 0.
    */
  Quaternion<_Scalar> inverse() const;

  /** Inplace variant of normalized(). */
  Quaternion<_Scalar>& normalized_();

  /** \returns the normalized quaternion with norm 1. */
  Quaternion<_Scalar> normalized() const;

  /** \returns the spherical linear interpolation between <CODE>*this</CODE> and <I>other</I> for \f$t \in [0,1]\f$. */
  Quaternion<_Scalar> slerp(const _Scalar &t, const Quaternion<_Scalar> &other) const;

  /** \returns <CODE>true</CODE> if <CODE>*this</CODE> is approximately equal to <I>other</I>, within the precision determined by <I>eps</I>. */
  bool isEqual(const Quaternion<_Scalar> &other, const _Scalar &eps = kDefaultEps) const;

  /** \returns the angular distance between <CODE>*this</CODE> and <I>other</I> in <I>rad</I>. */
  _Scalar angularDistance(const Quaternion<_Scalar> &other) const;

  /** Fills <CODE>*this</CODE> with zeros. */
  Quaternion<_Scalar>& setZero();

  /** \sa Identity() */
  Quaternion<_Scalar>& setIdentity();

  /** Rotates <I>point</I> \f$\boldsymbol{p}\f$ using \f$\mathrm{q} \mathrm{p} \mathrm{q}^{*}\f$ with \f$\mathrm{p} = \{ 0, \boldsymbol{p} \} \f$. */
  template<typename Derived>
  Vector3 transformPoint(const Eigen::MatrixBase<Derived> &point) const;

  /** \returns a description of <CODE>*this</CODE>. */
  std::string desc() const { return streamToString(*this); }

  /** Coefficient-wise multiplication of <I>*this</I> and <I>v</I>. */
  Quaternion<_Scalar> operator*(const _Scalar &v) const;

  /** \sa operator*(const _Scalar&) const */
  Quaternion<_Scalar>& operator*=(const _Scalar &v);

  /** \sa operator*(const _Scalar&) const */
  friend Quaternion<_Scalar> operator*(const _Scalar &v, const Quaternion<_Scalar> &q) { return q * v; }

  /** Quaternion product of <I>*this</I> and <I>other</I>. */
  Quaternion<_Scalar> operator*(const Quaternion<_Scalar> &other) const;

  /** \sa operator*(const Quaternion<_Scalar>&) const */
  Quaternion<_Scalar> operator*=(const Quaternion<_Scalar> &other);

  /** Coefficient-wise division of <I>*this</I> by <I>v</I>. */
  Quaternion<_Scalar> operator/(const _Scalar &v) const;

  /** \sa operator/(const _Scalar&) const */
  Quaternion<_Scalar>& operator/=(const _Scalar &v);

  /** Coefficient-wise summation of <I>*this</I> and <I>other</I>. */
  Quaternion<_Scalar> operator+(const Quaternion<_Scalar> &other) const;

  /** \sa operator+(const Quaternion<_Scalar>&) const */
  Quaternion<_Scalar>& operator+=(const Quaternion<_Scalar> &other);

  /** Coefficient-wise negative of <I>*this</I>. */
  Quaternion<_Scalar> operator-() const;

  /** Coefficient-wise subtraction of <I>*this</I> and <I>other</I>. */
  Quaternion<_Scalar> operator-(const Quaternion<_Scalar> &other) const;

  /** \sa operator-(const Quaternion<_Scalar>&) const */
  Quaternion<_Scalar>& operator-=(const Quaternion<_Scalar> &other);

  /** Inserts a description of <CODE>*this</CODE>. */
  template<typename _OtherScalar>
  friend std::ostream& operator<<(std::ostream& os, const Quaternion<_OtherScalar>& q);
};

template<typename _Scalar>
Quaternion<_Scalar> Quaternion<_Scalar>::Zero()
{
  return Quaternion<_Scalar>(Eigen::Matrix<_Scalar, kQuaternionDim, 1>::Zero());
}

template<typename _Scalar>
Quaternion<_Scalar> Quaternion<_Scalar>::Identity()
{
  return Eigen::Quaternion<_Scalar>::Identity();
}

template<typename _Scalar>
Quaternion<_Scalar> Quaternion<_Scalar>::FromVector(const std::vector<_Scalar> &data)
{
  if (data.size() != kQuaternionDim)
  {
    throw std::runtime_error("Invalid vector size");
  }
  return Quaternion<_Scalar>(data[0], data[1], data[2], data[3]);
}

template<typename _Scalar>
template<typename Derived>
Quaternion<_Scalar> Quaternion<_Scalar>::FromVector(const Eigen::DenseBase<Derived> &data)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, kQuaternionDim);
  return Quaternion<_Scalar>(data(0), data(1), data(2), data(3));
}

template<typename _Scalar>
template<class Derived>
Quaternion<_Scalar> Quaternion<_Scalar>::FromRotationMatrix(const Eigen::MatrixBase<Derived> &matrix)
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);
  return Quaternion<_Scalar>(matrix);
}

template<typename _Scalar>
std::vector<_Scalar> Quaternion<_Scalar>::toVector() const
{
  std::vector<_Scalar> data = {this->w(), this->x(), this->y(), this->z()};
  return data;
}

template<typename _Scalar>
Eigen::Matrix<_Scalar, kQuaternionDim, 1> Quaternion<_Scalar>::toEigenVector() const
{
  Eigen::Matrix<_Scalar, kQuaternionDim, 1> data;
  data << this->w(), this->vec();
  return data;
}

template<typename _Scalar>
Eigen::Matrix<_Scalar, kQuaternionDim, kQuaternionDim> Quaternion<_Scalar>::toPositiveMatrix() const
{
  Eigen::Matrix<_Scalar, kQuaternionDim, kQuaternionDim> m;
  m << this->w(), -this->x(), -this->y(), -this->z(),
      this->x(),  this->w(), -this->z(),  this->y(),
      this->y(),  this->z(),  this->w(), -this->x(),
      this->z(), -this->y(),  this->x(),  this->w();
  return m;
}

template<typename _Scalar>
Eigen::Matrix<_Scalar, kQuaternionDim, kQuaternionDim> Quaternion<_Scalar>::toNegativeMatrix() const
{
  Eigen::Matrix<_Scalar, kQuaternionDim, kQuaternionDim> m;
  m << this->w(), -this->x(), -this->y(), -this->z(),
      this->x(),  this->w(),  this->z(), -this->y(),
      this->y(), -this->z(),  this->w(),  this->x(),
      this->z(),  this->y(), -this->x(),  this->w();
  return m;
}

template<typename _Scalar>
_Scalar Quaternion<_Scalar>::rotationNorm() const
{
  _Scalar n = this->vec().norm();
  if (n != 0.0)
  {
    return 2.0 * std::atan2(n, std::abs(this->w()));  // NOLINT(*-magic-numbers)
  }
  return 0.0;
}

template<typename _Scalar>
Quaternion<_Scalar>& Quaternion<_Scalar>::conjugate_()
{
  this->vec() *= -1;
  return *this;
}

template<typename _Scalar>
Quaternion<_Scalar> Quaternion<_Scalar>::conjugate() const
{
  return Eigen::Quaternion<_Scalar>::conjugate();
}

template<typename _Scalar>
Quaternion<_Scalar>& Quaternion<_Scalar>::inverse_()
{
  _Scalar squared_norm = Eigen::Quaternion<_Scalar>::squaredNorm();
  if (squared_norm <= kDefaultEps)
  {
    throw MathException("The inverse of quaternions with norm 0 is undefined");
  }
  this->conjugate_();
  this->coeffs() /= squared_norm;
  return *this;
}

template<typename _Scalar>
Quaternion<_Scalar> Quaternion<_Scalar>::inverse() const
{
  return Quaternion<_Scalar>(*this).inverse_();
}

template<typename _Scalar>
Quaternion<_Scalar>& Quaternion<_Scalar>::normalized_()
{
  _Scalar factor = getAxisNormalizationFactorInverse(this->toEigenVector(), true);
  if (std::abs(factor) < kDefaultEps)
  {
    this->setIdentity();
  }
  else
  {
    this->coeffs() /= factor;
  }
  return *this;
}

template<typename _Scalar>
Quaternion<_Scalar> Quaternion<_Scalar>::normalized() const
{
  return Quaternion<_Scalar>(*this).normalized_();
}

template<typename _Scalar>
Quaternion<_Scalar> Quaternion<_Scalar>::slerp(const _Scalar &t, const Quaternion<_Scalar> &other) const
{
  return Eigen::Quaternion<_Scalar>::slerp(t, other);
}

template<typename _Scalar>
bool Quaternion<_Scalar>::isEqual(const Quaternion<_Scalar> &other, const _Scalar &eps) const
{
  // isApprox does not work for comparison against the zero vector
  if (other.norm() < eps)
  {
    return (this->coeffs() - other.coeffs()).norm() < eps;
  }
  return Eigen::Quaternion<_Scalar>::isApprox(other, eps);
}

template<typename _Scalar>
_Scalar Quaternion<_Scalar>::angularDistance(const Quaternion<_Scalar> &other) const
{
  return Eigen::Quaternion<_Scalar>::angularDistance(other);
}

template<typename _Scalar>
Quaternion<_Scalar>& Quaternion<_Scalar>::setZero()
{
  this->coeffs() << static_cast<_Scalar>(0), static_cast<_Scalar>(0), static_cast<_Scalar>(0), static_cast<_Scalar>(0);
  return *this;
}

template<typename _Scalar>
Quaternion<_Scalar>& Quaternion<_Scalar>::setIdentity()
{
  Eigen::Quaternion<_Scalar>::setIdentity();
  return *this;
}

template<typename _Scalar>
template<typename Derived>
typename Quaternion<_Scalar>::Vector3 Quaternion<_Scalar>::transformPoint(
  const Eigen::MatrixBase<Derived> &point) const
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 1);
  return this->_transformVector(point);
}

template<typename _Scalar>
Quaternion<_Scalar> Quaternion<_Scalar>::operator*(const _Scalar &v) const
{
  return Quaternion<_Scalar>(this->coeffs() * v);
}

template<typename _Scalar>
Quaternion<_Scalar>& Quaternion<_Scalar>::operator*=(const _Scalar &v)
{
  this->coeffs() *= v;
  return *this;
}

template<typename _Scalar>
Quaternion<_Scalar> Quaternion<_Scalar>::operator*(const Quaternion<_Scalar> &other) const
{
  return Eigen::Quaternion<_Scalar>::operator*(other);
}

template<typename _Scalar>
Quaternion<_Scalar> Quaternion<_Scalar>::operator*=(const Quaternion<_Scalar> &other)
{
  return Eigen::Quaternion<_Scalar>::operator*=(other);
}

template<typename _Scalar>
Quaternion<_Scalar> Quaternion<_Scalar>::operator/(const _Scalar &v) const
{
  return Quaternion<_Scalar>(this->coeffs() / v);
}

template<typename _Scalar>
Quaternion<_Scalar>& Quaternion<_Scalar>::operator/=(const _Scalar &v)
{
  this->coeffs() /= v;
  return *this;
}

template<typename _Scalar>
Quaternion<_Scalar> Quaternion<_Scalar>::operator+(const Quaternion<_Scalar> &other) const
{
  Quaternion<_Scalar> copy(*this);
  copy.coeffs() += other.coeffs();
  return copy;
}

template<typename _Scalar>
Quaternion<_Scalar>& Quaternion<_Scalar>::operator+=(const Quaternion<_Scalar> &other)
{
  this->coeffs() += other.coeffs();
  return *this;
}

template<typename _Scalar>
Quaternion<_Scalar> Quaternion<_Scalar>::operator-() const
{
  return Quaternion<_Scalar>(-this->coeffs());
}

template<typename _Scalar>
Quaternion<_Scalar> Quaternion<_Scalar>::operator-(const Quaternion<_Scalar> &other) const
{
  Quaternion<_Scalar> copy(*this);
  copy.coeffs() -= other.coeffs();
  return copy;
}

template<typename _Scalar>
Quaternion<_Scalar>& Quaternion<_Scalar>::operator-=(const Quaternion<_Scalar> &other)
{
  this->coeffs() -= other.coeffs();
  return *this;
}

template<typename _Scalar>
std::ostream& operator<<(std::ostream &os, const Quaternion<_Scalar> &q)
{
  return os << "Quaternion([" << q.w() << " " << q.vec().transpose() << "])";
}

using Quaternionf = Quaternion<float>;
using Quaterniond = Quaternion<double>;

} // namespace motion3d
