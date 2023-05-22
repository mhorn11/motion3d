#pragma once

#include <Eigen/Geometry>

#include <motion3d/common/math.hpp>
#include <motion3d/common/quaternion.hpp>
#include <motion3d/utils/templating.hpp>

namespace motion3d
{

constexpr int kDualQuaternionDim = 8;

// forward definitions for templated friend functions
template<class _Scalar> class DualQuaternion;
template<class _Scalar> std::ostream& operator<<(std::ostream& os, const DualQuaternion<_Scalar>& dq);

/** \brief The dual quaternion class used for performing computations involving dual quaternions, 
  *        including but not limited to 3D transformations.
  * \tparam _Scalar the scalar type, i.e., the type of the coefficients

  * This class represents a dual quaternion \f$\mathcal{Q}=\mathrm{r} + \epsilon \, \mathrm{d}\f$
  * with real part \f$\mathrm{r}\f$ and dual part \f$\mathrm{r}\f$, which are both a Quaternion.
  * It can also denoted vectorized as \f$\mathrm{vec}(\mathcal{Q})=[\boldsymbol{r}^\mathrm{T} \ \boldsymbol{d}^\mathrm{T}]^\mathrm{T}\f$.
  * 
  * A unit dual quaternion satifies \f$||\mathrm{r}|| = 1\f$ and \f$\mathrm{r}^{\ast} \, \mathrm{d} + \mathrm{d}^{\ast} \, \mathrm{r} = 0\f$.
  * Unit dual quaternions can be used to represent spatial transformations.
  *
  * \warning Operations interpreting the dual quaternion as transformation have undefined behavior if the dual quaternion is not normalized.
  */
template<typename _Scalar>
class DualQuaternion
{
 public:
  using QuaternionType = Quaternion<_Scalar>;
  using Vector3 = typename Quaternion<_Scalar>::Vector3;

  /** Default constructor leaving the dual quaternion uninitialized. */
  DualQuaternion() = default;

  /** Constructs and initializes a dual quaternion with the given <I>real</I> and <I>dual</I> part. */
  DualQuaternion(
    QuaternionType real,
    QuaternionType dual);

  /** Constructs and initializes a dual quaternion representing a transformation with the given 
    * <I>translation</I> and <I>rotation</I>.
    */
  template<class Derived>
  DualQuaternion(
    const Eigen::DenseBase<Derived> &translation,
    const QuaternionType &rotation);

  /** Constructs and initializes a dual quaternion with the given coefficients. */
  DualQuaternion(
    const _Scalar &real_w,
    const _Scalar &real_x,
    const _Scalar &real_y,
    const _Scalar &real_z,
    const _Scalar &dual_w,
    const _Scalar &dual_x,
    const _Scalar &dual_y,
    const _Scalar &dual_z);

  /** \returns a dual quaternion filled with zeros.
    */
  static DualQuaternion<_Scalar> Zero();

  /** \returns an identity dual quaternion \f$\mathcal{Q} = \{1 , \boldsymbol{0} \} + \epsilon \, \{ 0 , \boldsymbol{0} \} \f$.
    */
  static DualQuaternion<_Scalar> Identity();

  /** \returns a dual quaternion from an std vector with the order \f$[\boldsymbol{r}^\mathrm{T} \ \boldsymbol{d}^\mathrm{T}]^\mathrm{T}\f$. */
  static DualQuaternion<_Scalar> FromVector(const std::vector<_Scalar> &data);

  /** \returns a dual quaternion from an Eigen vector with the order \f$[\boldsymbol{r}^\mathrm{T} \ \boldsymbol{d}^\mathrm{T}]^\mathrm{T}\f$. */
  template<typename Derived>
  static DualQuaternion<_Scalar> FromVector(const Eigen::DenseBase<Derived> &data);

  /** \returns a dual quaternion from the <I>real</I> and <I>dual</I> part as std vector. */
  static DualQuaternion<_Scalar> FromVector(
    const std::vector<_Scalar> &real,
    const std::vector<_Scalar> &dual);

  /** \returns a dual quaternion from the <I>real</I> and <I>dual</I> part as Eigen vector. */
  template<typename DerivedReal, typename DerivedDual>
  static DualQuaternion<_Scalar> FromVector(
    const Eigen::DenseBase<DerivedReal> &real,
    const Eigen::DenseBase<DerivedDual> &dual);

  /** \returns the dual quaternion \f$\mathcal{P} = \{1, \boldsymbol{0}\} + \epsilon \, \{0, \boldsymbol{p}\}\f$
    * representing the <I>point</I> \f$\boldsymbol{p}\f$.
    */
  template<class Derived>
  static DualQuaternion<_Scalar> FromPoint(
    const Eigen::DenseBase<Derived> &point);

  /** Converts <CODE>*this</CODE> to std vector with the order \f$[\boldsymbol{r}^\mathrm{T} \ \boldsymbol{d}^\mathrm{T}]^\mathrm{T}\f$. */
  std::vector<_Scalar> toVector() const;

  /** Converts <CODE>*this</CODE> to Eigen vector with the order \f$[\boldsymbol{r}^\mathrm{T} \ \boldsymbol{d}^\mathrm{T}]^\mathrm{T}\f$. */
  Eigen::Matrix<_Scalar, kDualQuaternionDim, 1> toEigenVector() const;

  /** \returns a reference to the real part \f$\mathrm{r}\f$. */
  QuaternionType& real() { return real_; }

  /** \returns a const reference to the real part \f$\mathrm{r}\f$. */
  const QuaternionType& real() const { return real_; }

  /** \returns a reference to the dual part \f$\mathrm{d}\f$. */
  QuaternionType& dual() { return dual_; }

  /** \returns a const reference to the dual part \f$\mathrm{d}\f$. */
  const QuaternionType& dual() const { return dual_; }

  /** \returns the squared norm of all coefficients. */
  _Scalar squaredNorm() const;

  /** \returns the norm of all coefficients. */
  _Scalar norm() const;

  /** \returns the translation norm. */
  _Scalar translationNorm() const;

  /** \returns the rotation norm in <I>rad</I>. */
  _Scalar rotationNorm() const;

  /** Inplace variant of quatConjugate(). */
  DualQuaternion<_Scalar>& quatConjugate_();

  /** \returns the quaternion conjugate \f$\mathcal{Q}^{*} = \mathrm{r}^{*} + \epsilon \, \mathrm{d}^{*}\f$. */
  DualQuaternion<_Scalar> quatConjugate() const;

  /** Inplace variant of dualConjugate(). */
  DualQuaternion<_Scalar>& dualConjugate_();

  /** \returns the dual conjugate \f$\bar{\mathcal{Q}} = \mathrm{r} - \epsilon \, \mathrm{d}\f$. */
  DualQuaternion<_Scalar> dualConjugate() const;

  /** Inplace variant of combConjugate(). */
  DualQuaternion<_Scalar>& combConjugate_();

  /** \returns the combined conjugate \f$\bar{\mathcal{Q}}^{*} = \mathrm{r}^{*} - \epsilon \, \mathrm{d}^{*}\f$. */
  DualQuaternion<_Scalar> combConjugate() const;

  /** Inplace variant of inverse(). */
  DualQuaternion<_Scalar>& inverse_();

  /** \returns the inverted dual quaternion. */
  DualQuaternion<_Scalar> inverse() const;

  /** Inplace variant of normalized(). */
  DualQuaternion<_Scalar>& normalized_();

  /** \returns the normalized dual quaternion with real part norm \f$\boldsymbol{r}^\mathrm{T} \boldsymbol{r} = 1\f$ 
    * and \f$\boldsymbol{r}^\mathrm{T} \boldsymbol{d} = 0\f$.
    */
  DualQuaternion<_Scalar> normalized() const;

  /** \returns <CODE>true</CODE> if <CODE>*this</CODE> is approximately equal to <I>other</I>, within the precision determined by <I>eps</I>. */
  bool isEqual(const DualQuaternion<_Scalar> &other, const _Scalar &eps = kDefaultEps) const;

  /** Fills <CODE>*this</CODE> with zeros. */
  DualQuaternion<_Scalar>& setZero();

  /** \sa Identity() */
  DualQuaternion<_Scalar>& setIdentity();

  /** Converts the left dual quaternion of a dual quaternion multiplication to a 8x8 matrix
    * for representing the multiplication as matrix-vector product
    * \f$\text{vec}(\mathcal{Q}_a \mathcal{Q}_b) = \boldsymbol{Q}_a^{+} \text{vec}(\mathcal{Q}_b)\f$.
    * 
    * The following two lines yield identical results for <CODE>Quaterniond</CODE> <CODE>q1</CODE> and <CODE>q2</CODE>:
    * \code{.cpp}
    *   Eigen::Matrix<double, 8, 1> v1 = (q1 * q2).toEigenVector();
    *   Eigen::Matrix<double, 8, 1> v2 = q1.toPositiveMatrix() * q2.toEigenVector();
    * \endcode
    */
  Eigen::Matrix<_Scalar, kDualQuaternionDim, kDualQuaternionDim> toPositiveMatrix() const;

  /** Converts the right dual quaternion of a dual quaternion multiplication to a 8x8 matrix
    * for representing the multiplication as matrix-vector product
    * \f$\text{vec}(\mathcal{Q}_a \mathcal{Q}_b) = \boldsymbol{Q}_b^{-} \text{vec}(\mathcal{Q}_a)\f$.
    * 
    * The following two lines yield identical results for <CODE>DualQuaterniond</CODE> <CODE>q1</CODE> and <CODE>q2</CODE>:
    * \code{.cpp}
    *   Eigen::Matrix<double, 8, 1> v1 = (q1 * q2).toEigenVector();
    *   Eigen::Matrix<double, 8, 1> v2 = q2.toNegativeMatrix() * q1.toEigenVector();
    * \endcode
    */
  Eigen::Matrix<_Scalar, kDualQuaternionDim, kDualQuaternionDim> toNegativeMatrix() const;

  /** \returns the translation quaternion \f$\mathrm{t} = 2 \mathrm{d} \mathrm{r}^{*}\f$. */
  QuaternionType getTranslationQuaternion() const;

  /** \returns the vector part of \f$\mathrm{t}\f$ from getTranslationQuaternion(). */
  Vector3 getTranslationVector() const;

  /** Transforms <I>point</I> \f$\boldsymbol{p}\f$ using \f$\mathcal{Q} \mathcal{P} \bar{\mathcal{Q}}^{*}\f$
    * with \f$\mathcal{P}\f$ from FromPoint().
    */
  template<typename Derived>
  Vector3 transformPoint(const Eigen::MatrixBase<Derived> &point) const;

  /** \returns a description of <CODE>*this</CODE>. */
  std::string desc() const { return streamToString(*this); }
  
  /** Coefficient-wise multiplication of <I>*this</I> and <I>v</I>. */
  DualQuaternion<_Scalar> operator*(const _Scalar &v) const;

  /** \sa operator*(const _Scalar&) const */
  DualQuaternion<_Scalar>& operator*=(const _Scalar &v);

  /** \sa operator*(const _Scalar&) const */
  friend DualQuaternion<_Scalar> operator*(const _Scalar &v, const DualQuaternion<_Scalar> &dq) { return dq * v; }

  /** Dual quaternion product of <I>*this</I> and <I>other</I>. */
  DualQuaternion<_Scalar> operator*(const DualQuaternion<_Scalar> &other) const;

  /** \sa operator*(const DualQuaternion<_Scalar>&) const */
  DualQuaternion<_Scalar>& operator*=(const DualQuaternion<_Scalar> &other);

  /** Coefficient-wise division of <I>*this</I> by <I>v</I>. */
  DualQuaternion<_Scalar> operator/(const _Scalar &v) const;

  /** \sa operator/(const _Scalar&) const */
  DualQuaternion<_Scalar>& operator/=(const _Scalar &v);

  /** Coefficient-wise summation of <I>*this</I> and <I>other</I>. */
  DualQuaternion<_Scalar> operator+(const DualQuaternion<_Scalar> &other) const;

  /** \sa operator+(const DualQuaternion<_Scalar>&) const */
  DualQuaternion<_Scalar>& operator+=(const DualQuaternion<_Scalar> &other);
  
  /** Coefficient-wise negative of <I>*this</I>. */
  DualQuaternion<_Scalar> operator-() const;

  /** Coefficient-wise subtraction of <I>*this</I> and <I>other</I>. */
  DualQuaternion<_Scalar> operator-(const DualQuaternion<_Scalar> &other) const;

  /** \sa operator-(const DualQuaternion<_Scalar>&) const */
  DualQuaternion<_Scalar>& operator-=(const DualQuaternion<_Scalar> &other);

  /** Inserts a description of <CODE>*this</CODE>. */
  template<typename _OtherScalar>
  friend std::ostream& operator<<(std::ostream& os, const DualQuaternion<_OtherScalar>& dq);

 private:
  /** \returns the dual part \f$\mathrm{d}\f$ calculated from <I>translation</I> vector \f$\boldsymbol{t}\f$ and 
    * <I>rotation</I> quaternion \f$\mathrm{r}\f$. 
    * \f[ \mathrm{d} = \frac{1}{2} \mathrm{t} \mathrm{r} \quad \text{with} \ \mathrm{t} = \{ 0, \boldsymbol{t} \} \f]
    */
  template<class Derived>
  static QuaternionType createDualPart(
    const Eigen::DenseBase<Derived> &translation,
    const QuaternionType &rotation);

  QuaternionType real_;  ///< Real part
  QuaternionType dual_;  ///< Dual part
};


template<typename _Scalar>
DualQuaternion<_Scalar>::DualQuaternion(
  QuaternionType real,
  QuaternionType dual)
  : real_(std::move(real)), dual_(std::move(dual))
{
}

template<typename _Scalar>
template<typename Derived>
DualQuaternion<_Scalar>::DualQuaternion(
  const Eigen::DenseBase<Derived> &translation,
  const QuaternionType &rotation)
  : real_(rotation)
  , dual_(createDualPart(translation, rotation))
{
}

template<typename _Scalar>
DualQuaternion<_Scalar>::DualQuaternion(
  const _Scalar &real_w,
  const _Scalar &real_x,
  const _Scalar &real_y,
  const _Scalar &real_z,
  const _Scalar &dual_w,
  const _Scalar &dual_x,
  const _Scalar &dual_y,
  const _Scalar &dual_z
)
  : real_(real_w, real_x, real_y, real_z)
  , dual_(dual_w, dual_x, dual_y, dual_z)
{
}

template<typename _Scalar>
DualQuaternion<_Scalar> DualQuaternion<_Scalar>::Zero()
{
  return DualQuaternion<_Scalar>(
    QuaternionType::Zero(),
    QuaternionType::Zero()
  );
}

template<typename _Scalar>
DualQuaternion<_Scalar> DualQuaternion<_Scalar>::Identity()
{
  return DualQuaternion<_Scalar>(
    QuaternionType::Identity(),
    QuaternionType::Zero()
  );
}

template<typename _Scalar>
DualQuaternion<_Scalar> DualQuaternion<_Scalar>::FromVector(const std::vector<_Scalar> &data)
{
  if (data.size() != kDualQuaternionDim)
  {
    throw std::runtime_error("Invalid vector size");
  }
  return DualQuaternion<_Scalar>(
    QuaternionType(data[0], data[1], data[2], data[3]),
    QuaternionType(data[4], data[5], data[6], data[7])  // NOLINT(*-magic-numbers)
  );
}

template<typename _Scalar>
template<typename Derived>
DualQuaternion<_Scalar> DualQuaternion<_Scalar>::FromVector(const Eigen::DenseBase<Derived> &data)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, kDualQuaternionDim);
  return DualQuaternion<_Scalar>(
    QuaternionType(data(0), data(1), data(2), data(3)),
    QuaternionType(data(4), data(5), data(6), data(7))  // NOLINT(*-magic-numbers)
  );
}

template<typename _Scalar>
DualQuaternion<_Scalar> DualQuaternion<_Scalar>::FromVector(
  const std::vector<_Scalar> &real,
  const std::vector<_Scalar> &dual
)
{
  if (real.size() != kQuaternionDim || dual.size() != kQuaternionDim)
  {
    throw std::runtime_error("Invalid vector size");
  }
  return DualQuaternion<_Scalar>(
    QuaternionType(real[0], real[1], real[2], real[3]),
    QuaternionType(dual[0], dual[1], dual[2], dual[3])
  );
}

template<typename _Scalar>
template<typename DerivedReal, typename DerivedDual>
DualQuaternion<_Scalar> DualQuaternion<_Scalar>::FromVector(
  const Eigen::DenseBase<DerivedReal> &real,
  const Eigen::DenseBase<DerivedDual> &dual
)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedReal, kQuaternionDim);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedDual, kQuaternionDim);
  return DualQuaternion<_Scalar>(
    QuaternionType(real(0), real(1), real(2), real(3)),
    QuaternionType(dual(0), dual(1), dual(2), dual(3))
  );
}

template<typename _Scalar>
template<typename Derived>
DualQuaternion<_Scalar> DualQuaternion<_Scalar>::FromPoint(
  const Eigen::DenseBase<Derived> &point
)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  return DualQuaternion<_Scalar>(
    QuaternionType(1.0, 0.0, 0.0, 0.0),
    QuaternionType(0.0, point(0), point(1), point(2))
  );
}

template<typename _Scalar>
std::vector<_Scalar> DualQuaternion<_Scalar>::toVector() const
{
  std::vector<_Scalar> data = {
    real_.w(), real_.x(), real_.y(), real_.z(),
    dual_.w(), dual_.x(), dual_.y(), dual_.z()
  };
  return data;
}

template<typename _Scalar>
Eigen::Matrix<_Scalar, kDualQuaternionDim, 1> DualQuaternion<_Scalar>::toEigenVector() const
{
  Eigen::Matrix<_Scalar, kDualQuaternionDim, 1> data;
  data << real_.toEigenVector(), dual_.toEigenVector();
  return data;
}

template<typename _Scalar>
_Scalar DualQuaternion<_Scalar>::squaredNorm() const
{
  return real_.squaredNorm() + dual_.squaredNorm();
}

template<typename _Scalar>
_Scalar DualQuaternion<_Scalar>::norm() const
{
  return sqrt(squaredNorm());
}

template<typename _Scalar>
_Scalar DualQuaternion<_Scalar>::translationNorm() const
{
  return 2.0 * dual_.norm();  // NOLINT(*-magic-numbers)
}

template<typename _Scalar>
_Scalar DualQuaternion<_Scalar>::rotationNorm() const
{
  return real_.rotationNorm();
}

template<typename _Scalar>
DualQuaternion<_Scalar>& DualQuaternion<_Scalar>::quatConjugate_()
{
  real_.conjugate_();
  dual_.conjugate_();
  return *this;
}

template<typename _Scalar>
DualQuaternion<_Scalar> DualQuaternion<_Scalar>::quatConjugate() const
{
  return DualQuaternion<_Scalar>(real_.conjugate(), dual_.conjugate());
}

template<typename _Scalar>
DualQuaternion<_Scalar>& DualQuaternion<_Scalar>::dualConjugate_()
{
  dual_ *= -1;
  return *this;
}

template<typename _Scalar>
DualQuaternion<_Scalar> DualQuaternion<_Scalar>::dualConjugate() const
{
  return DualQuaternion<_Scalar>(real_, -dual_);
}

template<typename _Scalar>
DualQuaternion<_Scalar>& DualQuaternion<_Scalar>::combConjugate_()
{
  real_.conjugate_();
  dual_.conjugate_();
  dual_ *= -1;
  return *this;
}

template<typename _Scalar>
DualQuaternion<_Scalar> DualQuaternion<_Scalar>::combConjugate() const
{
  return DualQuaternion<_Scalar>(real_.conjugate(), -dual_.conjugate());
}

template<typename _Scalar>
DualQuaternion<_Scalar>& DualQuaternion<_Scalar>::inverse_()
{
  real_.inverse_();
  dual_ = - real_ * dual_ * real_;
  return *this;
}

template<typename _Scalar>
DualQuaternion<_Scalar> DualQuaternion<_Scalar>::inverse() const
{
  QuaternionType real_inv = real_.inverse();
  QuaternionType dual_inv = - real_inv * dual_ * real_inv;
  return DualQuaternion<_Scalar>(real_inv, dual_inv);
}

template<typename _Scalar>
DualQuaternion<_Scalar>& DualQuaternion<_Scalar>::normalized_()
{
  _Scalar factor = getAxisNormalizationFactorInverse(real_.toEigenVector(), true);
  if (std::abs(factor) < kDefaultEps)
  {
    real_.setIdentity();
    dual_.setZero();
  }
  else
  {
    // t is divided by norm^2 since the old r is used to recover t
    Eigen::Matrix<_Scalar, 3, 1> new_translation = getTranslationVector() / (factor * factor);
    real_ /= factor;
    dual_ = createDualPart(new_translation, real_);
  }
  return *this;
}

template<typename _Scalar>
DualQuaternion<_Scalar> DualQuaternion<_Scalar>::normalized() const
{
  return DualQuaternion<_Scalar>(*this).normalized_();
}

template<typename _Scalar>
bool DualQuaternion<_Scalar>::isEqual(const DualQuaternion<_Scalar> &other, const _Scalar &eps) const
{
  return real_.isEqual(other.real(), eps) && dual_.isEqual(other.dual(), eps);
}

template<typename _Scalar>
DualQuaternion<_Scalar>& DualQuaternion<_Scalar>::setZero()
{
  real_.setZero();
  dual_.setZero();
  return *this;
}

template<typename _Scalar>
DualQuaternion<_Scalar>& DualQuaternion<_Scalar>::setIdentity()
{
  real_.setIdentity();
  dual_.setZero();
  return *this;
}

template<typename _Scalar>
Eigen::Matrix<_Scalar, kDualQuaternionDim, kDualQuaternionDim> DualQuaternion<_Scalar>::toPositiveMatrix() const
{
  Eigen::Matrix<_Scalar, kDualQuaternionDim, kDualQuaternionDim> m;
  Eigen::Matrix<_Scalar, kQuaternionDim, kQuaternionDim> m_real = real_.toPositiveMatrix();
  Eigen::Matrix<_Scalar, kQuaternionDim, kQuaternionDim> m_dual = dual_.toPositiveMatrix();
  m << m_real, Eigen::Matrix<_Scalar, kQuaternionDim, kQuaternionDim>::Zero(),
      m_dual, m_real;
  return m;
}

template<typename _Scalar>
Eigen::Matrix<_Scalar, kDualQuaternionDim, kDualQuaternionDim> DualQuaternion<_Scalar>::toNegativeMatrix() const
{
  Eigen::Matrix<_Scalar, kDualQuaternionDim, kDualQuaternionDim> m;
  Eigen::Matrix<_Scalar, kQuaternionDim, kQuaternionDim> m_real = real_.toNegativeMatrix();
  Eigen::Matrix<_Scalar, kQuaternionDim, kQuaternionDim> m_dual = dual_.toNegativeMatrix();
  m << m_real, Eigen::Matrix<_Scalar, kQuaternionDim, kQuaternionDim>::Zero(),
      m_dual, m_real;
  return m;
}

template<typename _Scalar>
typename DualQuaternion<_Scalar>::QuaternionType DualQuaternion<_Scalar>::getTranslationQuaternion() const
{
  return 2 * dual_ * real_.conjugate();
}

template<typename _Scalar>
typename DualQuaternion<_Scalar>::Vector3 DualQuaternion<_Scalar>::getTranslationVector() const
{
  return getTranslationQuaternion().vec();
}

template<typename _Scalar>
template<typename Derived>
typename DualQuaternion<_Scalar>::Vector3 DualQuaternion<_Scalar>::transformPoint(const Eigen::MatrixBase<Derived> &point) const
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 1);
  return (*this * DualQuaternion::FromPoint(point) * this->combConjugate()).dual().vec();
}

template<typename _Scalar>
DualQuaternion<_Scalar> DualQuaternion<_Scalar>::operator*(const _Scalar &v) const
{
  return DualQuaternion<_Scalar>(real_ * v, dual_ * v);
}

template<typename _Scalar>
DualQuaternion<_Scalar>& DualQuaternion<_Scalar>::operator*=(const _Scalar &v)
{
  real_ *= v;
  dual_ *= v;
  return *this;
}

template<typename _Scalar>
DualQuaternion<_Scalar> DualQuaternion<_Scalar>::operator*(const DualQuaternion<_Scalar> &other) const
{
  QuaternionType new_real = real_ * other.real_;
  QuaternionType new_dual = (real_ * other.dual_) + (dual_ * other.real_);
  return DualQuaternion<_Scalar>(new_real, new_dual);
}

template<typename _Scalar>
DualQuaternion<_Scalar>& DualQuaternion<_Scalar>::operator*=(const DualQuaternion<_Scalar> &other)
{
  QuaternionType tmp_real = real_ * other.real_;
  dual_ = (real_ * other.dual_) + (dual_ * other.real_);
  real_ = tmp_real;
  return *this;
}

template<typename _Scalar>
DualQuaternion<_Scalar> DualQuaternion<_Scalar>::operator/(const _Scalar &v) const
{
  return DualQuaternion<_Scalar>(real_ / v, dual_ / v);
}

template<typename _Scalar>
DualQuaternion<_Scalar>& DualQuaternion<_Scalar>::operator/=(const _Scalar &v)
{
  real_ /= v;
  dual_ /= v;
  return *this;
}

template<typename _Scalar>
DualQuaternion<_Scalar> DualQuaternion<_Scalar>::operator+(const DualQuaternion<_Scalar> &other) const
{
  return DualQuaternion<_Scalar>(real_ + other.real_, dual_ + other.dual_);
}

template<typename _Scalar>
DualQuaternion<_Scalar>& DualQuaternion<_Scalar>::operator+=(const DualQuaternion<_Scalar> &other)
{
  real_ += other.real_;
  dual_ += other.dual_;
  return *this;
}

template<typename _Scalar>
DualQuaternion<_Scalar> DualQuaternion<_Scalar>::operator-() const
{
  return DualQuaternion<_Scalar>(-real_, -dual_);
}

template<typename _Scalar>
DualQuaternion<_Scalar> DualQuaternion<_Scalar>::operator-(const DualQuaternion<_Scalar> &other) const
{
  return DualQuaternion<_Scalar>(real_ - other.real_, dual_ - other.dual_);
}

template<typename _Scalar>
DualQuaternion<_Scalar>& DualQuaternion<_Scalar>::operator-=(const DualQuaternion<_Scalar> &other)
{
  real_ -= other.real_;
  dual_ -= other.dual_;
  return *this;
}

template<typename _Scalar>
std::ostream& operator<<(std::ostream &os, const DualQuaternion<_Scalar> &dq)
{
  return os << "DualQuaternion(" << 
    "real=[" << dq.real_.w() << " " << dq.real_.vec().transpose() << "], " <<
    "dual=[" << dq.dual_.w() << " " << dq.dual_.vec().transpose() << "])";
}

template<typename _Scalar>
template<typename Derived>
Quaternion<_Scalar> DualQuaternion<_Scalar>::createDualPart(
  const Eigen::DenseBase<Derived>& translation,
  const QuaternionType& rotation)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  QuaternionType translation_quat = QuaternionType::Zero();
  translation_quat.vec() = translation;
  return 0.5 * translation_quat * rotation;  // NOLINT(*-magic-numbers)
}

using DualQuaternionf = DualQuaternion<float>;
using DualQuaterniond = DualQuaternion<double>;

} // namespace motion3d
