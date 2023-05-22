#pragma once

#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include <motion3d/common/dual_quaternion.hpp>
#include <motion3d/transforms/base.hpp>
#include <motion3d/utils/conversion.hpp>
#include <motion3d/utils/templating.hpp>

namespace motion3d
{

/** \brief Transformation consisting of a unit DualQuaternion \f$\mathcal{Q}=\mathrm{r} + \epsilon \, \mathrm{d}\f$.
  *
  * For representing a valid spatial transformation, the dual quaternion must be unit, i.e., 
  * \f$||\mathrm{r}|| = 1\f$ and \f$\mathrm{r}^{\ast} \, \mathrm{d} + \mathrm{d}^{\ast} \, \mathrm{r} = 0\f$.
  * An unsafe transform means that the validity check, usually performed at each operation, is skipped.
  *
  * Operations:
  * \li Concatenation: \f$\mathcal{Q}_{ac} = \mathcal{Q}_{ab} \mathcal{Q}_{bc}\f$
  * \li Inversion: \f$V^{-1} \equiv \mathcal{Q}^{\ast}\f$
  * \li Point transformation: \f$(1 + \epsilon \,  \{ 0, \boldsymbol{p}_a \} ) =
  *     \mathcal{Q}_{ab} ( 1 + \epsilon \,  \{ 0, \boldsymbol{p}_b \} ) \ \bar{\mathcal{Q}}_{ab}^{*}\f$
  */
class DualQuaternionTransform : public TransformBase<DualQuaternionTransform>
{
 public:
  DEFINE_POINTERS(DualQuaternionTransform);

  using TransformBase<DualQuaternionTransform>::isValid;

  /** Constructs and initializes a DualQuaternionTransform as identity transform. */
  DualQuaternionTransform();

  /** Constructs and initializes a DualQuaternionTransform from dual quaternion <I>dq</I>. */
  explicit DualQuaternionTransform(
    DualQuaternion<double> dq,
    bool unsafe = false);

  /** Constructs and initializes a DualQuaternionTransform from <I>real</I> and <I>dual</I> part. */
  DualQuaternionTransform(
    const Quaternion<double> &real,
    const Quaternion<double> &dual,
    bool unsafe = false);

  /** Constructs and initializes a DualQuaternionTransform from <I>real</I> and <I>dual</I> part Eigen matrices. */
  template<typename DerivedReal, typename DerivedDual>
  DualQuaternionTransform(
    const Eigen::DenseBase<DerivedReal> &real,
    const Eigen::DenseBase<DerivedDual> &dual,
    bool unsafe = false);

  /** Constructs and initializes a DualQuaternionTransform from <I>binary</I> data.
    * \throws InvalidTypeSizeException for an invalid binary size.
    */
  explicit DualQuaternionTransform(const std::vector<std::uint8_t> &binary, bool unsafe = false);

  /** Constructs and initializes a DualQuaternionTransform from <I>vector</I> data.
    * \throws InvalidTypeSizeException for an invalid vector size.
    */
  explicit DualQuaternionTransform(const std::vector<double> &vector, bool unsafe = false);

  /** Constructs and initializes a DualQuaternionTransform from an Eigen vector <I>matrix</I>.
    * \throws InvalidTypeSizeException for an invalid matrix size.
    */
  template<typename Derived>
  explicit DualQuaternionTransform(const Eigen::DenseBase<Derived> &matrix, bool unsafe = false);

  /** Create a copy of <CODE>*this</CODE>. */
  DualQuaternionTransform copy() const { return {*this}; }

  /** Set <CODE>*this</CODE> to an identity transform. */
  DualQuaternionTransform& setIdentity();

  /** Converts <CODE>*this</CODE> to an std vector. */
  std::vector<double> toVector() const override;

  /** Converts <CODE>*this</CODE> to an Eigen vector. */
  Eigen::Matrix<double, Eigen::Dynamic, 1> toEigenVector() const override;

  /** Converts <CODE>*this</CODE> to transform type <I>T</I>. */
  template<class T>
  T asType() const;

  /** Checks if <CODE>*this</CODE> is valid w.r.t. the threshold <I>eps</I>. */ 
  bool isValid(double eps) const override;

  /** Checks if dual quaternion with <I>real</I> and <I>dual</I> parts is unit w.r.t. the threshold <I>eps</I>. */
  static bool isUnitDualQuaternion(const Quaternion<double> &real, const Quaternion<double> &dual, double eps);

  /** Inplace variant of inverse(). */
  DualQuaternionTransform& inverse_();

  /** \returns the inverse transform to <CODE>*this</CODE>. */
  DualQuaternionTransform inverse() const;

  /** Inplace variant of normalized(). */
  DualQuaternionTransform& normalized_();

  /** Normalizes <CODE>*this</CODE> to a valid dual quaternion transform and sets <CODE>unsafe</CODE> to <CODE>false</CODE>. */
  DualQuaternionTransform normalized() const;

  /** Inplace variant of scaleTranslation(). */
  DualQuaternionTransform& scaleTranslation_(double factor);

  /** Scales the translation by <I>factor</I>. */
  DualQuaternionTransform scaleTranslation(double factor) const;

  /** Inplace variant of applyPre(). */
  template<class T>
  DualQuaternionTransform& applyPre_(const T &other);

  /** Applies <I>other</I> transform \f$T_\mathrm{o}\f$ before <CODE>*this</CODE> transform \f$T\f$:
    * \f[T_\mathrm{o} \circ T\f]
    */
  template<class T>
  DualQuaternionTransform applyPre(const T &other) const;

  /** Inplace variant of applyPost(). */
  template<class T>
  DualQuaternionTransform& applyPost_(const T &other);

  /** Applies <I>other</I> transform \f$T_\mathrm{o}\f$ after <CODE>*this</CODE> transform \f$T\f$:
    * \f[T \circ T_\mathrm{o}\f]
    */
  template<class T>
  DualQuaternionTransform applyPost(const T &other) const;

  /** Applies <CODE>*this</CODE> on 3D <I>point</I>. */
  template<typename Derived>
  Eigen::Matrix<double, 3, 1> transformPoint(const Eigen::MatrixBase<Derived> &point) const;

  /** Applies <CODE>*this</CODE> on 3D point <I>cloud</I>. */
  template<typename Derived>
  Eigen::Matrix<double, 3, Derived::ColsAtCompileTime> transformCloud(const Eigen::MatrixBase<Derived> &cloud) const;

  /** \returns the rotation norm in <I>rad</I>. */
  double rotationNorm() const override;

  /** \returns the translation norm. */
  double translationNorm() const override;

  /** \returns the dual quaternion. */
  const DualQuaternion<double>& getDualQuaternion() const { return dq_; }

  /** \returns the real part. */
  const Quaternion<double>& getReal() const { return dq_.real(); }

  /** \returns the dual part. */
  const Quaternion<double>& getDual() const { return dq_.dual(); }

  /** Sets the dual quaternion. */
  DualQuaternionTransform& setDualQuaternion(const DualQuaternion<double> &dq, bool unsafe = false);

  /** Sets the real part. */
  DualQuaternionTransform& setReal(const Quaternion<double> &real, bool unsafe = false);

  /** Sets the dual part. */
  DualQuaternionTransform& setDual(const Quaternion<double> &dual, bool unsafe = false);

  /** \returns a description of <CODE>*this</CODE>. */
  std::string desc() const override { return streamToString(*this); }

  /** Inserts a description of <CODE>*this</CODE>. */
  std::ostream& stream(std::ostream& os) const override;

  /** Applies <I>other</I> after <CODE>*this</CODE>. */
  template<class T>
  DualQuaternionTransform& operator*=(const T &other);

  /** Applies the inverse of <I>other</I> after <CODE>*this</CODE>. */
  template<class T>
  DualQuaternionTransform& operator/=(const T &other);

 private:
  DualQuaternion<double> dq_;  ///< Dual quaternion
};

DualQuaternionTransform::DualQuaternionTransform()
  : TransformBase(false)
  , dq_(DualQuaternion<double>::Identity())
{
}

DualQuaternionTransform::DualQuaternionTransform(
  DualQuaternion<double> dq,
  const bool unsafe)
  : TransformBase(unsafe)
  , dq_(std::move(dq))
{
  checkValid();
}

DualQuaternionTransform::DualQuaternionTransform(
  const Quaternion<double> &real, 
  const Quaternion<double> &dual,
  const bool unsafe)
  : TransformBase(unsafe)
  , dq_(real, dual)
{
  checkValid();
}

template<typename DerivedReal, typename DerivedDual>
DualQuaternionTransform::DualQuaternionTransform(
  const Eigen::DenseBase<DerivedReal> &real, 
  const Eigen::DenseBase<DerivedDual> &dual,
  const bool unsafe)
  : TransformBase(unsafe)
  , dq_(DualQuaternion<double>::FromVector(real, dual))
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedReal, kQuaternionDim);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedDual, kQuaternionDim);
  checkValid();
}

DualQuaternionTransform::DualQuaternionTransform(const std::vector<std::uint8_t> &binary, const bool unsafe)
  : DualQuaternionTransform(convertVector<std::uint8_t, double>(binary), unsafe)
{
}

DualQuaternionTransform::DualQuaternionTransform(const std::vector<double> &vector, const bool unsafe)
  : TransformBase(unsafe)
{
  if (!unsafe_ && vector.size() != kVectorSize)
  {
    throw InvalidTypeSizeException();
  }
  // NOLINTBEGIN(*-magic-numbers)
  dq_.real().w() = vector.at(0);
  dq_.real().x() = vector.at(1);
  dq_.real().y() = vector.at(2);
  dq_.real().z() = vector.at(3);
  dq_.dual().w() = vector.at(4);
  dq_.dual().x() = vector.at(5);
  dq_.dual().y() = vector.at(6);
  dq_.dual().z() = vector.at(7);
  // NOLINTEND(*-magic-numbers)
  checkValid();
}

template<typename Derived>
DualQuaternionTransform::DualQuaternionTransform(const Eigen::DenseBase<Derived> &matrix, const bool unsafe)
  : TransformBase(unsafe)
{
  // check size
  if (!checkEigenVectorSize<kVectorSize>(matrix))
  {
    throw InvalidTypeSizeException();
  }

  // copy values
  // NOLINTBEGIN(*-magic-numbers)
  dq_.real().w() = matrix(0);
  dq_.real().x() = matrix(1);
  dq_.real().y() = matrix(2);
  dq_.real().z() = matrix(3);
  dq_.dual().w() = matrix(4);
  dq_.dual().x() = matrix(5);
  dq_.dual().y() = matrix(6);
  dq_.dual().z() = matrix(7);
  // NOLINTEND(*-magic-numbers)
  checkValid();
}

DualQuaternionTransform& DualQuaternionTransform::setIdentity()
{
  // set to identity
  dq_.setIdentity();

  // mark as safe
  unsafe_ = false;
  return *this;
}

std::vector<double> DualQuaternionTransform::toVector() const
{
  return {dq_.real().w(), dq_.real().x(), dq_.real().y(), dq_.real().z(),
          dq_.dual().w(), dq_.dual().x(), dq_.dual().y(), dq_.dual().z()};
}

Eigen::Matrix<double, Eigen::Dynamic, 1> DualQuaternionTransform::toEigenVector() const
{
  return dq_.toEigenVector();
}

bool DualQuaternionTransform::isValid(const double eps) const
{
  return isUnitDualQuaternion(dq_.real(), dq_.dual(), eps);
}

bool DualQuaternionTransform::isUnitDualQuaternion(
  const Quaternion<double> &real,
  const Quaternion<double> &dual,
  const double eps)
{
  bool cond1 = std::abs(real.norm() - 1.0) <= eps;
  bool cond2 = std::abs(((real.conjugate() * dual) + (dual.conjugate() * real)).norm()) <= eps;
  return cond1 && cond2;
}

DualQuaternionTransform& DualQuaternionTransform::normalized_()
{
  dq_.normalized_();
  unsafe_ = false;
  return *this;
}

DualQuaternionTransform DualQuaternionTransform::normalized() const
{
  return DualQuaternionTransform(dq_.normalized(), false);
}

DualQuaternionTransform& DualQuaternionTransform::scaleTranslation_(double factor)
{
  dq_.dual() *= factor;
  return *this;
}

DualQuaternionTransform DualQuaternionTransform::scaleTranslation(double factor) const
{
  return {dq_.real(), dq_.dual() * factor, unsafe_};
}

DualQuaternionTransform& DualQuaternionTransform::setDualQuaternion(const DualQuaternion<double> &dq, const bool unsafe)
{
  // unsafe
  bool new_unsafe = unsafe_ || unsafe;

  // set
  if (new_unsafe || isUnitDualQuaternion(dq.real(), dq.dual(), kDefaultEps))
  {
    dq_ = dq;
  }
  else
  {
    throw InvalidTransformException("Input dual quaternion is not unit");
  }

  // return
  unsafe_ = new_unsafe;
  return *this;
}

DualQuaternionTransform& DualQuaternionTransform::setReal(const Quaternion<double> &real, const bool unsafe)
{
  // unsafe
  bool new_unsafe = unsafe_ || unsafe;

  // set
  if (new_unsafe || isUnitDualQuaternion(real, dq_.dual(), kDefaultEps))
  {
    dq_.real() = real;
  }
  else
  {
    throw InvalidTransformException("Input dual quaternion is not unit");
  }

  // return
  unsafe_ = new_unsafe;
  return *this;
}

DualQuaternionTransform& DualQuaternionTransform::setDual(const Quaternion<double> &dual, const bool unsafe)
{
  // unsafe
  bool new_unsafe = unsafe_ || unsafe;

  // set
  if (new_unsafe || isUnitDualQuaternion(dq_.real(), dual, kDefaultEps))
  {
    dq_.dual() = dual;
  }
  else
  {
    throw InvalidTransformException("Input dual quaternion is not unit");
  }

  // return
  unsafe_ = new_unsafe;
  return *this;
}

std::ostream& DualQuaternionTransform::stream(std::ostream& os) const
{
  return os << "DualQuaternionTransform(" <<
            "real=[" << dq_.real().w() << " " << dq_.real().vec().transpose() << "], " <<
            "dual=[" << dq_.dual().w() << " " << dq_.dual().vec().transpose() << "])";
}

} // namespace motion3d
