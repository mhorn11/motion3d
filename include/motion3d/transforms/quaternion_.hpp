#pragma once

#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include <motion3d/common/quaternion.hpp>
#include <motion3d/transforms/base.hpp>
#include <motion3d/utils/conversion.hpp>
#include <motion3d/utils/templating.hpp>

namespace motion3d
{

/** \brief Quaternion transformation consisting of a translation vector \f$\boldsymbol{t}\f$ and a rotation Quaternion \f$\mathrm{q}\f$.
  * 
  * For representing a valid spatial transformation, the quaternion must be unit, i.e., \f$||\mathrm{q}|| = 1\f$.
  * An unsafe transform means that the validity check, usually performed at each operation, is skipped.
  * 
  * Operations:
  * \li Concatenation: \f$\mathrm{q}_{ac} = \mathrm{q}_{ab} \mathrm{q}_{bc}\f$ and
  *                    \f$\{ 0, \boldsymbol{t}_{ac} \} = \{ 0, \boldsymbol{t}_{ab} \} + \mathrm{q}_{ab} \, \{ 0, \boldsymbol{t}_{bc} \} \, \mathrm{q}_{ab}^{\ast}\f$
  * \li Inversion: \f$V^{-1}: \mathrm{q}_{\mathrm{inv}} = \mathrm{q}^{\ast}\f$ and 
                   \f$\{ 0, \boldsymbol{t}_{\mathrm{inv}} \} = \mathrm{q}^{\ast} \, \{ 0, \boldsymbol{t} \} \, \mathrm{q}\f$
  * \li Point transformation: \f$\{ 0, \boldsymbol{p}_a \} =  \mathrm{q}_{ab} \, \{ 0, \boldsymbol{p}_b \} \, \mathrm{q}_{ab}^{\ast} + \{ 0, \boldsymbol{t} \} \f$
  */
class QuaternionTransform : public TransformBase<QuaternionTransform>
{
 public:
  DEFINE_POINTERS(QuaternionTransform);

  using TransformBase<QuaternionTransform>::isValid;

  /** Constructs and initializes a QuaternionTransform as identity transform. */
  QuaternionTransform();

  /** Constructs and initializes a QuaternionTransform from a <I>translation</I> vector and a rotation <I>quaternion</I>. */
  template<typename Derived>
  QuaternionTransform(
    const Eigen::DenseBase<Derived> &translation,
    Quaternion<double> quaternion,
    bool unsafe = false);

  /** Constructs and initializes a QuaternionTransform from a <I>translation</I> vector and a rotation <I>quaternion</I>. */
  template<typename Derived>
  QuaternionTransform(
    const Eigen::DenseBase<Derived> &translation,
    const Eigen::Quaternion<double> &quaternion,
    bool unsafe = false);

  /** Constructs and initializes a QuaternionTransform from a <I>translation</I> vector and a rotation <I>quaternion</I> matrix. */
  template<typename DerivedTrans, typename DerivedQuat>
  QuaternionTransform(
    const Eigen::DenseBase<DerivedTrans> &translation,
    const Eigen::DenseBase<DerivedQuat> &quaternion,
    bool unsafe = false);

  /** Constructs and initializes a QuaternionTransform from <I>binary</I> data.
    * \throws InvalidTypeSizeException for an invalid binary size.
    */
  explicit QuaternionTransform(const std::vector<std::uint8_t> &binary, bool unsafe = false);

  /** Constructs and initializes a QuaternionTransform from <I>vector</I> data.
    * \throws InvalidTypeSizeException for an invalid vector size.
    */
  explicit QuaternionTransform(const std::vector<double> &vector, bool unsafe = false);

  /** Constructs and initializes a QuaternionTransform from an Eigen vector <I>matrix</I>.
    * \throws InvalidTypeSizeException for an invalid matrix size.
    */
  template<typename Derived>
  explicit QuaternionTransform(const Eigen::DenseBase<Derived> &matrix, bool unsafe = false);

  /** Create a copy of <CODE>*this</CODE>. */
  QuaternionTransform copy() const { return {*this}; }

  /** Set <CODE>*this</CODE> to an identity transform. */
  QuaternionTransform& setIdentity();

  /** Converts <CODE>*this</CODE> to an std vector. */
  std::vector<double> toVector() const override;

  /** Converts <CODE>*this</CODE> to an Eigen vector. */
  Eigen::Matrix<double, Eigen::Dynamic, 1> toEigenVector() const override;

  /** Converts <CODE>*this</CODE> to transform type <I>T</I>. */
  template<class T>
  T asType() const;

  /** Checks if <CODE>*this</CODE> is valid w.r.t. the threshold <I>eps</I>. */ 
  bool isValid(double eps) const override;

  /** Checks if <I>quaternion</I> is unit w.r.t. the threshold <I>eps</I>. */
  static bool isUnitQuaternion(const Quaternion<double> &quaternion, double eps);

  /** Inplace variant of inverse(). */
  QuaternionTransform& inverse_();

  /** \returns the inverse transform to <CODE>*this</CODE>. */
  QuaternionTransform inverse() const;

  /** Inplace variant of normalized(). */
  QuaternionTransform& normalized_();

  /** Normalizes <CODE>*this</CODE> to a valid quaternion transform and sets <CODE>unsafe</CODE> to <CODE>false</CODE>. */
  QuaternionTransform normalized() const;

  /** Inplace variant of scaleTranslation(). */
  QuaternionTransform& scaleTranslation_(double factor);

  /** Scales the translation by <I>factor</I>. */
  QuaternionTransform scaleTranslation(double factor) const;

  /** Inplace variant of applyPre(). */
  template<class T>
  QuaternionTransform& applyPre_(const T &other);

  /** Applies <I>other</I> transform \f$T_\mathrm{o}\f$ before <CODE>*this</CODE> transform \f$T\f$:
    * \f[T_\mathrm{o} \circ T\f]
    */
  template<class T>
  QuaternionTransform applyPre(const T &other) const;

  /** Inplace variant of applyPost(). */
  template<class T>
  QuaternionTransform& applyPost_(const T &other);

  /** Applies <I>other</I> transform \f$T_\mathrm{o}\f$ after <CODE>*this</CODE> transform \f$T\f$:
    * \f[T \circ T_\mathrm{o}\f]
    */
  template<class T>
  QuaternionTransform applyPost(const T &other) const;

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

  /** \returns the translation vector. */
  const Eigen::Matrix<double, 3, 1>& getTranslation() const { return translation_; }

  /** \returns the rotation quaternion. */
  const Quaternion<double>& getQuaternion() const { return quaternion_; }

  /** Sets the translation vector. */
  template<typename Derived>
  QuaternionTransform& setTranslation(const Eigen::MatrixBase<Derived> &translation, bool unsafe = false);

  /** Sets the rotation quaternion. */
  QuaternionTransform& setQuaternion(const Quaternion<double> &quaternion, bool unsafe = false);

  /** \returns a description of <CODE>*this</CODE>. */
  std::string desc() const override { return streamToString(*this); }

  /** Inserts a description of <CODE>*this</CODE>. */
  std::ostream& stream(std::ostream& os) const override;

  /** Applies <I>other</I> after <CODE>*this</CODE>. */
  template<class T>
  QuaternionTransform& operator*=(const T &other);

  /** Applies the inverse of <I>other</I> after <CODE>*this</CODE>. */
  template<class T>
  QuaternionTransform& operator/=(const T &other);

 private:
  Eigen::Matrix<double, 3, 1> translation_;  ///< Translation vector
  Quaternion<double> quaternion_;            ///< Rotation quaternion
};

QuaternionTransform::QuaternionTransform()
  : TransformBase(false)
  , translation_(Eigen::Matrix<double, 3, 1>::Zero())
  , quaternion_(Quaternion<double>::Identity())
{
}

template<typename Derived>
QuaternionTransform::QuaternionTransform(
  const Eigen::DenseBase<Derived> &translation, 
  Quaternion<double> quaternion,
  const bool unsafe)
  : TransformBase(unsafe)
  , translation_(translation)
  , quaternion_(std::move(quaternion))
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  checkValid();
}

template<typename Derived>
QuaternionTransform::QuaternionTransform(
  const Eigen::DenseBase<Derived> &translation, 
  const Eigen::Quaternion<double> &quaternion,
  const bool unsafe)
  : TransformBase(unsafe)
  , translation_(translation)
  , quaternion_(quaternion)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  checkValid();
}

template<typename DerivedTrans, typename DerivedQuat>
QuaternionTransform::QuaternionTransform(
  const Eigen::DenseBase<DerivedTrans> &translation, 
  const Eigen::DenseBase<DerivedQuat> &quaternion,
  const bool unsafe)
  : TransformBase(unsafe)
  , translation_(translation)
  , quaternion_(quaternion(0), quaternion(1), quaternion(2), quaternion(3))
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedTrans, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedQuat, 4);
  checkValid();
}

QuaternionTransform::QuaternionTransform(const std::vector<std::uint8_t> &binary, const bool unsafe)
  : QuaternionTransform(convertVector<std::uint8_t, double>(binary), unsafe)
{
}

QuaternionTransform::QuaternionTransform(const std::vector<double> &vector, const bool unsafe)
  : TransformBase(unsafe)
{
  if (!unsafe_ && vector.size() != kVectorSize)
  {
    throw InvalidTypeSizeException();
  }
  // NOLINTBEGIN(*-magic-numbers)
  translation_ << vector.at(0), vector.at(1), vector.at(2);
  quaternion_.w() = vector.at(3);
  quaternion_.x() = vector.at(4);
  quaternion_.y() = vector.at(5);
  quaternion_.z() = vector.at(6);
  // NOLINTEND(*-magic-numbers)
  checkValid();
}

template<typename Derived>
QuaternionTransform::QuaternionTransform(const Eigen::DenseBase<Derived> &matrix, const bool unsafe)
  : TransformBase(unsafe)
{
  // check size
  if (!checkEigenVectorSize<kVectorSize>(matrix))
  {
    throw InvalidTypeSizeException();
  }

  // copy values
  // NOLINTBEGIN(*-magic-numbers)
  translation_ = matrix.template segment<3>(0);
  quaternion_.w() = matrix(3);
  quaternion_.x() = matrix(4);
  quaternion_.y() = matrix(5);
  quaternion_.z() = matrix(6);
  // NOLINTEND(*-magic-numbers)
  checkValid();
}

QuaternionTransform& QuaternionTransform::setIdentity()
{
  // set to identity
  translation_.setZero();
  quaternion_.setIdentity();

  // mark as safe
  unsafe_ = false;
  return *this;
}

std::vector<double> QuaternionTransform::toVector() const
{
  return {translation_(0), translation_(1), translation_(2),
          quaternion_.w(), quaternion_.x(), quaternion_.y(), quaternion_.z()};
}

Eigen::Matrix<double, Eigen::Dynamic, 1> QuaternionTransform::toEigenVector() const
{
  Eigen::Matrix<double, kVectorSize, 1> data;
  data << translation_(0), translation_(1), translation_(2),
          quaternion_.w(), quaternion_.x(), quaternion_.y(), quaternion_.z();
  return data;
}

bool QuaternionTransform::isValid(const double eps) const
{
  return isUnitQuaternion(quaternion_, eps);
}

bool QuaternionTransform::isUnitQuaternion(const Quaternion<double> &quaternion, const double eps)
{
  return std::abs(quaternion.norm() - 1.0) <= eps;
}

QuaternionTransform& QuaternionTransform::normalized_()
{
  quaternion_.normalized_();
  unsafe_ = false;
  return *this;
}

QuaternionTransform QuaternionTransform::normalized() const
{
  Quaternion<double> new_quaternion = quaternion_.normalized();
  return {translation_, new_quaternion, false};
}

QuaternionTransform& QuaternionTransform::scaleTranslation_(double factor)
{
  translation_ *= factor;
  return *this;
}

QuaternionTransform QuaternionTransform::scaleTranslation(double factor) const
{
  return {translation_ * factor, quaternion_, unsafe_};
}

template<typename Derived>
QuaternionTransform& QuaternionTransform::setTranslation(const Eigen::MatrixBase<Derived> &translation, const bool unsafe)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

  // unsafe
  bool new_unsafe = unsafe_ || unsafe;

  // set
  translation_ = translation;

  // return
  unsafe_ = new_unsafe;
  return *this;
}

QuaternionTransform& QuaternionTransform::setQuaternion(const Quaternion<double> &quaternion, const bool unsafe)
{
  // unsafe
  bool new_unsafe = unsafe_ || unsafe;

  // set
  if (new_unsafe || isUnitQuaternion(quaternion, kDefaultEps))
  {
    quaternion_ = quaternion;
  }
  else
  {
    throw InvalidTransformException("Input quaternion is not unit");
  }

  // return
  unsafe_ = new_unsafe;
  return *this;
}

std::ostream& QuaternionTransform::stream(std::ostream& os) const
{
  return os << "QuaternionTransform(" << 
    "translation=[" << translation_.transpose() << "], " <<
    "quaternion=[" << quaternion_.w() << " " << quaternion_.vec().transpose() << "])";
}

} // namespace motion3d
