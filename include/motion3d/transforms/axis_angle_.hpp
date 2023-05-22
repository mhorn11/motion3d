#pragma once

#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include <motion3d/common/math.hpp>
#include <motion3d/transforms/base.hpp>
#include <motion3d/utils/conversion.hpp>
#include <motion3d/utils/templating.hpp>

namespace motion3d
{

/** \brief Axis-angle transformation consisting of a translation vector, a rotation axis and a rotation angle around this axis.
  *
  * For representing a valid spatial transformation, the rotation axis must have unit length.
  * An unsafe transform means that the validity check, usually performed at each operation, is skipped.
  */
class AxisAngleTransform : public TransformBase<AxisAngleTransform>
{
 public:
  DEFINE_POINTERS(AxisAngleTransform);

  using TransformBase<AxisAngleTransform>::isValid;

  /** Constructs and initializes an AxisAngleTransform as identity transform. */
  AxisAngleTransform();

  /** Constructs and initializes an AxisAngleTransform from <I>translation</I> vector and axis-angle rotation <I>angle_axis</I>. */
  template<typename Derived>
  AxisAngleTransform(
    const Eigen::DenseBase<Derived> &translation,
    Eigen::AngleAxis<double> angle_axis,
    bool unsafe = false);

  /** Constructs and initializes an AxisAngleTransform from <I>translation</I> vector, rotation <I>angle</I> in <I>rad</I>,
    * and rotation <I>axis</I>.
    */
  template<typename DerivedTrans, typename DerivedAxis>
  AxisAngleTransform(
    const Eigen::DenseBase<DerivedTrans> &translation,
    double angle,
    const Eigen::MatrixBase<DerivedAxis> &axis,
    bool unsafe = false);

  /** Constructs and initializes an AxisAngleTransform from <I>binary</I> data.
    * \throws InvalidTypeSizeException for an invalid binary size.
    */
  explicit AxisAngleTransform(const std::vector<std::uint8_t> &binary, bool unsafe = false);

  /** Constructs and initializes an AxisAngleTransform from <I>vector</I> data.
    * \throws InvalidTypeSizeException for an invalid vector size.
    */
  explicit AxisAngleTransform(const std::vector<double> &vector, bool unsafe = false);

  /** Constructs and initializes an AxisAngleTransform from an Eigen vector <I>matrix</I>.
    * \throws InvalidTypeSizeException for an invalid matrix size.
    */
  template<typename Derived>
  explicit AxisAngleTransform(const Eigen::DenseBase<Derived> &matrix, bool unsafe = false);

  /** Create a copy of <CODE>*this</CODE>. */
  AxisAngleTransform copy() const { return {*this}; }

  /** Set <CODE>*this</CODE> to an identity transform. */
  AxisAngleTransform& setIdentity();

  /** Converts <CODE>*this</CODE> to an std vector. */
  std::vector<double> toVector() const override;

  /** Converts <CODE>*this</CODE> to an Eigen vector. */
  Eigen::Matrix<double, Eigen::Dynamic, 1> toEigenVector() const override;

  /** Converts <CODE>*this</CODE> to transform type <I>T</I>. */
  template<class T>
  T asType() const;

  /** Checks if <CODE>*this</CODE> is valid w.r.t. the threshold <I>eps</I>. */
  bool isValid(double eps) const override;

  /** Checks if <I>axis</I> is unit w.r.t. the threshold <I>eps</I>. */
  template<typename Derived>
  static bool isUnitAxis(const Eigen::MatrixBase<Derived> &axis, double eps);

  /** Inplace variant of inverse(). */
  AxisAngleTransform& inverse_();

  /** \returns the inverse transform to <CODE>*this</CODE>. */
  AxisAngleTransform inverse() const;

  /** Inplace variant of normalized(). */
  AxisAngleTransform& normalized_();

  /** Normalizes <CODE>*this</CODE> to a valid axis-angle transform and sets <CODE>unsafe</CODE> to <CODE>false</CODE>. */
  AxisAngleTransform normalized() const;

  /** Inplace variant of scaleTranslation(). */
  AxisAngleTransform& scaleTranslation_(double factor);

  /** Scales the translation by <I>factor</I>. */
  AxisAngleTransform scaleTranslation(double factor) const;

  /** Inplace variant of applyPre(). */
  template<class T>
  AxisAngleTransform& applyPre_(const T &other);

  /** Applies <I>other</I> transform \f$T_\mathrm{o}\f$ before <CODE>*this</CODE> transform \f$T\f$:
    * \f[T_\mathrm{o} \circ T\f]
    */
  template<class T>
  AxisAngleTransform applyPre(const T &other) const;

  /** Inplace variant of applyPost(). */
  template<class T>
  AxisAngleTransform& applyPost_(const T &other);

  /** Applies <I>other</I> transform \f$T_\mathrm{o}\f$ after <CODE>*this</CODE> transform \f$T\f$:
    * \f[T \circ T_\mathrm{o}\f]
    */
  template<class T>
  AxisAngleTransform applyPost(const T &other) const;

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

  /** \returns the angle-axis rotation. */
  const Eigen::AngleAxis<double>& getAngleAxis() const { return angle_axis_; }

  /** \returns the rotation angle in <I>rad</I>. */
  double getAngle() const { return angle_axis_.angle(); }

  /** \returns the rotation axis. */
  const Eigen::Matrix<double, 3, 1>& getAxis() const { return angle_axis_.axis(); }

  /** Sets the translation vector. */
  template<typename Derived>
  AxisAngleTransform& setTranslation(const Eigen::MatrixBase<Derived> &translation, bool unsafe = false);

  /** Sets the angle-axis rotation. */
  AxisAngleTransform& setAngleAxis(const Eigen::AngleAxis<double> &angle_axis, bool unsafe = false);

  /** Sets the rotation angle in <I>rad</I>. */
  AxisAngleTransform& setAngle(const double &angle, bool unsafe = false);

  /** Sets the rotation axis. */
  template<typename Derived>
  AxisAngleTransform& setAxis(const Eigen::MatrixBase<Derived> &axis, bool unsafe = false);

  /** \returns a description of <CODE>*this</CODE>. */
  std::string desc() const override { return streamToString(*this); }

  /** Inserts a description of <CODE>*this</CODE>. */
  std::ostream& stream(std::ostream& os) const override;

  /** Applies <I>other</I> after <CODE>*this</CODE>. */
  template<class T>
  AxisAngleTransform& operator*=(const T &other);

  /** Applies the inverse of <I>other</I> after <CODE>*this</CODE>. */
  template<class T>
  AxisAngleTransform& operator/=(const T &other);

 private:
  Eigen::Matrix<double, 3, 1> translation_;  ///< Translation vector
  Eigen::AngleAxis<double> angle_axis_;      ///< Angle-axis rotation
};

AxisAngleTransform::AxisAngleTransform()
  : TransformBase(false)
  , translation_(Eigen::Matrix<double, 3, 1>::Zero())
  , angle_axis_(0.0, Eigen::Matrix<double, 3, 1>::UnitX())
{
}

template<typename Derived>
AxisAngleTransform::AxisAngleTransform(
  const Eigen::DenseBase<Derived> &translation,
  Eigen::AngleAxis<double> angle_axis,
  const bool unsafe)
  : TransformBase(unsafe)
  , translation_(translation)
  , angle_axis_(std::move(angle_axis))
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  checkValid();
}

template<typename DerivedTrans, typename DerivedAxis>
AxisAngleTransform::AxisAngleTransform(
  const Eigen::DenseBase<DerivedTrans> &translation,
  const double angle,
  const Eigen::MatrixBase<DerivedAxis> &axis,
  const bool unsafe)
  : TransformBase(unsafe)
  , translation_(translation)
  , angle_axis_(angle, axis)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedTrans, 3);
  checkValid();
}

AxisAngleTransform::AxisAngleTransform(const std::vector<std::uint8_t> &binary, const bool unsafe)
  : AxisAngleTransform(convertVector<std::uint8_t, double>(binary), unsafe)
{
}

AxisAngleTransform::AxisAngleTransform(const std::vector<double> &vector, const bool unsafe)
  : TransformBase(unsafe)
{
  if (!unsafe_ && vector.size() != kVectorSize)
  {
    throw InvalidTypeSizeException();
  }
  translation_ << vector.at(0), vector.at(1), vector.at(2);
  angle_axis_.angle() = vector.at(3);
  angle_axis_.axis() << vector.at(4), vector.at(5), vector.at(6);  // NOLINT(*-magic-numbers)
  checkValid();
}

template<typename Derived>
AxisAngleTransform::AxisAngleTransform(const Eigen::DenseBase<Derived> &matrix, const bool unsafe)
  : TransformBase(unsafe)
{
  // check size
  if (!checkEigenVectorSize<kVectorSize>(matrix))
  {
    throw InvalidTypeSizeException();
  }

  // copy values
  translation_ = matrix.template segment<3>(0);
  angle_axis_.angle() = matrix(3);
  angle_axis_.axis() = matrix.template segment<3>(4);
  checkValid();
}

AxisAngleTransform& AxisAngleTransform::setIdentity()
{
  // set to identity
  translation_.setZero();
  angle_axis_.angle() = 0.0;
  angle_axis_.axis() = Eigen::Matrix<double, 3, 1>::UnitX();

  // mark as safe
  unsafe_ = false;
  return *this;
}

std::vector<double> AxisAngleTransform::toVector() const
{
  return {translation_(0), translation_(1), translation_(2),
          angle_axis_.angle(),
          angle_axis_.axis()(0), angle_axis_.axis()(1), angle_axis_.axis()(2)};
}

Eigen::Matrix<double, Eigen::Dynamic, 1> AxisAngleTransform::toEigenVector() const
{
  Eigen::Matrix<double, kVectorSize, 1> data;
  data << translation_(0), translation_(1), translation_(2),
          angle_axis_.angle(),
          angle_axis_.axis()(0), angle_axis_.axis()(1), angle_axis_.axis()(2);
  return data;
}

bool AxisAngleTransform::isValid(const double eps) const
{
  return isUnitAxis(angle_axis_.axis(), eps);
}

template<typename Derived>
bool AxisAngleTransform::isUnitAxis(const Eigen::MatrixBase<Derived> &axis, const double eps)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  return std::abs(axis.norm() - 1.0) <= eps;
}

AxisAngleTransform& AxisAngleTransform::normalized_()
{
  // angle
  normalizeAngle_(angle_axis_.angle());

  // axis
  if (std::abs(angle_axis_.angle()) < kDefaultEps * kDefaultEps)
  {
    angle_axis_.angle() = 0.0;
    angle_axis_.axis() = Eigen::Matrix<double, 3, 1>::UnitX();
  }
  else
  {
    double factor = getAxisNormalizationFactorInverse(angle_axis_.axis(), true);
    if (std::abs(factor) < kDefaultEps)
    {
      angle_axis_.angle() = 0.0;
      angle_axis_.axis() = Eigen::Matrix<double, 3, 1>::UnitX();
    }
    else
    {
      angle_axis_.angle() *= std::copysign(1.0, factor);
      angle_axis_.axis() /= factor;
    }
  }

  // mark as safe
  unsafe_ = false;
  return *this;
}

AxisAngleTransform AxisAngleTransform::normalized() const
{
  return AxisAngleTransform(*this).normalized_();
}

AxisAngleTransform& AxisAngleTransform::scaleTranslation_(double factor)
{
  translation_ *= factor;
  return *this;
}

AxisAngleTransform AxisAngleTransform::scaleTranslation(double factor) const
{
  return {translation_ * factor, angle_axis_, unsafe_};
}

template<typename Derived>
AxisAngleTransform& AxisAngleTransform::setTranslation(const Eigen::MatrixBase<Derived> &translation, const bool unsafe)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

  // unsafe
  bool new_unsafe = unsafe_ || unsafe;

  // set
  translation_ = translation;

  // check
  unsafe_ = new_unsafe;
  return *this;
}

AxisAngleTransform& AxisAngleTransform::setAngleAxis(const Eigen::AngleAxis<double> &angle_axis, const bool unsafe)
{
  // unsafe
  bool new_unsafe = unsafe_ || unsafe;

  // set
  if (new_unsafe || isUnitAxis(angle_axis.axis(), kDefaultEps))
  {
    angle_axis_ = angle_axis;
  }
  else
  {
    throw InvalidTransformException("Input axis is not unit");
  }

  // check
  unsafe_ = new_unsafe;
  return *this;
}

AxisAngleTransform& AxisAngleTransform::setAngle(const double &angle, const bool unsafe)
{
  // unsafe
  bool new_unsafe = unsafe_ || unsafe;

  // set
  angle_axis_.angle() = angle;

  // check
  unsafe_ = new_unsafe;
  return *this;
}

template<typename Derived>
AxisAngleTransform& AxisAngleTransform::setAxis(const Eigen::MatrixBase<Derived> &axis, const bool unsafe)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

  // unsafe
  bool new_unsafe = unsafe_ || unsafe;

  // set
  if (new_unsafe || isUnitAxis(axis, kDefaultEps))
  {
    angle_axis_.axis() = axis;
  }
  else
  {
    throw InvalidTransformException("Input axis is not unit");
  }

  // check
  unsafe_ = new_unsafe;
  return *this;
}

std::ostream& AxisAngleTransform::stream(std::ostream& os) const
{
  return os << "AxisAngleTransform(" <<
            "translation=[" << translation_.transpose() << "], " <<
            "angle=" << angle_axis_.angle() << ", "
                                               "axis=[" << angle_axis_.axis().transpose() << "])";
}

} // namespace motion3d
