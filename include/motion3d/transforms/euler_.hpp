#pragma once

#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include <motion3d/common/math.hpp>
#include <motion3d/transforms/base.hpp>
#include <motion3d/transforms/implementation_/euler_axes.hpp>
#include <motion3d/utils/conversion.hpp>
#include <motion3d/utils/templating.hpp>

namespace motion3d
{

/** \brief Euler transformation consisting of a translation vector and Euler rotation angles around three axes in a configurable order.
  * 
  * All possible orders are defined in the motion3d::EulerAxes enumerate.
  * Since the Euler angles are not subject to any constraints, they are always valid.
  */
class EulerTransform : public TransformBase<EulerTransform>
{
 public:
  DEFINE_POINTERS(EulerTransform);

  using TransformBase<EulerTransform>::isValid;

  /** Constructs and initializes an EulerTransform as identity transform. */
  EulerTransform();

  /** Constructs and initializes an EulerTransform from AxisAngleTransform <I>obj</I> with specific Euler axes order. */
  EulerTransform(
    const AxisAngleTransform &obj,
    EulerAxes axes);

  /** Constructs and initializes an EulerTransform from DualQuaternionTransform <I>obj</I> with specific Euler axes order. */
  EulerTransform(
    const DualQuaternionTransform &obj,
    EulerAxes axes);

  /** Constructs and initializes an EulerTransform from EulerTransform <I>obj</I> with specific Euler axes order. */
  EulerTransform(
    const EulerTransform &obj,
    EulerAxes axes);

  /** Constructs and initializes an EulerTransform from MatrixTransform <I>obj</I> with specific Euler axes order. */
  EulerTransform(
    const MatrixTransform &obj,
    EulerAxes axes);

  /** Constructs and initializes an EulerTransform from QuaternionTransform <I>obj</I> with specific Euler axes order. */
  EulerTransform(
    const QuaternionTransform &obj,
    EulerAxes axes);

  /** Constructs and initializes an EulerTransform from a <I>translation</I> vector, separate rotation angles in <I>rad</I>, and Euler axes order. */
  template<typename Derived>
  EulerTransform(
    const Eigen::DenseBase<Derived> &translation,
    double ai,
    double aj,
    double ak,
    EulerAxes axes = kEulerAxesDefault,
    bool unsafe = false);

  /** Constructs and initializes an EulerTransform from a <I>translation</I> vector, rotation angles in <I>rad</I>, and Euler axes order. */
  template<typename DerivedTrans, typename DerivedAngles>
  EulerTransform(
    const Eigen::DenseBase<DerivedTrans> &translation,
    const Eigen::DenseBase<DerivedAngles> &angles,
    EulerAxes axes = kEulerAxesDefault,
    bool unsafe = false);

  /** Constructs and initializes an EulerTransform from <I>binary</I> data.
    * \throws InvalidTypeSizeException for an invalid binary size.
    */
  explicit EulerTransform(const std::vector<std::uint8_t> &binary, bool unsafe = false);

  /** Constructs and initializes an EulerTransform from <I>vector</I> data.
    * \throws InvalidTypeSizeException for an invalid vector size.
    */
  explicit EulerTransform(const std::vector<double> &vector, bool unsafe = false);

  /** Constructs and initializes an EulerTransform from an Eigen vector <I>matrix</I>.
    * \throws InvalidTypeSizeException for an invalid matrix size.
    */
  template<typename Derived>
  explicit EulerTransform(const Eigen::DenseBase<Derived> &matrix, bool unsafe = false);

  /** Create a copy of <CODE>*this</CODE>. */
  EulerTransform copy() const { return {*this}; }

  /** Set <CODE>*this</CODE> to an identity transform. */
  EulerTransform& setIdentity();

  /** Converts <CODE>*this</CODE> to an std vector. */
  std::vector<double> toVector() const override;

  /** Converts <CODE>*this</CODE> to an Eigen vector. */
  Eigen::Matrix<double, Eigen::Dynamic, 1> toEigenVector() const override;

  /** Converts <CODE>*this</CODE> to transform type <I>T</I>. */
  template<class T>
  T asType() const;

  /** Checks if <CODE>*this</CODE> is valid w.r.t. the threshold <I>eps</I>. */ 
  bool isValid(double eps) const override;

  /** Inplace variant of changeAxes(). */
  EulerTransform& changeAxes_(EulerAxes axes);

  /** Change Euler axes order. */
  EulerTransform changeAxes(EulerAxes axes) const;

  /** Inplace variant of inverse(). */
  EulerTransform& inverse_();

  /** \returns the inverse transform to <CODE>*this</CODE>. */
  EulerTransform inverse() const;

  /** Inplace variant of normalized(). */
  EulerTransform& normalized_();

  /** Normalizes <CODE>*this</CODE> to a valid Euler transform and sets <CODE>unsafe</CODE> to <CODE>false</CODE>. */
  EulerTransform normalized() const;

  /** Inplace variant of scaleTranslation(). */
  EulerTransform& scaleTranslation_(double factor);

  /** Scales the translation by <I>factor</I>. */
  EulerTransform scaleTranslation(double factor) const;

  /** Inplace variant of applyPre(). */
  template<class T>
  EulerTransform& applyPre_(const T &other);

  /** Applies <I>other</I> transform \f$T_\mathrm{o}\f$ before <CODE>*this</CODE> transform \f$T\f$:
    * \f[T_\mathrm{o} \circ T\f]
    */
  template<class T>
  EulerTransform applyPre(const T &other) const;

  /** Inplace variant of applyPost(). */
  template<class T>
  EulerTransform& applyPost_(const T &other);

  /** Applies <I>other</I> transform \f$T_\mathrm{o}\f$ after <CODE>*this</CODE> transform \f$T\f$:
    * \f[T \circ T_\mathrm{o}\f]
    */
  template<class T>
  EulerTransform applyPost(const T &other) const;

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

  /** \returns the rotation around axis \f$i\f$ in <I>rad</I>. */
  double getAi() const { return ai_; }

  /** \returns the rotation around axis \f$j\f$ in <I>rad</I>. */
  double getAj() const { return aj_; }

  /** \returns the rotation around axis \f$k\f$ in <I>rad</I>. */
  double getAk() const { return ak_; }
  
  /** \returns the rotations around all axes in order \f$[i \ j \ k]\f$ in <I>rad</I> as Eigen matrix. */
  Eigen::Matrix<double, 3, 1> getAngles() const;

  /** \returns the Euler axes order. */
  EulerAxes getAxes() const { return axes_; }

  /** Sets the translation vector. */
  template<typename Derived>
  EulerTransform& setTranslation(const Eigen::MatrixBase<Derived> &translation, bool unsafe = false);

  /** Sets the rotation around axis \f$i\f$ in <I>rad</I>. */
  EulerTransform& setAi(const double &ai, bool unsafe = false);

  /** Sets the rotation around axis \f$j\f$ in <I>rad</I>. */
  EulerTransform& setAj(const double &aj, bool unsafe = false);

  /** Sets the rotation around axis \f$k\f$ in <I>rad</I>. */
  EulerTransform& setAk(const double &ak, bool unsafe = false);

  /** Sets the rotations around all axes in order \f$[i \ j \ k]\f$ in <I>rad</I> as Eigen matrix. */
  template<typename Derived>
  EulerTransform& setAngles(const Eigen::MatrixBase<Derived> &angles, bool unsafe = false);

  /** Sets the Euler axes order. */
  EulerTransform& setAxes(const EulerAxes &axes, bool unsafe = false);

  /** \returns a description of <CODE>*this</CODE>. */
  std::string desc() const override { return streamToString(*this); }

  /** Inserts a description of <CODE>*this</CODE>. */
  std::ostream& stream(std::ostream& os) const override;

  /** Applies <I>other</I> after <CODE>*this</CODE>. */
  template<class T>
  EulerTransform& operator*=(const T &other);

  /** Applies the inverse of <I>other</I> after <CODE>*this</CODE>. */
  template<class T>
  EulerTransform& operator/=(const T &other);

 private:
  Eigen::Matrix<double, 3, 1> translation_;  ///< Translation vector
  double ai_;                                ///< Rotation around axis \f$i\f$ in <I>rad</I>
  double aj_;                                ///< Rotation around axis \f$j\f$ in <I>rad</I>
  double ak_;                                ///< Rotation around axis \f$k\f$ in <I>rad</I>
  EulerAxes axes_;                           ///< Euler axes order
};

EulerTransform::EulerTransform()
  : TransformBase(false)
  , translation_(Eigen::Matrix<double, 3, 1>::Zero())
  , ai_(0.0)
  , aj_(0.0)
  , ak_(0.0)
  , axes_(kEulerAxesDefault)
{
}

template<typename Derived>
EulerTransform::EulerTransform(
  const Eigen::DenseBase<Derived> &translation,
  const double ai,
  const double aj,
  const double ak,
  const EulerAxes axes,
  const bool unsafe)
  : TransformBase(unsafe)
  , translation_(translation)
  , ai_(ai)
  , aj_(aj)
  , ak_(ak)
  , axes_(axes)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  checkValid();
}

template<typename DerivedTrans, typename DerivedAngles>
EulerTransform::EulerTransform(
  const Eigen::DenseBase<DerivedTrans> &translation,
  const Eigen::DenseBase<DerivedAngles> &angles,
  const EulerAxes axes,
  const bool unsafe)
  : TransformBase(unsafe)
  , translation_(translation)
  , ai_(angles(0))
  , aj_(angles(1))
  , ak_(angles(2))
  , axes_(axes)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedTrans, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedAngles, 3);
  checkValid();
}

EulerTransform::EulerTransform(const std::vector<std::uint8_t> &binary, const bool unsafe)
  : EulerTransform(convertVector<std::uint8_t, double>(binary), unsafe)
{
}

EulerTransform::EulerTransform(const std::vector<double> &vector, const bool unsafe)
  : TransformBase(unsafe)
{
  if (!unsafe_ && vector.size() != kVectorSize)
  {
    throw InvalidTypeSizeException();
  }
  // NOLINTBEGIN(*-magic-numbers)
  translation_ << vector.at(0), vector.at(1), vector.at(2);
  ai_ = vector.at(3);
  aj_ = vector.at(4);
  ak_ = vector.at(5);
  axes_ = eulerAxesFromDouble(vector.at(6), unsafe_);
  // NOLINTEND(*-magic-numbers)
  checkValid();
}

template<typename Derived>
EulerTransform::EulerTransform(const Eigen::DenseBase<Derived> &matrix, const bool unsafe)
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
  ai_ = matrix(3);
  aj_ = matrix(4);
  ak_ = matrix(5);
  axes_ = eulerAxesFromDouble(matrix(6), unsafe_);
  // NOLINTEND(*-magic-numbers)
  checkValid();
}

EulerTransform& EulerTransform::setIdentity()
{
  // set to identity
  translation_.setZero();
  ai_ = 0.0;
  aj_ = 0.0;
  ak_ = 0.0;

  // mark as safe
  unsafe_ = false;
  return *this;
}

std::vector<double> EulerTransform::toVector() const
{
  return {translation_(0), translation_(1), translation_(2),
          ai_, aj_, ak_,
          eulerAxesToDouble(axes_)};
}

Eigen::Matrix<double, Eigen::Dynamic, 1> EulerTransform::toEigenVector() const
{
  Eigen::Matrix<double, kVectorSize, 1> data;
  data << translation_(0), translation_(1), translation_(2),
          ai_, aj_, ak_,
          eulerAxesToDouble(axes_);
  return data;
}

bool EulerTransform::isValid(const double /*eps*/) const
{
  return true;
}

EulerTransform& EulerTransform::changeAxes_(const EulerAxes axes)
{
  if (axes_ != axes)
  {
    Eigen::Matrix<double, 3, 3> matrix = euler::eulerToMatrix(ai_, aj_, ak_, axes_);
    std::tie(ai_, aj_, ak_) = euler::matrixToEuler(matrix, axes);
    axes_ = axes;
  }
  return *this;
}

EulerTransform EulerTransform::changeAxes(const EulerAxes axes) const
{
  return {*this, axes};
}

EulerTransform& EulerTransform::normalized_()
{
  normalizeAngle_(ai_);
  normalizeAngle_(aj_);
  normalizeAngle_(ak_);
  unsafe_ = false;
  return *this;
}

EulerTransform EulerTransform::normalized() const
{
  double new_ai = normalizeAngle(ai_);
  double new_aj = normalizeAngle(aj_);
  double new_ak = normalizeAngle(ak_);
  return {translation_, new_ai, new_aj, new_ak, axes_, false};
}

EulerTransform& EulerTransform::scaleTranslation_(double factor)
{
  translation_ *= factor;
  return *this;
}

EulerTransform EulerTransform::scaleTranslation(double factor) const
{
  return {translation_ * factor, ai_, aj_, ak_, axes_, unsafe_};
}

Eigen::Matrix<double, 3, 1> EulerTransform::getAngles() const
{
  Eigen::Matrix<double, 3, 1> angles;
  angles << ai_, aj_, ak_;
  return angles;
}

template<typename Derived>
EulerTransform& EulerTransform::setTranslation(const Eigen::MatrixBase<Derived> &translation, const bool unsafe)
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

EulerTransform& EulerTransform::setAi(const double &ai, const bool unsafe)
{
  // unsafe
  bool new_unsafe = unsafe_ || unsafe;

  // set
  ai_ = ai;

  // check
  unsafe_ = new_unsafe;
  return *this;
}

EulerTransform& EulerTransform::setAj(const double &aj, const bool unsafe)
{
  // unsafe
  bool new_unsafe = unsafe_ || unsafe;

  // set
  aj_ = aj;

  // check
  unsafe_ = new_unsafe;
  return *this;
}

EulerTransform& EulerTransform::setAk(const double &ak, const bool unsafe)
{
  // unsafe
  bool new_unsafe = unsafe_ || unsafe;

  // set
  ak_ = ak;

  // check
  unsafe_ = new_unsafe;
  return *this;
}

template<typename Derived>
EulerTransform& EulerTransform::setAngles(const Eigen::MatrixBase<Derived> &angles, const bool unsafe)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

  // unsafe
  bool new_unsafe = unsafe_ || unsafe;

  // set
  ai_ = angles(0);
  aj_ = angles(1);
  ak_ = angles(2);

  // check
  unsafe_ = new_unsafe;
  return *this;
}

EulerTransform& EulerTransform::setAxes(const EulerAxes &axes, const bool unsafe)
{
  // unsafe
  bool new_unsafe = unsafe_ || unsafe;

  // set
  axes_ = axes;

  // check
  unsafe_ = new_unsafe;
  return *this;
}

std::ostream& EulerTransform::stream(std::ostream& os) const
{
  return os << "EulerTransform(" <<
            "translation=[" << translation_.transpose() << "], " <<
            "ai=" << ai_ << ", " <<
            "aj=" << aj_ << ", " <<
            "ak=" << ak_ << ", " <<
            "axes=" << eulerAxesToStr(axes_) << ")";
}

} // namespace motion3d
