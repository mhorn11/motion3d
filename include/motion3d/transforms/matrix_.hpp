#pragma once

#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include <motion3d/common/math.hpp>
#include <motion3d/transforms/base.hpp>
#include <motion3d/utils/conversion.hpp>
#include <motion3d/utils/templating.hpp>

namespace motion3d
{

/** \brief Matrix transformation consisting of a homogeneous \f$\mathrm{SE(3)}\f$ transformation matrix.
  *
  * The matrix
  * \f[\quad \boldsymbol{M} = \begin{bmatrix} \boldsymbol{R} & \boldsymbol{t} \\ \boldsymbol{0} & 1 \end{bmatrix}\f]
  * consists of the 3x3 rotation matrix \f$\boldsymbol{R} \in \mathrm{SO(3)}\f$ and the translation vector \f$\boldsymbol{t}\f$.
  * 
  * For representing a valid spatial transformation, the rotation matrix must be orthogonal and have determinant +1.
  * An unsafe transform means that the validity check, usually performed at each operation, is skipped.
  * 
  * Operations:
  * \li Concatenation: \f$\mathrm{M}_{ac} = \mathrm{M}_{ab} \mathrm{M}_{bc}\f$
  * \li Inversion: \f$V^{-1} \equiv \begin{bmatrix} \boldsymbol{R}^\mathrm{T} & - \boldsymbol{R}^\mathrm{T} \boldsymbol{t} \\ \boldsymbol{0} & 1 \end{bmatrix}\f$
  * \li Point transformation: \f$[\boldsymbol{p}_a \ 1]^\mathrm{T} = \mathrm{M}_{ab} \, [\boldsymbol{p}_b \ 1]^\mathrm{T}\f$
  */
class MatrixTransform : public TransformBase<MatrixTransform>
{
 public:
  DEFINE_POINTERS(MatrixTransform);

  using TransformBase<MatrixTransform>::isValid;

  /** Constructs and initializes a MatrixTransform as identity transform. */
  MatrixTransform();

  /** Constructs and initializes a MatrixTransform from a <I>translation</I> vector and a \f$\mathrm{SO(3)}\f$ <I>rotation_matrix</I>. */
  MatrixTransform(
    const Eigen::Matrix<double, 3, 1> &translation,
    const Eigen::Matrix<double, 3, 3> &rotation_matrix,
    bool unsafe = false);

  /** Constructs and initializes a MatrixTransform from <I>binary</I> data.
    * \throws InvalidTypeSizeException for an invalid binary size.
    */
  explicit MatrixTransform(const std::vector<std::uint8_t> &binary, bool unsafe = false);

  /** Constructs and initializes a MatrixTransform from <I>vector</I> data.
    * \throws InvalidTypeSizeException for an invalid vector size.
    */
  explicit MatrixTransform(const std::vector<double> &vector, bool unsafe = false);

  /** Constructs and initializes a MatrixTransform from 
    * - a homogeneous \f$\mathrm{SE(3)}\f$ transformation <I>matrix</I>.
    * - the upper 3x4 part of a homogeneous \f$\mathrm{SE(3)}\f$ transformation <I>matrix</I>.
    * - an Eigen vector <I>matrix</I>.
    *
    * \throws InvalidTypeSizeException for an invalid size.
    */
  template<typename Derived>
  explicit MatrixTransform(const Eigen::DenseBase<Derived> &matrix, bool unsafe = false);

  /** Create a copy of <CODE>*this</CODE>. */
  MatrixTransform copy() const { return {*this}; }

  /** Set <CODE>*this</CODE> to an identity transform. */
  MatrixTransform& setIdentity();

  /** Converts <CODE>*this</CODE> to an std vector. */
  std::vector<double> toVector() const override;

  /** Converts <CODE>*this</CODE> to an Eigen vector. */
  Eigen::Matrix<double, Eigen::Dynamic, 1> toEigenVector() const override;

  /** Converts <CODE>*this</CODE> to transform type <I>T</I>. */
  template<class T>
  T asType() const;

  /** Checks if <CODE>*this</CODE> is valid w.r.t. the threshold <I>eps</I>. */ 
  bool isValid(double eps) const override;

  /** Checks if <I>matrix</I> is a valid rigid transformation matrix w.r.t. the threshold <I>eps</I>. */
  template<typename Derived>
  static bool isValidMatrix(const Eigen::MatrixBase<Derived> &matrix, double eps);

  /** Checks if <I>matrix</I> is a valid rotation matrix w.r.t. the threshold <I>eps</I>. */
  template<typename Derived>
  static bool isValidRotationMatrix(const Eigen::MatrixBase<Derived> &matrix, double eps);

  /** Inplace variant of inverse(). */
  MatrixTransform& inverse_();

  /** \returns the inverse transform to <CODE>*this</CODE>. */
  MatrixTransform inverse() const;

  /** Inplace variant of normalized(). */
  MatrixTransform& normalized_();

  /** Normalizes <CODE>*this</CODE> to a valid matrix transform and sets <CODE>unsafe</CODE> to <CODE>false</CODE>. */
  MatrixTransform normalized() const;

  /** Inplace variant of scaleTranslation(). */
  MatrixTransform& scaleTranslation_(double factor);

  /** Scales the translation by <I>factor</I>. */
  MatrixTransform scaleTranslation(double factor) const;

  /** Inplace variant of applyPre(). */
  template<class T>
  MatrixTransform& applyPre_(const T &other);

  /** Applies <I>other</I> transform \f$T_\mathrm{o}\f$ before <CODE>*this</CODE> transform \f$T\f$:
    * \f[T_\mathrm{o} \circ T\f]
    */
  template<class T>
  MatrixTransform applyPre(const T &other) const;

  /** Inplace variant of applyPost(). */
  template<class T>
  MatrixTransform& applyPost_(const T &other);

  /** Applies <I>other</I> transform \f$T_\mathrm{o}\f$ after <CODE>*this</CODE> transform \f$T\f$:
    * \f[T \circ T_\mathrm{o}\f]
    */
  template<class T>
  MatrixTransform applyPost(const T &other) const;

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
  
  /** \returns the homogeneous \f$\mathrm{SE(3)}\f$ transformation matrix. */
  const Eigen::Matrix<double, 4, 4>& getMatrix() const { return matrix_; }

  /** \returns the translation vector. */
  Eigen::Matrix<double, 3, 1> getTranslation() const { return matrix_.block<3, 1>(0, 3); }

  /** \returns the \f$\mathrm{SO(3)}\f$ rotation matrix. */
  Eigen::Matrix<double, 3, 3> getRotationMatrix() const { return matrix_.block<3, 3>(0, 0); }

  /** Sets the homogeneous \f$\mathrm{SE(3)}\f$ transformation matrix. */
  template<typename Derived>
  MatrixTransform& setMatrix(const Eigen::MatrixBase<Derived> &matrix, bool unsafe = false);

  /** Sets the translation vector. */
  MatrixTransform& setTranslation(const Eigen::Matrix<double, 3, 1> &translation, bool unsafe = false);

  /** Sets the \f$\mathrm{SO(3)}\f$ rotation matrix. */
  MatrixTransform& setRotationMatrix(const Eigen::Matrix<double, 3, 3> &rotation_matrix, bool unsafe = false);

  /** \returns a description of <CODE>*this</CODE>. */
  std::string desc() const override { return streamToString(*this); }
  
  /** Inserts a description of <CODE>*this</CODE>. */
  std::ostream& stream(std::ostream& os) const override;

  /** Applies <I>other</I> after <CODE>*this</CODE>. */
  template<class T>
  MatrixTransform& operator*=(const T &other);

  /** Applies the inverse of <I>other</I> after <CODE>*this</CODE>. */
  template<class T>
  MatrixTransform& operator/=(const T &other);

 private:
  Eigen::Matrix<double, 4, 4> matrix_;  ///< Homogeneous \f$\mathrm{SE(3)}\f$ transformation matrix
};

MatrixTransform::MatrixTransform()
 : TransformBase(false)
{
  matrix_.setIdentity();
}

MatrixTransform::MatrixTransform(
  const Eigen::Matrix<double, 3, 1>& translation,
  const Eigen::Matrix<double, 3, 3>& rotation_matrix,
  const bool unsafe)
  : TransformBase(unsafe)
{
  matrix_.setIdentity();
  matrix_.block<3, 1>(0, 3) = translation;
  matrix_.block<3, 3>(0, 0) = rotation_matrix;
  checkValid();
}

MatrixTransform::MatrixTransform(const std::vector<std::uint8_t> &binary, const bool unsafe)
  : MatrixTransform(convertVector<std::uint8_t, double>(binary), unsafe)
{
}

MatrixTransform::MatrixTransform(const std::vector<double> &vector, const bool unsafe)
  : TransformBase(unsafe)
{
  if (!unsafe_ && vector.size() != kVectorSize)
  {
    throw InvalidTypeSizeException();
  }
  // NOLINTBEGIN(*-magic-numbers)
  matrix_ << vector.at(0), vector.at(1),  vector.at(2),  vector.at(3),
             vector.at(4), vector.at(5),  vector.at(6),  vector.at(7),
             vector.at(8), vector.at(9), vector.at(10), vector.at(11),
                      0.0,          0.0,           0.0,           1.0;
  // NOLINTEND(*-magic-numbers)
  checkValid();
}

template<typename Derived>
MatrixTransform::MatrixTransform(const Eigen::DenseBase<Derived> &matrix, const bool unsafe)
  : TransformBase(unsafe)
{
  if constexpr (Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4)
  {
    // create MatrixTransform from a homogeneous SE(3) transformation matrix
    matrix_ = matrix;
  }
  else if constexpr (Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 4)
  {
    // create MatrixTransform from upper part of a homogeneous SE(3) transformation matrix
    matrix_.setIdentity();
    matrix_.block<3, 4>(0, 0) = matrix;
  }
  else
  {
    // create MatrixTransform from vector
    // check size
    if (!checkEigenVectorSize<kVectorSize>(matrix))
    {
      throw InvalidTypeSizeException();
    }

    // copy values
    // NOLINTBEGIN(*-magic-numbers)
    matrix_ << matrix(0), matrix(1),  matrix(2),  matrix(3),
               matrix(4), matrix(5),  matrix(6),  matrix(7),
               matrix(8), matrix(9), matrix(10), matrix(11),
                     0.0,       0.0,        0.0,        1.0;
    // NOLINTEND(*-magic-numbers)
  }

  checkValid();
}

MatrixTransform& MatrixTransform::setIdentity()
{
  // set to identity
  matrix_.setIdentity();

  // mark as safe
  unsafe_ = false;
  return *this;
}

std::vector<double> MatrixTransform::toVector() const
{
  return {matrix_(0, 0), matrix_(0, 1), matrix_(0, 2), matrix_(0, 3),
          matrix_(1, 0), matrix_(1, 1), matrix_(1, 2), matrix_(1, 3),
          matrix_(2, 0), matrix_(2, 1), matrix_(2, 2), matrix_(2, 3)};
}

Eigen::Matrix<double, Eigen::Dynamic, 1> MatrixTransform::toEigenVector() const
{
  Eigen::Matrix<double, kVectorSize, 1> data;
  data << matrix_(0, 0), matrix_(0, 1), matrix_(0, 2), matrix_(0, 3),
          matrix_(1, 0), matrix_(1, 1), matrix_(1, 2), matrix_(1, 3),
          matrix_(2, 0), matrix_(2, 1), matrix_(2, 2), matrix_(2, 3);
  return data;
}

bool MatrixTransform::isValid(const double eps) const
{
  return isValidMatrix(matrix_, eps);
}

template<typename Derived>
bool MatrixTransform::isValidMatrix(const Eigen::MatrixBase<Derived> &matrix, const double eps)
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 4, 4);
  bool cond1 = isValidRotationMatrix(matrix.template block<3, 3>(0, 0), eps);
  bool cond2 = matrix.template block<1, 3>(3, 0).template lpNorm<Eigen::Infinity>() <= eps;
  bool cond3 = std::abs(matrix(3, 3) - 1.0) <= eps;
  return cond1 && cond2 && cond3;
}

template<typename Derived>
bool MatrixTransform::isValidRotationMatrix(const Eigen::MatrixBase<Derived> &matrix, const double eps)
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);
  bool cond1 = (matrix.transpose() * matrix -
    Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity()).template lpNorm<Eigen::Infinity>() <= eps;
  bool cond2 = std::abs(matrix.determinant() - 1.0) <= eps;
  return cond1 && cond2;
}

MatrixTransform& MatrixTransform::normalized_()
{
  Eigen::Matrix<double, 3, 3> rmat = matrix_.block<3, 3>(0, 0);
  std::tie(rmat, std::ignore, std::ignore) = decomposeRZS(rmat);
  matrix_.block<3, 3>(0, 0) = rmat;
  unsafe_ = false;
  return *this;
}

MatrixTransform MatrixTransform::normalized() const
{
  Eigen::Matrix<double, 3, 3> rmat = matrix_.block<3, 3>(0, 0);
  std::tie(rmat, std::ignore, std::ignore) = decomposeRZS(rmat);
  return {getTranslation(), rmat, false};
}

MatrixTransform& MatrixTransform::scaleTranslation_(double factor)
{
  matrix_.block<3, 1>(0, 3) *= factor;
  return *this;
}

MatrixTransform MatrixTransform::scaleTranslation(double factor) const
{
  return MatrixTransform(*this).scaleTranslation_(factor);
}

template<typename Derived>
MatrixTransform& MatrixTransform::setMatrix(const Eigen::MatrixBase<Derived> &matrix, const bool unsafe)
{
  // unsafe
  bool new_unsafe = unsafe_ || unsafe;

  // set
  if constexpr (Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4)
  {
    // create MatrixTransform from a homogeneous SE(3) transformation matrix
    if (new_unsafe || isValidMatrix(matrix, kDefaultEps))
    {
      matrix_ = matrix;
    }
    else
    {
      throw InvalidTransformException("Input matrix is not valid");
    }
  }
  else if constexpr (Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 4)
  {
    // create MatrixTransform from upper part of a homogeneous SE(3) transformation matrix
    if (new_unsafe || isValidRotationMatrix(matrix.template block<3, 3>(0, 0), kDefaultEps))
    {
      matrix_.setIdentity();
      matrix_.block<3, 4>(0, 0) = matrix;
    }
    else
    {
      throw InvalidTransformException("Input matrix is not valid");
    }
  }
  else
  {
    static_assert(always_false<Derived>, "Input matrix dimensions not supported");
  }

  // return
  unsafe_ = new_unsafe;
  return *this;
}

MatrixTransform& MatrixTransform::setTranslation(const Eigen::Matrix<double, 3, 1> &translation, const bool unsafe)
{
  // unsafe
  bool new_unsafe = unsafe_ || unsafe;

  // set
  matrix_.block<3, 1>(0, 3) = translation;

  // return
  unsafe_ = new_unsafe;
  return *this;
}

MatrixTransform& MatrixTransform::setRotationMatrix(const Eigen::Matrix<double, 3, 3> &rotation_matrix, const bool unsafe)
{
  // unsafe
  bool new_unsafe = unsafe_ || unsafe;

  // set
  if (new_unsafe || isValidRotationMatrix(rotation_matrix, kDefaultEps))
  {
    matrix_.block<3, 3>(0, 0) = rotation_matrix;
  }
  else
  {
    throw InvalidTransformException("Input rotation matrix is not valid");
  }

  // return
  unsafe_ = new_unsafe;
  return *this;
}

std::ostream& MatrixTransform::stream(std::ostream& os) const
{
  return os << "MatrixTransform(matrix=[" <<
            "[" << matrix_.block<1, 4>(0, 0) << "], " <<
            "[" << matrix_.block<1, 4>(1, 0) << "], " <<
            "[" << matrix_.block<1, 4>(2, 0) << "], " <<
            "[" << matrix_.block<1, 4>(3, 0) << "]])";
}

} // namespace motion3d
