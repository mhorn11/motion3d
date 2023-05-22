#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <motion3d/common/math.hpp>
#include <motion3d/common/type_traits.hpp>
#include <motion3d/utils/conversion.hpp>
#include <motion3d/utils/templating.hpp>

namespace motion3d
{

/** Exception thrown when performing operations including non-unsafe and invalid transforms. */
struct InvalidTransformException : public MessageException
{
  explicit InvalidTransformException(std::string msg) : MessageException(std::move(msg)) {}
};

/** Exception thrown when using an invalid transform type, e.g., when providing an invalid <CODE>char</CODE>. */
struct InvalidTransformTypeException : public std::exception
{
  const char* what() const noexcept override
  {
    return "Invalid transform type provided";
  }
};

/** Exception thrown when a transform is created from binary or vector data of the wrong size. */
struct InvalidTypeSizeException : public std::exception
{
  const char* what() const noexcept override
  {
    return "Transform created from binary or vector data of the wrong size";
  }
};


/** Transformation representation type. */
enum class TransformType
{
  kAxisAngle,
  kDualQuaternion,
  kEuler,
  kMatrix,
  kQuaternion,
};


class AxisAngleTransform;
class DualQuaternionTransform;
class EulerTransform;
class MatrixTransform;
class QuaternionTransform;


namespace internal
{
  constexpr static std::uint32_t kVectorToBinary = sizeof(double) / sizeof(std::uint8_t);

  template <>
  struct traits<AxisAngleTransform>
  {
    constexpr static char kName = 'A';
    constexpr static TransformType kTType = TransformType::kAxisAngle;
    constexpr static std::uint32_t kVectorSize = 7;
    constexpr static std::uint32_t kBinarySize = kVectorToBinary * kVectorSize;
  };

  template <>
  struct traits<DualQuaternionTransform>
  {
    constexpr static char kName = 'D';
    constexpr static TransformType kTType = TransformType::kDualQuaternion;
    constexpr static std::uint32_t kVectorSize = 8;
    constexpr static std::uint32_t kBinarySize = kVectorToBinary * kVectorSize;
  };

  template <>
  struct traits<EulerTransform>
  {
    constexpr static char kName = 'E';
    constexpr static TransformType kTType = TransformType::kEuler;
    constexpr static std::uint32_t kVectorSize = 7;
    constexpr static std::uint32_t kBinarySize = kVectorToBinary * kVectorSize;
  };

  template <>
  struct traits<MatrixTransform>
  {
    constexpr static char kName = 'M';
    constexpr static TransformType kTType = TransformType::kMatrix;
    constexpr static std::uint32_t kVectorSize = 12;
    constexpr static std::uint32_t kBinarySize = kVectorToBinary * kVectorSize;
  };

  template <>
  struct traits<QuaternionTransform>
  {
    constexpr static char kName = 'Q';
    constexpr static TransformType kTType = TransformType::kQuaternion;
    constexpr static std::uint32_t kVectorSize = 7;
    constexpr static std::uint32_t kBinarySize = kVectorToBinary * kVectorSize;
  };
} // end namespace internal


/** \returns the respective TransformType for the <CODE>char</CODE> <I>name</I>.
  * \throws InvalidTransformTypeException if <I>name</I> is invalid.
  */
TransformType transformTypeFromChar(const char &name)
{
  switch (std::toupper(name))
  {
    case internal::traits<AxisAngleTransform>::kName: return internal::traits<AxisAngleTransform>::kTType;
    case internal::traits<DualQuaternionTransform>::kName: return internal::traits<DualQuaternionTransform>::kTType;
    case internal::traits<EulerTransform>::kName: return internal::traits<EulerTransform>::kTType;
    case internal::traits<MatrixTransform>::kName: return internal::traits<MatrixTransform>::kTType;
    case internal::traits<QuaternionTransform>::kName: return internal::traits<QuaternionTransform>::kTType;
  }
  throw InvalidTransformTypeException();
}

/** \returns the respective char for TransformType <I>type</I>.
  * \throws InvalidTransformTypeException if <I>type</I> is invalid.
  */
char transformTypeToChar(const TransformType &type)
{
  switch (type)
  {
    case internal::traits<AxisAngleTransform>::kTType: return internal::traits<AxisAngleTransform>::kName;
    case internal::traits<DualQuaternionTransform>::kTType: return internal::traits<DualQuaternionTransform>::kName;
    case internal::traits<EulerTransform>::kTType: return internal::traits<EulerTransform>::kName;
    case internal::traits<MatrixTransform>::kTType: return internal::traits<MatrixTransform>::kName;
    case internal::traits<QuaternionTransform>::kTType: return internal::traits<QuaternionTransform>::kName;
  }
  throw InvalidTransformTypeException();
}

/** \returns the respective data vector size for TransformType <I>type</I>. */
std::uint32_t getVectorSize(const TransformType &type)
{
  switch (type)
  {
    case internal::traits<AxisAngleTransform>::kTType: return internal::traits<AxisAngleTransform>::kVectorSize;
    case internal::traits<DualQuaternionTransform>::kTType: return internal::traits<DualQuaternionTransform>::kVectorSize;
    case internal::traits<EulerTransform>::kTType: return internal::traits<EulerTransform>::kVectorSize;
    case internal::traits<MatrixTransform>::kTType: return internal::traits<MatrixTransform>::kVectorSize;
    case internal::traits<QuaternionTransform>::kTType: return internal::traits<QuaternionTransform>::kVectorSize;
  }
  return 0;
}

/** \returns the respective binary vector size for TransformType <I>type</I>. */
std::uint32_t getBinarySize(const TransformType &type)
{
  switch (type)
  {
    case internal::traits<AxisAngleTransform>::kTType: return internal::traits<AxisAngleTransform>::kBinarySize;
    case internal::traits<DualQuaternionTransform>::kTType: return internal::traits<DualQuaternionTransform>::kBinarySize;
    case internal::traits<EulerTransform>::kTType: return internal::traits<EulerTransform>::kBinarySize;
    case internal::traits<MatrixTransform>::kTType: return internal::traits<MatrixTransform>::kBinarySize;
    case internal::traits<QuaternionTransform>::kTType: return internal::traits<QuaternionTransform>::kBinarySize;
  }
  return 0;
}


/** \brief Interface class for generic access to transformations */
// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class TransformInterface
{
 public:
  DEFINE_POINTERS(TransformInterface);
  
  virtual ~TransformInterface() = default;

  /** Constructs and initializes a copy of <CODE>*this</CODE>. */
  TransformInterface::Ptr copy() const { return this->copyImpl(); };

  /** Set <CODE>*this</CODE> to an identity transform. */
  TransformInterface::Ptr setIdentity() { return this->setIdentityImpl(); };

  /** Constructs and initializes an identity transform with the same type as <CODE>*this</CODE>. */
  TransformInterface::Ptr identity() const { return this->identityImpl(); };

  /** \returns the transform type of <CODE>*this</CODE>. */
  virtual TransformType getType() const = 0;

  /** \returns if the transform type of <CODE>*this</CODE> is <I>type</I>. */
  virtual bool isType(const TransformType &type) const = 0;
  
  /** Converts <CODE>*this</CODE> to transform type <I>type</I>. */
  virtual TransformInterface::Ptr asType(const TransformType &type) const = 0;

  /** Converts <CODE>*this</CODE> to transform type <I>T</I>. */
  template<typename T>
  std::shared_ptr<T> asType() const;

  /** Inplace variant of inverse(). */
  TransformInterface::Ptr inverse_() { return this->inverseImpl_(); };

  /** \returns the inverse transform to <CODE>*this</CODE>. */
  TransformInterface::Ptr inverse() const { return this->inverseImpl(); };

  /** Inplace variant of normalized(). */
  TransformInterface::Ptr normalized_() { return this->normalizedImpl_(); };

  /** Normalizes <CODE>*this</CODE> to a valid transform and sets <CODE>unsafe</CODE> to <CODE>false</CODE>. */
  TransformInterface::Ptr normalized() const { return this->normalizedImpl(); };

  /** Inplace variant of scaleTranslation(). */
  TransformInterface::Ptr scaleTranslation_(double factor) { return this->scaleTranslationImpl_(factor); };

  /** Scales the translation by <I>factor</I>. */
  TransformInterface::Ptr scaleTranslation(double factor) const { return this->scaleTranslationImpl(factor); };

  /** Inplace variant of applyPre(const TransformInterface::ConstPtr&). */
  virtual TransformInterface::Ptr applyPre_(const TransformInterface::ConstPtr &other) { return this->applyPreImpl_(other); };

  /** Applies <I>other</I> transform \f$T_\mathrm{o}\f$ before <CODE>*this</CODE> transform \f$T\f$:
    * \f[T_\mathrm{o} \circ T\f]
    */
  virtual TransformInterface::Ptr applyPre(const TransformInterface::ConstPtr &other) const { return this->applyPreImpl(other); };

  /** Inplace variant of applyPost(const TransformInterface::ConstPtr&). */
  virtual TransformInterface::Ptr applyPost_(const TransformInterface::ConstPtr &other) { return this->applyPostImpl_(other); };

  /** Applies <I>other</I> transform \f$T_\mathrm{o}\f$ after <CODE>*this</CODE> transform \f$T\f$:
    * \f[T \circ T_\mathrm{o}\f]
    */
  virtual TransformInterface::Ptr applyPost(const TransformInterface::ConstPtr &other) const { return this->applyPostImpl(other); };

  /** Inplace variant of applyPre(const TransformInterface&). */
  virtual TransformInterface::Ptr applyPre_(const TransformInterface &other) = 0;

  /** \sa applyPre(const TransformInterface::ConstPtr&) const */
  virtual TransformInterface::Ptr applyPre(const TransformInterface &other) const = 0;

  /** Inplace variant of applyPost(const TransformInterface&). */
  virtual TransformInterface::Ptr applyPost_(const TransformInterface &other) = 0;

  /** \sa applyPost(const TransformInterface::ConstPtr&) const */
  virtual TransformInterface::Ptr applyPost(const TransformInterface &other) const = 0;

  /** Applies <CODE>*this</CODE> on 3D <I>point</I>. */
  virtual Eigen::Matrix<double, 3, 1> transformPoint(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>> &point) const = 0;

  /** Applies <CODE>*this</CODE> on 3D point <I>cloud</I>. */
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> transformCloud(const Eigen::Ref<const Eigen::Matrix<double, 3, Eigen::Dynamic>> &cloud) const = 0;

  /** Converts <CODE>*this</CODE> to a binary vector. */
  virtual std::vector<std::uint8_t> toBinary() const = 0;

  /** Converts <CODE>*this</CODE> to an std vector. */
  virtual std::vector<double> toVector() const = 0;

  /** Converts <CODE>*this</CODE> to an Eigen vector. */
  virtual Eigen::Matrix<double, Eigen::Dynamic, 1> toEigenVector() const = 0;

  /** \returns if <CODE>*this</CODE> is marked unsafe, meaning that the validity check is skipped at each operation. */
  virtual bool isUnsafe() const = 0;

  /** \returns if <CODE>*this</CODE> is valid using a default <I>eps</I> for isValid(double) const. */
  virtual bool isValid() const = 0;

  /** \returns if <CODE>*this</CODE> is valid w.r.t the threshold <I>eps</I>. */
  virtual bool isValid(double eps) const = 0;
  
  /** \returns if <CODE>*this</CODE> is equal to <I>other</I> using a default <I>eps</I>
    * for isEqual(const TransformInterface::ConstPtr&, double) const.
    * */
  virtual bool isEqual(const TransformInterface::ConstPtr &other) const = 0;

  /** \returns if <CODE>*this</CODE> is equal to <I>other</I> w.r.t the threshold <I>eps</I>. */
  virtual bool isEqual(const TransformInterface::ConstPtr &other, double eps) const = 0;

  /** \returns the rotation norm in <I>rad</I>. */
  virtual double rotationNorm() const = 0;

  /** \returns the translation norm. */
  virtual double translationNorm() const = 0;

  /** \returns a description of <CODE>*this</CODE>. */
  virtual std::string desc() const = 0;

  /** Inserts a description of <CODE>*this</CODE>. */
  virtual std::ostream& stream(std::ostream& os) const = 0;

  /** Constructs and initializes a transform of TransformType <I>type</I>.
    * \throws InvalidTypeSizeException for an invalid vector size.
    */
  template<typename ...Args>
  static TransformInterface::Ptr Factory(const TransformType &type, Args&&... args);

 private:
  virtual TransformInterface::Ptr copyImpl() const = 0;

  virtual TransformInterface::Ptr setIdentityImpl() = 0;
  virtual TransformInterface::Ptr identityImpl() const = 0;

  virtual TransformInterface::Ptr inverseImpl_() = 0;
  virtual TransformInterface::Ptr inverseImpl() const = 0;

  virtual TransformInterface::Ptr normalizedImpl_() = 0;
  virtual TransformInterface::Ptr normalizedImpl() const = 0;

  virtual TransformInterface::Ptr scaleTranslationImpl_(double factor) = 0;
  virtual TransformInterface::Ptr scaleTranslationImpl(double factor) const = 0;

  virtual TransformInterface::Ptr applyPreImpl_(const TransformInterface::ConstPtr &other) = 0;
  virtual TransformInterface::Ptr applyPreImpl(const TransformInterface::ConstPtr &other) const = 0;

  virtual TransformInterface::Ptr applyPostImpl_(const TransformInterface::ConstPtr &other) = 0;
  virtual TransformInterface::Ptr applyPostImpl(const TransformInterface::ConstPtr &other) const = 0;
};

template<typename T>
std::shared_ptr<T> TransformInterface::asType() const
{
  if (TransformInterface::Ptr this_as_type = this->asType(internal::traits<T>::kTType))
  {
    return std::static_pointer_cast<T>(this_as_type);
  }
  return nullptr;
}

template<typename ...Args>
TransformInterface::Ptr TransformInterface::Factory(const TransformType &type, Args&&... args)
{
  switch (type)
  {
    case internal::traits<AxisAngleTransform>::kTType:
      return std::make_shared<AxisAngleTransform>(std::forward<Args>(args)...);
    case internal::traits<DualQuaternionTransform>::kTType:
      return std::make_shared<DualQuaternionTransform>(std::forward<Args>(args)...);
    case internal::traits<EulerTransform>::kTType:
      return std::make_shared<EulerTransform>(std::forward<Args>(args)...);
    case internal::traits<MatrixTransform>::kTType:
      return std::make_shared<MatrixTransform>(std::forward<Args>(args)...);
    case internal::traits<QuaternionTransform>::kTType:
      return std::make_shared<QuaternionTransform>(std::forward<Args>(args)...);
  }
  return nullptr;
}


/** \brief CRTP base class for transform classes. */
template <class Derived>
class TransformBase : public TransformInterface, public std::enable_shared_from_this<TransformBase<Derived>>
{
 public:
  friend Derived;

  constexpr static std::uint32_t kVectorSize = internal::traits<Derived>::kVectorSize;  ///< The data vector size for this transform type.
  constexpr static std::uint32_t kBinarySize = internal::traits<Derived>::kBinarySize;  ///< The binary vector size for this transform type.

  explicit TransformBase(bool unsafe);

  inline const Derived& derived() const { return *static_cast<const Derived*>(this); }
  inline Derived& derived() { return *static_cast<Derived*>(this); }

  TransformType getType() const override { return internal::traits<Derived>::kTType; }

  bool isType(const TransformType &type) const override { return type == internal::traits<Derived>::kTType; }

  TransformInterface::Ptr asType(const TransformType &type) const override;

  TransformInterface::Ptr applyPre_(const TransformInterface &other) override;

  TransformInterface::Ptr applyPre(const TransformInterface &other) const override;

  TransformInterface::Ptr applyPost_(const TransformInterface &other) override;

  TransformInterface::Ptr applyPost(const TransformInterface &other) const override;

  Eigen::Matrix<double, 3, 1> transformPoint(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>> &point) const override;

  Eigen::Matrix<double, 3, Eigen::Dynamic> transformCloud(const Eigen::Ref<const Eigen::Matrix<double, 3, Eigen::Dynamic>> &cloud) const override;

  std::vector<std::uint8_t> toBinary() const override;

  bool isUnsafe() const override;

  bool isValid() const override { return derived().isValid(kDefaultEps); };

  bool isEqual(const TransformInterface::ConstPtr &other) const override { return this->isEqual(other, kDefaultEps); };

  bool isEqual(const TransformInterface::ConstPtr &other, double eps) const override;

  template<class Other>
  bool isEqual(const TransformBase<Other> &other, double eps = kDefaultEps) const;

 private:
  void checkValid() const;

  TransformInterface::Ptr copyImpl() const override { return std::make_shared<Derived>(derived().copy()); }

  TransformInterface::Ptr setIdentityImpl() override { return derived().setIdentity().shared_from_this(); }
  TransformInterface::Ptr identityImpl() const override { return std::make_shared<Derived>(); }

  TransformInterface::Ptr inverseImpl_() override { return derived().inverse_().shared_from_this(); }
  TransformInterface::Ptr inverseImpl() const override { return std::make_shared<Derived>(derived().inverse()); }

  TransformInterface::Ptr normalizedImpl_() override { return derived().normalized_().shared_from_this(); }
  TransformInterface::Ptr normalizedImpl() const override { return std::make_shared<Derived>(derived().normalized()); }

  TransformInterface::Ptr scaleTranslationImpl_(double factor) override { return derived().scaleTranslation_(factor).shared_from_this(); }
  TransformInterface::Ptr scaleTranslationImpl(double factor) const override { return std::make_shared<Derived>(derived().scaleTranslation(factor)); }

  TransformInterface::Ptr applyPreImpl_(const TransformInterface::ConstPtr &other) override { return derived().applyPre_(other).shared_from_this(); }
  TransformInterface::Ptr applyPreImpl(const TransformInterface::ConstPtr &other) const override { return std::make_shared<Derived>(derived().applyPre(other)); }

  TransformInterface::Ptr applyPostImpl_(const TransformInterface::ConstPtr &other) override { return derived().applyPost_(other).shared_from_this(); }
  TransformInterface::Ptr applyPostImpl(const TransformInterface::ConstPtr &other) const override { return std::make_shared<Derived>(derived().applyPost(other)); }

  bool unsafe_ = false;  ///< For unsafe transforms, the validity check is skipped at each operation.
};

template <class Derived>
TransformBase<Derived>::TransformBase(const bool unsafe)
  : unsafe_(unsafe)
{
}

template <class Derived>
std::vector<std::uint8_t> TransformBase<Derived>::toBinary() const
{
  return convertVector<double, std::uint8_t>(toVector());
}

template <class Derived>
bool TransformBase<Derived>::isUnsafe() const
{
  return unsafe_;
}

template <class Derived>
void TransformBase<Derived>::checkValid() const
{
  if (!unsafe_ && !isValid())
  {
    throw InvalidTransformException("Non-unsafe transform is not valid");
  }
}

} // namespace motion3d
