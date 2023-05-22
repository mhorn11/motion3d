#pragma once

#include <memory>

namespace motion3d
{

// NOLINTBEGIN(*-macro-usage)
#define DEFINE_POINTERS(cls) \
  using Ptr = std::shared_ptr<cls>; \
  using ConstPtr = std::shared_ptr<const cls>;
// NOLINTEND(*-macro-usage)

template<typename ...T>
void ignore(const T&...)  // NOLINT(readability-named-parameter)
{
}

template<typename T> struct is_shared_ptr : std::false_type {};
template<typename T> struct is_shared_ptr<std::shared_ptr<T>> : std::true_type {};

template <class...> constexpr std::false_type always_false{};

/** Checks if the Eigen <I>matrix</I> is a vector with the correct <I>Size</I>. */
template<int Size, typename Derived>
bool checkEigenVectorSize(const Eigen::DenseBase<Derived> &matrix)
{
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
  if constexpr (Derived::SizeAtCompileTime == Eigen::Dynamic)
  {
    // runtime check since size is dynamic
    return (matrix.rows() == 1 || matrix.cols() == 1) && matrix.size() == Size;
  }
  else
  {
    // compile time check of vector size
    return Derived::SizeAtCompileTime == Size;
  }
}

} // namespace motion3d
