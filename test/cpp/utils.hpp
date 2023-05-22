#pragma once

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#include <Eigen/Core>

#include <gtest/gtest.h>

#include <motion3d/transforms.hpp>

namespace m3d = motion3d;


/************************************************
 *  Tests for Eigen matrices.
 ***********************************************/
namespace Eigen
{

template<typename T> inline typename NumTraits<T>::Real test_precision() { return NumTraits<T>::dummy_precision(); }
template<> inline float test_precision<float>() { return 1e-3f; }
template<> inline double test_precision<double>() { return 1e-6; }
template<> inline long double test_precision<long double>() { return 1e-6l; }
template<> inline float test_precision<std::complex<float> >() { return test_precision<float>(); }
template<> inline double test_precision<std::complex<double> >() { return test_precision<double>(); }
template<> inline long double test_precision<std::complex<long double> >() { return test_precision<long double>(); }

template<typename T1,typename T2>
typename NumTraits<typename T1::RealScalar>::NonInteger test_relative_error(const EigenBase<T1> &a, const EigenBase<T2> &b)
{
  using std::sqrt;
  typedef typename NumTraits<typename T1::RealScalar>::NonInteger RealScalar;
  typename internal::nested_eval<T1,2>::type ea(a.derived());
  typename internal::nested_eval<T2,2>::type eb(b.derived());
  return sqrt(RealScalar((ea-eb).cwiseAbs2().sum()) / RealScalar((std::min)(eb.cwiseAbs2().sum(),ea.cwiseAbs2().sum())));
}

template<typename T>
typename NumTraits<typename T::Scalar>::Real get_test_precision(const T&, const typename T::Scalar* = 0)
{
  return test_precision<typename NumTraits<typename T::Scalar>::Real>();
}

template<typename Type1, typename Type2>
inline bool verifyIsEqual(const Type1 &a, const Type2 &b)
{
  bool ret = a.isApprox(b);
  if (!ret)
  {
    std::cerr << "Eigen matrices are not equal, relative error is: " << test_relative_error(a,b) << std::endl;
  }
  return ret;
}

template<typename Type1, typename Type2>
inline bool test_isApprox(const Type1 &a, const Type2 &b, typename Type1::Scalar* = 0)
{
  // isApprox does not work for comparison against the zero matrix or vector
  if (b.norm() < test_precision<typename Type1::Scalar>())
  {
    return (a - b).norm() < test_precision<typename Type1::Scalar>();
  }
  else
  {
    return a.isApprox(b, test_precision<typename Type1::Scalar>());
  }
}

template<typename Type1, typename Type2>
inline bool verifyIsApprox(const Type1 &a, const Type2 &b)
{
  bool ret = test_isApprox(a, b);
  if (!ret)
  {
    std::cerr << "Difference too large wrt tolerance " << get_test_precision(a)  << ", relative error is: " << test_relative_error(a,b) << std::endl;
  }
  return ret;
}

} // end namespace Eigen


#define EIGEN_ASSERT_EQ(a, b) ASSERT_TRUE(Eigen::verifyIsEqual(a, b))
#define EIGEN_EXPECT_EQ(a, b) EXPECT_TRUE(Eigen::verifyIsEqual(a, b))
#define EIGEN_ASSERT_APPROX(a, b) ASSERT_TRUE(Eigen::verifyIsApprox(a, b))
#define EIGEN_EXPECT_APPROX(a, b) EXPECT_TRUE(Eigen::verifyIsApprox(a, b))


/************************************************
 *  Tests for files.
 ***********************************************/
inline bool verifyFilesEqual(const std::string &filename_a, const std::string &filename_b)
{
  // check file a
  if (!std::filesystem::exists(filename_a))
  {
    std::cerr << "File A (" << filename_a << ") does not exist" << std::endl;
    return false;
  }

  // check file b
  if (!std::filesystem::exists(filename_b))
  {
    std::cerr << "File B (" << filename_b << ") does not exist" << std::endl;
    return false;
  }

  // read file a
  std::ifstream t_a(filename_a);
  std::stringstream buffer_a;
  buffer_a << t_a.rdbuf();

  // read file b
  std::ifstream t_b(filename_b);
  std::stringstream buffer_b;
  buffer_b << t_b.rdbuf();

  // compare files
  if (buffer_a.str() != buffer_b.str())
  {
    std::cerr << "Files are not equal" << std::endl;
    return false;
  }
  return true;
}

#define ASSERT_FILE_EQUAL(a, b) ASSERT_TRUE(verifyFilesEqual(a, b))
#define EXPECT_FILE_EQUAL(a, b) EXPECT_TRUE(verifyFilesEqual(a, b))


/************************************************
 *  Tests for vectors.
 ***********************************************/
template<typename T>
inline bool verifyVectorsEqual(const std::vector<T> &a, const std::vector<T> &b)
{
  if (a.size() != b.size())
  {
    std::cerr << "Vector sizes are not equal: " << a.size() << " != " << b.size() << std::endl;
    return false;
  }

  std::size_t non_matching = 0;
  for (std::size_t i = 0; i < a.size(); ++i)
  {
    bool is_equal = (a[i] == b[i]) || (std::isnan(a[i]) && std::isnan(b[i])) || (std::isinf(a[i]) && std::isinf(b[i]));
    if (!is_equal)
    {
      non_matching++;
    }
  }

  if (non_matching != 0)
  {
    std::cerr << "Not all entries are equal: " << non_matching << "/" << a.size() << " do not match" << std::endl;
    return false;
  }
  return true;
}

#define ASSERT_VECTOR_APPROX(a, b) ASSERT_TRUE(verifyVectorsEqual(a, b))
#define EXPECT_VECTOR_APPROX(a, b) EXPECT_TRUE(verifyVectorsEqual(a, b))


/************************************************
 *  Tests for strings.
 ***********************************************/
inline bool verifyStringStartsWith(const std::string &a, const std::string &b)
{
  bool ret = (a.find(b) == 0);
  if (!ret)
  {
    std::cerr << "String '" << a  << "' does not start with '" << b << "'" << std::endl;
  }
  return ret;
}

#define ASSERT_STRING_STARTS_WITH(a, b) ASSERT_TRUE(verifyStringStartsWith(a, b))
#define EXPECT_STRING_STARTS_WITH(a, b) EXPECT_TRUE(verifyStringStartsWith(a, b))


/************************************************
 *  Tests for rotation matrices.
 ***********************************************/
template<typename _Scalar>
inline bool isValidRotationMatrix(const Eigen::Matrix<_Scalar, 3, 3> &m)
{
  _Scalar orthogonal_error = (m.transpose() * m - Eigen::Matrix<_Scalar, 3, 3>::Identity()).template lpNorm<Eigen::Infinity>();
  bool is_orthogonal = orthogonal_error < m3d::kDefaultEps;
  if (!is_orthogonal)
  {
    std::cerr << "Matrix is not orthogonal, error: " << orthogonal_error << std::endl;
  }

  _Scalar det = m.determinant();
  bool correct_determinant = std::abs(det - 1.0) < m3d::kDefaultEps;
  if (!correct_determinant)
  {
    std::cerr << "Matrix determinant is not 1.0 (" << det << ")" << std::endl;
  }

  return is_orthogonal && correct_determinant;
}

#define ASSERT_VALID_ROTATION_MATRIX(m) ASSERT_TRUE(isValidRotationMatrix(m))
#define EXPECT_VALID_ROTATION_MATRIX(m) EXPECT_TRUE(isValidRotationMatrix(m))


/************************************************
 *  Tests for quaternions.
 ***********************************************/
#define ASSERT_QUAT_EQ(a, b) EIGEN_ASSERT_EQ(a.coeffs(), b.coeffs())
#define EXPECT_QUAT_EQ(a, b) EIGEN_EXPECT_EQ(a.coeffs(), b.coeffs())
#define ASSERT_QUAT_APPROX(a, b) EIGEN_ASSERT_APPROX(a.coeffs(), b.coeffs())
#define EXPECT_QUAT_APPROX(a, b) EIGEN_EXPECT_APPROX(a.coeffs(), b.coeffs())


/************************************************
 *  Tests for dual quaternions.
 ***********************************************/
template<typename Type1, typename Type2>
bool verifyDualQuaternionMatrixIsApprox(const Type1 &a, const Type2 &b)
{
  bool ret = Eigen::test_isApprox(a, b) || Eigen::test_isApprox(a, -b);
  if(!ret)
  {
    auto rel_error1 = test_relative_error(a, b);
    auto rel_error2 = test_relative_error(a, -b);
    if (rel_error1 < rel_error2)
    {
      std::cerr << "Difference too large wrt tolerance " << get_test_precision(a)  << ", relative error is: " << rel_error1 << std::endl;
    }
    else
    {
      std::cerr << "Difference too large wrt tolerance " << get_test_precision(a)  << ", relative error is: " << rel_error2 << std::endl;
    }
  }
  return ret;
}

#define ASSERT_DQ_EQ(a, b) EIGEN_ASSERT_EQ(a.toEigenVector(), b.toEigenVector())
#define EXPECT_DQ_EQ(a, b) EIGEN_EXPECT_EQ(a.toEigenVector(), b.toEigenVector())
#define ASSERT_DQ_MAT_APPROX(a, b) ASSERT_TRUE(verifyDualQuaternionMatrixIsApprox(a, b))
#define EXPECT_DQ_MAT_APPROX(a, b) EXPECT_TRUE(verifyDualQuaternionMatrixIsApprox(a, b))
#define ASSERT_DQ_APPROX(a, b) ASSERT_DQ_MAT_APPROX(a.toEigenVector(), b.toEigenVector())
#define EXPECT_DQ_APPROX(a, b) EXPECT_DQ_MAT_APPROX(a.toEigenVector(), b.toEigenVector())


/************************************************
 *  Tests for transforms.
 ***********************************************/
bool verifyTransformIsApprox(const m3d::TransformInterface::ConstPtr &a, const m3d::TransformInterface::ConstPtr &b)
{
  return verifyDualQuaternionMatrixIsApprox(
    a->asType<m3d::DualQuaternionTransform>()->toEigenVector(),
    b->asType<m3d::DualQuaternionTransform>()->toEigenVector());
}

#define ASSERT_TRANSFORM_APPROX(a, b) ASSERT_TRUE(verifyTransformIsApprox(a, b))
#define EXPECT_TRANSFORM_APPROX(a, b) EXPECT_TRUE(verifyTransformIsApprox(a, b))


/************************************************
 *  Convert Eigen matrix to std vector.
 ***********************************************/
inline std::vector<double> eigenToVector(const Eigen::Matrix<double, Eigen::Dynamic, 1> &x)
{
  return std::vector<double>(x.data(), x.data() + x.rows());
}


/************************************************
 *  Print vector to output stream.
 ***********************************************/
template<typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v) 
{
  os << "[";
  for (std::size_t i = 0; i < v.size(); ++i)
  {
    os << std::to_string(v[i]);
    if (i != v.size() - 1) 
    {
      os << ", ";
    }
  }
  os << "]";
  return os;
}


/************************************************
 *  Create a random valid transformation in Euler format.
 ***********************************************/
m3d::EulerTransform::Ptr createRandomEulerTransformInterface()
{
  Eigen::Vector3d translation = (Eigen::Vector3d::Random() * 20.0).array() - 10.0;
  Eigen::Vector3d euler_angles = (Eigen::Vector3d::Random() * 2 * M_PI).array() - M_PI;
  return std::make_shared<m3d::EulerTransform>(translation, euler_angles(0), euler_angles(1), euler_angles(2),
    m3d::EulerAxes::kSXYZ);
}


/************************************************
 *  Number of random runs for each test.
 ***********************************************/
constexpr std::size_t kRandomRuns = 100;
