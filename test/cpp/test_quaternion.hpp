#pragma once

#include <gtest/gtest.h>

#include <cmath>

#include <motion3d/common.hpp>

#include "utils.hpp"

namespace m3d = motion3d;


template<typename Scalar>
void testQuaternionInit(void)
{
  typedef m3d::Quaternion<Scalar> Quaternionx;
  typedef Eigen::Quaternion<Scalar> EigenQuaternionx;
  typedef typename Quaternionx::Matrix3 Matrix3;

  // zero
  Quaternionx q1 = Quaternionx::Zero();
  EXPECT_EQ(q1.w(), 0.0);
  EXPECT_EQ(q1.x(), 0.0);
  EXPECT_EQ(q1.y(), 0.0);
  EXPECT_EQ(q1.z(), 0.0);
  q1.coeffs().setRandom();
  q1.setZero();
  EXPECT_EQ(q1.w(), 0.0);
  EXPECT_EQ(q1.x(), 0.0);
  EXPECT_EQ(q1.y(), 0.0);
  EXPECT_EQ(q1.z(), 0.0);

  // identity
  Quaternionx q2 = Quaternionx::Identity();
  EXPECT_EQ(q2.w(), 1.0);
  EXPECT_EQ(q2.x(), 0.0);
  EXPECT_EQ(q2.y(), 0.0);
  EXPECT_EQ(q2.z(), 0.0);
  q2.coeffs().setRandom();
  q2.setIdentity();
  EXPECT_EQ(q2.w(), 1.0);
  EXPECT_EQ(q2.x(), 0.0);
  EXPECT_EQ(q2.y(), 0.0);
  EXPECT_EQ(q2.z(), 0.0);

  // values
  Quaternionx q3(1.0, 2.0, 3.0, 4.0);
  EXPECT_EQ(q3.w(), 1.0);
  EXPECT_EQ(q3.x(), 2.0);
  EXPECT_EQ(q3.y(), 3.0);
  EXPECT_EQ(q3.z(), 4.0);

  // Eigen copy
  EigenQuaternionx q_eig(q3);
  Quaternionx q5(q_eig);
  EXPECT_EQ(q5.w(), 1.0);
  EXPECT_EQ(q5.x(), 2.0);
  EXPECT_EQ(q5.y(), 3.0);
  EXPECT_EQ(q5.z(), 4.0);

  // Eigen assignment
  Quaternionx q6 = q_eig;
  EXPECT_EQ(q6.w(), 1.0);
  EXPECT_EQ(q6.x(), 2.0);
  EXPECT_EQ(q6.y(), 3.0);
  EXPECT_EQ(q6.z(), 4.0);

  // Rotation matrix
  auto identity = Matrix3::Identity();
  Quaternionx q7 = Quaternionx::FromRotationMatrix(identity);
  EXPECT_EQ(q7.w(), 1.0);
  EXPECT_EQ(q7.x(), 0.0);
  EXPECT_EQ(q7.y(), 0.0);
  EXPECT_EQ(q7.z(), 0.0);
}

TEST(Quaternion, init)
{
  testQuaternionInit<float>();
  testQuaternionInit<double>();
}


template<typename Scalar>
void testQuaternionMemberAccess(void)
{
  typedef m3d::Quaternion<Scalar> Quaternionx;

  // create data
  Quaternionx q = Quaternionx::Zero();
  std::vector<Scalar> data = {1.0, 2.0, 3.0, 4.0};

  // assign
  q.w() = data[0];
  q.x() = data[1];
  q.y() = data[2];
  q.z() = data[3];
  EXPECT_QUAT_EQ(q, Quaternionx(data[0], data[1], data[2], data[3]));

  // modify inplace
  q.w() += data[0];
  q.x() += data[1];
  q.y() += data[2];
  q.z() += data[3];
  EXPECT_QUAT_EQ(q, Quaternionx(2 * data[0], 2 * data[1], 2 * data[2], 2 * data[3]));
}

TEST(Quaternion, memberAccess)
{
  testQuaternionMemberAccess<float>();
  testQuaternionMemberAccess<double>();
}


template<typename Scalar>
void testQuaternionInplaceReturn(void)
{
  typedef m3d::Quaternion<Scalar> Quaternionx;

  Quaternionx q(1.0, 2.0, 3.0, 4.0);

  ASSERT_TRUE(&q == &q.conjugate_());
  ASSERT_TRUE(&q == &q.inverse_());
  ASSERT_TRUE(&q == &q.normalized_());
}

TEST(Quaternion, inplaceReturn)
{
  testQuaternionInplaceReturn<float>();
  testQuaternionInplaceReturn<double>();
}


template<typename Scalar>
void testQuaternionMethods(void)
{
  typedef m3d::Quaternion<Scalar> Quaternionx;
  typedef Eigen::Quaternion<Scalar> EigenQuaternionx;

  Quaternionx q(1.0, 2.0, 3.0, 4.0);
  Scalar q_norm2 = 30.0;
  Scalar q_norm = sqrt(q_norm2);

  // conjugate
  EXPECT_QUAT_EQ(Quaternionx(q).conjugate_(), Quaternionx(1.0, -2.0, -3.0, -4.0));
  EXPECT_QUAT_EQ(q.conjugate(), Quaternionx(1.0, -2.0, -3.0, -4.0));

  // inverse
  Quaternionx q_inv_mult_inplace = Quaternionx(q).inverse_() * q;
  EXPECT_QUAT_APPROX(q_inv_mult_inplace, Quaternionx::Identity());

  Quaternionx q_inv_mult = q.inverse() * q;
  EXPECT_QUAT_APPROX(q_inv_mult, Quaternionx::Identity());

  Quaternionx q_zero = Quaternionx::Zero();
  EXPECT_THROW((q_zero.inverse_()), m3d::MathException);
  EXPECT_THROW((q_zero.inverse()), m3d::MathException);

  // norm
  EXPECT_NEAR(q.norm(), q_norm, 1e-6);
  EXPECT_EQ(q.squaredNorm(), q_norm2);

  // slerp
  Quaternionx q2(5.0, 6.0, 7.0, 8.0);
  EigenQuaternionx q_eig(q);
  EigenQuaternionx q2_eig(q2);
  Scalar factor = 0.7;

  Quaternionx q_slerp = q.slerp(factor, q2);
  EigenQuaternionx q_eig_slerp = q_eig.slerp(factor, q2_eig);
  EXPECT_QUAT_EQ(q_slerp, q_eig_slerp);

  // angular distance
  Quaternionx q_0deg(1.0, 0.0, 0.0, 0.0);
  Quaternionx q_180deg(0.0, 1.0, 0.0, 0.0);
  Scalar angular_dist = q_0deg.angularDistance(q_180deg);
  EXPECT_NEAR(angular_dist, M_PI, 1e-6);

  // rotation norm
  EXPECT_EQ(q_0deg.rotationNorm(), 0.0);
  EXPECT_NEAR(q_180deg.rotationNorm(), M_PI, 1e-6);
}

TEST(Quaternion, methods)
{
  testQuaternionMethods<float>();
  testQuaternionMethods<double>();
}


template<typename Scalar>
void testQuaternionIsEqual(void)
{
  typedef m3d::Quaternion<Scalar> Quaternionx;

  // epsilon for isEqual
  Scalar epsilon = 1e-4;

  // create quaternions
  Quaternionx q1(1.0, 2.0, 3.0, 4.0);

  Quaternionx q2(q1);
  q2.w() += epsilon / 10;

  Quaternionx q3(q1);
  q3.w() += epsilon * 10;

  Quaternionx q4(1e-4, 0.0, 0.0, 0.0);

  // compare
  EXPECT_TRUE(q1.isEqual(q1));
  EXPECT_FALSE(q1.isEqual(-q1));
  EXPECT_TRUE(q1.isEqual(q2, epsilon));
  EXPECT_FALSE(q1.isEqual(q3, epsilon));
  EXPECT_TRUE(q4.isEqual(Quaternionx::Zero(), 1e-3));
  EXPECT_FALSE(q4.isEqual(Quaternionx::Zero(), 1e-5));
}

TEST(Quaternion, isEqual)
{
  testQuaternionIsEqual<float>();
  testQuaternionIsEqual<double>();
}


template<typename Scalar>
void testQuaternionNormalized(void)
{
  typedef m3d::Quaternion<Scalar> Quaternionx;

  std::vector<Quaternionx > quaternions = {
    Quaternionx(1.0, 0.0, 0.0, 0.0),
    Quaternionx(0.0, 1.0, 0.0, 0.0),
    Quaternionx(std::sqrt(0.5), -std::sqrt(0.5), 0.0, 0.0)
  };

  for (const auto& q : quaternions)
  {
    // inplace
    EXPECT_QUAT_APPROX(q, (Quaternionx(q) * 2.0).normalized_());
    EXPECT_QUAT_APPROX(q, (Quaternionx(q) * -0.5).normalized_());

    // non-inplace
    EXPECT_QUAT_APPROX(q, (q * 2.0).normalized());
    EXPECT_QUAT_APPROX(q, (q * -0.5).normalized());
  }
}

TEST(Quaternion, normalized)
{
  testQuaternionNormalized<float>();
  testQuaternionNormalized<double>();
}


template<typename Scalar>
void testQuaternionOperators(void)
{
  typedef m3d::Quaternion<Scalar> Quaternionx;
  typedef Eigen::Quaternion<Scalar> EigenQuaternionx;

  // create quaternions
  Quaternionx q1;
  Quaternionx q2;
  q1.coeffs().setRandom();
  q2.coeffs().setRandom();
  Quaternionx q1_copy(q1);
  EigenQuaternionx q1_eig(q1);
  EigenQuaternionx q2_eig(q2);

  // create scalar
  Scalar factor = 2.0;

  // multiply with scalar
  Quaternionx q1_mult1 = q1 * factor;
  EXPECT_EQ(q1_mult1.w(), q1.w() * factor);
  EXPECT_EQ(q1_mult1.x(), q1.x() * factor);
  EXPECT_EQ(q1_mult1.y(), q1.y() * factor);
  EXPECT_EQ(q1_mult1.z(), q1.z() * factor);
  ASSERT_QUAT_APPROX(q1, q1_copy);

  Quaternionx q1_mult2 = factor * q1;
  EXPECT_EQ(q1_mult2.w(), q1.w() * factor);
  EXPECT_EQ(q1_mult2.x(), q1.x() * factor);
  EXPECT_EQ(q1_mult2.y(), q1.y() * factor);
  EXPECT_EQ(q1_mult2.z(), q1.z() * factor);
  ASSERT_QUAT_APPROX(q1, q1_copy);

  Quaternionx q1_mult_copy(q1);
  q1_mult_copy *= factor;
  EXPECT_EQ(q1_mult_copy.w(), q1.w() * factor);
  EXPECT_EQ(q1_mult_copy.x(), q1.x() * factor);
  EXPECT_EQ(q1_mult_copy.y(), q1.y() * factor);
  EXPECT_EQ(q1_mult_copy.z(), q1.z() * factor);

  // multiply with quaternion
  Quaternionx q12_mult = q1 * q2;
  EigenQuaternionx q12_eig_mult = q1_eig * q2_eig;
  ASSERT_QUAT_APPROX(q12_mult, q12_eig_mult);
  ASSERT_QUAT_APPROX(q1, q1_copy);

  Quaternionx q12_mult_copy(q1);
  EigenQuaternionx q12_eig_mult_copy(q1_eig);
  q12_mult_copy *= q2;
  q12_eig_mult_copy *= q2_eig;
  ASSERT_QUAT_APPROX(q12_mult_copy, q12_eig_mult_copy);

  // divide
  Quaternionx q1_div = q1 / factor;
  EXPECT_EQ(q1_div.w(), q1.w() / factor);
  EXPECT_EQ(q1_div.x(), q1.x() / factor);
  EXPECT_EQ(q1_div.y(), q1.y() / factor);
  EXPECT_EQ(q1_div.z(), q1.z() / factor);
  ASSERT_QUAT_APPROX(q1, q1_copy);

  Quaternionx q1_div_copy(q1);
  q1_div_copy /= factor;
  EXPECT_EQ(q1_div_copy.w(), q1.w() / factor);
  EXPECT_EQ(q1_div_copy.x(), q1.x() / factor);
  EXPECT_EQ(q1_div_copy.y(), q1.y() / factor);
  EXPECT_EQ(q1_div_copy.z(), q1.z() / factor);

  // add
  Quaternionx q_sum = q1 + q2;
  EXPECT_EQ(q_sum.w(), q1.w() + q2.w());
  EXPECT_EQ(q_sum.x(), q1.x() + q2.x());
  EXPECT_EQ(q_sum.y(), q1.y() + q2.y());
  EXPECT_EQ(q_sum.z(), q1.z() + q2.z());
  ASSERT_QUAT_APPROX(q1, q1_copy);

  Quaternionx q_sum_copy(q1);
  q_sum_copy += q2;
  EXPECT_EQ(q_sum_copy.w(), q1.w() + q2.w());
  EXPECT_EQ(q_sum_copy.x(), q1.x() + q2.x());
  EXPECT_EQ(q_sum_copy.y(), q1.y() + q2.y());
  EXPECT_EQ(q_sum_copy.z(), q1.z() + q2.z());

  // subtract
  Quaternionx q_sub1 = -q1;
  EXPECT_EQ(q_sub1.w(), -q1.w());
  EXPECT_EQ(q_sub1.x(), -q1.x());
  EXPECT_EQ(q_sub1.y(), -q1.y());
  EXPECT_EQ(q_sub1.z(), -q1.z());
  ASSERT_QUAT_APPROX(q1, q1_copy);

  Quaternionx q_sub2 = q1 - q2;
  EXPECT_EQ(q_sub2.w(), q1.w() - q2.w());
  EXPECT_EQ(q_sub2.x(), q1.x() - q2.x());
  EXPECT_EQ(q_sub2.y(), q1.y() - q2.y());
  EXPECT_EQ(q_sub2.z(), q1.z() - q2.z());
  ASSERT_QUAT_APPROX(q1, q1_copy);

  Quaternionx q_sub_copy(q1);
  q_sub_copy -= q2;
  EXPECT_EQ(q_sub_copy.w(), q1.w() - q2.w());
  EXPECT_EQ(q_sub_copy.x(), q1.x() - q2.x());
  EXPECT_EQ(q_sub_copy.y(), q1.y() - q2.y());
  EXPECT_EQ(q_sub_copy.z(), q1.z() - q2.z());
}

TEST(Quaternion, operators)
{
  testQuaternionOperators<float>();
  testQuaternionOperators<double>();
}


template<typename Scalar>
void testQuaternionToVector(void)
{
  typedef m3d::Quaternion<Scalar> Quaternionx;
  typedef Eigen::Matrix<Scalar, 4, 1> Vectorx;

  // create quaternion
  Quaternionx q;
  q.coeffs().setRandom();

  // std vector
  std::vector<Scalar> std_vec = q.toVector();
  Quaternionx q_std = Quaternionx::FromVector(std_vec);

  ASSERT_EQ(std_vec.size(), 4);
  EXPECT_EQ(std_vec[0], q.w());
  EXPECT_EQ(std_vec[1], q.x());
  EXPECT_EQ(std_vec[2], q.y());
  EXPECT_EQ(std_vec[3], q.z());
  EXPECT_QUAT_APPROX(q, q_std);

  // eigen vector
  Vectorx eigen_vec = q.toEigenVector();
  Quaternionx q_eig = Quaternionx::FromVector(eigen_vec);

  EXPECT_EQ(eigen_vec(0), q.w());
  EXPECT_EQ(eigen_vec(1), q.x());
  EXPECT_EQ(eigen_vec(2), q.y());
  EXPECT_EQ(eigen_vec(3), q.z());
  EXPECT_QUAT_APPROX(q, q_eig);
}

TEST(Quaternion, toVector)
{
  testQuaternionToVector<float>();
  testQuaternionToVector<double>();
}


template<typename Scalar>
void testQuaternionToMatrix(void)
{
  typedef m3d::Quaternion<Scalar> Quaternionx;
  typedef Eigen::Matrix<Scalar, 4, 1> Vector4;

  // create normalized quaternions
  Quaternionx q1, q2;
  q1.coeffs().setRandom();
  q2.coeffs().setRandom();

  // chain with multiplication
  Quaternionx q3 = q1 * q2;
  
  // chain in matrix form
  Vector4 v3_pos = q1.toPositiveMatrix() * q2.toEigenVector();
  Vector4 v3_neg = q2.toNegativeMatrix() * q1.toEigenVector();

  // checks
  EIGEN_EXPECT_APPROX(q3.toEigenVector(), v3_pos);
  EIGEN_EXPECT_APPROX(q3.toEigenVector(), v3_neg);
}

TEST(Quaternion, toMatrix)
{
  testQuaternionToMatrix<float>();
  testQuaternionToMatrix<double>();
}


template<typename Scalar>
void testQuaternionDescription(void)
{
  auto q = m3d::Quaternion<Scalar>::Identity();

  // stream
  std::stringstream ss;
  ss << q;
  ASSERT_STRING_STARTS_WITH(ss.str(), "Quaternion");

  // description
  ASSERT_EQ(q.desc(), ss.str());
}

TEST(Quaternion, description)
{
  testQuaternionDescription<float>();
  testQuaternionDescription<double>();
}
