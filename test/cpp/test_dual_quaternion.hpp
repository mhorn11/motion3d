#pragma once

#include <gtest/gtest.h>

#include <cmath>

#include <motion3d/common.hpp>

#include "utils.hpp"

namespace m3d = motion3d;


template<typename Scalar>
void testDualQuaternionInit(void)
{
  typedef m3d::DualQuaternion<Scalar> DualQuaternionx;
  typedef m3d::Quaternion<Scalar> Quaternionx;
  typedef Eigen::Quaternion<Scalar> EigenQuaternionx;

  // zero
  DualQuaternionx q1 = DualQuaternionx::Zero();
  EXPECT_EQ(q1.real().w(), 0.0);
  EXPECT_EQ(q1.real().x(), 0.0);
  EXPECT_EQ(q1.real().y(), 0.0);
  EXPECT_EQ(q1.real().z(), 0.0);
  EXPECT_EQ(q1.dual().w(), 0.0);
  EXPECT_EQ(q1.dual().x(), 0.0);
  EXPECT_EQ(q1.dual().y(), 0.0);
  EXPECT_EQ(q1.dual().z(), 0.0);
  q1.real().coeffs() << 1.0, 2.0, 3.0, 4.0;
  q1.dual().coeffs() << 1.0, 2.0, 3.0, 4.0;
  q1.setZero();
  EXPECT_EQ(q1.real().w(), 0.0);
  EXPECT_EQ(q1.real().x(), 0.0);
  EXPECT_EQ(q1.real().y(), 0.0);
  EXPECT_EQ(q1.real().z(), 0.0);
  EXPECT_EQ(q1.dual().w(), 0.0);
  EXPECT_EQ(q1.dual().x(), 0.0);
  EXPECT_EQ(q1.dual().y(), 0.0);
  EXPECT_EQ(q1.dual().z(), 0.0);

  // identity
  DualQuaternionx q2 = DualQuaternionx::Identity();
  EXPECT_EQ(q2.real().w(), 1.0);
  EXPECT_EQ(q2.real().x(), 0.0);
  EXPECT_EQ(q2.real().y(), 0.0);
  EXPECT_EQ(q2.real().z(), 0.0);
  EXPECT_EQ(q2.dual().w(), 0.0);
  EXPECT_EQ(q2.dual().x(), 0.0);
  EXPECT_EQ(q2.dual().y(), 0.0);
  EXPECT_EQ(q2.dual().z(), 0.0);
  q2.real().coeffs() << 1.0, 2.0, 3.0, 4.0;
  q2.dual().coeffs() << 1.0, 2.0, 3.0, 4.0;
  q2.setIdentity();
  EXPECT_EQ(q2.real().w(), 1.0);
  EXPECT_EQ(q2.real().x(), 0.0);
  EXPECT_EQ(q2.real().y(), 0.0);
  EXPECT_EQ(q2.real().z(), 0.0);
  EXPECT_EQ(q2.dual().w(), 0.0);
  EXPECT_EQ(q2.dual().x(), 0.0);
  EXPECT_EQ(q2.dual().y(), 0.0);
  EXPECT_EQ(q2.dual().z(), 0.0);

  // values
  DualQuaternionx q3(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);
  EXPECT_EQ(q3.real().w(), 1.0);
  EXPECT_EQ(q3.real().x(), 2.0);
  EXPECT_EQ(q3.real().y(), 3.0);
  EXPECT_EQ(q3.real().z(), 4.0);
  EXPECT_EQ(q3.dual().w(), 5.0);
  EXPECT_EQ(q3.dual().x(), 6.0);
  EXPECT_EQ(q3.dual().y(), 7.0);
  EXPECT_EQ(q3.dual().z(), 8.0);

  // Quaternions
  Quaternionx q3_real = q3.real();
  Quaternionx q3_dual = q3.dual();
  DualQuaternionx q4(q3_real, q3_dual);
  EXPECT_EQ(q4.real().w(), 1.0);
  EXPECT_EQ(q4.real().x(), 2.0);
  EXPECT_EQ(q4.real().y(), 3.0);
  EXPECT_EQ(q4.real().z(), 4.0);
  EXPECT_EQ(q4.dual().w(), 5.0);
  EXPECT_EQ(q4.dual().x(), 6.0);
  EXPECT_EQ(q4.dual().y(), 7.0);
  EXPECT_EQ(q4.dual().z(), 8.0);

  // Eigen quaternions
  EigenQuaternionx q3_real_eig(q3_real);
  EigenQuaternionx q3_dual_eig(q3_dual);
  DualQuaternionx q5(q3_real_eig, q3_dual_eig);
  EXPECT_EQ(q5.real().w(), 1.0);
  EXPECT_EQ(q5.real().x(), 2.0);
  EXPECT_EQ(q5.real().y(), 3.0);
  EXPECT_EQ(q5.real().z(), 4.0);
  EXPECT_EQ(q5.dual().w(), 5.0);
  EXPECT_EQ(q5.dual().x(), 6.0);
  EXPECT_EQ(q5.dual().y(), 7.0);
  EXPECT_EQ(q5.dual().z(), 8.0);
}

TEST(DualQuaternion, init)
{
  testDualQuaternionInit<float>();
  testDualQuaternionInit<double>();
}


template<typename Scalar>
void testDualQuaternionInplaceReturn(void)
{
  typedef m3d::DualQuaternion<Scalar> DualQuaternionx;

  DualQuaternionx q(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);

  ASSERT_TRUE(&q == &q.quatConjugate_());
  ASSERT_TRUE(&q == &q.dualConjugate_());
  ASSERT_TRUE(&q == &q.combConjugate_());
  ASSERT_TRUE(&q == &q.inverse_());
  ASSERT_TRUE(&q == &q.normalized_());
}

TEST(DualQuaternion, inplaceReturn)
{
  testDualQuaternionInplaceReturn<float>();
  testDualQuaternionInplaceReturn<double>();
}


template<typename Scalar>
void testDualQuaternionMethods(void)
{
  typedef m3d::DualQuaternion<Scalar> DualQuaternionx;

  DualQuaternionx q(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);
  Scalar q_norm2 = 204.0;
  Scalar q_norm = sqrt(q_norm2);

  // quaternion conjugate
  EXPECT_DQ_EQ(DualQuaternionx(q).quatConjugate_(), DualQuaternionx(1.0, -2.0, -3.0, -4.0, 5.0, -6.0, -7.0, -8.0));
  EXPECT_DQ_EQ(q.quatConjugate(), DualQuaternionx(1.0, -2.0, -3.0, -4.0, 5.0, -6.0, -7.0, -8.0));

  // dual conjugate
  EXPECT_DQ_EQ(DualQuaternionx(q).dualConjugate_(), DualQuaternionx(1.0, 2.0, 3.0, 4.0, -5.0, -6.0, -7.0, -8.0));
  EXPECT_DQ_EQ(q.dualConjugate(), DualQuaternionx(1.0, 2.0, 3.0, 4.0, -5.0, -6.0, -7.0, -8.0));

  // combined conjugate
  EXPECT_DQ_EQ(DualQuaternionx(q).combConjugate_(), DualQuaternionx(1.0, -2.0, -3.0, -4.0, -5.0, 6.0, 7.0, 8.0));
  EXPECT_DQ_EQ(q.combConjugate(), DualQuaternionx(1.0, -2.0, -3.0, -4.0, -5.0, 6.0, 7.0, 8.0));

  // inverse
  DualQuaternionx q_inv_mult_inplace = DualQuaternionx(q).inverse_() * q;
  EXPECT_DQ_APPROX(q_inv_mult_inplace, DualQuaternionx::Identity());

  DualQuaternionx q_inv_mult = q.inverse() * q;
  EXPECT_DQ_APPROX(q_inv_mult, DualQuaternionx::Identity());

  // norm
  EXPECT_NEAR(q.norm(), q_norm, 1e-6);
  EXPECT_EQ(q.squaredNorm(), q_norm2);
}

TEST(DualQuaternion, methods)
{
  testDualQuaternionMethods<float>();
  testDualQuaternionMethods<double>();
}


template<typename Scalar>
void testDualQuaternionIsEqual(void)
{
  typedef m3d::DualQuaternion<Scalar> DualQuaternionx;

  // epsilon for isEqual
  Scalar epsilon = 1e-4;

  // create quaternions
  DualQuaternionx q1(1.0, 2.0, 3.0, 4.0, 1.0, 2.0, 3.0, 4.0);

  DualQuaternionx q2(q1);
  q2.real().w() += epsilon / 10;

  DualQuaternionx q3(q1);
  q3.dual().w() += epsilon / 10;

  DualQuaternionx q4(q1);
  q4.real().w() += epsilon * 10;

  DualQuaternionx q5(q1);
  q5.dual().w() += epsilon * 10;

  DualQuaternionx q6(1e-4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  DualQuaternionx q7(0.0, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0);

  // compare
  EXPECT_TRUE(q1.isEqual(q1));
  EXPECT_FALSE(q1.isEqual(-q1));
  EXPECT_TRUE(q1.isEqual(q2, epsilon));
  EXPECT_TRUE(q1.isEqual(q3, epsilon));
  EXPECT_FALSE(q1.isEqual(q4, epsilon));
  EXPECT_FALSE(q1.isEqual(q5, epsilon));
  EXPECT_TRUE(q6.isEqual(DualQuaternionx::Zero(), 1e-3));
  EXPECT_FALSE(q6.isEqual(DualQuaternionx::Zero(), 1e-5));
  EXPECT_TRUE(q7.isEqual(DualQuaternionx::Zero(), 1e-3));
  EXPECT_FALSE(q7.isEqual(DualQuaternionx::Zero(), 1e-5));
}

TEST(DualQuaternion, isEqual)
{
  testDualQuaternionIsEqual<float>();
  testDualQuaternionIsEqual<double>();
}


template<typename Scalar>
void testDualQuaternionNormalized(void)
{
  typedef m3d::DualQuaternion<Scalar> DualQuaternionx;

  std::vector<DualQuaternionx> dual_quaternions = {
    DualQuaternionx(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0, -3.0),
    DualQuaternionx(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    DualQuaternionx(std::sqrt(0.5), -std::sqrt(0.5), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  };

  // check rT * r = 1
  for (const auto& dq : dual_quaternions)
  {
    // inplace
    EXPECT_DQ_APPROX(dq, (DualQuaternionx(dq) * 2.0).normalized_());
    EXPECT_DQ_APPROX(dq, (DualQuaternionx(dq) * -0.5).normalized_());

    // non-inplace
    EXPECT_DQ_APPROX(dq, (dq * 2.0).normalized());
    EXPECT_DQ_APPROX(dq, (dq * -0.5).normalized());
  }

  // check rT * d = 0
  DualQuaternionx dq1(1.0, 0.0, 0.0, 0.0, 2.0, 1.0, 2.0, -3.0);
  DualQuaternionx dq1_norm(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0, -3.0);
  EXPECT_DQ_APPROX(dq1_norm, DualQuaternionx(dq1).normalized_());
  EXPECT_DQ_APPROX(dq1_norm, (DualQuaternionx(dq1) * 2.0).normalized_());
  EXPECT_DQ_APPROX(dq1_norm, (DualQuaternionx(dq1) * -0.5).normalized_());
  EXPECT_DQ_APPROX(dq1_norm, dq1.normalized());
  EXPECT_DQ_APPROX(dq1_norm, (dq1 * 2.0).normalized());
  EXPECT_DQ_APPROX(dq1_norm, (dq1 * -0.5).normalized());
}

TEST(DualQuaternion, normalized)
{
  testDualQuaternionNormalized<float>();
  testDualQuaternionNormalized<double>();
}


template<typename Scalar>
void testDualQuaternionOperators(void)
{
  typedef m3d::DualQuaternion<Scalar> DualQuaternionx;
  typedef m3d::Quaternion<Scalar> Quaternionx;

  // create quaternions
  DualQuaternionx q1;
  DualQuaternionx q2;
  q1.real().coeffs() << 1.0, -2.0, 3.0, -4.0;
  q1.dual().coeffs() << 5.0, -6.0, 7.0, -8.0;
  q2.real().coeffs() << 9.0, -10.0, 11.0, -12.0;
  q2.dual().coeffs() << 13.0, -14.0, 15.0, -16.0;
  DualQuaternionx q1_copy(q1);

  // create scalar
  Scalar factor = 2.0;

  // multiply with scalar
  DualQuaternionx q1_mult1 = q1 * factor;
  EXPECT_DQ_EQ(q1_mult1,
    DualQuaternionx(q1.real().w() * factor, q1.real().x() * factor, q1.real().y() * factor, q1.real().z() * factor,
                    q1.dual().w() * factor, q1.dual().x() * factor, q1.dual().y() * factor, q1.dual().z() * factor));
  EXPECT_DQ_EQ(q1, q1_copy);

  DualQuaternionx q1_mult2 = factor * q1;
  EXPECT_DQ_EQ(q1_mult2,
    DualQuaternionx(q1.real().w() * factor, q1.real().x() * factor, q1.real().y() * factor, q1.real().z() * factor,
                    q1.dual().w() * factor, q1.dual().x() * factor, q1.dual().y() * factor, q1.dual().z() * factor));
  EXPECT_DQ_EQ(q1, q1_copy);

  DualQuaternionx q1_mult_copy(q1);
  q1_mult_copy *= factor;
  EXPECT_DQ_EQ(q1_mult_copy, 
    DualQuaternionx(q1.real().w() * factor, q1.real().x() * factor, q1.real().y() * factor, q1.real().z() * factor,
                    q1.dual().w() * factor, q1.dual().x() * factor, q1.dual().y() * factor, q1.dual().z() * factor));

  // multiply with dual quaternion
  Quaternionx q12_mult_real = q1.real() * q2.real();
  Quaternionx q12_mult_dual = (q1.real() * q2.dual()) + (q1.dual() * q2.real());

  DualQuaternionx q12_mult = q1 * q2;
  EXPECT_DQ_EQ(q12_mult, DualQuaternionx(q12_mult_real, q12_mult_dual));
  EXPECT_DQ_EQ(q1, q1_copy);

  DualQuaternionx q12_mult_copy(q1);
  q12_mult_copy *= q2;
  EXPECT_DQ_EQ(q12_mult, DualQuaternionx(q12_mult_real, q12_mult_dual));

  // divide
  DualQuaternionx q1_div = q1 / factor;
  EXPECT_DQ_EQ(q1_div,
    DualQuaternionx(q1.real().w() / factor, q1.real().x() / factor, q1.real().y() / factor, q1.real().z() / factor,
                    q1.dual().w() / factor, q1.dual().x() / factor, q1.dual().y() / factor, q1.dual().z() / factor));    
  EXPECT_DQ_EQ(q1, q1_copy);

  DualQuaternionx q1_div_copy(q1);
  q1_div_copy /= factor;
  EXPECT_DQ_EQ(q1_div_copy, 
    DualQuaternionx(q1.real().w() / factor, q1.real().x() / factor, q1.real().y() / factor, q1.real().z() / factor,
                    q1.dual().w() / factor, q1.dual().x() / factor, q1.dual().y() / factor, q1.dual().z() / factor));    

  // sum
  DualQuaternionx q_sum = q1 + q2;
  EXPECT_DQ_EQ(q_sum, 
    DualQuaternionx(q1.real().w() + q2.real().w(), q1.real().x() + q2.real().x(),
                    q1.real().y() + q2.real().y(), q1.real().z() + q2.real().z(),
                    q1.dual().w() + q2.dual().w(), q1.dual().x() + q2.dual().x(), 
                    q1.dual().y() + q2.dual().y(), q1.dual().z() + q2.dual().z()));
  EXPECT_DQ_EQ(q1, q1_copy);

  DualQuaternionx q_sum_copy(q1);
  q_sum_copy += q2;
  EXPECT_DQ_EQ(q_sum_copy, 
    DualQuaternionx(q1.real().w() + q2.real().w(), q1.real().x() + q2.real().x(),
                    q1.real().y() + q2.real().y(), q1.real().z() + q2.real().z(),
                    q1.dual().w() + q2.dual().w(), q1.dual().x() + q2.dual().x(), 
                    q1.dual().y() + q2.dual().y(), q1.dual().z() + q2.dual().z()));

  // subtract
  DualQuaternionx q_sub1 = -q1;
  EXPECT_DQ_EQ(q_sub1, 
    DualQuaternionx(-q1.real().w(), -q1.real().x(), -q1.real().y(), -q1.real().z(),
                    -q1.dual().w(), -q1.dual().x(), -q1.dual().y(), -q1.dual().z()));
  EXPECT_DQ_EQ(q1, q1_copy);

  DualQuaternionx q_sub2 = q1 - q2;
  EXPECT_DQ_EQ(q_sub2, 
    DualQuaternionx(q1.real().w() - q2.real().w(), q1.real().x() - q2.real().x(),
                    q1.real().y() - q2.real().y(), q1.real().z() - q2.real().z(),
                    q1.dual().w() - q2.dual().w(), q1.dual().x() - q2.dual().x(), 
                    q1.dual().y() - q2.dual().y(), q1.dual().z() - q2.dual().z()));
  EXPECT_DQ_EQ(q1, q1_copy);

  DualQuaternionx q_sub_copy(q1);
  q_sub_copy -= q2;
  EXPECT_DQ_EQ(q_sub_copy, 
    DualQuaternionx(q1.real().w() - q2.real().w(), q1.real().x() - q2.real().x(),
                    q1.real().y() - q2.real().y(), q1.real().z() - q2.real().z(),
                    q1.dual().w() - q2.dual().w(), q1.dual().x() - q2.dual().x(), 
                    q1.dual().y() - q2.dual().y(), q1.dual().z() - q2.dual().z()));
}

TEST(DualQuaternion, operators)
{
  testDualQuaternionOperators<float>();
  testDualQuaternionOperators<double>();
}


template<typename Scalar>
void testDualQuaternionToVector(void)
{
  typedef m3d::DualQuaternion<Scalar> DualQuaternionx;
  typedef Eigen::Matrix<Scalar, 4, 1> Vector4x;
  typedef Eigen::Matrix<Scalar, 8, 1> Vector8x;

  // create quaternion
  DualQuaternionx q;
  q.real().coeffs() << 1.0, -2.0, 3.0, -4.0;
  q.dual().coeffs() << 5.0, -6.0, 7.0, -8.0;

  // std vector
  std::vector<Scalar> std_vec = q.toVector();
  DualQuaternionx q_std1 = DualQuaternionx::FromVector(std_vec);

  ASSERT_EQ(std_vec.size(), 8);
  EXPECT_DQ_EQ(q, 
    DualQuaternionx(std_vec[0], std_vec[1], std_vec[2], std_vec[3],
                    std_vec[4], std_vec[5], std_vec[6], std_vec[7]));
  EXPECT_DQ_EQ(q, q_std1);

  std::vector<Scalar> std_vec_real = q.real().toVector();
  std::vector<Scalar> std_vec_dual = q.dual().toVector();
  DualQuaternionx q_std2 = DualQuaternionx::FromVector(std_vec_real, std_vec_dual);
  EXPECT_DQ_EQ(q, q_std2);

  // eigen vector
  Vector8x eig_vec = q.toEigenVector();
  EXPECT_DQ_EQ(q, 
    DualQuaternionx(eig_vec(0), eig_vec(1), eig_vec(2), eig_vec(3),
                    eig_vec(4), eig_vec(5), eig_vec(6), eig_vec(7)));

  DualQuaternionx q_eig1 = DualQuaternionx::FromVector(eig_vec);
  EXPECT_DQ_EQ(q, q_eig1);

  Vector4x eig_vec_real = q.real().toEigenVector();
  Vector4x eig_vec_dual = q.dual().toEigenVector();
  DualQuaternionx q_eig2 = DualQuaternionx::FromVector(eig_vec_real, eig_vec_dual);
  EXPECT_DQ_EQ(q, q_eig2);
}

TEST(DualQuaternion, toVector)
{
  testDualQuaternionToVector<float>();
  testDualQuaternionToVector<double>();
}


template<typename Scalar>
void testDualQuaternionToMatrixRandom(void)
{
  typedef m3d::DualQuaternion<Scalar> DualQuaternionx;
  typedef Eigen::Matrix<Scalar, 8, 1> Vector8;

  // create normalized quaternions
  DualQuaternionx q1, q2;
  q1.real().coeffs().setRandom();
  q1.dual().coeffs().setRandom();
  q2.real().coeffs().setRandom();
  q2.dual().coeffs().setRandom();

  // chain with multiplication
  DualQuaternionx q3 = q1 * q2;
  
  // chain in matrix form
  Vector8 v3_pos = q1.toPositiveMatrix() * q2.toEigenVector();
  Vector8 v3_neg = q2.toNegativeMatrix() * q1.toEigenVector();

  // compare
  EIGEN_EXPECT_APPROX(q3.toEigenVector(), v3_pos);
  EIGEN_EXPECT_APPROX(q3.toEigenVector(), v3_neg);
}

TEST(DualQuaternion, toMatrixRandom)
{
  for (size_t i = 0; i < kRandomRuns; ++i)
  {
    testDualQuaternionToMatrixRandom<float>();
    testDualQuaternionToMatrixRandom<double>();
  }
}

template<typename Scalar>
void testDualQuaternionDescription(void)
{
  auto q = m3d::DualQuaternion<Scalar>::Identity();

  // stream
  std::stringstream ss;
  ss << q;
  ASSERT_STRING_STARTS_WITH(ss.str(), "DualQuaternion");

  // description
  ASSERT_EQ(q.desc(), ss.str());
}

TEST(DualQuaternion, description)
{
  testDualQuaternionDescription<float>();
  testDualQuaternionDescription<double>();
}
