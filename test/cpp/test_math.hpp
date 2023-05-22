#pragma once

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <motion3d/common/math.hpp>

namespace m3d = motion3d;


TEST(Math, normalizeAngle)
{
  ASSERT_EQ(m3d::normalizeAngle(0.0), 0.0);
  ASSERT_EQ(m3d::normalizeAngle(1.5), 1.5);
  ASSERT_EQ(m3d::normalizeAngle(3.0), 3.0);
  ASSERT_EQ(m3d::normalizeAngle(-1.5), -1.5);
  ASSERT_EQ(m3d::normalizeAngle(-3.0), -3.0);
  ASSERT_EQ(m3d::normalizeAngle(-M_PI), -M_PI);
  ASSERT_EQ(m3d::normalizeAngle(M_PI), -M_PI);
  ASSERT_EQ(m3d::normalizeAngle(2 * M_PI), 0);
  ASSERT_EQ(m3d::normalizeAngle(3 * M_PI / 2), - M_PI / 2);
  ASSERT_EQ(m3d::normalizeAngle(- 3 * M_PI / 2), M_PI / 2);
}


TEST(Math, getAxisNormalizationFactorInverse)
{
  ASSERT_EQ(1.0, m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << 0, 0, 0).finished(), false));
  ASSERT_EQ(1.0, m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << 1, 0, 0).finished(), false));
  ASSERT_EQ(1.0, m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << 2, 0, 0).finished(), false));
  ASSERT_EQ(1.0, m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << 1, -1, 0).finished(), false));
  ASSERT_EQ(-1.0, m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << -1, 1, 0).finished(), false));
  ASSERT_EQ(-1.0, m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << -1, -1, 0).finished(), false));
  ASSERT_EQ(1.0, m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << 0, 3, 0).finished(), false));
  ASSERT_EQ(-1.0, m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << 0, -1, 0).finished(), false));
  ASSERT_EQ(-1.0, m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << 0, -0.2, -1).finished(), false));
  ASSERT_EQ(1.0, m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished(), false));
  ASSERT_EQ(-1.0, m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << 0, 0, -0.5).finished(), false));

  ASSERT_EQ(0.0, m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << 0, 0, 0).finished(), true));
  ASSERT_EQ(1.0, m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << 1, 0, 0).finished(), true));
  ASSERT_EQ(2.0, m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << 2, 0, 0).finished(), true));
  ASSERT_EQ(std::sqrt(2), m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << 1, -1, 0).finished(), true));
  ASSERT_EQ(-std::sqrt(2), m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << -1, 1, 0).finished(), true));
  ASSERT_EQ(-std::sqrt(2), m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << -1, -1, 0).finished(), true));
  ASSERT_EQ(3.0, m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << 0, 3, 0).finished(), true));
  ASSERT_EQ(-1.0, m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << 0, -1, 0).finished(), true));
  ASSERT_EQ(0.5, m3d::getAxisNormalizationFactorInverse((Eigen::Matrix<double, 3, 1>() << 0, 0, 0.5).finished(), true));
}

TEST(Math, normalizeRotationMatrix)
{
  auto zero = Eigen::Matrix<double, 3, 3>::Zero();
  auto identity = Eigen::Matrix<double, 3, 3>::Identity();
  EIGEN_ASSERT_APPROX(identity, m3d::normalizeRotationMatrix(zero));
  EIGEN_ASSERT_APPROX(identity, m3d::normalizeRotationMatrix(identity));
  EIGEN_ASSERT_APPROX(identity, m3d::normalizeRotationMatrix(0.5 * identity));
  EIGEN_ASSERT_APPROX(identity, m3d::normalizeRotationMatrix(2 * identity));

  Eigen::Matrix<double, 3, 3> rotation;
  rotation << 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  EIGEN_ASSERT_APPROX(rotation, m3d::normalizeRotationMatrix(rotation));
  EIGEN_ASSERT_APPROX(rotation, m3d::normalizeRotationMatrix(0.5 * rotation));
  EIGEN_ASSERT_APPROX(rotation, m3d::normalizeRotationMatrix(2 * rotation));

  ASSERT_VALID_ROTATION_MATRIX(m3d::normalizeRotationMatrix(-identity));
  ASSERT_VALID_ROTATION_MATRIX(m3d::normalizeRotationMatrix(-rotation));
}

TEST(Math, decomposeRZS)
{
  // create data
  Eigen::Matrix<double, 3, 3> rotation;
  rotation << 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;

  Eigen::Matrix<double, 3, 1> zoom;
  zoom << 2.0, 3.0, 4.0;

  Eigen::Matrix<double, 3, 1> shear;
  shear << 5.0, -6.0, 7.0;

  // compose matrix
  Eigen::Matrix<double, 3, 3> zoom_mat = Eigen::Matrix<double, 3, 3>::Zero();
  zoom_mat.diagonal() << zoom;

  Eigen::Matrix<double, 3, 3> shear_mat = Eigen::Matrix<double, 3, 3>::Identity();
  shear_mat(0, 1) = shear(0);
  shear_mat(0, 2) = shear(1);
  shear_mat(1, 2) = shear(2);

  Eigen::Matrix<double, 3, 3> mat = rotation * zoom_mat * shear_mat;

  // decompose matrix
  Eigen::Matrix<double, 3, 3> rotation_dec;
  Eigen::Matrix<double, 3, 1> zoom_dec;
  Eigen::Matrix<double, 3, 1> shear_dec;
  std::tie(rotation_dec, zoom_dec, shear_dec) = m3d::decomposeRZS(mat);

  // check results
  ASSERT_VALID_ROTATION_MATRIX(rotation_dec);
  EIGEN_EXPECT_APPROX(rotation, rotation_dec);
  EIGEN_EXPECT_APPROX(zoom, zoom_dec);
  EIGEN_EXPECT_APPROX(shear, shear_dec);
}
