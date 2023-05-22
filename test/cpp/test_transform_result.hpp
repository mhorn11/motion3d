#pragma once

#include <gtest/gtest.h>

#include <motion3d/transforms.hpp>

#include "data.hpp"
#include "utils.hpp"

namespace m3d = motion3d;


TEST(TransformResult, unitTransform)
{
  auto axis_angle = m3d::AxisAngleTransform();
  auto dual_quaternion = m3d::DualQuaternionTransform();
  auto euler = m3d::EulerTransform();
  auto matrix = m3d::MatrixTransform();
  auto quaternion = m3d::QuaternionTransform();

  EIGEN_EXPECT_APPROX(axis_angle.toEigenVector(), kAxisAngleUnitVec);
  EIGEN_EXPECT_APPROX(dual_quaternion.toEigenVector(), kDualQuaternionUnitVec);
  EIGEN_EXPECT_APPROX(euler.toEigenVector(), kEulerUnitVec);
  EIGEN_EXPECT_APPROX(matrix.toEigenVector(), kMatrixUnitVec);
  EIGEN_EXPECT_APPROX(quaternion.toEigenVector(), kQuaternionUnitVec);
}

TEST(TransformResult, binary)
{
  auto axis_angle = m3d::AxisAngleTransform(kAxisAngle1Vec);
  auto dual_quaternion = m3d::DualQuaternionTransform(kDualQuaternion1Vec);
  auto euler = m3d::EulerTransform(kEuler1Vec);
  auto matrix = m3d::MatrixTransform(kMatrix1Vec);
  auto quaternion = m3d::QuaternionTransform(kQuaternion1Vec);

  EIGEN_EXPECT_APPROX(m3d::AxisAngleTransform(axis_angle.toBinary()).toEigenVector(), kAxisAngle1Vec);
  EIGEN_EXPECT_APPROX(m3d::DualQuaternionTransform(dual_quaternion.toBinary()).toEigenVector(), kDualQuaternion1Vec);
  EIGEN_EXPECT_APPROX(m3d::EulerTransform(euler.toBinary()).toEigenVector(), kEuler1Vec);
  EIGEN_EXPECT_APPROX(m3d::MatrixTransform(matrix.toBinary()).toEigenVector(), kMatrix1Vec);
  EIGEN_EXPECT_APPROX(m3d::QuaternionTransform(quaternion.toBinary()).toEigenVector(), kQuaternion1Vec);
}

TEST(TransformResult, normalized)
{
  // basic
  auto axis_angle = m3d::AxisAngleTransform(kAxisAngle1Vec, true);
  auto dual_quaternion = m3d::DualQuaternionTransform(kDualQuaternion1Vec, true);
  auto euler = m3d::EulerTransform(kEuler1Vec, true);
  auto matrix = m3d::MatrixTransform(kMatrix1Vec, true);
  auto quaternion = m3d::QuaternionTransform(kQuaternion1Vec, true);

  // without modification
  EIGEN_EXPECT_APPROX(axis_angle.normalized().toEigenVector(), kAxisAngle1Vec);
  EIGEN_EXPECT_APPROX(dual_quaternion.normalized().toEigenVector(), kDualQuaternion1Vec);
  EIGEN_EXPECT_APPROX(euler.normalized().toEigenVector(), kEuler1Vec);
  EIGEN_EXPECT_APPROX(matrix.normalized().toEigenVector(), kMatrix1Vec);
  EIGEN_EXPECT_APPROX(quaternion.normalized().toEigenVector(), kQuaternion1Vec);

  // axis angle
  auto axis_angle_mod = m3d::AxisAngleTransform(
    axis_angle.getTranslation(), -axis_angle.getAngle() - 2 * M_PI, -2.0 * axis_angle.getAxis(), true);
  m3d::AxisAngleTransform axis_angle_mod_norm = axis_angle_mod.normalized();
  ASSERT_FALSE(axis_angle_mod_norm.isUnsafe());
  EIGEN_EXPECT_APPROX(axis_angle_mod_norm.toEigenVector(), kAxisAngle1Vec);
  EIGEN_EXPECT_APPROX(axis_angle_mod.normalized_().toEigenVector(), kAxisAngle1Vec);

  // dual quaternion
  auto dual_quaternion_mod1 = m3d::DualQuaternionTransform(
    dual_quaternion.getReal() * -2.0,
    dual_quaternion.getDual() * -2.0,
    true);
  m3d::DualQuaternionTransform dual_quaternion_mod1_norm = dual_quaternion_mod1.normalized();
  ASSERT_FALSE(dual_quaternion_mod1_norm.isUnsafe());
  EIGEN_EXPECT_APPROX(dual_quaternion_mod1_norm.toEigenVector(), kDualQuaternion1Vec);
  EIGEN_EXPECT_APPROX(dual_quaternion_mod1.normalized_().toEigenVector(), kDualQuaternion1Vec);

  m3d::Quaternion<double> translation_quaternion = dual_quaternion.getDualQuaternion().getTranslationQuaternion();
  translation_quaternion.w() = 1.0;
  m3d::Quaternion<double> real = dual_quaternion.getReal() * -2.0;
  m3d::Quaternion<double> dual = (translation_quaternion * real) * 0.5;
  auto dual_quaternion_mod2 = m3d::DualQuaternionTransform(real, dual, true);
  EIGEN_EXPECT_APPROX(dual_quaternion_mod2.normalized().toEigenVector(), kDualQuaternion1Vec);
  EIGEN_EXPECT_APPROX(dual_quaternion_mod2.normalized_().toEigenVector(), kDualQuaternion1Vec);

  // euler
  auto euler_mod = m3d::EulerTransform(
    euler.getTranslation(), euler.getAi() + 2 * M_PI, euler.getAj() - 2 * M_PI, euler.getAk() + 2 * M_PI,
    euler.getAxes(), true);
  m3d::EulerTransform euler_mod_norm = euler_mod.normalized();
  ASSERT_FALSE(euler_mod_norm.isUnsafe());
  EIGEN_EXPECT_APPROX(euler_mod_norm.toEigenVector(), kEuler1Vec);
  EIGEN_EXPECT_APPROX(euler_mod.normalized_().toEigenVector(), kEuler1Vec);

  // matrix
  Eigen::Matrix<double, 3, 3> zoom = Eigen::Matrix<double, 3, 3>::Zero();
  zoom.diagonal() << 2.0, 3.0, 4.0;
  Eigen::Matrix<double, 3, 3> shear = Eigen::Matrix<double, 3, 3>::Identity();
  shear(0, 1) = 5.0;
  shear(0, 2) = -6.0;
  shear(1, 2) = 7.0;

  Eigen::Matrix<double, 4, 4> matrix_mod_data = matrix.getMatrix();
  matrix_mod_data.block<3, 3>(0, 0) *= zoom * shear;
  matrix_mod_data.block<1, 4>(3, 0) << 1.0, 2.0, 3.0, 4.0;

  auto matrix_mod = m3d::MatrixTransform(matrix_mod_data, true);
  m3d::MatrixTransform matrix_mod_norm = matrix_mod.normalized();
  ASSERT_FALSE(matrix_mod_norm.isUnsafe());
  EIGEN_EXPECT_APPROX(matrix_mod_norm.toEigenVector(), kMatrix1Vec);
  EXPECT_EQ(matrix_mod_norm.getMatrix()(3, 0), 0.0);
  EXPECT_EQ(matrix_mod_norm.getMatrix()(3, 1), 0.0);
  EXPECT_EQ(matrix_mod_norm.getMatrix()(3, 2), 0.0);
  EXPECT_EQ(matrix_mod_norm.getMatrix()(3, 3), 1.0);
  EIGEN_EXPECT_APPROX(matrix_mod.normalized_().toEigenVector(), kMatrix1Vec);

  // quaternion
  auto quaternion_mod = m3d::QuaternionTransform(
    quaternion.getTranslation(), quaternion.getQuaternion() * -2.0, true);
  m3d::QuaternionTransform quaternion_mod_norm = quaternion_mod.normalized();
  ASSERT_FALSE(quaternion_mod_norm.isUnsafe());
  EIGEN_EXPECT_APPROX(quaternion_mod_norm.toEigenVector(), kQuaternion1Vec);
  EIGEN_EXPECT_APPROX(quaternion_mod.normalized_().toEigenVector(), kQuaternion1Vec);

  // inplace without modification
  EIGEN_EXPECT_APPROX(axis_angle.normalized_().toEigenVector(), kAxisAngle1Vec);
  ASSERT_FALSE(axis_angle.isUnsafe());
  EIGEN_EXPECT_APPROX(dual_quaternion.normalized_().toEigenVector(), kDualQuaternion1Vec);
  ASSERT_FALSE(dual_quaternion.isUnsafe());
  EIGEN_EXPECT_APPROX(euler.normalized_().toEigenVector(), kEuler1Vec);
  ASSERT_FALSE(euler.isUnsafe());
  EIGEN_EXPECT_APPROX(matrix.normalized_().toEigenVector(), kMatrix1Vec);
  ASSERT_FALSE(matrix.isUnsafe());
  EIGEN_EXPECT_APPROX(quaternion.normalized_().toEigenVector(), kQuaternion1Vec);
  ASSERT_FALSE(quaternion.isUnsafe());
}

TEST(TransformResult, scaleTranslation)
{
  double factor = 2.0;

  // reference vectors
  auto kAxisAngle1VecScaled = kAxisAngle1Vec;
  auto kDualQuaternion1VecScaled = kDualQuaternion1Vec;
  auto kEuler1VecScaled = kEuler1Vec;
  auto kMatrix1VecScaled = kMatrix1Vec;
  auto kQuaternion1VecScaled = kQuaternion1Vec;

  kAxisAngle1VecScaled.block<3, 1>(0, 0) *= factor;
  kDualQuaternion1VecScaled.block<4, 1>(4, 0) *= factor;
  kEuler1VecScaled.block<3, 1>(0, 0) *= factor;
  kMatrix1VecScaled(3, 0) *= factor;
  kMatrix1VecScaled(7, 0) *= factor;
  kMatrix1VecScaled(11, 0) *= factor;
  kQuaternion1VecScaled.block<3, 1>(0, 0) *= factor;

  // scale
  auto axis_angle = m3d::AxisAngleTransform(kAxisAngle1Vec);
  auto dual_quaternion = m3d::DualQuaternionTransform(kDualQuaternion1Vec);
  auto euler = m3d::EulerTransform(kEuler1Vec);
  auto matrix = m3d::MatrixTransform(kMatrix1Vec);
  auto quaternion = m3d::QuaternionTransform(kQuaternion1Vec);

  EIGEN_EXPECT_APPROX(axis_angle.scaleTranslation(factor).toEigenVector(), kAxisAngle1VecScaled);
  EIGEN_EXPECT_APPROX(dual_quaternion.scaleTranslation(factor).toEigenVector(), kDualQuaternion1VecScaled);
  EIGEN_EXPECT_APPROX(euler.scaleTranslation(factor).toEigenVector(), kEuler1VecScaled);
  EIGEN_EXPECT_APPROX(matrix.scaleTranslation(factor).toEigenVector(), kMatrix1VecScaled);
  EIGEN_EXPECT_APPROX(quaternion.scaleTranslation(factor).toEigenVector(), kQuaternion1VecScaled);

  // scale inplace
  axis_angle.scaleTranslation_(factor);
  EIGEN_EXPECT_APPROX(axis_angle.toEigenVector(), kAxisAngle1VecScaled);
  dual_quaternion.scaleTranslation_(factor);
  EIGEN_EXPECT_APPROX(dual_quaternion.toEigenVector(), kDualQuaternion1VecScaled);
  euler.scaleTranslation_(factor);
  EIGEN_EXPECT_APPROX(euler.toEigenVector(), kEuler1VecScaled);
  matrix.scaleTranslation_(factor);
  EIGEN_EXPECT_APPROX(matrix.toEigenVector(), kMatrix1VecScaled);
  quaternion.scaleTranslation_(factor);
  EIGEN_EXPECT_APPROX(quaternion.toEigenVector(), kQuaternion1VecScaled);
}

TEST(TransformResult, asType)
{
  // create transforms
  auto axis_angle_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle1Vec);
  auto dual_quaternion_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion1Vec);
  auto euler_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler1Vec);
  auto matrix_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix1Vec);
  auto quaternion_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion1Vec);

  std::vector<std::shared_ptr<m3d::TransformInterface>> interface_vec = {
    axis_angle_if, dual_quaternion_if, euler_if, matrix_if, quaternion_if
  };

  // as type
  for (auto interface : interface_vec)
  {
    m3d::TransformInterface::Ptr axis_angle_tmp = interface->asType<m3d::AxisAngleTransform>();
    m3d::TransformInterface::Ptr dual_quaternion_tmp = interface->asType<m3d::DualQuaternionTransform>();
    m3d::TransformInterface::Ptr euler_tmp = interface->asType<m3d::EulerTransform>();
    m3d::TransformInterface::Ptr matrix_tmp = interface->asType<m3d::MatrixTransform>();
    m3d::TransformInterface::Ptr quat_tmp = interface->asType<m3d::QuaternionTransform>();

    EIGEN_EXPECT_APPROX(axis_angle_tmp->normalized()->toEigenVector(), kAxisAngle1Vec);
    EIGEN_EXPECT_APPROX(dual_quaternion_tmp->normalized()->toEigenVector(), kDualQuaternion1Vec);
    EIGEN_EXPECT_APPROX(euler_tmp->normalized()->toEigenVector(), kEuler1Vec);
    EIGEN_EXPECT_APPROX(matrix_tmp->normalized()->toEigenVector(), kMatrix1Vec);
    EIGEN_EXPECT_APPROX(quat_tmp->normalized()->toEigenVector(), kQuaternion1Vec);
  }

  // inverse
  auto axis_angle_inv_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle1InvVec);
  auto dual_quaternion_inv_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion1InvVec);
  auto euler_inv_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler1InvVec);
  auto matrix_inv_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix1InvVec);
  auto quaternion_inv_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion1InvVec);

  std::vector<std::shared_ptr<m3d::TransformInterface>> interface_inv_vec = {
    axis_angle_inv_if, dual_quaternion_inv_if, euler_inv_if, matrix_inv_if, quaternion_inv_if
  };

  for (auto interface : interface_inv_vec)
  {
    auto axis_angle_tmp = interface->asType<m3d::AxisAngleTransform>();
    auto dual_quaternion_tmp = interface->asType<m3d::DualQuaternionTransform>();
    auto euler_tmp = interface->asType<m3d::EulerTransform>();
    auto matrix_tmp = interface->asType<m3d::MatrixTransform>();
    auto quat_tmp = interface->asType<m3d::QuaternionTransform>();

    EIGEN_EXPECT_APPROX(axis_angle_tmp->normalized().toEigenVector(), kAxisAngle1InvVec);
    EIGEN_EXPECT_APPROX(dual_quaternion_tmp->normalized().toEigenVector(), kDualQuaternion1InvVec);
    EIGEN_EXPECT_APPROX(euler_tmp->normalized().toEigenVector(), kEuler1InvVec);
    EIGEN_EXPECT_APPROX(matrix_tmp->normalized().toEigenVector(), kMatrix1InvVec);
    EIGEN_EXPECT_APPROX(quat_tmp->normalized().toEigenVector(), kQuaternion1InvVec);
  }
}

void transformResultAsTypeRandom(void)
{
  // generate data
  m3d::TransformInterface::Ptr euler_if = createRandomEulerTransformInterface();
  m3d::TransformInterface::Ptr axis_angle_if = euler_if->asType<m3d::AxisAngleTransform>();
  m3d::TransformInterface::Ptr dual_quaternion_if = euler_if->asType<m3d::DualQuaternionTransform>();
  m3d::TransformInterface::Ptr matrix_if = euler_if->asType<m3d::MatrixTransform>();
  m3d::TransformInterface::Ptr quat_if = euler_if->asType<m3d::QuaternionTransform>();

  std::vector<std::shared_ptr<m3d::TransformInterface>> interface_inv_vec = {
    axis_angle_if, dual_quaternion_if, euler_if, matrix_if, quat_if
  };

  // convert
  for (auto interface : interface_inv_vec)
  {
    ASSERT_TRANSFORM_APPROX(dual_quaternion_if, interface->asType<m3d::AxisAngleTransform>());
    ASSERT_TRANSFORM_APPROX(dual_quaternion_if, interface->asType<m3d::DualQuaternionTransform>());
    ASSERT_TRANSFORM_APPROX(dual_quaternion_if, interface->asType<m3d::EulerTransform>());
    ASSERT_TRANSFORM_APPROX(dual_quaternion_if, interface->asType<m3d::MatrixTransform>());
    ASSERT_TRANSFORM_APPROX(dual_quaternion_if, interface->asType<m3d::QuaternionTransform>());
  }
}

TEST(TransformResult, asTypeRandom)
{
  for (size_t i = 0; i < kRandomRuns; ++i)
  {
    transformResultAsTypeRandom();
  }
}

TEST(TransformResult, eulerConversion)
{
  // generate data
  auto axis_angle = m3d::AxisAngleTransform(kAxisAngle1Vec);
  auto dual_quaternion = m3d::DualQuaternionTransform(kDualQuaternion1Vec);
  auto euler = m3d::EulerTransform(kEuler1Vec);
  auto matrix = m3d::MatrixTransform(kMatrix1Vec);
  auto quaternion = m3d::QuaternionTransform(kQuaternion1Vec);

  auto dual_quaternion_matrix = dual_quaternion.toEigenVector();

  for (auto axes1 : m3d::kEulerAxesAll)
  {
    // convert axes
    m3d::EulerTransform euler_axes1 = euler.changeAxes(axes1);
    ASSERT_EQ(euler_axes1.getAxes(), axes1);

    // convert axes again
    for (auto axes2 : m3d::kEulerAxesAll)
    {
      m3d::EulerTransform euler_axes2 = euler_axes1.changeAxes(axes2);
      ASSERT_EQ(euler_axes2.getAxes(), axes2);
      ASSERT_DQ_MAT_APPROX(euler_axes2.asType<m3d::DualQuaternionTransform>().toEigenVector(), dual_quaternion_matrix);
    }

    // convert axes inplace
    for (auto axes2 : m3d::kEulerAxesAll)
    {
      m3d::EulerTransform euler_axes1_copy(euler_axes1);
      euler_axes1_copy.changeAxes_(axes2);
      ASSERT_EQ(euler_axes1_copy.getAxes(), axes2);
      ASSERT_DQ_MAT_APPROX(euler_axes1_copy.asType<m3d::DualQuaternionTransform>().toEigenVector(), dual_quaternion_matrix);
    }

    // conversion constructors
    ASSERT_EQ(m3d::EulerTransform(axis_angle, axes1).getAxes(), axes1);
    ASSERT_EQ(m3d::EulerTransform(dual_quaternion, axes1).getAxes(), axes1);
    ASSERT_EQ(m3d::EulerTransform(euler, axes1).getAxes(), axes1);
    ASSERT_EQ(m3d::EulerTransform(matrix, axes1).getAxes(), axes1);
    ASSERT_EQ(m3d::EulerTransform(quaternion, axes1).getAxes(), axes1);

    // inverse
    ASSERT_EQ(euler_axes1.inverse().getAxes(), axes1);
    ASSERT_EQ(m3d::EulerTransform(euler_axes1).inverse_().getAxes(), axes1);

    // apply
    ASSERT_EQ((euler_axes1.applyPre(axis_angle)).getAxes(), axes1);
    ASSERT_EQ((euler_axes1.applyPre(dual_quaternion)).getAxes(), axes1);
    ASSERT_EQ((euler_axes1.applyPre(euler)).getAxes(), axes1);
    ASSERT_EQ((euler_axes1.applyPre(matrix)).getAxes(), axes1);
    ASSERT_EQ((euler_axes1.applyPre(quaternion)).getAxes(), axes1);
    ASSERT_EQ((euler_axes1.applyPost(axis_angle)).getAxes(), axes1);
    ASSERT_EQ((euler_axes1.applyPost(dual_quaternion)).getAxes(), axes1);
    ASSERT_EQ((euler_axes1.applyPost(euler)).getAxes(), axes1);
    ASSERT_EQ((euler_axes1.applyPost(matrix)).getAxes(), axes1);
    ASSERT_EQ((euler_axes1.applyPost(quaternion)).getAxes(), axes1);

    // apply inplace
    ASSERT_EQ((euler_axes1.applyPre_(axis_angle)).getAxes(), axes1);
    ASSERT_EQ((euler_axes1.applyPre_(dual_quaternion)).getAxes(), axes1);
    ASSERT_EQ((euler_axes1.applyPre_(euler)).getAxes(), axes1);
    ASSERT_EQ((euler_axes1.applyPre_(matrix)).getAxes(), axes1);
    ASSERT_EQ((euler_axes1.applyPre_(quaternion)).getAxes(), axes1);
    ASSERT_EQ((euler_axes1.applyPost_(axis_angle)).getAxes(), axes1);
    ASSERT_EQ((euler_axes1.applyPost_(dual_quaternion)).getAxes(), axes1);
    ASSERT_EQ((euler_axes1.applyPost_(euler)).getAxes(), axes1);
    ASSERT_EQ((euler_axes1.applyPost_(matrix)).getAxes(), axes1);
    ASSERT_EQ((euler_axes1.applyPost_(quaternion)).getAxes(), axes1);

    // operators
    ASSERT_EQ((euler_axes1 * axis_angle).getAxes(), axes1);
    ASSERT_EQ((euler_axes1 * dual_quaternion).getAxes(), axes1);
    ASSERT_EQ((euler_axes1 * euler).getAxes(), axes1);
    ASSERT_EQ((euler_axes1 * matrix).getAxes(), axes1);
    ASSERT_EQ((euler_axes1 * quaternion).getAxes(), axes1);
    ASSERT_EQ((euler_axes1 / axis_angle).getAxes(), axes1);
    ASSERT_EQ((euler_axes1 / dual_quaternion).getAxes(), axes1);
    ASSERT_EQ((euler_axes1 / euler).getAxes(), axes1);
    ASSERT_EQ((euler_axes1 / matrix).getAxes(), axes1);
    ASSERT_EQ((euler_axes1 / quaternion).getAxes(), axes1);
  }

}

void transformResultEulerConversionRandom(void)
{
  // generate data
  m3d::EulerTransform transform = *createRandomEulerTransformInterface();
  auto dual_quaternion_matrix = transform.asType<m3d::DualQuaternionTransform>().toEigenVector();

  // convert
  for (auto axes1 : m3d::kEulerAxesAll)
  {
    m3d::EulerTransform transform_axes1 = transform.changeAxes(axes1);
    for (auto axes2 : m3d::kEulerAxesAll)
    {
      // non-inplace
      m3d::EulerTransform transform_axes2 = transform_axes1.changeAxes(axes2);
      ASSERT_DQ_MAT_APPROX(transform_axes2.asType<m3d::DualQuaternionTransform>().toEigenVector(), dual_quaternion_matrix);

      // inplace
      m3d::EulerTransform transform_axes1_copy(transform_axes1);
      transform_axes1_copy.changeAxes_(axes2);
      ASSERT_DQ_MAT_APPROX(transform_axes1_copy.asType<m3d::DualQuaternionTransform>().toEigenVector(), dual_quaternion_matrix);
    }
  }
}

TEST(TransformResult, eulerConversionRandom)
{
  for (size_t i = 0; i < kRandomRuns; ++i)
  {
    transformResultEulerConversionRandom();
  }
}

TEST(TransformResult, isEqual)
{ 
  // generate data
  auto axis_angle1_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle1Vec);
  auto dual_quaternion1_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion1Vec);
  auto euler1_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler1Vec);
  auto matrix1_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix1Vec);
  auto quaternion1_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion1Vec);

  std::vector<std::shared_ptr<m3d::TransformInterface>> interface1_vec = {
    axis_angle1_if, dual_quaternion1_if, euler1_if, matrix1_if, quaternion1_if
  };

  auto axis_angle2_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle2Vec);
  auto dual_quaternion2_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion2Vec);
  auto euler2_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler2Vec);
  auto matrix2_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix2Vec);
  auto quaternion2_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion2Vec);

  std::vector<std::shared_ptr<m3d::TransformInterface>> interface2_vec = {
    axis_angle2_if, dual_quaternion2_if, euler2_if, matrix2_if, quaternion2_if
  };

  // check equal
  for (auto interface1a : interface1_vec)
  {
    for (auto interface1b : interface1_vec)
    {
      EXPECT_TRUE(interface1a->isEqual(interface1b));
    }
    for (auto interface2 : interface2_vec)
    {
      EXPECT_FALSE(interface1a->isEqual(interface2));
    }
  }

  // negative dual quaternion
  Eigen::Matrix<double, 8, 1> dual_quaternion1_vec_neg = -kDualQuaternion1Vec;
  auto dual_quaternion1_if_neg = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, dual_quaternion1_vec_neg);
  EXPECT_TRUE(dual_quaternion1_if->isEqual(dual_quaternion1_if_neg));
}

TEST(TransformResult, inverse)
{
  // generate data
  auto axis_angle = m3d::AxisAngleTransform(kAxisAngle1Vec);
  auto dual_quaternion = m3d::DualQuaternionTransform(kDualQuaternion1Vec);
  auto euler = m3d::EulerTransform(kEuler1Vec);
  auto matrix = m3d::MatrixTransform(kMatrix1Vec);
  auto quaternion = m3d::QuaternionTransform(kQuaternion1Vec);

  // inverse
  EIGEN_EXPECT_APPROX(axis_angle.inverse().normalized().toEigenVector(), kAxisAngle1InvVec);
  EIGEN_EXPECT_APPROX(dual_quaternion.inverse().normalized().toEigenVector(), kDualQuaternion1InvVec);
  EIGEN_EXPECT_APPROX(euler.inverse().normalized().toEigenVector(), kEuler1InvVec);
  EIGEN_EXPECT_APPROX(matrix.inverse().normalized().toEigenVector(), kMatrix1InvVec);
  EIGEN_EXPECT_APPROX(quaternion.inverse().normalized().toEigenVector(), kQuaternion1InvVec);

  // inverse inplace
  axis_angle.inverse_();
  EIGEN_EXPECT_APPROX(axis_angle.normalized().toEigenVector(), kAxisAngle1InvVec);
  dual_quaternion.inverse_();
  EIGEN_EXPECT_APPROX(dual_quaternion.normalized().toEigenVector(), kDualQuaternion1InvVec);
  euler.inverse_();
  EIGEN_EXPECT_APPROX(euler.normalized().toEigenVector(), kEuler1InvVec);
  matrix.inverse_();
  EIGEN_EXPECT_APPROX(matrix.normalized().toEigenVector(), kMatrix1InvVec);
  quaternion.inverse_();
  EIGEN_EXPECT_APPROX(quaternion.normalized().toEigenVector(), kQuaternion1InvVec);
}

TEST(TransformResult, dualQuaternionMatrix)
{
  // generate data
  auto dq1 = m3d::DualQuaternionTransform(kDualQuaternion1Vec);
  auto dq2 = m3d::DualQuaternionTransform(kDualQuaternion2Vec);

  // multiply
  m3d::DualQuaternionTransform dq12 = dq1 * dq2;
  Eigen::Matrix<double, 8, 1> dq12_pos = dq1.getDualQuaternion().toPositiveMatrix() * dq2.toEigenVector();
  Eigen::Matrix<double, 8, 1> dq12_neg = dq2.getDualQuaternion().toNegativeMatrix() * dq1.toEigenVector();

  // check
  EIGEN_EXPECT_APPROX(dq12_pos, dq12.toEigenVector());
  EIGEN_EXPECT_APPROX(dq12_neg, dq12.toEigenVector());
}

TEST(TransformResult, applyDirect)
{
  // generate data
  auto axis_angle1 = m3d::AxisAngleTransform(kAxisAngle1Vec);
  auto dual_quaternion1 = m3d::DualQuaternionTransform(kDualQuaternion1Vec);
  auto euler1 = m3d::EulerTransform(kEuler1Vec);
  auto matrix1 = m3d::MatrixTransform(kMatrix1Vec);
  auto quaternion1 = m3d::QuaternionTransform(kQuaternion1Vec);

  auto axis_angle2 = m3d::AxisAngleTransform(kAxisAngle2Vec);
  auto dual_quaternion2 = m3d::DualQuaternionTransform(kDualQuaternion2Vec);
  auto euler2 = m3d::EulerTransform(kEuler2Vec);
  auto matrix2 = m3d::MatrixTransform(kMatrix2Vec);
  auto quaternion2 = m3d::QuaternionTransform(kQuaternion2Vec);

  auto axis_angle1_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle1Vec);
  auto dual_quaternion1_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion1Vec);
  auto euler1_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler1Vec);
  auto matrix1_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix1Vec);
  auto quaternion1_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion1Vec);

  auto axis_angle2_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle2Vec);
  auto dual_quaternion2_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion2Vec);
  auto euler2_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler2Vec);
  auto matrix2_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix2Vec);
  auto quaternion2_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion2Vec);

  std::vector<std::shared_ptr<m3d::TransformInterface>> interface1_vec = {
    axis_angle1_if, dual_quaternion1_if, euler1_if, matrix1_if, quaternion1_if
  };
  std::vector<std::shared_ptr<m3d::TransformInterface>> interface2_vec = {
    axis_angle2_if, dual_quaternion2_if, euler2_if, matrix2_if, quaternion2_if
  };

  Eigen::Matrix<double, 3, 5> cloud0;
  Eigen::Matrix<double, 3, 5> cloud1;
  cloud0 << kPoint0Vec, kPoint0Vec, kPoint0Vec, kPoint0Vec, kPoint0Vec;
  cloud1 << kPoint1Vec, kPoint1Vec, kPoint1Vec, kPoint1Vec, kPoint1Vec;

  Eigen::Matrix<double, 3, Eigen::Dynamic> cloud0_dyn;
  Eigen::Matrix<double, 3, Eigen::Dynamic> cloud1_dyn;
  cloud0_dyn.resize(3, 5);
  cloud1_dyn.resize(3, 5);
  cloud0_dyn << kPoint0Vec, kPoint0Vec, kPoint0Vec, kPoint0Vec, kPoint0Vec;
  cloud1_dyn << kPoint1Vec, kPoint1Vec, kPoint1Vec, kPoint1Vec, kPoint1Vec;

  // apply
  auto applyDirectFun = [&](const auto &transform1, const auto &transform2, const auto &vec12) {
    // copy
    EIGEN_EXPECT_APPROX(transform1.applyPost(axis_angle2).normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(transform1.applyPost(dual_quaternion2).normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(transform1.applyPost(euler2).normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(transform1.applyPost(matrix2).normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(transform1.applyPost(quaternion2).normalized().toEigenVector(), vec12);

    EIGEN_EXPECT_APPROX(transform2.applyPre(axis_angle1).normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(transform2.applyPre(dual_quaternion1).normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(transform2.applyPre(euler1).normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(transform2.applyPre(matrix1).normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(transform2.applyPre(quaternion1).normalized().toEigenVector(), vec12);

    // inplace
    EIGEN_EXPECT_APPROX(transform1.copy().applyPost_(axis_angle2).normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(transform1.copy().applyPost_(dual_quaternion2).normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(transform1.copy().applyPost_(euler2).normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(transform1.copy().applyPost_(matrix2).normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(transform1.copy().applyPost_(quaternion2).normalized().toEigenVector(), vec12);

    EIGEN_EXPECT_APPROX(transform2.copy().applyPre_(axis_angle1).normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(transform2.copy().applyPre_(dual_quaternion1).normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(transform2.copy().applyPre_(euler1).normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(transform2.copy().applyPre_(matrix1).normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(transform2.copy().applyPre_(quaternion1).normalized().toEigenVector(), vec12);

    // interface
    for (auto interface2 : interface2_vec)
    {
      EIGEN_EXPECT_APPROX(transform1.applyPost(interface2).normalized().toEigenVector(), vec12);
      EIGEN_EXPECT_APPROX(transform1.copy().applyPost_(interface2).normalized().toEigenVector(), vec12);
    }
    for (auto interface1 : interface1_vec)
    {
      EIGEN_EXPECT_APPROX(transform2.applyPre(interface1).normalized().toEigenVector(), vec12);
      EIGEN_EXPECT_APPROX(transform2.copy().applyPre_(interface1).normalized().toEigenVector(), vec12);
    }

    // point
    EIGEN_EXPECT_APPROX(transform1.transformPoint(kPoint0Vec), kPoint1Vec);

    // cloud
    EIGEN_EXPECT_APPROX(transform1.transformCloud(cloud0), cloud1);
    EIGEN_EXPECT_APPROX(transform1.transformCloud(cloud0_dyn), cloud1_dyn);
  };

  applyDirectFun(axis_angle1, axis_angle2, kAxisAngle12Vec);
  applyDirectFun(dual_quaternion1, dual_quaternion2, kDualQuaternion12Vec);
  applyDirectFun(euler1, euler2, kEuler12Vec);
  applyDirectFun(matrix1, matrix2, kMatrix12Vec);
  applyDirectFun(quaternion1, quaternion2, kQuaternion12Vec);
}

TEST(TransformResult, applyInterface)
{
  // generate data
  auto axis_angle_unit_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngleUnitVec);
  auto dual_quaternion_unit_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternionUnitVec);
  auto euler_unit_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEulerUnitVec);
  auto matrix_unit_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrixUnitVec);
  auto quaternion_unit_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternionUnitVec);

  auto axis_angle1_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle1Vec);
  auto dual_quaternion1_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion1Vec);
  auto euler1_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler1Vec);
  auto matrix1_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix1Vec);
  auto quaternion1_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion1Vec);

  auto axis_angle1 = m3d::AxisAngleTransform(kAxisAngle1Vec);
  auto dual_quaternion1 = m3d::DualQuaternionTransform(kDualQuaternion1Vec);
  auto euler1 = m3d::EulerTransform(kEuler1Vec);
  auto matrix1 = m3d::MatrixTransform(kMatrix1Vec);
  auto quaternion1 = m3d::QuaternionTransform(kQuaternion1Vec);

  auto axis_angle2 = m3d::AxisAngleTransform(kAxisAngle2Vec);
  auto dual_quaternion2 = m3d::DualQuaternionTransform(kDualQuaternion2Vec);
  auto euler2 = m3d::EulerTransform(kEuler2Vec);
  auto matrix2 = m3d::MatrixTransform(kMatrix2Vec);
  auto quaternion2 = m3d::QuaternionTransform(kQuaternion2Vec);

  auto axis_angle2_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle2Vec);
  auto dual_quaternion2_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion2Vec);
  auto euler2_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler2Vec);
  auto matrix2_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix2Vec);
  auto quaternion2_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion2Vec);

  auto axis_angle12_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle12Vec);
  auto dual_quaternion12_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion12Vec);
  auto euler12_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler12Vec);
  auto matrix12_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix12Vec);
  auto quaternion12_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion12Vec);

  auto axis_angle1_inv_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle1InvVec);
  auto dual_quaternion1_inv_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion1InvVec);
  auto euler1_inv_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler1InvVec);
  auto matrix1_inv_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix1InvVec);
  auto quaternion1_inv_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion1InvVec);

  std::vector<std::shared_ptr<m3d::TransformInterface>> interface_unit_vec = {
    axis_angle_unit_if, dual_quaternion_unit_if, euler_unit_if, matrix_unit_if, quaternion_unit_if
  };
  std::vector<std::shared_ptr<m3d::TransformInterface>> interface1_vec = {
    axis_angle1_if, dual_quaternion1_if, euler1_if, matrix1_if, quaternion1_if
  };
  std::vector<std::shared_ptr<m3d::TransformInterface>> interface2_vec = {
    axis_angle2_if, dual_quaternion2_if, euler2_if, matrix2_if, quaternion2_if
  };
  std::vector<std::shared_ptr<m3d::TransformInterface>> interface12_vec = {
    axis_angle12_if, dual_quaternion12_if, euler12_if, matrix12_if, quaternion12_if
  };
  std::vector<std::shared_ptr<m3d::TransformInterface>> interface1_inv_vec = {
    axis_angle1_inv_if, dual_quaternion1_inv_if, euler1_inv_if, matrix1_inv_if, quaternion1_inv_if
  };

  Eigen::Matrix<double, 3, 5> cloud0;
  Eigen::Matrix<double, 3, 5> cloud1;
  cloud0 << kPoint0Vec, kPoint0Vec, kPoint0Vec, kPoint0Vec, kPoint0Vec;
  cloud1 << kPoint1Vec, kPoint1Vec, kPoint1Vec, kPoint1Vec, kPoint1Vec;

  Eigen::Matrix<double, 3, Eigen::Dynamic> cloud0_dyn;
  Eigen::Matrix<double, 3, Eigen::Dynamic> cloud1_dyn;
  cloud0_dyn.resize(3, 5);
  cloud1_dyn.resize(3, 5);
  cloud0_dyn << kPoint0Vec, kPoint0Vec, kPoint0Vec, kPoint0Vec, kPoint0Vec;
  cloud1_dyn << kPoint1Vec, kPoint1Vec, kPoint1Vec, kPoint1Vec, kPoint1Vec;

  // combine 1 and 2 post
  for (size_t i = 0; i < interface1_vec.size(); ++i)
  {
    // interface
    m3d::TransformInterface::Ptr interface1 = interface1_vec.at(i);
    m3d::TransformInterface::Ptr interface12 = interface12_vec.at(i);

    for (auto interface2 : interface2_vec)
    {
      // copy
      m3d::TransformInterface::Ptr chain12 = interface1->applyPost(interface2);
      EIGEN_EXPECT_APPROX(chain12->normalized()->toEigenVector(), interface12->toEigenVector());

      // inplace
      m3d::TransformInterface::Ptr chain12_inpl = interface1->copy()->applyPost_(interface2);
      EIGEN_EXPECT_APPROX(chain12_inpl->normalized()->toEigenVector(), interface12->toEigenVector());
    }

    // direct
    m3d::TransformInterface::Ptr ia12 = interface1->applyPost(axis_angle2);
    m3d::TransformInterface::Ptr id12 = interface1->applyPost(dual_quaternion2);
    m3d::TransformInterface::Ptr ie12 = interface1->applyPost(euler2);
    m3d::TransformInterface::Ptr im12 = interface1->applyPost(matrix2);
    m3d::TransformInterface::Ptr iq12 = interface1->applyPost(quaternion2);

    EIGEN_EXPECT_APPROX(ia12->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(id12->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(ie12->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(im12->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(iq12->normalized()->toEigenVector(), interface12->toEigenVector());

    // direct inplace
    m3d::TransformInterface::Ptr ia12_inpl = interface1->copy()->applyPost_(axis_angle2);
    m3d::TransformInterface::Ptr id12_inpl = interface1->copy()->applyPost_(dual_quaternion2);
    m3d::TransformInterface::Ptr ie12_inpl = interface1->copy()->applyPost_(euler2);
    m3d::TransformInterface::Ptr im12_inpl = interface1->copy()->applyPost_(matrix2);
    m3d::TransformInterface::Ptr iq12_inpl = interface1->copy()->applyPost_(quaternion2);

    EIGEN_EXPECT_APPROX(ia12_inpl->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(id12_inpl->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(ie12_inpl->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(im12_inpl->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(iq12_inpl->normalized()->toEigenVector(), interface12->toEigenVector());
  }

  // combine 1 and 2 pre
  for (size_t i = 0; i < interface2_vec.size(); ++i)
  {
    // interface
    m3d::TransformInterface::Ptr interface2 = interface2_vec.at(i);
    m3d::TransformInterface::Ptr interface12 = interface12_vec.at(i);

    for (auto interface1 : interface1_vec)
    {
      // copy
      m3d::TransformInterface::Ptr chain12 = interface2->applyPre(interface1);
      EIGEN_EXPECT_APPROX(chain12->normalized()->toEigenVector(), interface12->toEigenVector());

      // inplace
      m3d::TransformInterface::Ptr chain12_inpl = interface2->copy()->applyPre_(interface1);
      EIGEN_EXPECT_APPROX(chain12_inpl->normalized()->toEigenVector(), interface12->toEigenVector());
    }

    // direct
    m3d::TransformInterface::Ptr ia12 = interface2->applyPre(axis_angle1);
    m3d::TransformInterface::Ptr id12 = interface2->applyPre(dual_quaternion1);
    m3d::TransformInterface::Ptr ie12 = interface2->applyPre(euler1);
    m3d::TransformInterface::Ptr im12 = interface2->applyPre(matrix1);
    m3d::TransformInterface::Ptr iq12 = interface2->applyPre(quaternion1);

    EIGEN_EXPECT_APPROX(ia12->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(id12->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(ie12->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(im12->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(iq12->normalized()->toEigenVector(), interface12->toEigenVector());

    // direct inplace
    m3d::TransformInterface::Ptr ia12_inpl = interface2->copy()->applyPre_(axis_angle1);
    m3d::TransformInterface::Ptr id12_inpl = interface2->copy()->applyPre_(dual_quaternion1);
    m3d::TransformInterface::Ptr ie12_inpl = interface2->copy()->applyPre_(euler1);
    m3d::TransformInterface::Ptr im12_inpl = interface2->copy()->applyPre_(matrix1);
    m3d::TransformInterface::Ptr iq12_inpl = interface2->copy()->applyPre_(quaternion1);

    EIGEN_EXPECT_APPROX(ia12_inpl->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(id12_inpl->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(ie12_inpl->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(im12_inpl->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(iq12_inpl->normalized()->toEigenVector(), interface12->toEigenVector());
  }

  // combined 1 and inverted
  for (size_t i = 0; i < interface1_vec.size(); ++i)
  {
    m3d::TransformInterface::Ptr interface1 = interface1_vec.at(i);
    m3d::TransformInterface::Ptr interface_unit = interface_unit_vec.at(i);

    for (auto interface1_inv : interface1_inv_vec)
    {
      // copy
      m3d::TransformInterface::Ptr pre1 = interface1->applyPre(interface1_inv);
      m3d::TransformInterface::Ptr pre2 = interface1_inv->applyPre(interface1)->asType(interface1->getType());
      m3d::TransformInterface::Ptr post1 = interface1->applyPost(interface1_inv);
      m3d::TransformInterface::Ptr post2 = interface1_inv->applyPost(interface1)->asType(interface1->getType());

      EIGEN_EXPECT_APPROX(pre1->normalized()->toEigenVector(), interface_unit->toEigenVector());
      EIGEN_EXPECT_APPROX(pre2->normalized()->toEigenVector(), interface_unit->toEigenVector());
      EIGEN_EXPECT_APPROX(post1->normalized()->toEigenVector(), interface_unit->toEigenVector());
      EIGEN_EXPECT_APPROX(post2->normalized()->toEigenVector(), interface_unit->toEigenVector());

      // inplace
      m3d::TransformInterface::Ptr pre1_inpl = interface1->copy()->applyPre_(interface1_inv);
      m3d::TransformInterface::Ptr pre2_inpl = interface1_inv->copy()->applyPre_(interface1)->asType(interface1->getType());
      m3d::TransformInterface::Ptr post1_inpl = interface1->copy()->applyPost_(interface1_inv);
      m3d::TransformInterface::Ptr post2_inpl = interface1_inv->copy()->applyPost_(interface1)->asType(interface1->getType());

      EIGEN_EXPECT_APPROX(pre1_inpl->normalized()->toEigenVector(), interface_unit->toEigenVector());
      EIGEN_EXPECT_APPROX(pre2_inpl->normalized()->toEigenVector(), interface_unit->toEigenVector());
      EIGEN_EXPECT_APPROX(post1_inpl->normalized()->toEigenVector(), interface_unit->toEigenVector());
      EIGEN_EXPECT_APPROX(post2_inpl->normalized()->toEigenVector(), interface_unit->toEigenVector());
    }

    // point
    EIGEN_EXPECT_APPROX(interface1->transformPoint(kPoint0Vec), kPoint1Vec);

    // cloud
    EIGEN_EXPECT_APPROX(interface1->transformCloud(cloud0), cloud1);
    EIGEN_EXPECT_APPROX(interface1->transformCloud(cloud0_dyn), cloud1_dyn);
  }
}

TEST(TransformResult, norm)
{
  // generate transforms
  auto axis_angle1_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle1Vec);
  auto dual_quaternion1_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion1Vec);
  auto euler1_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler1Vec);
  auto matrix1_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix1Vec);
  auto quaternion1_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion1Vec);

  std::vector<std::shared_ptr<m3d::TransformInterface>> interface1_vec = {
    axis_angle1_if, dual_quaternion1_if, euler1_if, matrix1_if, quaternion1_if
  };

  // check norms
  for (auto interface1 : interface1_vec)
  {
    EXPECT_NEAR(interface1->rotationNorm(), kRotationNorm1, 1e-6);
    EXPECT_NEAR(interface1->translationNorm(), kTranslationNorm1, 1e-6);
  }

  // generate transforms
  auto axis_angle2_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle2Vec);
  auto dual_quaternion2_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion2Vec);
  auto euler2_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler2Vec);
  auto matrix2_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix2Vec);
  auto quaternion2_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion2Vec);

  std::vector<std::shared_ptr<m3d::TransformInterface>> interface2_vec = {
    axis_angle2_if, dual_quaternion2_if, euler2_if, matrix2_if, quaternion2_if
  };

  // check norms
  for (auto interface2 : interface2_vec)
  {
    EXPECT_NEAR(interface2->rotationNorm(), kRotationNorm2, 1e-6);
    EXPECT_NEAR(interface2->translationNorm(), kTranslationNorm2, 1e-6);
  }
}

TEST(TransformResult, operatorsDirect)
{
  // generate data
  auto axis_angle1 = m3d::AxisAngleTransform(kAxisAngle1Vec);
  auto dual_quaternion1 = m3d::DualQuaternionTransform(kDualQuaternion1Vec);
  auto euler1 = m3d::EulerTransform(kEuler1Vec);
  auto matrix1 = m3d::MatrixTransform(kMatrix1Vec);
  auto quaternion1 = m3d::QuaternionTransform(kQuaternion1Vec);

  auto axis_angle1_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle1Vec);
  auto dual_quaternion1_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion1Vec);
  auto euler1_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler1Vec);
  auto matrix1_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix1Vec);
  auto quaternion1_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion1Vec);

  auto axis_angle2 = m3d::AxisAngleTransform(kAxisAngle2Vec);
  auto dual_quaternion2 = m3d::DualQuaternionTransform(kDualQuaternion2Vec);
  auto euler2 = m3d::EulerTransform(kEuler2Vec);
  auto matrix2 = m3d::MatrixTransform(kMatrix2Vec);
  auto quaternion2 = m3d::QuaternionTransform(kQuaternion2Vec);

  auto axis_angle2_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle2Vec);
  auto dual_quaternion2_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion2Vec);
  auto euler2_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler2Vec);
  auto matrix2_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix2Vec);
  auto quaternion2_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion2Vec);

  auto axis_angle1_inv = m3d::AxisAngleTransform(kAxisAngle1InvVec);
  auto dual_quaternion1_inv = m3d::DualQuaternionTransform(kDualQuaternion1InvVec);
  auto euler1_inv = m3d::EulerTransform(kEuler1InvVec);
  auto matrix1_inv = m3d::MatrixTransform(kMatrix1InvVec);
  auto quaternion1_inv = m3d::QuaternionTransform(kQuaternion1InvVec);

  std::vector<std::shared_ptr<m3d::TransformInterface>> interface1_vec = {
    axis_angle1_if, dual_quaternion1_if, euler1_if, matrix1_if, quaternion1_if
  };

  std::vector<std::shared_ptr<m3d::TransformInterface>> interface2_vec = {
    axis_angle2_if, dual_quaternion2_if, euler2_if, matrix2_if, quaternion2_if
  };

  // check
  auto operatorsDirectFun = [&](const auto &transform, const auto &vec12, const auto &unit_vec) {
    // multiply
    auto ma = transform * axis_angle2;
    auto md = transform * dual_quaternion2;
    auto me = transform * euler2;
    auto mm = transform * matrix2;
    auto mq = transform * quaternion2;

    EIGEN_EXPECT_APPROX(ma.normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(md.normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(me.normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(mm.normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(mq.normalized().toEigenVector(), vec12);

    auto ma_copy = transform.copy();
    auto md_copy = transform.copy();
    auto me_copy = transform.copy();
    auto mm_copy = transform.copy();
    auto mq_copy = transform.copy();

    ma_copy *= axis_angle2;
    md_copy *= dual_quaternion2;
    me_copy *= euler2;
    mm_copy *= matrix2;
    mq_copy *= quaternion2;

    EIGEN_EXPECT_APPROX(ma_copy.normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(md_copy.normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(me_copy.normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(mm_copy.normalized().toEigenVector(), vec12);
    EIGEN_EXPECT_APPROX(mq_copy.normalized().toEigenVector(), vec12);

    // divide
    auto da = transform / axis_angle1;
    auto dd = transform / dual_quaternion1;
    auto de = transform / euler1;
    auto dm = transform / matrix1;
    auto dq = transform / quaternion1;

    EIGEN_EXPECT_APPROX(da.normalized().toEigenVector(), unit_vec);
    EIGEN_EXPECT_APPROX(dd.normalized().toEigenVector(), unit_vec);
    EIGEN_EXPECT_APPROX(de.normalized().toEigenVector(), unit_vec);
    EIGEN_EXPECT_APPROX(dm.normalized().toEigenVector(), unit_vec);
    EIGEN_EXPECT_APPROX(dq.normalized().toEigenVector(), unit_vec);

    auto da_copy = transform.copy();
    auto dd_copy = transform.copy();
    auto de_copy = transform.copy();
    auto dm_copy = transform.copy();
    auto dq_copy = transform.copy();

    da_copy /= axis_angle1;
    dd_copy /= dual_quaternion1;
    de_copy /= euler1;
    dm_copy /= matrix1;
    dq_copy /= quaternion1;

    EIGEN_EXPECT_APPROX(da_copy.normalized().toEigenVector(), unit_vec);
    EIGEN_EXPECT_APPROX(dd_copy.normalized().toEigenVector(), unit_vec);
    EIGEN_EXPECT_APPROX(de_copy.normalized().toEigenVector(), unit_vec);
    EIGEN_EXPECT_APPROX(dm_copy.normalized().toEigenVector(), unit_vec);
    EIGEN_EXPECT_APPROX(dq_copy.normalized().toEigenVector(), unit_vec);

    // interface
    for (auto interface2 : interface2_vec)
    {
      auto mi = transform * interface2;
      EIGEN_EXPECT_APPROX(mi.normalized().toEigenVector(), vec12);

      auto mi_copy = transform.copy();
      mi_copy *= interface2;
      EIGEN_EXPECT_APPROX(mi_copy.normalized().toEigenVector(), vec12);
    }
    for (auto interface1 : interface1_vec)
    {
      auto di = transform / interface1;
      EIGEN_EXPECT_APPROX(di.normalized().toEigenVector(), unit_vec);

      auto di_copy = transform.copy();
      di_copy /= interface1;
      EIGEN_EXPECT_APPROX(di_copy.normalized().toEigenVector(), unit_vec);
    }
  };

  operatorsDirectFun(axis_angle1, kAxisAngle12Vec, kAxisAngleUnitVec);
  operatorsDirectFun(dual_quaternion1, kDualQuaternion12Vec, kDualQuaternionUnitVec);
  operatorsDirectFun(euler1, kEuler12Vec, kEulerUnitVec);
  operatorsDirectFun(matrix1, kMatrix12Vec, kMatrixUnitVec);
  operatorsDirectFun(quaternion1, kQuaternion12Vec, kQuaternionUnitVec);
}

TEST(TransformResult, operatorsInterface)
{
  // generate data
  auto axis_angle_unit_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngleUnitVec);
  auto dual_quaternion_unit_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternionUnitVec);
  auto euler_unit_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEulerUnitVec);
  auto matrix_unit_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrixUnitVec);
  auto quaternion_unit_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternionUnitVec);

  auto axis_angle1 = m3d::AxisAngleTransform(kAxisAngle1Vec);
  auto dual_quaternion1 = m3d::DualQuaternionTransform(kDualQuaternion1Vec);
  auto euler1 = m3d::EulerTransform(kEuler1Vec);
  auto matrix1 = m3d::MatrixTransform(kMatrix1Vec);
  auto quaternion1 = m3d::QuaternionTransform(kQuaternion1Vec);

  auto axis_angle1_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle1Vec);
  auto dual_quaternion1_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion1Vec);
  auto euler1_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler1Vec);
  auto matrix1_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix1Vec);
  auto quaternion1_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion1Vec);

  auto axis_angle2 = m3d::AxisAngleTransform(kAxisAngle2Vec);
  auto dual_quaternion2 = m3d::DualQuaternionTransform(kDualQuaternion2Vec);
  auto euler2 = m3d::EulerTransform(kEuler2Vec);
  auto matrix2 = m3d::MatrixTransform(kMatrix2Vec);
  auto quaternion2 = m3d::QuaternionTransform(kQuaternion2Vec);

  auto axis_angle2_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle2Vec);
  auto dual_quaternion2_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion2Vec);
  auto euler2_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler2Vec);
  auto matrix2_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix2Vec);
  auto quaternion2_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion2Vec);

  auto axis_angle12_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle12Vec);
  auto dual_quaternion12_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion12Vec);
  auto euler12_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler12Vec);
  auto matrix12_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix12Vec);
  auto quaternion12_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion12Vec);

  auto axis_angle1_inv_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle1InvVec);
  auto dual_quaternion1_inv_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion1InvVec);
  auto euler1_inv_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler1InvVec);
  auto matrix1_inv_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix1InvVec);
  auto quaternion1_inv_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion1InvVec);

  std::vector<std::shared_ptr<m3d::TransformInterface>> interface_unit_vec = {
    axis_angle_unit_if, dual_quaternion_unit_if, euler_unit_if, matrix_unit_if, quaternion_unit_if
  };
  std::vector<std::shared_ptr<m3d::TransformInterface>> interface1_vec = {
    axis_angle1_if, dual_quaternion1_if, euler1_if, matrix1_if, quaternion1_if
  };
  std::vector<std::shared_ptr<m3d::TransformInterface>> interface2_vec = {
    axis_angle2_if, dual_quaternion2_if, euler2_if, matrix2_if, quaternion2_if
  };
  std::vector<std::shared_ptr<m3d::TransformInterface>> interface12_vec = {
    axis_angle12_if, dual_quaternion12_if, euler12_if, matrix12_if, quaternion12_if
  };
  std::vector<std::shared_ptr<m3d::TransformInterface>> interface1_inv_vec = {
    axis_angle1_inv_if, dual_quaternion1_inv_if, euler1_inv_if, matrix1_inv_if, quaternion1_inv_if
  };

  for (size_t i = 0; i < interface1_vec.size(); ++i)
  {
    // access vectors
    m3d::TransformInterface::Ptr interface1 = interface1_vec.at(i);
    m3d::TransformInterface::Ptr interface12 = interface12_vec.at(i);
    m3d::TransformInterface::Ptr interface_unit = interface_unit_vec.at(i);
    m3d::TransformInterface::Ptr interface1_inv = interface1_inv_vec.at(i);

    // interface
    for (auto interface2 : interface2_vec)
    {
      m3d::TransformInterface::Ptr imi = interface1 * interface2;
      EIGEN_EXPECT_APPROX(imi->normalized()->toEigenVector(), interface12->toEigenVector());

      m3d::TransformInterface::Ptr idi = interface1 / interface1;
      EIGEN_EXPECT_APPROX(idi->normalized()->toEigenVector(), interface_unit->normalized()->toEigenVector());
    }

    // direct
    m3d::TransformInterface::Ptr ima = interface1 * axis_angle2;
    m3d::TransformInterface::Ptr imd = interface1 * dual_quaternion2;
    m3d::TransformInterface::Ptr ime = interface1 * euler2;
    m3d::TransformInterface::Ptr imm = interface1 * matrix2;
    m3d::TransformInterface::Ptr imq = interface1 * quaternion2;

    EIGEN_EXPECT_APPROX(ima->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(imd->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(ime->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(imm->normalized()->toEigenVector(), interface12->toEigenVector());
    EIGEN_EXPECT_APPROX(imq->normalized()->toEigenVector(), interface12->toEigenVector());

    m3d::TransformInterface::Ptr ida = interface1 / axis_angle1;
    m3d::TransformInterface::Ptr idd = interface1 / dual_quaternion1;
    m3d::TransformInterface::Ptr ide = interface1 / euler1;
    m3d::TransformInterface::Ptr idm = interface1 / matrix1;
    m3d::TransformInterface::Ptr idq = interface1 / quaternion1;

    EIGEN_EXPECT_APPROX(ida->normalized()->toEigenVector(), interface_unit->toEigenVector());
    EIGEN_EXPECT_APPROX(idd->normalized()->toEigenVector(), interface_unit->toEigenVector());
    EIGEN_EXPECT_APPROX(ide->normalized()->toEigenVector(), interface_unit->toEigenVector());
    EIGEN_EXPECT_APPROX(idm->normalized()->toEigenVector(), interface_unit->toEigenVector());
    EIGEN_EXPECT_APPROX(idq->normalized()->toEigenVector(), interface_unit->toEigenVector());
  }
}

void transformResultApplyRandom(void)
{
  // generate data
  m3d::TransformInterface::Ptr euler_if1 = createRandomEulerTransformInterface();
  m3d::TransformInterface::Ptr axis_angle_if1 = euler_if1->asType<m3d::AxisAngleTransform>();
  m3d::TransformInterface::Ptr dual_quaternion_if1 = euler_if1->asType<m3d::DualQuaternionTransform>();
  m3d::TransformInterface::Ptr matrix_if1 = euler_if1->asType<m3d::MatrixTransform>();
  m3d::TransformInterface::Ptr quat_if1 = euler_if1->asType<m3d::QuaternionTransform>();

  m3d::TransformInterface::Ptr euler_if2 = createRandomEulerTransformInterface();
  m3d::TransformInterface::Ptr axis_angle_if2 = euler_if2->asType<m3d::AxisAngleTransform>();
  m3d::TransformInterface::Ptr dual_quaternion_if2 = euler_if2->asType<m3d::DualQuaternionTransform>();
  m3d::TransformInterface::Ptr matrix_if2 = euler_if2->asType<m3d::MatrixTransform>();
  m3d::TransformInterface::Ptr quat_if2 = euler_if2->asType<m3d::QuaternionTransform>();

  std::vector<std::shared_ptr<m3d::TransformInterface>> interface_inv_vec1 = {
    axis_angle_if1, dual_quaternion_if1, euler_if1, matrix_if1, quat_if1
  };
   std::vector<std::shared_ptr<m3d::TransformInterface>> interface_inv_vec2 = {
    axis_angle_if2, dual_quaternion_if2, euler_if2, matrix_if2, quat_if2
  };

  m3d::TransformInterface::Ptr identity_if = std::make_shared<m3d::DualQuaternionTransform>();
  m3d::TransformInterface::Ptr fixed_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion1Vec);
  m3d::TransformInterface::Ptr dual_quaternion_if12 = dual_quaternion_if1 * dual_quaternion_if2;

  // apply
  for (auto interface1 : interface_inv_vec1)
  {
    ASSERT_TRANSFORM_APPROX(identity_if, interface1 * interface1->inverse());
    ASSERT_TRANSFORM_APPROX(identity_if, interface1->inverse() * interface1);
    ASSERT_TRANSFORM_APPROX(identity_if, interface1 / interface1);
    EXPECT_NEAR((interface1 / interface1)->rotationNorm(), 0.0, 1e-6);
    EXPECT_NEAR((interface1 / interface1)->translationNorm(), 0.0, 1e-6);
    ASSERT_FALSE(interface1->isEqual(interface1 * fixed_if));

    for (auto interface2 : interface_inv_vec2)
    {
      ASSERT_TRANSFORM_APPROX(dual_quaternion_if12, interface1 * interface2);
      ASSERT_TRANSFORM_APPROX(dual_quaternion_if12, interface1->applyPost(interface2));
      ASSERT_TRANSFORM_APPROX(dual_quaternion_if12, interface2->applyPre(interface1));
      ASSERT_TRANSFORM_APPROX(dual_quaternion_if12, interface1->copy()->applyPost_(interface2));
      ASSERT_TRANSFORM_APPROX(dual_quaternion_if12, interface2->copy()->applyPre_(interface1));
      ASSERT_TRANSFORM_APPROX(dual_quaternion_if1, (interface1 * interface2) / interface2);
      ASSERT_TRUE(dual_quaternion_if12->isEqual(interface1 * interface2));
    }
  }
}

TEST(TransformResult, applyRandom)
{
  for (size_t i = 0; i < kRandomRuns; ++i)
  {
    transformResultApplyRandom();
  }
}