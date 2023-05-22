#pragma once

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <motion3d/transforms.hpp>

#include "data.hpp"

namespace m3d = motion3d;


TEST(TransformInterface, types)
{
  // from char 
  EXPECT_EQ(m3d::transformTypeFromChar('A'), m3d::TransformType::kAxisAngle);
  EXPECT_EQ(m3d::transformTypeFromChar('D'), m3d::TransformType::kDualQuaternion);
  EXPECT_EQ(m3d::transformTypeFromChar('E'), m3d::TransformType::kEuler);
  EXPECT_EQ(m3d::transformTypeFromChar('M'), m3d::TransformType::kMatrix);
  EXPECT_EQ(m3d::transformTypeFromChar('Q'), m3d::TransformType::kQuaternion);

  EXPECT_EQ(m3d::transformTypeFromChar('a'), m3d::TransformType::kAxisAngle);
  EXPECT_EQ(m3d::transformTypeFromChar('d'), m3d::TransformType::kDualQuaternion);
  EXPECT_EQ(m3d::transformTypeFromChar('e'), m3d::TransformType::kEuler);
  EXPECT_EQ(m3d::transformTypeFromChar('m'), m3d::TransformType::kMatrix);
  EXPECT_EQ(m3d::transformTypeFromChar('q'), m3d::TransformType::kQuaternion);

  // to char
  EXPECT_EQ('A', m3d::transformTypeToChar(m3d::TransformType::kAxisAngle));
  EXPECT_EQ('D', m3d::transformTypeToChar(m3d::TransformType::kDualQuaternion));
  EXPECT_EQ('E', m3d::transformTypeToChar(m3d::TransformType::kEuler));
  EXPECT_EQ('M', m3d::transformTypeToChar(m3d::TransformType::kMatrix));
  EXPECT_EQ('Q', m3d::transformTypeToChar(m3d::TransformType::kQuaternion));

  // error handling
  EXPECT_THROW(m3d::transformTypeFromChar('X'), m3d::InvalidTransformTypeException);
}

TEST(TransformInterface, constructors)
{
  Eigen::Matrix<double, 3, 1> translation = kAxisAngle1Vec.block<3, 1>(0, 0);
  Eigen::Matrix<double, Eigen::Dynamic, 1> invalid_size = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(2);

  // AxisAngleTransform
  double angle = kAxisAngle1Vec(3);
  Eigen::Matrix<double, 3, 1> axis = kAxisAngle1Vec.block<3, 1>(4, 0);
  Eigen::AngleAxis<double> angle_axis(angle, axis);
  Eigen::Matrix<double, Eigen::Dynamic, 1> axis_angle_vec = kAxisAngle1Vec;

  m3d::AxisAngleTransform a1;
  m3d::AxisAngleTransform a2(translation, angle_axis);
  m3d::AxisAngleTransform a3(translation, angle, axis);
  m3d::AxisAngleTransform a4(eigenToVector(kAxisAngle1Vec));
  m3d::AxisAngleTransform a5(kAxisAngle1Vec);
  m3d::AxisAngleTransform a6(axis_angle_vec);

  EIGEN_EXPECT_APPROX(a1.toEigenVector(), kAxisAngleUnitVec);
  EIGEN_EXPECT_APPROX(a2.toEigenVector(), kAxisAngle1Vec);
  EIGEN_EXPECT_APPROX(a3.toEigenVector(), kAxisAngle1Vec);
  EIGEN_EXPECT_APPROX(a4.toEigenVector(), kAxisAngle1Vec);
  EIGEN_EXPECT_APPROX(a5.toEigenVector(), kAxisAngle1Vec);
  EIGEN_EXPECT_APPROX(a6.toEigenVector(), kAxisAngle1Vec);

  Eigen::Matrix<double, 3, 1> axis_invalid = Eigen::Matrix<double, 3, 1>::Zero();
  Eigen::AngleAxis<double> angle_axis_invalid(0.0, axis_invalid);
  Eigen::Matrix<double, 7, 1> axis_angle_vec_invalid1 = Eigen::Matrix<double, 7, 1>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> axis_angle_vec_invalid2 = axis_angle_vec_invalid1;

  EXPECT_THROW(std::make_shared<m3d::AxisAngleTransform>(translation, angle_axis_invalid), m3d::InvalidTransformException);
  EXPECT_THROW(std::make_shared<m3d::AxisAngleTransform>(translation, 0.0, axis_invalid), m3d::InvalidTransformException);
  EXPECT_THROW(std::make_shared<m3d::AxisAngleTransform>(eigenToVector(axis_angle_vec_invalid1)), m3d::InvalidTransformException);
  EXPECT_THROW(std::make_shared<m3d::AxisAngleTransform>(axis_angle_vec_invalid1), m3d::InvalidTransformException);
  EXPECT_THROW(std::make_shared<m3d::AxisAngleTransform>(axis_angle_vec_invalid2), m3d::InvalidTransformException);
  EXPECT_THROW(std::make_shared<m3d::AxisAngleTransform>(invalid_size), m3d::InvalidTypeSizeException);

  // DualQuaternionTransform
  m3d::Quaternion<double> real(kDualQuaternion1Vec(0), kDualQuaternion1Vec(1), kDualQuaternion1Vec(2), kDualQuaternion1Vec(3));
  m3d::Quaternion<double> dual(kDualQuaternion1Vec(4), kDualQuaternion1Vec(5), kDualQuaternion1Vec(6), kDualQuaternion1Vec(7));
  Eigen::Matrix<double, 4, 1> real_mat = real.toEigenVector();
  Eigen::Matrix<double, 4, 1> dual_mat = dual.toEigenVector();
  Eigen::Matrix<double, Eigen::Dynamic, 1> dual_quaternion_vec = kDualQuaternion1Vec;

  m3d::DualQuaternionTransform d1;
  m3d::DualQuaternionTransform d2(real, dual);
  m3d::DualQuaternionTransform d3(real_mat, dual_mat);
  m3d::DualQuaternionTransform d4(eigenToVector(kDualQuaternion1Vec));
  m3d::DualQuaternionTransform d5(kDualQuaternion1Vec);
  m3d::DualQuaternionTransform d6(dual_quaternion_vec);

  EIGEN_EXPECT_APPROX(d1.toEigenVector(), kDualQuaternionUnitVec);
  EIGEN_EXPECT_APPROX(d2.toEigenVector(), kDualQuaternion1Vec);
  EIGEN_EXPECT_APPROX(d3.toEigenVector(), kDualQuaternion1Vec);
  EIGEN_EXPECT_APPROX(d4.toEigenVector(), kDualQuaternion1Vec);
  EIGEN_EXPECT_APPROX(d5.toEigenVector(), kDualQuaternion1Vec);
  EIGEN_EXPECT_APPROX(d6.toEigenVector(), kDualQuaternion1Vec);

  Eigen::Matrix<double, 8, 1> dual_quaternion_vec_invalid1 = Eigen::Matrix<double, 8, 1>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> dual_quaternion_vec_invalid2 = dual_quaternion_vec_invalid1;

  EXPECT_THROW(std::make_shared<m3d::DualQuaternionTransform>(dual, dual), m3d::InvalidTransformException);
  EXPECT_THROW(std::make_shared<m3d::DualQuaternionTransform>(eigenToVector(dual_quaternion_vec_invalid1)), m3d::InvalidTransformException);
  EXPECT_THROW(std::make_shared<m3d::DualQuaternionTransform>(dual_quaternion_vec_invalid1), m3d::InvalidTransformException);
  EXPECT_THROW(std::make_shared<m3d::DualQuaternionTransform>(dual_quaternion_vec_invalid2), m3d::InvalidTransformException);
  EXPECT_THROW(std::make_shared<m3d::DualQuaternionTransform>(invalid_size), m3d::InvalidTypeSizeException);

  // EulerTransform
  Eigen::Matrix<double, 3, 1> angles = kEuler1Vec.block<3, 1>(3, 0);
  Eigen::Matrix<double, Eigen::Dynamic, 1> euler_vec = kEuler1Vec;

  m3d::EulerTransform e1;
  m3d::EulerTransform e2(translation, angles(0), angles(1), angles(2), m3d::EulerAxes::kSXYZ);
  m3d::EulerTransform e3(translation, angles, m3d::EulerAxes::kSXYZ);
  m3d::EulerTransform e4(eigenToVector(kEuler1Vec));
  m3d::EulerTransform e5(kEuler1Vec);
  m3d::EulerTransform e6(euler_vec);

  EIGEN_EXPECT_APPROX(e1.toEigenVector(), kEulerUnitVec);
  EIGEN_EXPECT_APPROX(e2.toEigenVector(), kEuler1Vec);
  EIGEN_EXPECT_APPROX(e3.toEigenVector(), kEuler1Vec);
  EIGEN_EXPECT_APPROX(e4.toEigenVector(), kEuler1Vec);
  EIGEN_EXPECT_APPROX(e5.toEigenVector(), kEuler1Vec);
  EIGEN_EXPECT_APPROX(e6.toEigenVector(), kEuler1Vec);

  Eigen::Matrix<double, 7, 1> euler_vec_invalid1 = Eigen::Matrix<double, 7, 1>::Zero();
  Eigen::Matrix<double, 7, 1> euler_vec_invalid2 = Eigen::Matrix<double, 7, 1>::Zero();
  Eigen::Matrix<double, 7, 1> euler_vec_invalid3 = Eigen::Matrix<double, 7, 1>::Zero();
  euler_vec_invalid1(6) = 255;
  euler_vec_invalid2(6) = -256;
  euler_vec_invalid3(6) = 256;
  Eigen::Matrix<double, Eigen::Dynamic, 1> euler_vec_invalid4 = euler_vec_invalid1;

  EXPECT_THROW(std::make_shared<m3d::EulerTransform>(eigenToVector(euler_vec_invalid1)), m3d::InvalidEulerAxesException);
  EXPECT_THROW(std::make_shared<m3d::EulerTransform>(euler_vec_invalid1), m3d::InvalidEulerAxesException);
  EXPECT_THROW(std::make_shared<m3d::EulerTransform>(eigenToVector(euler_vec_invalid2)), m3d::InvalidEulerAxesException);
  EXPECT_THROW(std::make_shared<m3d::EulerTransform>(euler_vec_invalid2), m3d::InvalidEulerAxesException);
  EXPECT_THROW(std::make_shared<m3d::EulerTransform>(eigenToVector(euler_vec_invalid3)), m3d::InvalidEulerAxesException);
  EXPECT_THROW(std::make_shared<m3d::EulerTransform>(euler_vec_invalid3), m3d::InvalidEulerAxesException);
  EXPECT_THROW(std::make_shared<m3d::EulerTransform>(euler_vec_invalid4), m3d::InvalidEulerAxesException);
  EXPECT_THROW(std::make_shared<m3d::EulerTransform>(invalid_size), m3d::InvalidTypeSizeException);

  // MatrixTransform
  Eigen::Matrix<double, 3, 4> matrix34;
  matrix34 << kMatrix1Vec(0), kMatrix1Vec(1),  kMatrix1Vec(2),  kMatrix1Vec(3),
              kMatrix1Vec(4), kMatrix1Vec(5),  kMatrix1Vec(6),  kMatrix1Vec(7),
              kMatrix1Vec(8), kMatrix1Vec(9), kMatrix1Vec(10), kMatrix1Vec(11);
  Eigen::Matrix<double, 4, 4> matrix44;
  matrix44.setIdentity();
  matrix44.block<3, 4>(0, 0) = matrix34;
  Eigen::Matrix<double, 3, 3> rotation_matrix = matrix34.block<3, 3>(0, 0);
  Eigen::Matrix<double, Eigen::Dynamic, 1> matrix_vec = kMatrix1Vec;

  m3d::MatrixTransform m1;
  m3d::MatrixTransform m2(matrix44);
  m3d::MatrixTransform m3(matrix34);
  m3d::MatrixTransform m4(translation, rotation_matrix);
  m3d::MatrixTransform m5(eigenToVector(kMatrix1Vec));
  m3d::MatrixTransform m6(kMatrix1Vec);
  m3d::MatrixTransform m7(matrix_vec);

  EIGEN_EXPECT_APPROX(m1.toEigenVector(), kMatrixUnitVec);
  EIGEN_EXPECT_APPROX(m2.toEigenVector(), kMatrix1Vec);
  EIGEN_EXPECT_APPROX(m3.toEigenVector(), kMatrix1Vec);
  EIGEN_EXPECT_APPROX(m4.toEigenVector(), kMatrix1Vec);
  EIGEN_EXPECT_APPROX(m5.toEigenVector(), kMatrix1Vec);
  EIGEN_EXPECT_APPROX(m6.toEigenVector(), kMatrix1Vec);
  EIGEN_EXPECT_APPROX(m7.toEigenVector(), kMatrix1Vec);

  Eigen::Matrix<double, 4, 4> matrix1_invalid = Eigen::Matrix<double, 4, 4>::Zero();
  Eigen::Matrix<double, 3, 4> matrix2_invalid = Eigen::Matrix<double, 3, 4>::Zero();
  Eigen::Matrix<double, 3, 3> rotation_matrix_invalid = Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, 12, 1> matrix_vec_invalid1 = Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> matrix_vec_invalid2 = matrix_vec_invalid1;

  EXPECT_THROW(std::make_shared<m3d::MatrixTransform>(matrix1_invalid), m3d::InvalidTransformException);
  EXPECT_THROW(std::make_shared<m3d::MatrixTransform>(matrix2_invalid), m3d::InvalidTransformException);
  EXPECT_THROW(std::make_shared<m3d::MatrixTransform>(translation, rotation_matrix_invalid), m3d::InvalidTransformException);
  EXPECT_THROW(std::make_shared<m3d::MatrixTransform>(eigenToVector(matrix_vec_invalid1)), m3d::InvalidTransformException);
  EXPECT_THROW(std::make_shared<m3d::MatrixTransform>(matrix_vec_invalid1), m3d::InvalidTransformException);
  EXPECT_THROW(std::make_shared<m3d::MatrixTransform>(matrix_vec_invalid2), m3d::InvalidTransformException);
  EXPECT_THROW(std::make_shared<m3d::MatrixTransform>(invalid_size), m3d::InvalidTypeSizeException);

  // QuaternionTransform
  m3d::Quaternion<double> quaternion(kQuaternion1Vec(3), kQuaternion1Vec(4), kQuaternion1Vec(5), kQuaternion1Vec(6));
  Eigen::Matrix<double, 4, 1> quaternion_mat = quaternion.toEigenVector();
  Eigen::Matrix<double, Eigen::Dynamic, 1> quaternion_vec = kQuaternion1Vec;

  m3d::QuaternionTransform q1;
  m3d::QuaternionTransform q2(translation, quaternion);
  m3d::QuaternionTransform q3(translation, quaternion_mat);
  m3d::QuaternionTransform q4(eigenToVector(kQuaternion1Vec));
  m3d::QuaternionTransform q5(kQuaternion1Vec);
  m3d::QuaternionTransform q6(quaternion_vec);

  EIGEN_EXPECT_APPROX(q1.toEigenVector(), kQuaternionUnitVec);
  EIGEN_EXPECT_APPROX(q2.toEigenVector(), kQuaternion1Vec);
  EIGEN_EXPECT_APPROX(q3.toEigenVector(), kQuaternion1Vec);
  EIGEN_EXPECT_APPROX(q4.toEigenVector(), kQuaternion1Vec);
  EIGEN_EXPECT_APPROX(q5.toEigenVector(), kQuaternion1Vec);
  EIGEN_EXPECT_APPROX(q6.toEigenVector(), kQuaternion1Vec);

  Eigen::Matrix<double, 7, 1> quaternion_vec_invalid1 = Eigen::Matrix<double, 7, 1>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> quaternion_vec_invalid2 = quaternion_vec_invalid1;

  EXPECT_THROW(std::make_shared<m3d::QuaternionTransform>(translation, dual), m3d::InvalidTransformException);
  EXPECT_THROW(std::make_shared<m3d::QuaternionTransform>(eigenToVector(quaternion_vec_invalid1)), m3d::InvalidTransformException);
  EXPECT_THROW(std::make_shared<m3d::QuaternionTransform>(quaternion_vec_invalid2), m3d::InvalidTransformException);
  EXPECT_THROW(std::make_shared<m3d::QuaternionTransform>(invalid_size), m3d::InvalidTypeSizeException);
}

TEST(TransformInterface, getter)
{
  Eigen::Matrix<double, 3, 1> translation = kAxisAngle1Vec.block<3, 1>(0, 0);

  // AxisAngleTransform
  m3d::AxisAngleTransform axis_angle(kAxisAngle1Vec);
  Eigen::Matrix<double, 3, 1> axis = kAxisAngle1Vec.block<3, 1>(4, 0);

  EIGEN_EXPECT_APPROX(axis_angle.getTranslation(), translation);
  EXPECT_EQ(axis_angle.getAngle(), kAxisAngle1Vec(3));
  EIGEN_EXPECT_APPROX(axis_angle.getAxis(), axis);

  // DualQuaternionTransform
  m3d::DualQuaternionTransform dual_quaternion(kDualQuaternion1Vec);
  Eigen::Matrix<double, 4, 1> real = kDualQuaternion1Vec.block<4, 1>(0, 0);
  Eigen::Matrix<double, 4, 1> dual = kDualQuaternion1Vec.block<4, 1>(4, 0);

  EIGEN_EXPECT_APPROX(dual_quaternion.getDualQuaternion().toEigenVector(), kDualQuaternion1Vec);
  EIGEN_EXPECT_APPROX(dual_quaternion.getReal().toEigenVector(), real);
  EIGEN_EXPECT_APPROX(dual_quaternion.getDual().toEigenVector(), dual);

  m3d::Quaternion<double> translation_quat = dual_quaternion.getDualQuaternion().getTranslationQuaternion();
  EXPECT_NEAR(translation_quat.w(), 0.0, 1e-6);
  EIGEN_EXPECT_APPROX(translation_quat.vec(), translation);
  
  // EulerTransform
  m3d::EulerTransform euler(kEuler1Vec);
  Eigen::Matrix<double, 3, 1> angles = kEuler1Vec.block<3, 1>(3, 0);

  EIGEN_EXPECT_APPROX(euler.getTranslation(), translation);
  EIGEN_EXPECT_APPROX(euler.getAngles(), angles);
  EXPECT_EQ(euler.getAi(), angles(0));
  EXPECT_EQ(euler.getAj(), angles(1));
  EXPECT_EQ(euler.getAk(), angles(2));
  EXPECT_EQ(euler.getAxes(), m3d::EulerAxes::kSXYZ);

  // MatrixTransform
  m3d::MatrixTransform matrix(kMatrix1Vec);
  Eigen::Matrix<double, 4, 4> matrix44;
  matrix44 << kMatrix1Vec(0), kMatrix1Vec(1),  kMatrix1Vec(2),  kMatrix1Vec(3),
              kMatrix1Vec(4), kMatrix1Vec(5),  kMatrix1Vec(6),  kMatrix1Vec(7),
              kMatrix1Vec(8), kMatrix1Vec(9), kMatrix1Vec(10), kMatrix1Vec(11),
                         0.0,            0.0,             0.0,             1.0;
  Eigen::Matrix<double, 3, 3> rotation_matrix = matrix44.block<3, 3>(0, 0);
  
  EXPECT_EQ(matrix.getMatrix(), matrix44);
  EXPECT_EQ(matrix.getTranslation(), translation);
  EXPECT_EQ(matrix.getRotationMatrix(), rotation_matrix);

  // QuaternionTransform
  m3d::QuaternionTransform quaternion(kQuaternion1Vec);
  Eigen::Matrix<double, 4, 1> quat = kQuaternion1Vec.block<4, 1>(3, 0);

  EIGEN_EXPECT_APPROX(quaternion.getTranslation(), translation);
  EIGEN_EXPECT_APPROX(quaternion.getQuaternion().toEigenVector(), quat);
}

TEST(TransformInterface, setter)
{
  // AxisAngleTransform
  m3d::AxisAngleTransform aa;
  m3d::AxisAngleTransform aa1(kAxisAngle1Vec);
  m3d::AxisAngleTransform aa2(kAxisAngle2Vec);

  EIGEN_EXPECT_EQ(aa.setTranslation(aa1.getTranslation()).getTranslation(), aa1.getTranslation());
  EIGEN_EXPECT_EQ(aa.setAngleAxis(aa1.getAngleAxis()).toEigenVector(), aa1.toEigenVector());
  EXPECT_EQ(aa.setAngle(aa2.getAngle()).getAngle(), aa2.getAngle());
  EIGEN_EXPECT_EQ(aa.setAxis(aa2.getAxis()).getAxis(), aa2.getAxis());
  EXPECT_FALSE(aa.isUnsafe());

  auto aa_copy = aa.copy();
  EXPECT_THROW(aa.setAxis(Eigen::Matrix<double, 3, 1>::Zero()),
               m3d::InvalidTransformException);
  EIGEN_EXPECT_EQ(aa.toEigenVector(), aa_copy.toEigenVector());

  aa.setAxis(Eigen::Matrix<double, 3, 1>::Zero(), true);
  EXPECT_TRUE(aa.isUnsafe());

  // DualQuaternionTransform
  m3d::DualQuaternionTransform dq;
  m3d::DualQuaternionTransform dq1(kDualQuaternion1Vec);

  EIGEN_EXPECT_EQ(dq.setDualQuaternion(dq1.getDualQuaternion()).toEigenVector(), dq1.toEigenVector());
  EIGEN_EXPECT_EQ(dq.setReal(-dq1.getReal()).getReal().toEigenVector(), -dq1.getReal().toEigenVector());
  EIGEN_EXPECT_EQ(dq.setDual(-dq1.getDual()).getDual().toEigenVector(), -dq1.getDual().toEigenVector());
  EXPECT_FALSE(dq.isUnsafe());

  auto dq_copy = dq.copy();
  EXPECT_THROW(dq.setDualQuaternion(m3d::DualQuaterniond::Zero()),
               m3d::InvalidTransformException);
  EIGEN_EXPECT_EQ(dq.toEigenVector(), dq_copy.toEigenVector());
  EXPECT_THROW(dq.setReal(m3d::Quaterniond::Zero()),
               m3d::InvalidTransformException);
  EIGEN_EXPECT_EQ(dq.toEigenVector(), dq_copy.toEigenVector());

  dq.setDualQuaternion(m3d::DualQuaterniond::Zero(), true);
  EXPECT_TRUE(dq.isUnsafe());
  dq = m3d::DualQuaternionTransform();
  dq.setReal(m3d::Quaterniond::Zero(), true);
  EXPECT_TRUE(dq.isUnsafe());

  // EulerTransform
  m3d::EulerTransform euler;
  m3d::EulerTransform euler1(kEuler1Vec);
  m3d::EulerTransform euler2(kEuler2Vec);

  EIGEN_EXPECT_EQ(euler.setTranslation(euler1.getTranslation()).getTranslation(), euler1.getTranslation());
  EXPECT_EQ(euler.setAi(euler1.getAi()).getAi(), euler1.getAi());
  EXPECT_EQ(euler.setAj(euler1.getAj()).getAj(), euler1.getAj());
  EXPECT_EQ(euler.setAk(euler1.getAk()).getAk(), euler1.getAk());
  EIGEN_EXPECT_EQ(euler.setAngles(euler2.getAngles()).getAngles(), euler2.getAngles());
  EXPECT_EQ(euler.setAxes(m3d::EulerAxes::kSXYX).getAxes(), m3d::EulerAxes::kSXYX);
  EXPECT_FALSE(euler.isUnsafe());

  // MatrixTransform
  m3d::MatrixTransform mat;
  m3d::MatrixTransform mat1(kMatrix1Vec);
  m3d::MatrixTransform mat2(kMatrix2Vec);

  EIGEN_EXPECT_EQ(mat.setRotationMatrix(mat1.getRotationMatrix()).getRotationMatrix(), mat1.getRotationMatrix());
  EIGEN_EXPECT_EQ(mat.setTranslation(mat1.getTranslation()).getTranslation(), mat1.getTranslation());
  EIGEN_EXPECT_EQ(mat.setMatrix(mat2.getMatrix()).getMatrix(), mat2.getMatrix());
  EXPECT_FALSE(mat.isUnsafe());

  auto mat_copy = mat.copy();
  EXPECT_THROW(mat.setRotationMatrix(Eigen::Matrix<double, 3, 3>::Zero()),
               m3d::InvalidTransformException);
  EIGEN_EXPECT_EQ(mat.toEigenVector(), mat_copy.toEigenVector());
  EXPECT_THROW(mat.setMatrix(Eigen::Matrix<double, 4, 4>::Zero()),
               m3d::InvalidTransformException);
  EIGEN_EXPECT_EQ(mat.toEigenVector(), mat_copy.toEigenVector());
  EXPECT_THROW(mat.setMatrix(Eigen::Matrix<double, 3, 4>::Zero()),
               m3d::InvalidTransformException);
  EIGEN_EXPECT_EQ(mat.toEigenVector(), mat_copy.toEigenVector());

  mat.setRotationMatrix(Eigen::Matrix<double, 3, 3>::Zero(), true);
  EXPECT_TRUE(mat.isUnsafe());
  mat = m3d::MatrixTransform();
  mat.setMatrix(Eigen::Matrix<double, 4, 4>::Zero(), true);
  EXPECT_TRUE(mat.isUnsafe());
  mat = m3d::MatrixTransform();
  mat.setMatrix(Eigen::Matrix<double, 3, 4>::Zero(), true);
  EXPECT_TRUE(mat.isUnsafe());

  // QuaternionTransform
  m3d::QuaternionTransform quat;
  m3d::QuaternionTransform quat1(kQuaternion1Vec);

  EIGEN_EXPECT_EQ(quat.setTranslation(quat1.getTranslation()).getTranslation(), quat1.getTranslation());
  EIGEN_EXPECT_EQ(quat.setQuaternion(quat1.getQuaternion()).getQuaternion().toEigenVector(), quat1.getQuaternion().toEigenVector());
  EXPECT_FALSE(quat.isUnsafe());

  auto quat_copy = quat.copy();
  EXPECT_THROW(quat.setQuaternion(m3d::Quaterniond::Zero()),
               m3d::InvalidTransformException);
  EIGEN_EXPECT_EQ(quat.toEigenVector(), quat_copy.toEigenVector());

  quat.setQuaternion(m3d::Quaterniond::Zero(), true);
  EXPECT_TRUE(quat.isUnsafe());
}

TEST(TransformInterface, factory)
{
  // identity transform
  m3d::TransformInterface::Ptr axis_angle1 = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle);
  m3d::TransformInterface::Ptr dual_quaternion1 = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion);
  m3d::TransformInterface::Ptr euler1 = m3d::TransformInterface::Factory(m3d::TransformType::kEuler);
  m3d::TransformInterface::Ptr matrix1 = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix);
  m3d::TransformInterface::Ptr quaternion1 = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion);

  EXPECT_TRUE(axis_angle1->isType(m3d::TransformType::kAxisAngle));
  EXPECT_FALSE(axis_angle1->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_FALSE(axis_angle1->isType(m3d::TransformType::kEuler));
  EXPECT_FALSE(axis_angle1->isType(m3d::TransformType::kMatrix));
  EXPECT_FALSE(axis_angle1->isType(m3d::TransformType::kQuaternion));

  EXPECT_FALSE(dual_quaternion1->isType(m3d::TransformType::kAxisAngle));
  EXPECT_TRUE(dual_quaternion1->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_FALSE(dual_quaternion1->isType(m3d::TransformType::kEuler));
  EXPECT_FALSE(dual_quaternion1->isType(m3d::TransformType::kMatrix));
  EXPECT_FALSE(dual_quaternion1->isType(m3d::TransformType::kQuaternion));

  EXPECT_FALSE(euler1->isType(m3d::TransformType::kAxisAngle));
  EXPECT_FALSE(euler1->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(euler1->isType(m3d::TransformType::kEuler));
  EXPECT_FALSE(euler1->isType(m3d::TransformType::kMatrix));
  EXPECT_FALSE(euler1->isType(m3d::TransformType::kQuaternion));

  EXPECT_FALSE(matrix1->isType(m3d::TransformType::kAxisAngle));
  EXPECT_FALSE(matrix1->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_FALSE(matrix1->isType(m3d::TransformType::kEuler));
  EXPECT_TRUE(matrix1->isType(m3d::TransformType::kMatrix));
  EXPECT_FALSE(matrix1->isType(m3d::TransformType::kQuaternion));

  EXPECT_FALSE(quaternion1->isType(m3d::TransformType::kAxisAngle));
  EXPECT_FALSE(quaternion1->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_FALSE(quaternion1->isType(m3d::TransformType::kEuler));
  EXPECT_FALSE(quaternion1->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(quaternion1->isType(m3d::TransformType::kQuaternion));

  // transform from vector
  std::vector<double> axis_angle_vec = eigenToVector(kAxisAngleUnitVec);
  std::vector<double> dual_quaternion_vec = eigenToVector(kDualQuaternionUnitVec);
  std::vector<double> euler_vec = eigenToVector(kEulerUnitVec);
  std::vector<double> matrix_vec = eigenToVector(kMatrixUnitVec);
  std::vector<double> quaternion_vec = eigenToVector(kQuaternionUnitVec);

  m3d::TransformInterface::Ptr axis_angle3 = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, axis_angle_vec);
  m3d::TransformInterface::Ptr dual_quaternion3 = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, dual_quaternion_vec);
  m3d::TransformInterface::Ptr euler3 = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, euler_vec);
  m3d::TransformInterface::Ptr matrix3 = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, matrix_vec);
  m3d::TransformInterface::Ptr quaternion3 = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, quaternion_vec);

  std::vector<double> invalid_vec = eigenToVector(kInvalidVec);

  EXPECT_THROW(m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, invalid_vec), 
    m3d::InvalidTypeSizeException);
  EXPECT_THROW(m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, invalid_vec), 
    m3d::InvalidTypeSizeException);
  EXPECT_THROW(m3d::TransformInterface::Factory(m3d::TransformType::kEuler, invalid_vec), 
    m3d::InvalidTypeSizeException);
  EXPECT_THROW(m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, invalid_vec), 
    m3d::InvalidTypeSizeException);
  EXPECT_THROW(m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, invalid_vec), 
    m3d::InvalidTypeSizeException);

  // transform from binary
  m3d::TransformInterface::Ptr axis_angle4 = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, axis_angle3->toBinary());
  m3d::TransformInterface::Ptr dual_quaternion4 = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, dual_quaternion3->toBinary());
  m3d::TransformInterface::Ptr euler4 = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, euler3->toBinary());
  m3d::TransformInterface::Ptr matrix4 = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, matrix3->toBinary());
  m3d::TransformInterface::Ptr quaternion4 = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, quaternion3->toBinary());

  // transform from eigen
  m3d::TransformInterface::Ptr axis_angle5 = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngleUnitVec);
  m3d::TransformInterface::Ptr dual_quaternion5 = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternionUnitVec);
  m3d::TransformInterface::Ptr euler5 = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEulerUnitVec);
  m3d::TransformInterface::Ptr matrix5 = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrixUnitVec);
  m3d::TransformInterface::Ptr quaternion5 = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternionUnitVec);

  EXPECT_THROW(m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kInvalidVec), 
    m3d::InvalidTypeSizeException);
  EXPECT_THROW(m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kInvalidVec), 
    m3d::InvalidTypeSizeException);
  EXPECT_THROW(m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kInvalidVec), 
    m3d::InvalidTypeSizeException);
  EXPECT_THROW(m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kInvalidVec), 
    m3d::InvalidTypeSizeException);
  EXPECT_THROW(m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kInvalidVec), 
    m3d::InvalidTypeSizeException);
}

TEST(TransformInterface, copyDirect)
{
  m3d::AxisAngleTransform axis_angle(kAxisAngle1Vec);
  m3d::DualQuaternionTransform dual_quaternion(kDualQuaternion1Vec);
  m3d::EulerTransform euler(kEuler1Vec);
  m3d::MatrixTransform matrix(kMatrix1Vec);
  m3d::QuaternionTransform quaternion(kQuaternion1Vec);

  m3d::AxisAngleTransform axis_angle_copy = axis_angle.copy();
  m3d::DualQuaternionTransform dual_quaternion_copy = dual_quaternion.copy();
  m3d::EulerTransform euler_copy = euler.copy();
  m3d::MatrixTransform matrix_copy = matrix.copy();
  m3d::QuaternionTransform quaternion_copy = quaternion.copy();

  EIGEN_EXPECT_EQ(axis_angle.toEigenVector(), axis_angle_copy.toEigenVector());
  EIGEN_EXPECT_EQ(dual_quaternion.toEigenVector(), dual_quaternion_copy.toEigenVector());
  EIGEN_EXPECT_EQ(euler.toEigenVector(), euler_copy.toEigenVector());
  EIGEN_EXPECT_EQ(matrix.toEigenVector(), matrix_copy.toEigenVector());
  EIGEN_EXPECT_EQ(quaternion.toEigenVector(), quaternion_copy.toEigenVector());
}

TEST(TransformInterface, copyInterface)
{
  m3d::TransformInterface::Ptr axis_angle = std::make_shared<m3d::AxisAngleTransform>(kAxisAngle1Vec);
  m3d::TransformInterface::Ptr dual_quaternion = std::make_shared<m3d::DualQuaternionTransform>(kDualQuaternion1Vec);
  m3d::TransformInterface::Ptr euler = std::make_shared<m3d::EulerTransform>(kEuler1Vec);
  m3d::TransformInterface::Ptr matrix = std::make_shared<m3d::MatrixTransform>(kMatrix1Vec);
  m3d::TransformInterface::Ptr quaternion = std::make_shared<m3d::QuaternionTransform>(kQuaternion1Vec);

  m3d::TransformInterface::Ptr axis_angle_copy = axis_angle->copy();
  m3d::TransformInterface::Ptr dual_quaternion_copy = dual_quaternion->copy();
  m3d::TransformInterface::Ptr euler_copy = euler->copy();
  m3d::TransformInterface::Ptr matrix_copy = matrix->copy();
  m3d::TransformInterface::Ptr quaternion_copy = quaternion->copy();

  EXPECT_NE(axis_angle, axis_angle_copy);
  EXPECT_NE(dual_quaternion, dual_quaternion_copy);
  EXPECT_NE(euler, euler_copy);
  EXPECT_NE(matrix, matrix_copy);
  EXPECT_NE(quaternion, quaternion_copy);

  EIGEN_EXPECT_EQ(axis_angle->toEigenVector(), axis_angle_copy->toEigenVector());
  EIGEN_EXPECT_EQ(dual_quaternion->toEigenVector(), dual_quaternion_copy->toEigenVector());
  EIGEN_EXPECT_EQ(euler->toEigenVector(), euler_copy->toEigenVector());
  EIGEN_EXPECT_EQ(matrix->toEigenVector(), matrix_copy->toEigenVector());
  EIGEN_EXPECT_EQ(quaternion->toEigenVector(), quaternion_copy->toEigenVector());
}

TEST(TransformInterface, identity)
{
  // generate data
  m3d::TransformInterface::Ptr axis_angle = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle1Vec);
  m3d::TransformInterface::Ptr dual_quaternion = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion1Vec);
  m3d::TransformInterface::Ptr euler = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler1Vec);
  m3d::TransformInterface::Ptr matrix = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix1Vec);
  m3d::TransformInterface::Ptr quaternion = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion1Vec);

  // copy
  m3d::TransformInterface::Ptr axis_angle_id = axis_angle->identity();
  m3d::TransformInterface::Ptr dual_quaternion_id = dual_quaternion->identity();
  m3d::TransformInterface::Ptr euler_id = euler->identity();
  m3d::TransformInterface::Ptr matrix_id = matrix->identity();
  m3d::TransformInterface::Ptr quaternion_id = quaternion->identity();

  EXPECT_TRUE(axis_angle_id->isType(m3d::TransformType::kAxisAngle));
  EXPECT_TRUE(dual_quaternion_id->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(euler_id->isType(m3d::TransformType::kEuler));
  EXPECT_TRUE(matrix_id->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(quaternion_id->isType(m3d::TransformType::kQuaternion));

  EIGEN_EXPECT_EQ(axis_angle_id->toEigenVector(), kAxisAngleUnitVec);
  EIGEN_EXPECT_EQ(dual_quaternion_id->toEigenVector(), kDualQuaternionUnitVec);
  EIGEN_EXPECT_EQ(euler_id->toEigenVector(), kEulerUnitVec);
  EIGEN_EXPECT_EQ(matrix_id->toEigenVector(), kMatrixUnitVec);
  EIGEN_EXPECT_EQ(quaternion_id->toEigenVector(), kQuaternionUnitVec);

  // inplace
  axis_angle->setIdentity();
  dual_quaternion->setIdentity();
  euler->setIdentity();
  matrix->setIdentity();
  quaternion->setIdentity();

  EXPECT_TRUE(axis_angle->isType(m3d::TransformType::kAxisAngle));
  EXPECT_TRUE(dual_quaternion->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(euler->isType(m3d::TransformType::kEuler));
  EXPECT_TRUE(matrix->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(quaternion->isType(m3d::TransformType::kQuaternion));

  EIGEN_EXPECT_EQ(axis_angle->toEigenVector(), kAxisAngleUnitVec);
  EIGEN_EXPECT_EQ(dual_quaternion->toEigenVector(), kDualQuaternionUnitVec);
  EIGEN_EXPECT_EQ(euler->toEigenVector(), kEulerUnitVec);
  EIGEN_EXPECT_EQ(matrix->toEigenVector(), kMatrixUnitVec);
  EIGEN_EXPECT_EQ(quaternion->toEigenVector(), kQuaternionUnitVec);
}

TEST(TransformInterface, typeConversion)
{
  // interfaces
  m3d::TransformInterface::Ptr axis_angle = std::make_shared<m3d::AxisAngleTransform>();
  m3d::TransformInterface::Ptr dual_quaternion = std::make_shared<m3d::DualQuaternionTransform>();
  m3d::TransformInterface::Ptr euler = std::make_shared<m3d::EulerTransform>();
  m3d::TransformInterface::Ptr matrix = std::make_shared<m3d::MatrixTransform>();
  m3d::TransformInterface::Ptr quaternion = std::make_shared<m3d::QuaternionTransform>();
  std::vector<m3d::TransformInterface::Ptr> interface_vec = {
    axis_angle, dual_quaternion, euler, matrix, quaternion
  };

  for (const m3d::TransformInterface::Ptr &interface : interface_vec)
  {
    // as type
    m3d::TransformInterface::Ptr axis_angle_tmp = interface->asType<m3d::AxisAngleTransform>();
    m3d::TransformInterface::Ptr dual_quaternion_tmp = interface->asType<m3d::DualQuaternionTransform>();
    m3d::TransformInterface::Ptr euler_tmp = interface->asType<m3d::EulerTransform>();
    m3d::TransformInterface::Ptr matrix_tmp = interface->asType<m3d::MatrixTransform>();
    m3d::TransformInterface::Ptr quat_tmp = interface->asType<m3d::QuaternionTransform>();

    EXPECT_TRUE(axis_angle_tmp->isType(m3d::TransformType::kAxisAngle));
    EXPECT_TRUE(dual_quaternion_tmp->isType(m3d::TransformType::kDualQuaternion));
    EXPECT_TRUE(euler_tmp->isType(m3d::TransformType::kEuler));
    EXPECT_TRUE(matrix_tmp->isType(m3d::TransformType::kMatrix));
    EXPECT_TRUE(quat_tmp->isType(m3d::TransformType::kQuaternion));

    // typecast to direct pointer
    std::shared_ptr<m3d::AxisAngleTransform> as_axis_angle_cast = interface->asType<m3d::AxisAngleTransform>();
    std::shared_ptr<m3d::DualQuaternionTransform> as_dual_quaternion_cast = interface->asType<m3d::DualQuaternionTransform>();
    std::shared_ptr<m3d::EulerTransform> as_euler_cast = interface->asType<m3d::EulerTransform>();
    std::shared_ptr<m3d::MatrixTransform> as_matrix_cast = interface->asType<m3d::MatrixTransform>();
    std::shared_ptr<m3d::QuaternionTransform> as_quat_cast = interface->asType<m3d::QuaternionTransform>();

    EXPECT_NE(as_axis_angle_cast, nullptr);
    EXPECT_NE(as_dual_quaternion_cast, nullptr);
    EXPECT_NE(as_euler_cast, nullptr);
    EXPECT_NE(as_matrix_cast, nullptr);
    EXPECT_NE(as_quat_cast, nullptr);
  }
}

TEST(TransformInterface, pointerCast)
{
  // interface
  m3d::TransformInterface::Ptr axis_angle = std::make_shared<m3d::AxisAngleTransform>();
  m3d::TransformInterface::Ptr dual_quaternion = std::make_shared<m3d::DualQuaternionTransform>();
  m3d::TransformInterface::Ptr euler = std::make_shared<m3d::EulerTransform>();
  m3d::TransformInterface::Ptr matrix = std::make_shared<m3d::MatrixTransform>();
  m3d::TransformInterface::Ptr quaternion = std::make_shared<m3d::QuaternionTransform>();

  // pointer cast success
  std::shared_ptr<m3d::AxisAngleTransform> axis_angle_success = std::dynamic_pointer_cast<m3d::AxisAngleTransform>(axis_angle);
  std::shared_ptr<m3d::DualQuaternionTransform> dual_quaternion_success = std::dynamic_pointer_cast<m3d::DualQuaternionTransform>(dual_quaternion);
  std::shared_ptr<m3d::EulerTransform> euler_success = std::dynamic_pointer_cast<m3d::EulerTransform>(euler);
  std::shared_ptr<m3d::MatrixTransform> matrix_success = std::dynamic_pointer_cast<m3d::MatrixTransform>(matrix);
  std::shared_ptr<m3d::QuaternionTransform> quaternion_success = std::dynamic_pointer_cast<m3d::QuaternionTransform>(quaternion);

  EXPECT_TRUE(axis_angle_success != nullptr);
  EXPECT_TRUE(dual_quaternion_success != nullptr);
  EXPECT_TRUE(euler_success != nullptr);
  EXPECT_TRUE(matrix_success != nullptr);
  EXPECT_TRUE(quaternion_success != nullptr);

  // pointer cast failure
  std::shared_ptr<m3d::AxisAngleTransform> axis_angle_fail1 = std::dynamic_pointer_cast<m3d::AxisAngleTransform>(dual_quaternion);
  std::shared_ptr<m3d::AxisAngleTransform> axis_angle_fail2 = std::dynamic_pointer_cast<m3d::AxisAngleTransform>(euler);
  std::shared_ptr<m3d::AxisAngleTransform> axis_angle_fail3 = std::dynamic_pointer_cast<m3d::AxisAngleTransform>(matrix);
  std::shared_ptr<m3d::AxisAngleTransform> axis_angle_fail4 = std::dynamic_pointer_cast<m3d::AxisAngleTransform>(quaternion);

  std::shared_ptr<m3d::DualQuaternionTransform> dual_quaternion_fail1 = std::dynamic_pointer_cast<m3d::DualQuaternionTransform>(axis_angle);
  std::shared_ptr<m3d::DualQuaternionTransform> dual_quaternion_fail2 = std::dynamic_pointer_cast<m3d::DualQuaternionTransform>(euler);
  std::shared_ptr<m3d::DualQuaternionTransform> dual_quaternion_fail3 = std::dynamic_pointer_cast<m3d::DualQuaternionTransform>(matrix);
  std::shared_ptr<m3d::DualQuaternionTransform> dual_quaternion_fail4 = std::dynamic_pointer_cast<m3d::DualQuaternionTransform>(quaternion);

  std::shared_ptr<m3d::EulerTransform> euler_fail1 = std::dynamic_pointer_cast<m3d::EulerTransform>(axis_angle);
  std::shared_ptr<m3d::EulerTransform> euler_fail2 = std::dynamic_pointer_cast<m3d::EulerTransform>(dual_quaternion);
  std::shared_ptr<m3d::EulerTransform> euler_fail3 = std::dynamic_pointer_cast<m3d::EulerTransform>(matrix);
  std::shared_ptr<m3d::EulerTransform> euler_fail4 = std::dynamic_pointer_cast<m3d::EulerTransform>(quaternion);

  std::shared_ptr<m3d::MatrixTransform> matrix_fail1 = std::dynamic_pointer_cast<m3d::MatrixTransform>(axis_angle);
  std::shared_ptr<m3d::MatrixTransform> matrix_fail2 = std::dynamic_pointer_cast<m3d::MatrixTransform>(dual_quaternion);
  std::shared_ptr<m3d::MatrixTransform> matrix_fail3 = std::dynamic_pointer_cast<m3d::MatrixTransform>(euler);
  std::shared_ptr<m3d::MatrixTransform> matrix_fail4 = std::dynamic_pointer_cast<m3d::MatrixTransform>(quaternion);

  std::shared_ptr<m3d::QuaternionTransform> quaternion_fail1 = std::dynamic_pointer_cast<m3d::QuaternionTransform>(axis_angle);
  std::shared_ptr<m3d::QuaternionTransform> quaternion_fail2 = std::dynamic_pointer_cast<m3d::QuaternionTransform>(dual_quaternion);
  std::shared_ptr<m3d::QuaternionTransform> quaternion_fail3 = std::dynamic_pointer_cast<m3d::QuaternionTransform>(euler);
  std::shared_ptr<m3d::QuaternionTransform> quaternion_fail4 = std::dynamic_pointer_cast<m3d::QuaternionTransform>(matrix);

  EXPECT_FALSE(axis_angle_fail1 != nullptr);
  EXPECT_FALSE(axis_angle_fail2 != nullptr);
  EXPECT_FALSE(axis_angle_fail3 != nullptr);
  EXPECT_FALSE(axis_angle_fail4 != nullptr);

  EXPECT_FALSE(dual_quaternion_fail1 != nullptr);
  EXPECT_FALSE(dual_quaternion_fail2 != nullptr);
  EXPECT_FALSE(dual_quaternion_fail3 != nullptr);
  EXPECT_FALSE(dual_quaternion_fail4 != nullptr);

  EXPECT_FALSE(euler_fail1 != nullptr);
  EXPECT_FALSE(euler_fail2 != nullptr);
  EXPECT_FALSE(euler_fail3 != nullptr);
  EXPECT_FALSE(euler_fail4 != nullptr);

  EXPECT_FALSE(matrix_fail1 != nullptr);
  EXPECT_FALSE(matrix_fail2 != nullptr);
  EXPECT_FALSE(matrix_fail3 != nullptr);
  EXPECT_FALSE(matrix_fail4 != nullptr);

  EXPECT_FALSE(quaternion_fail1 != nullptr);
  EXPECT_FALSE(quaternion_fail2 != nullptr);
  EXPECT_FALSE(quaternion_fail3 != nullptr);
  EXPECT_FALSE(quaternion_fail4 != nullptr);

  // pointer cast return value
  EXPECT_TRUE(std::dynamic_pointer_cast<m3d::AxisAngleTransform>(axis_angle));
  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::DualQuaternionTransform>(axis_angle));
  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::EulerTransform>(axis_angle));
  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::MatrixTransform>(axis_angle));
  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::QuaternionTransform>(axis_angle));

  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::AxisAngleTransform>(dual_quaternion));
  EXPECT_TRUE(std::dynamic_pointer_cast<m3d::DualQuaternionTransform>(dual_quaternion));
  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::EulerTransform>(dual_quaternion));
  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::MatrixTransform>(dual_quaternion));
  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::QuaternionTransform>(dual_quaternion));

  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::AxisAngleTransform>(euler));
  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::DualQuaternionTransform>(euler));
  EXPECT_TRUE(std::dynamic_pointer_cast<m3d::EulerTransform>(euler));
  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::MatrixTransform>(euler));
  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::QuaternionTransform>(euler));

  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::AxisAngleTransform>(matrix));
  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::DualQuaternionTransform>(matrix));
  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::EulerTransform>(matrix));
  EXPECT_TRUE(std::dynamic_pointer_cast<m3d::MatrixTransform>(matrix));
  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::QuaternionTransform>(matrix));

  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::AxisAngleTransform>(quaternion));
  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::DualQuaternionTransform>(quaternion));
  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::EulerTransform>(quaternion));
  EXPECT_FALSE(std::dynamic_pointer_cast<m3d::MatrixTransform>(quaternion));
  EXPECT_TRUE(std::dynamic_pointer_cast<m3d::QuaternionTransform>(quaternion));
}

TEST(TransformInterface, getVector)
{
  // direct
  m3d::AxisAngleTransform::Ptr axis_angle_if = std::make_shared<m3d::AxisAngleTransform>();
  m3d::DualQuaternionTransform::Ptr dual_quaternion_if = std::make_shared<m3d::DualQuaternionTransform>();
  m3d::EulerTransform::Ptr euler_if = std::make_shared<m3d::EulerTransform>();
  m3d::MatrixTransform::Ptr matrix_if = std::make_shared<m3d::MatrixTransform>();
  m3d::QuaternionTransform::Ptr quaternion_if = std::make_shared<m3d::QuaternionTransform>();

  // binary
  EXPECT_EQ((*axis_angle_if).toBinary().size(), m3d::AxisAngleTransform::kBinarySize);
  EXPECT_EQ((*dual_quaternion_if).toBinary().size(), m3d::DualQuaternionTransform::kBinarySize);
  EXPECT_EQ((*euler_if).toBinary().size(), m3d::EulerTransform::kBinarySize);
  EXPECT_EQ((*matrix_if).toBinary().size(), m3d::MatrixTransform::kBinarySize);
  EXPECT_EQ((*quaternion_if).toBinary().size(), m3d::QuaternionTransform::kBinarySize);

  EXPECT_EQ((*axis_angle_if).toBinary().size(), m3d::getBinarySize(m3d::TransformType::kAxisAngle));
  EXPECT_EQ((*dual_quaternion_if).toBinary().size(), m3d::getBinarySize(m3d::TransformType::kDualQuaternion));
  EXPECT_EQ((*euler_if).toBinary().size(), m3d::getBinarySize(m3d::TransformType::kEuler));
  EXPECT_EQ((*matrix_if).toBinary().size(), m3d::getBinarySize(m3d::TransformType::kMatrix));
  EXPECT_EQ((*quaternion_if).toBinary().size(), m3d::getBinarySize(m3d::TransformType::kQuaternion));

  // vector
  EXPECT_EQ((*axis_angle_if).toVector().size(), m3d::AxisAngleTransform::kVectorSize);
  EXPECT_EQ((*dual_quaternion_if).toVector().size(), m3d::DualQuaternionTransform::kVectorSize);
  EXPECT_EQ((*euler_if).toVector().size(), m3d::EulerTransform::kVectorSize);
  EXPECT_EQ((*matrix_if).toVector().size(), m3d::MatrixTransform::kVectorSize);
  EXPECT_EQ((*quaternion_if).toVector().size(), m3d::QuaternionTransform::kVectorSize);

  EXPECT_EQ((*axis_angle_if).toVector().size(), m3d::getVectorSize(m3d::TransformType::kAxisAngle));
  EXPECT_EQ((*dual_quaternion_if).toVector().size(), m3d::getVectorSize(m3d::TransformType::kDualQuaternion));
  EXPECT_EQ((*euler_if).toVector().size(), m3d::getVectorSize(m3d::TransformType::kEuler));
  EXPECT_EQ((*matrix_if).toVector().size(), m3d::getVectorSize(m3d::TransformType::kMatrix));
  EXPECT_EQ((*quaternion_if).toVector().size(), m3d::getVectorSize(m3d::TransformType::kQuaternion));

  // interfaces
  std::vector<m3d::TransformInterface::Ptr> interface_vec = {
    axis_angle_if, dual_quaternion_if, euler_if, matrix_if, quaternion_if
  };

  for (const m3d::TransformInterface::Ptr &interface : interface_vec)
  {
    m3d::TransformInterface::Ptr axis_angle = interface->asType<m3d::AxisAngleTransform>();
    m3d::TransformInterface::Ptr dual_quaternion = interface->asType<m3d::DualQuaternionTransform>();
    m3d::TransformInterface::Ptr euler = interface->asType<m3d::EulerTransform>();
    m3d::TransformInterface::Ptr matrix = interface->asType<m3d::MatrixTransform>();
    m3d::TransformInterface::Ptr quaternion = interface->asType<m3d::QuaternionTransform>();

    // binary
    EXPECT_EQ(axis_angle->toBinary().size(), m3d::AxisAngleTransform::kBinarySize);
    EXPECT_EQ(dual_quaternion->toBinary().size(), m3d::DualQuaternionTransform::kBinarySize);
    EXPECT_EQ(euler->toBinary().size(), m3d::EulerTransform::kBinarySize);
    EXPECT_EQ(matrix->toBinary().size(), m3d::MatrixTransform::kBinarySize);
    EXPECT_EQ(quaternion->toBinary().size(), m3d::QuaternionTransform::kBinarySize);

    // vector
    EXPECT_EQ(axis_angle->toVector().size(), m3d::AxisAngleTransform::kVectorSize);
    EXPECT_EQ(dual_quaternion->toVector().size(), m3d::DualQuaternionTransform::kVectorSize);
    EXPECT_EQ(euler->toVector().size(), m3d::EulerTransform::kVectorSize);
    EXPECT_EQ(matrix->toVector().size(), m3d::MatrixTransform::kVectorSize);
    EXPECT_EQ(quaternion->toVector().size(), m3d::QuaternionTransform::kVectorSize);
  }
}

TEST(TransformInterface, isValid)
{
  // direct
  m3d::AxisAngleTransform axis_angle(kAxisAngleUnitVec);
  m3d::DualQuaternionTransform dual_quaternion(kDualQuaternionUnitVec);
  m3d::EulerTransform euler(kEulerUnitVec);
  m3d::MatrixTransform matrix(kMatrixUnitVec);
  m3d::QuaternionTransform quaternion(kQuaternionUnitVec);

  EXPECT_TRUE(axis_angle.isValid());
  EXPECT_TRUE(dual_quaternion.isValid());
  EXPECT_TRUE(euler.isValid());
  EXPECT_TRUE(matrix.isValid());
  EXPECT_TRUE(quaternion.isValid());

  // failure
  EXPECT_THROW(m3d::AxisAngleTransform angle_axis_fail(kAxisAngleZeroVec), 
    m3d::InvalidTransformException);
  EXPECT_THROW(m3d::DualQuaternionTransform dual_quaternion_fail(kDualQuaternionZeroVec), 
    m3d::InvalidTransformException);
  EXPECT_THROW(m3d::MatrixTransform matrix_fail(kMatrixZeroVec), 
    m3d::InvalidTransformException);
  EXPECT_THROW(m3d::QuaternionTransform quaternion_fail(kQuaternionZeroVec), 
    m3d::InvalidTransformException);

  // unsafe
  m3d::AxisAngleTransform axis_angle_invalid(kAxisAngleZeroVec, true);
  m3d::DualQuaternionTransform dual_quaternion_invalid(kDualQuaternionZeroVec, true);
  m3d::MatrixTransform matrix_invalid(kMatrixZeroVec, true);
  m3d::QuaternionTransform quaternion_invalid(kQuaternionZeroVec, true);

  EXPECT_FALSE(axis_angle_invalid.isValid());
  EXPECT_FALSE(dual_quaternion_invalid.isValid());
  // Euler is always valid
  EXPECT_FALSE(matrix_invalid.isValid());
  EXPECT_FALSE(quaternion_invalid.isValid());

  // epsilon
  EXPECT_TRUE(axis_angle_invalid.isValid(std::numeric_limits<double>::infinity()));
  EXPECT_TRUE(dual_quaternion_invalid.isValid(std::numeric_limits<double>::infinity()));
  EXPECT_TRUE(matrix_invalid.isValid(std::numeric_limits<double>::infinity()));
  EXPECT_TRUE(quaternion_invalid.isValid(std::numeric_limits<double>::infinity()));

  // interface
  m3d::TransformInterface::Ptr axis_angle_if = std::make_shared<m3d::AxisAngleTransform>(kAxisAngleUnitVec);
  m3d::TransformInterface::Ptr dual_quaternion_if = std::make_shared<m3d::DualQuaternionTransform>(kDualQuaternionUnitVec);
  m3d::TransformInterface::Ptr euler_if = std::make_shared<m3d::EulerTransform>(kEulerUnitVec);
  m3d::TransformInterface::Ptr matrix_if = std::make_shared<m3d::MatrixTransform>(kMatrixUnitVec);
  m3d::TransformInterface::Ptr quaternion_if = std::make_shared<m3d::QuaternionTransform>(kQuaternionUnitVec);

  EXPECT_TRUE(axis_angle_if->isValid());
  EXPECT_TRUE(dual_quaternion_if->isValid());
  EXPECT_TRUE(euler_if->isValid());
  EXPECT_TRUE(matrix_if->isValid());
  EXPECT_TRUE(quaternion_if->isValid());

  // failure
  EXPECT_THROW(m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngleZeroVec),
    m3d::InvalidTransformException);
  EXPECT_THROW(m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternionZeroVec),
    m3d::InvalidTransformException);
  EXPECT_THROW(m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrixZeroVec),
    m3d::InvalidTransformException);
  EXPECT_THROW(m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternionZeroVec),
    m3d::InvalidTransformException);

  // unsafe
  m3d::TransformInterface::Ptr axis_angle_invalid_if = 
    std::make_shared<m3d::AxisAngleTransform>(kAxisAngleZeroVec, true);
  m3d::TransformInterface::Ptr dual_quaternion_invalid_if = 
    std::make_shared<m3d::DualQuaternionTransform>(kDualQuaternionZeroVec, true);
  m3d::TransformInterface::Ptr matrix_invalid_if = 
    std::make_shared<m3d::MatrixTransform>(kMatrixZeroVec, true);
  m3d::TransformInterface::Ptr quaternion_invalid_if = 
    std::make_shared<m3d::QuaternionTransform>(kQuaternionZeroVec, true);

  EXPECT_FALSE(axis_angle_invalid_if->isValid());
  EXPECT_FALSE(dual_quaternion_invalid_if->isValid());
  // Euler is always valid
  EXPECT_FALSE(matrix_invalid_if->isValid());
  EXPECT_FALSE(quaternion_invalid_if->isValid());

  // epsilon
  EXPECT_TRUE(axis_angle_invalid_if->isValid(std::numeric_limits<double>::infinity()));
  EXPECT_TRUE(dual_quaternion_invalid_if->isValid(std::numeric_limits<double>::infinity()));
  EXPECT_TRUE(matrix_invalid_if->isValid(std::numeric_limits<double>::infinity()));
  EXPECT_TRUE(quaternion_invalid_if->isValid(std::numeric_limits<double>::infinity()));
}

TEST(TransformInterface, inplaceReturn)
{
  // create transforms
  m3d::AxisAngleTransform axis_angle;
  m3d::DualQuaternionTransform dual_quaternion;
  m3d::EulerTransform euler;
  m3d::MatrixTransform matrix;
  m3d::QuaternionTransform quaternion;
  m3d::TransformInterface::Ptr interface = std::make_shared<m3d::AxisAngleTransform>();

  // setter
  ASSERT_TRUE(&axis_angle == &axis_angle.setTranslation(axis_angle.getTranslation()));
  ASSERT_TRUE(&axis_angle == &axis_angle.setAngleAxis(axis_angle.getAngleAxis()));
  ASSERT_TRUE(&axis_angle == &axis_angle.setAxis(axis_angle.getAxis()));
  ASSERT_TRUE(&axis_angle == &axis_angle.setAngle(axis_angle.getAngle()));

  ASSERT_TRUE(&dual_quaternion == &dual_quaternion.setDualQuaternion(dual_quaternion.getDualQuaternion()));
  ASSERT_TRUE(&dual_quaternion == &dual_quaternion.setReal(dual_quaternion.getReal()));
  ASSERT_TRUE(&dual_quaternion == &dual_quaternion.setDual(dual_quaternion.getDual()));

  ASSERT_TRUE(&euler == &euler.setTranslation(euler.getTranslation()));
  ASSERT_TRUE(&euler == &euler.setAi(euler.getAi()));
  ASSERT_TRUE(&euler == &euler.setAj(euler.getAj()));
  ASSERT_TRUE(&euler == &euler.setAk(euler.getAk()));
  ASSERT_TRUE(&euler == &euler.setAngles(euler.getAngles()));
  ASSERT_TRUE(&euler == &euler.setAxes(euler.getAxes()));

  ASSERT_TRUE(&matrix == &matrix.setTranslation(matrix.getTranslation()));
  ASSERT_TRUE(&matrix == &matrix.setRotationMatrix(matrix.getRotationMatrix()));
  ASSERT_TRUE(&matrix == &matrix.setMatrix(matrix.getMatrix()));
  ASSERT_TRUE(&matrix == &matrix.setMatrix(matrix.getMatrix().block<3, 4>(0, 0)));

  ASSERT_TRUE(&quaternion == &quaternion.setTranslation(quaternion.getTranslation()));
  ASSERT_TRUE(&quaternion == &quaternion.setQuaternion(quaternion.getQuaternion()));

  // identity
  ASSERT_TRUE(&axis_angle == &axis_angle.setIdentity());
  ASSERT_TRUE(&dual_quaternion == &dual_quaternion.setIdentity());
  ASSERT_TRUE(&euler == &euler.setIdentity());
  ASSERT_TRUE(&matrix == &matrix.setIdentity());
  ASSERT_TRUE(&quaternion == &quaternion.setIdentity());
  ASSERT_FALSE(interface == interface->identity());
  ASSERT_TRUE(interface == interface->setIdentity());

  // inverse
  ASSERT_TRUE(&axis_angle == &axis_angle.inverse_());
  ASSERT_TRUE(&dual_quaternion == &dual_quaternion.inverse_());
  ASSERT_TRUE(&euler == &euler.inverse_());
  ASSERT_TRUE(&matrix == &matrix.inverse_());
  ASSERT_TRUE(&quaternion == &quaternion.inverse_());
  ASSERT_FALSE(interface == interface->inverse());
  ASSERT_TRUE(interface == interface->inverse_());

  // normalized
  ASSERT_TRUE(&axis_angle == &axis_angle.normalized_());
  ASSERT_TRUE(&dual_quaternion == &dual_quaternion.normalized_());
  ASSERT_TRUE(&euler == &euler.normalized_());
  ASSERT_TRUE(&matrix == &matrix.normalized_());
  ASSERT_TRUE(&quaternion == &quaternion.normalized_());
  ASSERT_FALSE(interface == interface->normalized());
  ASSERT_TRUE(interface == interface->normalized_());

  // scale translation
  ASSERT_TRUE(&axis_angle == &axis_angle.scaleTranslation_(1.0));
  ASSERT_TRUE(&dual_quaternion == &dual_quaternion.scaleTranslation_(1.0));
  ASSERT_TRUE(&euler == &euler.scaleTranslation_(1.0));
  ASSERT_TRUE(&matrix == &matrix.scaleTranslation_(1.0));
  ASSERT_TRUE(&quaternion == &quaternion.scaleTranslation_(1.0));
  ASSERT_FALSE(interface == interface->scaleTranslation(1.0));
  ASSERT_TRUE(interface == interface->scaleTranslation_(1.0));

  // apply
  ASSERT_TRUE(&axis_angle == &axis_angle.applyPre_(axis_angle));
  ASSERT_TRUE(&axis_angle == &axis_angle.applyPre_(interface));
  ASSERT_TRUE(&dual_quaternion == &dual_quaternion.applyPre_(axis_angle));
  ASSERT_TRUE(&dual_quaternion == &dual_quaternion.applyPre_(interface));
  ASSERT_TRUE(&euler == &euler.applyPre_(interface));
  ASSERT_TRUE(&euler == &euler.applyPre_(interface));
  ASSERT_TRUE(&matrix == &matrix.applyPre_(axis_angle));
  ASSERT_TRUE(&matrix == &matrix.applyPre_(interface));
  ASSERT_TRUE(&quaternion == &quaternion.applyPre_(axis_angle));
  ASSERT_TRUE(&quaternion == &quaternion.applyPre_(interface));
  ASSERT_FALSE(interface == interface->applyPre(axis_angle));
  ASSERT_TRUE(interface == interface->applyPre_(axis_angle));
  ASSERT_TRUE(interface == interface->applyPre_(interface));

  ASSERT_TRUE(&axis_angle == &axis_angle.applyPost_(axis_angle));
  ASSERT_TRUE(&axis_angle == &axis_angle.applyPost_(interface));
  ASSERT_TRUE(&dual_quaternion == &dual_quaternion.applyPost_(axis_angle));
  ASSERT_TRUE(&dual_quaternion == &dual_quaternion.applyPost_(interface));
  ASSERT_TRUE(&euler == &euler.applyPost_(interface));
  ASSERT_TRUE(&euler == &euler.applyPost_(interface));
  ASSERT_TRUE(&matrix == &matrix.applyPost_(axis_angle));
  ASSERT_TRUE(&matrix == &matrix.applyPost_(interface));
  ASSERT_TRUE(&quaternion == &quaternion.applyPost_(axis_angle));
  ASSERT_TRUE(&quaternion == &quaternion.applyPost_(interface));
  ASSERT_FALSE(interface == interface->applyPost(axis_angle));
  ASSERT_TRUE(interface == interface->applyPost_(axis_angle));
  ASSERT_TRUE(interface == interface->applyPost_(interface));

  // transform specific
  ASSERT_TRUE(&euler == &euler.changeAxes_(m3d::EulerAxes::kSZYX));
}

TEST(TransformInterface, inverse)
{
  // direct
  m3d::AxisAngleTransform axis_angle;
  m3d::DualQuaternionTransform dual_quaternion;
  m3d::EulerTransform euler;
  m3d::MatrixTransform matrix;
  m3d::QuaternionTransform quaternion;

  axis_angle.inverse();
  dual_quaternion.inverse();
  euler.inverse();
  matrix.inverse();
  quaternion.inverse();

  // interface
  m3d::TransformInterface::Ptr axis_angle_if = std::make_shared<m3d::AxisAngleTransform>();
  m3d::TransformInterface::Ptr dual_quaternion_if = std::make_shared<m3d::DualQuaternionTransform>();
  m3d::TransformInterface::Ptr euler_if = std::make_shared<m3d::EulerTransform>();
  m3d::TransformInterface::Ptr matrix_if = std::make_shared<m3d::MatrixTransform>();
  m3d::TransformInterface::Ptr quaternion_if = std::make_shared<m3d::QuaternionTransform>();

  std::vector<m3d::TransformInterface::Ptr> interface_vec = {
    axis_angle_if, dual_quaternion_if, euler_if, matrix_if, quaternion_if
  };

  for (const m3d::TransformInterface::Ptr &interface : interface_vec)
  {
    interface->inverse();
  }
}

TEST(TransformInterface, normalized)
{
  // direct
  m3d::AxisAngleTransform axis_angle;
  m3d::DualQuaternionTransform dual_quaternion;
  m3d::EulerTransform euler;
  m3d::MatrixTransform matrix;
  m3d::QuaternionTransform quaternion;

  axis_angle.normalized();
  dual_quaternion.normalized();
  euler.normalized();
  matrix.normalized();
  quaternion.normalized();

  // interface
  m3d::TransformInterface::Ptr axis_angle_if = std::make_shared<m3d::AxisAngleTransform>();
  m3d::TransformInterface::Ptr dual_quaternion_if = std::make_shared<m3d::DualQuaternionTransform>();
  m3d::TransformInterface::Ptr euler_if = std::make_shared<m3d::EulerTransform>();
  m3d::TransformInterface::Ptr matrix_if = std::make_shared<m3d::MatrixTransform>();
  m3d::TransformInterface::Ptr quaternion_if = std::make_shared<m3d::QuaternionTransform>();

  std::vector<m3d::TransformInterface::Ptr> interface_vec = {
    axis_angle_if, dual_quaternion_if, euler_if, matrix_if, quaternion_if
  };

  for (const m3d::TransformInterface::Ptr &interface : interface_vec)
  {
    interface->normalized();
  }
}

TEST(TransformInterface, scaleTranslation)
{
  // direct
  m3d::AxisAngleTransform axis_angle;
  m3d::DualQuaternionTransform dual_quaternion;
  m3d::EulerTransform euler;
  m3d::MatrixTransform matrix;
  m3d::QuaternionTransform quaternion;

  axis_angle.scaleTranslation(2.0);
  dual_quaternion.scaleTranslation(2.0);
  euler.scaleTranslation(2.0);
  matrix.scaleTranslation(2.0);
  quaternion.scaleTranslation(2.0);

  // interface
  m3d::TransformInterface::Ptr axis_angle_if = std::make_shared<m3d::AxisAngleTransform>();
  m3d::TransformInterface::Ptr dual_quaternion_if = std::make_shared<m3d::DualQuaternionTransform>();
  m3d::TransformInterface::Ptr euler_if = std::make_shared<m3d::EulerTransform>();
  m3d::TransformInterface::Ptr matrix_if = std::make_shared<m3d::MatrixTransform>();
  m3d::TransformInterface::Ptr quaternion_if = std::make_shared<m3d::QuaternionTransform>();

  std::vector<m3d::TransformInterface::Ptr> interface_vec = {
    axis_angle_if, dual_quaternion_if, euler_if, matrix_if, quaternion_if
  };

  for (const m3d::TransformInterface::Ptr &interface : interface_vec)
  {
    interface->scaleTranslation(2.0);
  }
}

TEST(TransformInterface, applyDirect)
{
  m3d::AxisAngleTransform axis_angle;
  m3d::DualQuaternionTransform dual_quaternion;
  m3d::EulerTransform euler;
  m3d::MatrixTransform matrix;
  m3d::QuaternionTransform quaternion;
  m3d::TransformInterface::Ptr interface = std::make_shared<m3d::AxisAngleTransform>();

  // apply pre
  m3d::AxisAngleTransform aa_pre = axis_angle.applyPre(axis_angle);
  m3d::AxisAngleTransform ad_pre = axis_angle.applyPre(dual_quaternion);
  m3d::AxisAngleTransform ae_pre = axis_angle.applyPre(euler);
  m3d::AxisAngleTransform am_pre = axis_angle.applyPre(matrix);
  m3d::AxisAngleTransform aq_pre = axis_angle.applyPre(quaternion);
  m3d::AxisAngleTransform ai_pre = axis_angle.applyPre(interface);

  m3d::DualQuaternionTransform da_pre = dual_quaternion.applyPre(axis_angle);
  m3d::DualQuaternionTransform dd_pre = dual_quaternion.applyPre(dual_quaternion);
  m3d::DualQuaternionTransform de_pre = dual_quaternion.applyPre(euler);
  m3d::DualQuaternionTransform dm_pre = dual_quaternion.applyPre(matrix);
  m3d::DualQuaternionTransform dq_pre = dual_quaternion.applyPre(quaternion);
  m3d::DualQuaternionTransform di_pre = dual_quaternion.applyPre(interface);

  m3d::EulerTransform ea_pre = euler.applyPre(axis_angle);
  m3d::EulerTransform ed_pre = euler.applyPre(dual_quaternion);
  m3d::EulerTransform ee_pre = euler.applyPre(euler);
  m3d::EulerTransform em_pre = euler.applyPre(matrix);
  m3d::EulerTransform eq_pre = euler.applyPre(quaternion);
  m3d::EulerTransform ei_pre = euler.applyPre(interface);

  m3d::MatrixTransform ma_pre = matrix.applyPre(axis_angle);
  m3d::MatrixTransform md_pre = matrix.applyPre(dual_quaternion);
  m3d::MatrixTransform me_pre = matrix.applyPre(euler);
  m3d::MatrixTransform mm_pre = matrix.applyPre(matrix);
  m3d::MatrixTransform mq_pre = matrix.applyPre(quaternion);
  m3d::MatrixTransform mi_pre = matrix.applyPre(interface);

  m3d::QuaternionTransform qa_pre = quaternion.applyPre(axis_angle);
  m3d::QuaternionTransform qd_pre = quaternion.applyPre(dual_quaternion);
  m3d::QuaternionTransform qe_pre = quaternion.applyPre(euler);
  m3d::QuaternionTransform qm_pre = quaternion.applyPre(matrix);
  m3d::QuaternionTransform qq_pre = quaternion.applyPre(quaternion);
  m3d::QuaternionTransform qi_pre = quaternion.applyPre(interface);

  // apply post
  m3d::AxisAngleTransform aa_post = axis_angle.applyPost(axis_angle);
  m3d::AxisAngleTransform ad_post = axis_angle.applyPost(dual_quaternion);
  m3d::AxisAngleTransform ae_post = axis_angle.applyPost(euler);
  m3d::AxisAngleTransform am_post = axis_angle.applyPost(matrix);
  m3d::AxisAngleTransform aq_post = axis_angle.applyPost(quaternion);
  m3d::AxisAngleTransform ai_post = axis_angle.applyPost(interface);

  m3d::DualQuaternionTransform da_post = dual_quaternion.applyPost(axis_angle);
  m3d::DualQuaternionTransform dd_post = dual_quaternion.applyPost(dual_quaternion);
  m3d::DualQuaternionTransform de_post = dual_quaternion.applyPost(euler);
  m3d::DualQuaternionTransform dm_post = dual_quaternion.applyPost(matrix);
  m3d::DualQuaternionTransform dq_post = dual_quaternion.applyPost(quaternion);
  m3d::DualQuaternionTransform di_post = dual_quaternion.applyPost(interface);

  m3d::EulerTransform ea_post = euler.applyPost(axis_angle);
  m3d::EulerTransform ed_post = euler.applyPost(dual_quaternion);
  m3d::EulerTransform ee_post = euler.applyPost(euler);
  m3d::EulerTransform em_post = euler.applyPost(matrix);
  m3d::EulerTransform eq_post = euler.applyPost(quaternion);
  m3d::EulerTransform ei_post = euler.applyPost(interface);

  m3d::MatrixTransform ma_post = matrix.applyPost(axis_angle);
  m3d::MatrixTransform md_post = matrix.applyPost(dual_quaternion);
  m3d::MatrixTransform me_post = matrix.applyPost(euler);
  m3d::MatrixTransform mm_post = matrix.applyPost(matrix);
  m3d::MatrixTransform mq_post = matrix.applyPost(quaternion);
  m3d::MatrixTransform mi_post = matrix.applyPost(interface);

  m3d::QuaternionTransform qa_post = quaternion.applyPost(axis_angle);
  m3d::QuaternionTransform qd_post = quaternion.applyPost(dual_quaternion);
  m3d::QuaternionTransform qe_post = quaternion.applyPost(euler);
  m3d::QuaternionTransform qm_post = quaternion.applyPost(matrix);
  m3d::QuaternionTransform qq_post = quaternion.applyPost(quaternion);
  m3d::QuaternionTransform qi_post = quaternion.applyPost(interface);
}

TEST(TransformInterface, applyInterface)
{
  // generate data
  m3d::AxisAngleTransform axis_angle;
  m3d::DualQuaternionTransform dual_quaternion;
  m3d::EulerTransform euler;
  m3d::MatrixTransform matrix;
  m3d::QuaternionTransform quaternion;

  m3d::TransformInterface::Ptr axis_angle_if = std::make_shared<m3d::AxisAngleTransform>();
  m3d::TransformInterface::Ptr dual_quaternion_if = std::make_shared<m3d::DualQuaternionTransform>();
  m3d::TransformInterface::Ptr euler_if = std::make_shared<m3d::EulerTransform>();
  m3d::TransformInterface::Ptr matrix_if = std::make_shared<m3d::MatrixTransform>();
  m3d::TransformInterface::Ptr quaternion_if = std::make_shared<m3d::QuaternionTransform>();

  std::vector<m3d::TransformInterface::Ptr> interface_vec = {
    axis_angle_if, dual_quaternion_if, euler_if, matrix_if, quaternion_if
  };

  for (const m3d::TransformInterface::Ptr &interface1 : interface_vec)
  {
    // store type
    auto interface1_type = interface1->getType();

    // apply interface on interface
    for (const m3d::TransformInterface::Ptr &interface2 : interface_vec)
    {
      // copy
      m3d::TransformInterface::Ptr ii_pre = interface1->applyPre(interface2);
      EXPECT_TRUE(ii_pre->isType(interface1_type));
      
      m3d::TransformInterface::Ptr ii_post = interface1->applyPost(interface2);
      EXPECT_TRUE(ii_post->isType(interface1_type));

      // inplace
      interface1->applyPre_(interface2);
      EXPECT_TRUE(interface1->isType(interface1_type));

      interface1->applyPost_(interface2);
      EXPECT_TRUE(interface1->isType(interface1_type));
    }

    // apply direct on interface
    m3d::TransformInterface::Ptr ia_pre = interface1->applyPre(axis_angle);
    m3d::TransformInterface::Ptr id_pre = interface1->applyPre(dual_quaternion);
    m3d::TransformInterface::Ptr ie_pre = interface1->applyPre(euler);
    m3d::TransformInterface::Ptr im_pre = interface1->applyPre(matrix);
    m3d::TransformInterface::Ptr iq_pre = interface1->applyPre(quaternion);

    EXPECT_TRUE(ia_pre->isType(interface1->getType()));
    EXPECT_TRUE(id_pre->isType(interface1->getType()));
    EXPECT_TRUE(ie_pre->isType(interface1->getType()));
    EXPECT_TRUE(im_pre->isType(interface1->getType()));
    EXPECT_TRUE(iq_pre->isType(interface1->getType()));

    m3d::TransformInterface::Ptr ia_post = interface1->applyPost(axis_angle);
    m3d::TransformInterface::Ptr id_post = interface1->applyPost(dual_quaternion);
    m3d::TransformInterface::Ptr ie_post = interface1->applyPost(euler);
    m3d::TransformInterface::Ptr im_post = interface1->applyPost(matrix);
    m3d::TransformInterface::Ptr iq_post = interface1->applyPost(quaternion);

    EXPECT_TRUE(ia_post->isType(interface1->getType()));
    EXPECT_TRUE(id_post->isType(interface1->getType()));
    EXPECT_TRUE(ie_post->isType(interface1->getType()));
    EXPECT_TRUE(im_post->isType(interface1->getType()));
    EXPECT_TRUE(iq_post->isType(interface1->getType()));

    // apply direct on interface (inplace)
    interface1->applyPre_(axis_angle);
    EXPECT_TRUE(interface1->isType(interface1_type));
    interface1->applyPre_(dual_quaternion);
    EXPECT_TRUE(interface1->isType(interface1_type));
    interface1->applyPre_(euler);
    EXPECT_TRUE(interface1->isType(interface1_type));
    interface1->applyPre_(matrix);
    EXPECT_TRUE(interface1->isType(interface1_type));
    interface1->applyPre_(quaternion);
    EXPECT_TRUE(interface1->isType(interface1_type));

    interface1->applyPost_(axis_angle);
    EXPECT_TRUE(interface1->isType(interface1_type));
    interface1->applyPost_(dual_quaternion);
    EXPECT_TRUE(interface1->isType(interface1_type));
    interface1->applyPost_(euler);
    EXPECT_TRUE(interface1->isType(interface1_type));
    interface1->applyPost_(matrix);
    EXPECT_TRUE(interface1->isType(interface1_type));
    interface1->applyPost_(quaternion);
    EXPECT_TRUE(interface1->isType(interface1_type));
  }
}

TEST(TransformInterface, norm)
{
  // direct
  m3d::AxisAngleTransform axis_angle;
  m3d::DualQuaternionTransform dual_quaternion;
  m3d::EulerTransform euler;
  m3d::MatrixTransform matrix;
  m3d::QuaternionTransform quaternion;

  double ar = axis_angle.rotationNorm();
  double at = axis_angle.translationNorm();
  double dr = dual_quaternion.rotationNorm();
  double dt = dual_quaternion.translationNorm();
  double er = euler.rotationNorm();
  double et = euler.translationNorm();
  double mr = matrix.rotationNorm();
  double mt = matrix.translationNorm();
  double qr = quaternion.rotationNorm();
  double qt = quaternion.translationNorm();

  EXPECT_EQ(ar, 0.0);
  EXPECT_EQ(at, 0.0);
  EXPECT_EQ(dr, 0.0);
  EXPECT_EQ(dt, 0.0);
  EXPECT_EQ(er, 0.0);
  EXPECT_EQ(et, 0.0);
  EXPECT_EQ(mr, 0.0);
  EXPECT_EQ(mt, 0.0);
  EXPECT_EQ(qr, 0.0);
  EXPECT_EQ(qt, 0.0);

  // interface
  m3d::TransformInterface::Ptr axis_angle_if = std::make_shared<m3d::AxisAngleTransform>();
  m3d::TransformInterface::Ptr dual_quaternion_if = std::make_shared<m3d::DualQuaternionTransform>();
  m3d::TransformInterface::Ptr euler_if = std::make_shared<m3d::EulerTransform>();
  m3d::TransformInterface::Ptr matrix_if = std::make_shared<m3d::MatrixTransform>();
  m3d::TransformInterface::Ptr quaternion_if = std::make_shared<m3d::QuaternionTransform>();

  double air = axis_angle_if->rotationNorm();
  double ait = axis_angle_if->translationNorm();
  double dir = dual_quaternion_if->rotationNorm();
  double dit = dual_quaternion_if->translationNorm();
  double eir = euler_if->rotationNorm();
  double eit = euler_if->translationNorm();
  double mir = matrix_if->rotationNorm();
  double mit = matrix_if->translationNorm();
  double qir = quaternion_if->rotationNorm();
  double qit = quaternion_if->translationNorm();

  EXPECT_EQ(air, 0.0);
  EXPECT_EQ(ait, 0.0);
  EXPECT_EQ(dir, 0.0);
  EXPECT_EQ(dit, 0.0);
  EXPECT_EQ(eir, 0.0);
  EXPECT_EQ(eit, 0.0);
  EXPECT_EQ(mir, 0.0);
  EXPECT_EQ(mit, 0.0);
  EXPECT_EQ(qir, 0.0);
  EXPECT_EQ(qit, 0.0);
}

TEST(TransformInterface, operatorsDirect)
{
  m3d::AxisAngleTransform axis_angle;
  m3d::DualQuaternionTransform dual_quaternion;
  m3d::EulerTransform euler;
  m3d::MatrixTransform matrix;
  m3d::QuaternionTransform quaternion;

  // mutiply
  m3d::AxisAngleTransform ama = axis_angle * axis_angle;
  m3d::AxisAngleTransform amd = axis_angle * dual_quaternion;
  m3d::AxisAngleTransform ame = axis_angle * euler;
  m3d::AxisAngleTransform amm = axis_angle * matrix;
  m3d::AxisAngleTransform amq = axis_angle * quaternion;
  m3d::DualQuaternionTransform dma = dual_quaternion * axis_angle;
  m3d::DualQuaternionTransform dmd = dual_quaternion * dual_quaternion;
  m3d::DualQuaternionTransform dme = dual_quaternion * euler;
  m3d::DualQuaternionTransform dmm = dual_quaternion * matrix;
  m3d::DualQuaternionTransform dmq = dual_quaternion * quaternion;
  m3d::EulerTransform ema = euler * axis_angle;
  m3d::EulerTransform emd = euler * dual_quaternion;
  m3d::EulerTransform eme = euler * euler;
  m3d::EulerTransform emm = euler * matrix;
  m3d::EulerTransform emq = euler * quaternion;
  m3d::MatrixTransform mma = matrix * axis_angle;
  m3d::MatrixTransform mmd = matrix * dual_quaternion;
  m3d::MatrixTransform mme = matrix * euler;
  m3d::MatrixTransform mmm = matrix * matrix;
  m3d::MatrixTransform mmq = matrix * quaternion;
  m3d::QuaternionTransform qma = quaternion * axis_angle;
  m3d::QuaternionTransform qmd = quaternion * dual_quaternion;
  m3d::QuaternionTransform qme = quaternion * euler;
  m3d::QuaternionTransform qmm = quaternion * matrix;
  m3d::QuaternionTransform qmq = quaternion * quaternion;

  // divide
  m3d::AxisAngleTransform ada = axis_angle / axis_angle;
  m3d::AxisAngleTransform add = axis_angle / dual_quaternion;
  m3d::AxisAngleTransform ade = axis_angle / euler;
  m3d::AxisAngleTransform adm = axis_angle / matrix;
  m3d::AxisAngleTransform adq = axis_angle / quaternion;
  m3d::DualQuaternionTransform dda = dual_quaternion / axis_angle;
  m3d::DualQuaternionTransform ddd = dual_quaternion / dual_quaternion;
  m3d::DualQuaternionTransform dde = dual_quaternion / euler;
  m3d::DualQuaternionTransform ddm = dual_quaternion / matrix;
  m3d::DualQuaternionTransform ddq = dual_quaternion / quaternion;
  m3d::EulerTransform eda = euler / axis_angle;
  m3d::EulerTransform edd = euler / dual_quaternion;
  m3d::EulerTransform ede = euler / euler;
  m3d::EulerTransform edm = euler / matrix;
  m3d::EulerTransform edq = euler / quaternion;
  m3d::MatrixTransform mda = matrix / axis_angle;
  m3d::MatrixTransform mdd = matrix / dual_quaternion;
  m3d::MatrixTransform mde = matrix / euler;
  m3d::MatrixTransform mdm = matrix / matrix;
  m3d::MatrixTransform mdq = matrix / quaternion;
  m3d::QuaternionTransform qda = quaternion / axis_angle;
  m3d::QuaternionTransform qdd = quaternion / dual_quaternion;
  m3d::QuaternionTransform qde = quaternion / euler;
  m3d::QuaternionTransform qdm = quaternion / matrix;
  m3d::QuaternionTransform qdq = quaternion / quaternion;
}

TEST(TransformInterface, operatorsInterface)
{
  // generate data
  m3d::AxisAngleTransform axis_angle;
  m3d::DualQuaternionTransform dual_quaternion;
  m3d::EulerTransform euler;
  m3d::MatrixTransform matrix;
  m3d::QuaternionTransform quaternion;

  m3d::TransformInterface::Ptr axis_angle_if = std::make_shared<m3d::AxisAngleTransform>();
  m3d::TransformInterface::Ptr dual_quaternion_if = std::make_shared<m3d::DualQuaternionTransform>();
  m3d::TransformInterface::Ptr euler_if = std::make_shared<m3d::EulerTransform>();
  m3d::TransformInterface::Ptr matrix_if = std::make_shared<m3d::MatrixTransform>();
  m3d::TransformInterface::Ptr quaternion_if = std::make_shared<m3d::QuaternionTransform>();

  std::vector<m3d::TransformInterface::Ptr> interface_vec = {
    axis_angle_if, dual_quaternion_if, euler_if, matrix_if, quaternion_if
  };

  for (const m3d::TransformInterface::Ptr &interface1 : interface_vec)
  {
    // interface only
    for (const m3d::TransformInterface::Ptr &interface2 : interface_vec)
    {
      m3d::TransformInterface::Ptr imi = interface1 * interface2;
      m3d::TransformInterface::Ptr idi = interface1 / interface2;
    }

    // direct and interface
    m3d::TransformInterface::Ptr ima = interface1 * axis_angle;
    m3d::TransformInterface::Ptr imd = interface1 * dual_quaternion;
    m3d::TransformInterface::Ptr ime = interface1 * euler;
    m3d::TransformInterface::Ptr imm = interface1 * matrix;
    m3d::TransformInterface::Ptr imq = interface1 * quaternion;

    m3d::TransformInterface::Ptr ida = interface1 / axis_angle;
    m3d::TransformInterface::Ptr idd = interface1 / dual_quaternion;
    m3d::TransformInterface::Ptr ide = interface1 / euler;
    m3d::TransformInterface::Ptr idm = interface1 / matrix;
    m3d::TransformInterface::Ptr idq = interface1 / quaternion;

    m3d::AxisAngleTransform ami = axis_angle * interface1;
    m3d::DualQuaternionTransform dmi = dual_quaternion * interface1;
    m3d::EulerTransform emi = euler * interface1;
    m3d::MatrixTransform mmi = matrix * interface1;
    m3d::QuaternionTransform qmi = quaternion * interface1;

    m3d::AxisAngleTransform adi = axis_angle / interface1;
    m3d::DualQuaternionTransform ddi = dual_quaternion / interface1;
    m3d::EulerTransform edi = euler / interface1;
    m3d::MatrixTransform mdi = matrix / interface1;
    m3d::QuaternionTransform qdi = quaternion / interface1;
  }
}

TEST(TransformInterface, unsafe)
{
  // create transforms
  auto axis_angle_safe_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle1Vec, false);
  auto dual_quaternion_safe_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion1Vec, false);
  auto euler_safe_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler1Vec, false);
  auto matrix_safe_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix1Vec, false);
  auto quaternion_safe_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion1Vec, false);

  std::vector<std::shared_ptr<m3d::TransformInterface>> safe_vec = {
    axis_angle_safe_if, dual_quaternion_safe_if, euler_safe_if, matrix_safe_if, quaternion_safe_if
  };

  auto axis_angle_unsafe_if = m3d::TransformInterface::Factory(m3d::TransformType::kAxisAngle, kAxisAngle1Vec, true);
  auto dual_quaternion_unsafe_if = m3d::TransformInterface::Factory(m3d::TransformType::kDualQuaternion, kDualQuaternion1Vec, true);
  auto euler_unsafe_if = m3d::TransformInterface::Factory(m3d::TransformType::kEuler, kEuler1Vec, true);
  auto matrix_unsafe_if = m3d::TransformInterface::Factory(m3d::TransformType::kMatrix, kMatrix1Vec, true);
  auto quaternion_unsafe_if = m3d::TransformInterface::Factory(m3d::TransformType::kQuaternion, kQuaternion1Vec, true);

  std::vector<std::shared_ptr<m3d::TransformInterface>> unsafe_vec = {
    axis_angle_unsafe_if, dual_quaternion_unsafe_if, euler_unsafe_if, matrix_unsafe_if, quaternion_unsafe_if
  };

  // check safe
  for (auto interface : safe_vec)
  {
    EXPECT_FALSE(interface->isUnsafe());
    EXPECT_FALSE(interface->inverse()->isUnsafe());
    EXPECT_FALSE(interface->scaleTranslation(1.0)->isUnsafe());
    EXPECT_FALSE(interface->asType<m3d::AxisAngleTransform>()->isUnsafe());
    EXPECT_FALSE(interface->asType<m3d::DualQuaternionTransform>()->isUnsafe());
    EXPECT_FALSE(interface->asType<m3d::EulerTransform>()->isUnsafe());
    EXPECT_FALSE(interface->asType<m3d::MatrixTransform>()->isUnsafe());
    EXPECT_FALSE(interface->asType<m3d::QuaternionTransform>()->isUnsafe());
    EXPECT_FALSE(interface->normalized()->isUnsafe());
  }

  // check unsafe
  for (auto interface : unsafe_vec)
  {
    EXPECT_TRUE(interface->isUnsafe());
    EXPECT_TRUE(interface->inverse()->isUnsafe());
    EXPECT_TRUE(interface->scaleTranslation(1.0)->isUnsafe());
    EXPECT_TRUE(interface->asType<m3d::AxisAngleTransform>()->isUnsafe());
    EXPECT_TRUE(interface->asType<m3d::DualQuaternionTransform>()->isUnsafe());
    EXPECT_TRUE(interface->asType<m3d::EulerTransform>()->isUnsafe());
    EXPECT_TRUE(interface->asType<m3d::MatrixTransform>()->isUnsafe());
    EXPECT_TRUE(interface->asType<m3d::QuaternionTransform>()->isUnsafe());
    EXPECT_FALSE(interface->normalized()->isUnsafe());
  }

  // check combination
  for (auto safe_if1 : safe_vec)
  {
    for (auto safe_if2 : safe_vec)
    {
      EXPECT_FALSE(safe_if1->applyPre(safe_if2)->isUnsafe());
      EXPECT_FALSE(safe_if1->applyPost(safe_if2)->isUnsafe());
      EXPECT_FALSE(safe_if1->copy()->applyPre_(safe_if2)->isUnsafe());
      EXPECT_FALSE(safe_if1->copy()->applyPost_(safe_if2)->isUnsafe());
    }
  }

  for (auto safe_if : safe_vec)
  {
    for (auto unsafe_if : unsafe_vec)
    {
      EXPECT_TRUE(safe_if->applyPre(unsafe_if)->isUnsafe());
      EXPECT_TRUE(unsafe_if->applyPre(safe_if)->isUnsafe());
      EXPECT_TRUE(safe_if->applyPost(unsafe_if)->isUnsafe());
      EXPECT_TRUE(unsafe_if->applyPost(safe_if)->isUnsafe());
      EXPECT_TRUE(safe_if->copy()->applyPre_(unsafe_if)->isUnsafe());
      EXPECT_TRUE(unsafe_if->copy()->applyPre_(safe_if)->isUnsafe());
      EXPECT_TRUE(safe_if->copy()->applyPost_(unsafe_if)->isUnsafe());
      EXPECT_TRUE(unsafe_if->copy()->applyPost_(safe_if)->isUnsafe());
    }
  }

  for (auto unsafe_if1 : unsafe_vec)
  {
    for (auto unsafe_if2 : unsafe_vec)
    {
      EXPECT_TRUE(unsafe_if1->applyPre(unsafe_if2)->isUnsafe());
      EXPECT_TRUE(unsafe_if1->applyPost(unsafe_if2)->isUnsafe());
      EXPECT_TRUE(unsafe_if1->copy()->applyPre_(unsafe_if2)->isUnsafe());
      EXPECT_TRUE(unsafe_if1->copy()->applyPost_(unsafe_if2)->isUnsafe());
    }
  }
}

TEST(TransformInterface, description)
{
  // generate data
  m3d::AxisAngleTransform axis_angle;
  m3d::DualQuaternionTransform dual_quaternion;
  m3d::EulerTransform euler;
  m3d::MatrixTransform matrix;
  m3d::QuaternionTransform quaternion;

  m3d::TransformInterface::Ptr axis_angle_if = std::make_shared<m3d::AxisAngleTransform>();
  m3d::TransformInterface::Ptr dual_quaternion_if = std::make_shared<m3d::DualQuaternionTransform>();
  m3d::TransformInterface::Ptr euler_if = std::make_shared<m3d::EulerTransform>();
  m3d::TransformInterface::Ptr matrix_if = std::make_shared<m3d::MatrixTransform>();
  m3d::TransformInterface::Ptr quaternion_if = std::make_shared<m3d::QuaternionTransform>();

  // direct stream
  std::stringstream axis_angle_ss;
  std::stringstream dual_quaternion_ss;
  std::stringstream euler_ss;
  std::stringstream matrix_ss;
  std::stringstream quaternion_ss;

  axis_angle_ss << axis_angle;
  dual_quaternion_ss << dual_quaternion;
  euler_ss << euler;
  matrix_ss << matrix;
  quaternion_ss << quaternion;

  ASSERT_STRING_STARTS_WITH(axis_angle_ss.str(), "AxisAngleTransform");
  ASSERT_STRING_STARTS_WITH(dual_quaternion_ss.str(), "DualQuaternionTransform");
  ASSERT_STRING_STARTS_WITH(euler_ss.str(), "EulerTransform");
  ASSERT_STRING_STARTS_WITH(matrix_ss.str(), "MatrixTransform");
  ASSERT_STRING_STARTS_WITH(quaternion_ss.str(), "QuaternionTransform");

  // interface stream
  std::stringstream axis_angle_if_ss;
  std::stringstream dual_quaternion_if_ss;
  std::stringstream euler_if_ss;
  std::stringstream matrix_if_ss;
  std::stringstream quaternion_if_ss;

  axis_angle_if_ss << axis_angle_if;
  dual_quaternion_if_ss << dual_quaternion_if;
  euler_if_ss << euler_if;
  matrix_if_ss << matrix_if;
  quaternion_if_ss << quaternion_if;

  ASSERT_STRING_STARTS_WITH(axis_angle_if_ss.str(), "AxisAngleTransform");
  ASSERT_STRING_STARTS_WITH(dual_quaternion_if_ss.str(), "DualQuaternionTransform");
  ASSERT_STRING_STARTS_WITH(euler_if_ss.str(), "EulerTransform");
  ASSERT_STRING_STARTS_WITH(matrix_if_ss.str(), "MatrixTransform");
  ASSERT_STRING_STARTS_WITH(quaternion_if_ss.str(), "QuaternionTransform");

  // description
  ASSERT_EQ(axis_angle.desc(), axis_angle_ss.str());
  ASSERT_EQ(dual_quaternion.desc(), dual_quaternion_ss.str());
  ASSERT_EQ(euler.desc(), euler_ss.str());
  ASSERT_EQ(matrix.desc(), matrix_ss.str());
  ASSERT_EQ(quaternion.desc(), quaternion_ss.str());

  ASSERT_EQ(axis_angle_if->desc(), axis_angle_if_ss.str());
  ASSERT_EQ(dual_quaternion_if->desc(), dual_quaternion_if_ss.str());
  ASSERT_EQ(euler_if->desc(), euler_if_ss.str());
  ASSERT_EQ(matrix_if->desc(), matrix_if_ss.str());
  ASSERT_EQ(quaternion_if->desc(), quaternion_if_ss.str());
}
