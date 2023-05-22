#pragma once

#include <gtest/gtest.h>

#include <motion3d/containers.hpp>
#include <motion3d/transforms.hpp>

#include "data.hpp"

namespace m3d = motion3d;


/// Dummy function for applying directly on transforms.
m3d::TransformInterface::Ptr trafoFunc(const m3d::TransformInterface::ConstPtr &t)
{
  auto translation = t->asType<m3d::EulerTransform>()->getTranslation();
  translation[0] += 10.0;
  return std::make_shared<m3d::EulerTransform>(translation, 0.0, 0.0, 0.0, m3d::EulerAxes::kSXYX);
}

/// Dummy function for applying on indices and transforms.
m3d::TransformInterface::Ptr indexFunc(std::size_t index, const m3d::TransformInterface::ConstPtr &t)
{
  auto translation = t->asType<m3d::EulerTransform>()->getTranslation();
  translation[0] += 10.0 + index * 10;
  return std::make_shared<m3d::EulerTransform>(translation, 0.0, 0.0, 0.0, m3d::EulerAxes::kSXYX);
}

/// Dummy function for applying on stamps and transforms.
m3d::TransformInterface::Ptr stampFunc(const m3d::Time &stamp, const m3d::TransformInterface::ConstPtr &t)
{
  auto translation = t->asType<m3d::EulerTransform>()->getTranslation();
  translation[0] += 10.0 + stamp.toSec();
  return std::make_shared<m3d::EulerTransform>(translation, 0.0, 0.0, 0.0, m3d::EulerAxes::kSXYX);
}


TEST(TransformContainer, constructor)
{
  // empty
  m3d::TransformContainer data_empty_m(false, false);
  EXPECT_FALSE(data_empty_m.hasStamps());
  EXPECT_FALSE(data_empty_m.hasPoses());
  EXPECT_EQ(data_empty_m.size(), 0);

  m3d::TransformContainer data_empty_p(false, true);
  EXPECT_FALSE(data_empty_p.hasStamps());
  EXPECT_TRUE(data_empty_p.hasPoses());
  EXPECT_EQ(data_empty_p.size(), 0);

  m3d::TransformContainer data_empty_ms(true, false);
  EXPECT_TRUE(data_empty_ms.hasStamps());
  EXPECT_FALSE(data_empty_ms.hasPoses());
  EXPECT_EQ(data_empty_ms.size(), 0);

  m3d::TransformContainer data_empty_ps(true, true);
  EXPECT_TRUE(data_empty_ps.hasStamps());
  EXPECT_TRUE(data_empty_ps.hasPoses());
  EXPECT_EQ(data_empty_ps.size(), 0);

  // direct
  std::vector<m3d::DualQuaternionTransform> transforms1 = {
    m3d::DualQuaternionTransform(),
    m3d::DualQuaternionTransform()
  };

  m3d::TransformContainer data_direct_m(transforms1, false);
  EXPECT_FALSE(data_direct_m.hasStamps());
  EXPECT_FALSE(data_direct_m.hasPoses());
  ASSERT_EQ(data_direct_m.size(), 2);
  EXPECT_TRUE(data_direct_m.at(0)->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_direct_m.at(1)->isType(m3d::TransformType::kDualQuaternion));

  m3d::TransformContainer data_direct_p(transforms1, true);
  EXPECT_FALSE(data_direct_p.hasStamps());
  EXPECT_TRUE(data_direct_p.hasPoses());
  ASSERT_EQ(data_direct_p.size(), 2);
  EXPECT_TRUE(data_direct_p.at(0)->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_direct_p.at(1)->isType(m3d::TransformType::kDualQuaternion));

  // direct with stamps
  std::map<m3d::Time, m3d::DualQuaternionTransform> transforms_stamped1;
  transforms_stamped1[m3d::Time(0.0)] = m3d::DualQuaternionTransform();
  transforms_stamped1[m3d::Time(1.0)] = m3d::DualQuaternionTransform();

  m3d::TransformContainer data_direct_m_stamped(transforms_stamped1, false);
  EXPECT_TRUE(data_direct_m_stamped.hasStamps());
  EXPECT_FALSE(data_direct_m_stamped.hasPoses());
  ASSERT_EQ(data_direct_m_stamped.size(), 2);
  EXPECT_TRUE(data_direct_m_stamped.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_direct_m_stamped.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kDualQuaternion));

  m3d::TransformContainer data_direct_p_stamped(transforms_stamped1, true);
  EXPECT_TRUE(data_direct_p_stamped.hasStamps());
  EXPECT_TRUE(data_direct_p_stamped.hasPoses());
  ASSERT_EQ(data_direct_p_stamped.size(), 2);
  EXPECT_TRUE(data_direct_p_stamped.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_direct_p_stamped.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kDualQuaternion));

  // direct with separate stamps and data
  std::vector<m3d::Time> stamps = {
    m3d::Time(0.0),
    m3d::Time(1.0)
  };

  m3d::TransformContainer data_direct_m_stamped_sep1(stamps, transforms1, false, false);
  EXPECT_TRUE(data_direct_m_stamped_sep1.hasStamps());
  EXPECT_FALSE(data_direct_m_stamped_sep1.hasPoses());
  ASSERT_EQ(data_direct_m_stamped_sep1.size(), 2);
  EXPECT_TRUE(data_direct_m_stamped_sep1.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_direct_m_stamped_sep1.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kDualQuaternion));

  m3d::TransformContainer data_direct_m_stamped_sep2(stamps, transforms1, false, true);
  EXPECT_TRUE(data_direct_m_stamped_sep2.hasStamps());
  EXPECT_FALSE(data_direct_m_stamped_sep2.hasPoses());
  ASSERT_EQ(data_direct_m_stamped_sep2.size(), 2);
  EXPECT_TRUE(data_direct_m_stamped_sep2.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_direct_m_stamped_sep2.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kDualQuaternion));

  m3d::TransformContainer data_direct_p_stamped_sep1(stamps, transforms1, true, false);
  EXPECT_TRUE(data_direct_p_stamped_sep1.hasStamps());
  EXPECT_TRUE(data_direct_p_stamped_sep1.hasPoses());
  ASSERT_EQ(data_direct_p_stamped_sep1.size(), 2);
  EXPECT_TRUE(data_direct_p_stamped_sep1.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_direct_p_stamped_sep1.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kDualQuaternion));

  m3d::TransformContainer data_direct_p_stamped_sep2(stamps, transforms1, true, true);
  EXPECT_TRUE(data_direct_p_stamped_sep2.hasStamps());
  EXPECT_TRUE(data_direct_p_stamped_sep2.hasPoses());
  ASSERT_EQ(data_direct_p_stamped_sep2.size(), 2);
  EXPECT_TRUE(data_direct_p_stamped_sep2.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_direct_p_stamped_sep2.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kDualQuaternion));

  // interface
  std::vector<m3d::TransformInterface::Ptr> transforms2 = {
    std::make_shared<m3d::DualQuaternionTransform>(),
    std::make_shared<m3d::MatrixTransform>()
  };
  std::vector<m3d::TransformInterface::ConstPtr> transforms3 = {
    std::make_shared<m3d::DualQuaternionTransform>(),
    std::make_shared<m3d::MatrixTransform>()
  };

  m3d::TransformContainer data_if_m(transforms2, false);
  EXPECT_FALSE(data_if_m.hasStamps());
  EXPECT_FALSE(data_if_m.hasPoses());
  ASSERT_EQ(data_if_m.size(), 2);
  EXPECT_TRUE(data_if_m.at(0)->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_if_m.at(1)->isType(m3d::TransformType::kMatrix));

  m3d::TransformContainer data_if_p(transforms2, true);
  EXPECT_FALSE(data_if_p.hasStamps());
  EXPECT_TRUE(data_if_p.hasPoses());
  ASSERT_EQ(data_if_p.size(), 2);
  EXPECT_TRUE(data_if_p.at(0)->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_if_p.at(1)->isType(m3d::TransformType::kMatrix));

  m3d::TransformContainer data_const_if_m(transforms3, false);
  EXPECT_FALSE(data_const_if_m.hasStamps());
  EXPECT_FALSE(data_const_if_m.hasPoses());
  ASSERT_EQ(data_const_if_m.size(), 2);
  EXPECT_TRUE(data_const_if_m.at(0)->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_const_if_m.at(1)->isType(m3d::TransformType::kMatrix));

  m3d::TransformContainer data_const_if_p(transforms3, true);
  EXPECT_FALSE(data_const_if_p.hasStamps());
  EXPECT_TRUE(data_const_if_p.hasPoses());
  ASSERT_EQ(data_const_if_p.size(), 2);
  EXPECT_TRUE(data_const_if_p.at(0)->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_const_if_p.at(1)->isType(m3d::TransformType::kMatrix));

  // interface with stamps
  std::map<m3d::Time, m3d::TransformInterface::Ptr> transforms_stamped2;
  transforms_stamped2[m3d::Time(0.0)] = std::make_shared<m3d::DualQuaternionTransform>();
  transforms_stamped2[m3d::Time(1.0)] = std::make_shared<m3d::MatrixTransform>();

  std::map<m3d::Time, m3d::TransformInterface::ConstPtr> transforms_stamped3;
  transforms_stamped3[m3d::Time(0.0)] = std::make_shared<m3d::DualQuaternionTransform>();
  transforms_stamped3[m3d::Time(1.0)] = std::make_shared<m3d::MatrixTransform>();

  m3d::TransformContainer data_if_m_stamped(transforms_stamped2, false);
  EXPECT_TRUE(data_if_m_stamped.hasStamps());
  EXPECT_FALSE(data_if_m_stamped.hasPoses());
  ASSERT_EQ(data_if_m_stamped.size(), 2);
  EXPECT_TRUE(data_if_m_stamped.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_if_m_stamped.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kMatrix));

  m3d::TransformContainer data_if_p_stamped(transforms_stamped2, true);
  EXPECT_TRUE(data_if_p_stamped.hasStamps());
  EXPECT_TRUE(data_if_p_stamped.hasPoses());
  ASSERT_EQ(data_if_p_stamped.size(), 2);
  EXPECT_TRUE(data_if_p_stamped.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_if_p_stamped.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kMatrix));

  m3d::TransformContainer data_const_if_m_stamped(transforms_stamped3, false);
  EXPECT_TRUE(data_const_if_m_stamped.hasStamps());
  EXPECT_FALSE(data_const_if_m_stamped.hasPoses());
  ASSERT_EQ(data_const_if_m_stamped.size(), 2);
  EXPECT_TRUE(data_const_if_m_stamped.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_const_if_m_stamped.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kMatrix));

  m3d::TransformContainer data_const_if_p_stamped(transforms_stamped3, true);
  EXPECT_TRUE(data_const_if_p_stamped.hasStamps());
  EXPECT_TRUE(data_const_if_p_stamped.hasPoses());
  ASSERT_EQ(data_const_if_p_stamped.size(), 2);
  EXPECT_TRUE(data_const_if_p_stamped.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_const_if_p_stamped.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kMatrix));

  // interface with separate stamps and transforms
  m3d::TransformContainer data_if_m_stamped_sep1(stamps, transforms2, false, false);
  EXPECT_TRUE(data_if_m_stamped_sep1.hasStamps());
  EXPECT_FALSE(data_if_m_stamped_sep1.hasPoses());
  ASSERT_EQ(data_if_m_stamped_sep1.size(), 2);
  EXPECT_TRUE(data_if_m_stamped_sep1.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_if_m_stamped_sep1.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kMatrix));

  m3d::TransformContainer data_if_m_stamped_sep2(stamps, transforms2, false, true);
  EXPECT_TRUE(data_if_m_stamped_sep2.hasStamps());
  EXPECT_FALSE(data_if_m_stamped_sep2.hasPoses());
  ASSERT_EQ(data_if_m_stamped_sep2.size(), 2);
  EXPECT_TRUE(data_if_m_stamped_sep2.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_if_m_stamped_sep2.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kMatrix));

  m3d::TransformContainer data_if_p_stamped_sep1(stamps, transforms2, true, false);
  EXPECT_TRUE(data_if_p_stamped_sep1.hasStamps());
  EXPECT_TRUE(data_if_p_stamped_sep1.hasPoses());
  ASSERT_EQ(data_if_p_stamped_sep1.size(), 2);
  EXPECT_TRUE(data_if_p_stamped_sep1.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_if_p_stamped_sep1.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kMatrix));

  m3d::TransformContainer data_if_p_stamped_sep2(stamps, transforms2, true, true);
  EXPECT_TRUE(data_if_p_stamped_sep2.hasStamps());
  EXPECT_TRUE(data_if_p_stamped_sep2.hasPoses());
  ASSERT_EQ(data_if_p_stamped_sep2.size(), 2);
  EXPECT_TRUE(data_if_p_stamped_sep2.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_if_p_stamped_sep2.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kMatrix));

  m3d::TransformContainer data_const_if_m_stamped_sep1(stamps, transforms3, false, false);
  EXPECT_TRUE(data_const_if_m_stamped_sep1.hasStamps());
  EXPECT_FALSE(data_const_if_m_stamped_sep1.hasPoses());
  ASSERT_EQ(data_const_if_m_stamped_sep1.size(), 2);
  EXPECT_TRUE(data_const_if_m_stamped_sep1.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_const_if_m_stamped_sep1.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kMatrix));

  m3d::TransformContainer data_const_if_m_stamped_sep2(stamps, transforms3, false, true);
  EXPECT_TRUE(data_const_if_m_stamped_sep2.hasStamps());
  EXPECT_FALSE(data_const_if_m_stamped_sep2.hasPoses());
  ASSERT_EQ(data_const_if_m_stamped_sep2.size(), 2);
  EXPECT_TRUE(data_const_if_m_stamped_sep2.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_const_if_m_stamped_sep2.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kMatrix));

  m3d::TransformContainer data_const_if_p_stamped_sep1(stamps, transforms3, true, false);
  EXPECT_TRUE(data_const_if_p_stamped_sep1.hasStamps());
  EXPECT_TRUE(data_const_if_p_stamped_sep1.hasPoses());
  ASSERT_EQ(data_const_if_p_stamped_sep1.size(), 2);
  EXPECT_TRUE(data_const_if_p_stamped_sep1.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_const_if_p_stamped_sep1.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kMatrix));

  m3d::TransformContainer data_const_if_p_stamped_sep2(stamps, transforms3, true, true);
  EXPECT_TRUE(data_const_if_p_stamped_sep2.hasStamps());
  EXPECT_TRUE(data_const_if_p_stamped_sep2.hasPoses());
  ASSERT_EQ(data_const_if_p_stamped_sep2.size(), 2);
  EXPECT_TRUE(data_const_if_p_stamped_sep2.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_const_if_p_stamped_sep2.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kMatrix));

  // copy
  m3d::TransformContainer data_empty_m_copy(data_empty_m);
  EXPECT_FALSE(data_empty_m_copy.hasStamps());
  EXPECT_FALSE(data_empty_m_copy.hasPoses());
  m3d::TransformContainer data_empty_p_copy(data_empty_p);
  EXPECT_FALSE(data_empty_p_copy.hasStamps());
  EXPECT_TRUE(data_empty_p_copy.hasPoses());
  m3d::TransformContainer data_empty_ms_copy(data_empty_ms);
  EXPECT_TRUE(data_empty_ms_copy.hasStamps());
  EXPECT_FALSE(data_empty_ms_copy.hasPoses());
  m3d::TransformContainer data_empty_ps_copy(data_empty_ps);
  EXPECT_TRUE(data_empty_ps_copy.hasStamps());
  EXPECT_TRUE(data_empty_ps_copy.hasPoses());

  m3d::TransformContainer data_if_m_copy(data_if_m);
  EXPECT_NE(data_if_m.at(0), data_if_m_copy.at(0));
  EXPECT_NE(data_if_m.at(1), data_if_m_copy.at(1));
  m3d::TransformContainer data_if_m_stamped_copy(data_if_m_stamped);
  EXPECT_NE(data_if_m_stamped.at(0), data_if_m_stamped_copy.at(0));
  EXPECT_NE(data_if_m_stamped.at(1), data_if_m_stamped_copy.at(1));

  // exceptions
  std::vector<m3d::Time> stamps_error = {
    m3d::Time(0.0)
  };

  std::vector<m3d::TransformInterface::Ptr> transforms2_nullptr = {
    nullptr,
    std::make_shared<m3d::MatrixTransform>()
  };
  std::vector<m3d::TransformInterface::ConstPtr> transforms3_nullptr = {
    nullptr,
    std::make_shared<m3d::MatrixTransform>()
  };

  std::map<m3d::Time, m3d::TransformInterface::Ptr> transforms2_map_nullptr;
  transforms2_map_nullptr[stamps[0]] = transforms2_nullptr[0];
  transforms2_map_nullptr[stamps[1]] = transforms2_nullptr[1];

  std::map<m3d::Time, m3d::TransformInterface::ConstPtr> transforms3_map_nullptr;
  transforms3_map_nullptr[stamps[0]] = transforms3_nullptr[0];
  transforms3_map_nullptr[stamps[1]] = transforms3_nullptr[1];

  EXPECT_THROW(m3d::TransformContainer(transforms2_nullptr, false), std::invalid_argument);
  EXPECT_THROW(m3d::TransformContainer(transforms2_nullptr, true), std::invalid_argument);
  EXPECT_THROW(m3d::TransformContainer(transforms3_nullptr, false), std::invalid_argument);
  EXPECT_THROW(m3d::TransformContainer(transforms3_nullptr, true), std::invalid_argument);

  EXPECT_THROW(m3d::TransformContainer(transforms2_map_nullptr, false), std::invalid_argument);
  EXPECT_THROW(m3d::TransformContainer(transforms2_map_nullptr, true), std::invalid_argument);
  EXPECT_THROW(m3d::TransformContainer(transforms3_map_nullptr, false), std::invalid_argument);
  EXPECT_THROW(m3d::TransformContainer(transforms3_map_nullptr, true), std::invalid_argument);

  EXPECT_THROW(m3d::TransformContainer(stamps_error, transforms1, true, false), std::invalid_argument);
  EXPECT_THROW(m3d::TransformContainer(stamps_error, transforms1, true, true), std::invalid_argument);
  EXPECT_THROW(m3d::TransformContainer(stamps_error, transforms2, true, false), std::invalid_argument);
  EXPECT_THROW(m3d::TransformContainer(stamps_error, transforms2, true, true), std::invalid_argument);
  EXPECT_THROW(m3d::TransformContainer(stamps_error, transforms3, true, false), std::invalid_argument);
  EXPECT_THROW(m3d::TransformContainer(stamps_error, transforms3, true, true), std::invalid_argument);
  EXPECT_THROW(m3d::TransformContainer(stamps, transforms2_nullptr, true, false), std::invalid_argument);
  EXPECT_THROW(m3d::TransformContainer(stamps, transforms2_nullptr, true, true), std::invalid_argument);
  EXPECT_THROW(m3d::TransformContainer(stamps, transforms3_nullptr, true, false), std::invalid_argument);
  EXPECT_THROW(m3d::TransformContainer(stamps, transforms3_nullptr, true, true), std::invalid_argument);
}

TEST(TransformContainer, insert)
{
  // unstamped
  m3d::TransformContainer data(false, false);
  EXPECT_EQ(data.size(), 0);

  data.append(std::make_shared<m3d::MatrixTransform>());
  data.append(std::make_shared<m3d::EulerTransform>(Eigen::Matrix<double, 3, 1>::Zero(), 0.0, 0.0, 0.0, m3d::EulerAxes::kSXYZ));
  ASSERT_EQ(data.size(), 2);
  EXPECT_TRUE(data.at(0)->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data.at(1)->isType(m3d::TransformType::kEuler));

  data.insert(1, std::make_shared<m3d::DualQuaternionTransform>());
  ASSERT_EQ(data.size(), 3);
  EXPECT_TRUE(data.at(1)->isType(m3d::TransformType::kDualQuaternion));

  // stamped
  m3d::TransformContainer data_stamped(true, false);
  EXPECT_EQ(data_stamped.size(), 0);

  data_stamped.insert(m3d::Time(0.0), std::make_shared<m3d::MatrixTransform>(), false);
  data_stamped.insert(m3d::Time(1.0), std::make_shared<m3d::EulerTransform>(Eigen::Matrix<double, 3, 1>::Zero(), 0.0, 0.0, 0.0, m3d::EulerAxes::kSXYZ), false);
  ASSERT_EQ(data_stamped.size(), 2);
  EXPECT_TRUE(data_stamped.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data_stamped.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kEuler));

  data_stamped.insert(m3d::Time(0.0), std::make_shared<m3d::DualQuaternionTransform>(), true);
  ASSERT_EQ(data_stamped.size(), 2);
  EXPECT_TRUE(data_stamped.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kDualQuaternion));

  data_stamped.append(m3d::Time(2.0), std::make_shared<m3d::MatrixTransform>());
  data_stamped.append(m3d::Time(0.5), std::make_shared<m3d::MatrixTransform>());
  ASSERT_EQ(data_stamped.size(), 4);

  // exception handling
  EXPECT_THROW(data.insert(4, std::make_shared<m3d::MatrixTransform>()), std::out_of_range);
  EXPECT_THROW(data.insert(m3d::Time(0.0), std::make_shared<m3d::MatrixTransform>()), m3d::TransformContainerException);
  EXPECT_THROW(data_stamped.insert(0, std::make_shared<m3d::MatrixTransform>()), m3d::TransformContainerException);
  EXPECT_THROW(data_stamped.append(std::make_shared<m3d::MatrixTransform>()), m3d::TransformContainerException);
}

TEST(TransformContainer, dataAccess)
{
  // create data
  m3d::TransformContainer data(false, false);
  data.append(std::make_shared<m3d::MatrixTransform>(kMatrix1Vec));
  data.append(std::make_shared<m3d::EulerTransform>(kEuler1Vec));
  data.append(std::make_shared<m3d::DualQuaternionTransform>(kDualQuaternion1Vec));
  m3d::TransformContainer data_stamped(true, false);
  data_stamped.insert(m3d::Time(0.0), std::make_shared<m3d::MatrixTransform>(kMatrix1Vec));
  data_stamped.insert(m3d::Time(1.0), std::make_shared<m3d::EulerTransform>(kEuler1Vec));
  data_stamped.insert(m3d::Time(2.0), std::make_shared<m3d::DualQuaternionTransform>(kDualQuaternion1Vec));

  m3d::TransformContainer::ConstPtr data_ptr = std::make_shared<m3d::TransformContainer>(data);
  m3d::TransformContainer::ConstPtr data_stamped_ptr = std::make_shared<m3d::TransformContainer>(data_stamped);

  // stamp check
  EXPECT_TRUE(data_stamped.hasStamp(m3d::Time(0.0)));
  EXPECT_TRUE(data_stamped.hasStamp(m3d::Time(1.0)));
  EXPECT_FALSE(data_stamped.hasStamp(m3d::Time(0.5)));

  EXPECT_THROW(data.hasStamp(m3d::Time(0.0)), m3d::TransformContainerException);

  // index access
  EXPECT_TRUE(data.at(0)->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data.at(1)->isType(m3d::TransformType::kEuler));

  EXPECT_TRUE(data_stamped.at(0)->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data_stamped.at(1)->isType(m3d::TransformType::kEuler));

  EXPECT_EQ(data_stamped.stamp_at(0), m3d::Time(0.0));
  EXPECT_EQ(data_stamped.stamp_at(1), m3d::Time(1.0));

  EXPECT_EQ(data_stamped.item_at(0).first, m3d::Time(0.0));
  EXPECT_TRUE(data_stamped.item_at(0).second->isType(m3d::TransformType::kMatrix));
  EXPECT_EQ(data_stamped.item_at(1).first, m3d::Time(1.0));
  EXPECT_TRUE(data_stamped.item_at(1).second->isType(m3d::TransformType::kEuler));

  EXPECT_THROW(data.at(3), std::out_of_range);
  EXPECT_THROW(data.item_at(0), m3d::TransformContainerException);
  EXPECT_THROW(data.item_at(3), m3d::TransformContainerException);
  EXPECT_THROW(data.stamp_at(0), m3d::TransformContainerException);
  EXPECT_THROW(data.stamp_at(3), m3d::TransformContainerException);
  EXPECT_THROW(data_stamped.at(3), std::out_of_range);
  EXPECT_THROW(data_stamped.stamp_at(3), std::out_of_range);
  EXPECT_THROW(data_stamped.item_at(3), std::out_of_range);

  // const index access
  EXPECT_TRUE(data_ptr->at(0)->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data_ptr->at(1)->isType(m3d::TransformType::kEuler));

  EXPECT_TRUE(data_stamped_ptr->at(0)->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data_stamped_ptr->at(1)->isType(m3d::TransformType::kEuler));

  EXPECT_EQ(data_stamped_ptr->stamp_at(0), m3d::Time(0.0));
  EXPECT_EQ(data_stamped_ptr->stamp_at(1), m3d::Time(1.0));

  EXPECT_EQ(data_stamped_ptr->item_at(0).first, m3d::Time(0.0));
  EXPECT_TRUE(data_stamped_ptr->item_at(0).second->isType(m3d::TransformType::kMatrix));
  EXPECT_EQ(data_stamped_ptr->item_at(1).first, m3d::Time(1.0));
  EXPECT_TRUE(data_stamped_ptr->item_at(1).second->isType(m3d::TransformType::kEuler));

  EXPECT_THROW(data_ptr->at(3), std::out_of_range);
  EXPECT_THROW(data_ptr->item_at(0), m3d::TransformContainerException);
  EXPECT_THROW(data_ptr->item_at(3), m3d::TransformContainerException);
  EXPECT_THROW(data_ptr->stamp_at(0), m3d::TransformContainerException);
  EXPECT_THROW(data_ptr->stamp_at(3), m3d::TransformContainerException);
  EXPECT_THROW(data_stamped_ptr->at(3), std::out_of_range);
  EXPECT_THROW(data_stamped_ptr->stamp_at(3), std::out_of_range);
  EXPECT_THROW(data_stamped_ptr->item_at(3), std::out_of_range);

  // stamp access
  EXPECT_TRUE(data_stamped.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kMatrix));

  EXPECT_THROW(data.at_stamp(m3d::Time(0.0)), m3d::TransformContainerException);
  EXPECT_THROW(data_stamped.at_stamp(m3d::Time(3.0)), std::out_of_range);

  // const stamp access
  EXPECT_TRUE(data_stamped_ptr->at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kMatrix));

  EXPECT_THROW(data_ptr->at_stamp(m3d::Time(0.0)), m3d::TransformContainerException);
  EXPECT_THROW(data_stamped_ptr->at_stamp(m3d::Time(3.0)), std::out_of_range);

  // inplace access
  EXPECT_TRUE(data.at(0)->scaleTranslation_(2.0));
  EXPECT_EQ(data.at(0)->translationNorm(), 2.0 * data.at(1)->translationNorm());

  EXPECT_TRUE(data_stamped.at(0)->scaleTranslation_(2.0));
  EXPECT_EQ(data_stamped.at(0)->translationNorm(), 2.0 * data_stamped.at(1)->translationNorm());

  // reference access
  data.at(0) = std::make_shared<m3d::AxisAngleTransform>();
  EXPECT_TRUE(data.at(0)->isType(m3d::TransformType::kAxisAngle));

  data_stamped.at(0) = std::make_shared<m3d::AxisAngleTransform>();
  EXPECT_TRUE(data_stamped.at(0)->isType(m3d::TransformType::kAxisAngle));

  // erase
  EXPECT_THROW(data.erase(m3d::Time(2.0)), m3d::TransformContainerException);
  EXPECT_THROW(data.erase(3), std::out_of_range);
  EXPECT_THROW(data_stamped.erase(3), std::out_of_range);

  data.erase(0);
  EXPECT_EQ(data.size(), 2);
  data.erase(data.begin());
  EXPECT_EQ(data.size(), 1);

  data_stamped.erase(m3d::Time(2.0));
  EXPECT_EQ(data_stamped.size(), 2);
  data_stamped.erase(0);
  EXPECT_EQ(data_stamped.size(), 1);
  data_stamped.erase(m3d::Time(1.0));
  EXPECT_EQ(data_stamped.size(), 0);
  data_stamped.insert(m3d::Time(0.0), std::make_shared<m3d::MatrixTransform>());
  data_stamped.erase(data_stamped.begin());
  EXPECT_EQ(data_stamped.size(), 0);
  data_stamped.insert(m3d::Time(0.0), std::make_shared<m3d::MatrixTransform>());

  // clear and empty
  data.clear();
  EXPECT_EQ(data.size(), 0);
  EXPECT_TRUE(data.empty());

  data_stamped.clear();
  EXPECT_EQ(data_stamped.size(), 0);
  EXPECT_TRUE(data_stamped.empty());
}

TEST(TransformContainer, iterator)
{
  // create containers
  m3d::TransformContainer data(false, false);
  data.append(std::make_shared<m3d::DualQuaternionTransform>(kDualQuaternion1Vec));
  data.append(std::make_shared<m3d::MatrixTransform>(kMatrix1Vec));
  auto data_copy = m3d::TransformContainer(data);

  m3d::TransformContainer data_stamped(true, false);
  data_stamped.insert(m3d::Time(0.0), std::make_shared<m3d::DualQuaternionTransform>(kDualQuaternion1Vec));
  data_stamped.insert(m3d::Time(1.0), std::make_shared<m3d::MatrixTransform>(kMatrix1Vec));

  // const iterator
  int counter = 0;
  for (auto it = data.cbegin(); it != data.cend(); ++it, ++counter)
  {
    // type
    switch (counter)
    {
      case 0: EXPECT_TRUE(it->isType(m3d::TransformType::kDualQuaternion)); break;
      case 1: EXPECT_TRUE(it->isType(m3d::TransformType::kMatrix)); break;
      default: FAIL();
    }
  }

  counter = 0;
  for (auto it = data_stamped.cbegin(); it != data_stamped.cend(); ++it, ++counter)
  {
    // type
    switch (counter)
    {
      case 0: EXPECT_TRUE(it->isType(m3d::TransformType::kDualQuaternion)); break;
      case 1: EXPECT_TRUE(it->isType(m3d::TransformType::kMatrix)); break;
      default: FAIL();
    }
  }

  // iterator
  counter = 0;
  for (auto it = data.begin(); it != data.end(); ++it, ++counter)
  {
    // inplace
    it->scaleTranslation_(2.0);
    EXPECT_EQ(data.at(counter)->translationNorm(), 2.0 * data_copy.at(counter)->translationNorm());

    // type
    switch (counter)
    {
      case 0: EXPECT_TRUE(it->isType(m3d::TransformType::kDualQuaternion)); break;
      case 1: EXPECT_TRUE(it->isType(m3d::TransformType::kMatrix)); break;
      default: FAIL();
    }
  }

  counter = 0;
  for (auto it = data_stamped.begin(); it != data_stamped.end(); ++it, ++counter)
  {
    // inplace
    it->scaleTranslation_(2.0);
    EXPECT_EQ(data_stamped.at(counter)->translationNorm(), 2.0 * data_copy.at(counter)->translationNorm());

    // type
    switch (counter)
    {
      case 0: EXPECT_TRUE(it->isType(m3d::TransformType::kDualQuaternion)); break;
      case 1: EXPECT_TRUE(it->isType(m3d::TransformType::kMatrix)); break;
      default: FAIL();
    }
  }

  // stamped iterators
  counter = 0;
  for (auto it = data_stamped.cbegin_stamps(); it != data_stamped.cend_stamps(); ++it, ++counter)
  {
    switch (counter)
    {
      case 0: EXPECT_EQ(*it, m3d::Time(0.0)); break;
      case 1: EXPECT_EQ(*it, m3d::Time(1.0)); break;
      default: FAIL();
    }
  }

  counter = 0;
  for (auto it = data_stamped.begin_items(); it != data_stamped.end_items(); ++it, ++counter)
  {
    // inplace
    it->second->scaleTranslation_(2.0);
    EXPECT_EQ(data_stamped.at(counter)->translationNorm(), 4.0 * data_copy.at(counter)->translationNorm());

    //type
    switch (counter)
    {
      case 0:
        EXPECT_EQ(it->first, m3d::Time(0.0));
        EXPECT_TRUE(it->second->isType(m3d::TransformType::kDualQuaternion));
        break;
      case 1:
        EXPECT_EQ(it->first, m3d::Time(1.0));
        EXPECT_TRUE(it->second->isType(m3d::TransformType::kMatrix));
        break;
      default: FAIL();
    }
  }

  counter = 0;
  for (auto it = data_stamped.cbegin_items(); it != data_stamped.cend_items(); ++it, ++counter)
  {
    switch (counter)
    {
      case 0:
        EXPECT_EQ(it->first, m3d::Time(0.0));
        EXPECT_TRUE(it->second->isType(m3d::TransformType::kDualQuaternion));
        break;
      case 1:
        EXPECT_EQ(it->first, m3d::Time(1.0));
        EXPECT_TRUE(it->second->isType(m3d::TransformType::kMatrix));
        break;
      default: FAIL();
    }
  }

  // non-const reference access
  counter = 0;
  for (auto it = data.begin(); it != data.end(); ++it, ++counter)
  {
    *it = std::make_shared<m3d::AxisAngleTransform>();
    EXPECT_TRUE(data.at(counter)->isType(m3d::TransformType::kAxisAngle));
  }

  counter = 0;
  for (auto it = data_stamped.begin(); it != data_stamped.end(); ++it, ++counter)
  {
    *it = std::make_shared<m3d::AxisAngleTransform>();
    EXPECT_TRUE(data.at(counter)->isType(m3d::TransformType::kAxisAngle));
  }

  counter = 0;
  for (auto it = data_stamped.begin_items(); it != data_stamped.end_items(); ++it, ++counter)
  {
    it->second = std::make_shared<m3d::DualQuaternionTransform>();
    EXPECT_TRUE(data_stamped.at(counter)->isType(m3d::TransformType::kDualQuaternion));
  }

  // comparison
  EXPECT_FALSE(data.begin() == data_stamped.begin());
  EXPECT_TRUE(data.begin() != data_stamped.begin());
  EXPECT_FALSE(data.cbegin() == data_stamped.cbegin());
  EXPECT_TRUE(data.cbegin() != data_stamped.cbegin());

  // exceptions
  EXPECT_THROW(data.begin_items(), m3d::TransformContainerException);
  EXPECT_THROW(data.end_items(), m3d::TransformContainerException);
  EXPECT_THROW(data.cbegin_stamps(), m3d::TransformContainerException);
  EXPECT_THROW(data.cend_stamps(), m3d::TransformContainerException);
}

TEST(TransformContainer, find)
{
  // create unstamped
  std::vector<m3d::TransformInterface::Ptr> transforms = {
    std::make_shared<m3d::DualQuaternionTransform>(),
    std::make_shared<m3d::MatrixTransform>()
  };
  m3d::TransformContainer container(false, false);
  for (auto transform : transforms)
  {
    container.append(transform);
  }
  m3d::TransformContainer::ConstPtr container_const = std::make_shared<m3d::TransformContainer>(container);

  // create stamped
  m3d::ExtendedMap<m3d::Time, m3d::TransformInterface::Ptr> transforms_stamped;
  transforms_stamped[m3d::Time(2.0)] = std::make_shared<m3d::MatrixTransform>();
  transforms_stamped[m3d::Time(1.0)] = std::make_shared<m3d::DualQuaternionTransform>();
  transforms_stamped[m3d::Time(3.0)] = std::make_shared<m3d::EulerTransform>();
  m3d::TransformContainer container_stamped(true, false);
  for (auto transform : transforms_stamped)
  {
    container_stamped.insert(transform.first, transform.second);
  }
  m3d::TransformContainer::ConstPtr container_stamped_const = std::make_shared<m3d::TransformContainer>(container_stamped);

  // create empty stamped
  m3d::TransformContainer container_stamped_empty(true, false);
  m3d::TransformContainer::ConstPtr container_stamped_empty_const = std::make_shared<m3d::TransformContainer>(container_stamped_empty);

  // unstamped
  EXPECT_THROW(container.find_eq(m3d::Time(0.0)), m3d::TransformContainerException);
  EXPECT_THROW(container.find_ge(m3d::Time(0.0)), m3d::TransformContainerException);
  EXPECT_THROW(container.find_gt(m3d::Time(0.0)), m3d::TransformContainerException);
  EXPECT_THROW(container.find_le(m3d::Time(0.0)), m3d::TransformContainerException);
  EXPECT_THROW(container.find_lt(m3d::Time(0.0)), m3d::TransformContainerException);
  EXPECT_THROW(container.find_closest(m3d::Time(0.0)), m3d::TransformContainerException);
  EXPECT_THROW(container_const->find_eq(m3d::Time(0.0)), m3d::TransformContainerException);
  EXPECT_THROW(container_const->find_ge(m3d::Time(0.0)), m3d::TransformContainerException);
  EXPECT_THROW(container_const->find_gt(m3d::Time(0.0)), m3d::TransformContainerException);
  EXPECT_THROW(container_const->find_le(m3d::Time(0.0)), m3d::TransformContainerException);
  EXPECT_THROW(container_const->find_lt(m3d::Time(0.0)), m3d::TransformContainerException);
  EXPECT_THROW(container_const->find_closest(m3d::Time(0.0)), m3d::TransformContainerException);

  // find equal
  auto it = container_stamped.find_eq(m3d::Time(1.0));
  ASSERT_FALSE(it == container_stamped.end_items());
  ASSERT_EQ(it->first, m3d::Time(1.0));

  it = container_stamped.find_eq(m3d::Time(1.5));
  ASSERT_TRUE(it == container_stamped.end_items());

  // find greater equal
  it = container_stamped.find_ge(m3d::Time(1.5));
  ASSERT_FALSE(it == container_stamped.end_items());
  ASSERT_EQ(it->first, m3d::Time(2.0));

  it = container_stamped.find_ge(m3d::Time(3.0));
  ASSERT_FALSE(it == container_stamped.end_items());
  ASSERT_EQ(it->first, m3d::Time(3.0));

  it = container_stamped.find_ge(m3d::Time(3.5));
  ASSERT_TRUE(it == container_stamped.end_items());

  // find greater than
  it = container_stamped.find_gt(m3d::Time(1.5));
  ASSERT_FALSE(it == container_stamped.end_items());
  ASSERT_EQ(it->first, m3d::Time(2.0));

  it = container_stamped.find_gt(m3d::Time(3.0));
  ASSERT_TRUE(it == container_stamped.end_items());

  it = container_stamped.find_gt(m3d::Time(3.5));
  ASSERT_TRUE(it == container_stamped.end_items());

  // find lower equal
  it = container_stamped.find_le(m3d::Time(1.5));
  ASSERT_FALSE(it == container_stamped.end_items());
  ASSERT_EQ(it->first, m3d::Time(1.0));

  it = container_stamped.find_le(m3d::Time(1.0));
  ASSERT_FALSE(it == container_stamped.end_items());
  ASSERT_EQ(it->first, m3d::Time(1.0));

  it = container_stamped.find_le(m3d::Time(0.5));
  ASSERT_TRUE(it == container_stamped.end_items());

  // find lower than
  it = container_stamped.find_lt(m3d::Time(1.5));
  ASSERT_FALSE(it == container_stamped.end_items());
  ASSERT_EQ(it->first, m3d::Time(1.0));

  it = container_stamped.find_lt(m3d::Time(1.0));
  ASSERT_TRUE(it == container_stamped.end_items());

  it = container_stamped.find_lt(m3d::Time(0.5));
  ASSERT_TRUE(it == container_stamped.end_items());

  // find closest
  it = container_stamped.find_closest(m3d::Time(0.0));
  ASSERT_FALSE(it == container_stamped.end_items());
  ASSERT_EQ(it->first, m3d::Time(1.0));

  it = container_stamped.find_closest(m3d::Time(1.4));
  ASSERT_FALSE(it == container_stamped.end_items());
  ASSERT_EQ(it->first, m3d::Time(1.0));

  it = container_stamped.find_closest(m3d::Time(1.5));
  ASSERT_FALSE(it == container_stamped.end_items());
  ASSERT_EQ(it->first, m3d::Time(1.0));

  it = container_stamped.find_closest(m3d::Time(1.6));
  ASSERT_FALSE(it == container_stamped.end_items());
  ASSERT_EQ(it->first, m3d::Time(2.0));

  it = container_stamped.find_closest(m3d::Time(2.0));
  ASSERT_FALSE(it == container_stamped.end_items());
  ASSERT_EQ(it->first, m3d::Time(2.0));

  it = container_stamped.find_closest(m3d::Time(5.0));
  ASSERT_FALSE(it == container_stamped.end_items());
  ASSERT_EQ(it->first, m3d::Time(3.0));

  // find empty
  ASSERT_TRUE(container_stamped_empty.find_eq(m3d::Time(0.0)) == container_stamped_empty.end_items());
  ASSERT_TRUE(container_stamped_empty.find_ge(m3d::Time(0.0)) == container_stamped_empty.end_items());
  ASSERT_TRUE(container_stamped_empty.find_gt(m3d::Time(0.0)) == container_stamped_empty.end_items());
  ASSERT_TRUE(container_stamped_empty.find_le(m3d::Time(0.0)) == container_stamped_empty.end_items());
  ASSERT_TRUE(container_stamped_empty.find_lt(m3d::Time(0.0)) == container_stamped_empty.end_items());
  ASSERT_TRUE(container_stamped_empty.find_closest(m3d::Time(0.0)) == container_stamped_empty.end_items());

  // find const equal
  auto const_it = container_stamped_const->find_eq(m3d::Time(1.0));
  ASSERT_FALSE(const_it == container_stamped_const->cend_items());
  ASSERT_EQ(const_it->first, m3d::Time(1.0));

  const_it = container_stamped_const->find_eq(m3d::Time(1.5));
  ASSERT_TRUE(const_it == container_stamped_const->cend_items());

  // find const greater equal
  const_it = container_stamped_const->find_ge(m3d::Time(1.5));
  ASSERT_FALSE(const_it == container_stamped_const->cend_items());
  ASSERT_EQ(const_it->first, m3d::Time(2.0));

  const_it = container_stamped_const->find_ge(m3d::Time(3.0));
  ASSERT_FALSE(const_it == container_stamped_const->cend_items());
  ASSERT_EQ(const_it->first, m3d::Time(3.0));

  const_it = container_stamped_const->find_ge(m3d::Time(3.5));
  ASSERT_TRUE(const_it == container_stamped_const->cend_items());

  // find const greater than
  const_it = container_stamped_const->find_gt(m3d::Time(1.5));
  ASSERT_FALSE(const_it == container_stamped_const->cend_items());
  ASSERT_EQ(const_it->first, m3d::Time(2.0));

  const_it = container_stamped_const->find_gt(m3d::Time(3.0));
  ASSERT_TRUE(const_it == container_stamped_const->cend_items());

  const_it = container_stamped_const->find_gt(m3d::Time(3.5));
  ASSERT_TRUE(const_it == container_stamped_const->cend_items());

  // find const lower equal
  const_it = container_stamped_const->find_le(m3d::Time(1.5));
  ASSERT_FALSE(const_it == container_stamped_const->cend_items());
  ASSERT_EQ(const_it->first, m3d::Time(1.0));

  const_it = container_stamped_const->find_le(m3d::Time(1.0));
  ASSERT_FALSE(const_it == container_stamped_const->cend_items());
  ASSERT_EQ(const_it->first, m3d::Time(1.0));

  const_it = container_stamped_const->find_le(m3d::Time(0.5));
  ASSERT_TRUE(const_it == container_stamped_const->cend_items());

  // find const lower than
  const_it = container_stamped_const->find_lt(m3d::Time(1.5));
  ASSERT_FALSE(const_it == container_stamped_const->cend_items());
  ASSERT_EQ(const_it->first, m3d::Time(1.0));

  const_it = container_stamped_const->find_lt(m3d::Time(1.0));
  ASSERT_TRUE(const_it == container_stamped_const->cend_items());

  const_it = container_stamped_const->find_lt(m3d::Time(0.5));
  ASSERT_TRUE(const_it == container_stamped_const->cend_items());

  // find const closest
  const_it = container_stamped_const->find_closest(m3d::Time(0.0));
  ASSERT_FALSE(const_it == container_stamped_const->cend_items());
  ASSERT_EQ(const_it->first, m3d::Time(1.0));

  const_it = container_stamped_const->find_closest(m3d::Time(1.4));
  ASSERT_FALSE(const_it == container_stamped_const->cend_items());
  ASSERT_EQ(const_it->first, m3d::Time(1.0));

  const_it = container_stamped_const->find_closest(m3d::Time(1.5));
  ASSERT_FALSE(const_it == container_stamped_const->cend_items());
  ASSERT_EQ(const_it->first, m3d::Time(1.0));

  const_it = container_stamped_const->find_closest(m3d::Time(1.6));
  ASSERT_FALSE(const_it == container_stamped_const->cend_items());
  ASSERT_EQ(const_it->first, m3d::Time(2.0));

  const_it = container_stamped_const->find_closest(m3d::Time(2.0));
  ASSERT_FALSE(const_it == container_stamped_const->cend_items());
  ASSERT_EQ(const_it->first, m3d::Time(2.0));

  const_it = container_stamped_const->find_closest(m3d::Time(5.0));
  ASSERT_FALSE(const_it == container_stamped_const->cend_items());
  ASSERT_EQ(const_it->first, m3d::Time(3.0));

  // find const empty
  ASSERT_TRUE(container_stamped_empty_const->find_eq(m3d::Time(0.0)) == container_stamped_empty_const->cend_items());
  ASSERT_TRUE(container_stamped_empty_const->find_ge(m3d::Time(0.0)) == container_stamped_empty_const->cend_items());
  ASSERT_TRUE(container_stamped_empty_const->find_gt(m3d::Time(0.0)) == container_stamped_empty_const->cend_items());
  ASSERT_TRUE(container_stamped_empty_const->find_le(m3d::Time(0.0)) == container_stamped_empty_const->cend_items());
  ASSERT_TRUE(container_stamped_empty_const->find_lt(m3d::Time(0.0)) == container_stamped_empty_const->cend_items());
  ASSERT_TRUE(container_stamped_empty_const->find_closest(m3d::Time(0.0)) == container_stamped_empty_const->cend_items());
}

TEST(TransformContainer, extend)
{
  // create containers
  m3d::TransformContainer data1(false, false);
  data1.append(std::make_shared<m3d::AxisAngleTransform>());
  data1.append(std::make_shared<m3d::DualQuaternionTransform>());

  m3d::TransformContainer data2(false, false);
  data2.append(std::make_shared<m3d::EulerTransform>());
  data2.append(std::make_shared<m3d::MatrixTransform>());
  data2.append(std::make_shared<m3d::QuaternionTransform>());

  m3d::TransformContainer data_stamped1(true, false);
  data_stamped1.insert(m3d::Time(2.0), std::make_shared<m3d::DualQuaternionTransform>());
  data_stamped1.insert(m3d::Time(1.0), std::make_shared<m3d::AxisAngleTransform>());

  m3d::TransformContainer data_stamped2(true, false);
  data_stamped2.insert(m3d::Time(0.0), std::make_shared<m3d::EulerTransform>());
  data_stamped2.insert(m3d::Time(3.0), std::make_shared<m3d::MatrixTransform>());
  data_stamped2.insert(m3d::Time(4.0), std::make_shared<m3d::QuaternionTransform>());

  m3d::TransformContainer data_stamped3(true, false);
  data_stamped3.insert(m3d::Time(1.0), std::make_shared<m3d::EulerTransform>());
  data_stamped3.insert(m3d::Time(0.0), std::make_shared<m3d::MatrixTransform>());

  std::map<m3d::Time, m3d::TransformInterface::Ptr> transforms_map;
  transforms_map[m3d::Time(5.0)] = std::make_shared<m3d::MatrixTransform>();
  transforms_map[m3d::Time(6.0)] = std::make_shared<m3d::DualQuaternionTransform>();

  // unstamped
  data1.extend(data2);
  ASSERT_EQ(data1.size(), 5);
  EXPECT_TRUE(data1.at(0)->isType(m3d::TransformType::kAxisAngle));
  EXPECT_TRUE(data1.at(1)->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data1.at(2)->isType(m3d::TransformType::kEuler));
  EXPECT_TRUE(data1.at(3)->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data1.at(4)->isType(m3d::TransformType::kQuaternion));

  // unstamped exceptions
  EXPECT_THROW(data1.extend(transforms_map), m3d::TransformContainerException);

  // stamped
  m3d::TransformContainer data_stamped_ext1(data_stamped1);
  data_stamped_ext1.extend(data_stamped2, false);
  ASSERT_EQ(data_stamped_ext1.size(), 5);
  EXPECT_TRUE(data_stamped_ext1.at_stamp(m3d::Time(0.0))->isType(m3d::TransformType::kEuler));
  EXPECT_TRUE(data_stamped_ext1.at_stamp(m3d::Time(1.0))->isType(m3d::TransformType::kAxisAngle));
  EXPECT_TRUE(data_stamped_ext1.at_stamp(m3d::Time(2.0))->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_stamped_ext1.at_stamp(m3d::Time(3.0))->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data_stamped_ext1.at_stamp(m3d::Time(4.0))->isType(m3d::TransformType::kQuaternion));

  // stamped non-overwrite
  m3d::TransformContainer data_stamped_ext2(data_stamped1);
  data_stamped_ext2.extend(data_stamped3, false);
  ASSERT_EQ(data_stamped_ext2.size(), 3);
  EXPECT_TRUE(data_stamped_ext2.at(0)->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data_stamped_ext2.at(1)->isType(m3d::TransformType::kAxisAngle));
  EXPECT_TRUE(data_stamped_ext2.at(2)->isType(m3d::TransformType::kDualQuaternion));

  // stamped overwrite
  m3d::TransformContainer data_stamped_ext3(data_stamped1);
  data_stamped_ext3.extend(data_stamped3, true);
  ASSERT_EQ(data_stamped_ext3.size(), 3);
  EXPECT_TRUE(data_stamped_ext3.at(0)->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data_stamped_ext3.at(1)->isType(m3d::TransformType::kEuler));
  EXPECT_TRUE(data_stamped_ext3.at(2)->isType(m3d::TransformType::kDualQuaternion));

  // mixed motions and poses
  m3d::TransformContainer data_poses(false, true);
  EXPECT_NO_THROW(data_poses.extend(data_poses));
  EXPECT_THROW(data1.extend(data_poses), m3d::TransformContainerException);
  EXPECT_THROW(data_poses.extend(data1), m3d::TransformContainerException);

  m3d::TransformContainer data_poses_stamped(false, true);
  EXPECT_NO_THROW(data_poses_stamped.extend(data_poses_stamped));
  EXPECT_THROW(data_stamped1.extend(data_poses_stamped), m3d::TransformContainerException);
  EXPECT_THROW(data_poses_stamped.extend(data_stamped1), m3d::TransformContainerException);

  // mixed stamped and unstamped
  EXPECT_THROW(data1.extend(data_stamped1), m3d::TransformContainerException);
  EXPECT_THROW(data_stamped1.extend(data1), m3d::TransformContainerException);
}

TEST(TransformContainer, transformPtrCopy)
{
  // create data
  m3d::TransformInterface::Ptr transform1 = std::make_shared<m3d::MatrixTransform>(kMatrix1Vec);

  std::vector<m3d::TransformInterface::Ptr> transforms1 = {std::make_shared<m3d::MatrixTransform>(kMatrix1Vec)};
  std::vector<m3d::TransformInterface::Ptr> transforms2 = {std::make_shared<m3d::MatrixTransform>(kMatrix1Vec)};

  std::vector<m3d::Time> stamps1 = {m3d::Time(2.0)};

  std::map<m3d::Time, m3d::TransformInterface::Ptr> transforms_stamped1;
  transforms_stamped1[m3d::Time(0.0)] = std::make_shared<m3d::MatrixTransform>(kMatrix1Vec);
  std::map<m3d::Time, m3d::TransformInterface::Ptr> transforms_stamped2;
  transforms_stamped2[m3d::Time(1.0)] = std::make_shared<m3d::MatrixTransform>(kMatrix1Vec);

  double translation_norm = transform1->translationNorm();

  // containers
  m3d::TransformContainer data1(transforms1, false);
  data1.scaleTranslation_(2.0);
  EXPECT_EQ(transforms1.at(0)->translationNorm(), translation_norm);

  m3d::TransformContainer data_stamped1(transforms_stamped1, false);
  data_stamped1.scaleTranslation_(2.0);
  EXPECT_EQ(transforms_stamped1[m3d::Time(0.0)]->translationNorm(), translation_norm);

  m3d::TransformContainer data_stamped2(stamps1, transforms1, false);
  data_stamped2.scaleTranslation_(2.0);
  EXPECT_EQ(transforms1.at(0)->translationNorm(), translation_norm);

  // copy constructor
  m3d::TransformContainer data_copy1(data1);
  data_copy1.scaleTranslation_(2.0);
  EXPECT_EQ(data1.at(0)->translationNorm(), 2.0 * translation_norm);

  m3d::TransformContainer data_stamped_copy1(data_stamped1);
  data_stamped_copy1.scaleTranslation_(2.0);
  EXPECT_EQ(data_stamped1.at(0)->translationNorm(), 2.0 * translation_norm);

  // extend
  data1.extend(transforms2);
  data1.scaleTranslation_(2.0);
  EXPECT_EQ(transforms2.at(0)->translationNorm(), translation_norm);

  data_stamped1.extend(transforms_stamped2, false);
  data_stamped1.scaleTranslation_(2.0);
  EXPECT_EQ(transforms_stamped2[m3d::Time(1.0)]->translationNorm(), translation_norm);

  data_stamped1.extend(transforms_stamped2, true);
  data_stamped1.scaleTranslation_(2.0);
  EXPECT_EQ(transforms_stamped2[m3d::Time(1.0)]->translationNorm(), translation_norm);

  data1.extend(data_copy1);
  data1.scaleTranslation_(2.0);
  EXPECT_EQ(data_copy1.at(0)->translationNorm(), 4.0 * translation_norm);

  data_stamped1.extend(data_stamped2, false);
  data_stamped1.scaleTranslation_(2.0);
  EXPECT_EQ(data_stamped2.at(0)->translationNorm(), 2.0 * translation_norm);

  data_stamped1.extend(data_stamped2, true);
  data_stamped1.scaleTranslation_(2.0);
  EXPECT_EQ(data_stamped2.at(0)->translationNorm(), 2.0 * translation_norm);

  // append
  data1.append(transform1);
  data1.scaleTranslation_(2.0);
  EXPECT_EQ(transform1->translationNorm(), translation_norm);

  data_stamped1.append(m3d::Time(3.0), transform1);
  data_stamped1.scaleTranslation_(2.0);
  EXPECT_EQ(transform1->translationNorm(), translation_norm);

  // insert
  data1.insert(0, transform1);
  data1.scaleTranslation_(2.0);
  EXPECT_EQ(transform1->translationNorm(), translation_norm);

  data_stamped1.insert(m3d::Time(4.0), transform1, false);
  data_stamped1.scaleTranslation_(2.0);
  EXPECT_EQ(transform1->translationNorm(), translation_norm);

  data_stamped1.insert(m3d::Time(4.0), transform1, true);
  data_stamped1.scaleTranslation_(2.0);
  EXPECT_EQ(transform1->translationNorm(), translation_norm);
}

TEST(TransformContainer, inplaceReturn)
{
  // create container and transform
  m3d::TransformContainer data(false, false);
  data.append(std::make_shared<m3d::AxisAngleTransform>());
  data.append(std::make_shared<m3d::AxisAngleTransform>());
  auto transform = std::make_shared<m3d::AxisAngleTransform>();

  // check return value
  ASSERT_TRUE(&data == &data.asType_<m3d::DualQuaternionTransform>());
  ASSERT_TRUE(&data == &data.asType_(m3d::TransformType::kEuler));
  ASSERT_TRUE(&data == &data.asPoses_());
  ASSERT_TRUE(&data == &data.asPoses_(transform));
  ASSERT_TRUE(&data == &data.asMotions_());
  ASSERT_TRUE(&data == &data.inverse_());
  ASSERT_TRUE(&data == &data.normalized_());
  ASSERT_TRUE(&data == &data.scaleTranslation_(1.0));
  ASSERT_TRUE(&data == &data.applyPre_(transform));
  ASSERT_TRUE(&data == &data.applyPost_(transform));
  ASSERT_TRUE(&data == &data.apply_(transform, transform));
  ASSERT_TRUE(&data == &data.applyFunc_(trafoFunc));
  ASSERT_TRUE(&data == &data.applyIndexFunc_(indexFunc));
  ASSERT_TRUE(&data == &data.changeFrame_(transform));
  ASSERT_TRUE(&data == &data.addStamps_({m3d::Time(0.0), m3d::Time(1.0)}));
  ASSERT_TRUE(&data == &data.applyStampFunc_(stampFunc));
  ASSERT_TRUE(&data == &data.removeStamps_());
}

TEST(TransformContainer, asType)
{
  // create container
  m3d::TransformContainer data(false, false);
  data.append(std::make_shared<m3d::MatrixTransform>());
  data.append(std::make_shared<m3d::DualQuaternionTransform>());

  // inplace
  data.asType_<m3d::AxisAngleTransform>();
  EXPECT_TRUE(data.at(0)->isType(m3d::TransformType::kAxisAngle));
  EXPECT_TRUE(data.at(1)->isType(m3d::TransformType::kAxisAngle));
  data.asType_(m3d::TransformType::kAxisAngle);
  EXPECT_TRUE(data.at(0)->isType(m3d::TransformType::kAxisAngle));
  EXPECT_TRUE(data.at(1)->isType(m3d::TransformType::kAxisAngle));

  data.asType_<m3d::DualQuaternionTransform>();
  EXPECT_TRUE(data.at(0)->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data.at(1)->isType(m3d::TransformType::kDualQuaternion));
  data.asType_(m3d::TransformType::kDualQuaternion);
  EXPECT_TRUE(data.at(0)->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data.at(1)->isType(m3d::TransformType::kDualQuaternion));

  data.asType_<m3d::EulerTransform>();
  EXPECT_TRUE(data.at(0)->isType(m3d::TransformType::kEuler));
  EXPECT_TRUE(data.at(1)->isType(m3d::TransformType::kEuler));
  data.asType_(m3d::TransformType::kEuler);
  EXPECT_TRUE(data.at(0)->isType(m3d::TransformType::kEuler));
  EXPECT_TRUE(data.at(1)->isType(m3d::TransformType::kEuler));

  data.asType_<m3d::MatrixTransform>();
  EXPECT_TRUE(data.at(0)->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data.at(1)->isType(m3d::TransformType::kMatrix));
  data.asType_(m3d::TransformType::kMatrix);
  EXPECT_TRUE(data.at(0)->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data.at(1)->isType(m3d::TransformType::kMatrix));

  data.asType_<m3d::QuaternionTransform>();
  EXPECT_TRUE(data.at(0)->isType(m3d::TransformType::kQuaternion));
  EXPECT_TRUE(data.at(1)->isType(m3d::TransformType::kQuaternion));
  data.asType_(m3d::TransformType::kQuaternion);
  EXPECT_TRUE(data.at(0)->isType(m3d::TransformType::kQuaternion));
  EXPECT_TRUE(data.at(1)->isType(m3d::TransformType::kQuaternion));

  // non-inplace
  data.clear();
  data.append(std::make_shared<m3d::MatrixTransform>());
  data.append(std::make_shared<m3d::DualQuaternionTransform>());

  m3d::TransformContainer data_copy = data.asType<m3d::AxisAngleTransform>();
  EXPECT_TRUE(data_copy.at(0)->isType(m3d::TransformType::kAxisAngle));
  EXPECT_TRUE(data_copy.at(1)->isType(m3d::TransformType::kAxisAngle));
  EXPECT_TRUE(data.at(0)->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data.at(1)->isType(m3d::TransformType::kDualQuaternion));

  data_copy = data.asType(m3d::TransformType::kAxisAngle);
  EXPECT_TRUE(data_copy.at(0)->isType(m3d::TransformType::kAxisAngle));
  EXPECT_TRUE(data_copy.at(1)->isType(m3d::TransformType::kAxisAngle));
  EXPECT_TRUE(data.at(0)->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data.at(1)->isType(m3d::TransformType::kDualQuaternion));

  data_copy = data.asType<m3d::DualQuaternionTransform>();
  EXPECT_TRUE(data_copy.at(0)->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_copy.at(1)->isType(m3d::TransformType::kDualQuaternion));
  data_copy = data.asType(m3d::TransformType::kDualQuaternion);
  EXPECT_TRUE(data_copy.at(0)->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data_copy.at(1)->isType(m3d::TransformType::kDualQuaternion));

  data_copy = data.asType<m3d::EulerTransform>();
  EXPECT_TRUE(data_copy.at(0)->isType(m3d::TransformType::kEuler));
  EXPECT_TRUE(data_copy.at(1)->isType(m3d::TransformType::kEuler));
  data_copy = data.asType(m3d::TransformType::kEuler);
  EXPECT_TRUE(data_copy.at(0)->isType(m3d::TransformType::kEuler));
  EXPECT_TRUE(data_copy.at(1)->isType(m3d::TransformType::kEuler));

  data_copy = data.asType<m3d::MatrixTransform>();
  EXPECT_TRUE(data_copy.at(0)->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data_copy.at(1)->isType(m3d::TransformType::kMatrix));
  data_copy = data.asType(m3d::TransformType::kMatrix);
  EXPECT_TRUE(data_copy.at(0)->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data_copy.at(1)->isType(m3d::TransformType::kMatrix));

  data_copy = data.asType<m3d::QuaternionTransform>();
  EXPECT_TRUE(data_copy.at(0)->isType(m3d::TransformType::kQuaternion));
  EXPECT_TRUE(data_copy.at(1)->isType(m3d::TransformType::kQuaternion));
  data_copy = data.asType(m3d::TransformType::kQuaternion);
  EXPECT_TRUE(data_copy.at(0)->isType(m3d::TransformType::kQuaternion));
  EXPECT_TRUE(data_copy.at(1)->isType(m3d::TransformType::kQuaternion));
}

TEST(TransformContainer, motionsAndPosesData)
{
  // rotation and translation
  Eigen::Matrix<double, 3, 3> rotation_matrix;
  rotation_matrix.setIdentity();

  Eigen::Matrix<double, 3, 1> translation1;
  Eigen::Matrix<double, 3, 1> translation2;
  Eigen::Matrix<double, 3, 1> translation3;
  translation1 << 1.0, 0.0, 0.0;
  translation2 << 3.0, 0.0, 0.0;
  translation3 << 5.0, 0.0, 0.0;

  // transform container with motions
  m3d::TransformContainer data(false, false);
  data.append(std::make_shared<m3d::MatrixTransform>(translation1, rotation_matrix));
  data.append(std::make_shared<m3d::MatrixTransform>(translation2, rotation_matrix));
  data.append(std::make_shared<m3d::MatrixTransform>(translation3, rotation_matrix));

  // check if motions
  EXPECT_TRUE(data.hasMotions());
  EXPECT_FALSE(data.hasPoses());
  EXPECT_EQ(data.size(), 3);
  EXPECT_EQ(data.at(0)->asType<m3d::MatrixTransform>()->getTranslation()(0), 1.0);
  EXPECT_EQ(data.at(1)->asType<m3d::MatrixTransform>()->getTranslation()(0), 3.0);
  EXPECT_EQ(data.at(2)->asType<m3d::MatrixTransform>()->getTranslation()(0), 5.0);

  // again as motions
  data.asMotions_();
  EXPECT_TRUE(data.hasMotions());
  EXPECT_FALSE(data.hasPoses());
  EXPECT_EQ(data.size(), 3);
  EXPECT_EQ(data.at(0)->asType<m3d::MatrixTransform>()->getTranslation()(0), 1.0);
  EXPECT_EQ(data.at(1)->asType<m3d::MatrixTransform>()->getTranslation()(0), 3.0);
  EXPECT_EQ(data.at(2)->asType<m3d::MatrixTransform>()->getTranslation()(0), 5.0);

  // poses
  data.asPoses_();
  EXPECT_FALSE(data.hasMotions());
  EXPECT_TRUE(data.hasPoses());
  EXPECT_EQ(data.size(), 4);
  EXPECT_EQ(data.at(0)->asType<m3d::MatrixTransform>()->getTranslation()(0), 0.0);
  EXPECT_EQ(data.at(1)->asType<m3d::MatrixTransform>()->getTranslation()(0), 1.0);
  EXPECT_EQ(data.at(2)->asType<m3d::MatrixTransform>()->getTranslation()(0), 4.0);
  EXPECT_EQ(data.at(3)->asType<m3d::MatrixTransform>()->getTranslation()(0), 9.0);

  // again as poses
  data.asPoses_();
  EXPECT_FALSE(data.hasMotions());
  EXPECT_TRUE(data.hasPoses());
  EXPECT_EQ(data.size(), 4);
  EXPECT_EQ(data.at(0)->asType<m3d::MatrixTransform>()->getTranslation()(0), 0.0);
  EXPECT_EQ(data.at(1)->asType<m3d::MatrixTransform>()->getTranslation()(0), 1.0);
  EXPECT_EQ(data.at(2)->asType<m3d::MatrixTransform>()->getTranslation()(0), 4.0);
  EXPECT_EQ(data.at(3)->asType<m3d::MatrixTransform>()->getTranslation()(0), 9.0);

  // back to motions
  data.asMotions_();
  EXPECT_TRUE(data.hasMotions());
  EXPECT_FALSE(data.hasPoses());
  EXPECT_EQ(data.size(), 3);
  EXPECT_EQ(data.at(0)->asType<m3d::MatrixTransform>()->getTranslation()(0), 1.0);
  EXPECT_EQ(data.at(1)->asType<m3d::MatrixTransform>()->getTranslation()(0), 3.0);
  EXPECT_EQ(data.at(2)->asType<m3d::MatrixTransform>()->getTranslation()(0), 5.0);

  // non-inplace
  EXPECT_TRUE(data.asMotions().hasMotions());
  auto data_poses = data.asPoses(); 
  EXPECT_TRUE(data_poses.hasPoses());
  EXPECT_TRUE(data.hasMotions());

  EXPECT_TRUE(data_poses.asPoses().hasPoses());
  EXPECT_TRUE(data_poses.asMotions().hasMotions());
  EXPECT_TRUE(data_poses.hasPoses());
}

TEST(TransformContainer, motionsAndPosesTypes)
{
  // create container
  m3d::TransformContainer data(false, false);
  data.append(std::make_shared<m3d::MatrixTransform>());
  data.append(std::make_shared<m3d::DualQuaternionTransform>());
  data.append(std::make_shared<m3d::EulerTransform>());

  // poses
  data.asPoses_();
  EXPECT_TRUE(data.at(0)->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data.at(1)->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data.at(2)->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data.at(3)->isType(m3d::TransformType::kEuler));

  // motions
  data.asMotions_();
  EXPECT_TRUE(data.at(0)->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(data.at(1)->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(data.at(2)->isType(m3d::TransformType::kEuler));
}

TEST(TransformContainer, initialPoseChange)
{
  // rotation and translation
  Eigen::Matrix<double, 3, 3> rotation_matrix;
  rotation_matrix.setIdentity();

  Eigen::Matrix<double, 3, 1> translation1;
  Eigen::Matrix<double, 3, 1> translation2;
  Eigen::Matrix<double, 3, 1> translation3;
  translation1 << 1.0, 0.0, 0.0;
  translation2 << 3.0, 0.0, 0.0;
  translation3 << 5.0, 0.0, 0.0;

  // container with poses
  m3d::TransformContainer data_poses(false, true);
  data_poses.append(std::make_shared<m3d::MatrixTransform>(translation1, rotation_matrix));
  data_poses.append(std::make_shared<m3d::MatrixTransform>(translation2, rotation_matrix));
  data_poses.append(std::make_shared<m3d::MatrixTransform>(translation3, rotation_matrix));

  // keep initial pose
  m3d::TransformContainer data_poses_keep = data_poses.asPoses();
  EXPECT_EQ(data_poses_keep.at(0)->asType<m3d::MatrixTransform>()->getTranslation()(0), 1.0);
  EXPECT_EQ(data_poses_keep.at(1)->asType<m3d::MatrixTransform>()->getTranslation()(0), 3.0);
  EXPECT_EQ(data_poses_keep.at(2)->asType<m3d::MatrixTransform>()->getTranslation()(0), 5.0);

  // change initial pose
  Eigen::Matrix<double, 3, 1> initial_translation;
  initial_translation << 10.0, 10.0, 0.0;
  m3d::TransformInterface::Ptr initial_pose = std::make_shared<m3d::MatrixTransform>(initial_translation, rotation_matrix);

  m3d::TransformContainer data_poses_init = data_poses.asPoses(initial_pose);
  EXPECT_EQ(data_poses_init.at(0)->asType<m3d::MatrixTransform>()->getTranslation()(0), 10.0);
  EXPECT_EQ(data_poses_init.at(1)->asType<m3d::MatrixTransform>()->getTranslation()(0), 12.0);
  EXPECT_EQ(data_poses_init.at(2)->asType<m3d::MatrixTransform>()->getTranslation()(0), 14.0);
  EXPECT_EQ(data_poses_init.at(0)->asType<m3d::MatrixTransform>()->getTranslation()(1), 10.0);
  EXPECT_EQ(data_poses_init.at(1)->asType<m3d::MatrixTransform>()->getTranslation()(1), 10.0);
  EXPECT_EQ(data_poses_init.at(2)->asType<m3d::MatrixTransform>()->getTranslation()(1), 10.0);

  // poses from motions with initial pose
  m3d::TransformContainer data_motions = data_poses.asMotions();
  m3d::TransformContainer data_poses_from_motions = data_motions.asPoses(initial_pose);
  EXPECT_EQ(data_poses_from_motions.at(0)->asType<m3d::MatrixTransform>()->getTranslation()(0), 10.0);
  EXPECT_EQ(data_poses_from_motions.at(1)->asType<m3d::MatrixTransform>()->getTranslation()(0), 12.0);
  EXPECT_EQ(data_poses_from_motions.at(2)->asType<m3d::MatrixTransform>()->getTranslation()(0), 14.0);
  EXPECT_EQ(data_poses_from_motions.at(0)->asType<m3d::MatrixTransform>()->getTranslation()(1), 10.0);
  EXPECT_EQ(data_poses_from_motions.at(1)->asType<m3d::MatrixTransform>()->getTranslation()(1), 10.0);
  EXPECT_EQ(data_poses_from_motions.at(2)->asType<m3d::MatrixTransform>()->getTranslation()(1), 10.0);

  // inplace
  data_poses.asPoses_();
  EXPECT_EQ(data_poses.at(0)->asType<m3d::MatrixTransform>()->getTranslation()(0), 1.0);
  EXPECT_EQ(data_poses.at(1)->asType<m3d::MatrixTransform>()->getTranslation()(0), 3.0);
  EXPECT_EQ(data_poses.at(2)->asType<m3d::MatrixTransform>()->getTranslation()(0), 5.0);

  data_poses.asPoses_(initial_pose);
  EXPECT_EQ(data_poses.at(0)->asType<m3d::MatrixTransform>()->getTranslation()(0), 10.0);
  EXPECT_EQ(data_poses.at(1)->asType<m3d::MatrixTransform>()->getTranslation()(0), 12.0);
  EXPECT_EQ(data_poses.at(2)->asType<m3d::MatrixTransform>()->getTranslation()(0), 14.0);
  EXPECT_EQ(data_poses.at(0)->asType<m3d::MatrixTransform>()->getTranslation()(1), 10.0);
  EXPECT_EQ(data_poses.at(1)->asType<m3d::MatrixTransform>()->getTranslation()(1), 10.0);
  EXPECT_EQ(data_poses.at(2)->asType<m3d::MatrixTransform>()->getTranslation()(1), 10.0);

  data_poses.asMotions_();
  data_poses.asPoses_(initial_pose);
  EXPECT_EQ(data_poses.at(0)->asType<m3d::MatrixTransform>()->getTranslation()(0), 10.0);
  EXPECT_EQ(data_poses.at(1)->asType<m3d::MatrixTransform>()->getTranslation()(0), 12.0);
  EXPECT_EQ(data_poses.at(2)->asType<m3d::MatrixTransform>()->getTranslation()(0), 14.0);
  EXPECT_EQ(data_poses.at(0)->asType<m3d::MatrixTransform>()->getTranslation()(1), 10.0);
  EXPECT_EQ(data_poses.at(1)->asType<m3d::MatrixTransform>()->getTranslation()(1), 10.0);
  EXPECT_EQ(data_poses.at(2)->asType<m3d::MatrixTransform>()->getTranslation()(1), 10.0);
}

TEST(TransformContainer, inverse)
{
  // rotation and translation
  Eigen::Matrix<double, 3, 3> rotation_matrix;
  rotation_matrix.setIdentity();

  Eigen::Matrix<double, 3, 1> translation1;
  Eigen::Matrix<double, 3, 1> translation2;
  translation1 << 1.0, 0.0, 0.0;
  translation2 << 3.0, 0.0, 0.0;

  // container
  m3d::TransformContainer data(false, false);
  data.append(std::make_shared<m3d::MatrixTransform>(translation1, rotation_matrix));
  data.append(std::make_shared<m3d::MatrixTransform>(translation2, rotation_matrix));

  // non-inplace
  m3d::TransformContainer data_inv = data.inverse();
  EXPECT_EQ(data_inv.at(0)->asType<m3d::MatrixTransform>()->getTranslation()(0), -1.0);
  EXPECT_EQ(data_inv.at(1)->asType<m3d::MatrixTransform>()->getTranslation()(0), -3.0);
  EXPECT_EQ(data.at(0)->asType<m3d::MatrixTransform>()->getTranslation()(0), 1.0);
  EXPECT_EQ(data.at(1)->asType<m3d::MatrixTransform>()->getTranslation()(0), 3.0);

  // inplace
  data.inverse_();
  EXPECT_EQ(data.at(0)->asType<m3d::MatrixTransform>()->getTranslation()(0), -1.0);
  EXPECT_EQ(data.at(1)->asType<m3d::MatrixTransform>()->getTranslation()(0), -3.0);
}

TEST(TransformContainer, normalized)
{
  // transforms
  Eigen::Matrix<double, 3, 1> translation = Eigen::Matrix<double, 3, 1>::Zero();

  // container
  m3d::TransformContainer data(false, false);
  data.append(std::make_shared<m3d::EulerTransform>(translation, 2*M_PI, 0.0, 0.0, m3d::EulerAxes::kSXYX));
  data.append(std::make_shared<m3d::EulerTransform>(translation, 3*M_PI, 0.0, 0.0, m3d::EulerAxes::kSXYX));

  // non-inplace
  m3d::TransformContainer data_norm = data.normalized();
  EXPECT_NEAR(data_norm.at(0)->asType<m3d::EulerTransform>()->getAi(), 0.0, 1e-12);
  EXPECT_NEAR(data_norm.at(1)->asType<m3d::EulerTransform>()->getAi(), -M_PI, 1e-12);
  EXPECT_NEAR(data.at(0)->asType<m3d::EulerTransform>()->getAi(), 2*M_PI, 1e-12);
  EXPECT_NEAR(data.at(1)->asType<m3d::EulerTransform>()->getAi(), 3*M_PI, 1e-12);

  // inplace
  data.normalized_();
  EXPECT_NEAR(data.at(0)->asType<m3d::EulerTransform>()->getAi(), 0.0, 1e-12);
  EXPECT_NEAR(data.at(1)->asType<m3d::EulerTransform>()->getAi(), -M_PI, 1e-12);
}

TEST(TransformContainer, scaleTranslation)
{
  // transforms
  Eigen::Matrix<double, 3, 1> translation1 = Eigen::Matrix<double, 3, 1>::Ones() * 1;
  Eigen::Matrix<double, 3, 1> translation2 = Eigen::Matrix<double, 3, 1>::Ones() * 2;
  Eigen::Matrix<double, 3, 1> translation4 = Eigen::Matrix<double, 3, 1>::Ones() * 4;

  // container
  m3d::TransformContainer data(false, false);
  data.append(std::make_shared<m3d::EulerTransform>(translation1, 0.0, 0.0, 0.0, m3d::EulerAxes::kSXYX));
  data.append(std::make_shared<m3d::EulerTransform>(translation2, 0.0, 0.0, 0.0, m3d::EulerAxes::kSXYX));

  // non-inplace
  auto data_copy = data.scaleTranslation(2.0);
  ASSERT_TRUE(&data_copy != &data);
  EIGEN_EXPECT_APPROX(data_copy.at(0)->asType<m3d::EulerTransform>()->getTranslation(), translation2);
  EIGEN_EXPECT_APPROX(data_copy.at(1)->asType<m3d::EulerTransform>()->getTranslation(), translation4);
  EIGEN_EXPECT_APPROX(data.at(0)->asType<m3d::EulerTransform>()->getTranslation(), translation1);
  EIGEN_EXPECT_APPROX(data.at(1)->asType<m3d::EulerTransform>()->getTranslation(), translation2);

  // inplace
  auto& data_inplace = data.scaleTranslation_(2.0);
  ASSERT_TRUE(&data_inplace == &data);
  EIGEN_EXPECT_APPROX(data.at(0)->asType<m3d::EulerTransform>()->getTranslation(), translation2);
  EIGEN_EXPECT_APPROX(data.at(1)->asType<m3d::EulerTransform>()->getTranslation(), translation4);
}

TEST(TransformContainer, apply)
{
  // transforms
  Eigen::Matrix<double, 3, 1> translation1 = Eigen::Matrix<double, 3, 1>::Zero();
  translation1[0] = 1.0;
  Eigen::Matrix<double, 3, 1> translation2 = Eigen::Matrix<double, 3, 1>::Zero();
  translation2[0] = 2.0;
  Eigen::Matrix<double, 3, 1> translation4 = Eigen::Matrix<double, 3, 1>::Zero();
  translation4[1] = 4.0;

  m3d::EulerTransform transform(translation4, 0.0, 0.0, M_PI, m3d::EulerAxes::kSXYZ);
  auto transform_if = std::make_shared<m3d::EulerTransform>(transform);

  // containers
  m3d::TransformContainer data(false, false);
  data.append(std::make_shared<m3d::EulerTransform>(translation1, 0.0, 0.0, 0.0, m3d::EulerAxes::kSXYX));
  data.append(std::make_shared<m3d::EulerTransform>(translation2, 0.0, 0.0, 0.0, m3d::EulerAxes::kSXYX));

  m3d::TransformContainer data_stamped(true, false);
  data_stamped.insert(m3d::Time(100.0), std::make_shared<m3d::EulerTransform>(
    translation1, 0.0, 0.0, 0.0, m3d::EulerAxes::kSXYX));
  data_stamped.insert(m3d::Time(200.0), std::make_shared<m3d::EulerTransform>(
    translation2, 0.0, 0.0, 0.0, m3d::EulerAxes::kSXYX));

  // apply direct
  m3d::TransformContainer data_apply_post = data.applyPost(transform);
  EXPECT_NEAR(data_apply_post.at(0)->asType<m3d::EulerTransform>()->getTranslation()[0], 1.0, 1e-12);
  EXPECT_NEAR(data_apply_post.at(1)->asType<m3d::EulerTransform>()->getTranslation()[0], 2.0, 1e-12);
  EXPECT_NEAR(data_apply_post.at(0)->asType<m3d::EulerTransform>()->getTranslation()[1], 4.0, 1e-12);
  EXPECT_NEAR(data_apply_post.at(1)->asType<m3d::EulerTransform>()->getTranslation()[1], 4.0, 1e-12);

  m3d::TransformContainer data_apply_pre = data.applyPre(transform);
  EXPECT_NEAR(data_apply_pre.at(0)->asType<m3d::EulerTransform>()->getTranslation()[0], -1.0, 1e-12);
  EXPECT_NEAR(data_apply_pre.at(1)->asType<m3d::EulerTransform>()->getTranslation()[0], -2.0, 1e-12);
  EXPECT_NEAR(data_apply_pre.at(0)->asType<m3d::EulerTransform>()->getTranslation()[1], 4.0, 1e-12);
  EXPECT_NEAR(data_apply_pre.at(1)->asType<m3d::EulerTransform>()->getTranslation()[1], 4.0, 1e-12);

  m3d::TransformContainer data_apply = data.apply(transform, transform);
  EXPECT_NEAR(data_apply.at(0)->asType<m3d::EulerTransform>()->getTranslation()[0], -1.0, 1e-12);
  EXPECT_NEAR(data_apply.at(1)->asType<m3d::EulerTransform>()->getTranslation()[0], -2.0, 1e-12);
  EXPECT_NEAR(data_apply.at(0)->asType<m3d::EulerTransform>()->getTranslation()[1], 0.0, 1e-12);
  EXPECT_NEAR(data_apply.at(1)->asType<m3d::EulerTransform>()->getTranslation()[1], 0.0, 1e-12);

  // apply interface
  m3d::TransformContainer data_apply_post_if = data.applyPost(transform_if);
  EXPECT_NEAR(data_apply_post_if.at(0)->asType<m3d::EulerTransform>()->getTranslation()[0], 1.0, 1e-12);
  EXPECT_NEAR(data_apply_post_if.at(1)->asType<m3d::EulerTransform>()->getTranslation()[0], 2.0, 1e-12);
  EXPECT_NEAR(data_apply_post_if.at(0)->asType<m3d::EulerTransform>()->getTranslation()[1], 4.0, 1e-12);
  EXPECT_NEAR(data_apply_post_if.at(1)->asType<m3d::EulerTransform>()->getTranslation()[1], 4.0, 1e-12);

  m3d::TransformContainer data_apply_pre_if = data.applyPre(transform_if);
  EXPECT_NEAR(data_apply_pre_if.at(0)->asType<m3d::EulerTransform>()->getTranslation()[0], -1.0, 1e-12);
  EXPECT_NEAR(data_apply_pre_if.at(1)->asType<m3d::EulerTransform>()->getTranslation()[0], -2.0, 1e-12);
  EXPECT_NEAR(data_apply_pre_if.at(0)->asType<m3d::EulerTransform>()->getTranslation()[1], 4.0, 1e-12);
  EXPECT_NEAR(data_apply_pre_if.at(1)->asType<m3d::EulerTransform>()->getTranslation()[1], 4.0, 1e-12);

  m3d::TransformContainer data_apply_if = data.apply(transform_if, transform_if);
  EXPECT_NEAR(data_apply_if.at(0)->asType<m3d::EulerTransform>()->getTranslation()[0], -1.0, 1e-12);
  EXPECT_NEAR(data_apply_if.at(1)->asType<m3d::EulerTransform>()->getTranslation()[0], -2.0, 1e-12);
  EXPECT_NEAR(data_apply_if.at(0)->asType<m3d::EulerTransform>()->getTranslation()[1], 0.0, 1e-12);
  EXPECT_NEAR(data_apply_if.at(1)->asType<m3d::EulerTransform>()->getTranslation()[1], 0.0, 1e-12);

  // apply functions
  m3d::TransformContainer data_apply_func = data.applyFunc(trafoFunc);
  EXPECT_NEAR(data_apply_func.at(0)->asType<m3d::EulerTransform>()->getTranslation()[0], 11.0, 1e-12);
  EXPECT_NEAR(data_apply_func.at(1)->asType<m3d::EulerTransform>()->getTranslation()[0], 12.0, 1e-12);

  m3d::TransformContainer data_apply_ifunc = data.applyIndexFunc(indexFunc);
  EXPECT_NEAR(data_apply_ifunc.at(0)->asType<m3d::EulerTransform>()->getTranslation()[0], 11.0, 1e-12);
  EXPECT_NEAR(data_apply_ifunc.at(1)->asType<m3d::EulerTransform>()->getTranslation()[0], 22.0, 1e-12);

  m3d::TransformContainer data_apply_sfunc = data_stamped.applyStampFunc(stampFunc);
  EXPECT_NEAR(data_apply_sfunc.at(0)->asType<m3d::EulerTransform>()->getTranslation()[0], 111.0, 1e-12);
  EXPECT_NEAR(data_apply_sfunc.at(1)->asType<m3d::EulerTransform>()->getTranslation()[0], 212.0, 1e-12);

  // apply direct inplace
  m3d::TransformContainer data_apply_post_inpl(data);
  data_apply_post_inpl.applyPost_(transform);
  EXPECT_NEAR(data_apply_post_inpl.at(0)->asType<m3d::EulerTransform>()->getTranslation()[0], 1.0, 1e-12);
  EXPECT_NEAR(data_apply_post_inpl.at(1)->asType<m3d::EulerTransform>()->getTranslation()[0], 2.0, 1e-12);
  EXPECT_NEAR(data_apply_post_inpl.at(0)->asType<m3d::EulerTransform>()->getTranslation()[1], 4.0, 1e-12);
  EXPECT_NEAR(data_apply_post_inpl.at(1)->asType<m3d::EulerTransform>()->getTranslation()[1], 4.0, 1e-12);

  m3d::TransformContainer data_apply_pre_inpl(data);
  data_apply_pre_inpl.applyPre_(transform);
  EXPECT_NEAR(data_apply_pre_inpl.at(0)->asType<m3d::EulerTransform>()->getTranslation()[0], -1.0, 1e-12);
  EXPECT_NEAR(data_apply_pre_inpl.at(1)->asType<m3d::EulerTransform>()->getTranslation()[0], -2.0, 1e-12);
  EXPECT_NEAR(data_apply_pre_inpl.at(0)->asType<m3d::EulerTransform>()->getTranslation()[1], 4.0, 1e-12);
  EXPECT_NEAR(data_apply_pre_inpl.at(1)->asType<m3d::EulerTransform>()->getTranslation()[1], 4.0, 1e-12);

  m3d::TransformContainer data_apply_inpl(data);
  data_apply_inpl.apply_(transform, transform);
  EXPECT_NEAR(data_apply_inpl.at(0)->asType<m3d::EulerTransform>()->getTranslation()[0], -1.0, 1e-12);
  EXPECT_NEAR(data_apply_inpl.at(1)->asType<m3d::EulerTransform>()->getTranslation()[0], -2.0, 1e-12);
  EXPECT_NEAR(data_apply_inpl.at(0)->asType<m3d::EulerTransform>()->getTranslation()[1], 0.0, 1e-12);
  EXPECT_NEAR(data_apply_inpl.at(1)->asType<m3d::EulerTransform>()->getTranslation()[1], 0.0, 1e-12);

  // apply interface inplace
  m3d::TransformContainer data_apply_post_if_inpl(data);
  data_apply_post_if_inpl.applyPost_(transform_if);
  EXPECT_NEAR(data_apply_post_if_inpl.at(0)->asType<m3d::EulerTransform>()->getTranslation()[0], 1.0, 1e-12);
  EXPECT_NEAR(data_apply_post_if_inpl.at(1)->asType<m3d::EulerTransform>()->getTranslation()[0], 2.0, 1e-12);
  EXPECT_NEAR(data_apply_post_if_inpl.at(0)->asType<m3d::EulerTransform>()->getTranslation()[1], 4.0, 1e-12);
  EXPECT_NEAR(data_apply_post_if_inpl.at(1)->asType<m3d::EulerTransform>()->getTranslation()[1], 4.0, 1e-12);

  m3d::TransformContainer data_apply_pre_if_inpl(data);
  data_apply_pre_if_inpl.applyPre_(transform_if);
  EXPECT_NEAR(data_apply_pre_if_inpl.at(0)->asType<m3d::EulerTransform>()->getTranslation()[0], -1.0, 1e-12);
  EXPECT_NEAR(data_apply_pre_if_inpl.at(1)->asType<m3d::EulerTransform>()->getTranslation()[0], -2.0, 1e-12);
  EXPECT_NEAR(data_apply_pre_if_inpl.at(0)->asType<m3d::EulerTransform>()->getTranslation()[1], 4.0, 1e-12);
  EXPECT_NEAR(data_apply_pre_if_inpl.at(1)->asType<m3d::EulerTransform>()->getTranslation()[1], 4.0, 1e-12);

  m3d::TransformContainer data_apply_if_inpl(data);
  data_apply_if_inpl.apply_(transform_if, transform_if);
  EXPECT_NEAR(data_apply_if_inpl.at(0)->asType<m3d::EulerTransform>()->getTranslation()[0], -1.0, 1e-12);
  EXPECT_NEAR(data_apply_if_inpl.at(1)->asType<m3d::EulerTransform>()->getTranslation()[0], -2.0, 1e-12);
  EXPECT_NEAR(data_apply_if_inpl.at(0)->asType<m3d::EulerTransform>()->getTranslation()[1], 0.0, 1e-12);
  EXPECT_NEAR(data_apply_if_inpl.at(1)->asType<m3d::EulerTransform>()->getTranslation()[1], 0.0, 1e-12);

  // apply functions inplace
  m3d::TransformContainer data_apply_func_inpl(data);
  data_apply_func_inpl.applyFunc_(trafoFunc);
  EXPECT_NEAR(data_apply_func_inpl.at(0)->asType<m3d::EulerTransform>()->getTranslation()[0], 11.0, 1e-12);
  EXPECT_NEAR(data_apply_func_inpl.at(1)->asType<m3d::EulerTransform>()->getTranslation()[0], 12.0, 1e-12);

  m3d::TransformContainer data_apply_ifunc_inpl(data);
  data_apply_ifunc_inpl.applyIndexFunc_(indexFunc);
  EXPECT_NEAR(data_apply_ifunc_inpl.at(0)->asType<m3d::EulerTransform>()->getTranslation()[0], 11.0, 1e-12);
  EXPECT_NEAR(data_apply_ifunc_inpl.at(1)->asType<m3d::EulerTransform>()->getTranslation()[0], 22.0, 1e-12);

  m3d::TransformContainer data_apply_sfunc_inpl(data_stamped);
  data_apply_sfunc_inpl.applyStampFunc_(stampFunc);
  EXPECT_NEAR(data_apply_sfunc_inpl.at(0)->asType<m3d::EulerTransform>()->getTranslation()[0], 111.0, 1e-12);
  EXPECT_NEAR(data_apply_sfunc_inpl.at(1)->asType<m3d::EulerTransform>()->getTranslation()[0], 212.0, 1e-12);
}

TEST(TransformContainer, toVector)
{
  // container
  m3d::TransformContainer data(false, false);
  data.append(std::make_shared<m3d::DualQuaternionTransform>());
  data.append(std::make_shared<m3d::EulerTransform>());

  // direct
  std::vector<m3d::MatrixTransform> matrix_vec = data.toVector<m3d::MatrixTransform>();
  EXPECT_EQ(matrix_vec.size(), 2);

  // interfaces with original types
  std::vector<m3d::TransformInterface::Ptr> interface_vec = data.toVector();
  ASSERT_EQ(interface_vec.size(), 2);
  EXPECT_TRUE(interface_vec.at(0)->isType(m3d::TransformType::kDualQuaternion));
  EXPECT_TRUE(interface_vec.at(1)->isType(m3d::TransformType::kEuler));

  // interfaces with specific type
  std::vector<m3d::TransformInterface::Ptr> matrix_if_vec = data.toVector(m3d::TransformType::kMatrix);
  ASSERT_EQ(matrix_if_vec.size(), 2);
  EXPECT_TRUE(matrix_if_vec.at(0)->isType(m3d::TransformType::kMatrix));
  EXPECT_TRUE(matrix_if_vec.at(1)->isType(m3d::TransformType::kMatrix));
}

TEST(TransformContainer, toEigenVectorTemplated)
{
  // container
  m3d::TransformContainer data(false, false);
  data.append(std::make_shared<m3d::MatrixTransform>());
  data.append(std::make_shared<m3d::DualQuaternionTransform>());
  data.append(std::make_shared<m3d::EulerTransform>());

  // check types
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> axis_angle = data.toEigenVector<m3d::AxisAngleTransform>();
  EXPECT_EQ(axis_angle.rows(), 3);
  EXPECT_EQ(axis_angle.cols(), 7);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> dual_quaternion = data.toEigenVector<m3d::DualQuaternionTransform>();
  EXPECT_EQ(dual_quaternion.rows(), 3);
  EXPECT_EQ(dual_quaternion.cols(), 8);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> euler = data.toEigenVector<m3d::EulerTransform>();
  EXPECT_EQ(euler.rows(), 3);
  EXPECT_EQ(euler.cols(), 7);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix = data.toEigenVector<m3d::MatrixTransform>();
  EXPECT_EQ(matrix.rows(), 3);
  EXPECT_EQ(matrix.cols(), 12);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> quaternion = data.toEigenVector<m3d::QuaternionTransform>();
  EXPECT_EQ(quaternion.rows(), 3);
  EXPECT_EQ(quaternion.cols(), 7);
}

TEST(TransformContainer, toEigenVectorTyped)
{
  // container
  m3d::TransformContainer data(false, false);
  data.append(std::make_shared<m3d::MatrixTransform>());
  data.append(std::make_shared<m3d::DualQuaternionTransform>());
  data.append(std::make_shared<m3d::EulerTransform>());

  // check types
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> axis_angle = data.toEigenVector(m3d::TransformType::kAxisAngle);
  EXPECT_EQ(axis_angle.rows(), 3);
  EXPECT_EQ(axis_angle.cols(), 7);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> dual_quaternion = data.toEigenVector(m3d::TransformType::kDualQuaternion);
  EXPECT_EQ(dual_quaternion.rows(), 3);
  EXPECT_EQ(dual_quaternion.cols(), 8);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> euler = data.toEigenVector(m3d::TransformType::kEuler);
  EXPECT_EQ(euler.rows(), 3);
  EXPECT_EQ(euler.cols(), 7);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix = data.toEigenVector(m3d::TransformType::kMatrix);
  EXPECT_EQ(matrix.rows(), 3);
  EXPECT_EQ(matrix.cols(), 12);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> quaternion = data.toEigenVector(m3d::TransformType::kQuaternion);
  EXPECT_EQ(quaternion.rows(), 3);
  EXPECT_EQ(quaternion.cols(), 7);
}

TEST(TransformContainer, description)
{
  m3d::TransformContainer data(false, false);

  // stream
  std::stringstream ss;
  ss << data;
  ASSERT_STRING_STARTS_WITH(ss.str(), "TransformContainer");

  // description
  ASSERT_STRING_STARTS_WITH(data.desc(), ss.str());
}
