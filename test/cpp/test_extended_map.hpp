#pragma once

#include <gtest/gtest.h>

#include <motion3d/containers.hpp>

namespace m3d = motion3d;

/// Container class for a single double scalar.
struct DoubleData
{
  explicit DoubleData(const double& x) : x(x) { }
  double x;
};

/// Comparator for two double data containers.
struct DoubleDataCmp {
  bool operator()(const DoubleData& a, const DoubleData& b) const {
    return a.x < b.x;
  }
};

/// Container class for a single integer scalar.
struct IntData
{
  explicit IntData(const int& x) : x(x) { }
  int x;
};

TEST(ExtendedMap, constructor)
{
  // without cast
  std::vector<int> keys = {0, 3, 1};
  std::vector<double> values = {0.0, 33.0, 11.0};

  m3d::ExtendedMap<int, double> data(keys, values, false);
  for (const auto &it : data)
  {
    EXPECT_EQ(it.second, it.first * 11.0);
  }

  m3d::ExtendedMap<int, double> data_sorted(keys, values, true);
  for (const auto &it : data_sorted)
  {
    EXPECT_EQ(it.second, it.first * 11.0);
  }

  // with cast
  std::vector<std::uint8_t> keys_conv = {0, 3, 1};
  std::vector<int> values_conv = {0, 33, 11};

  m3d::ExtendedMap<int, double> data_conv(keys_conv, values_conv, false);
  for (const auto &it : data_conv)
  {
    EXPECT_EQ(it.second, it.first * 11.0);
  }

  m3d::ExtendedMap<int, double> data_conv_sorted(keys_conv, values_conv, true);
  for (const auto &it : data_conv_sorted)
  {
    EXPECT_EQ(it.second, it.first * 11.0);
  }

  // exceptions
  std::vector<double> values_err = {0, 33};
  EXPECT_THROW((m3d::ExtendedMap<int, double>(keys, values_err)), std::invalid_argument);
}

TEST(ExtendedMap, iterators)
{
  // create data
  std::vector<double> keys = {1.1, 2.2};
  std::vector<int> values = {11, 22};

  m3d::ExtendedMap<DoubleData, IntData, DoubleDataCmp> data;
  for (unsigned int i = 0; i < keys.size(); ++i)
  {
    data.emplace(keys.at(i), values.at(i));
  }

  // key iterator
  auto key_it = data.cbegin_keys();
  EXPECT_EQ(key_it->x, 1.1);
  EXPECT_EQ((*key_it).x, 1.1);

  key_it++;
  EXPECT_EQ(key_it->x, 2.2);
  EXPECT_EQ((*key_it).x, 2.2);

  key_it++;
  EXPECT_TRUE(key_it == data.cend_keys());

  // value iterator
  auto val_it = data.cbegin_values();
  EXPECT_EQ(val_it->x, 11);
  EXPECT_EQ((*val_it).x, 11);

  val_it++;
  EXPECT_EQ(val_it->x, 22);
  EXPECT_EQ((*val_it).x, 22);

  val_it++;
  EXPECT_TRUE(val_it == data.cend_values());
}

TEST(ExtendedMap, find)
{
  // create data
  m3d::ExtendedMap<double, int> data;
  data[1.1] = 11;
  data[2.2] = 22;
  data[3.3] = 33;

  // equal
  m3d::ExtendedMap<double, int>::iterator it;
  it = data.find_eq(2.2);
  EXPECT_TRUE(it != data.end());
  EXPECT_TRUE(it->second == 22);

  it = data.find_eq(2.5);
  EXPECT_TRUE(it == data.end());

  // greater equal
  it = data.find_ge(2.2);
  EXPECT_TRUE(it != data.end());
  EXPECT_TRUE(it->second == 22);

  it = data.find_ge(3.0);
  EXPECT_TRUE(it != data.end());
  EXPECT_TRUE(it->second == 33);

  it = data.find_ge(4.0);
  EXPECT_TRUE(it == data.end());

  // greater than
  it = data.find_gt(2.2);
  EXPECT_TRUE(it != data.end());
  EXPECT_TRUE(it->second == 33);

  it = data.find_gt(3.0);
  EXPECT_TRUE(it != data.end());
  EXPECT_TRUE(it->second == 33);

  it = data.find_gt(4.0);
  EXPECT_TRUE(it == data.end());

  // lower equal
  it = data.find_le(2.2);
  EXPECT_TRUE(it != data.end());
  EXPECT_TRUE(it->second == 22);

  it = data.find_le(2.0);
  EXPECT_TRUE(it != data.end());
  EXPECT_TRUE(it->second == 11);

  it = data.find_le(1.0);
  EXPECT_TRUE(it == data.end());

  // lower than
  it = data.find_lt(2.2);
  EXPECT_TRUE(it != data.end());
  EXPECT_TRUE(it->second == 11);

  it = data.find_lt(2.0);
  EXPECT_TRUE(it != data.end());
  EXPECT_TRUE(it->second == 11);

  it = data.find_lt(1.0);
  EXPECT_TRUE(it == data.end());
}

TEST(ExtendedMap, find_const)
{
  // create data
  m3d::ExtendedMap<double, int>::Ptr data = std::make_shared<m3d::ExtendedMap<double, int>>();
  data->insert({1.1, 11});
  data->insert({2.2, 22});
  data->insert({3.3, 33});

  m3d::ExtendedMap<double, int>::ConstPtr data_const(data);

  // equal
  m3d::ExtendedMap<double, int>::const_iterator it;
  it = data_const->find_eq(2.2);
  EXPECT_TRUE(it != data_const->cend());
  EXPECT_TRUE(it->second == 22);

  it = data_const->find_eq(2.5);
  EXPECT_TRUE(it == data_const->cend());

  // greater equal
  it = data_const->find_ge(2.2);
  EXPECT_TRUE(it != data_const->cend());
  EXPECT_TRUE(it->second == 22);

  it = data_const->find_ge(3.0);
  EXPECT_TRUE(it != data_const->cend());
  EXPECT_TRUE(it->second == 33);

  it = data_const->find_ge(4.0);
  EXPECT_TRUE(it == data_const->cend());

  // greater than
  it = data_const->find_gt(2.2);
  EXPECT_TRUE(it != data_const->cend());
  EXPECT_TRUE(it->second == 33);

  it = data_const->find_gt(3.0);
  EXPECT_TRUE(it != data_const->cend());
  EXPECT_TRUE(it->second == 33);

  it = data_const->find_gt(4.0);
  EXPECT_TRUE(it == data_const->cend());

  // lower equal
  it = data_const->find_le(2.2);
  EXPECT_TRUE(it != data_const->cend());
  EXPECT_TRUE(it->second == 22);

  it = data_const->find_le(2.0);
  EXPECT_TRUE(it != data_const->cend());
  EXPECT_TRUE(it->second == 11);

  it = data_const->find_le(1.0);
  EXPECT_TRUE(it == data_const->cend());

  // lower than
  it = data_const->find_lt(2.2);
  EXPECT_TRUE(it != data_const->cend());
  EXPECT_TRUE(it->second == 11);

  it = data_const->find_lt(2.0);
  EXPECT_TRUE(it != data_const->cend());
  EXPECT_TRUE(it->second == 11);

  it = data_const->find_lt(1.0);
  EXPECT_TRUE(it == data_const->cend());
}
