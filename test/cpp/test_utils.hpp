#pragma once

#include <gtest/gtest.h>

#include <motion3d/utils.hpp>

#include "utils.hpp"

namespace m3d = motion3d;


TEST(Utils, vectorVectorConversion)
{
  // convert double vector to binary vector and back
  std::vector<double> vector1 = {1.0, -2.0, 3.0, -4.0, std::numeric_limits<double>::quiet_NaN(), 
    std::numeric_limits<double>::max(), std::numeric_limits<double>::infinity()};
  std::vector<std::uint8_t> binary1 = m3d::convertVector<double, std::uint8_t>(vector1);
  std::vector<double> vector1_back = m3d::convertVector<std::uint8_t, double>(binary1);
  EXPECT_VECTOR_APPROX(vector1, vector1_back);

  // convert binary vector to double vector and back
  std::vector<std::uint8_t> binary2 = {1, 2, 3, 4, 5, 6, 7, 8};
  std::vector<double> vector2 = m3d::convertVector<std::uint8_t, double>(binary2);
  std::vector<std::uint8_t> binary2_back = m3d::convertVector<double, std::uint8_t>(vector2);
  EXPECT_VECTOR_APPROX(binary2, binary2_back);

  // exception handling
  EXPECT_THROW((m3d::convertVector<std::uint8_t, double>({1})), m3d::VectorConversionException);
}

TEST(Utils, vectorValueConversion)
{ 
  // convert scalar to binary vector and back
  std::uint64_t value1 = 100;
  std::vector<std::uint8_t> binary1 = m3d::convertToVector<std::uint64_t, std::uint8_t>(value1);
  std::uint64_t value1_back = m3d::convertFromVector<std::uint8_t, std::uint64_t>(binary1);
  EXPECT_EQ(value1, value1_back);

  // convert binary vector to scalar and back
  std::vector<std::uint8_t> binary2 = {1, 2, 3, 4, 5, 6, 7, 8};
  std::uint64_t value2 = m3d::convertFromVector<std::uint8_t, std::uint64_t>(binary2);
  std::vector<std::uint8_t> binary2_back = m3d::convertToVector<std::uint64_t, std::uint8_t>(value2);
  EXPECT_VECTOR_APPROX(binary2, binary2_back);

  // exception handling
  EXPECT_THROW((m3d::convertFromVector<std::uint8_t, std::uint64_t>({1})),
    m3d::VectorConversionException);
  EXPECT_THROW((m3d::convertFromVector<std::uint8_t, std::uint64_t>({1, 2, 3, 4, 5, 6, 7, 8, 9})),
    m3d::VectorConversionException);
  EXPECT_THROW((m3d::convertFromVector<std::uint8_t, std::uint64_t>({1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16})), 
    m3d::VectorConversionException);
  EXPECT_THROW((m3d::convertToVector<std::uint8_t, std::uint16_t>(1)), m3d::VectorConversionException);
}
