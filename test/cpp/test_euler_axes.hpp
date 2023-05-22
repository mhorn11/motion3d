#pragma once

#include <gtest/gtest.h>

#include <motion3d/transforms/implementation_/euler_axes.hpp>

namespace m3d = motion3d;


TEST(EulerAxes, eulerAxesFromStr)
{
  EXPECT_EQ(m3d::eulerAxesFromStr("SXYZ"), m3d::EulerAxes::kSXYZ);
  EXPECT_EQ(m3d::eulerAxesFromStr("SXYX"), m3d::EulerAxes::kSXYX);
  EXPECT_EQ(m3d::eulerAxesFromStr("SXZY"), m3d::EulerAxes::kSXZY);
  EXPECT_EQ(m3d::eulerAxesFromStr("SXZX"), m3d::EulerAxes::kSXZX);
  EXPECT_EQ(m3d::eulerAxesFromStr("SYZX"), m3d::EulerAxes::kSYZX);
  EXPECT_EQ(m3d::eulerAxesFromStr("SYZY"), m3d::EulerAxes::kSYZY);
  EXPECT_EQ(m3d::eulerAxesFromStr("SYXZ"), m3d::EulerAxes::kSYXZ);
  EXPECT_EQ(m3d::eulerAxesFromStr("SYXY"), m3d::EulerAxes::kSYXY);
  EXPECT_EQ(m3d::eulerAxesFromStr("SZXY"), m3d::EulerAxes::kSZXY);
  EXPECT_EQ(m3d::eulerAxesFromStr("SZXZ"), m3d::EulerAxes::kSZXZ);
  EXPECT_EQ(m3d::eulerAxesFromStr("SZYX"), m3d::EulerAxes::kSZYX);
  EXPECT_EQ(m3d::eulerAxesFromStr("SZYZ"), m3d::EulerAxes::kSZYZ);
  EXPECT_EQ(m3d::eulerAxesFromStr("RZYX"), m3d::EulerAxes::kRZYX);
  EXPECT_EQ(m3d::eulerAxesFromStr("RXYX"), m3d::EulerAxes::kRXYX);
  EXPECT_EQ(m3d::eulerAxesFromStr("RYZX"), m3d::EulerAxes::kRYZX);
  EXPECT_EQ(m3d::eulerAxesFromStr("RXZX"), m3d::EulerAxes::kRXZX);
  EXPECT_EQ(m3d::eulerAxesFromStr("RXZY"), m3d::EulerAxes::kRXZY);
  EXPECT_EQ(m3d::eulerAxesFromStr("RYZY"), m3d::EulerAxes::kRYZY);
  EXPECT_EQ(m3d::eulerAxesFromStr("RZXY"), m3d::EulerAxes::kRZXY);
  EXPECT_EQ(m3d::eulerAxesFromStr("RYXY"), m3d::EulerAxes::kRYXY);
  EXPECT_EQ(m3d::eulerAxesFromStr("RYXZ"), m3d::EulerAxes::kRYXZ);
  EXPECT_EQ(m3d::eulerAxesFromStr("RZXZ"), m3d::EulerAxes::kRZXZ);
  EXPECT_EQ(m3d::eulerAxesFromStr("RXYZ"), m3d::EulerAxes::kRXYZ);
  EXPECT_EQ(m3d::eulerAxesFromStr("RZYZ"), m3d::EulerAxes::kRZYZ);
  EXPECT_THROW(m3d::eulerAxesFromStr("invalid"), m3d::InvalidEulerAxesException);
}

TEST(EulerAxes, eulerAxesToStr)
{
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kSXYZ), "SXYZ");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kSXYX), "SXYX");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kSXZY), "SXZY");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kSXZX), "SXZX");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kSYZX), "SYZX");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kSYZY), "SYZY");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kSYXZ), "SYXZ");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kSYXY), "SYXY");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kSZXY), "SZXY");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kSZXZ), "SZXZ");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kSZYX), "SZYX");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kSZYZ), "SZYZ");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kRZYX), "RZYX");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kRXYX), "RXYX");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kRYZX), "RYZX");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kRXZX), "RXZX");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kRXZY), "RXZY");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kRYZY), "RYZY");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kRZXY), "RZXY");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kRYXY), "RYXY");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kRYXZ), "RYXZ");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kRZXZ), "RZXZ");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kRXYZ), "RXYZ");
  EXPECT_EQ(m3d::eulerAxesToStr(m3d::EulerAxes::kRZYZ), "RZYZ");
}

TEST(EulerAxes, tuple)
{
  std::uint8_t firstaxis;
  std::uint8_t parity;
  std::uint8_t repetition;
  std::uint8_t frame;

  m3d::eulerAxesToTuple(m3d::EulerAxes::kSZYZ, firstaxis, parity, repetition, frame);

  EXPECT_EQ(firstaxis, 2);
  EXPECT_EQ(parity, 1);
  EXPECT_EQ(repetition, 1);
  EXPECT_EQ(frame, 0);
}
