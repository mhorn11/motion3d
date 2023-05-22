#pragma once

#include <chrono>

#include <gtest/gtest.h>

#include <motion3d/common/time.hpp>

namespace m3d = motion3d;


using namespace std::chrono_literals;
using Clock = std::chrono::high_resolution_clock;
using Ms = std::chrono::milliseconds;
using Sec = std::chrono::seconds;
template<class Duration>
using TimePoint = std::chrono::time_point<Clock, Duration>;


TEST(Time, constructor)
{
  // default constructor
  m3d::Time time_default;
  ASSERT_EQ(time_default.toNSec(), 0);

  // from seconds as double
  double sec1 = 1.0;
  double sec2 = -1.0;
  m3d::Time time_sec1(sec1);
  m3d::Time time_sec2 = m3d::Time::FromSec(1.0);
  m3d::Time time_sec3(sec2);
  m3d::Time time_sec4 = m3d::Time::FromSec(-1.0);
  ASSERT_EQ(time_sec1.toNSec(), 1000000000);
  ASSERT_EQ(time_sec2.toNSec(), 1000000000);
  ASSERT_EQ(time_sec3.toNSec(), 0);
  ASSERT_EQ(time_sec4.toNSec(), 0);

  // from nanoseconds as uint64
  std::uint64_t nsec = 100000;
  m3d::Time time_nsec1(nsec);
  m3d::Time time_nsec2 = m3d::Time::FromNSec(100000);
  ASSERT_EQ(time_nsec1.toNSec(), 100000);
  ASSERT_EQ(time_nsec2.toNSec(), 100000);

  // from chrono
  TimePoint<Sec> time_point_s{1s};
  m3d::Time time_chrono_s(time_point_s);
  ASSERT_EQ(time_chrono_s.toNSec(), 1000000000);

  TimePoint<Ms> time_point_ms{1ms};
  m3d::Time time_chrono_ms(time_point_ms);
  ASSERT_EQ(time_chrono_ms.toNSec(), 1000000);

  // from time separated into seconds and nanoseconds as uint32
  std::uint32_t sep_sec = 200000;
  std::uint32_t sep_nsec = 100000;
  m3d::Time time_sep1(sep_sec, sep_nsec);
  m3d::Time time_sep2 = m3d::Time::FromSecNSec(200000, 100000);
  ASSERT_EQ(time_sep1.toNSec(), 200000000100000);
  ASSERT_EQ(time_sep2.toNSec(), 200000000100000);
}

TEST(Time, conversion)
{
  m3d::Time time = m3d::Time::FromSec(2.0);

  // seconds
  ASSERT_EQ(time.toSec(), 2.0);

  // nanoseconds
  ASSERT_EQ(time.toNSec(), 2000000000);

  // to chrono
  ASSERT_EQ(time.toChrono().count(), 2000000000);

  // separated timestamp
  std::uint32_t sep_sec, sep_nsec;
  std::tie(sep_sec, sep_nsec) = time.toSecNSec();
  ASSERT_EQ(sep_sec, 2);
  ASSERT_EQ(sep_nsec, 0);
}

TEST(Time, operators)
{
  m3d::Time time1a = m3d::Time::FromSec(1.0);
  m3d::Time time1b = m3d::Time::FromSec(1.0);
  m3d::Time time2 = m3d::Time::FromSec(2.0);

  // equality
  ASSERT_TRUE(time1a == time1b);
  ASSERT_FALSE(time1a == time2);

  ASSERT_FALSE(time1a != time1b);
  ASSERT_TRUE(time1a != time2);

  // comparison
  ASSERT_FALSE(time1a > time1b);
  ASSERT_FALSE(time1a > time2);
  ASSERT_TRUE(time2 > time1a);

  ASSERT_FALSE(time1a < time1b);
  ASSERT_TRUE(time1a < time2);
  ASSERT_FALSE(time2 < time1a);

  ASSERT_TRUE(time1a >= time1b);
  ASSERT_FALSE(time1a >= time2);
  ASSERT_TRUE(time2 >= time1a);

  ASSERT_TRUE(time1a <= time1b);
  ASSERT_TRUE(time1a <= time2);
  ASSERT_FALSE(time2 <= time1a);
}

TEST(Time, diff)
{
  m3d::Time time1a = m3d::Time::FromSec(10.0);
  m3d::Time time1b = m3d::Time::FromSec(10.0);
  m3d::Time time2 = m3d::Time::FromSec(12.0);

  ASSERT_EQ(timeDiffAbs(time1a, time1b).toNSec(), 0);
  ASSERT_EQ(timeDiffAbs(time1a, time2).toNSec(), 2000000000);
  ASSERT_EQ(timeDiffAbs(time2, time1a).toNSec(), 2000000000);
}

TEST(Time, description)
{
  m3d::Time t = m3d::Time::FromNSec(1234);

  // stream
  std::stringstream ss;
  ss << t;
  ASSERT_EQ(ss.str(), "1234");

  // description
  ASSERT_EQ(t.desc(), ss.str());
}
