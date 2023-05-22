#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <tuple>

#include <motion3d/utils/conversion.hpp>

namespace motion3d
{

constexpr std::uint64_t kNanoSecsPerSec = 1000000000ULL;
constexpr double kSecsPerNanoSec = 1e-9;

/** \brief A class for storing, converting and comparing timestamps. */
class Time
{
 public:
  constexpr static std::uint32_t kBinarySize = 8;

  /** Default constructor leaving the time uninitialized. */
  Time() = default;

  /** Constructs and initializes from time in seconds. */
  explicit Time(double t_sec);

  /** Constructs and initializes from time in nanoseconds. */
  explicit Time(std::uint64_t t_nsec);

  /** Constructs and initializes from <CODE>std::chrono::time_point</CODE>. */
  template<class Clock, class Duration>
  explicit Time(const std::chrono::time_point<Clock, Duration> &t);

  /** Constructs and initializes from a string containing the time in nanoseconds. */
  explicit Time(const std::string &t_nsec);

  /** Constructs and initializes from a vector with binary data. */
  explicit Time(const std::vector<std::uint8_t> &binary);

  /** Constructs and initializes from time separated into seconds and nanoseconds. */
  Time(std::uint32_t t_sec, std::uint32_t t_nsec);

  /** \sa Time(double) */
  static Time FromSec(double t_sec);

  /** \sa Time(std::uint64_t) */
  static Time FromNSec(std::uint64_t t_nsec);

  /** \sa Time(std::uint32_t, std::uint32_t) */
  static Time FromSecNSec(std::uint32_t t_sec, std::uint32_t t_nsec);

  /** \returns time in seconds. */
  double toSec() const;

  /** \returns time in nanoseconds. */
  std::uint64_t toNSec() const;

  /** \returns time as <CODE>std::chrono</CODE> duration. */
  std::chrono::nanoseconds toChrono() const;

  /** \returns time separated into seconds and nanoseconds. */
  std::pair<std::uint32_t, std::uint32_t> toSecNSec() const;

  /** \returns time as binary data. */
  std::vector<std::uint8_t> toBinary() const;

  /** \returns a description of <CODE>*this</CODE>. */
  std::string desc() const { return streamToString(*this); }
  
  friend bool operator==(const Time &lhs, const Time &rhs) { return lhs.nsec_ == rhs.nsec_; }
  friend bool operator!=(const Time &lhs, const Time &rhs) { return lhs.nsec_ != rhs.nsec_; }
  friend bool operator<(const Time &lhs, const Time &rhs) { return lhs.nsec_ < rhs.nsec_; }
  friend bool operator<=(const Time &lhs, const Time &rhs) { return lhs.nsec_ <= rhs.nsec_; }
  friend bool operator>(const Time &lhs, const Time &rhs) { return lhs.nsec_ > rhs.nsec_; }
  friend bool operator>=(const Time &lhs, const Time &rhs) { return lhs.nsec_ >= rhs.nsec_; }

  /** Inserts the time in nanoseconds. */
  friend std::ostream& operator<<(std::ostream &os, const Time &t) { return os << t.nsec_; }

  /** \returns the absolute difference between Time <I>ta</I> and Time <I>tb</I>. */
  friend Time timeDiffAbs(const Time &ta, const Time &tb);

 private:
  std::uint64_t nsec_ {0};  ///< Time in nanoseconds
};

///////////////////////////////////////////////////////////////////////////////

Time::Time(double t_sec)
{
  if (t_sec < 0)
  {
    nsec_ = 0;
  }
  else
  {
    double secd = std::floor(t_sec);
    nsec_ = static_cast<std::uint64_t>(secd) * kNanoSecsPerSec + static_cast<std::uint64_t>((t_sec - secd) * kNanoSecsPerSec);
  }
}

Time::Time(std::uint64_t t_nsec)
  : nsec_(t_nsec)
{
}

template<class Clock, class Duration>
Time::Time(const std::chrono::time_point<Clock, Duration> &t)
  : nsec_(std::chrono::time_point_cast<std::chrono::nanoseconds>(t).time_since_epoch().count())
{
}

Time::Time(const std::string &t_nsec)
  : nsec_(std::stoull(t_nsec))
{
}

Time::Time(const std::vector<std::uint8_t> &binary)
  : nsec_(convertFromVector<std::uint8_t, std::uint64_t>(binary))
{
}

Time::Time(std::uint32_t t_sec, std::uint32_t t_nsec)
  : nsec_(static_cast<std::uint64_t>(t_sec) * kNanoSecsPerSec + static_cast<std::uint64_t>(t_nsec))
{
}

Time Time::FromSec(double t_sec)
{
  return Time(t_sec);
}

Time Time::FromNSec(std::uint64_t t_nsec)
{
  return Time(t_nsec);
}

Time Time::FromSecNSec(std::uint32_t t_sec, std::uint32_t t_nsec)
{
  return {t_sec, t_nsec};
}

double Time::toSec() const
{
  auto secd = static_cast<double>(nsec_ / kNanoSecsPerSec);
  auto nsecd = static_cast<double>(nsec_ % kNanoSecsPerSec);
  return secd + kSecsPerNanoSec * nsecd;
}

std::uint64_t Time::toNSec() const
{
  return nsec_;
}

std::chrono::nanoseconds Time::toChrono() const
{
  return std::chrono::nanoseconds(nsec_);
}

std::pair<std::uint32_t, std::uint32_t> Time::toSecNSec() const
{
  auto sec32 = static_cast<std::uint32_t>(nsec_ / kNanoSecsPerSec);
  auto nsec32 = static_cast<std::uint32_t>(nsec_ % kNanoSecsPerSec);
  return std::make_pair(sec32, nsec32);
}

std::vector<std::uint8_t> Time::toBinary() const
{
  return convertToVector<std::uint64_t, std::uint8_t>(nsec_);
}

Time timeDiffAbs(const Time &ta, const Time &tb)
{
  if (ta.nsec_ > tb.nsec_)
  {
    return Time(ta.nsec_ - tb.nsec_);
  }
  return Time(tb.nsec_ - ta.nsec_);
}

} // namespace motion3d
