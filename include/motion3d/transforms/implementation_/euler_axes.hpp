#pragma once

#include <functional>
#include <string>

#include <motion3d/common/type_traits.hpp>

namespace motion3d
{

/** Exception thrown when using an invalid Euler axes order, e.g., when providing an invalid <CODE>uint8_t</CODE>. */
struct InvalidEulerAxesException : public std::exception
{
  const char* what() const noexcept override
  {
    return "Invalid or unsupported Euler axes given";
  }
};

namespace internal
{
  /** \returns an <CODE>uint8_t</CODE> that unambigiously describes an Euler axis order. */
  template <std::uint8_t firstaxis, std::uint8_t parity, std::uint8_t repetition, std::uint8_t frame>
  constexpr uint8_t eulerAxesValue() 
  {
    static_assert(firstaxis <= 3 && parity <= 3 && repetition <= 3 && frame <= 3);
    return (firstaxis << 6) + (parity << 4) + (repetition << 2) + frame;  // NOLINT(*magic-numbers)
  }
} // end namespace internal

/** Enumerate of all available Euler axes orders. */
enum class EulerAxes : std::uint8_t
{
  // firstaxis, parity, repetition, frame
  kSXYZ = internal::eulerAxesValue<0, 0, 0, 0>(),
  kSXYX = internal::eulerAxesValue<0, 0, 1, 0>(),
  kSXZY = internal::eulerAxesValue<0, 1, 0, 0>(),
  kSXZX = internal::eulerAxesValue<0, 1, 1, 0>(),
  kSYZX = internal::eulerAxesValue<1, 0, 0, 0>(),
  kSYZY = internal::eulerAxesValue<1, 0, 1, 0>(),
  kSYXZ = internal::eulerAxesValue<1, 1, 0, 0>(),
  kSYXY = internal::eulerAxesValue<1, 1, 1, 0>(),
  kSZXY = internal::eulerAxesValue<2, 0, 0, 0>(),
  kSZXZ = internal::eulerAxesValue<2, 0, 1, 0>(),
  kSZYX = internal::eulerAxesValue<2, 1, 0, 0>(),
  kSZYZ = internal::eulerAxesValue<2, 1, 1, 0>(),
  kRZYX = internal::eulerAxesValue<0, 0, 0, 1>(),
  kRXYX = internal::eulerAxesValue<0, 0, 1, 1>(),
  kRYZX = internal::eulerAxesValue<0, 1, 0, 1>(),
  kRXZX = internal::eulerAxesValue<0, 1, 1, 1>(),
  kRXZY = internal::eulerAxesValue<1, 0, 0, 1>(),
  kRYZY = internal::eulerAxesValue<1, 0, 1, 1>(),
  kRZXY = internal::eulerAxesValue<1, 1, 0, 1>(),
  kRYXY = internal::eulerAxesValue<1, 1, 1, 1>(),
  kRYXZ = internal::eulerAxesValue<2, 0, 0, 1>(),
  kRZXZ = internal::eulerAxesValue<2, 0, 1, 1>(),
  kRXYZ = internal::eulerAxesValue<2, 1, 0, 1>(),
  kRZYZ = internal::eulerAxesValue<2, 1, 1, 1>(),
};

constexpr EulerAxes kEulerAxesDefault = EulerAxes::kSXYZ;

/** List of all available Euler axes orders. */
constexpr std::initializer_list<EulerAxes> kEulerAxesAll = {
  EulerAxes::kSXYZ,
  EulerAxes::kSXYX,
  EulerAxes::kSXZY,
  EulerAxes::kSXZX,
  EulerAxes::kSYZX,
  EulerAxes::kSYZY,
  EulerAxes::kSYXZ,
  EulerAxes::kSYXY,
  EulerAxes::kSZXY,
  EulerAxes::kSZXZ,
  EulerAxes::kSZYX,
  EulerAxes::kSZYZ,
  EulerAxes::kRZYX,
  EulerAxes::kRXYX,
  EulerAxes::kRYZX,
  EulerAxes::kRXZX,
  EulerAxes::kRXZY,
  EulerAxes::kRYZY,
  EulerAxes::kRZXY,
  EulerAxes::kRYXY,
  EulerAxes::kRYXZ,
  EulerAxes::kRZXZ,
  EulerAxes::kRXYZ,
  EulerAxes::kRZYZ
};


namespace internal
{
  template<EulerAxes T> struct euler_traits;
  template <> struct euler_traits<EulerAxes::kSXYZ> { static inline const std::string Name = "SXYZ"; };
  template <> struct euler_traits<EulerAxes::kSXYX> { static inline const std::string Name = "SXYX"; };
  template <> struct euler_traits<EulerAxes::kSXZY> { static inline const std::string Name = "SXZY"; };
  template <> struct euler_traits<EulerAxes::kSXZX> { static inline const std::string Name = "SXZX"; };
  template <> struct euler_traits<EulerAxes::kSYZX> { static inline const std::string Name = "SYZX"; };
  template <> struct euler_traits<EulerAxes::kSYZY> { static inline const std::string Name = "SYZY"; };
  template <> struct euler_traits<EulerAxes::kSYXZ> { static inline const std::string Name = "SYXZ"; };
  template <> struct euler_traits<EulerAxes::kSYXY> { static inline const std::string Name = "SYXY"; };
  template <> struct euler_traits<EulerAxes::kSZXY> { static inline const std::string Name = "SZXY"; };
  template <> struct euler_traits<EulerAxes::kSZXZ> { static inline const std::string Name = "SZXZ"; };
  template <> struct euler_traits<EulerAxes::kSZYX> { static inline const std::string Name = "SZYX"; };
  template <> struct euler_traits<EulerAxes::kSZYZ> { static inline const std::string Name = "SZYZ"; };
  template <> struct euler_traits<EulerAxes::kRZYX> { static inline const std::string Name = "RZYX"; };
  template <> struct euler_traits<EulerAxes::kRXYX> { static inline const std::string Name = "RXYX"; };
  template <> struct euler_traits<EulerAxes::kRYZX> { static inline const std::string Name = "RYZX"; };
  template <> struct euler_traits<EulerAxes::kRXZX> { static inline const std::string Name = "RXZX"; };
  template <> struct euler_traits<EulerAxes::kRXZY> { static inline const std::string Name = "RXZY"; };
  template <> struct euler_traits<EulerAxes::kRYZY> { static inline const std::string Name = "RYZY"; };
  template <> struct euler_traits<EulerAxes::kRZXY> { static inline const std::string Name = "RZXY"; };
  template <> struct euler_traits<EulerAxes::kRYXY> { static inline const std::string Name = "RYXY"; };
  template <> struct euler_traits<EulerAxes::kRYXZ> { static inline const std::string Name = "RYXZ"; };
  template <> struct euler_traits<EulerAxes::kRZXZ> { static inline const std::string Name = "RZXZ"; };
  template <> struct euler_traits<EulerAxes::kRXYZ> { static inline const std::string Name = "RXYZ"; };
  template <> struct euler_traits<EulerAxes::kRZYZ> { static inline const std::string Name = "RZYZ"; };
} // end namespace internal

/** \returns the respective EulerAxes for the given string.
  * \throws InvalidEulerAxesException if the string is invalid.
  */
EulerAxes eulerAxesFromStr(const std::string& str)
{
  if (str == internal::euler_traits<EulerAxes::kSXYZ>::Name) { return EulerAxes::kSXYZ; }
  if (str == internal::euler_traits<EulerAxes::kSXYX>::Name) { return EulerAxes::kSXYX; }
  if (str == internal::euler_traits<EulerAxes::kSXZY>::Name) { return EulerAxes::kSXZY; }
  if (str == internal::euler_traits<EulerAxes::kSXZX>::Name) { return EulerAxes::kSXZX; }
  if (str == internal::euler_traits<EulerAxes::kSYZX>::Name) { return EulerAxes::kSYZX; }
  if (str == internal::euler_traits<EulerAxes::kSYZY>::Name) { return EulerAxes::kSYZY; }
  if (str == internal::euler_traits<EulerAxes::kSYXZ>::Name) { return EulerAxes::kSYXZ; }
  if (str == internal::euler_traits<EulerAxes::kSYXY>::Name) { return EulerAxes::kSYXY; }
  if (str == internal::euler_traits<EulerAxes::kSZXY>::Name) { return EulerAxes::kSZXY; }
  if (str == internal::euler_traits<EulerAxes::kSZXZ>::Name) { return EulerAxes::kSZXZ; }
  if (str == internal::euler_traits<EulerAxes::kSZYX>::Name) { return EulerAxes::kSZYX; }
  if (str == internal::euler_traits<EulerAxes::kSZYZ>::Name) { return EulerAxes::kSZYZ; }
  if (str == internal::euler_traits<EulerAxes::kRZYX>::Name) { return EulerAxes::kRZYX; }
  if (str == internal::euler_traits<EulerAxes::kRXYX>::Name) { return EulerAxes::kRXYX; }
  if (str == internal::euler_traits<EulerAxes::kRYZX>::Name) { return EulerAxes::kRYZX; }
  if (str == internal::euler_traits<EulerAxes::kRXZX>::Name) { return EulerAxes::kRXZX; }
  if (str == internal::euler_traits<EulerAxes::kRXZY>::Name) { return EulerAxes::kRXZY; }
  if (str == internal::euler_traits<EulerAxes::kRYZY>::Name) { return EulerAxes::kRYZY; }
  if (str == internal::euler_traits<EulerAxes::kRZXY>::Name) { return EulerAxes::kRZXY; }
  if (str == internal::euler_traits<EulerAxes::kRYXY>::Name) { return EulerAxes::kRYXY; }
  if (str == internal::euler_traits<EulerAxes::kRYXZ>::Name) { return EulerAxes::kRYXZ; }
  if (str == internal::euler_traits<EulerAxes::kRZXZ>::Name) { return EulerAxes::kRZXZ; }
  if (str == internal::euler_traits<EulerAxes::kRXYZ>::Name) { return EulerAxes::kRXYZ; }
  if (str == internal::euler_traits<EulerAxes::kRZYZ>::Name) { return EulerAxes::kRZYZ; }
  throw InvalidEulerAxesException();
}

/** \returns the respective string for the EulerAxes <I>axes</I>.
  * \throws InvalidEulerAxesException if <I>axes</I> is invalid.
  */
std::string eulerAxesToStr(const EulerAxes &axes)
{
  switch (axes)
  {
    case EulerAxes::kSXYZ: return internal::euler_traits<EulerAxes::kSXYZ>::Name;
    case EulerAxes::kSXYX: return internal::euler_traits<EulerAxes::kSXYX>::Name;
    case EulerAxes::kSXZY: return internal::euler_traits<EulerAxes::kSXZY>::Name;
    case EulerAxes::kSXZX: return internal::euler_traits<EulerAxes::kSXZX>::Name;
    case EulerAxes::kSYZX: return internal::euler_traits<EulerAxes::kSYZX>::Name;
    case EulerAxes::kSYZY: return internal::euler_traits<EulerAxes::kSYZY>::Name;
    case EulerAxes::kSYXZ: return internal::euler_traits<EulerAxes::kSYXZ>::Name;
    case EulerAxes::kSYXY: return internal::euler_traits<EulerAxes::kSYXY>::Name;
    case EulerAxes::kSZXY: return internal::euler_traits<EulerAxes::kSZXY>::Name;
    case EulerAxes::kSZXZ: return internal::euler_traits<EulerAxes::kSZXZ>::Name;
    case EulerAxes::kSZYX: return internal::euler_traits<EulerAxes::kSZYX>::Name;
    case EulerAxes::kSZYZ: return internal::euler_traits<EulerAxes::kSZYZ>::Name;
    case EulerAxes::kRZYX: return internal::euler_traits<EulerAxes::kRZYX>::Name;
    case EulerAxes::kRXYX: return internal::euler_traits<EulerAxes::kRXYX>::Name;
    case EulerAxes::kRYZX: return internal::euler_traits<EulerAxes::kRYZX>::Name;
    case EulerAxes::kRXZX: return internal::euler_traits<EulerAxes::kRXZX>::Name;
    case EulerAxes::kRXZY: return internal::euler_traits<EulerAxes::kRXZY>::Name;
    case EulerAxes::kRYZY: return internal::euler_traits<EulerAxes::kRYZY>::Name;
    case EulerAxes::kRZXY: return internal::euler_traits<EulerAxes::kRZXY>::Name;
    case EulerAxes::kRYXY: return internal::euler_traits<EulerAxes::kRYXY>::Name;
    case EulerAxes::kRYXZ: return internal::euler_traits<EulerAxes::kRYXZ>::Name;
    case EulerAxes::kRZXZ: return internal::euler_traits<EulerAxes::kRZXZ>::Name;
    case EulerAxes::kRXYZ: return internal::euler_traits<EulerAxes::kRXYZ>::Name;
    case EulerAxes::kRZYZ: return internal::euler_traits<EulerAxes::kRZYZ>::Name;
  }
  throw InvalidEulerAxesException();
}

/** \returns the respective EulerAxes for the <CODE>uint8_t</CODE> <I>value</I>.
  * \throws InvalidEulerAxesException if the <I>value</I> is invalid.
  */
EulerAxes eulerAxesFromUInt8(std::uint8_t value, bool unsafe = false)
{
  if (!unsafe)
  {
    switch (value)
    {
      case static_cast<std::uint8_t>(EulerAxes::kSXYZ):
      case static_cast<std::uint8_t>(EulerAxes::kSXYX):
      case static_cast<std::uint8_t>(EulerAxes::kSXZY):
      case static_cast<std::uint8_t>(EulerAxes::kSXZX):
      case static_cast<std::uint8_t>(EulerAxes::kSYZX):
      case static_cast<std::uint8_t>(EulerAxes::kSYZY):
      case static_cast<std::uint8_t>(EulerAxes::kSYXZ):
      case static_cast<std::uint8_t>(EulerAxes::kSYXY):
      case static_cast<std::uint8_t>(EulerAxes::kSZXY):
      case static_cast<std::uint8_t>(EulerAxes::kSZXZ):
      case static_cast<std::uint8_t>(EulerAxes::kSZYX):
      case static_cast<std::uint8_t>(EulerAxes::kSZYZ):
      case static_cast<std::uint8_t>(EulerAxes::kRZYX):
      case static_cast<std::uint8_t>(EulerAxes::kRXYX):
      case static_cast<std::uint8_t>(EulerAxes::kRYZX):
      case static_cast<std::uint8_t>(EulerAxes::kRXZX):
      case static_cast<std::uint8_t>(EulerAxes::kRXZY):
      case static_cast<std::uint8_t>(EulerAxes::kRYZY):
      case static_cast<std::uint8_t>(EulerAxes::kRZXY):
      case static_cast<std::uint8_t>(EulerAxes::kRYXY):
      case static_cast<std::uint8_t>(EulerAxes::kRYXZ):
      case static_cast<std::uint8_t>(EulerAxes::kRZXZ):
      case static_cast<std::uint8_t>(EulerAxes::kRXYZ):
      case static_cast<std::uint8_t>(EulerAxes::kRZYZ):
        break;
      default:
        throw InvalidEulerAxesException();
    }
  }
  return static_cast<EulerAxes>(value);
}

/** \returns the respective EulerAxes for the <CODE>double</CODE> <I>value</I>.
  * \throws InvalidEulerAxesException if the <I>value</I> is invalid.
  * \sa eulerAxesFromUInt8()
  */
EulerAxes eulerAxesFromDouble(double value, bool unsafe = false)
{
  if (!unsafe && (value < static_cast<double>(std::numeric_limits<std::uint8_t>::min()) || 
                  value > static_cast<double>(std::numeric_limits<std::uint8_t>::max())))
  {
    throw InvalidEulerAxesException();
  }
  return eulerAxesFromUInt8(static_cast<std::uint8_t>(value));
}

/** \returns the respective <CODE>double</CODE> for the EulerAxes <I>axes</I>. */
double eulerAxesToDouble(const EulerAxes &axes)
{
  return static_cast<std::uint8_t>(axes);
}

/** \returns the EulerAxes description coefficients of <I>axes</I>. */
void eulerAxesToTuple(const EulerAxes &axes, std::uint8_t &firstaxis, std::uint8_t &parity, std::uint8_t &repetition, std::uint8_t &frame)
{
  auto value = static_cast<std::uint8_t>(axes);
  // NOLINTBEGIN(*magic-numbers)
  firstaxis = (value & (3 << 6)) >> 6;
  parity = (value & (3 << 4)) >> 4;
  repetition = (value & (3 << 2)) >> 2;
  frame = (value & 3);
  // NOLINTEND(*magic-numbers)
}


namespace euler
{

constexpr std::array<size_t, 4> EulerNextAxis {1, 2, 0, 1};

/** \returns the rotation matrix for Euler angles <I>ai</I>, <I>aj</I>, and <I>ak</I> in <I>rad</I> and Euler axes order <I>axes</I>
  * \note The implementation is based on <CODE>transforms3d.euler.euler2mat</CODE>:
  *       https://github.com/matthew-brett/transforms3d/blob/master/transforms3d/euler.py
  */
template<typename Scalar>
Eigen::Matrix<Scalar, 3, 3> eulerToMatrix(Scalar ai, Scalar aj, Scalar ak, EulerAxes axes)
{
  std::uint8_t firstaxis = 0;
  std::uint8_t parity = 0;
  std::uint8_t repetition = 0;
  std::uint8_t frame = 0;
  motion3d::eulerAxesToTuple(axes, firstaxis, parity, repetition, frame);

  std::size_t i = firstaxis;
  std::size_t j = EulerNextAxis.at(i + parity);
  std::size_t k = EulerNextAxis.at(i - parity + 1);

  if (frame > 0)
  {
    std::swap(ai, ak);
  }
  if (parity > 0)
  {
    ai *= -1;
    aj *= -1;
    ak *= -1;
  }

  Scalar si = std::sin(ai);
  Scalar sj = std::sin(aj);
  Scalar sk = std::sin(ak);
  Scalar ci = std::cos(ai);
  Scalar cj = std::cos(aj);
  Scalar ck = std::cos(ak);
  Scalar cc = ci*ck;
  Scalar cs = ci*sk;
  Scalar sc = si*ck;
  Scalar ss = si*sk;

  Eigen::Matrix<Scalar, 3, 3> matrix;
  if (repetition > 0)
  {
    matrix(i, i) = cj;
    matrix(i, j) = sj*si;
    matrix(i, k) = sj*ci;
    matrix(j, i) = sj*sk;
    matrix(j, j) = -cj*ss+cc;
    matrix(j, k) = -cj*cs-sc;
    matrix(k, i) = -sj*ck;
    matrix(k, j) = cj*sc+cs;
    matrix(k, k) = cj*cc-ss;
  }
  else
  {
    matrix(i, i) = cj*ck;
    matrix(i, j) = sj*sc-cs;
    matrix(i, k) = sj*cc+ss;
    matrix(j, i) = cj*sk;
    matrix(j, j) = sj*ss+cc;
    matrix(j, k) = sj*cs-sc;
    matrix(k, i) = -sj;
    matrix(k, j) = cj*si;
    matrix(k, k) = cj*ci;
  }
  return matrix;
}

/** \returns the Euler angles <I>ai</I>, <I>aj</I>, and <I>ak</I> in <I>rad</I> for a rotation matrix <I>M</I> and Euler axes order <I>axes</I>.
  * \note The implementation is based on <CODE>transforms3d.euler.mat2euler</CODE>:
  *       https://github.com/matthew-brett/transforms3d/blob/master/transforms3d/euler.py
  */
template<typename Scalar>
std::tuple<Scalar, Scalar, Scalar> matrixToEuler(const Eigen::Matrix<Scalar, 3, 3>& M, EulerAxes axes)
{
  std::uint8_t firstaxis = 0;
  std::uint8_t parity = 0;
  std::uint8_t repetition = 0;
  std::uint8_t frame = 0;
  motion3d::eulerAxesToTuple(axes, firstaxis, parity, repetition, frame);

  std::size_t i = firstaxis;
  std::size_t j = EulerNextAxis.at(i + parity);
  std::size_t k = EulerNextAxis.at(i - parity + 1);

  Scalar ax = 0;
  Scalar ay = 0;
  Scalar az = 0;
  if (repetition > 0)
  {
    Scalar sy = std::sqrt(M(i, j)*M(i, j) + M(i, k)*M(i, k));
    if (sy > std::numeric_limits<Scalar>::epsilon() * 4)
    {
      ax = std::atan2( M(i, j),  M(i, k));
      ay = std::atan2( sy,       M(i, i));
      az = std::atan2( M(j, i), -M(k, i));
    }
    else
    {
      ax = std::atan2(-M(j, k),  M(j, j));
      ay = std::atan2( sy,       M(i, i));
      az = 0.0;
    }
  }
  else
  {
    Scalar cy = std::sqrt(M(i, i)*M(i, i) + M(j, i)*M(j, i));
    if (cy > std::numeric_limits<Scalar>::epsilon() * 4)
    {
      ax = std::atan2( M(k, j),  M(k, k));
      ay = std::atan2(-M(k, i),  cy);
      az = std::atan2( M(j, i),  M(i, i));
    }
    else
    {
      ax = std::atan2(-M(j, k),  M(j, j));
      ay = std::atan2(-M(k, i),  cy);
      az = 0.0;
    }
  }

  if (parity > 0)
  {
    ax *= -1;
    ay *= -1;
    az *= -1;
  }

  if (frame > 0)
  {
    std::swap(ax, az);
  }

  return std::make_tuple(ax, ay, az);
}


/** \returns the rotation quaternion for Euler angles <I>ai</I>, <I>aj</I>, and <I>ak</I> in <I>rad</I> and Euler axis order <I>axes</I>.
  * \note The implementation is based on <CODE>transforms3d.euler.euler2quat</CODE>:
  *       https://github.com/matthew-brett/transforms3d/blob/master/transforms3d/euler.py
  */
template<typename Scalar>
Quaternion<Scalar> eulerToQuaternion(Scalar ai, Scalar aj, Scalar ak, EulerAxes axes)
{
  std::uint8_t firstaxis = 0;
  std::uint8_t parity = 0;
  std::uint8_t repetition = 0;
  std::uint8_t frame = 0;
  motion3d::eulerAxesToTuple(axes, firstaxis, parity, repetition, frame);

  std::size_t i = firstaxis + 1;
  std::size_t j = EulerNextAxis.at(i + parity - 1) + 1;
  std::size_t k = EulerNextAxis.at(i - parity) + 1;

  if (frame > 0)
  {
    std::swap(ai, ak);
  }
  if (parity > 0)
  {
    aj *= -1;
  }

  ai /= 2.0;  // NOLINT(*magic-numbers)
  aj /= 2.0;  // NOLINT(*magic-numbers)
  ak /= 2.0;  // NOLINT(*magic-numbers)
  Scalar ci = std::cos(ai);
  Scalar si = std::sin(ai);
  Scalar cj = std::cos(aj);
  Scalar sj = std::sin(aj);
  Scalar ck = std::cos(ak);
  Scalar sk = std::sin(ak);
  Scalar cc = ci*ck;
  Scalar cs = ci*sk;
  Scalar sc = si*ck;
  Scalar ss = si*sk;

  std::array<Scalar, 4> q {0, 0, 0, 0};
  if (repetition > 0)
  {
    q[0] = cj * (cc - ss);
    q.at(i) = cj * (cs + sc);
    q.at(j) = sj * (cc + ss);
    q.at(k) = sj * (cs - sc);
  }
  else
  {
    q[0] = cj*cc + sj*ss;
    q.at(i) = cj*sc - sj*cs;
    q.at(j) = cj*ss + sj*cc;
    q.at(k) = cj*cs - sj*sc;
  }

  if (parity > 0)
  {
    q.at(j) *= -1.0;
  }

  Quaternion<Scalar> quaternion(q[0], q[1], q[2], q[3]);
  return quaternion;
}

} // end namespace euler

} // namespace motion3d
