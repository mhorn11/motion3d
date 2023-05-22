#pragma once

#include <cstdint>

namespace motion3d::internal
{

template<typename T> struct traits;
template<typename T> struct traits<const T> : traits<T> {};

} // namespace motion3d::internal
