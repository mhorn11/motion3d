#pragma once

#include <motion3d/containers/transform_container.hpp>

namespace motion3d
{

template<class TransformClass>
std::vector<TransformClass> TransformContainer::toVector() const
{
  std::vector<TransformClass> output;
  output.reserve(size());

  for (const auto &it : *this)
  {
    output.push_back(static_cast<TransformClass>(*(it->asType<TransformClass>())));
  }
  return output;
}

std::vector<TransformContainer::DataType> TransformContainer::toVector() const
{
  std::vector<DataType> output;
  output.reserve(size());

  for (const auto &it : *this)
  {
    output.push_back(it->copy());
  }
  return output;
}

std::vector<TransformContainer::DataType> TransformContainer::toVector(const TransformType &type) const
{
  std::vector<DataType> output;
  output.reserve(size());

  for (const auto &it : *this)
  {
    output.push_back(it->asType(type));
  }
  return output;
}

template<class TransformClass>
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> TransformContainer::toEigenVector() const
{
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> output(
    size(), internal::traits<TransformClass>::kVectorSize);

  Eigen::Index index = 0;
  for (const auto &it : *this)
  {
    output.row(index) = it->asType<TransformClass>()->toEigenVector();
    ++index;
  }
  return output;
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> TransformContainer::toEigenVector(const TransformType &type) const
{
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> output(
    size(), getVectorSize(type));

  Eigen::Index index = 0;
  for (const auto &it : *this)
  {
    output.row(index) = it->asType(type)->toEigenVector();
    ++index;
  }
  return output;
}

std::ostream& operator<<(std::ostream& os, const TransformContainer& container)
{
  return os << "TransformContainer(" <<
            "has_stamps=" << (container.hasStamps() ? "true" : "false") << ", " <<
            "has_poses=" << (container.hasPoses() ? "true" : "false") << ", " <<
            "size=" << container.size() << ")";
}

} // namespace motion3d
