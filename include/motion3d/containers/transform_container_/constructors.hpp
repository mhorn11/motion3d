#pragma once

#include <motion3d/containers/transform_container.hpp>

namespace motion3d
{

TransformContainer::TransformContainer(bool has_stamps, bool has_poses)
  : has_stamps_(has_stamps)
  , has_poses_(has_poses)
{
}

template<class T>
TransformContainer::TransformContainer(const std::vector<T> &transforms, bool has_poses)
  : TransformContainer(false, has_poses)
{
  vector().reserve(transforms.size());
  for (const T& trafo : transforms)
  {
    if constexpr (std::is_same<DataType, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
    {
      if (trafo == nullptr)
      {
        throw std::invalid_argument("Adding null pointers to motion3d::TransformContainer is not allowed");
      }
      vector().push_back(trafo->copy());
    }
    else if constexpr (std::is_base_of<TransformInterface, T>::value)
    {
      vector().push_back(std::make_shared<T>(trafo));
    }
    else
    {
      static_assert(always_false<T>, "Input must contain transforms");
    }
  }
}

template<class T>
TransformContainer::TransformContainer(
  const std::map<StampType, T> &transforms,
  bool has_poses)
  : TransformContainer(true, has_poses)
{
  for (auto const& [stamp, transform] : transforms)
  {
    if constexpr (std::is_same<DataType, T>::value || std::is_same<TransformInterface::ConstPtr, T>::value)
    {
      if (transform == nullptr)
      {
        throw std::invalid_argument("Adding null pointers to motion3d::TransformContainer is not allowed");
      }
      map()[stamp] = transform->copy();
    }
    else if constexpr (std::is_base_of<TransformInterface, T>::value)
    {
      map()[stamp] = std::make_shared<T>(transform);
    }
    else
    {
      static_assert(always_false<T>, "Input must contain transforms");
    }
  }
}

template<class T>
TransformContainer::TransformContainer(
  const std::vector<StampType> &stamps,
  const std::vector<T> &transforms,
  bool has_poses,
  const bool sorted_data)
  : TransformContainer(true, has_poses)
{
  using StampIt = typename std::vector<StampType>::const_iterator;
  using TIt = typename std::vector<T>::const_iterator;

  // check sizes
  if (stamps.size() != transforms.size())
  {
    throw std::invalid_argument("The sizes of keys and values must be equal");
  }

  // copy function
  std::function<DataType(const TIt&)> copy_fun;
  if constexpr (std::is_same<DataType, T>::value || std::is_same<DataTypeConst, T>::value)
  {
    copy_fun = [](const TIt &it) {
      if (*it == nullptr)
      {
        throw std::invalid_argument("Adding null pointers to motion3d::TransformContainer is not allowed");
      }
      return (*it)->copy();
    };
  }
  else if constexpr (std::is_base_of<TransformInterface, T>::value)
  {
    copy_fun = [](const TIt &it) {
      return std::make_shared<T>(*it);
    };
  }
  else
  {
    static_assert(always_false<T>, "Input must contain transforms");
  }

  // insert function
  std::function<void(const StampIt&, const DataType&)> insert_fun;
  if (sorted_data)
  {
    insert_fun = [this](const StampIt &stamp_it, const DataType &transform_ptr) {
      this->map().insert(this->map().end(), {*stamp_it, transform_ptr});
    };
  }
  else
  {
    insert_fun = [this](const StampIt &stamp_it, const DataType &transform_ptr) {
      this->map()[*stamp_it] = transform_ptr;
    };
  }

  // iterate data
  auto stamp_it = stamps.cbegin();
  auto transform_it = transforms.cbegin();
  for (; stamp_it != stamps.cend(); ++stamp_it, ++transform_it)
  {
    insert_fun(stamp_it, copy_fun(transform_it));
  }
}

TransformContainer::TransformContainer(const TransformContainer &other)
  : TransformContainer(other.hasStamps(), other.hasPoses())
{
  if (other.hasStamps())
  {
    for (auto const& [stamp, transform] : other.map())
    {
      this->map()[stamp] = transform->copy();
    }
  }
  else
  {
    this->vector().reserve(other.vector().size());
    for (auto const& transform : other.vector())
    {
      this->vector().push_back(transform->copy());
    }
  }
}

} // namespace motion3d
