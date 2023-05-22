#pragma once

#include <motion3d/containers/transform_container.hpp>

namespace motion3d
{

std::size_t TransformContainer::size() const
{
  if (has_stamps_)
  {
    return map_.size();
  }
  return vector_.size();
}

bool TransformContainer::empty() const
{
  if (has_stamps_)
  {
    return map_.empty();
  }
  return vector_.empty();
}

TransformContainer::DataType& TransformContainer::at(const std::size_t &index)
{
  if (has_stamps_)
  {
    if (index >= size())
    {
      throw std::out_of_range("Given index is out of range");
    }
    auto it = map_.begin_values();
    std::advance(it, index);
    return *it;
  }
  return vector_.at(index);
}

TransformContainer::DataTypeConst TransformContainer::at(const std::size_t &index) const
{
  if (has_stamps_)
  {
    if (index >= size())
    {
      throw std::out_of_range("Given index is out of range");
    }
    auto it = map_.cbegin_values();
    std::advance(it, index);
    return *it;
  }
  return vector_.at(index);
}

TransformContainer::DataType& TransformContainer::at_stamp(const StampType &stamp)
{
  auto it = find_eq(stamp);
  if (it == end_items())
  {
    throw std::out_of_range("Given stamp does not exist");
  }
  return it->second;
}

TransformContainer::DataTypeConst TransformContainer::at_stamp(const StampType &stamp) const
{
  auto it = cfind_eq(stamp);
  if (it == cend_items())
  {
    throw std::out_of_range("Given stamp does not exist");
  }
  return it->second;
}

TransformContainer::StampType TransformContainer::stamp_at(const std::size_t &index) const
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  if (index >= size())
  {
    throw std::out_of_range("Given index is out of range");
  }
  auto it = map_.cbegin();
  std::advance(it, index);
  return it->first;
}

std::pair<const TransformContainer::StampType, TransformContainer::DataType>&
  TransformContainer::item_at(const std::size_t &index)
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  if (index >= size())
  {
    throw std::out_of_range("Given index is out of range");
  }
  auto it = map_.begin();
  std::advance(it, index);
  return *it;
}

std::pair<const TransformContainer::StampType, TransformContainer::DataTypeConst>
  TransformContainer::item_at(const std::size_t &index) const
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  if (index >= size())
  {
    throw std::out_of_range("Given index is out of range");
  }
  auto it = map_.cbegin();
  std::advance(it, index);
  return *it;
}

void TransformContainer::append(const DataTypeConst &transform)
{
  if (has_stamps_)
  {
    throw TransformContainerException("This method is only valid for unstamped containers");
  }
  if (transform == nullptr)
  {
    throw std::invalid_argument("Adding null pointers to motion3d::TransformContainer is not allowed");
  }
  vector_.push_back(transform->copy());
}

void TransformContainer::append(const StampType &stamp, const DataTypeConst &transform)
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  if (transform == nullptr)
  {
    throw std::invalid_argument("Adding null pointers to motion3d::TransformContainer is not allowed");
  }
  map_.insert(map_.end(), {stamp, transform->copy()});
}

void TransformContainer::insert(const std::size_t &index, const DataTypeConst &transform)
{
  if (has_stamps_)
  {
    throw TransformContainerException("This method is only valid for unstamped containers");
  }
  if (index > size())
  {
    throw std::out_of_range("Given index is out of range");
  }
  if (transform == nullptr)
  {
    throw std::invalid_argument("Adding null pointers to motion3d::TransformContainer is not allowed");
  }
  auto it = vector_.begin();
  std::advance(it, index);
  vector_.insert(it, transform->copy());
}

bool TransformContainer::insert(
  const StampType &stamp,
  const DataTypeConst &transform,
  const bool overwrite)
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  if (transform == nullptr)
  {
    throw std::invalid_argument("Adding null pointers to motion3d::TransformContainer is not allowed");
  }
  if (overwrite)
  {
    map_[stamp] = transform->copy();
    return true;
  }
  return map_.insert({stamp, transform->copy()}).second;
}

template<class T>
void TransformContainer::extend(const std::vector<T> &other)
{
  static_assert(std::is_same<T, DataType>::value || std::is_same<T, DataTypeConst>::value,
    "T must be a TransformInterface pointer");

  if (has_stamps_)
  {
    throw TransformContainerException("This method is only valid for unstamped containers");
  }
  std::size_t this_size = vector_.size();
  std::size_t other_size = other.size();
  vector_.reserve(this_size + other_size);
  for (auto const& other_trafo : other)
  {
    if (other_trafo == nullptr)
    {
      throw std::invalid_argument("Adding null pointers to motion3d::TransformContainer is not allowed");
    }
    vector_.push_back(other_trafo->copy());
  }
}

template<class T>
void TransformContainer::extend(const std::map<StampType, T> &other, const bool overwrite)
{
  static_assert(std::is_same<T, DataType>::value || std::is_same<T, DataTypeConst>::value,
    "T must be a TransformInterface pointer");

  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }

  // insert function
  using MapValueType = typename std::map<StampType, T>::value_type;
  std::function<void(const MapValueType&)> insert_fun;
  if (overwrite)
  {
    insert_fun = [this](const MapValueType& v) {
      this->map()[v.first] = v.second->copy();
    };
  }
  else
  {
    insert_fun = [this](const MapValueType& v) {
      this->map().insert({v.first, v.second->copy()});
    };
  }

  // iterate data
  for (const auto &it : other)
  {
    if (it.second == nullptr)
    {
      throw std::invalid_argument("Adding null pointers to motion3d::TransformContainer is not allowed");
    }
    insert_fun(it);
  }
}

void TransformContainer::extend(const TransformContainer &other, const bool overwrite)
{
  if (this->has_poses_ != other.has_poses_)
  {
    throw TransformContainerException("This and other container must both have either motions or poses");
  }

  if (this->has_stamps_ && other.has_stamps_)
  {
    this->extend(other.map_, overwrite);
  }
  else if (!this->has_stamps_ && !other.has_stamps_)
  {
    this->extend(other.vector_);
  }
  else
  {
    throw TransformContainerException("This and other container must be both stamped or both unstamped");
  }
}

void TransformContainer::erase(const std::size_t &index)
{
  if (index >= size())
  {
    throw std::out_of_range("Given index is out of range");
  }
  auto it = begin();
  std::advance(it, index);
  erase(it);
}

void TransformContainer::erase(const StampType &stamp)
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  map_.erase(stamp);
}

void TransformContainer::erase(iterator position)
{
  if (has_stamps_)
  {
    map_.erase(position.map_it_);
  }
  else
  {
    vector_.erase(position.vector_it_);
  }
}

void TransformContainer::clear()
{
  map_.clear();
  vector_.clear();
}

} // namespace motion3d
