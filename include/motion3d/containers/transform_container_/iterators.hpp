#pragma once

#include <motion3d/containers/transform_container.hpp>

namespace motion3d
{

///////////////////////////////////////////////////////////////////////////////
/// Iterator
///////////////////////////////////////////////////////////////////////////////

TransformContainer::iterator::iterator(VectorType::iterator vector_it) 
  : vector_it_(vector_it)
  , is_map_(false)
{
}

TransformContainer::iterator::iterator(MapType::iterator map_it) 
  : map_it_(map_it)
  , is_map_(true)
{
}

TransformContainer::iterator::reference TransformContainer::iterator::operator*() const
{
  if (is_map_)
  {
    return map_it_->second;
  }
  return *vector_it_;
}

TransformContainer::iterator::pointer TransformContainer::iterator::operator->() const
{
  return iterator::operator*();
}

TransformContainer::iterator& TransformContainer::iterator::operator++()
{
  if (is_map_)
  {
    map_it_++;
  }
  else
  {
    vector_it_++;
  }
  return *this;
}

TransformContainer::iterator TransformContainer::iterator::operator++(int)
{
  iterator tmp = *this;
  ++(*this);
  return tmp;
}

bool operator==(const TransformContainer::iterator &a, const TransformContainer::iterator &b)
{
  if (a.is_map_ && b.is_map_)
  {
    return a.map_it_ == b.map_it_;
  }
  if (!a.is_map_ && !b.is_map_)
  {
    return a.vector_it_ == b.vector_it_;
  }
  return false;
};

bool operator!=(const TransformContainer::iterator &a, const TransformContainer::iterator &b)
{
  return !(a == b);
};


///////////////////////////////////////////////////////////////////////////////
/// Const Iterator
///////////////////////////////////////////////////////////////////////////////

TransformContainer::const_iterator::const_iterator(VectorType::const_iterator vector_it) 
  : vector_it_(vector_it)
  , is_map_(false)
{
}

TransformContainer::const_iterator::const_iterator(MapType::const_iterator map_it) 
  : map_it_(map_it)
  , is_map_(true)
{
}

TransformContainer::const_iterator::reference TransformContainer::const_iterator::operator*() const
{
  if (is_map_)
  {
    return map_it_->second;
  }
  return *vector_it_;
}

TransformContainer::const_iterator::pointer TransformContainer::const_iterator::operator->() const
{
  return const_iterator::operator*();
}

TransformContainer::const_iterator& TransformContainer::const_iterator::operator++()
{
  if (is_map_)
  {
    map_it_++;
  }
  else
  {
    vector_it_++;
  }
  return *this;
}

TransformContainer::const_iterator TransformContainer::const_iterator::operator++(int)
{
  const_iterator tmp = *this;
  ++(*this);
  return tmp;
}

bool operator==(const TransformContainer::const_iterator &a, const TransformContainer::const_iterator &b)
{
  if (a.is_map_ && b.is_map_)
  {
    return a.map_it_ == b.map_it_;
  }
  if (!a.is_map_ && !b.is_map_)
  {
    return a.vector_it_ == b.vector_it_;
  }
  return false;
};

bool operator!=(const TransformContainer::const_iterator &a, const TransformContainer::const_iterator &b)
{
  return !(a == b);
};


///////////////////////////////////////////////////////////////////////////////
/// Const Item Iterator
///////////////////////////////////////////////////////////////////////////////

TransformContainer::const_item_iterator::const_item_iterator(MapType::const_iterator map_it)
  : map_it_(map_it)
{
}

TransformContainer::const_item_iterator::reference TransformContainer::const_item_iterator::operator*() const
{
  return std::make_pair(map_it_->first, map_it_->second);
}

TransformContainer::const_item_iterator::pointer TransformContainer::const_item_iterator::operator->() const
{
  return reinterpret_cast<const_item_iterator::pointer>(map_it_.operator->());  // NOLINT(*-reinterpret-cast)
}

TransformContainer::const_item_iterator& TransformContainer::const_item_iterator::operator++()
{
  map_it_++;
  return *this;
}

TransformContainer::const_item_iterator TransformContainer::const_item_iterator::operator++(int)
{
  const_item_iterator tmp = *this;
  ++(*this);
  return tmp;
}

bool operator==(const TransformContainer::const_item_iterator &a, const TransformContainer::const_item_iterator &b)
{
  return a.map_it_ == b.map_it_;
};

bool operator!=(const TransformContainer::const_item_iterator &a, const TransformContainer::const_item_iterator &b)
{
  return !(a == b);
};


///////////////////////////////////////////////////////////////////////////////
/// Iterator Access
///////////////////////////////////////////////////////////////////////////////

TransformContainer::iterator TransformContainer::begin()
{
  if (has_stamps_)
  {
    return iterator(map_.begin());
  }
  return iterator(vector_.begin());
}

TransformContainer::const_iterator TransformContainer::cbegin() const
{
  if (has_stamps_)
  {
    return const_iterator(map_.cbegin());
  }
  return const_iterator(vector_.cbegin());
}

TransformContainer::iterator TransformContainer::end()
{
  if (has_stamps_)
  {
    return iterator(map_.end());
  }
  return iterator(vector_.end());
}

TransformContainer::const_iterator TransformContainer::cend() const
{
  if (has_stamps_)
  {
    return const_iterator(map_.cend());
  }
  return const_iterator(vector_.cend());
}

TransformContainer::const_stamp_iterator TransformContainer::cbegin_stamps() const
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  return map_.cbegin_keys();
}

TransformContainer::const_stamp_iterator TransformContainer::cend_stamps() const
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  return map_.cend_keys();
}

TransformContainer::item_iterator TransformContainer::begin_items()
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  return map_.begin();
}

TransformContainer::const_item_iterator TransformContainer::cbegin_items() const
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  return const_item_iterator(map_.cbegin());
}

TransformContainer::item_iterator TransformContainer::end_items()
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  return map_.end();
}

TransformContainer::const_item_iterator TransformContainer::cend_items() const
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  return const_item_iterator(map_.cend());
}

} // namespace motion3d
