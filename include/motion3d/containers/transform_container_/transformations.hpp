#pragma once

#include <motion3d/containers/transform_container.hpp>

namespace motion3d
{

///////////////////////////////////////////////////////////////////////////////
/// Inplace Implementation
///////////////////////////////////////////////////////////////////////////////

TransformContainer& TransformContainer::removeStamps_()
{
  if (hasStamps())
  {
    vector().reserve(map().size());
    for (auto const& [stamp, transform] : map())
    {
      vector().push_back(transform);
    }
    has_stamps_ = false;
    map().clear();
  }
  return *this;
}

TransformContainer& TransformContainer::addStamps_(const std::vector<StampType> &stamps)
{
  if (hasStamps())
  {
    throw TransformContainerException("Container already has stamps");
  }
  if (stamps.size() != vector().size())
  {
    throw TransformContainerException("Size of stamps does not match vector size");
  }
  map() = ExtendedMap<StampType, DataType>(stamps, vector());
  has_stamps_ = true;
  vector().clear();
  return *this;
}

template<class TransformClass>
TransformContainer& TransformContainer::asType_()
{
  for (auto &it : *this)
  {
    it = it->asType<TransformClass>();
  }
  return *this;
}

TransformContainer& TransformContainer::asType_(const TransformType &type)
{
  for (auto &it : *this)
  {
    it = it->asType(type);
  }
  return *this;
}

TransformContainer& TransformContainer::asMotions_()
{
  if (hasStamps())
  {
    throw TransformContainerException("Cannot convert stamped container to motions, stamps would be unclear");
  }
  if (!has_poses_)
  {
    return *this;
  }
  if (size() > 1)
  {
    has_poses_ = false;
    auto it1 = begin();
    for (auto it2 = std::next(it1); it2 != end(); it1 = it2++)
    {
      *it1 =  it2->applyPre(it1->inverse());
    }
    erase(it1);
  }
  else
  {
    has_poses_ = false;
  }
  return *this;
}

TransformContainer& TransformContainer::asPoses_()
{
  return asPoses_(nullptr);
}

TransformContainer& TransformContainer::asPoses_(const TransformInterface::ConstPtr &initial_pose)
{
  if (hasStamps())
  {
    throw TransformContainerException("Cannot convert stamped container to poses, stamps would be unclear");
  }
  if (empty())
  {
    has_poses_ = true;
  }
  else if (has_poses_)
  {
    if (initial_pose && !initial_pose->isEqual(*cbegin()))
    {
      TransformInterface::Ptr pose_diff = initial_pose / *cbegin();
      applyPre_(pose_diff);
    }
  }
  else
  {
    motionsToPoses(initial_pose);
  }
  return *this;
}

TransformContainer& TransformContainer::inverse_()
{
  for (auto &it : *this)
  {
    it->inverse_();
  }
  return *this;
}

TransformContainer& TransformContainer::normalized_()
{
  for (auto &it : *this)
  {
    it->normalized_();
  }
  return *this;
}

TransformContainer& TransformContainer::scaleTranslation_(double factor)
{
  for (auto &it : *this)
  {
    it->scaleTranslation_(factor);
  }
  return *this;
}

template<class T>
TransformContainer& TransformContainer::applyPre_(const T &transform)
{
  for (auto &it : *this)
  {
    it->applyPre_(transform);
  }
  return *this;
}

template<class T>
TransformContainer& TransformContainer::applyPost_(const T &transform)
{
  for (auto &it : *this)
  {
    it->applyPost_(transform);
  }
  return *this;
}

template<class T1, class T2>
TransformContainer& TransformContainer::apply_(const T1 &transform_pre, const T2 &transform_post)
{
  for (auto &it : *this)
  {
    it->applyPre_(transform_pre)->applyPost_(transform_post);
  }
  return *this;
}

TransformContainer& TransformContainer::applyFunc_(
    const std::function<DataType(const DataType&)> &func)
{
  for (auto &it : *this)
  {
    it = func(it);
  }
  return *this;
}

TransformContainer& TransformContainer::applyIndexFunc_(
    const std::function<DataType(std::size_t, const DataType&)> &func)
{
  size_t index = 0;
  for (auto &it : *this)
  {
    it = func(index, it);
    ++index;
  }
  return *this;
}

TransformContainer& TransformContainer::applyStampFunc_(
    const std::function<DataType(const StampType&, const DataType&)> &func)
{
  for (auto it = begin_items(); it != end_items(); ++it)
  {
    it->second = func(it->first, it->second);
  }
  return *this;
}

template<class T>
TransformContainer& TransformContainer::changeFrame_(const T &transform)
{
  if constexpr (is_shared_ptr<T>::value)
  {
    return apply_(transform->inverse(), transform);
  }
  else
  {
    return apply_(transform.inverse(), transform);
  }
}


///////////////////////////////////////////////////////////////////////////////
/// Helper
///////////////////////////////////////////////////////////////////////////////

void TransformContainer::motionsToPoses(const TransformInterface::ConstPtr &initial_pose)
{
  if (hasStamps())
  {
    throw TransformContainerException("Cannot convert stamped motions to poses, stamps would be unclear");
  }

  // first pose
  DataType current_pose;
  if (!initial_pose)
  {
    current_pose = cbegin()->identity();  // use identity transform in case no initial pose is given
  }
  else
  {
    current_pose = initial_pose->copy();  // copy initial pose
  }

  // iterate
  has_poses_ = true;
  for (auto &it : *this)
  {
    // current_pose: new pose P_{t} for index t
    // it: current motion M_{t} at index t
    DataType tmp = it->applyPre(current_pose);  // create new pose for index t+1: P_{t+1} = P_{t} * M_{t}
    it = current_pose;  // assign current_pose P_{t} to iterator location t
    current_pose = tmp;
  }
  append(current_pose);
}


///////////////////////////////////////////////////////////////////////////////
/// Copy Implementation
///////////////////////////////////////////////////////////////////////////////

TransformContainer TransformContainer::removeStamps() const
{
  return TransformContainer(*this).removeStamps_();
}

TransformContainer TransformContainer::addStamps(const std::vector<StampType> &stamps) const
{
  return TransformContainer(*this).addStamps_(stamps);
}

template<class TransformClass>
TransformContainer TransformContainer::asType() const
{
  return TransformContainer(*this).asType_<TransformClass>();
}

TransformContainer TransformContainer::asType(const TransformType &type) const
{
  return TransformContainer(*this).asType_(type);
}

TransformContainer TransformContainer::asMotions() const
{
  return TransformContainer(*this).asMotions_();
}

TransformContainer TransformContainer::asPoses() const
{
  return TransformContainer(*this).asPoses_();
}

TransformContainer TransformContainer::asPoses(const TransformInterface::ConstPtr &initial_pose) const
{
  return TransformContainer(*this).asPoses_(initial_pose);
}

TransformContainer TransformContainer::inverse() const
{
  return TransformContainer(*this).inverse_();
}

TransformContainer TransformContainer::normalized() const
{
  return TransformContainer(*this).normalized_();
}

TransformContainer TransformContainer::scaleTranslation(double factor) const
{
  return TransformContainer(*this).scaleTranslation_(factor);
}

template<class T>
TransformContainer TransformContainer::applyPre(const T &transform) const
{
  return TransformContainer(*this).applyPre_<T>(transform);
}

template<class T>
TransformContainer TransformContainer::applyPost(const T &transform) const
{
  return TransformContainer(*this).applyPost_<T>(transform);
}

template<class T1, class T2>
TransformContainer TransformContainer::apply(const T1 &transform_pre, const T2 &transform_post) const
{
  return TransformContainer(*this).apply_<T1, T2>(transform_pre, transform_post);
}

TransformContainer TransformContainer::applyFunc(
    const std::function<DataType(const DataTypeConst&)> &func) const
{
  return TransformContainer(*this).applyFunc_(func);
}

TransformContainer TransformContainer::applyIndexFunc(
    const std::function<DataType(std::size_t, const DataTypeConst&)> &func) const
{
  return TransformContainer(*this).applyIndexFunc_(func);
}

TransformContainer TransformContainer::applyStampFunc(
    const std::function<DataType(const StampType&, const DataTypeConst&)> &func) const
{
  return TransformContainer(*this).applyStampFunc_(func);
}

template<class T>
TransformContainer TransformContainer::changeFrame(const T &transform) const
{
  return TransformContainer(*this).changeFrame_<T>(transform);
}

} // namespace motion3d
