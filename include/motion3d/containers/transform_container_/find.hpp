#pragma once

#include <motion3d/containers/transform_container.hpp>

namespace motion3d
{

TransformContainer::item_iterator TransformContainer::find_eq(const StampType &stamp)
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  return map_.find_eq(stamp);
}

TransformContainer::const_item_iterator TransformContainer::cfind_eq(const StampType &stamp) const
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  return const_item_iterator(map_.find_eq(stamp));
}

TransformContainer::item_iterator TransformContainer::find_ge(const StampType &stamp)
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  return map_.find_ge(stamp);
}

TransformContainer::const_item_iterator TransformContainer::cfind_ge(const StampType &stamp) const
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  return const_item_iterator(map_.find_ge(stamp));
}

TransformContainer::item_iterator TransformContainer::find_gt(const StampType &stamp)
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  return map_.find_gt(stamp);
}

TransformContainer::const_item_iterator TransformContainer::cfind_gt(const StampType &stamp) const
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  return const_item_iterator(map_.find_gt(stamp));
}

TransformContainer::item_iterator TransformContainer::find_le(const StampType &stamp)
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  return map_.find_le(stamp);
}

TransformContainer::const_item_iterator TransformContainer::cfind_le(const StampType &stamp) const
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  return const_item_iterator(map_.find_le(stamp));
}

TransformContainer::item_iterator TransformContainer::find_lt(const StampType &stamp)
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  return map_.find_lt(stamp);
}

TransformContainer::const_item_iterator TransformContainer::cfind_lt(const StampType &stamp) const
{
  if (!has_stamps_)
  {
    throw TransformContainerException("This method is only valid for stamped containers");
  }
  return const_item_iterator(map_.find_lt(stamp));
}

TransformContainer::item_iterator TransformContainer::find_closest(const StampType &stamp)
{
  // get next stamp in both directions
  auto it_lt = find_lt(stamp);
  auto it_ge = find_ge(stamp);

  // check stamps
  if (it_lt == end_items())
  {
    return it_ge;
  }
  if (it_ge == end_items())
  {
    return it_lt;
  }

  // compare absolute time difference
  Time stamp_diff_lt = timeDiffAbs(stamp, it_lt->first);
  Time stamp_diff_ge = timeDiffAbs(stamp, it_ge->first);
  if (stamp_diff_lt > stamp_diff_ge)
  {
    return it_ge;
  }
  return it_lt;
}

TransformContainer::const_item_iterator TransformContainer::cfind_closest(const StampType &stamp) const
{
  // get next stamp in both directions
  auto it_lt = find_lt(stamp);
  auto it_ge = find_ge(stamp);

  // check stamps
  if (it_lt == cend_items())
  {
    return it_ge;
  }
  if (it_ge == cend_items())
  {
    return it_lt;
  }

  // compare absolute time difference
  Time stamp_diff_lt = timeDiffAbs(stamp, it_lt->first);
  Time stamp_diff_ge = timeDiffAbs(stamp, it_ge->first);
  if (stamp_diff_lt > stamp_diff_ge)
  {
    return it_ge;
  }
  return it_lt;
}

} // namespace motion3d
