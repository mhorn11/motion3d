#pragma once

#include <string>
#include <vector>

#include <motion3d/containers.hpp>
#include <motion3d/transforms.hpp>

namespace motion3d
{

// forward declarations
class M3DReader;
class M3DWriter;

/** \brief Storage class for motion data.
  */
class MotionData
{
  friend class M3DReader;
  friend class M3DWriter;

 public:
  /** Constructs motion data without initializing the transform type or the transforms. */
  explicit MotionData(std::string frame_id = "");

  /** Constructs motion data and initializes the transform type and the transforms. */
  MotionData(const TransformType &type, const TransformContainer::ConstPtr &transforms, std::string frame_id = "");

  /** Constructs motion data and initializes the transform type, the transforms and the origin. */
  MotionData(const TransformType &type, 
    const TransformContainer::ConstPtr &transforms, 
    const TransformInterface::ConstPtr &origin, 
    std::string frame_id = "");

  const std::string& getFrameId() const { return frame_id_; }
  std::shared_ptr<const TransformType> getTransformType() const { return transform_type_; }
  TransformInterface::ConstPtr getOrigin() const { return origin_; }
  TransformContainer::ConstPtr getTransforms() const { return transforms_; }

  void setFrameId(const std::string &frame_id) { frame_id_ = frame_id; }
  void setTransformType(const TransformType &type) { transform_type_ = std::make_shared<TransformType>(type); }
  void setOrigin(const TransformInterface::ConstPtr &origin) { origin_ = origin->copy(); }
  void setTransforms(const TransformContainer::ConstPtr &transforms) { transforms_ = std::make_shared<TransformContainer>(*transforms); }

  /** \returns a description of <CODE>*this</CODE>. */
  std::string desc() const { return streamToString(*this); }
  
  /** Inserts a description of <CODE>*this</CODE>. */
  friend std::ostream& operator<<(std::ostream& os, const MotionData& data);

 private:
  std::shared_ptr<TransformType> transform_type_;  ///< Transform type used for exporting the transforms
  TransformInterface::Ptr origin_;                 ///< Coordinate frame origin
  TransformContainer::Ptr transforms_;             ///< Container with all transforms
  std::string frame_id_;                           ///< Coordinate frame identifier
}; // class MotionFile


MotionData::MotionData(std::string frame_id)
  : frame_id_(std::move(frame_id))
{
}

MotionData::MotionData(const TransformType &type, const TransformContainer::ConstPtr &transforms, std::string frame_id)
  : transform_type_(std::make_shared<TransformType>(type)), transforms_(std::make_shared<TransformContainer>(*transforms)), frame_id_(std::move(frame_id))
{
}

MotionData::MotionData(
  const TransformType &type,
  const TransformContainer::ConstPtr &transforms,
  const TransformInterface::ConstPtr &origin,
  std::string frame_id)
  : transform_type_(std::make_shared<TransformType>(type)), transforms_(std::make_shared<TransformContainer>(*transforms)), frame_id_(std::move(frame_id))
{
  if (origin)
  {
    origin_ = origin->copy();
  }
}

std::ostream& operator<<(std::ostream& os, const MotionData& data)
{
  // transform type
  std::stringstream transform_type_ss;
  if (data.transform_type_)
  {
    transform_type_ss << transformTypeToChar(*(data.transform_type_));
  }
  else
  {
    transform_type_ss << "undefined";
  }

  // transforms
  std::stringstream transforms_ss;
  if (data.transforms_)
  {
    transforms_ss << data.transforms_->size();
  }
  else
  {
    transforms_ss << "undefined";
  }

  // insert
  return os << "MotionData(" << 
    "transform_type=" << transform_type_ss.str() << ", " <<
    "has_origin=" << (data.origin_ ? "true" : "false") << ", " <<
    "transforms=" << transforms_ss.str() << ", " <<
    "frame_id='" << data.frame_id_ << "')";
}

} // namespace motion3d
