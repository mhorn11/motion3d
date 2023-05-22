#pragma once

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include <motion3d/io/motion_data.hpp>
#include <motion3d/io/utils.hpp>
#include <motion3d/utils/templating.hpp>

namespace motion3d
{

/** \brief Motion3d (M3D) file format writer.
  * \note The implementation is based on <CODE>pcl::PCDWriter</CODE>: https://pointclouds.org/documentation/classpcl_1_1_p_c_d_writer.html
  */
class M3DWriter
{
 public:
  static constexpr int kDefaultASCIIPrecision = 16;

  M3DWriter() = delete;

  /** \brief Generates the header of a M3D file format
    *
    * \param[in] motion the motion data
    * \param[in] binary generate binary header without origin scalars
    *
    * \returns the header as string.
    */
  static std::string generateHeader(const MotionData &motion, bool binary);

  /** \brief Saves motion data to a M3D file in either ASCII or binary format.
    *
    * \param[in] file_name the output file name
    * \param[in] motion the motion data
    * \param[in] file_type output file type
    * \param[in] precision the specified output numeric stream precision (optional parameter)
    *
    * \returns status
    */
  static M3DIOStatus write(const std::string &file_name, const MotionData &motion,
                           const M3DFileType &file_type, int precision = kDefaultASCIIPrecision);

  /** \brief Saves motion data to a M3D file in ASCII format.
    *
    * \param[in] file_name the output file name
    * \param[in] motion the motion data
    * \param[in] precision the specified output numeric stream precision (optional parameter)
    *
    * \returns status
    */
  static M3DIOStatus writeASCII(const std::string &file_name, const MotionData &motion,
                                int precision = kDefaultASCIIPrecision);

  /** \brief Saves motion data to a M3D file in binary format.
    *
    * \param[in] file_name the output file name
    * \param[in] motion the motion data
    *
    * \returns status
    */
  static M3DIOStatus writeBinary(const std::string &file_name, const MotionData &motion);
};


std::string M3DWriter::generateHeader(const MotionData &motion, const bool binary)
{
  // open string stream
  std::ostringstream oss;
  oss.imbue(std::locale::classic());

  oss << "# .M3D v" << kM3DFileFormatVersion << " - Motion3D file format\n"
         "VERSION " << kM3DFileFormatVersion << "\n";

  std::ostringstream stream;
  stream.imbue(std::locale::classic());

  if (!motion.frame_id_.empty())
  {
    oss << "FRAME " << motion.frame_id_ << "\n";
  }

  if (motion.getTransformType())
  {
    oss << "TYPE " << transformTypeToChar(*(motion.getTransformType())) << "\n";
  }

  if (motion.transforms_)
  {
    char has_stamps = motion.transforms_->hasStamps() ? '1' : '0';
    oss << "STAMPS " << has_stamps << "\n";

    char has_poses = motion.transforms_->hasPoses() ? '1' : '0';
    oss << "POSES " << has_poses << "\n";
  }

  if (motion.origin_ && motion.getTransformType())
  {
    if (binary)
    {
      oss << "ORIGIN 1\n";
    }
    else
    {
      TransformInterface::Ptr origin = motion.origin_->asType(*(motion.getTransformType()));
      std::vector<double> values = origin->toVector();
      oss << "ORIGIN ";
      for (std::size_t d = 0; d < values.size(); ++d)
      {
        oss << values.at(d);
        if (d < values.size() - 1)
        {
          oss << " ";
        }
      }
      oss << "\n";
    }
  }

  if (motion.transforms_)
  {
    oss << "SIZE " << motion.transforms_->size() << "\n";
  }

  return oss.str();
}


M3DIOStatus M3DWriter::write(const std::string &file_name, const MotionData &motion, const M3DFileType &file_type, const int precision)
{
  switch (file_type)
  {
    case M3DFileType::kASCII: return writeASCII(file_name, motion, precision);
    case M3DFileType::kBinary: return writeBinary(file_name, motion);
  }
  return M3DIOStatus::kUnsupportedFileType;
}


M3DIOStatus M3DWriter::writeASCII(const std::string &file_name, const MotionData &motion, const int precision)
{
  // check transforms
  if (!motion.transforms_)
  {
    return M3DIOStatus::kNoTransforms;
  }
  if (motion.transforms_->empty())
  {
    return M3DIOStatus::kNoTransforms;
  }
  
  // check transform type
  if (!motion.getTransformType())
  {
    return M3DIOStatus::kNoTransformType;
  }
  TransformType transform_type = *(motion.getTransformType());

  // open stream
  std::ofstream fs;
  fs.precision(precision);
  fs.imbue(std::locale::classic ());
  fs.open(file_name.c_str ());
  if (!fs.is_open() || fs.fail())
  {
    return M3DIOStatus::kFileOpenError;
  }

  // write the header information
  fs << generateHeader(motion, false) << "DATA ascii\n";

  std::ostringstream stream;
  stream.precision(precision);
  stream.imbue(std::locale::classic());

  // iterate through the transforms
  if (motion.transforms_->hasStamps())
  {
    for (auto it = motion.transforms_->cbegin_items(); it != motion.transforms_->cend_items(); ++it)
    {
      // write stamp
      stream << it->first << " ";

      // write values
      std::vector<double> values = it->second->asType(transform_type)->toVector();
      for (std::size_t d = 0; d < values.size(); ++d)
      {
        stream << values.at(d);
        if (d < values.size() - 1)
        {
          stream << " ";
        }
      }
    
      // copy the stream, trim it, and write it to disk
      std::string result = stream.str();
      trim(result);
      stream.str("");
      fs << result << "\n";
    }
  }
  else
  {
    for (const auto &it : *motion.transforms_)
    {
      // write values
      std::vector<double> values = it->asType(transform_type)->toVector();
      for (std::size_t d = 0; d < values.size(); ++d)
      {
        stream << values.at(d);
        if (d < values.size() - 1)
        {
          stream << " ";
        }
      }
    
      // copy the stream, trim it, and write it to disk
      std::string result = stream.str();
      trim(result);
      stream.str("");
      fs << result << "\n";
    }
  }

  // close file
  fs.close();

  return M3DIOStatus::kSuccess;
}


M3DIOStatus M3DWriter::writeBinary(const std::string &file_name, const MotionData &motion)
{
  // check transforms
  if (!motion.transforms_)
  {
    return M3DIOStatus::kNoTransforms;
  }
  if (motion.transforms_->empty())
  {
    return M3DIOStatus::kNoTransforms;
  }

  // check transform type
  if (!motion.getTransformType())
  {
    return M3DIOStatus::kNoTransformType;
  }
  TransformType transform_type = *(motion.getTransformType());

  // open stream
  std::ofstream fs;
  fs.imbue(std::locale::classic ());
  fs.open(file_name.c_str ());
  if (!fs.is_open() || fs.fail())
  {
    return M3DIOStatus::kFileOpenError;
  }

  // calculate binary output size
  std::uint32_t binary_transform_size = getBinarySize(transform_type);
  std::uint32_t binary_sample_size = binary_transform_size;
  if (motion.transforms_->hasStamps())
  {
    binary_sample_size += Time::kBinarySize;
  }
  std::size_t binary_vector_size = binary_sample_size * motion.transforms_->size();
  if (motion.origin_)
  {
    binary_vector_size += binary_transform_size;
  }

  // create vector for binary output
  std::vector<std::uint8_t> binary(binary_vector_size);
  auto binary_it = binary.begin();

  // define copy function
  auto copyIntoBinary = [&] (const std::vector<std::uint8_t> &input) {
    std::copy(input.cbegin(), input.cend(), binary_it);
    std::advance(binary_it, input.size());
  };

  // convert origin
  if (motion.origin_)
  {
    std::vector<std::uint8_t> origin_binary = motion.origin_->asType(transform_type)->toBinary();
    copyIntoBinary(origin_binary);
  }

  // convert transforms
  if (motion.transforms_->hasStamps())
  {
    for (auto it = motion.transforms_->cbegin_items(); it != motion.transforms_->cend_items(); ++it)
    {
      std::vector<std::uint8_t> stamp_binary = it->first.toBinary();
      std::vector<std::uint8_t> data_binary = it->second->asType(transform_type)->toBinary();
      copyIntoBinary(stamp_binary);
      copyIntoBinary(data_binary);
    }
  }
  else
  {
    for (const auto &it : *motion.transforms_)
    {
      std::vector<std::uint8_t> data_binary = it->asType(transform_type)->toBinary();
      copyIntoBinary(data_binary);
    }
  }
  
  // write the header information
  fs << generateHeader(motion, true) << "DATA binary\n";

  // write binary data
  fs.write(reinterpret_cast<char*>(binary.data()),  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
           static_cast<std::streamsize>(binary.size() * sizeof(std::uint8_t)));

  // close file
  fs.close();

  return M3DIOStatus::kSuccess;
}


} // namespace motion3d
