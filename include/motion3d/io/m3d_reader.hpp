#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <vector>

#include <motion3d/io/motion_data.hpp>
#include <motion3d/io/utils.hpp>

namespace motion3d
{

/** \brief Motion3d (M3D) file format reader.
  * \note The implementation is based on <CODE>pcl::PCDReader</CODE>: https://pointclouds.org/documentation/classpcl_1_1_p_c_d_reader.html
  */
class M3DReader
{
 public:
  M3DReader() = delete;

  /** \brief Reads a motion data header from a M3D-formatted, binary istream.
    *
    * Load only the meta information (number of transforms, their types, etc),
    * and not the transforms themselves, from a given M3D stream. Useful for fast
    * evaluation of the underlying data structure.
    *
    * \param[in]  fs a <CODE>std::istream</CODE> with openmode set to std::ios::binary
    * \param[out] motion the resultant motion data (only the header data will be filled)
    * \param[out] data_size the number of transforms
    * \param[out] file_type the type of data
    * \param[out] data_idx the offset of transform data within the file
    * \param[in]  unsafe read origin transform as unsafe (optional parameter)
    *
    * \returns status
    */
  static M3DIOStatus readHeader(
    std::istream &fs,
    MotionData &motion,
    unsigned int &data_size,
    M3DFileType &file_type,
    unsigned int &data_idx,
    bool unsafe = false);

  /** \brief Reads a motion data header from a M3D file.
    *
    * Load only the meta information (number of transforms, their types, etc),
    * and not the transforms themselves, from a given M3D stream. Useful for fast
    * evaluation of the underlying data structure.
    *
    * \param[in]  file_name the name of the file to load
    * \param[out] motion the resultant motion data (only the header data will be filled)
    * \param[out] data_size the number of transforms
    * \param[out] file_type the type of data
    * \param[out] data_idx the offset of transform data within the file
    * \param[in]  unsafe read origin transform as unsafe (optional parameter)
    * \param[in]  offset the offset of where to expect the M3D Header in the file (optional parameter)
    *
    * \returns status
    */
  static M3DIOStatus readHeader(
    const std::string &file_name,
    MotionData &motion,
    unsigned int &data_size,
    M3DFileType &file_type,
    unsigned int &data_idx,
    bool unsafe = false,
    int offset = 0);

  /** \brief Reads the ASCII motion data body from a M3D stream.
    *
    * Reads the transforms from a text-formatted stream. For use after
    * readHeader(), when the resulting file_type is ASCII.
    *
    * \attention This assumes the stream has been seeked to the position
    * indicated by the data_idx result of readHeader().
    *
    * \param[in]  fs the stream from which to read the body
    * \param[out] motion the resultant motion data to be filled
    * \param[out] data_size the number of transforms
    * \param[in]  unsafe read transform as unsafe (optional parameter)
    *
    * \returns status
    */
  static M3DIOStatus readBodyASCII(std::istream &fs, MotionData &motion, unsigned int data_size, bool unsafe = false);

  /** \brief Reads the binary motion data body from a M3D stream.
    *
    * Reads the transforms from a binary-formatted memory block. For use
    * after readHeader(), when the resulting file_type is binary.
    *
    * \attention This assumes the stream has been seeked to the position
    * indicated by the data_idx result of readHeader().
    *
    * \param[in]  fs the stream from which to read the body
    * \param[out] motion the resultant motion data to be filled
    * \param[out] data_size the number of transforms
    * \param[in]  unsafe read transform as unsafe (optional parameter)
    *
    * \returns status
    */
  static M3DIOStatus readBodyBinary(std::istream &fs, MotionData &motion, unsigned int data_size, bool unsafe = false);

  /** \brief Reads motion data from a M3D file.
    *
    * \param[in]  file_name the name of the file to load
    * \param[out] motion the resultant motion data to be filled
    * \param[in]  unsafe read transforms as unsafe (optional parameter)
    * \param[in]  offset the offset of where to expect the M3D Header in the file (optional parameter)
    *
    * \returns status
    */
  static M3DIOStatus read(const std::string &file_name, MotionData &motion, bool unsafe = false, int offset = 0);

  /** \brief Reads motion data from a M3D file.
    *
    * \param[in]  file_name the name of the file to load
    * \param[out] result the read status
    * \param[in]  unsafe read transforms as unsafe (optional parameter)
    * \param[in]  offset the offset of where to expect the M3D Header in the file (optional parameter)
    *
    * \returns an <CODE>std::optional</CODE> with the resultant motion data in case of success.
    */
  static std::optional<MotionData> read(const std::string &file_name, M3DIOStatus &result, bool unsafe = false, int offset = 0);

  /** \brief Reads motion data from a M3D file.
    *
    * \param[in]  file_name the name of the file to load
    * \param[in]  unsafe read transforms as unsafe (optional parameter)
    * \param[in]  offset the offset of where to expect the M3D Header in the file (optional parameter)
    *
    * \returns an <CODE>std::optional</CODE> with the resultant motion data in case of success.
    */
  static std::optional<MotionData> read(const std::string &file_name, bool unsafe = false, int offset = 0);
};

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
M3DIOStatus M3DReader::readHeader(
  std::istream &fs,
  MotionData &motion,
  unsigned int &data_size,
  M3DFileType &file_type,
  unsigned int &data_idx,
  const bool unsafe)
{
  // default values
  data_size = 0;
  file_type = M3DFileType::kASCII;
  data_idx = 0;
  bool has_stamps = false;
  bool has_poses = false;
  motion.frame_id_ = "";
  motion.transform_type_.reset();
  motion.origin_.reset();
  motion.transforms_.reset();

  std::vector<double> origin_values;

  std::string line;
  std::vector<std::string> st;

  // read the header and fill it in with values
  while (!fs.eof())
  {
    getline(fs, line);

    // trim and tokenize
    trim(line);
    if (line.empty())
    {
      continue;
    }
    tokenize(line, st);

    std::stringstream sstream(line);
    sstream.imbue(std::locale::classic ());

    std::string line_type;
    sstream >> line_type;

    // version numbers are not needed for now, but we are checking to see if they're there
    if (line_type.rfind("VERSION", 0) == 0)
    {
      continue;
    }

    // frame identifier
    if (line_type.rfind("FRAME", 0) == 0)
    {
      sstream >> motion.frame_id_;
      if (sstream.fail())
      {
        return M3DIOStatus::kInvalidFrame;
      }
      continue;
    }

    // type
    if (line_type.rfind("TYPE", 0) == 0)
    {
      char transform_type = 0;
      sstream >> transform_type;
      if (sstream.fail())
      {
        return M3DIOStatus::kInvalidType;
      }
      try
      {
        motion.setTransformType(motion3d::transformTypeFromChar(transform_type));
      }
      catch (const InvalidTransformTypeException&)
      {
        return M3DIOStatus::kInvalidTransformType;
      }
      continue;
    }

    // stamps
    if (line_type.rfind("STAMPS", 0) == 0)
    {
      sstream >> has_stamps;
      if (sstream.fail())
      {
        return M3DIOStatus::kInvalidStamps;
      }
      continue;
    }

    // poses
    if (line_type.rfind("POSES", 0) == 0)
    {
      sstream >> has_poses;
      if (sstream.fail())
      {
        return M3DIOStatus::kInvalidPoses;
      }
      continue;
    }

    // origin
    if (line_type.rfind("ORIGIN", 0) == 0)
    {
      if (st.size() > 1)
      {
        origin_values.resize(st.size() - 1);
        for (std::size_t d = 1; d < st.size(); ++d)
        {
          origin_values.at(d - 1) = std::stod(st.at(d));
        }
      }
      continue;
    }

    // size
    if (line_type.rfind("SIZE", 0) == 0)
    {
      sstream >> data_size;
      if (sstream.fail())
      {
        return M3DIOStatus::kInvalidSize;
      }
      continue;
    }

    // read the header + comments line by line until we get to <DATA>
    if (line_type.rfind("DATA", 0) == 0)
    {
      data_idx = static_cast<int>(fs.tellg());
      if (st.size() > 1)
      {
        if (st.at(1).rfind("binary", 0) == 0)
        {
          file_type = M3DFileType::kBinary;
        }
      }
      break;
    }
    break;
  }

  // exit early if no transform type has been given
  if (!motion.getTransformType())
  {
    return M3DIOStatus::kNoTransformType;
  }

  // exit early if no transforms have been given
  if (data_size == 0)
  {
    return M3DIOStatus::kNoTransforms;
  }

  // create origin transform from origin values
  if (!origin_values.empty())
  {
    if (file_type == M3DFileType::kASCII)
    {
      // create origin transform from values
      motion.origin_ = TransformInterface::Factory(*(motion.getTransformType()), origin_values, unsafe);
    }
    else
    {
      // create identity transform as origin to signalize the following method that the data contain an origin
      motion.origin_ = TransformInterface::Factory(*(motion.getTransformType()));
    }

  }

  // initialize transform container
  motion.transforms_ = std::make_shared<TransformContainer>(has_stamps, has_poses);
  
  return M3DIOStatus::kSuccess;
}

M3DIOStatus M3DReader::readHeader(
  const std::string &file_name,
  MotionData &motion,
  unsigned int &data_size,
  M3DFileType &file_type,
  unsigned int &data_idx,
  const bool unsafe,
  const int offset)
{
  if (file_name.empty() || !std::filesystem::exists(file_name))
  {
    return M3DIOStatus::kFileNotFound;
  }

  // open file in binary mode to avoid problem of
  // std::getline() corrupting the result of ifstream::tellg()
  std::ifstream fs;
  fs.open(file_name.c_str(), std::ios::binary);
  if (!fs.is_open() || fs.fail())
  {
    fs.close();
    return M3DIOStatus::kFileOpenError;
  }

  // seek at the given offset
  fs.seekg(offset, std::ios::beg);

  // delegate parsing to the istream overload
  M3DIOStatus result = readHeader(fs, motion, data_size, file_type, data_idx, unsafe);

  // close file
  fs.close();

  return result;
}

M3DIOStatus M3DReader::readBodyASCII(std::istream &fs, MotionData &motion, unsigned int data_size, const bool unsafe)
{
  // check transforms
  if (!motion.transforms_)
  {
    return M3DIOStatus::kNoTransforms;
  }

  // check transform type
  if (!motion.getTransformType())
  {
    return M3DIOStatus::kNoTransformType;
  }
  TransformType transform_type = *(motion.getTransformType());
  std::uint32_t vector_size = getVectorSize(transform_type);

  // clear output
  motion.transforms_->clear();

  // iterate data
  unsigned int idx = 0;
  std::string line;
  std::vector<std::string> st;
  std::istringstream is;
  is.imbue (std::locale::classic());

  while (idx < data_size && !fs.eof())
  {
    getline (fs, line);

    // trim and tokenize
    trim(line);
    if (line.empty())
    {
      continue;
    }
    tokenize(line, st);

    if (idx >= data_size)
    {
      return M3DIOStatus::kMoreTransforms;
    }

    if (motion.transforms_->hasStamps())
    {
      // check data
      if (st.size() != vector_size + 1)
      {
        return M3DIOStatus::kInvalidTransform;
      }

      // copy data
      motion3d::Time stamp(st.at(0));
      std::vector<double> values(st.size() - 1);
      for (unsigned int d = 1; d < st.size(); ++d)
      {
        values.at(d - 1) = std::stod(st.at(d));
      }

      // create transform
      motion.transforms_->insert(stamp, TransformInterface::Factory(transform_type, values, unsafe));
    }
    else
    {
      // check data
      if (st.size() != vector_size)
      {
        return M3DIOStatus::kInvalidTransform;
      }

      // copy data
      std::vector<double> values(st.size());
      for (unsigned int d = 0; d < st.size(); ++d)
      {
        values.at(d) = std::stod(st.at(d));
      }

      // create transform
      motion.transforms_->append(TransformInterface::Factory(transform_type, values, unsafe));
    }

    idx++;
  }

  if (idx < data_size)
  {
    return M3DIOStatus::kLessTransforms;
  }

  return M3DIOStatus::kSuccess;
}

M3DIOStatus M3DReader::readBodyBinary(std::istream &fs, MotionData &motion, unsigned int data_size, const bool unsafe)
{
  // check transforms
  if (!motion.transforms_)
  {
    return M3DIOStatus::kNoTransforms;
  }

  // check transform type
  if (!motion.getTransformType())
  {
    return M3DIOStatus::kNoTransformType;
  }
  TransformType transform_type = *(motion.getTransformType());

  // clear output
  motion.transforms_->clear();

  // read data
  std::vector<std::uint8_t> data((std::istreambuf_iterator<char>(fs)), std::istreambuf_iterator<char>());

  // check binary size and transform count
  std::uint32_t binary_transform_size = getBinarySize(transform_type);
  std::uint32_t binary_size = binary_transform_size;
  if (motion.transforms_->hasStamps())
  {
    binary_size += Time::kBinarySize;
  }

  // check binary size
  auto data_size_no_origin = static_cast<std::int64_t>(data.size());
  if (motion.origin_)
  {
    // remove origin size from data size for the following check
    data_size_no_origin -= binary_transform_size;
  }

  auto transform_count_div = std::div(data_size_no_origin, static_cast<std::int64_t>(binary_size));
  if (transform_count_div.rem != 0)
  {
    return M3DIOStatus::kInvalidBinarySize; 
  }
  if (transform_count_div.quot < data_size)
  {
    return M3DIOStatus::kLessTransforms;
  }
  if (transform_count_div.quot > data_size)
  {
    return M3DIOStatus::kMoreTransforms;
  }

  // read origin
  std::size_t start_idx = 0;
  if (motion.origin_)
  {
    std::vector<std::uint8_t> transform_binary(&data[start_idx], &data[start_idx + binary_transform_size]);
    motion.origin_ = TransformInterface::Factory(transform_type, transform_binary, unsafe);
    start_idx += binary_transform_size;
  }

  // iterate transform data
  for (; start_idx < data.size(); start_idx += binary_size)
  {
    if (motion.transforms_->hasStamps())
    {
      std::vector<std::uint8_t> stamp_binary(&data[start_idx], &data[start_idx + Time::kBinarySize]);
      std::vector<std::uint8_t> transform_binary(&data[start_idx + Time::kBinarySize], &data[start_idx + binary_size]);
      motion.transforms_->insert(Time(stamp_binary), TransformInterface::Factory(transform_type, transform_binary, unsafe));
    }
    else
    {
      std::vector<std::uint8_t> transform_binary(&data[start_idx], &data[start_idx + binary_size]);
      motion.transforms_->append(TransformInterface::Factory(transform_type, transform_binary, unsafe));
    }
  }

  return M3DIOStatus::kSuccess;
}

M3DIOStatus M3DReader::read(const std::string &file_name, MotionData &motion, const bool unsafe, const int offset)
{
  if (file_name.empty() || !std::filesystem::exists(file_name))
  {
    return M3DIOStatus::kFileNotFound;
  }

  unsigned int data_size = 0;
  M3DFileType file_type = M3DFileType::kASCII;
  unsigned int data_idx = 0;

  M3DIOStatus res = readHeader(file_name, motion, data_size, file_type, data_idx, unsafe, offset);

  if (res != M3DIOStatus::kSuccess)
  {
    return res;
  }

  // ascii
  if (file_type == M3DFileType::kASCII)
  {
    // re-open the file (readHeader closes it)
    std::ifstream fs;
    fs.open(file_name.c_str());
    if (!fs.is_open() || fs.fail())
    {
      fs.close();
      return M3DIOStatus::kFileOpenError;
    }

    fs.seekg(data_idx + offset);

    // read the rest of the file
    res = readBodyASCII(fs, motion, data_size, unsafe);

    // close file
    fs.close();
  }
  // binary
  else if (file_type == M3DFileType::kBinary)
  {
    // re-open the file (readHeader closes it)
    std::ifstream fs;
    fs.open(file_name.c_str());
    if (!fs.is_open() || fs.fail())
    {
      fs.close();
      return M3DIOStatus::kFileOpenError;
    }

    fs.seekg(data_idx + offset);

    // read the rest of the file
    res = readBodyBinary(fs, motion, data_size, unsafe);

    // close file
    fs.close();
  }
  else
  {
    return M3DIOStatus::kUnsupportedFileType;
  }

  return res;
}

std::optional<MotionData> M3DReader::read(const std::string &file_name, M3DIOStatus &result, const bool unsafe, const int offset)
{
  MotionData motion;
  result = read(file_name, motion, unsafe, offset);
  if (result == M3DIOStatus::kSuccess)
  {
    return motion;
  }
  return std::nullopt;
}

std::optional<MotionData> M3DReader::read(const std::string &file_name, const bool unsafe, const int offset)
{
  M3DIOStatus result = M3DIOStatus::kSuccess;
  return read(file_name, result, unsafe, offset);
}

} // namespace motion3d
