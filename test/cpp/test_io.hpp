#pragma once

#include <filesystem>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <motion3d/io.hpp>
#include <motion3d/transforms.hpp>

namespace m3d = motion3d;

class IO : public ::testing::Test
{
 protected:
  inline static std::filesystem::path output_dir;

  static void SetUpTestCase()
  {
    // generate random suffix
    std::string suffix = std::to_string(rand() % 100000 + 100000);

    // create output directory
    output_dir = std::filesystem::temp_directory_path();
    output_dir.append("motion3d_" + suffix);
    if (!std::filesystem::exists(output_dir))
    {
      std::filesystem::create_directory(output_dir);
    }
  }

  static void TearDownTestCase()
  {
    // clear and remove output directory
    if (std::filesystem::exists(output_dir))
    {
      std::filesystem::remove_all(output_dir);
    }
    output_dir.clear();
  }

  /// \returns the absolute path of a file in the data directory.
  std::string getDataFile(const std::string &filename)
  {
    std::filesystem::path p(__FILE__);
    std::filesystem::path dir = p.parent_path().parent_path();
    dir.append("data");
    dir.append(filename);
    return dir.string();
  }

  /// \returns the absolute path of a file in the output directory.
  std::string getOutputFile(const std::string &filename)
  {
    std::filesystem::path dir(output_dir);
    dir.append(filename);
    return dir.string();
  }
};

TEST_F(IO, motionData)
{
  // data
  auto container = std::make_shared<m3d::TransformContainer>(false, false);
  container->append(std::make_shared<m3d::DualQuaternionTransform>());
  container->append(std::make_shared<m3d::MatrixTransform>());
  auto origin = std::make_shared<m3d::QuaternionTransform>();

  // empty constructor
  m3d::MotionData motion1;
  ASSERT_EQ(motion1.getFrameId(), "");
  ASSERT_TRUE(motion1.getTransformType() == nullptr);
  ASSERT_TRUE(motion1.getOrigin() == nullptr);
  ASSERT_TRUE(motion1.getTransforms() == nullptr);

  // frame id constructor
  m3d::MotionData motion2("frame");
  ASSERT_EQ(motion2.getFrameId(), "frame");
  ASSERT_TRUE(motion2.getTransformType() == nullptr);
  ASSERT_TRUE(motion2.getOrigin() == nullptr);
  ASSERT_TRUE(motion2.getTransforms() == nullptr);

  // transforms constructor
  m3d::MotionData motion3(m3d::TransformType::kAxisAngle, container);
  ASSERT_EQ(motion3.getFrameId(), "");
  ASSERT_EQ(*(motion3.getTransformType()), m3d::TransformType::kAxisAngle);
  ASSERT_TRUE(motion3.getOrigin() == nullptr);
  ASSERT_EQ(motion3.getTransforms()->size(), 2);

  // transforms and origin constructor
  m3d::MotionData motion4(m3d::TransformType::kAxisAngle, container, origin);
  ASSERT_EQ(motion4.getFrameId(), "");
  ASSERT_EQ(*(motion4.getTransformType()), m3d::TransformType::kAxisAngle);
  ASSERT_NE(motion4.getOrigin(), origin);
  ASSERT_TRUE(motion4.getOrigin()->isType(m3d::TransformType::kQuaternion));
  ASSERT_TRUE(motion4.getOrigin()->isEqual(origin));
  ASSERT_EQ(motion4.getTransforms()->size(), 2);

  // setter
  motion4.setFrameId("frame");
  ASSERT_EQ(motion2.getFrameId(), "frame");

  motion4.setTransformType(m3d::TransformType::kEuler);
  ASSERT_EQ(*(motion4.getTransformType()), m3d::TransformType::kEuler);

  motion4.setOrigin(std::make_shared<m3d::EulerTransform>());
  ASSERT_TRUE(motion4.getOrigin()->isType(m3d::TransformType::kEuler));

  motion4.setTransforms(std::make_shared<m3d::TransformContainer>(false, true));
  ASSERT_TRUE(motion4.getTransforms()->hasPoses());

  // stream
  std::stringstream ss;
  ss << motion4;
  ASSERT_STRING_STARTS_WITH(ss.str(), "MotionData");

  // description
  ASSERT_EQ(motion4.desc(), ss.str());
}

TEST_F(IO, readHeader)
{
  // ascii file
  m3d::MotionData motion1;
  unsigned int data_size1;
  m3d::M3DFileType file_type1;
  unsigned int data_idx1;
  m3d::M3DIOStatus return_code1 = m3d::M3DReader::readHeader(
    getDataFile("motion1.m3d"), motion1, data_size1, file_type1, data_idx1, true);

  ASSERT_EQ(return_code1, m3d::M3DIOStatus::kSuccess);
  ASSERT_EQ(motion1.getFrameId(), "camera1");
  ASSERT_EQ(data_size1, 3);
  ASSERT_TRUE(motion1.getTransformType() != nullptr);
  ASSERT_EQ(*(motion1.getTransformType()), m3d::TransformType::kAxisAngle);
  ASSERT_FALSE(motion1.getTransforms()->hasStamps());
  ASSERT_FALSE(motion1.getTransforms()->hasPoses());
  ASSERT_EQ(file_type1, m3d::M3DFileType::kASCII);
  ASSERT_EQ(data_idx1, 74);

  // ascii file with comments
  m3d::MotionData motion2;
  unsigned int data_size2;
  m3d::M3DFileType file_type2;
  unsigned int data_idx2;
  m3d::M3DIOStatus return_code2 = m3d::M3DReader::readHeader(
    getDataFile("motion2.m3d"), motion2, data_size2, file_type2, data_idx2, true);

  ASSERT_EQ(return_code2, m3d::M3DIOStatus::kSuccess);
  ASSERT_EQ(motion2.getFrameId(), "camera2");
  ASSERT_EQ(data_size2, 6);
  ASSERT_TRUE(motion2.getTransformType() != nullptr);
  ASSERT_EQ(*(motion2.getTransformType()), m3d::TransformType::kDualQuaternion);
  ASSERT_FALSE(motion2.getTransforms()->hasStamps());
  ASSERT_TRUE(motion2.getTransforms()->hasPoses());
  ASSERT_EQ(file_type2, m3d::M3DFileType::kASCII);
  ASSERT_EQ(data_idx2, 110);

  // ascii file with stamps
  m3d::MotionData motion3;
  unsigned int data_size3;
  m3d::M3DFileType file_type3;
  unsigned int data_idx3;
  m3d::M3DIOStatus return_code3 = m3d::M3DReader::readHeader(
    getDataFile("motion3.m3d"), motion3, data_size3, file_type3, data_idx3, true);

  ASSERT_EQ(return_code3, m3d::M3DIOStatus::kSuccess);
  ASSERT_EQ(motion3.getFrameId(), "camera3");
  ASSERT_EQ(data_size3, 3);
  ASSERT_TRUE(motion3.getTransformType() != nullptr);
  ASSERT_EQ(*(motion3.getTransformType()), m3d::TransformType::kAxisAngle);
  ASSERT_TRUE(motion3.getTransforms()->hasStamps());
  ASSERT_FALSE(motion3.getTransforms()->hasPoses());
  ASSERT_EQ(file_type3, m3d::M3DFileType::kASCII);
  ASSERT_EQ(data_idx3, 83);
}

TEST_F(IO, readInterface)
{
  // data as argument
  m3d::MotionData motion1;
  m3d::M3DIOStatus return_code1 = m3d::M3DReader::read(getDataFile("motion1.m3d"), motion1, true);
  ASSERT_EQ(return_code1, m3d::M3DIOStatus::kSuccess);

  m3d::M3DIOStatus return_code2 = m3d::M3DReader::read(getDataFile("motion_invalid.m3d"), motion1, true);
  ASSERT_NE(return_code2, m3d::M3DIOStatus::kSuccess);
  
  // data as return with result as argument
  m3d::M3DIOStatus return_code3;
  std::optional<m3d::MotionData> motion_opt1 = m3d::M3DReader::read(getDataFile("motion1.m3d"), return_code3, true);
  ASSERT_EQ(return_code3, m3d::M3DIOStatus::kSuccess);
  ASSERT_TRUE(motion_opt1.has_value());

  m3d::M3DIOStatus return_code4;
  std::optional<m3d::MotionData> motion_opt2 = m3d::M3DReader::read(getDataFile("motion_invalid.m3d"), return_code4, true);
  ASSERT_NE(return_code4, m3d::M3DIOStatus::kSuccess);
  ASSERT_FALSE(motion_opt2.has_value());

  // data as return
  std::optional<m3d::MotionData> motion_opt3 = m3d::M3DReader::read(getDataFile("motion1.m3d"), true);
  ASSERT_TRUE(motion_opt3.has_value());

  std::optional<m3d::MotionData> motion_opt4 = m3d::M3DReader::read(getDataFile("motion_invalid.m3d"), true);
  ASSERT_FALSE(motion_opt4.has_value());
}

TEST_F(IO, readASCIIData)
{
  // read data
  m3d::MotionData motion1;
  m3d::M3DIOStatus return_code1 = m3d::M3DReader::read(getDataFile("motion1.m3d"), motion1, true);
  ASSERT_EQ(return_code1, m3d::M3DIOStatus::kSuccess);

  // check types
  ASSERT_EQ(motion1.getFrameId(), "camera1");
  ASSERT_EQ(*(motion1.getTransformType()), m3d::TransformType::kAxisAngle);
  ASSERT_TRUE(motion1.getOrigin()->isType(m3d::TransformType::kAxisAngle));
  for (std::size_t i = 0; i < motion1.getTransforms()->size(); ++i)
  {
    ASSERT_TRUE(motion1.getTransforms()->at(i)->isType(m3d::TransformType::kAxisAngle));
  }

  // check origin
  std::vector<double> origin_values = motion1.getOrigin()->toVector();
  for (std::size_t v_idx = 0; v_idx < origin_values.size(); ++v_idx)
  {
    EXPECT_NEAR(origin_values.at(v_idx), v_idx + 1, 1e-12);
  }

  // check data
  ASSERT_EQ(motion1.getTransforms()->size(), 3);
  for (std::size_t t_idx = 0; t_idx < motion1.getTransforms()->size(); ++t_idx)
  {
    std::vector<double> values = motion1.getTransforms()->at(t_idx)->toVector();
    for (std::size_t v_idx = 0; v_idx < values.size(); ++v_idx)
    {
      EXPECT_NEAR(values.at(v_idx), (v_idx + 1) * std::pow(10.0, ((int)t_idx) - 1) + 1.0, 1e-12);
    }
  }

  // read data with stamps
  m3d::MotionData motion3;
  m3d::M3DIOStatus return_code3 = m3d::M3DReader::read(getDataFile("motion3.m3d"), motion3, true);
  ASSERT_EQ(return_code3, m3d::M3DIOStatus::kSuccess);

  // check types
  ASSERT_EQ(motion3.getFrameId(), "camera3");
  ASSERT_EQ(*(motion3.getTransformType()), m3d::TransformType::kAxisAngle);
  ASSERT_TRUE(motion3.getOrigin()->isType(m3d::TransformType::kAxisAngle));
  for (std::size_t i = 0; i < motion3.getTransforms()->size(); ++i)
  {
    ASSERT_TRUE(motion3.getTransforms()->at(i)->isType(m3d::TransformType::kAxisAngle));
  }

  // check data with stamps
  ASSERT_EQ(motion3.getTransforms()->size(), 3);
  for (std::size_t t_idx = 0; t_idx < motion3.getTransforms()->size(); ++t_idx)
  {
    std::uint64_t stamp = motion3.getTransforms()->stamp_at(t_idx).toNSec();
    EXPECT_EQ(stamp, (t_idx + 1) * 100);

    std::vector<double> values = motion3.getTransforms()->at(t_idx)->toVector();
    for (std::size_t v_idx = 0; v_idx < values.size(); ++v_idx)
    {
      EXPECT_NEAR(values.at(v_idx), (v_idx + 1) * std::pow(10.0, ((int)t_idx) - 1), 1e-12);
    }
  }
}

TEST_F(IO, writeAndRead)
{
  // filenames
  std::string data_file = getDataFile("motion1.m3d");
  std::string output_file_ascii = getOutputFile("motion1_ascii.m3d");
  std::string output_file_binary = getOutputFile("motion1_binary.m3d");

  // read data
  m3d::MotionData motion;
  m3d::M3DIOStatus return_code_reader = m3d::M3DReader::read(data_file, motion, true);
  ASSERT_EQ(return_code_reader, m3d::M3DIOStatus::kSuccess);

  // write data
  m3d::M3DIOStatus return_code_writer_ascii = m3d::M3DWriter::writeASCII(output_file_ascii, motion);
  ASSERT_EQ(return_code_writer_ascii, m3d::M3DIOStatus::kSuccess);

  m3d::M3DIOStatus return_code_writer_binary = m3d::M3DWriter::writeBinary(output_file_binary, motion);
  ASSERT_EQ(return_code_writer_binary, m3d::M3DIOStatus::kSuccess);

  // read data for checking
  m3d::MotionData motion_check_ascii;
  m3d::M3DIOStatus return_code_reader_check_ascii = m3d::M3DReader::read(output_file_ascii, motion_check_ascii, true);
  ASSERT_EQ(return_code_reader_check_ascii, m3d::M3DIOStatus::kSuccess);

  m3d::MotionData motion_check_binary;
  m3d::M3DIOStatus return_code_reader_check_binary = m3d::M3DReader::read(output_file_binary, motion_check_binary, true);
  ASSERT_EQ(return_code_reader_check_binary, m3d::M3DIOStatus::kSuccess);

  // check origin
  std::vector<double> origin_values = motion.getOrigin()->toVector();
  std::vector<double> origin_values_check_ascii = motion_check_ascii.getOrigin()->toVector();
  std::vector<double> origin_values_check_binary = motion_check_binary.getOrigin()->toVector();
  ASSERT_EQ(origin_values.size(), origin_values_check_ascii.size());
  ASSERT_EQ(origin_values.size(), origin_values_check_binary.size());
  for (std::size_t v_idx = 0; v_idx < origin_values.size(); ++v_idx)
  {
    EXPECT_EQ(origin_values.at(v_idx), origin_values_check_ascii.at(v_idx));
    EXPECT_EQ(origin_values.at(v_idx), origin_values_check_binary.at(v_idx));
  }

  // check data
  ASSERT_EQ(motion.getTransforms()->size(), motion_check_ascii.getTransforms()->size());
  ASSERT_EQ(motion.getTransforms()->size(), motion_check_binary.getTransforms()->size());
  for (std::size_t t_idx = 0; t_idx < motion.getTransforms()->size(); ++t_idx)
  {
    std::vector<double> values = motion.getTransforms()->at(t_idx)->toVector();
    std::vector<double> values_check_ascii = motion_check_ascii.getTransforms()->at(t_idx)->toVector();
    std::vector<double> values_check_binary = motion_check_binary.getTransforms()->at(t_idx)->toVector();
    ASSERT_EQ(values.size(), values_check_ascii.size());
    ASSERT_EQ(values.size(), values_check_binary.size());
    for (std::size_t v_idx = 0; v_idx < values.size(); ++v_idx)
    {
      EXPECT_EQ(values.at(v_idx), values_check_ascii.at(v_idx));
      EXPECT_EQ(values.at(v_idx), values_check_binary.at(v_idx));
    }
  }
}

TEST_F(IO, writeAndReadWithStamps)
{
  // filenames
  std::string data_file = getDataFile("motion3.m3d");
  std::string output_file_ascii = getOutputFile("motion3_ascii.m3d");
  std::string output_file_binary = getOutputFile("motion3_binary.m3d");

  // read data
  m3d::MotionData motion;
  m3d::M3DIOStatus return_code_reader = m3d::M3DReader::read(data_file, motion, true);
  ASSERT_EQ(return_code_reader, m3d::M3DIOStatus::kSuccess);

  // write data
  m3d::M3DIOStatus return_code_writer_ascii = m3d::M3DWriter::writeASCII(output_file_ascii, motion);
  ASSERT_EQ(return_code_writer_ascii, m3d::M3DIOStatus::kSuccess);

  m3d::M3DIOStatus return_code_writer_binary = m3d::M3DWriter::writeBinary(output_file_binary, motion);
  ASSERT_EQ(return_code_writer_binary, m3d::M3DIOStatus::kSuccess);

  // read data for checking
  m3d::MotionData motion_check_ascii;
  m3d::M3DIOStatus return_code_reader_check_ascii = m3d::M3DReader::read(output_file_ascii, motion_check_ascii, true);
  ASSERT_EQ(return_code_reader_check_ascii, m3d::M3DIOStatus::kSuccess);

  m3d::MotionData motion_check_binary;
  m3d::M3DIOStatus return_code_reader_check_binary = m3d::M3DReader::read(output_file_binary, motion_check_binary, true);
  ASSERT_EQ(return_code_reader_check_binary, m3d::M3DIOStatus::kSuccess);

  // check origin
  std::vector<double> origin_values = motion.getOrigin()->toVector();
  std::vector<double> origin_values_check_ascii = motion_check_ascii.getOrigin()->toVector();
  std::vector<double> origin_values_check_binary = motion_check_binary.getOrigin()->toVector();
  ASSERT_EQ(origin_values.size(), origin_values_check_ascii.size());
  ASSERT_EQ(origin_values.size(), origin_values_check_binary.size());
  for (std::size_t v_idx = 0; v_idx < origin_values.size(); ++v_idx)
  {
    EXPECT_EQ(origin_values.at(v_idx), origin_values_check_ascii.at(v_idx));
    EXPECT_EQ(origin_values.at(v_idx), origin_values_check_binary.at(v_idx));
  }

  // check data
  ASSERT_EQ(motion.getTransforms()->size(), motion_check_ascii.getTransforms()->size());
  ASSERT_EQ(motion.getTransforms()->size(), motion_check_binary.getTransforms()->size());
  for (std::size_t t_idx = 0; t_idx < motion.getTransforms()->size(); ++t_idx)
  {
    EXPECT_EQ(motion.getTransforms()->stamp_at(t_idx).toNSec(),
              motion_check_ascii.getTransforms()->stamp_at(t_idx).toNSec());
    EXPECT_EQ(motion.getTransforms()->stamp_at(t_idx).toNSec(),
              motion_check_binary.getTransforms()->stamp_at(t_idx).toNSec());

    std::vector<double> values = motion.getTransforms()->at(t_idx)->toVector();
    std::vector<double> values_check_ascii = motion_check_ascii.getTransforms()->at(t_idx)->toVector();
    std::vector<double> values_check_binary = motion_check_binary.getTransforms()->at(t_idx)->toVector();
    ASSERT_EQ(values.size(), values_check_ascii.size());
    ASSERT_EQ(values.size(), values_check_binary.size());
    for (std::size_t v_idx = 0; v_idx < values.size(); ++v_idx)
    {
      EXPECT_EQ(values.at(v_idx), values_check_ascii.at(v_idx));
      EXPECT_EQ(values.at(v_idx), values_check_binary.at(v_idx));
    }
  }
}

TEST_F(IO, writeAndReadOrigin)
{
  // filenames
  std::string input_file_origin = getDataFile("motion1.m3d");
  std::string input_file_no_origin = getDataFile("motion2.m3d");
  std::string output_file_origin_ascii = getOutputFile("motion_origin_ascii.m3d");
  std::string output_file_origin_binary = getOutputFile("motion_origin_binary.m3d");
  std::string output_file_no_origin_ascii = getOutputFile("motion_no_origin_ascii.m3d");
  std::string output_file_no_origin_binary = getOutputFile("motion_no_origin_binary.m3d");

  // read data with origin
  m3d::MotionData motion1;
  m3d::M3DIOStatus return_code1 = m3d::M3DReader::read(input_file_origin, motion1, true);
  ASSERT_EQ(return_code1, m3d::M3DIOStatus::kSuccess);

  // read data without origin
  m3d::MotionData motion2;
  m3d::M3DIOStatus return_code2 = m3d::M3DReader::read(input_file_no_origin, motion2, true);
  ASSERT_EQ(return_code2, m3d::M3DIOStatus::kSuccess);

  // check origins
  ASSERT_TRUE(motion1.getOrigin() != nullptr);
  ASSERT_TRUE(motion2.getOrigin() == nullptr);

  // write data
  m3d::M3DIOStatus return_code_writer_origin_ascii = m3d::M3DWriter::writeASCII(output_file_origin_ascii, motion1);
  ASSERT_EQ(return_code_writer_origin_ascii, m3d::M3DIOStatus::kSuccess);

  m3d::M3DIOStatus return_code_writer_origin_binary = m3d::M3DWriter::writeBinary(output_file_origin_binary, motion1);
  ASSERT_EQ(return_code_writer_origin_binary, m3d::M3DIOStatus::kSuccess);

  m3d::M3DIOStatus return_code_writer_no_origin_ascii = m3d::M3DWriter::writeASCII(output_file_no_origin_ascii, motion2);
  ASSERT_EQ(return_code_writer_no_origin_ascii, m3d::M3DIOStatus::kSuccess);

  m3d::M3DIOStatus return_code_writer_no_origin_binary = m3d::M3DWriter::writeBinary(output_file_no_origin_binary, motion2);
  ASSERT_EQ(return_code_writer_no_origin_binary, m3d::M3DIOStatus::kSuccess);

  // read data for checking
  m3d::MotionData motion_check_origin_ascii;
  m3d::M3DIOStatus return_code_reader_check_origin_ascii = m3d::M3DReader::read(output_file_origin_ascii, motion_check_origin_ascii, true);
  ASSERT_EQ(return_code_reader_check_origin_ascii, m3d::M3DIOStatus::kSuccess);

  m3d::MotionData motion_check_origin_binary;
  m3d::M3DIOStatus return_code_reader_check_origin_binary = m3d::M3DReader::read(output_file_origin_binary, motion_check_origin_binary, true);
  ASSERT_EQ(return_code_reader_check_origin_binary, m3d::M3DIOStatus::kSuccess);

  m3d::MotionData motion_check_no_origin_ascii;
  m3d::M3DIOStatus return_code_reader_check_no_origin_ascii = m3d::M3DReader::read(output_file_no_origin_ascii, motion_check_no_origin_ascii, true);
  ASSERT_EQ(return_code_reader_check_no_origin_ascii, m3d::M3DIOStatus::kSuccess);

  m3d::MotionData motion_check_no_origin_binary;
  m3d::M3DIOStatus return_code_reader_check_no_origin_binary = m3d::M3DReader::read(output_file_no_origin_binary, motion_check_no_origin_binary, true);
  ASSERT_EQ(return_code_reader_check_no_origin_binary, m3d::M3DIOStatus::kSuccess);

  // check origins
  ASSERT_TRUE(motion_check_origin_ascii.getOrigin() != nullptr);
  ASSERT_TRUE(motion_check_origin_binary.getOrigin() != nullptr);
  ASSERT_TRUE(motion_check_no_origin_ascii.getOrigin() == nullptr);
  ASSERT_TRUE(motion_check_no_origin_binary.getOrigin() == nullptr);
}

TEST_F(IO, writeGeneric)
{
  // filenames
  std::string data_file = getDataFile("motion3.m3d");
  std::string output_file_ascii1 = getOutputFile("motion3_ascii1.m3d");
  std::string output_file_ascii2 = getOutputFile("motion3_ascii2.m3d");
  std::string output_file_binary1 = getOutputFile("motion3_binary1.m3d");
  std::string output_file_binary2 = getOutputFile("motion3_binary2.m3d");

  // read data
  m3d::MotionData motion;
  m3d::M3DIOStatus return_code_reader = m3d::M3DReader::read(data_file, motion, true);
  ASSERT_EQ(return_code_reader, m3d::M3DIOStatus::kSuccess);

  // write ascii
  m3d::M3DIOStatus return_code_writer_ascii1 = m3d::M3DWriter::writeASCII(output_file_ascii1, motion, 12);
  ASSERT_EQ(return_code_writer_ascii1, m3d::M3DIOStatus::kSuccess);
  m3d::M3DIOStatus return_code_writer_ascii2 = m3d::M3DWriter::write(output_file_ascii2, motion, m3d::M3DFileType::kASCII, 12);
  ASSERT_EQ(return_code_writer_ascii2, m3d::M3DIOStatus::kSuccess);
  ASSERT_FILE_EQUAL(output_file_ascii1, output_file_ascii2);

  // write binary
  m3d::M3DIOStatus return_code_writer_binary1 = m3d::M3DWriter::writeBinary(output_file_binary1, motion);
  ASSERT_EQ(return_code_writer_binary1, m3d::M3DIOStatus::kSuccess);
  m3d::M3DIOStatus return_code_writer_binary2 = m3d::M3DWriter::write(output_file_binary2, motion, m3d::M3DFileType::kBinary);
  ASSERT_EQ(return_code_writer_binary2, m3d::M3DIOStatus::kSuccess);
  ASSERT_FILE_EQUAL(output_file_binary1, output_file_binary2);
}

TEST_F(IO, writeErrors)
{
  // filename
  std::string error_file = getDataFile("motion_error.m3d");

  // no transforms
  m3d::MotionData motion;

  m3d::M3DIOStatus return_code1a = m3d::M3DWriter::writeASCII(error_file, motion);
  ASSERT_EQ(return_code1a, m3d::M3DIOStatus::kNoTransforms);
  m3d::M3DIOStatus return_code1b = m3d::M3DWriter::writeBinary(error_file, motion);
  ASSERT_EQ(return_code1b, m3d::M3DIOStatus::kNoTransforms);

  // no transform type
  auto data = std::make_shared<m3d::TransformContainer>(false, false);
  data->append(std::make_shared<m3d::AxisAngleTransform>());
  motion.setTransforms(data);

  m3d::M3DIOStatus return_code2a = m3d::M3DWriter::writeASCII(error_file, motion);
  ASSERT_EQ(return_code2a, m3d::M3DIOStatus::kNoTransformType);
  m3d::M3DIOStatus return_code2b = m3d::M3DWriter::writeBinary(error_file, motion);
  ASSERT_EQ(return_code2b, m3d::M3DIOStatus::kNoTransformType);
}
