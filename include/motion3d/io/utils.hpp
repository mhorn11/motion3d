#pragma once

#include <exception>

namespace motion3d
{

constexpr std::string_view kM3DFileFormatVersion = "0.1";

/** Status return type for M3DWriter and M3DReader. */
enum class M3DIOStatus
{
  kSuccess = 0,           ///< no errors
  kFileNotFound,          ///< the given file does not exist
  kFileOpenError,         ///< could not open file
  kFileLockError,         ///< could not lock file
  kFilePermissionsError,  ///< could not set file permissions
  kUnsupportedFileType,   ///< the file type is not supported
  kNoTransformType,       ///< input has no transform type
  kInvalidTransformType,  ///< given transform type does not exist
  kNoTransforms,          ///< transforms are uninitialized or empty
  kInvalidBinarySize,     ///< binary data does not result in complete transformations
  kInvalidTransform,      ///< number of values does not match the advertised number
  kMoreTransforms,        ///< input has more transforms than advertised
  kLessTransforms,        ///< input has fewer transforms than advertised
  kInvalidFrame,          ///< input has an invalid FRAME value
  kInvalidType,           ///< input has an invalid TYPE value
  kInvalidStamps,         ///< input has an invalid STAMPS value
  kInvalidPoses,          ///< input has an invalid POSES value
  kInvalidSize,           ///< input has an invalid SIZE value
};

/** M3D File type. */
enum class M3DFileType
{
  kASCII,   ///< human-readable ascii 
  kBinary,  ///< binary-encoded
};

/** Removes leading and trailing spaces from string <I>s</I> and removes comments (#). */
void trim(std::string &s)
{
  // trim comments
  auto comment_pos = s.find('#');
  if (comment_pos != std::string::npos)
  {
    s.erase(comment_pos);
  }

  // trim left
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
    return std::isspace(ch) == 0;
  }));
  
  // trim right
  s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
      return std::isspace(ch) == 0;
  }).base(), s.end());
}

/** \brief Trims and splits string at spaces and tabs. 
  * \param[in]  line the string to trim and split
  * \param[out] st the target vector
  */
void tokenize(std::string &line, std::vector<std::string> &st)
{
  st.clear();
  std::regex re("[\t\r ]+");
  std::copy(std::sregex_token_iterator(line.begin(), line.end(), re, -1),
            std::sregex_token_iterator(),
            std::back_inserter(st));
}

} // namespace motion3d
