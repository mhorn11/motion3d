#pragma once

#include <exception>

namespace motion3d
{

/** Generic exception class with custom message. */
struct MessageException : public std::exception
{
 public:
  explicit MessageException(std::string msg) : msg_(std::move(msg)) {}

  const char* what() const noexcept override
  {
    return msg_.c_str();
  }

 private:
  std::string msg_;
};

} // namespace motion3d
