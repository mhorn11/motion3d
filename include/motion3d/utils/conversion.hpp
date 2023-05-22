#pragma once

#include <string>
#include <vector>

#include <motion3d/utils/exceptions.hpp>

namespace motion3d
{

/** Exception thrown in case a vector conversion fails. */
struct VectorConversionException : public MessageException
{
  explicit VectorConversionException(std::string msg) : MessageException(std::move(msg)) {}
};

/** Safely convert <I>InputType</I> vector to <I>OutputType</I> vector using memcopy. */
template<typename InputType, typename OutputType>
std::vector<OutputType> convertVector(const std::vector<InputType> &input)
{
  // get output size
  using SignedType = std::make_signed_t<std::size_t>;
  auto new_size_div = std::div(
    static_cast<SignedType>(input.size() * sizeof(InputType)),
    static_cast<SignedType>(sizeof(OutputType)));
  if (new_size_div.rem != 0)
  {
    throw VectorConversionException("Input vector and type size does not match output type size");
  }

  // create output and copy data
  std::vector<OutputType> output(new_size_div.quot);
  std::memcpy(output.data(), input.data(), input.size() * sizeof(InputType));
  return output;
}

/** Safely convert <I>InputType</I> vector to <I>OutputType</I> value. */
template<typename InputType, typename OutputType>
OutputType convertFromVector(const std::vector<InputType> &input)
{
  std::vector<OutputType> output = convertVector<InputType, OutputType>(input);
  if (output.size() != 1)
  {
    throw VectorConversionException("Input vector does not contain exactly one value");
  }
  return output[0];
}

/** Safely convert <I>InputType</I> value to <I>OutputType</I> vector. */
template<typename InputType, typename OutputType>
std::vector<OutputType> convertToVector(const InputType &input)
{
  try
  {
    return convertVector<InputType, OutputType>({input});
  }
  catch (const VectorConversionException& ex)
  {
    throw VectorConversionException("Input value cannot be separated completely into vector of output type");
  }
}

/** Helper method for creating a string from the stream output of an object. */
template<typename T>
std::string streamToString(const T& obj)
{
  std::stringstream ss;
  ss << obj;
  return ss.str();
}

} // namespace motion3d
