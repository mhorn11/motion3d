#include <gtest/gtest.h>

#include "test_dual_quaternion.hpp"
#include "test_euler_axes.hpp"
#include "test_extended_map.hpp"
#include "test_io.hpp"
#include "test_math.hpp"
#include "test_quaternion.hpp"
#include "test_time.hpp"
#include "test_transform_container.hpp"
#include "test_transform_interface.hpp"
#include "test_transform_result.hpp"
#include "test_utils.hpp"

int main(int argc, char **argv)
{
  srand(0);
  ::testing::InitGoogleTest(&argc, argv); 
  return RUN_ALL_TESTS();
}
