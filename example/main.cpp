#include <motion3d/motion3d.hpp>

namespace m3d = motion3d;

int main(int argc, char **argv)
{
  m3d::DualQuaternionTransform dq;
  std::cout << dq.toString() << std::endl;
}
