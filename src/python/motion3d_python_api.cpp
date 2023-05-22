#include <pybind11/pybind11.h>

#include <motion3d/motion3d.hpp>

#include "common.hpp"
#include "containers.hpp"
#include "io.hpp"
#include "transforms.hpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

PYBIND11_MODULE(motion3d, m)
{
  // common
  init_common(m);

  // containers
  init_containers(m);

  // io
  init_io(m);

  // transforms
  init_transforms(m);

  // version
  m.attr("__version__") = MACRO_STRINGIFY(MOTION3D_MAJOR_VERSION.MOTION3D_MINOR_VERSION.MOTION3D_PATCH_VERSION);
}
