# Motion3D

[![PyPI Version](https://img.shields.io/pypi/v/motion3d?color=0f80c1)](https://pypi.org/project/motion3d)
[![Documentation Status](https://readthedocs.org/projects/motion3d/badge/?version=latest)](https://motion3d.readthedocs.io/en/latest/?badge=latest)  
[![C++ Tests](https://github.com/mhorn11/motion3d/actions/workflows/cpp_tests.yml/badge.svg?branch=main)](https://github.com/mhorn11/motion3d/actions/workflows/cpp_tests.yml)
[![Python Tests](https://github.com/mhorn11/motion3d/actions/workflows/python_tests.yml/badge.svg?branch=main)](https://github.com/mhorn11/motion3d/actions/workflows/python_tests.yml)
[![Codecov](https://codecov.io/gh/mhorn11/motion3d/graph/badge.svg?token=DXJBH2UE7R)](https://codecov.io/gh/mhorn11/motion3d)


A header-only C++ library with full Python bindings for handling, converting, and storing 3D motions and poses.

It supports converting, chaining and applying transformations in many common representations:
 * Axis-angle rotation and translation vector
 * Dual quaternion
 * Euler angles and translation vector
 * SE(3) transformation matrix
 * Rotation quaternion and translation vector

Using the `TransformInterface` base class, transformations can be applied and chained without worrying about the underlying type.

The `TransformContainer` enables handling multiple transformations as motions or as poses, with or without corresponding timestamps.
Furthermore, the `M3DWriter` and `M3DReader` can be used to store and load transformations from both C++ and Python in the M3D file format.



## Installation

### Requirements

Supported operating systems:
 * Ubuntu 20.04
 * Ubuntu 22.04


| Dependency | Version  | Note                                  |
|------------|----------| ------------------------------------- |
| CMake      | >= 3.14  |                                       |
| GCC        | >= 9.4   | C++17 is required                     |
| Eigen      | >= 3.3   |                                       |
| Python     | >= 3.8   | Tests are run for 3.8, 3.10, and 3.12 |


All dependencies can be installed using:

```bash
apt install build-essential cmake libeigen3-dev python3 python3-dev python3-pip
```

See `docker/motion3d.dockerfile` for further installation details, e.g., for packages required for development.


### C++

After all requirements are available, use CMake and Make to install the library headers:

```bash
mkdir build && cd build
cmake ..
make install
```

For uninstalling use `make uninstall`.


### Python

The library can be installed either over PyPI using pip:

```bash
python3 -m pip install motion3d
```

or directly from within the cloned repository:

```bash
python3 -m pip install .
```

If Eigen is not found on the system, the setup downloads a suitable version locally before building the package.


## Documentation

Fore more information about the transformation types, definitions, and the usage in C++ and Python, see the documentation at [motion3d.readthedocs.io](https://motion3d.readthedocs.io).

Furthermore, the `doc` directory provides a detailed readme on how to create the documentation directly from the repository.
