Installation
============

Requirements
------------

==========  =======  =================
Dependency  Version  Note
==========  =======  =================
CMake       >= 3.14
GCC         >= 9.4   C++17 is required
Eigen       >= 3.3
Python      >= 3.6
==========  =======  =================

With Ubuntu 20.04, all dependencies can be installed using:

.. code-block:: bash

   apt install build-essential cmake libeigen3-dev python3 python3-dev python3-pip


See ``docker/motion3d.dockerfile`` for further installation details, e.g., for packages required for development.


C++
---

After all requirements are available, use CMake and Make to install the library headers:

.. code-block:: bash

   mkdir build && cd build
   cmake ..
   make install

For uninstalling use ``make uninstall``.


Python
------

The library can be installed either over PyPI using pip:

.. code-block:: bash

   python3 -m pip install motion3d


or directly from within the cloned repository:

.. code-block:: bash

   python3 -m pip install .

If Eigen is not found on the system, the setup downloads a suitable version locally before building the package.
