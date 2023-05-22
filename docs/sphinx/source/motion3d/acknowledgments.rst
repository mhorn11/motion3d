Acknowledgments
===============


Eigen
-----

The underlying transformation types and calculations are mostly based on Eigen matrices, quaternions and other Eigen types.
Eigen ist primarily MPL2 licensed.

Link: https://eigen.tuxfamily.org


Point Cloud Library (PCL)
-------------------------

The M3D data format and the reader and writer implementations are strongly based on the PCD data format and implementation, which is licensed under the the 3-clause BSD License.

Link: https://pointclouds.org


pykitti
-------

The conversion script from KITTI odometry data to M3D uses the calibration file reader implementation from pykitti, which is licensed under the MIT license.

Link: https://github.com/utiasSTARS/pykitti


Transforms3d
------------

The Euler angle representation and conversion is ported to C++ from the Transforms3d Python library, which is licensed under the 2-clause BSD license.

Link: https://matthew-brett.github.io/transforms3d
