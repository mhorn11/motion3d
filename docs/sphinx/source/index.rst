.. Motion3d documentation master file, created by
   sphinx-quickstart on Fri Mar 18 09:19:34 2022.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Motion3d Documentation
======================

A header-only C++ library with full Python bindings for handling, converting, and storing 3D motions and poses.

It supports converting, chaining and applying transformations in many common representations:

 * Axis-angle rotation and translation vector
 * Dual quaternion
 * Euler angles and translation vector
 * SE(3) transformation matrix
 * Rotation quaternion and translation vector

Using the :cpp:class:`motion3d::TransformInterface` base class, transformations can be applied and chained without worrying about the underlying type.

The :cpp:class:`motion3d::TransformContainer` enables handling multiple transformations as motions or as poses, with or without corresponding timestamps.
Furthermore, the :cpp:class:`motion3d::M3DWriter` and :cpp:class:`motion3d::M3DReader` can be used to store and load transformations from both C++ and Python in the M3D file format.


Contents
--------

This documentations contains the C++ as well as the Python documentation for Motion3d.

Since the library is implemented header-only in C++ and only binded to Python, most of the documentation is done within the C++ headers.
Therefore, in most cases, the Python documentation only links to the respective part of the C++ documentation.

However, if you are interested in the C++ documentation only, the original Doxygen documentation provides a more detailed overview.

.. toctree::
   :titlesonly:
   :maxdepth: 2

   motion3d/installation
   motion3d/transformations
   motion3d/usage
   motion3d/cpp
   motion3d/python
   motion3d/development
   motion3d/acknowledgments
