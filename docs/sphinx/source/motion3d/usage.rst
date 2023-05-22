Usage
=====


C++ Example
-----------

After installing the ``motion3d`` C++ library, you can place ``CMakeLists.txt`` and ``main.cpp`` in one directory and run the following commands to build and run the example:

.. code-block:: bash

    mkdir build && cd build
    cmake ..
    make
    ./motion3d_example

.. literalinclude:: ./../../../../example/CMakeLists.txt
    :language: cmake
    :caption: CMakeLists.txt

.. literalinclude:: ./../../../../example/main.cpp
    :language: cpp
    :caption: main.cpp


Python Example
--------------

After installing `motion3d` in your Python venv, you can run the following example:

.. literalinclude:: ./../../../../example/motion3d_example.py
    :language: python
    :caption: motion3d_example.py


Python Scripts
--------------

All scripts are installed executable with the ``motion3d`` Python bindings.
You can the respective script with ``-h`` for further details.

==========================  ===================================================  =======================
Script                      Description                                          Additional Requirements
==========================  ===================================================  =======================
m3d_convert_kitti_odometry  Convert data from the KITTI odometry dataset         
m3d_convert_rosbag          Extract transformations from a rosbag                rosbags
m3d_plot_poses              Plot poses from an M3D file                          matplotlib
==========================  ===================================================  =======================
