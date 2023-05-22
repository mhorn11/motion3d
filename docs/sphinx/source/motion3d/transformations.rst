Transformations
===============


Mathematical Conventions
------------------------


Generic Transformations
^^^^^^^^^^^^^^^^^^^^^^^

Transformations are denoted in general as functions :math:`T_{ab}`.
In this case, :math:`T_{ab}` is the frame-forward transformation from frame :math:`a` to frame :math:`b`, also shown in Fig. 1.

.. figure:: ./img/transforms.png
    :height: 80px
    :align: center

    Fig.1: Transformations between different frames.

The concatenation of multiple transformations is denoted as

.. math:: T_{ad} = T_{ab} \circ T_{bc} \circ T_{cd} \, .

Since we use frame-forward transformations, :math:`T_{ad}` expressed as multiplication of homogeneous transformation matrices (HMs) yields

.. math:: \boldsymbol{T}_{ad} = \boldsymbol{T}_{ab} \, \boldsymbol{T}_{bc} \, \boldsymbol{T}_{cd} \, .

This also implies that a point :math:`\boldsymbol{p}_{d}` in frame :math:`d` is transformed to frame :math:`a` using

.. math:: \boldsymbol{p}_{a}\ = T_{ad}(\boldsymbol{p}_{d}) = T_{ab}(T_{bc}(T_{cd}(\boldsymbol{p}_{d}))) \, ,

or expressed as HMs using

.. math:: \boldsymbol{p}_{a}\ = \boldsymbol{T}_{ad} \, \boldsymbol{p}_{d} = \boldsymbol{T}_{ab} \, \boldsymbol{T}_{bc} \, \boldsymbol{T}_{cd} \, \boldsymbol{p}_{d} \, .

The inverse of a transformation is denoted as

.. math:: T_{ab}^{-1} = T_{ba} \, .


Motions and Poses
^^^^^^^^^^^^^^^^^

The TransformContainer makes it possible to store transformations as either motions or poses and to convert between both representations.
Fig. 2 gives an overview over the relation between motions and poses.

.. figure:: ./img/motions_poses.png
    :height: 400px
    :align: center

    Fig. 2: Motions and poses.

The pose of a frame :math:`a` at step :math:`t` is denoted as :math:`P_{a,t}`, the motion between two poses at step :math:`t` is denoted `V_{a,t}`.
Thus, the motion between two poses is calculated as

.. math:: V_{a,t} = P_{a,t}^{-1} \circ P_{a,t+1} \, .

Vice versa, the next pose, given the previous pose and the motion, is calculated as

.. math:: P_{a,t+1} = P_{a,t} \circ V_{a,t} \, .

When converting a full dataset from motions to poses, the initial pose :math:`P_{a,1}` must be provided.
The identity transformation is a trivial choice for this.

In case of two rigidly coupled frames :math:`a` and :math:`b` as in Fig. 2, e.g., sensors mounted on a vehicle, the respective motions and poses can be converted into each other
using the rigid frame transformation :math:`T`:

.. math::
    V_{b,t} &= T^{-1} \circ V_{a,t} \circ T \\
    P_{b,t} &= T^{-1} \circ P_{a,t} \circ T


Representations
^^^^^^^^^^^^^^^

A transformation representation is necessary for using and applying the previously described generic transformations.
The following table lists all implemented transformation representations.
Some representations do not directly support operations like concatenation or inversion.
However, they are implicitly converted by this library to other representations for this.
We refer to the respective documentation of each class for more detailed information.

==========================================   ==============================================
Representation                               Class
==========================================   ==============================================
Axis-Angle Rotation and Translation Vector   :cpp:class:`motion3d::AxisAngleTransform`
DualQuaternion                               :cpp:class:`motion3d::DualQuaternionTransform`
Euler Angles and Translation Vector          :cpp:class:`motion3d::EulerTransform`
Homogenous Matrix                            :cpp:class:`motion3d::MatrixTransform`
Quaternion and Translation Vector            :cpp:class:`motion3d::QuaternionTransform`
==========================================   ==============================================


Implementation
^^^^^^^^^^^^^^

Representations
^^^^^^^^^^^^^^^

For each representation, a separate class is implemented.
Furthermore, in order to enable representation-independent processing, all these classes inherit from :cpp:class:`motion3d::TransformInterface`.
This makes it possible to store, access, modify and combine transformations without taking care of the underlying representation.
The ``asType()`` methods of each class can be used to convert between the different representations.

For maximizing performance, it is still necessary to convert transformations to a fixed representation, since the usage of :cpp:class:`motion3d::TransformInterface` involves vtable access.
We recommend to use either :cpp:class:`motion3d::DualQuaternionTransform` or :cpp:class:`motion3d::MatrixTransform` for this, since they provide direct implementations for all possible operations instead of converting to a different representation first.


Single Transformations
^^^^^^^^^^^^^^^^^^^^^^

Transformations are created using either the respective constructors or the :cpp:func:`motion3d::TransformInterface::Factory` method.
The default constructor automatically initializes an identity transformation.

By default, transformations are marked as *safe*.
This means that at each operation, the transformation is automatically checked for validity.
The *unsafe* parameter of the constructors can be used to disable this.
However, this can lead to errors since most operations only work properly on valid transformations.
A safe and valid transformation can be obtained using the ``normalized()`` methods.

The following table gives an overview of possible operations on single transformations.

=================================================  ==========================  ======================================
Operation                                          Equation                    Implementation(s)
=================================================  ==========================  ======================================
Apply :math:`T_b` after :math:`T_a`                :math:`T_a \circ T_b`       ``a.applyPost(b)`` ``a * b``
Apply :math:`T_b` before :math:`T_a`               :math:`T_b \circ T_a`       ``a.applyPre(b)``
Invert :math:`T_a`                                 :math:`T_a^{-1}`            ``a.inverse()``
Apply :math:`T_b^{-1}` after :math:`T_a`           :math:`T_a \circ T_b^{-1}`  ``a.applyPost(b.inverse())`` ``a / b``
Translation norm of :math:`T_a`                                                ``a.translationNorm()``
Rotation norm of :math:`T_a`                                                   ``a.rotationNorm()``
Scale translation of :math:`T_a` by :math:`x`                                  ``a.scaleTranslation(x)``
Normalize :math:`T_a` and ensure that it is valid                              ``a.normalized(x)``
=================================================  ==========================  ======================================


Transformation Container
^^^^^^^^^^^^^^^^^^^^^^^^

In addition to the previously described operations on single transformations, :cpp:class:`motion3d::TransformContainer` also provides container specific functions.

============================================================================  ============================
Description                                                                   Implementation
============================================================================  ============================
Apply :math:`T_a` before and :math:`T_b` after all transformations            ``container.apply(a, b)``
Change the coordinate frame using the rigid frame transformation :math:`T_a`  ``container.changeFrame(a)``
Get container as poses with the identity pose as initial pose                 ``container.asPoses()``
Get container as poses with the :math:`T_a` as initial pose                   ``container.asPoses(a)``
Get container as motions                                                      ``container.asMotions()``
============================================================================  ============================

More methods, e.g., involving stamped data, can be found in the :cpp:class:`motion3d::TransformContainer` documentation.


Inplace Operations
^^^^^^^^^^^^^^^^^^

All previously described method do not alter the original objects.
For performing operations directly on the object, some methods have an inplace variant, marked by a trailing underscore, e.g., ``normalized_()`` instead of ``normalized()``.
