import numpy as np

import motion3d as m3d


# Number of random runs for each test.
N_RANDOM_RUNS = 100


def assert_transform_almost_equal(test_case, a, b):
    """Compare two motion3d transformations."""
    a_arr = a.asType(m3d.TransformType.kDualQuaternion).toArray()
    b_arr = b.asType(m3d.TransformType.kDualQuaternion).toArray()
    test_case.assertTrue(np.allclose(a_arr, b_arr) or np.allclose(a_arr, -b_arr))


def create_random_euler_transform():
    """Create a random valid transformation in Euler format."""
    translation = np.random.rand(3) * 20.0 - 10.0
    euler_angles = np.random.rand(3) * 2 * np.pi - np.pi
    return m3d.EulerTransform(translation, euler_angles[0], euler_angles[1], euler_angles[2], m3d.EulerAxes.kSXYZ)
