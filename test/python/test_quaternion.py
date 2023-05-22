import unittest

import numpy as np

import motion3d as m3d


class TestQuaternion(unittest.TestCase):
    def test_init(self):
        # test data
        data = np.array([1.0, 2.0, 3.0, 4.0])

        # from array
        q = m3d.Quaternion.FromArray(data)
        np.testing.assert_almost_equal(q.w, data[0])
        np.testing.assert_almost_equal(q.x, data[1])
        np.testing.assert_almost_equal(q.y, data[2])
        np.testing.assert_almost_equal(q.z, data[3])
        np.testing.assert_almost_equal(q.toArray(), data)

        # from list
        q = m3d.Quaternion.FromList(data.tolist())
        np.testing.assert_almost_equal(q.toArray(), data)

        # values
        q = m3d.Quaternion(data[0], data[1], data[2], data[3])
        np.testing.assert_almost_equal(q.toArray(), data)

        # zero
        q = m3d.Quaternion.Zero()
        np.testing.assert_almost_equal(q.toArray(), np.zeros(4))
        q = m3d.Quaternion.FromArray(np.random.rand(4))
        q.setZero()
        np.testing.assert_almost_equal(q.toArray(), np.zeros(4))

        # identity
        q = m3d.Quaternion.Identity()
        np.testing.assert_almost_equal(q.toArray(), np.array([1, 0, 0, 0]))
        q = m3d.Quaternion.FromArray(np.random.rand(4))
        q.setIdentity()
        np.testing.assert_almost_equal(q.toArray(), np.array([1, 0, 0, 0]))

        # rotation matrix
        q = m3d.Quaternion.FromRotationMatrix(np.eye(3))
        np.testing.assert_almost_equal(q.toArray(), np.array([1, 0, 0, 0]))

    def test_member_access(self):
        # create data
        q = m3d.Quaternion.Zero()
        data = np.array([1.0, 2.0, 3.0, 4.0])

        # assign
        q.w = data[0]
        q.x = data[1]
        q.y = data[2]
        q.z = data[3]
        self.assertEqual(q.w, data[0])
        self.assertEqual(q.x, data[1])
        self.assertEqual(q.y, data[2])
        self.assertEqual(q.z, data[3])
        np.testing.assert_equal(q.toArray(), data)

        # modify inplace
        q.w += data[0]
        q.x += data[1]
        q.y += data[2]
        q.z += data[3]
        np.testing.assert_equal(q.toArray(), 2 * data)

    def test_inplace_return(self):
        # create data
        q = m3d.Quaternion(1.0, 2.0, 3.0, 4.0)

        # inplace
        self.assertEqual(q, q.conjugate_())
        self.assertEqual(q, q.inverse_())
        self.assertEqual(q, q.normalized_())

        # non-inplace
        self.assertNotEqual(q, q.conjugate())
        self.assertNotEqual(q, q.inverse())
        self.assertNotEqual(q, q.normalized())

    def test_methods(self):
        # test data
        q = m3d.Quaternion(1.0, 2.0, 3.0, 4.0)
        q_norm2 = 30.0
        q_norm = np.sqrt(q_norm2)

        # conjugate
        np.testing.assert_almost_equal(
            q.copy().conjugate_().toArray(), np.array([1.0, -2.0, -3.0, -4.0]))

        np.testing.assert_almost_equal(
            q.conjugate().toArray(), np.array([1.0, -2.0, -3.0, -4.0]))

        # inverse
        q_inv_mult_inplace = q.copy().inverse_() * q
        np.testing.assert_almost_equal(
            q_inv_mult_inplace.inverse().toArray(), m3d.Quaternion.Identity().toArray())

        q_inv_mult = q.inverse() * q
        np.testing.assert_almost_equal(
            q_inv_mult.inverse().toArray(), m3d.Quaternion.Identity().toArray())

        q_zero = m3d.Quaternion.Zero()
        self.assertRaises(m3d.MathException, lambda: q_zero.inverse_())
        self.assertRaises(m3d.MathException, lambda: q_zero.inverse())

        # norm
        self.assertAlmostEqual(q.norm(), q_norm)
        self.assertEqual(q.squaredNorm(), q_norm2)

        # slerp
        quat2 = m3d.Quaternion(5.0, 6.0, 7.0, 8.0)
        quat_slerp = q.slerp(0.7, quat2)
        self.assertIsInstance(quat_slerp, m3d.Quaternion)

        # angular distance
        quat_0deg = m3d.Quaternion(1.0, 0.0, 0.0, 0.0)
        quat_180deg = m3d.Quaternion(0.0, 1.0, 0.0, 0.0)
        angular_dist = quat_0deg.angularDistance(quat_180deg)
        self.assertAlmostEqual(angular_dist, np.pi)

        # rotation norm
        self.assertEqual(quat_0deg.rotationNorm(), 0.0)
        self.assertAlmostEqual(quat_180deg.rotationNorm(), np.pi)

    def test_is_equal(self):
        # epsilon for isEqual
        epsilon = 1e-4

        # create quaternions
        q1 = m3d.Quaternion(1.0, 2.0, 3.0, 4.0)

        q2 = q1.copy()
        q2.w += epsilon / 10

        q3 = q1.copy()
        q3.w += epsilon * 10

        q4 = m3d.Quaternion(1e-4, 0.0, 0.0, 0.0)

        # compare
        self.assertTrue(q1.isEqual(q1))
        self.assertFalse(q1.isEqual(-q1))
        self.assertTrue(q1.isEqual(q2, epsilon))
        self.assertFalse(q1.isEqual(q3, epsilon))
        self.assertTrue(q4.isEqual(m3d.Quaternion.Zero(), 1e-3))
        self.assertFalse(q4.isEqual(m3d.Quaternion.Zero(), 1e-5))

    def test_normalized(self):
        q1 = m3d.Quaternion(1.0, 0.0, 0.0, 0.0)
        q2 = m3d.Quaternion(0.0, 1.0, 0.0, 0.0)
        q3 = m3d.Quaternion(np.sqrt(0.5), -np.sqrt(0.5), 0.0, 0.0)

        for q in [q1, q2, q3]:
            # inplace
            np.testing.assert_almost_equal(q.toArray(), (q * 2.0).normalized_().toArray())
            np.testing.assert_almost_equal(q.toArray(), (q * -0.5).normalized_().toArray())

            # non-inplace
            np.testing.assert_almost_equal(q.toArray(), (q.copy() * 2.0).normalized().toArray())
            np.testing.assert_almost_equal(q.toArray(), (q.copy() * -0.5).normalized().toArray())

    def test_operators(self):
        # create data
        q1 = m3d.Quaternion.FromArray(np.random.rand(4))
        q2 = m3d.Quaternion.FromArray(np.random.rand(4))
        q1_copy = q1.copy()
        factor = 2.0

        # multiply with scalar
        q1_mult1 = q1 * factor
        np.testing.assert_almost_equal(q1_mult1.toArray(), q1.toArray() * factor)
        np.testing.assert_almost_equal(q1.toArray(), q1_copy.toArray())

        q1_mult2 = factor * q1
        np.testing.assert_almost_equal(q1_mult2.toArray(), q1.toArray() * factor)
        np.testing.assert_almost_equal(q1.toArray(), q1_copy.toArray())

        q1_mult_copy = q1.copy()
        q1_mult_copy *= factor
        np.testing.assert_almost_equal(q1_mult_copy.toArray(), q1_copy.toArray() * factor)

        # multiply with quaternion
        q12_mult = q1 * q2
        self.assertIsInstance(q12_mult, m3d.Quaternion)
        np.testing.assert_almost_equal(q1.toArray(), q1_copy.toArray())

        q12_mult_copy = q1.copy()
        q12_mult_copy *= q2
        self.assertIsInstance(q12_mult_copy, m3d.Quaternion)

        # divide
        q1_div = q1 / factor
        np.testing.assert_almost_equal(q1_div.toArray(), q1.toArray() / factor)
        np.testing.assert_almost_equal(q1.toArray(), q1_copy.toArray())

        q1_div_copy = q1.copy()
        q1_div_copy /= factor
        np.testing.assert_almost_equal(q1_div_copy.toArray(), q1.toArray() / factor)

        # add
        q1_add = q1 + q2
        np.testing.assert_almost_equal(q1_add.toArray(), q1.toArray() + q2.toArray())
        np.testing.assert_almost_equal(q1.toArray(), q1_copy.toArray())

        q1_add_copy = q1.copy()
        q1_add_copy += q2
        np.testing.assert_almost_equal(q1_add_copy.toArray(), q1.toArray() + q2.toArray())

        # subtract
        q1_sub1 = -q1
        np.testing.assert_almost_equal(q1_sub1.toArray(), -q1.toArray())
        np.testing.assert_almost_equal(q1.toArray(), q1_copy.toArray())

        q1_sub2 = q1 - q2
        np.testing.assert_almost_equal(q1_sub2.toArray(), q1.toArray() - q2.toArray())
        np.testing.assert_almost_equal(q1.toArray(), q1_copy.toArray())

        q1_sub_copy = q1.copy()
        q1_sub_copy -= q2
        np.testing.assert_almost_equal(q1_sub_copy.toArray(), q1.toArray() - q2.toArray())

    def test_vector(self):
        # create quaternion
        q = m3d.Quaternion.FromArray(np.random.rand(4))

        # list
        list_data = q.toList()
        q_list = m3d.Quaternion.FromList(list_data)

        self.assertEqual(len(list_data), 4)
        self.assertEqual(list_data[0], q.w)
        self.assertEqual(list_data[1], q.x)
        self.assertEqual(list_data[2], q.y)
        self.assertEqual(list_data[3], q.z)
        np.testing.assert_equal(q.toArray(), q_list.toArray())

        # array
        array_data = q.toArray()
        q_array = m3d.Quaternion.FromArray(array_data)

        self.assertEqual(len(array_data), 4)
        self.assertEqual(array_data[0], q.w)
        self.assertEqual(array_data[1], q.x)
        self.assertEqual(array_data[2], q.y)
        self.assertEqual(array_data[3], q.z)
        np.testing.assert_equal(q.toArray(), q_array.toArray())

    def test_matrix(self):
        # create data
        q1 = m3d.Quaternion.FromArray(np.random.rand(4))
        q2 = m3d.Quaternion.FromArray(np.random.rand(4))

        # chain with multiplication
        q12 = q1 * q2

        # chain in matrix form
        q12_pos = q1.toPositiveMatrix().dot(q2.toArray())
        q12_neg = q2.toNegativeMatrix().dot(q1.toArray())

        # compare
        np.testing.assert_almost_equal(q12_pos, q12.toArray())
        np.testing.assert_almost_equal(q12_neg, q12.toArray())

    def test_description(self):
        q = m3d.Quaternion()
        self.assertTrue(str(q).startswith("Quaternion"))


if __name__ == '__main__':
    unittest.main()
