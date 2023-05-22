import unittest

import numpy as np

import motion3d as m3d


class TestDualQuaternion(unittest.TestCase):
    def test_init(self):
        # create data
        data = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0])

        # from array
        q = m3d.DualQuaternion.FromArray(data)
        np.testing.assert_almost_equal(q.toArray(), data)

        q = m3d.DualQuaternion.FromArray(data[:4], data[4:])
        np.testing.assert_almost_equal(q.toArray(), data)

        # from list
        q = m3d.DualQuaternion.FromList(data.tolist())
        np.testing.assert_almost_equal(q.toArray(), data)

        q = m3d.DualQuaternion.FromList(data[:4].tolist(), data[4:].tolist())
        np.testing.assert_almost_equal(q.toArray(), data)

        # values
        q = m3d.DualQuaternion(data[0], data[1], data[2], data[3],
                               data[4], data[5], data[6], data[7])
        np.testing.assert_almost_equal(q.toArray(), data)

        # zero
        q = m3d.DualQuaternion.Zero()
        np.testing.assert_almost_equal(q.toArray(), np.zeros(8))
        q = m3d.DualQuaternion.FromArray(np.random.rand(8))
        q.setZero()
        np.testing.assert_almost_equal(q.toArray(), np.zeros(8))

        # identity
        q = m3d.DualQuaternion.Identity()
        np.testing.assert_almost_equal(q.toArray(), np.array([1, 0, 0, 0, 0, 0, 0, 0]))
        q = m3d.DualQuaternion.FromArray(np.random.rand(8))
        q.setIdentity()
        np.testing.assert_almost_equal(q.toArray(), np.array([1, 0, 0, 0, 0, 0, 0, 0]))

    def test_member_access(self):
        # create data
        q = m3d.DualQuaternion.Zero()
        data = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0])

        # assign quaternions
        q.real = m3d.Quaternion.FromArray(data[:4])
        q.dual = m3d.Quaternion.FromArray(data[4:])
        np.testing.assert_equal(q.toArray(), data)

        # modify quaternions inplace
        q.real += m3d.Quaternion.FromArray(data[:4])
        q.dual += m3d.Quaternion.FromArray(data[4:])
        np.testing.assert_equal(q.toArray(), 2 * data)

        # assign values
        q.setZero()
        q.real.w = data[0]
        q.real.x = data[1]
        q.real.y = data[2]
        q.real.z = data[3]
        q.dual.w = data[4]
        q.dual.x = data[5]
        q.dual.y = data[6]
        q.dual.z = data[7]
        self.assertEqual(q.real.w, data[0])
        self.assertEqual(q.real.x, data[1])
        self.assertEqual(q.real.y, data[2])
        self.assertEqual(q.real.z, data[3])
        self.assertEqual(q.dual.w, data[4])
        self.assertEqual(q.dual.x, data[5])
        self.assertEqual(q.dual.y, data[6])
        self.assertEqual(q.dual.z, data[7])
        np.testing.assert_equal(q.toArray(), data)

        # modify values inplace
        q.real.w += data[0]
        q.real.x += data[1]
        q.real.y += data[2]
        q.real.z += data[3]
        q.dual.w += data[4]
        q.dual.x += data[5]
        q.dual.y += data[6]
        q.dual.z += data[7]
        np.testing.assert_equal(q.toArray(), 2 * data)

    def test_inplace_return(self):
        # create data
        q = m3d.DualQuaternion(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0)

        # inplace
        self.assertEqual(q, q.quatConjugate_())
        self.assertEqual(q, q.dualConjugate_())
        self.assertEqual(q, q.combConjugate_())
        self.assertEqual(q, q.inverse_())
        self.assertEqual(q, q.normalized_())

        # non-inplace
        self.assertNotEqual(q, q.quatConjugate())
        self.assertNotEqual(q, q.dualConjugate())
        self.assertNotEqual(q, q.combConjugate())
        self.assertNotEqual(q, q.inverse())
        self.assertNotEqual(q, q.normalized())

    def test_methods(self):
        # create data
        q = m3d.DualQuaternion(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0)
        q_norm2 = 204.0
        q_norm = np.sqrt(q_norm2)

        # quaternion conjugate
        np.testing.assert_almost_equal(
            q.copy().quatConjugate_().toArray(), np.array([1.0, -2.0, -3.0, -4.0, 5.0, -6.0, -7.0, -8.0]))
        np.testing.assert_almost_equal(
            q.quatConjugate().toArray(), np.array([1.0, -2.0, -3.0, -4.0, 5.0, -6.0, -7.0, -8.0]))

        # dual conjugate
        np.testing.assert_almost_equal(
            q.copy().dualConjugate_().toArray(), np.array([1.0, 2.0, 3.0, 4.0, -5.0, -6.0, -7.0, -8.0]))
        np.testing.assert_almost_equal(
            q.dualConjugate().toArray(), np.array([1.0, 2.0, 3.0, 4.0, -5.0, -6.0, -7.0, -8.0]))

        # combined conjugate
        np.testing.assert_almost_equal(
            q.copy().combConjugate_().toArray(), np.array([1.0, -2.0, -3.0, -4.0, -5.0, 6.0, 7.0, 8.0]))
        np.testing.assert_almost_equal(
            q.combConjugate().toArray(), np.array([1.0, -2.0, -3.0, -4.0, -5.0, 6.0, 7.0, 8.0]))

        # inverse
        q_inv_mult1_inplace = q * q.copy().inverse_()
        np.testing.assert_almost_equal(
            q_inv_mult1_inplace.inverse().toArray(), m3d.DualQuaternion.Identity().toArray())

        q_inv_mult = q * q.inverse()
        np.testing.assert_almost_equal(
            q_inv_mult.inverse().toArray(), m3d.DualQuaternion.Identity().toArray())

        # norm
        self.assertAlmostEqual(q.norm(), q_norm)
        self.assertEqual(q.squaredNorm(), q_norm2)

    def test_is_equal(self):
        # epsilon for isEqual
        epsilon = 1e-4

        # create quaternions
        q1 = m3d.DualQuaternion(1.0, 2.0, 3.0, 4.0, 1.0, 2.0, 3.0, 4.0)

        q2 = q1.copy()
        q2.real.w += epsilon / 10

        q3 = q1.copy()
        q3.dual.w += epsilon / 10

        q4 = q1.copy()
        q4.real.w += epsilon * 10

        q5 = q1.copy()
        q5.dual.w += epsilon * 10

        q6 = m3d.DualQuaternion(1e-4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        q7 = m3d.DualQuaternion(0.0, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0)

        # compare
        self.assertTrue(q1.isEqual(q1))
        self.assertFalse(q1.isEqual(-q1))
        self.assertTrue(q1.isEqual(q2, epsilon))
        self.assertTrue(q1.isEqual(q3, epsilon))
        self.assertFalse(q1.isEqual(q4, epsilon))
        self.assertFalse(q1.isEqual(q5, epsilon))
        self.assertTrue(q6.isEqual(m3d.DualQuaternion.Zero(), 1e-3))
        self.assertFalse(q6.isEqual(m3d.DualQuaternion.Zero(), 1e-5))
        self.assertTrue(q7.isEqual(m3d.DualQuaternion.Zero(), 1e-3))
        self.assertFalse(q7.isEqual(m3d.DualQuaternion.Zero(), 1e-5))

    def test_normalized(self):
        # create data
        q1 = m3d.DualQuaternion(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0, -3.0)
        q2 = m3d.DualQuaternion(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        q3 = m3d.DualQuaternion(np.sqrt(0.5), -np.sqrt(0.5), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        # check rT * r = 1
        for q in [q1, q2, q3]:
            # inplace
            np.testing.assert_almost_equal(q.toArray(), (q * 2.0).normalized_().toArray())
            np.testing.assert_almost_equal(q.toArray(), (q * -0.5).normalized_().toArray())

            # non-inplace
            np.testing.assert_almost_equal(q.toArray(), (q.copy() * 2.0).normalized().toArray())
            np.testing.assert_almost_equal(q.toArray(), (q.copy() * -0.5).normalized().toArray())

        # check rT * d = 0
        q1_copy = q1.copy()
        q1_copy.dual.w = 2.0
        np.testing.assert_almost_equal(q1.toArray(), q1_copy.copy().normalized_().toArray())
        np.testing.assert_almost_equal(q1.toArray(), (q1_copy.copy() * 2.0).normalized_().toArray())
        np.testing.assert_almost_equal(q1.toArray(), (q1_copy.copy() * -0.5).normalized_().toArray())
        np.testing.assert_almost_equal(q1.toArray(), q1_copy.normalized().toArray())
        np.testing.assert_almost_equal(q1.toArray(), (q1_copy * 2.0).normalized().toArray())
        np.testing.assert_almost_equal(q1.toArray(), (q1_copy * -0.5).normalized().toArray())

    def test_operators(self):
        # create data
        q1 = m3d.DualQuaternion.FromArray(np.random.rand(8))
        q2 = m3d.DualQuaternion.FromArray(np.random.rand(8))
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

        # multiply with dual quaternion
        q12_mult = q1 * q2
        self.assertIsInstance(q12_mult, m3d.DualQuaternion)
        np.testing.assert_almost_equal(q1.toArray(), q1_copy.toArray())

        q12_mult_copy = q1.copy()
        q12_mult_copy *= q2
        self.assertIsInstance(q12_mult_copy, m3d.DualQuaternion)

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
        # create data
        q = m3d.DualQuaternion.FromArray(np.random.rand(8))

        # list
        list_data = q.toList()
        q_list = m3d.DualQuaternion.FromList(list_data)

        self.assertEqual(len(list_data), 8)
        self.assertEqual(list_data[0], q.real.w)
        self.assertEqual(list_data[1], q.real.x)
        self.assertEqual(list_data[2], q.real.y)
        self.assertEqual(list_data[3], q.real.z)
        self.assertEqual(list_data[4], q.dual.w)
        self.assertEqual(list_data[5], q.dual.x)
        self.assertEqual(list_data[6], q.dual.y)
        self.assertEqual(list_data[7], q.dual.z)
        np.testing.assert_equal(q.toArray(), q_list.toArray())

        # array
        array_data = q.toArray()
        q_array = m3d.DualQuaternion.FromArray(array_data)

        self.assertEqual(len(array_data), 8)
        self.assertEqual(array_data[0], q.real.w)
        self.assertEqual(array_data[1], q.real.x)
        self.assertEqual(array_data[2], q.real.y)
        self.assertEqual(array_data[3], q.real.z)
        self.assertEqual(array_data[4], q.dual.w)
        self.assertEqual(array_data[5], q.dual.x)
        self.assertEqual(array_data[6], q.dual.y)
        self.assertEqual(array_data[7], q.dual.z)
        np.testing.assert_equal(q.toArray(), q_array.toArray())

    def test_matrix(self):
        # create data
        q1 = m3d.DualQuaternion.FromArray(np.random.rand(8))
        q2 = m3d.DualQuaternion.FromArray(np.random.rand(8))

        # chain with multiplication
        q12 = q1 * q2

        # chain in matrix form
        q12_pos = q1.toPositiveMatrix().dot(q2.toArray())
        q12_neg = q2.toNegativeMatrix().dot(q1.toArray())

        # compare
        np.testing.assert_almost_equal(q12_pos, q12.toArray())
        np.testing.assert_almost_equal(q12_neg, q12.toArray())

    def test_description(self):
        q = m3d.DualQuaternion()
        self.assertTrue(str(q).startswith("DualQuaternion"))


if __name__ == '__main__':
    unittest.main()
