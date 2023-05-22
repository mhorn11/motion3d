import unittest

import motion3d as m3d
import numpy as np


class TestMath(unittest.TestCase):
    def _assert_valid_rotation_matrix(self, m):
        orthogonal_error = np.linalg.norm(m.T @ m - np.eye(3), ord=np.inf)
        is_orthogonal = orthogonal_error < 1e-6
        if not is_orthogonal:
            raise AssertionError(f"Matrix is not orthogonal, error: {orthogonal_error}")

        det = np.linalg.det(m)
        correct_determinant = np.abs(det - 1.0) < 1e-6
        if not correct_determinant:
            raise AssertionError(f"Matrix determinant is not 1.0 ({det})")

    def test_normalize_angle(self):
        self.assertEqual(m3d.normalizeAngle(0.0), 0.0)
        self.assertEqual(m3d.normalizeAngle(1.5), 1.5)
        self.assertEqual(m3d.normalizeAngle(3.0), 3.0)
        self.assertEqual(m3d.normalizeAngle(-1.5), -1.5)
        self.assertEqual(m3d.normalizeAngle(-3.0), -3.0)
        self.assertEqual(m3d.normalizeAngle(-np.pi), -np.pi)
        self.assertEqual(m3d.normalizeAngle(np.pi), -np.pi)
        self.assertEqual(m3d.normalizeAngle(2 * np.pi), 0)
        self.assertEqual(m3d.normalizeAngle(3 * np.pi / 2), - np.pi / 2)
        self.assertEqual(m3d.normalizeAngle(- 3 * np.pi / 2), np.pi / 2)

    def test_axis_normalization_factor_inverse(self):
        self.assertEqual(1.0, m3d.getAxisNormalizationFactorInverse(np.array([0, 0, 0]), False))
        self.assertEqual(1.0, m3d.getAxisNormalizationFactorInverse(np.array([1, 0, 0]), False))
        self.assertEqual(1.0, m3d.getAxisNormalizationFactorInverse(np.array([2, 0, 0]), False))
        self.assertEqual(1.0, m3d.getAxisNormalizationFactorInverse(np.array([1, -1, 0]), False))
        self.assertEqual(-1.0, m3d.getAxisNormalizationFactorInverse(np.array([-1, 1, 0]), False))
        self.assertEqual(-1.0, m3d.getAxisNormalizationFactorInverse(np.array([-1, -1, 0]), False))
        self.assertEqual(1.0, m3d.getAxisNormalizationFactorInverse(np.array([0, 3, 0]), False))
        self.assertEqual(-1.0, m3d.getAxisNormalizationFactorInverse(np.array([0, -1, 0]), False))
        self.assertEqual(-1.0, m3d.getAxisNormalizationFactorInverse(np.array([0, -0.2, -1]), False))
        self.assertEqual(1.0, m3d.getAxisNormalizationFactorInverse(np.array([0, 0, 1]), False))
        self.assertEqual(-1.0, m3d.getAxisNormalizationFactorInverse(np.array([0, 0, -0.5]), False))

        self.assertEqual(0.0, m3d.getAxisNormalizationFactorInverse(np.array([0, 0, 0]), True))
        self.assertEqual(1.0, m3d.getAxisNormalizationFactorInverse(np.array([1, 0, 0]), True))
        self.assertEqual(2.0, m3d.getAxisNormalizationFactorInverse(np.array([2, 0, 0]), True))
        self.assertEqual(np.sqrt(2), m3d.getAxisNormalizationFactorInverse(np.array([1, -1, 0]), True))
        self.assertEqual(-np.sqrt(2), m3d.getAxisNormalizationFactorInverse(np.array([-1, 1, 0]), True))
        self.assertEqual(-np.sqrt(2), m3d.getAxisNormalizationFactorInverse(np.array([-1, -1, 0]), True))
        self.assertEqual(3.0, m3d.getAxisNormalizationFactorInverse(np.array([0, 3, 0]), True))
        self.assertEqual(-1.0, m3d.getAxisNormalizationFactorInverse(np.array([0, -1, 0]), True))
        self.assertEqual(0.5, m3d.getAxisNormalizationFactorInverse(np.array([0, 0, 0.5]), True))

    def test_normalize_rotation_matrix(self):
        zero = np.zeros((3, 3))
        identity = np.eye(3)
        np.testing.assert_equal(identity, m3d.normalizeRotationMatrix(zero))
        np.testing.assert_almost_equal(identity, m3d.normalizeRotationMatrix(identity))
        np.testing.assert_almost_equal(identity, m3d.normalizeRotationMatrix(0.5 * identity))
        np.testing.assert_almost_equal(identity, m3d.normalizeRotationMatrix(2 * identity))

        rotation = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        np.testing.assert_almost_equal(rotation, m3d.normalizeRotationMatrix(rotation))
        np.testing.assert_almost_equal(rotation, m3d.normalizeRotationMatrix(0.5 * rotation))
        np.testing.assert_almost_equal(rotation, m3d.normalizeRotationMatrix(2 * rotation))

        self._assert_valid_rotation_matrix(m3d.normalizeRotationMatrix(-identity))
        self._assert_valid_rotation_matrix(m3d.normalizeRotationMatrix(-rotation))

    def test_decompose_rzs(self):
        # create data
        rotation = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        zoom = np.array([2.0, 3.0, 4.0])
        shear = np.array([5.0, -6.0, 7.0])

        # compose matrix
        shear_mat = np.eye(3)
        shear_mat[0, 1] = shear[0]
        shear_mat[0, 2] = shear[1]
        shear_mat[1, 2] = shear[2]

        mat = rotation @ np.diag(zoom) @ shear_mat

        # decompose matrix
        rotation_dec, zoom_dec, shear_dec = m3d.decomposeRZS(mat)

        # check results
        self._assert_valid_rotation_matrix(rotation_dec)
        np.testing.assert_almost_equal(rotation, rotation_dec)
        np.testing.assert_almost_equal(zoom, zoom_dec)
        np.testing.assert_almost_equal(shear, shear_dec)


if __name__ == '__main__':
    unittest.main()
