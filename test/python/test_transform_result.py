import copy
import unittest

import numpy as np

import motion3d as m3d

from data import TRANSFORM_CLASSES, TRANSFORM_ARRAYS_UNIT, TRANSFORM_ARRAYS1, TRANSFORM_ARRAYS2, TRANSFORM_ARRAYS12, \
    TRANSFORM_ARRAYS1_INV, ROTATION_NORM1, ROTATION_NORM2, TRANSLATION_NORM1, TRANSLATION_NORM2, POINT_0, POINT_1
from utils import N_RANDOM_RUNS, create_random_euler_transform, assert_transform_almost_equal


class TestTransformResult(unittest.TestCase):
    def test_unit_transform(self):
        for tcls, tarr in TRANSFORM_ARRAYS_UNIT.items():
            obj = tcls()
            np.testing.assert_array_equal(obj.toArray(), tarr)

    def test_normalized(self):
        # without modification
        for tcls, tarr in TRANSFORM_ARRAYS1.items():
            obj = tcls(tarr, True)
            obj_norm = obj.normalized()
            self.assertFalse(obj_norm.isUnsafe())
            np.testing.assert_almost_equal(obj_norm.toArray(), tarr)

        # axis angle
        axis_angle_arr = TRANSFORM_ARRAYS1[m3d.AxisAngleTransform]
        axis_angle = m3d.AxisAngleTransform(axis_angle_arr)

        axis_angle_mod = m3d.AxisAngleTransform(
            axis_angle.getTranslation(), -axis_angle.getAngle() - 2 * np.pi, -2.0 * axis_angle.getAxis(), True)
        axis_angle_mod_norm = axis_angle_mod.normalized()
        self.assertFalse(axis_angle_mod_norm.isUnsafe())
        np.testing.assert_array_almost_equal(axis_angle_mod_norm.toArray(), axis_angle_arr)
        np.testing.assert_array_almost_equal(axis_angle_mod.normalized_().toArray(), axis_angle_arr)

        # dual quaternion
        dual_quaternion_arr = TRANSFORM_ARRAYS1[m3d.DualQuaternionTransform]
        dual_quaternion = m3d.DualQuaternionTransform(dual_quaternion_arr)

        dual_quaternion_mod1 = m3d.DualQuaternionTransform(
            dual_quaternion.getReal() * -2.0,
            dual_quaternion.getDual() * -2.0,
            True)
        dual_quaternion_mod1_norm = dual_quaternion_mod1.normalized()
        self.assertFalse(dual_quaternion_mod1_norm.isUnsafe())
        np.testing.assert_array_almost_equal(dual_quaternion_mod1_norm.toArray(), dual_quaternion_arr)
        np.testing.assert_array_almost_equal(dual_quaternion_mod1.normalized_().toArray(), dual_quaternion_arr)

        translation_quaternion = dual_quaternion.getDualQuaternion().getTranslationQuaternion()
        translation_quaternion.w = 1.0
        real = dual_quaternion.getReal() * -2.0
        dual = (translation_quaternion * real) * 0.5
        dual_quaternion_mod2 = m3d.DualQuaternionTransform(real, dual, True)
        np.testing.assert_array_almost_equal(dual_quaternion_mod2.normalized().toArray(), dual_quaternion_arr)
        np.testing.assert_array_almost_equal(dual_quaternion_mod2.normalized_().toArray(), dual_quaternion_arr)

        # euler
        euler_arr = TRANSFORM_ARRAYS1[m3d.EulerTransform]
        euler = m3d.EulerTransform(euler_arr)

        euler_mod = m3d.EulerTransform(euler.getTranslation(), euler.getAi() + 2 * np.pi, euler.getAj() - 2 * np.pi,
                                       euler.getAk() + 2 * np.pi, euler.getAxes(), True)
        euler_mod_norm = euler_mod.normalized()
        self.assertFalse(euler_mod_norm.isUnsafe())
        np.testing.assert_array_almost_equal(euler_mod_norm.toArray(), euler_arr)
        np.testing.assert_array_almost_equal(euler_mod.normalized_().toArray(), euler_arr)

        # matrix
        matrix_arr = TRANSFORM_ARRAYS1[m3d.MatrixTransform]
        matrix = m3d.MatrixTransform(matrix_arr)

        zoom = np.diag([2.0, 3.0, 4.0])
        shear = np.eye(3)
        shear[0, 1] = 5.0
        shear[0, 2] = -6.0
        shear[1, 2] = 7.0

        matrix_mod_data = matrix.getMatrix()
        matrix_mod_data[:3, :3] = matrix_mod_data[:3, :3].dot(zoom).dot(shear)
        matrix_mod_data[3, :] = [1.0, 2.0, 3.0, 4.0]

        matrix_mod = m3d.MatrixTransform(matrix_mod_data, True)
        matrix_mod_norm = matrix_mod.normalized()
        self.assertFalse(matrix_mod_norm.isUnsafe())
        np.testing.assert_array_almost_equal(matrix_mod_norm.toArray(), matrix_arr)
        np.testing.assert_array_equal(matrix_mod_norm.getMatrix()[3, :], [0.0, 0.0, 0.0, 1.0])
        np.testing.assert_array_almost_equal(matrix_mod.normalized_().toArray(), matrix_arr)

        # quaternion
        quaternion_arr = TRANSFORM_ARRAYS1[m3d.QuaternionTransform]
        quaternion = m3d.QuaternionTransform(quaternion_arr)

        quaternion_mod = m3d.QuaternionTransform(
             quaternion.getTranslation(), quaternion.getQuaternion() * -2.0, True)
        quaternion_mod_norm = quaternion_mod.normalized()
        self.assertFalse(quaternion_mod_norm.isUnsafe())
        np.testing.assert_array_almost_equal(quaternion_mod_norm.toArray(), quaternion_arr)
        np.testing.assert_array_almost_equal(quaternion_mod.normalized_().toArray(), quaternion_arr)

        # inplace without modification
        for tcls, tarr in TRANSFORM_ARRAYS1.items():
            obj = tcls(tarr, True)
            obj.normalized_()
            self.assertFalse(obj.isUnsafe())
            np.testing.assert_almost_equal(obj.toArray(), tarr)

    def test_scale_translation(self):
        factor = 2.0

        # reference arrays
        scaled = copy.deepcopy(TRANSFORM_ARRAYS1)
        scaled[m3d.AxisAngleTransform][:3] = \
            np.array(scaled[m3d.AxisAngleTransform][:3]) * factor
        scaled[m3d.DualQuaternionTransform][4:] = \
            np.array(scaled[m3d.DualQuaternionTransform][4:]) * factor
        scaled[m3d.MatrixTransform][3] *= factor
        scaled[m3d.MatrixTransform][7] *= factor
        scaled[m3d.MatrixTransform][11] *= factor
        scaled[m3d.EulerTransform][:3] = \
            np.array(scaled[m3d.EulerTransform][:3]) * factor
        scaled[m3d.QuaternionTransform][:3] = \
            np.array(scaled[m3d.QuaternionTransform][:3]) * factor

        # scale
        for tcls, tarr in TRANSFORM_ARRAYS1.items():
            obj = tcls(tarr)

            # non-inplace
            np.testing.assert_almost_equal(obj.scaleTranslation(factor).toArray(), scaled[tcls])

            # inplace
            obj.scaleTranslation_(factor)
            np.testing.assert_almost_equal(obj.toArray(), scaled[tcls])

    def test_as_type(self):
        for tcls1, tarr1 in TRANSFORM_ARRAYS1.items():
            obj1 = tcls1(tarr1)
            for ttype2, tcls2 in TRANSFORM_CLASSES.items():
                tarr2 = TRANSFORM_ARRAYS1[tcls2]

                obj2 = obj1.asType(ttype2)
                np.testing.assert_almost_equal(obj2.normalized().toArray(), tarr2)

    def _as_type_random(self):
        random_transform = create_random_euler_transform()
        for (ttype1, _), (ttype2, _) in zip(TRANSFORM_CLASSES.items(), TRANSFORM_CLASSES.items()):
            t1 = random_transform.asType(ttype1)
            t2 = t1.asType(ttype2)
            assert_transform_almost_equal(self, t1, t2)

    def test_as_type_random(self):
        np.random.seed(0)
        for _ in range(N_RANDOM_RUNS):
            self._as_type_random()

    def test_euler_conversion(self):
        # generate data
        euler = m3d.EulerTransform(TRANSFORM_ARRAYS1[m3d.EulerTransform])
        transforms = [tcls(tarr) for tcls, tarr in TRANSFORM_ARRAYS1.items()]

        euler_axes_list = [a for a, _ in m3d.EulerAxes.__dict__['__entries'].values()]
        for axes1 in euler_axes_list:
            # convert axes
            euler_axes1 = euler.changeAxes(axes1)
            self.assertEqual(euler_axes1.getAxes(), axes1)

            # convert axes again
            for axes2 in euler_axes_list:
                euler_axes2 = euler_axes1.changeAxes(axes2)
                self.assertEqual(euler_axes2.getAxes(), axes2)
                assert_transform_almost_equal(self, euler_axes1, euler_axes2)

            # inverse
            self.assertEqual(euler_axes1.inverse().getAxes(), axes1)
            self.assertEqual(euler_axes1.copy().inverse_().getAxes(), axes1)

            # other types
            for transform in transforms:
                # constructor
                self.assertEqual(m3d.EulerTransform(transform, axes1).getAxes(), axes1)

                # apply
                self.assertEqual(euler_axes1.applyPre(transform).getAxes(), axes1)
                self.assertEqual(euler_axes1.applyPost(transform).getAxes(), axes1)
                self.assertEqual(euler_axes1.copy().applyPre_(transform).getAxes(), axes1)
                self.assertEqual(euler_axes1.copy().applyPost_(transform).getAxes(), axes1)

                # operators
                self.assertEqual((euler_axes1 * transform).getAxes(), axes1)
                self.assertEqual((euler_axes1 / transform).getAxes(), axes1)

    def _euler_conversion_random(self):
        # generate data
        random_transform = create_random_euler_transform()

        # convert
        euler_axes_list = [a for a, _ in m3d.EulerAxes.__dict__['__entries'].values()]
        for axes1, axes2 in zip(euler_axes_list, euler_axes_list):
            t1 = random_transform.changeAxes(axes1)
            t2 = t1.changeAxes(axes2)
            assert_transform_almost_equal(self, t1, t2)

    def test_euler_conversion_random(self):
        np.random.seed(0)
        for _ in range(N_RANDOM_RUNS):
            self._euler_conversion_random()

    def test_is_equal(self):
        # check representations
        for tcls1a, tarr1a in TRANSFORM_ARRAYS1.items():
            obj1a = tcls1a(tarr1a)
            for tcls1b, tarr1b in TRANSFORM_ARRAYS1.items():
                obj1b = tcls1b(tarr1b)
                self.assertTrue(obj1a.isEqual(obj1b))
            for tcls2, tarr2 in TRANSFORM_ARRAYS2.items():
                obj2 = tcls2(tarr2)
                self.assertFalse(obj1a.isEqual(obj2))

        # negative dual quaternion
        dq1 = m3d.DualQuaternionTransform(TRANSFORM_ARRAYS1[m3d.DualQuaternionTransform])
        dq1_neg = m3d.DualQuaternionTransform(-np.array(TRANSFORM_ARRAYS1[m3d.DualQuaternionTransform]))
        self.assertTrue(dq1.isEqual(dq1_neg))

    def test_inverse(self):
        for tcls, tarr in TRANSFORM_ARRAYS1.items():
            obj = tcls(tarr)

            # non-inplace
            np.testing.assert_almost_equal(obj.inverse().normalized().toArray(), TRANSFORM_ARRAYS1_INV[tcls])

            # inplace
            obj.inverse_()
            np.testing.assert_almost_equal(obj.normalized().toArray(), TRANSFORM_ARRAYS1_INV[tcls])

    def test_dual_quaternion_matrix(self):
        # generate data
        dq1 = m3d.DualQuaternionTransform(TRANSFORM_ARRAYS1[m3d.DualQuaternionTransform])
        dq2 = m3d.DualQuaternionTransform(TRANSFORM_ARRAYS2[m3d.DualQuaternionTransform])

        # multiply
        dq12 = dq1 * dq2

        dq12_pos = dq1.getDualQuaternion().toPositiveMatrix().dot(dq2.toArray())
        dq12_neg = dq2.getDualQuaternion().toNegativeMatrix().dot(dq1.toArray())

        # check
        np.testing.assert_almost_equal(dq12_pos, dq12.toArray())
        np.testing.assert_almost_equal(dq12_neg, dq12.toArray())

    def test_apply(self):
        point0 = np.array(POINT_0)
        point1 = np.array(POINT_1)
        cloud0 = np.array([point0, point0, point0, point0, point0]).T
        cloud1 = np.array([point1, point1, point1, point1, point1]).T

        for tcls1, tarr1 in TRANSFORM_ARRAYS1.items():
            obj1 = tcls1(tarr1)

            # transforms
            for tcls2, tarr2 in TRANSFORM_ARRAYS2.items():
                obj2 = tcls2(tarr2)

                tmp = obj1.applyPost(obj2)
                np.testing.assert_almost_equal(tmp.normalized().toArray(), TRANSFORM_ARRAYS12[tcls1])
                tmp = obj2.applyPre(obj1)
                np.testing.assert_almost_equal(tmp.normalized().toArray(), TRANSFORM_ARRAYS12[tcls2])

                tmp = obj1.copy().applyPost_(obj2)
                np.testing.assert_almost_equal(tmp.normalized().toArray(), TRANSFORM_ARRAYS12[tcls1])
                tmp = obj2.copy().applyPre_(obj1)
                np.testing.assert_almost_equal(tmp.normalized().toArray(), TRANSFORM_ARRAYS12[tcls2])

            # points
            np.testing.assert_almost_equal(obj1.transformPoint(point0), point1)
            np.testing.assert_almost_equal(obj1.transformCloud(cloud0), cloud1)

    def test_norm(self):
        # first transform
        for tcls1, tarr1 in TRANSFORM_ARRAYS1.items():
            obj = tcls1(tarr1)
            np.testing.assert_almost_equal(obj.rotationNorm(), ROTATION_NORM1)
            np.testing.assert_almost_equal(obj.translationNorm(), TRANSLATION_NORM1)

        # second transform
        for tcls2, tarr2 in TRANSFORM_ARRAYS2.items():
            obj = tcls2(tarr2)
            np.testing.assert_almost_equal(obj.rotationNorm(), ROTATION_NORM2)
            np.testing.assert_almost_equal(obj.translationNorm(), TRANSLATION_NORM2)

    def test_operators(self):
        for tcls1, tarr1 in TRANSFORM_ARRAYS1.items():
            obj1 = tcls1(tarr1)

            for tcls2, tarr2 in TRANSFORM_ARRAYS2.items():
                obj2 = tcls2(tarr2)

                # multiply
                tmp = obj1 * obj2
                np.testing.assert_almost_equal(tmp.normalized().toArray(), TRANSFORM_ARRAYS12[tcls1])

                # divide
                tmp = obj1 / obj1
                np.testing.assert_almost_equal(tmp.normalized().toArray(), TRANSFORM_ARRAYS_UNIT[tcls1])

    def _apply_random(self):
        transform1 = create_random_euler_transform()
        transform2 = create_random_euler_transform()
        transform_identity = m3d.DualQuaternionTransform()
        transform_fixed = m3d.DualQuaternionTransform(TRANSFORM_ARRAYS1[m3d.DualQuaternionTransform])
        transform12 = transform1 * transform2

        for ttype1, _ in TRANSFORM_CLASSES.items():
            # identity as result
            t1 = transform1.asType(ttype1)
            assert_transform_almost_equal(self, transform_identity, t1 * t1.inverse())
            assert_transform_almost_equal(self, transform_identity, t1.inverse() * t1)
            assert_transform_almost_equal(self, transform_identity, t1 / t1)
            np.testing.assert_almost_equal((t1 / t1).rotationNorm(), 0.0)
            np.testing.assert_almost_equal((t1 / t1).translationNorm(), 0.0)
            self.assertFalse(t1.isEqual(t1 * transform_fixed))

            t1_copy = t1.copy()
            t1_copy *= t1.inverse()
            assert_transform_almost_equal(self, transform_identity, t1_copy)

            t1_copy = t1.copy()
            t1_copy /= t1
            assert_transform_almost_equal(self, transform_identity, t1_copy)

            # chain with other transform
            for ttype2, _ in TRANSFORM_CLASSES.items():
                t2 = transform2.asType(ttype2)
                assert_transform_almost_equal(self, transform12, t1 * t2)
                assert_transform_almost_equal(self, transform12, t1.applyPost(t2))
                assert_transform_almost_equal(self, transform12, t2.applyPre(t1))
                assert_transform_almost_equal(self, transform12, t1.copy().applyPost_(t2))
                assert_transform_almost_equal(self, transform12, t2.copy().applyPre_(t1))
                assert_transform_almost_equal(self, transform1, (t1 * t2) / t2)
                self.assertTrue(transform12.isEqual(t1 * t2))

                t1_copy = t1.copy()
                t1_copy *= t2
                assert_transform_almost_equal(self, transform12, t1_copy)
                t1_copy /= t2
                assert_transform_almost_equal(self, transform1, t1_copy)

    def test_apply_random(self):
        np.random.seed(0)
        for _ in range(N_RANDOM_RUNS):
            self._apply_random()


if __name__ == '__main__':
    unittest.main()
