import unittest

import numpy as np

import motion3d as m3d

from data import TRANSFORM_CLASSES, TRANSFORM_ARRAYS, TRANSFORM_ARRAYS_UNIT, TRANSFORM_ARRAYS1, TRANSFORM_ARRAYS2


class TestTransformInterface(unittest.TestCase):
    def test_types(self):
        # from char
        self.assertEqual(m3d.TransformType.FromChar('A'), m3d.TransformType.kAxisAngle)
        self.assertEqual(m3d.TransformType.FromChar('D'), m3d.TransformType.kDualQuaternion)
        self.assertEqual(m3d.TransformType.FromChar('E'), m3d.TransformType.kEuler)
        self.assertEqual(m3d.TransformType.FromChar('M'), m3d.TransformType.kMatrix)
        self.assertEqual(m3d.TransformType.FromChar('Q'), m3d.TransformType.kQuaternion)

        self.assertEqual(m3d.TransformType.FromChar('A'), m3d.TransformType.kAxisAngle)
        self.assertEqual(m3d.TransformType.FromChar('D'), m3d.TransformType.kDualQuaternion)
        self.assertEqual(m3d.TransformType.FromChar('E'), m3d.TransformType.kEuler)
        self.assertEqual(m3d.TransformType.FromChar('M'), m3d.TransformType.kMatrix)
        self.assertEqual(m3d.TransformType.FromChar('Q'), m3d.TransformType.kQuaternion)

        # to char
        self.assertEqual('A', m3d.TransformType.kAxisAngle.toChar())
        self.assertEqual('D', m3d.TransformType.kDualQuaternion.toChar())
        self.assertEqual('E', m3d.TransformType.kEuler.toChar())
        self.assertEqual('M', m3d.TransformType.kMatrix.toChar())
        self.assertEqual('Q', m3d.TransformType.kQuaternion.toChar())

        # error handling
        self.assertRaises(m3d.InvalidTransformTypeException, lambda: m3d.TransformType.FromChar('X'))

    def test_constructors(self):
        # data arrays
        axis_angle1_arr = np.array(TRANSFORM_ARRAYS1[m3d.AxisAngleTransform])
        dual_quaternion1_arr = np.array(TRANSFORM_ARRAYS1[m3d.DualQuaternionTransform])
        euler1_arr = np.array(TRANSFORM_ARRAYS1[m3d.EulerTransform])
        matrix1_arr = np.array(TRANSFORM_ARRAYS1[m3d.MatrixTransform])
        quaternion1_arr = np.array(TRANSFORM_ARRAYS1[m3d.QuaternionTransform])

        # comparison between transform and array
        def check(transform, array):
            np.testing.assert_equal(transform.toArray(), array)

        # AxisAngleTransform
        check(m3d.AxisAngleTransform(), TRANSFORM_ARRAYS_UNIT[m3d.AxisAngleTransform])
        check(m3d.AxisAngleTransform(axis_angle1_arr[:3], axis_angle1_arr[3], axis_angle1_arr[4:]), axis_angle1_arr)
        check(m3d.AxisAngleTransform(axis_angle1_arr[:3].tolist(), axis_angle1_arr[3].tolist(),
                                     axis_angle1_arr[4:].tolist()), axis_angle1_arr)
        check(m3d.AxisAngleTransform(axis_angle1_arr), axis_angle1_arr)
        check(m3d.AxisAngleTransform(axis_angle1_arr.tolist()), axis_angle1_arr)

        # DualQuaternionTransform
        real = m3d.Quaternion.FromArray(dual_quaternion1_arr[:4])
        dual = m3d.Quaternion.FromArray(dual_quaternion1_arr[4:])

        check(m3d.DualQuaternionTransform(), TRANSFORM_ARRAYS_UNIT[m3d.DualQuaternionTransform])
        check(m3d.DualQuaternionTransform(real, dual), dual_quaternion1_arr)
        check(m3d.DualQuaternionTransform(dual_quaternion1_arr[:4], dual_quaternion1_arr[4:]), dual_quaternion1_arr)
        check(m3d.DualQuaternionTransform(dual_quaternion1_arr[:4].tolist(), dual_quaternion1_arr[4:].tolist()),
              dual_quaternion1_arr)
        check(m3d.DualQuaternionTransform(dual_quaternion1_arr), dual_quaternion1_arr)
        check(m3d.DualQuaternionTransform(dual_quaternion1_arr.tolist()), dual_quaternion1_arr)

        # EulerTransform
        check(m3d.EulerTransform(), TRANSFORM_ARRAYS_UNIT[m3d.EulerTransform])
        check(m3d.EulerTransform(euler1_arr[:3], euler1_arr[3], euler1_arr[4], euler1_arr[5],
                                 m3d.EulerAxes.kSXYZ), euler1_arr)
        check(m3d.EulerTransform(euler1_arr[:3].tolist(), euler1_arr[3], euler1_arr[4], euler1_arr[5],
                                 m3d.EulerAxes.kSXYZ), euler1_arr)
        check(m3d.EulerTransform(euler1_arr[:3], euler1_arr[3:6], m3d.EulerAxes.kSXYZ), euler1_arr)
        check(m3d.EulerTransform(euler1_arr[:3].tolist(), euler1_arr[3:6].tolist(), m3d.EulerAxes.kSXYZ), euler1_arr)
        check(m3d.EulerTransform(euler1_arr), euler1_arr)
        check(m3d.EulerTransform(euler1_arr.tolist()), euler1_arr)

        # MatrixTransform
        matrix44 = np.eye(4)
        matrix44[:3, :] = matrix1_arr.reshape(3, 4)

        check(m3d.MatrixTransform(), TRANSFORM_ARRAYS_UNIT[m3d.MatrixTransform])
        check(m3d.MatrixTransform(matrix44), matrix1_arr)
        check(m3d.MatrixTransform(matrix44.tolist()), matrix1_arr)
        check(m3d.MatrixTransform(matrix44[:3, :]), matrix1_arr)
        check(m3d.MatrixTransform(matrix44[:3, :].tolist()), matrix1_arr)
        check(m3d.MatrixTransform(matrix44[:3, 3], matrix44[:3, :3]), matrix1_arr)
        check(m3d.MatrixTransform(matrix44[:3, 3].tolist(), matrix44[:3, :3].tolist()), matrix1_arr)
        check(m3d.MatrixTransform(matrix1_arr), matrix1_arr)
        check(m3d.MatrixTransform(matrix1_arr.tolist()), matrix1_arr)

        # QuaternionTransform
        quat = m3d.Quaternion.FromArray(quaternion1_arr[3:])

        check(m3d.QuaternionTransform(), TRANSFORM_ARRAYS_UNIT[m3d.QuaternionTransform])
        check(m3d.QuaternionTransform(quaternion1_arr[:3], quat), quaternion1_arr)
        check(m3d.QuaternionTransform(quaternion1_arr[:3], quaternion1_arr[3:]), quaternion1_arr)
        check(m3d.QuaternionTransform(quaternion1_arr[:3].tolist(), quaternion1_arr[3:].tolist()), quaternion1_arr)
        check(m3d.QuaternionTransform(quaternion1_arr), quaternion1_arr)
        check(m3d.QuaternionTransform(quaternion1_arr.tolist()), quaternion1_arr)

    def test_getter(self):
        # create transforms
        axis_angle1_arr = np.array(TRANSFORM_ARRAYS1[m3d.AxisAngleTransform])
        dual_quaternion1_arr = np.array(TRANSFORM_ARRAYS1[m3d.DualQuaternionTransform])
        euler1_arr = np.array(TRANSFORM_ARRAYS1[m3d.EulerTransform])
        matrix1_arr = np.array(TRANSFORM_ARRAYS1[m3d.MatrixTransform])
        quaternion1_arr = np.array(TRANSFORM_ARRAYS1[m3d.QuaternionTransform])

        axis_angle = m3d.AxisAngleTransform(axis_angle1_arr)
        dual_quaternion = m3d.DualQuaternionTransform(dual_quaternion1_arr)
        euler = m3d.EulerTransform(euler1_arr)
        matrix = m3d.MatrixTransform(matrix1_arr)
        quaternion = m3d.QuaternionTransform(quaternion1_arr)

        # AxisAngleTransform
        np.testing.assert_equal(axis_angle.getTranslation(), axis_angle1_arr[:3])
        np.testing.assert_equal(axis_angle.getAngle(), axis_angle1_arr[3])
        np.testing.assert_equal(axis_angle.getAxis(), axis_angle1_arr[4:])

        # DualQuaternionTransform
        np.testing.assert_equal(dual_quaternion.getDualQuaternion().toArray(), dual_quaternion1_arr)
        np.testing.assert_equal(dual_quaternion.getReal().toArray(), dual_quaternion1_arr[:4])
        np.testing.assert_equal(dual_quaternion.getDual().toArray(), dual_quaternion1_arr[4:])
        self.assertAlmostEqual(dual_quaternion.getDualQuaternion().getTranslationQuaternion().w, 0.0)
        np.testing.assert_almost_equal(dual_quaternion.getDualQuaternion().getTranslationQuaternion().vec(),
                                       axis_angle1_arr[:3])

        # EulerTransform
        np.testing.assert_equal(euler.getTranslation(), euler1_arr[:3])
        np.testing.assert_equal(euler.getAi(), euler1_arr[3])
        np.testing.assert_equal(euler.getAj(), euler1_arr[4])
        np.testing.assert_equal(euler.getAk(), euler1_arr[5])
        np.testing.assert_equal(euler.getAngles(), euler1_arr[3:6])
        np.testing.assert_equal(euler.getAxes(), m3d.EulerAxes.kSXYZ)

        # MatrixTransform
        matrix44 = np.eye(4)
        matrix44[:3, :] = matrix1_arr.reshape(3, 4)

        np.testing.assert_equal(matrix.getMatrix(), matrix44)
        np.testing.assert_equal(matrix.getTranslation(), matrix44[:3, 3])
        np.testing.assert_equal(matrix.getRotationMatrix(), matrix44[:3, :3])

        # QuaternionTransform
        np.testing.assert_equal(quaternion.getTranslation(), quaternion1_arr[:3])
        np.testing.assert_equal(quaternion.getQuaternion().toArray(), quaternion1_arr[3:])

    def test_setter(self):
        # AxisAngleTransform
        aa = m3d.AxisAngleTransform()
        aa1 = m3d.AxisAngleTransform(TRANSFORM_ARRAYS1[m3d.AxisAngleTransform])
        aa2 = m3d.AxisAngleTransform(TRANSFORM_ARRAYS2[m3d.AxisAngleTransform])

        np.testing.assert_array_equal(aa.setTranslation(aa1.getTranslation()).getTranslation(), aa1.getTranslation())
        self.assertEqual(aa.setAngle(aa2.getAngle()).getAngle(), aa2.getAngle())
        np.testing.assert_array_equal(aa.setAxis(aa2.getAxis()).getAxis(), aa2.getAxis())
        self.assertFalse(aa.isUnsafe())

        aa_copy = aa.copy()
        self.assertRaises(m3d.InvalidTransformException, lambda: aa.setAxis(np.zeros(3)))
        np.testing.assert_array_equal(aa.toArray(), aa_copy.toArray())

        aa.setAxis(np.zeros(3), True)
        self.assertTrue(aa.isUnsafe())

        # DualQuaternionTransform
        dq = m3d.DualQuaternionTransform()
        dq1 = m3d.DualQuaternionTransform(TRANSFORM_ARRAYS1[m3d.DualQuaternionTransform])

        np.testing.assert_array_equal(dq.setDualQuaternion(dq1.getDualQuaternion()).toArray(), dq1.toArray())
        np.testing.assert_array_equal(dq.setReal(-dq1.getReal()).getReal().toArray(), -dq1.getReal().toArray())
        np.testing.assert_array_equal(dq.setDual(-dq1.getDual()).getDual().toArray(), -dq1.getDual().toArray())
        self.assertFalse(dq.isUnsafe())

        dq_copy = dq.copy()
        self.assertRaises(m3d.InvalidTransformException, lambda: dq.setDualQuaternion(m3d.DualQuaternion.Zero()))
        np.testing.assert_array_equal(dq.toArray(), dq_copy.toArray())
        self.assertRaises(m3d.InvalidTransformException, lambda: dq.setReal(m3d.Quaternion.Zero()))
        np.testing.assert_array_equal(dq.toArray(), dq_copy.toArray())

        dq.setDualQuaternion(m3d.DualQuaternion.Zero(), True)
        self.assertTrue(dq.isUnsafe())
        dq = m3d.DualQuaternionTransform()
        dq.setReal(m3d.Quaternion.Zero(), True)
        self.assertTrue(dq.isUnsafe())

        # EulerTransform
        euler = m3d.EulerTransform()
        euler1 = m3d.EulerTransform(TRANSFORM_ARRAYS1[m3d.EulerTransform])
        euler2 = m3d.EulerTransform(TRANSFORM_ARRAYS2[m3d.EulerTransform])

        np.testing.assert_array_equal(euler.setTranslation(euler1.getTranslation()).getTranslation(),
                                      euler1.getTranslation())
        self.assertEqual(euler.setAi(euler1.getAi()).getAi(), euler1.getAi())
        self.assertEqual(euler.setAj(euler1.getAj()).getAj(), euler1.getAj())
        self.assertEqual(euler.setAk(euler1.getAk()).getAk(), euler1.getAk())
        np.testing.assert_array_equal(euler.setAngles(euler2.getAngles()).getAngles(), euler2.getAngles())
        self.assertEqual(euler.setAxes(m3d.EulerAxes.kSXYX).getAxes(), m3d.EulerAxes.kSXYX)
        self.assertFalse(euler.isUnsafe())

        # MatrixTransform
        mat = m3d.MatrixTransform()
        mat1 = m3d.MatrixTransform(TRANSFORM_ARRAYS1[m3d.MatrixTransform])
        mat2 = m3d.MatrixTransform(TRANSFORM_ARRAYS2[m3d.MatrixTransform])

        np.testing.assert_array_equal(mat.setRotationMatrix(mat1.getRotationMatrix()).getRotationMatrix(),
                                      mat1.getRotationMatrix())
        np.testing.assert_array_equal(mat.setTranslation(mat1.getTranslation()).getTranslation(),
                                      mat1.getTranslation())
        np.testing.assert_array_equal(mat.setMatrix(mat2.getMatrix()).getMatrix(),
                                      mat2.getMatrix())
        self.assertFalse(mat.isUnsafe())

        mat_copy = mat.copy()
        self.assertRaises(m3d.InvalidTransformException, lambda: mat.setRotationMatrix(np.zeros((3, 3))))
        np.testing.assert_array_equal(mat.toArray(), mat_copy.toArray())
        self.assertRaises(m3d.InvalidTransformException, lambda: mat.setMatrix(np.zeros((4, 4))))
        np.testing.assert_array_equal(mat.toArray(), mat_copy.toArray())
        self.assertRaises(m3d.InvalidTransformException, lambda: mat.setMatrix(np.zeros((3, 4))))
        np.testing.assert_array_equal(mat.toArray(), mat_copy.toArray())

        mat.setRotationMatrix(np.zeros((3, 3)), True)
        self.assertTrue(mat.isUnsafe())
        mat = m3d.MatrixTransform()
        mat.setMatrix(np.zeros((4, 4)), True)
        self.assertTrue(mat.isUnsafe())
        mat = m3d.MatrixTransform()
        mat.setMatrix(np.zeros((3, 4)), True)
        self.assertTrue(mat.isUnsafe())

        # QuaternionTransform
        quat = m3d.QuaternionTransform()
        quat1 = m3d.QuaternionTransform(TRANSFORM_ARRAYS1[m3d.QuaternionTransform])

        np.testing.assert_array_equal(quat.setTranslation(quat1.getTranslation()).getTranslation(),
                                      quat1.getTranslation())
        np.testing.assert_array_equal(quat.setQuaternion(quat1.getQuaternion()).getQuaternion().toArray(),
                                      quat1.getQuaternion().toArray())
        self.assertFalse(quat.isUnsafe())

        quat_copy = quat.copy()
        self.assertRaises(m3d.InvalidTransformException, lambda: quat.setQuaternion(m3d.Quaternion.Zero()))
        np.testing.assert_array_equal(quat.toArray(), quat_copy.toArray())

        quat.setQuaternion(m3d.Quaternion.Zero(), True)
        self.assertTrue(quat.isUnsafe())

    def test_is_type(self):
        for ttype1, tcls1 in TRANSFORM_CLASSES.items():
            obj = tcls1()
            for ttype2 in TRANSFORM_CLASSES:
                if ttype1 == ttype2:
                    self.assertTrue(obj.isType(ttype2))
                else:
                    self.assertFalse(obj.isType(ttype2))

    def test_factory(self):
        # identity transform
        for ttype1 in TRANSFORM_CLASSES:
            transform = m3d.TransformInterface.Factory(ttype1)
            for ttype2, tcls2 in TRANSFORM_CLASSES.items():
                if ttype1 == ttype2:
                    self.assertIsInstance(transform, tcls2)
                    self.assertTrue(transform.isType(ttype2))
                else:
                    self.assertNotIsInstance(transform, tcls2)
                    self.assertFalse(transform.isType(ttype2))

        # transform from array
        for ttype1, tarr1 in TRANSFORM_ARRAYS.items():
            transform = m3d.TransformInterface.Factory(ttype1, tarr1)
            for ttype2, tcls2 in TRANSFORM_CLASSES.items():
                if ttype1 == ttype2:
                    self.assertIsInstance(transform, tcls2)
                    self.assertTrue(transform.isType(ttype2))
                else:
                    self.assertNotIsInstance(transform, tcls2)
                    self.assertFalse(transform.isType(ttype2))

        # transform from list
        for ttype1, tarr1 in TRANSFORM_ARRAYS.items():
            transform = m3d.TransformInterface.Factory(ttype1, tarr1.tolist())
            for ttype2, tcls2 in TRANSFORM_CLASSES.items():
                if ttype1 == ttype2:
                    self.assertIsInstance(transform, tcls2)
                    self.assertTrue(transform.isType(ttype2))
                else:
                    self.assertNotIsInstance(transform, tcls2)
                    self.assertFalse(transform.isType(ttype2))

    def test_copy(self):
        for ttype, tcls in TRANSFORM_CLASSES.items():
            transform = tcls()
            transform_copy = transform.copy()
            self.assertNotEqual(transform, transform_copy)
            self.assertTrue(transform_copy.isType(ttype))

    def test_identity(self):
        for ttype, tcls in TRANSFORM_CLASSES.items():
            transform = m3d.TransformInterface.Factory(ttype, TRANSFORM_ARRAYS1[tcls])

            # copy
            transform_id = transform.identity()
            self.assertTrue(transform_id.isType(ttype))
            np.testing.assert_array_equal(transform_id.toArray(), TRANSFORM_ARRAYS_UNIT[tcls])
            np.testing.assert_array_equal(transform.toArray(), TRANSFORM_ARRAYS1[tcls])

            # inplace
            transform.setIdentity()
            self.assertTrue(transform.isType(ttype))
            np.testing.assert_array_equal(transform.toArray(), TRANSFORM_ARRAYS_UNIT[tcls])

    def test_type_conversion(self):
        for ttype1, tcls1 in TRANSFORM_CLASSES.items():
            obj = tcls1()

            # interface
            for ttype2, tcls2 in TRANSFORM_CLASSES.items():
                tmp = obj.asType(ttype2)
                self.assertIsInstance(tmp, tcls2)
                self.assertTrue(tmp.isType(ttype2))

    def test_get_vector(self):
        # direct
        for ttype1, tarr1 in TRANSFORM_ARRAYS.items():
            obj1 = TRANSFORM_CLASSES[ttype1](tarr1)

            # list
            tmp = obj1.toList()
            self.assertIsInstance(tmp, list)
            self.assertEqual(len(tmp), tarr1.shape[0])
            self.assertEqual(len(tmp), ttype1.getVectorSize())

            # binary
            tmp = obj1.toBinary()
            self.assertIsInstance(tmp, list)
            self.assertEqual(len(tmp), ttype1.getBinarySize())

            # array
            tmp = obj1.toArray()
            self.assertIsInstance(tmp, np.ndarray)
            np.testing.assert_array_equal(tmp, tarr1)
            self.assertEqual(len(tmp), ttype1.getVectorSize())
            self.assertEqual(tmp.dtype, np.float64)

            # indirect
            for ttype2, tarr2 in TRANSFORM_ARRAYS.items():
                obj2 = obj1.asType(ttype2)

                # list
                tmp = obj2.toList()
                self.assertIsInstance(tmp, list)
                self.assertEqual(len(tmp), tarr2.shape[0])
                self.assertEqual(len(tmp), ttype2.getVectorSize())

                # binary
                tmp = obj2.toBinary()
                self.assertIsInstance(tmp, list)
                self.assertEqual(len(tmp), ttype2.getBinarySize())

                # array
                tmp = obj2.toArray()
                self.assertIsInstance(tmp, np.ndarray)
                self.assertEqual(tmp.shape, tarr2.shape)
                self.assertEqual(len(tmp), ttype2.getVectorSize())
                self.assertEqual(tmp.dtype, np.float64)

    def test_is_valid(self):
        for ttype, tarr in TRANSFORM_ARRAYS.items():
            transform_valid = m3d.TransformInterface.Factory(ttype, tarr)
            self.assertTrue(transform_valid.isValid())

            # everything is valid for Euler
            if ttype != m3d.TransformType.kEuler:
                self.assertRaises(m3d.InvalidTransformException,
                                  lambda: m3d.TransformInterface.Factory(ttype, np.ones(tarr.shape)))
                transform_invalid = m3d.TransformInterface.Factory(ttype, np.ones(tarr.shape), unsafe=True)
                self.assertFalse(transform_invalid.isValid())
                self.assertTrue(transform_invalid.isValid(np.inf))

    def test_inplace_return(self):
        # general
        for ttype in TRANSFORM_ARRAYS.keys():
            # create transform
            obj = TRANSFORM_CLASSES[ttype]()

            # check return value
            self.assertEqual(obj, obj.setIdentity())
            self.assertEqual(obj, obj.inverse_())
            self.assertEqual(obj, obj.normalized_())
            self.assertEqual(obj, obj.scaleTranslation_(1.0))

            for ttype2 in TRANSFORM_ARRAYS.keys():
                other = TRANSFORM_CLASSES[ttype2]()
                self.assertEqual(obj, obj.applyPre_(other))
                self.assertEqual(obj, obj.applyPost_(other))

        # transform-specific
        aa = m3d.AxisAngleTransform()
        self.assertEqual(aa, aa.setTranslation(aa.getTranslation()))
        self.assertEqual(aa, aa.setAngle(aa.getAngle()))
        self.assertEqual(aa, aa.setAxis(aa.getAxis()))

        dq = m3d.DualQuaternionTransform()
        self.assertEqual(dq, dq.setDualQuaternion(dq.getDualQuaternion()))
        self.assertEqual(dq, dq.setReal(dq.getReal()))
        self.assertEqual(dq, dq.setDual(dq.getDual()))

        euler = m3d.EulerTransform()
        self.assertEqual(euler, euler.setTranslation(euler.getTranslation()))
        self.assertEqual(euler, euler.setAi(euler.getAi()))
        self.assertEqual(euler, euler.setAj(euler.getAj()))
        self.assertEqual(euler, euler.setAk(euler.getAk()))
        self.assertEqual(euler, euler.setAngles(euler.getAngles()))
        self.assertEqual(euler, euler.setAxes(euler.getAxes()))
        self.assertEqual(euler, euler.changeAxes_(m3d.EulerAxes.kSZYX))

        mat = m3d.MatrixTransform()
        self.assertEqual(mat, mat.setMatrix(mat.getMatrix()[:3, :]))
        self.assertEqual(mat, mat.setMatrix(mat.getMatrix()))
        self.assertEqual(mat, mat.setTranslation(mat.getTranslation()))
        self.assertEqual(mat, mat.setRotationMatrix(mat.getRotationMatrix()))

        quat = m3d.QuaternionTransform()
        self.assertEqual(quat, quat.setTranslation(quat.getTranslation()))
        self.assertEqual(quat, quat.setQuaternion(quat.getQuaternion()))

    def test_inverse(self):
        for ttype, tcls in TRANSFORM_CLASSES.items():
            obj = tcls()
            self.assertIsInstance(obj.inverse(), tcls)

    def test_normalized(self):
        for ttype, tcls in TRANSFORM_CLASSES.items():
            obj = tcls()
            self.assertIsInstance(obj.normalized(), tcls)

    def test_scale_translation(self):
        for ttype, tcls in TRANSFORM_CLASSES.items():
            obj = tcls()
            self.assertIsInstance(obj.scaleTranslation(2.0), tcls)

    def test_apply(self):
        for ttype1, tcls1 in TRANSFORM_CLASSES.items():
            obj1 = tcls1()
            for ttype2, tcls2 in TRANSFORM_CLASSES.items():
                obj2 = tcls2()
                self.assertIsInstance(obj1.applyPost(obj2), tcls1)
                self.assertIsInstance(obj1.applyPre(obj2), tcls1)
                self.assertIsInstance(obj1.copy().applyPost_(obj2), tcls1)
                self.assertIsInstance(obj1.copy().applyPre_(obj2), tcls1)

    def test_norm(self):
        for ttype1, tcls1 in TRANSFORM_CLASSES.items():
            obj = tcls1()
            self.assertEqual(obj.rotationNorm(), 0.0)
            self.assertEqual(obj.translationNorm(), 0.0)

    def test_operators(self):
        for ttype1, tcls1 in TRANSFORM_CLASSES.items():
            obj1 = tcls1()
            for ttype2, tcls2 in TRANSFORM_CLASSES.items():
                obj2 = tcls2()
                self.assertIsInstance(obj1 * obj2, tcls1)
                self.assertIsInstance(obj1 / obj2, tcls1)
                obj1 *= obj2
                self.assertIsInstance(obj1, tcls1)
                obj1 /= obj2
                self.assertIsInstance(obj1, tcls1)

    def test_unsafe(self):
        # create transforms
        safe_list = []
        unsafe_list = []
        for ttype, tarr in TRANSFORM_ARRAYS.items():
            safe_list.append(m3d.TransformInterface.Factory(ttype, tarr, False))
            unsafe_list.append(m3d.TransformInterface.Factory(ttype, tarr, True))

        # check safe
        for transform in safe_list:
            self.assertFalse(transform.isUnsafe())
            self.assertFalse(transform.inverse().isUnsafe())
            self.assertFalse(transform.scaleTranslation(1.0).isUnsafe())
            for ttype in TRANSFORM_CLASSES.keys():
                self.assertFalse(transform.asType(ttype).isUnsafe())
            self.assertFalse(transform.normalized().isUnsafe())

        # check unsafe
        for transform in unsafe_list:
            self.assertTrue(transform.isUnsafe())
            self.assertTrue(transform.inverse().isUnsafe())
            self.assertTrue(transform.scaleTranslation(1.0).isUnsafe())
            for ttype in TRANSFORM_CLASSES.keys():
                self.assertTrue(transform.asType(ttype).isUnsafe())
            self.assertFalse(transform.normalized().isUnsafe())

        # check combinations
        for safe1, safe2 in zip(safe_list, safe_list):
            self.assertFalse(safe1.applyPre(safe2).isUnsafe())
            self.assertFalse(safe1.applyPost(safe2).isUnsafe())
            self.assertFalse(safe1.copy().applyPre_(safe2).isUnsafe())
            self.assertFalse(safe1.copy().applyPost_(safe2).isUnsafe())

        for safe, unsafe in zip(safe_list, unsafe_list):
            self.assertTrue(safe.applyPre(unsafe).isUnsafe())
            self.assertTrue(unsafe.applyPre(safe).isUnsafe())
            self.assertTrue(safe.applyPost(unsafe).isUnsafe())
            self.assertTrue(unsafe.applyPost(safe).isUnsafe())
            self.assertTrue(safe.copy().applyPre_(unsafe).isUnsafe())
            self.assertTrue(unsafe.copy().applyPre_(safe).isUnsafe())
            self.assertTrue(safe.copy().applyPost_(unsafe).isUnsafe())
            self.assertTrue(unsafe.copy().applyPost_(safe).isUnsafe())

        for unsafe1, unsafe2 in zip(unsafe_list, unsafe_list):
            self.assertTrue(unsafe1.applyPre(unsafe2).isUnsafe())
            self.assertTrue(unsafe1.applyPost(unsafe2).isUnsafe())
            self.assertTrue(unsafe1.copy().applyPre_(unsafe2).isUnsafe())
            self.assertTrue(unsafe1.copy().applyPost_(unsafe2).isUnsafe())

    def test_description(self):
        a = m3d.AxisAngleTransform()
        d = m3d.DualQuaternionTransform()
        e = m3d.EulerTransform()
        m = m3d.MatrixTransform()
        q = m3d.QuaternionTransform()

        self.assertTrue(str(a).startswith("AxisAngleTransform"))
        self.assertTrue(str(d).startswith("DualQuaternionTransform"))
        self.assertTrue(str(e).startswith("EulerTransform"))
        self.assertTrue(str(m).startswith("MatrixTransform"))
        self.assertTrue(str(q).startswith("QuaternionTransform"))


if __name__ == '__main__':
    unittest.main()
