import unittest

import numpy as np

import motion3d as m3d

from data import TRANSFORM_CLASSES, TRANSFORM_ARRAYS, TRANSFORM_ARRAYS1


def trafo_func(t):
    """Dummy function for applying directly on transforms."""
    translation = t.asType(m3d.TransformType.kEuler).getTranslation()
    translation[0] += 10
    return m3d.EulerTransform(translation, 0.0, 0.0, 0.0, m3d.EulerAxes.kSXYX)


def index_func(index, t):
    """Dummy function for applying on indices and transforms."""
    translation = t.asType(m3d.TransformType.kEuler).getTranslation()
    translation[0] += 10.0 + index * 10
    return m3d.EulerTransform(translation, 0.0, 0.0, 0.0, m3d.EulerAxes.kSXYX)


def stamp_func(stamp, t):
    """Dummy function for applying on stamps and transforms."""
    translation = t.asType(m3d.TransformType.kEuler).getTranslation()
    translation[0] += 10.0 + stamp.toSec()
    return m3d.EulerTransform(translation, 0.0, 0.0, 0.0, m3d.EulerAxes.kSXYX)


class TestTransformContainer(unittest.TestCase):
    def test_constructor(self):
        # empty
        data_empty_m = m3d.TransformContainer(False, False)
        self.assertFalse(data_empty_m.hasStamps())
        self.assertFalse(data_empty_m.hasPoses())
        self.assertEqual(data_empty_m.size(), 0)

        data_empty_p = m3d.TransformContainer(False, True)
        self.assertFalse(data_empty_p.hasStamps())
        self.assertTrue(data_empty_p.hasPoses())
        self.assertEqual(data_empty_p.size(), 0)

        data_empty_ms = m3d.TransformContainer(True, False)
        self.assertTrue(data_empty_ms.hasStamps())
        self.assertFalse(data_empty_ms.hasPoses())
        self.assertEqual(data_empty_ms.size(), 0)

        data_empty_ps = m3d.TransformContainer(True, True)
        self.assertTrue(data_empty_ps.hasStamps())
        self.assertTrue(data_empty_ps.hasPoses())
        self.assertEqual(data_empty_ps.size(), 0)

        # unstamped
        data_list_m = m3d.TransformContainer([m3d.DualQuaternionTransform(), m3d.MatrixTransform()], False)
        self.assertFalse(data_list_m.hasStamps())
        self.assertFalse(data_list_m.hasPoses())
        self.assertEqual(data_list_m.size(), 2)
        self.assertTrue(data_list_m.at(0).isType(m3d.TransformType.kDualQuaternion))
        self.assertTrue(data_list_m.at(1).isType(m3d.TransformType.kMatrix))

        data_list_p = m3d.TransformContainer([m3d.DualQuaternionTransform(), m3d.MatrixTransform()], True)
        self.assertFalse(data_list_p.hasStamps())
        self.assertTrue(data_list_p.hasPoses())
        self.assertEqual(data_list_p.size(), 2)
        self.assertTrue(data_list_p.at(0).isType(m3d.TransformType.kDualQuaternion))
        self.assertTrue(data_list_p.at(1).isType(m3d.TransformType.kMatrix))

        # stamped
        data_stamped_m = m3d.TransformContainer({m3d.Time(0.0): m3d.DualQuaternionTransform(),
                                                 m3d.Time(1.0): m3d.MatrixTransform()}, False)
        self.assertTrue(data_stamped_m.hasStamps())
        self.assertFalse(data_stamped_m.hasPoses())
        self.assertEqual(data_stamped_m.size(), 2)
        self.assertTrue(data_stamped_m.at_stamp(m3d.Time(0.0)).isType(m3d.TransformType.kDualQuaternion))
        self.assertTrue(data_stamped_m.at_stamp(m3d.Time(1.0)).isType(m3d.TransformType.kMatrix))

        data_stamped_p = m3d.TransformContainer({m3d.Time(0.0): m3d.DualQuaternionTransform(),
                                                 m3d.Time(1.0): m3d.MatrixTransform()}, True)
        self.assertTrue(data_stamped_p.hasStamps())
        self.assertTrue(data_stamped_p.hasPoses())
        self.assertEqual(data_stamped_p.size(), 2)
        self.assertTrue(data_stamped_p.at_stamp(m3d.Time(0.0)).isType(m3d.TransformType.kDualQuaternion))
        self.assertTrue(data_stamped_p.at_stamp(m3d.Time(1.0)).isType(m3d.TransformType.kMatrix))

        # separate stamps and transforms
        data_stamped_m_sep1 = m3d.TransformContainer([m3d.Time(0.0), m3d.Time(1.0)],
                                                     [m3d.DualQuaternionTransform(), m3d.MatrixTransform()],
                                                     False, False)
        self.assertTrue(data_stamped_m_sep1.hasStamps())
        self.assertFalse(data_stamped_m_sep1.hasPoses())
        self.assertEqual(data_stamped_m_sep1.size(), 2)
        self.assertTrue(data_stamped_m_sep1.at_stamp(m3d.Time(0.0)).isType(m3d.TransformType.kDualQuaternion))
        self.assertTrue(data_stamped_m_sep1.at_stamp(m3d.Time(1.0)).isType(m3d.TransformType.kMatrix))

        data_stamped_m_sep2 = m3d.TransformContainer([m3d.Time(0.0), m3d.Time(1.0)],
                                                     [m3d.DualQuaternionTransform(), m3d.MatrixTransform()],
                                                     False, False)
        self.assertTrue(data_stamped_m_sep2.hasStamps())
        self.assertFalse(data_stamped_m_sep2.hasPoses())
        self.assertEqual(data_stamped_m_sep2.size(), 2)
        self.assertTrue(data_stamped_m_sep2.at_stamp(m3d.Time(0.0)).isType(m3d.TransformType.kDualQuaternion))
        self.assertTrue(data_stamped_m_sep2.at_stamp(m3d.Time(1.0)).isType(m3d.TransformType.kMatrix))

        data_stamped_p_sep1 = m3d.TransformContainer([m3d.Time(0.0), m3d.Time(1.0)],
                                                     [m3d.DualQuaternionTransform(), m3d.MatrixTransform()],
                                                     True, False)
        self.assertTrue(data_stamped_p_sep1.hasStamps())
        self.assertTrue(data_stamped_p_sep1.hasPoses())
        self.assertEqual(data_stamped_p_sep1.size(), 2)
        self.assertTrue(data_stamped_p_sep1.at_stamp(m3d.Time(0.0)).isType(m3d.TransformType.kDualQuaternion))
        self.assertTrue(data_stamped_p_sep1.at_stamp(m3d.Time(1.0)).isType(m3d.TransformType.kMatrix))

        data_stamped_p_sep2 = m3d.TransformContainer([m3d.Time(0.0), m3d.Time(1.0)],
                                                     [m3d.DualQuaternionTransform(), m3d.MatrixTransform()],
                                                     True, False)
        self.assertTrue(data_stamped_p_sep2.hasStamps())
        self.assertTrue(data_stamped_p_sep2.hasPoses())
        self.assertEqual(data_stamped_p_sep2.size(), 2)
        self.assertTrue(data_stamped_p_sep2.at_stamp(m3d.Time(0.0)).isType(m3d.TransformType.kDualQuaternion))
        self.assertTrue(data_stamped_p_sep2.at_stamp(m3d.Time(1.0)).isType(m3d.TransformType.kMatrix))

        # copy
        data_empty_m_copy = data_empty_m.copy()
        self.assertFalse(data_empty_m_copy.hasStamps())
        self.assertFalse(data_empty_m_copy.hasPoses())
        data_empty_p_copy = data_empty_p.copy()
        self.assertFalse(data_empty_p_copy.hasStamps())
        self.assertTrue(data_empty_p_copy.hasPoses())
        data_empty_ms_copy = data_empty_ms.copy()
        self.assertTrue(data_empty_ms_copy.hasStamps())
        self.assertFalse(data_empty_ms_copy.hasPoses())
        data_empty_ps_copy = data_empty_ps.copy()
        self.assertTrue(data_empty_ps_copy.hasStamps())
        self.assertTrue(data_empty_ps_copy.hasPoses())

        data_list_m_copy = data_list_m.copy()
        self.assertFalse(data_list_m.at(0) == data_list_m_copy.at(0))
        self.assertFalse(data_list_m.at(1) == data_list_m_copy.at(1))
        data_stamped_m_copy = data_stamped_m.copy()
        self.assertFalse(data_stamped_m.at(0) == data_stamped_m_copy.at(0))
        self.assertFalse(data_stamped_m.at(1) == data_stamped_m_copy.at(1))

        # exceptions
        self.assertRaises(ValueError, lambda: m3d.TransformContainer(
            [m3d.Time(0.0)], [m3d.DualQuaternionTransform(), m3d.MatrixTransform()], False))

    def test_insert(self):
        # unstamped
        data = m3d.TransformContainer(False, False)
        self.assertEqual(data.size(), 0)
        self.assertEqual(len(data), 0)
        data.append(m3d.EulerTransform())
        self.assertEqual(data.size(), 1)
        self.assertEqual(len(data), 1)

        data.insert(0, m3d.MatrixTransform())
        self.assertEqual(data.size(), 2)
        self.assertTrue(data.at(0).isType(m3d.TransformType.kMatrix))
        self.assertTrue(data.at(1).isType(m3d.TransformType.kEuler))

        data_copy = data.copy()
        data_copy.insert(2, m3d.QuaternionTransform())
        self.assertEqual(data_copy.size(), 3)
        self.assertTrue(data_copy.at(2).isType(m3d.TransformType.kQuaternion))

        # stamped
        data_stamped = m3d.TransformContainer(True, False)
        self.assertTrue(data_stamped.insert(m3d.Time(0), m3d.EulerTransform(), False))
        self.assertTrue(data_stamped.insert(m3d.Time(2), m3d.EulerTransform(), False))
        self.assertEqual(data_stamped.size(), 2)

        self.assertFalse(data_stamped.insert(m3d.Time(2), m3d.EulerTransform(), False))
        self.assertTrue(data_stamped.insert(m3d.Time(2), m3d.EulerTransform(), True))
        self.assertEqual(data_stamped.size(), 2)

        data_stamped.append(m3d.Time(3), m3d.EulerTransform())
        data_stamped.append(m3d.Time(1), m3d.EulerTransform())
        data_stamped.append(m3d.Time(0), m3d.MatrixTransform())
        self.assertTrue(data_stamped.at(0).isType(m3d.TransformType.kEuler))

        # exceptions
        self.assertRaises(m3d.TransformContainerException, lambda: data.append(m3d.Time(0), m3d.EulerTransform()))
        self.assertRaises(m3d.TransformContainerException, lambda: data.insert(m3d.Time(0), m3d.EulerTransform()))
        self.assertRaises(IndexError, lambda: data.insert(3, m3d.EulerTransform()))
        self.assertRaises(m3d.TransformContainerException, lambda: data_stamped.append(m3d.EulerTransform()))
        self.assertRaises(m3d.TransformContainerException, lambda: data_stamped.insert(0, m3d.EulerTransform()))
        self.assertRaises(m3d.TransformContainerException, lambda: data_stamped.insert(3, m3d.EulerTransform()))

    def test_iterators(self):
        data = m3d.TransformContainer([
            m3d.DualQuaternionTransform(TRANSFORM_ARRAYS1[m3d.DualQuaternionTransform]),
            m3d.MatrixTransform(TRANSFORM_ARRAYS1[m3d.MatrixTransform])
        ], False)
        data_copy = data.copy()
        data_stamped = m3d.TransformContainer(True, False)
        data_stamped.insert(m3d.Time(0), m3d.DualQuaternionTransform(TRANSFORM_ARRAYS1[m3d.DualQuaternionTransform]))
        data_stamped.insert(m3d.Time(1), m3d.MatrixTransform(TRANSFORM_ARRAYS1[m3d.MatrixTransform]))
        data_stamped_long = m3d.TransformContainer(True, False)
        data_stamped_long.insert(m3d.Time(10), m3d.MatrixTransform())
        data_stamped_long.insert(m3d.Time(20), m3d.MatrixTransform())
        data_stamped_long.insert(m3d.Time(30), m3d.MatrixTransform())
        data_stamped_long.insert(m3d.Time(40), m3d.MatrixTransform())

        # transforms
        counter = 0
        for i, (t, ts) in enumerate(zip(data, data_stamped)):
            # inplace
            t.scaleTranslation_(2.0)
            self.assertEqual(data.at(i).translationNorm(), 2.0 * data_copy.at(i).translationNorm())
            ts.scaleTranslation_(2.0)
            self.assertEqual(data_stamped.at(i).translationNorm(), 2.0 * data_copy.at(i).translationNorm())

            # type
            counter += 1
            if i == 0:
                self.assertTrue(t.isType(m3d.TransformType.kDualQuaternion))
                self.assertTrue(ts.isType(m3d.TransformType.kDualQuaternion))
            elif i == 1:
                self.assertTrue(t.isType(m3d.TransformType.kMatrix))
                self.assertTrue(ts.isType(m3d.TransformType.kMatrix))
            else:
                self.fail()
        self.assertEqual(counter, 2)

        counter = 0
        for i, (t, ts) in enumerate(zip(data.transforms(), data_stamped.transforms())):
            # inplace
            t.scaleTranslation_(2.0)
            self.assertEqual(data.at(i).translationNorm(), 4.0 * data_copy.at(i).translationNorm())
            ts.scaleTranslation_(2.0)
            self.assertEqual(data_stamped.at(i).translationNorm(), 4.0 * data_copy.at(i).translationNorm())

            # type
            counter += 1
            if i == 0:
                self.assertTrue(t.isType(m3d.TransformType.kDualQuaternion))
                self.assertTrue(ts.isType(m3d.TransformType.kDualQuaternion))
            elif i == 1:
                self.assertTrue(t.isType(m3d.TransformType.kMatrix))
                self.assertTrue(ts.isType(m3d.TransformType.kMatrix))
            else:
                self.fail()
        self.assertEqual(counter, 2)

        # stamps
        counter = 0
        for i, stamp in enumerate(data_stamped.stamps()):
            counter += 1
            self.assertEqual(stamp.toNSec(), i)
        self.assertEqual(counter, 2)

        # items
        counter = 0
        for i, (stamp, t) in enumerate(data_stamped.items()):
            # inplace
            t.scaleTranslation_(2.0)
            self.assertEqual(data_stamped.at(i).translationNorm(), 8.0 * data_copy.at(i).translationNorm())

            # type
            counter += 1
            if i == 0:
                self.assertTrue(t.isType(m3d.TransformType.kDualQuaternion))
            elif i == 1:
                self.assertTrue(t.isType(m3d.TransformType.kMatrix))
            else:
                self.fail()
        self.assertEqual(counter, 2)

        # items with start
        counter = 0
        for (stamp, _), time_ns in zip(data_stamped_long.items_ge(m3d.Time(20)), [20, 30, 40]):
            counter += 1
            self.assertEqual(stamp.toNSec(), time_ns)
        self.assertEqual(counter, 3)

        counter = 0
        for (stamp, _), time_ns in zip(data_stamped_long.items_gt(m3d.Time(20)), [30, 40]):
            counter += 1
            self.assertEqual(stamp.toNSec(), time_ns)
        self.assertEqual(counter, 2)

        counter = 0
        for (stamp, _), time_ns in zip(data_stamped_long.items_le(m3d.Time(20)), [20, 30, 40]):
            counter += 1
            self.assertEqual(stamp.toNSec(), time_ns)
        self.assertEqual(counter, 3)

        counter = 0
        for (stamp, _), time_ns in zip(data_stamped_long.items_lt(m3d.Time(20)), [10, 20, 30, 40]):
            counter += 1
            self.assertEqual(stamp.toNSec(), time_ns)
        self.assertEqual(counter, 4)

        counter = 0
        for (stamp, _), time_ns in zip(data_stamped_long.items_closest(m3d.Time(24)), [20, 30, 40]):
            counter += 1
            self.assertEqual(stamp.toNSec(), time_ns)
        self.assertEqual(counter, 3)

        counter = 0
        for (stamp, _), time_ns in zip(data_stamped_long.items_closest(m3d.Time(16)), [20, 30, 40]):
            counter += 1
            self.assertEqual(stamp.toNSec(), time_ns)
        self.assertEqual(counter, 3)

        # exceptions
        self.assertRaises(m3d.TransformContainerException, lambda: data.stamps())
        self.assertRaises(m3d.TransformContainerException, lambda: data.items())
        self.assertRaises(m3d.TransformContainerException, lambda: data.items_ge(m3d.Time(0)))
        self.assertRaises(m3d.TransformContainerException, lambda: data.items_gt(m3d.Time(0)))
        self.assertRaises(m3d.TransformContainerException, lambda: data.items_le(m3d.Time(0)))
        self.assertRaises(m3d.TransformContainerException, lambda: data.items_lt(m3d.Time(0)))

    def test_container_functions(self):
        # create containers
        data = m3d.TransformContainer(
            [m3d.AxisAngleTransform(TRANSFORM_ARRAYS1[m3d.AxisAngleTransform]),
             m3d.DualQuaternionTransform(TRANSFORM_ARRAYS1[m3d.DualQuaternionTransform]),
             m3d.EulerTransform(TRANSFORM_ARRAYS1[m3d.EulerTransform]),
             m3d.MatrixTransform(TRANSFORM_ARRAYS1[m3d.MatrixTransform]),
             m3d.QuaternionTransform(TRANSFORM_ARRAYS1[m3d.QuaternionTransform])],
            False)
        data_stamped = m3d.TransformContainer(
            {m3d.Time(0.0): m3d.AxisAngleTransform(TRANSFORM_ARRAYS1[m3d.AxisAngleTransform]),
             m3d.Time(1.0): m3d.DualQuaternionTransform(TRANSFORM_ARRAYS1[m3d.DualQuaternionTransform]),
             m3d.Time(2.0): m3d.EulerTransform(TRANSFORM_ARRAYS1[m3d.EulerTransform])},
            False)

        # length
        self.assertEqual(len(data), 5)
        self.assertEqual(data.size(), 5)
        self.assertEqual(len(data_stamped), 3)
        self.assertEqual(data_stamped.size(), 3)

        # stamp check
        self.assertTrue(data_stamped.hasStamp(m3d.Time(0.0)))
        self.assertTrue(data_stamped.hasStamp(m3d.Time(1.0)))
        self.assertFalse(data_stamped.hasStamp(m3d.Time(0.5)))

        self.assertTrue(m3d.Time(0.0) in data_stamped)
        self.assertFalse(m3d.Time(0.5) in data_stamped)

        self.assertRaises(m3d.TransformContainerException, lambda: data.hasStamp(m3d.Time(0.0)))
        self.assertRaises(m3d.TransformContainerException, lambda: m3d.Time(0.0) in data)
        self.assertRaises(TypeError, lambda: 1.0 in data)
        self.assertRaises(TypeError, lambda: 1.0 in data_stamped)

        # index and stamp access
        self.assertTrue(data[0].isType(m3d.TransformType.kAxisAngle))
        self.assertTrue(data[1].isType(m3d.TransformType.kDualQuaternion))
        self.assertRaises(IndexError, lambda: data[5])

        self.assertTrue(data_stamped[0].isType(m3d.TransformType.kAxisAngle))
        self.assertTrue(data_stamped[1].isType(m3d.TransformType.kDualQuaternion))
        self.assertRaises(IndexError, lambda: data_stamped[3])

        self.assertTrue(data_stamped[m3d.Time(0.0)].isType(m3d.TransformType.kAxisAngle))
        self.assertTrue(data_stamped[m3d.Time(1.0)].isType(m3d.TransformType.kDualQuaternion))
        self.assertRaises(m3d.TransformContainerException, lambda: data[m3d.Time(0.0)])
        self.assertRaises(IndexError, lambda: data_stamped[m3d.Time(0.5)])

        # at
        self.assertTrue(data.at(0).isType(m3d.TransformType.kAxisAngle))
        self.assertTrue(data_stamped.at(0).isType(m3d.TransformType.kAxisAngle))

        # stamp at
        self.assertIsInstance(data_stamped.stamp_at(0), m3d.Time)

        # item_at
        data_item = data_stamped.item_at(0)
        self.assertIsInstance(data_item[0], m3d.Time)
        self.assertTrue(data_item[1].isType(m3d.TransformType.kAxisAngle))

        # at_stamp
        self.assertTrue(data_stamped.at_stamp(m3d.Time(0.0)).isType(m3d.TransformType.kAxisAngle))

        # at exceptions
        self.assertRaises(IndexError, lambda: data.at(5))
        self.assertRaises(m3d.TransformContainerException, lambda: data.stamp_at(0))
        self.assertRaises(m3d.TransformContainerException, lambda: data.item_at(0))
        self.assertRaises(m3d.TransformContainerException, lambda: data.stamp_at(5))
        self.assertRaises(m3d.TransformContainerException, lambda: data.item_at(5))
        self.assertRaises(m3d.TransformContainerException, lambda: data.at_stamp(m3d.Time(0.0)))
        self.assertRaises(IndexError, lambda: data_stamped.at(3))
        self.assertRaises(IndexError, lambda: data_stamped.stamp_at(3))
        self.assertRaises(IndexError, lambda: data_stamped.item_at(3))
        self.assertRaises(IndexError, lambda: data_stamped.at_stamp(m3d.Time(0.5)))

        # inplace access
        data.at(0).scaleTranslation_(2.0)
        self.assertAlmostEqual(data.at(0).translationNorm(), 2.0 * data.at(1).translationNorm())
        data[0].scaleTranslation_(2.0)
        self.assertAlmostEqual(data.at(0).translationNorm(), 4.0 * data.at(1).translationNorm())

        data_stamped.at(0).scaleTranslation_(2.0)
        self.assertAlmostEqual(data_stamped.at(0).translationNorm(), 2.0 * data_stamped.at(1).translationNorm())
        data_stamped.item_at(0)[1].scaleTranslation_(2.0)
        self.assertAlmostEqual(data_stamped.at(0).translationNorm(), 4.0 * data_stamped.at(1).translationNorm())
        data_stamped.at_stamp(m3d.Time(0.0)).scaleTranslation_(2.0)
        self.assertAlmostEqual(data_stamped.at(0).translationNorm(), 8.0 * data_stamped.at(1).translationNorm())
        data_stamped[0].scaleTranslation_(2.0)
        self.assertAlmostEqual(data_stamped.at(0).translationNorm(), 16.0 * data_stamped.at(1).translationNorm())
        data_stamped[m3d.Time(0.0)].scaleTranslation_(2.0)
        self.assertAlmostEqual(data_stamped.at(0).translationNorm(), 32.0 * data_stamped.at(1).translationNorm())

        # slicing
        data_slice1 = data[1:3:2]
        self.assertTrue(data_slice1.hasMotions())
        self.assertFalse(data_slice1.hasStamps())
        self.assertEqual(len(data_slice1), 1)
        self.assertTrue(data_slice1[0].isType(m3d.TransformType.kDualQuaternion))

        data_slice2 = data[1:4:2]
        self.assertTrue(data_slice2.hasMotions())
        self.assertFalse(data_slice2.hasStamps())
        self.assertEqual(len(data_slice2), 2)
        self.assertTrue(data_slice2[0].isType(m3d.TransformType.kDualQuaternion))
        self.assertTrue(data_slice2[1].isType(m3d.TransformType.kMatrix))

        data_slice3 = data_stamped[1:3]
        self.assertTrue(data_slice3.hasMotions())
        self.assertTrue(data_slice3.hasStamps())
        self.assertEqual(len(data_slice3), 2)
        self.assertTrue(data_slice3[0].isType(m3d.TransformType.kDualQuaternion))

        # setitem
        data[0] = m3d.EulerTransform()
        self.assertTrue(data[0].isType(m3d.TransformType.kEuler))

        data_stamped[0] = m3d.EulerTransform()
        self.assertTrue(data_stamped[0].isType(m3d.TransformType.kEuler))

        data_stamped[m3d.Time(0.0)] = m3d.AxisAngleTransform()
        self.assertTrue(data_stamped[m3d.Time(0.0)].isType(m3d.TransformType.kAxisAngle))

        data_stamped[m3d.Time(0.5)] = m3d.AxisAngleTransform()
        self.assertTrue(data_stamped[m3d.Time(0.5)].isType(m3d.TransformType.kAxisAngle))

        # setitem exceptions
        def setitem_error():
            data[6] = m3d.AxisAngleTransform()
        self.assertRaises(IndexError, setitem_error)

        def setitem_error():
            data_stamped[6] = m3d.AxisAngleTransform()
        self.assertRaises(IndexError, setitem_error)

        def setitem_error():
            data[m3d.Time(0.0)] = m3d.AxisAngleTransform()
        self.assertRaises(m3d.TransformContainerException, setitem_error)

        def setitem_error():
            data[:2] = [m3d.AxisAngleTransform(), m3d.AxisAngleTransform()]
        self.assertRaises(TypeError, setitem_error)

        def setitem_error():
            data_stamped[:2] = [m3d.AxisAngleTransform(), m3d.AxisAngleTransform()]
        self.assertRaises(TypeError, setitem_error)

        # erase
        self.assertRaises(m3d.TransformContainerException, lambda: data.erase(m3d.Time(2.0)))
        self.assertRaises(IndexError, lambda: data.erase(5))
        self.assertRaises(IndexError, lambda: data_stamped.erase(4))

        data.erase(0)
        self.assertEqual(data.size(), 4)

        data_stamped.erase(0)
        self.assertEqual(data_stamped.size(), 3)
        data_stamped.erase(m3d.Time(1.0))
        self.assertEqual(data_stamped.size(), 2)
        data_stamped.erase(m3d.Time(5.0))
        self.assertEqual(data_stamped.size(), 2)

        # clear and empty
        self.assertFalse(data.empty())
        data.clear()
        self.assertEqual(data.size(), 0)
        self.assertTrue(data.empty())

        self.assertFalse(data_stamped.empty())
        data_stamped.clear()
        self.assertEqual(data_stamped.size(), 0)
        self.assertTrue(data_stamped.empty())

    def test_find(self):
        # create containers
        data = m3d.TransformContainer(False, False)
        data_stamped = m3d.TransformContainer(
            {m3d.Time(1.0): m3d.AxisAngleTransform(), m3d.Time(2.0): m3d.DualQuaternionTransform(),
             m3d.Time(3.0): m3d.EulerTransform()}, False)

        # equal
        stamp, transform = data_stamped.find_eq(m3d.Time(1.0))
        self.assertEqual(stamp, m3d.Time(1.0))
        self.assertTrue(transform.isType(m3d.TransformType.kAxisAngle))

        stamp, transform = data_stamped.find_eq(m3d.Time(1.5))
        self.assertTrue(stamp is None)
        self.assertTrue(transform is None)

        # greater equal
        stamp, transform = data_stamped.find_ge(m3d.Time(1.5))
        self.assertEqual(stamp, m3d.Time(2.0))
        self.assertTrue(transform.isType(m3d.TransformType.kDualQuaternion))

        stamp, transform = data_stamped.find_ge(m3d.Time(3.0))
        self.assertEqual(stamp, m3d.Time(3.0))
        self.assertTrue(transform.isType(m3d.TransformType.kEuler))

        stamp, transform = data_stamped.find_ge(m3d.Time(3.5))
        self.assertTrue(stamp is None)
        self.assertTrue(transform is None)

        # greater than
        stamp, transform = data_stamped.find_gt(m3d.Time(1.5))
        self.assertEqual(stamp, m3d.Time(2.0))
        self.assertTrue(transform.isType(m3d.TransformType.kDualQuaternion))

        stamp, transform = data_stamped.find_gt(m3d.Time(3.0))
        self.assertTrue(stamp is None)
        self.assertTrue(transform is None)

        stamp, transform = data_stamped.find_gt(m3d.Time(3.5))
        self.assertTrue(stamp is None)
        self.assertTrue(transform is None)

        # lower equal
        stamp, transform = data_stamped.find_le(m3d.Time(1.5))
        self.assertEqual(stamp, m3d.Time(1.0))
        self.assertTrue(transform.isType(m3d.TransformType.kAxisAngle))

        stamp, transform = data_stamped.find_le(m3d.Time(1.0))
        self.assertEqual(stamp, m3d.Time(1.0))
        self.assertTrue(transform.isType(m3d.TransformType.kAxisAngle))

        stamp, transform = data_stamped.find_le(m3d.Time(0.5))
        self.assertTrue(stamp is None)
        self.assertTrue(transform is None)

        # lower than
        stamp, transform = data_stamped.find_lt(m3d.Time(1.5))
        self.assertEqual(stamp, m3d.Time(1.0))
        self.assertTrue(transform.isType(m3d.TransformType.kAxisAngle))

        stamp, transform = data_stamped.find_lt(m3d.Time(1.0))
        self.assertTrue(stamp is None)
        self.assertTrue(transform is None)

        stamp, transform = data_stamped.find_lt(m3d.Time(0.5))
        self.assertTrue(stamp is None)
        self.assertTrue(transform is None)

        # closest
        stamp, transform = data_stamped.find_closest(m3d.Time(0.0))
        self.assertEqual(stamp, m3d.Time(1.0))

        stamp, transform = data_stamped.find_closest(m3d.Time(1.4))
        self.assertEqual(stamp, m3d.Time(1.0))

        stamp, transform = data_stamped.find_closest(m3d.Time(1.5))
        self.assertEqual(stamp, m3d.Time(1.0))

        stamp, transform = data_stamped.find_closest(m3d.Time(1.6))
        self.assertEqual(stamp, m3d.Time(2.0))

        stamp, transform = data_stamped.find_closest(m3d.Time(2.0))
        self.assertEqual(stamp, m3d.Time(2.0))

        stamp, transform = data_stamped.find_closest(m3d.Time(5.0))
        self.assertEqual(stamp, m3d.Time(3.0))

        # exceptions
        self.assertRaises(m3d.TransformContainerException, lambda: data.find_eq(m3d.Time(0.0)))
        self.assertRaises(m3d.TransformContainerException, lambda: data.find_ge(m3d.Time(0.0)))
        self.assertRaises(m3d.TransformContainerException, lambda: data.find_gt(m3d.Time(0.0)))
        self.assertRaises(m3d.TransformContainerException, lambda: data.find_le(m3d.Time(0.0)))
        self.assertRaises(m3d.TransformContainerException, lambda: data.find_lt(m3d.Time(0.0)))
        self.assertRaises(m3d.TransformContainerException, lambda: data.find_closest(m3d.Time(0.0)))

    def test_extend(self):
        # unstamped
        data1 = m3d.TransformContainer([m3d.AxisAngleTransform(), m3d.DualQuaternionTransform()], False)
        data2 = m3d.TransformContainer([m3d.EulerTransform(), m3d.MatrixTransform(), m3d.QuaternionTransform()], False)
        data1.extend(data2)
        self.assertEqual(len(data1), 5)
        self.assertTrue(data1[0].isType(m3d.TransformType.kAxisAngle))
        self.assertTrue(data1[1].isType(m3d.TransformType.kDualQuaternion))
        self.assertTrue(data1[2].isType(m3d.TransformType.kEuler))
        self.assertTrue(data1[3].isType(m3d.TransformType.kMatrix))
        self.assertTrue(data1[4].isType(m3d.TransformType.kQuaternion))

        # stamped
        data_stamped1 = m3d.TransformContainer(True, False)
        data_stamped1.insert(m3d.Time(1), m3d.EulerTransform())
        data_stamped2 = m3d.TransformContainer(True, False)
        data_stamped2.insert(m3d.Time(0), m3d.AxisAngleTransform())
        data_stamped1.extend(data_stamped2)
        self.assertEqual(len(data_stamped1), 2)
        self.assertTrue(data_stamped1[0].isType(m3d.TransformType.kAxisAngle))
        self.assertTrue(data_stamped1[1].isType(m3d.TransformType.kEuler))

        # stamped overwrite
        data_stamped1_over = m3d.TransformContainer(True, False)
        data_stamped1_over.insert(m3d.Time(1), m3d.QuaternionTransform())
        data_stamped1.extend(data_stamped1_over, False)
        self.assertEqual(len(data_stamped1), 2)
        self.assertTrue(data_stamped1[1].isType(m3d.TransformType.kEuler))

        # stamped non-overwrite
        data_stamped1.extend(data_stamped1_over, True)
        self.assertEqual(len(data_stamped1), 2)
        self.assertTrue(data_stamped1[1].isType(m3d.TransformType.kQuaternion))

        # exceptions
        data3_poses = m3d.TransformContainer([m3d.AxisAngleTransform(), m3d.DualQuaternionTransform()], True)
        data3_poses.extend(data3_poses)

        # mixed motions and poses
        self.assertRaises(m3d.TransformContainerException, lambda: data1.extend(data3_poses))
        self.assertRaises(m3d.TransformContainerException, lambda: data3_poses.extend(data1))

        # mixed stamped and unstamped
        self.assertRaises(m3d.TransformContainerException, lambda: data1.extend(data_stamped1))
        self.assertRaises(m3d.TransformContainerException, lambda: data_stamped1.extend(data1))

    def test_transform_ptr_copy(self):
        # create data
        transform1 = m3d.MatrixTransform(TRANSFORM_ARRAYS1[m3d.MatrixTransform])

        transforms1 = [m3d.MatrixTransform(TRANSFORM_ARRAYS1[m3d.MatrixTransform])]

        stamps1 = [m3d.Time(2.0)]
        transforms_stamped1 = {m3d.Time(0.0): m3d.MatrixTransform(TRANSFORM_ARRAYS1[m3d.MatrixTransform])}

        translation_norm = transform1.translationNorm()

        # containers
        data1 = m3d.TransformContainer(transforms1, False)
        data1.scaleTranslation_(2.0)
        self.assertEqual(transforms1[0].translationNorm(), translation_norm)

        data_stamped1 = m3d.TransformContainer(transforms_stamped1, False)
        data_stamped1.scaleTranslation_(2.0)
        self.assertEqual(transforms_stamped1[m3d.Time(0.0)].translationNorm(), translation_norm)

        data_stamped2 = m3d.TransformContainer(stamps1, transforms1, False)
        data_stamped2.scaleTranslation_(2.0)
        self.assertEqual(transforms1[0].translationNorm(), translation_norm)

        # copy constructor
        data_copy1 = m3d.TransformContainer(data1)
        data_copy1.scaleTranslation_(2.0)
        self.assertEqual(data1.at(0).translationNorm(), 2.0 * translation_norm)

        data_stamped_copy1 = m3d.TransformContainer(data_stamped1)
        data_stamped_copy1.scaleTranslation_(2.0)
        self.assertEqual(data_stamped1.at(0).translationNorm(), 2.0 * translation_norm)

        # copy method
        data_copy2 = data1.copy()
        data_copy2.scaleTranslation_(2.0)
        self.assertEqual(data1.at(0).translationNorm(), 2.0 * translation_norm)

        data_stamped_copy2 = data_stamped1.copy()
        data_stamped_copy2.scaleTranslation_(2.0)
        self.assertEqual(data_stamped1.at(0).translationNorm(), 2.0 * translation_norm)

        # extend
        data1.extend(data_copy1)
        data1.scaleTranslation_(2.0)
        self.assertEqual(data_copy1.at(0).translationNorm(), 4.0 * translation_norm)

        data_stamped1.extend(data_stamped2, False)
        data_stamped1.scaleTranslation_(2.0)
        self.assertEqual(data_stamped2.at(0).translationNorm(), 2.0 * translation_norm)

        data_stamped1.extend(data_stamped2, True)
        data_stamped1.scaleTranslation_(2.0)
        self.assertEqual(data_stamped2.at(0).translationNorm(), 2.0 * translation_norm)

        # append
        data1.append(transform1)
        data1.scaleTranslation_(2.0)
        self.assertEqual(transform1.translationNorm(), translation_norm)

        data_stamped1.append(m3d.Time(3.0), transform1)
        data_stamped1.scaleTranslation_(2.0)
        self.assertEqual(transform1.translationNorm(), translation_norm)

        # insert
        data1.insert(0, transform1)
        data1.scaleTranslation_(2.0)
        self.assertEqual(transform1.translationNorm(), translation_norm)

        data1[0] = transform1
        data1.scaleTranslation_(2.0)
        self.assertEqual(transform1.translationNorm(), translation_norm)

        data_stamped1.insert(m3d.Time(4.0), transform1, False)
        data_stamped1.scaleTranslation_(2.0)
        self.assertEqual(transform1.translationNorm(), translation_norm)

        data_stamped1.insert(m3d.Time(4.0), transform1, True)
        data_stamped1.scaleTranslation_(2.0)
        self.assertEqual(transform1.translationNorm(), translation_norm)

        data_stamped1[m3d.Time(4.0)] = transform1
        data_stamped1.scaleTranslation_(2.0)
        self.assertEqual(transform1.translationNorm(), translation_norm)

    def test_inplace_return(self):
        # create container and transform
        data = m3d.TransformContainer([m3d.AxisAngleTransform(), m3d.AxisAngleTransform()], False)
        transform = m3d.AxisAngleTransform()

        # check return value
        self.assertEqual(data, data.asType_(m3d.TransformType.kEuler))
        self.assertEqual(data, data.asPoses_())
        self.assertEqual(data, data.asPoses_(transform))
        self.assertEqual(data, data.asMotions_())
        self.assertEqual(data, data.inverse_())
        self.assertEqual(data, data.normalized_())
        self.assertEqual(data, data.scaleTranslation_(1.0))
        self.assertEqual(data, data.applyPre_(transform))
        self.assertEqual(data, data.applyPost_(transform))
        self.assertEqual(data, data.apply_(transform, transform))
        self.assertEqual(data, data.applyFunc_(trafo_func))
        self.assertEqual(data, data.applyIndexFunc_(index_func))
        self.assertEqual(data, data.changeFrame_(transform))
        self.assertEqual(data, data.addStamps_([m3d.Time(0.0), m3d.Time(1.0)]))
        self.assertEqual(data, data.applyStampFunc_(stamp_func))
        self.assertEqual(data, data.removeStamps_())

    def test_as_type(self):
        data = m3d.TransformContainer([m3d.DualQuaternionTransform(), m3d.MatrixTransform()], False)
        for ttype, _ in TRANSFORM_CLASSES.items():
            # non-inplace
            tmp = data.asType(ttype)
            self.assertTrue(tmp.at(0).isType(ttype))
            self.assertTrue(tmp.at(1).isType(ttype))
            self.assertTrue(data.at(0).isType(m3d.TransformType.kDualQuaternion))
            self.assertTrue(data.at(1).isType(m3d.TransformType.kMatrix))

            # inplace
            tmp = data.copy().asType_(ttype)
            self.assertTrue(tmp.at(0).isType(ttype))
            self.assertTrue(tmp.at(1).isType(ttype))

    def test_motions_and_poses_data(self):
        # create container with motions
        data = m3d.TransformContainer([
            m3d.MatrixTransform([1.0, 0, 0], np.eye(3)),
            m3d.MatrixTransform([3.0, 0, 0], np.eye(3)),
            m3d.MatrixTransform([5.0, 0, 0], np.eye(3))
        ], False)

        # check motions
        self.assertTrue(data.hasMotions())
        self.assertFalse(data.hasPoses())
        self.assertEqual(data.size(), 3)
        self.assertEqual(data.at(0).getTranslation()[0], 1.0)
        self.assertEqual(data.at(1).getTranslation()[0], 3.0)
        self.assertEqual(data.at(2).getTranslation()[0], 5.0)

        # again as motions
        data.asMotions_()
        self.assertTrue(data.hasMotions())
        self.assertFalse(data.hasPoses())
        self.assertEqual(data.size(), 3)
        self.assertEqual(data.at(0).getTranslation()[0], 1.0)
        self.assertEqual(data.at(1).getTranslation()[0], 3.0)
        self.assertEqual(data.at(2).getTranslation()[0], 5.0)

        # poses
        data.asPoses_()
        self.assertFalse(data.hasMotions())
        self.assertTrue(data.hasPoses())
        self.assertEqual(data.size(), 4)
        self.assertEqual(data.at(0).getTranslation()[0], 0.0)
        self.assertEqual(data.at(1).getTranslation()[0], 1.0)
        self.assertEqual(data.at(2).getTranslation()[0], 4.0)
        self.assertEqual(data.at(3).getTranslation()[0], 9.0)

        # again as poses
        data.asPoses_()
        self.assertFalse(data.hasMotions())
        self.assertTrue(data.hasPoses())
        self.assertEqual(data.size(), 4)
        self.assertEqual(data.at(0).getTranslation()[0], 0.0)
        self.assertEqual(data.at(1).getTranslation()[0], 1.0)
        self.assertEqual(data.at(2).getTranslation()[0], 4.0)
        self.assertEqual(data.at(3).getTranslation()[0], 9.0)

        # back to motions
        data.asMotions_()
        self.assertTrue(data.hasMotions())
        self.assertFalse(data.hasPoses())
        self.assertEqual(data.size(), 3)
        self.assertEqual(data.at(0).getTranslation()[0], 1.0)
        self.assertEqual(data.at(1).getTranslation()[0], 3.0)
        self.assertEqual(data.at(2).getTranslation()[0], 5.0)

        # non-inplace
        self.assertTrue(data.asMotions().hasMotions())
        data_poses = data.asPoses()
        self.assertTrue(data_poses.hasPoses())
        self.assertTrue(data.hasMotions())

        self.assertTrue(data_poses.asPoses().hasPoses())
        self.assertTrue(data_poses.asMotions().hasMotions())
        self.assertTrue(data_poses.hasPoses())

    def test_motions_and_poses_types(self):
        # create container
        data = m3d.TransformContainer([
            m3d.MatrixTransform(),
            m3d.DualQuaternionTransform(),
            m3d.EulerTransform()], False)

        # poses
        data.asPoses_()
        self.assertTrue(data.at(0).isType(m3d.TransformType.kMatrix))
        self.assertTrue(data.at(1).isType(m3d.TransformType.kMatrix))
        self.assertTrue(data.at(2).isType(m3d.TransformType.kDualQuaternion))
        self.assertTrue(data.at(3).isType(m3d.TransformType.kEuler))

        # motions
        data.asMotions_()
        self.assertTrue(data.at(0).isType(m3d.TransformType.kMatrix))
        self.assertTrue(data.at(1).isType(m3d.TransformType.kDualQuaternion))
        self.assertTrue(data.at(2).isType(m3d.TransformType.kEuler))

    def test_initial_pose_change(self):
        # conainer with poses
        data_poses = m3d.TransformContainer([
            m3d.MatrixTransform([1.0, 0, 0], np.eye(3)),
            m3d.MatrixTransform([3.0, 0, 0], np.eye(3)),
            m3d.MatrixTransform([5.0, 0, 0], np.eye(3))
        ], True)

        # keep initial pose
        data_poses_keep = data_poses.asPoses()
        self.assertEqual(data_poses_keep.at(0).getTranslation()[0], 1.0)
        self.assertEqual(data_poses_keep.at(1).getTranslation()[0], 3.0)
        self.assertEqual(data_poses_keep.at(2).getTranslation()[0], 5.0)

        # change initial pose
        initial_pose = m3d.MatrixTransform([10.0, 10.0, 0], np.eye(3))

        data_poses_init = data_poses.asPoses(initial_pose)
        self.assertEqual(data_poses_init.at(0).getTranslation()[0], 10.0)
        self.assertEqual(data_poses_init.at(1).getTranslation()[0], 12.0)
        self.assertEqual(data_poses_init.at(2).getTranslation()[0], 14.0)
        self.assertEqual(data_poses_init.at(0).getTranslation()[1], 10.0)
        self.assertEqual(data_poses_init.at(1).getTranslation()[1], 10.0)
        self.assertEqual(data_poses_init.at(2).getTranslation()[1], 10.0)

        # poses from motions with initial pose
        data_motions = data_poses.asMotions()
        data_poses_from_motions = data_motions.asPoses(initial_pose)
        self.assertEqual(data_poses_from_motions.at(0).getTranslation()[0], 10.0)
        self.assertEqual(data_poses_from_motions.at(1).getTranslation()[0], 12.0)
        self.assertEqual(data_poses_from_motions.at(2).getTranslation()[0], 14.0)
        self.assertEqual(data_poses_from_motions.at(0).getTranslation()[1], 10.0)
        self.assertEqual(data_poses_from_motions.at(1).getTranslation()[1], 10.0)
        self.assertEqual(data_poses_from_motions.at(2).getTranslation()[1], 10.0)

        # inplace
        data_poses.asPoses_()
        self.assertEqual(data_poses.at(0).getTranslation()[0], 1.0)
        self.assertEqual(data_poses.at(1).getTranslation()[0], 3.0)
        self.assertEqual(data_poses.at(2).getTranslation()[0], 5.0)

        data_poses.asPoses_(initial_pose)
        self.assertEqual(data_poses.at(0).getTranslation()[0], 10.0)
        self.assertEqual(data_poses.at(1).getTranslation()[0], 12.0)
        self.assertEqual(data_poses.at(2).getTranslation()[0], 14.0)
        self.assertEqual(data_poses.at(0).getTranslation()[1], 10.0)
        self.assertEqual(data_poses.at(1).getTranslation()[1], 10.0)
        self.assertEqual(data_poses.at(2).getTranslation()[1], 10.0)

        data_poses.asMotions_()
        data_poses.asPoses_(initial_pose)
        self.assertEqual(data_poses.at(0).getTranslation()[0], 10.0)
        self.assertEqual(data_poses.at(1).getTranslation()[0], 12.0)
        self.assertEqual(data_poses.at(2).getTranslation()[0], 14.0)
        self.assertEqual(data_poses.at(0).getTranslation()[1], 10.0)
        self.assertEqual(data_poses.at(1).getTranslation()[1], 10.0)
        self.assertEqual(data_poses.at(2).getTranslation()[1], 10.0)

    def test_inverse(self):
        # container
        data = m3d.TransformContainer([
            m3d.MatrixTransform([1.0, 0, 0], np.eye(3)),
            m3d.MatrixTransform([3.0, 0, 0], np.eye(3))
        ], False)

        # non-inplace
        data_inv = data.inverse()
        self.assertEqual(data_inv.at(0).getTranslation()[0], -1.0)
        self.assertEqual(data_inv.at(1).getTranslation()[0], -3.0)
        self.assertEqual(data.at(0).getTranslation()[0], 1.0)
        self.assertEqual(data.at(1).getTranslation()[0], 3.0)

        # inplace
        data.inverse_()
        self.assertEqual(data.at(0).getTranslation()[0], -1.0)
        self.assertEqual(data.at(1).getTranslation()[0], -3.0)

    def test_normalized(self):
        # container
        data = m3d.TransformContainer([
            m3d.EulerTransform(np.zeros(3), 2 * np.pi, 0.0, 0.0, m3d.EulerAxes.kSXYZ),
            m3d.EulerTransform(np.zeros(3), 3 * np.pi, 0.0, 0.0, m3d.EulerAxes.kSXYZ)
        ], False)

        # non-inplace
        data_norm = data.normalized()
        np.testing.assert_almost_equal(data_norm.at(0).getAi(), 0.0)
        np.testing.assert_almost_equal(data_norm.at(1).getAi(), -np.pi)
        np.testing.assert_almost_equal(data.at(0).getAi(), 2 * np.pi)
        np.testing.assert_almost_equal(data.at(1).getAi(), 3 * np.pi)

        # inplace
        data.normalized_()
        np.testing.assert_almost_equal(data.at(0).getAi(), 0.0)
        np.testing.assert_almost_equal(data.at(1).getAi(), -np.pi)

    def test_scale_translation(self):
        # container
        data = m3d.TransformContainer([
            m3d.EulerTransform(np.ones(3) * 1.0, 0.0, 0.0, 0.0, m3d.EulerAxes.kSXYZ),
            m3d.EulerTransform(np.ones(3) * 2.0, 0.0, 0.0, 0.0, m3d.EulerAxes.kSXYZ)
        ], False)

        # non-inplace
        data_scaled = data.scaleTranslation(2.0)
        np.testing.assert_almost_equal(data_scaled.at(0).getTranslation(), np.ones(3) * 2.0)
        np.testing.assert_almost_equal(data_scaled.at(1).getTranslation(), np.ones(3) * 4.0)
        np.testing.assert_almost_equal(data.at(0).getTranslation(), np.ones(3) * 1.0)
        np.testing.assert_almost_equal(data.at(1).getTranslation(), np.ones(3) * 2.0)

        # inplace
        data.scaleTranslation_(2.0)
        np.testing.assert_almost_equal(data.at(0).getTranslation(), np.ones(3) * 2.0)
        np.testing.assert_almost_equal(data.at(1).getTranslation(), np.ones(3) * 4.0)

    def test_apply(self):
        # transform
        transform = m3d.DualQuaternionTransform()

        # containers
        data = m3d.TransformContainer([
            m3d.EulerTransform([1.0, 0.0, 0.0], 0.0, 0.0, 0.0,
                               m3d.EulerAxes.kSXYX).asType(m3d.TransformType.kMatrix),
            m3d.EulerTransform([2.0, 0.0, 0.0], 0.0, 0.0, 0.0,
                               m3d.EulerAxes.kSXYX).asType(m3d.TransformType.kDualQuaternion)], False)
        data_stamped = m3d.TransformContainer(True, False)
        data_stamped.insert(m3d.Time(100.0), m3d.EulerTransform([1.0, 0.0, 0.0], 0.0, 0.0, 0.0,
                                                                m3d.EulerAxes.kSXYX).asType(m3d.TransformType.kMatrix))
        data_stamped.insert(m3d.Time(200.0), m3d.EulerTransform([2.0, 0.0, 0.0], 0.0, 0.0, 0.0,
                                                                m3d.EulerAxes.kSXYX).asType(m3d.TransformType.kMatrix))

        # apply direct
        data_apply_pre = data.applyPre(transform)
        self.assertTrue(data_apply_pre.at(0).isType(m3d.TransformType.kMatrix))
        self.assertTrue(data_apply_pre.at(1).isType(m3d.TransformType.kDualQuaternion))

        data_apply_post = data.applyPost(transform)
        self.assertTrue(data_apply_post.at(0).isType(m3d.TransformType.kMatrix))
        self.assertTrue(data_apply_post.at(1).isType(m3d.TransformType.kDualQuaternion))

        data_apply = data.apply(transform, transform)
        self.assertTrue(data_apply.at(0).isType(m3d.TransformType.kMatrix))
        self.assertTrue(data_apply.at(1).isType(m3d.TransformType.kDualQuaternion))

        # apply functions
        data_apply_func = data.applyFunc(trafo_func)
        np.testing.assert_almost_equal(
            data_apply_func.at(0).asType(m3d.TransformType.kEuler).getTranslation()[0], 11.0)
        np.testing.assert_almost_equal(
            data_apply_func.at(1).asType(m3d.TransformType.kEuler).getTranslation()[0], 12.0)

        data_apply_ifunc = data.applyIndexFunc(index_func)
        np.testing.assert_almost_equal(
            data_apply_ifunc.at(0).asType(m3d.TransformType.kEuler).getTranslation()[0], 11.0)
        np.testing.assert_almost_equal(
            data_apply_ifunc.at(1).asType(m3d.TransformType.kEuler).getTranslation()[0], 22.0)

        data_apply_sfunc = data_stamped.applyStampFunc(stamp_func)
        np.testing.assert_almost_equal(
            data_apply_sfunc.at(0).asType(m3d.TransformType.kEuler).getTranslation()[0], 111.0)
        np.testing.assert_almost_equal(
            data_apply_sfunc.at(1).asType(m3d.TransformType.kEuler).getTranslation()[0], 212.0)

        # apply direct inplace
        data_apply_pre_inpl = data.copy().applyPre_(transform)
        self.assertTrue(data_apply_pre_inpl.at(0).isType(m3d.TransformType.kMatrix))
        self.assertTrue(data_apply_pre_inpl.at(1).isType(m3d.TransformType.kDualQuaternion))

        data_apply_post_inpl = data.copy().applyPost_(transform)
        self.assertTrue(data_apply_post_inpl.at(0).isType(m3d.TransformType.kMatrix))
        self.assertTrue(data_apply_post_inpl.at(1).isType(m3d.TransformType.kDualQuaternion))

        data_apply_inpl = data.copy().apply_(transform, transform)
        self.assertTrue(data_apply_inpl.at(0).isType(m3d.TransformType.kMatrix))
        self.assertTrue(data_apply_inpl.at(1).isType(m3d.TransformType.kDualQuaternion))

        # apply functions inplace
        data_apply_func_inpl = data.copy().applyFunc_(trafo_func)
        np.testing.assert_almost_equal(
            data_apply_func_inpl.at(0).asType(m3d.TransformType.kEuler).getTranslation()[0], 11.0)
        np.testing.assert_almost_equal(
            data_apply_func_inpl.at(1).asType(m3d.TransformType.kEuler).getTranslation()[0], 12.0)

        data_apply_ifunc_inpl = data.copy().applyIndexFunc_(index_func)
        np.testing.assert_almost_equal(
            data_apply_ifunc_inpl.at(0).asType(m3d.TransformType.kEuler).getTranslation()[0], 11.0)
        np.testing.assert_almost_equal(
            data_apply_ifunc_inpl.at(1).asType(m3d.TransformType.kEuler).getTranslation()[0], 22.0)

        data_apply_sfunc_inpl = data_stamped.copy().applyStampFunc_(stamp_func)
        np.testing.assert_almost_equal(
            data_apply_sfunc_inpl.at(0).asType(m3d.TransformType.kEuler).getTranslation()[0], 111.0)
        np.testing.assert_almost_equal(
            data_apply_sfunc_inpl.at(1).asType(m3d.TransformType.kEuler).getTranslation()[0], 212.0)

    def test_get_list(self):
        # container
        data = m3d.TransformContainer([m3d.MatrixTransform(), m3d.DualQuaternionTransform()], False)

        # original type
        data_original = data.toList()
        self.assertIsInstance(data_original, list)
        self.assertTrue(data_original[0].isType(m3d.TransformType.kMatrix))
        self.assertTrue(data_original[1].isType(m3d.TransformType.kDualQuaternion))

        # specific types
        for ttype, _ in TRANSFORM_CLASSES.items():
            data_list = data.toList(ttype)
            self.assertTrue(data_list[0].isType(ttype))
            self.assertTrue(data_list[1].isType(ttype))

    def test_get_eigen(self):
        # container
        data = m3d.TransformContainer([
            m3d.MatrixTransform(),
            m3d.DualQuaternionTransform(),
            m3d.EulerTransform()], False)

        # check types
        for ttype, tarr in TRANSFORM_ARRAYS.items():
            mat = data.toArray(ttype)
            self.assertEqual(mat.shape, (3, len(tarr)))

    def test_description(self):
        data = m3d.TransformContainer(False, False)
        self.assertTrue(str(data).startswith('TransformContainer'))


if __name__ == '__main__':
    unittest.main()
