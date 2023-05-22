import filecmp
import unittest

import numpy as np

import motion3d as m3d

from data import get_data_file, get_output_file


class TestIO(unittest.TestCase):
    def test_motion_data(self):
        # data
        container = m3d.TransformContainer([m3d.DualQuaternionTransform(), m3d.MatrixTransform()], False)
        origin = m3d.QuaternionTransform()
        ttype = m3d.TransformType.kAxisAngle

        # empty constructor
        motion = m3d.MotionData()
        self.assertEqual(motion.getFrameId(), '')
        self.assertIsNone(motion.getTransformType())
        self.assertIsNone(motion.getOrigin())
        self.assertIsNone(motion.getTransforms())

        # frame id constructor
        motion = m3d.MotionData('frame')
        self.assertEqual(motion.getFrameId(), 'frame')
        self.assertIsNone(motion.getTransformType())
        self.assertIsNone(motion.getOrigin())
        self.assertIsNone(motion.getTransforms())

        # transforms constructor
        motion = m3d.MotionData(ttype, container)
        self.assertEqual(motion.getFrameId(), '')
        self.assertEqual(motion.getTransformType(), ttype)
        self.assertIsNone(motion.getOrigin())
        self.assertEqual(motion.getTransforms().size(), 2)

        # transforms and origin constructor
        motion = m3d.MotionData(ttype, container, origin)
        self.assertEqual(motion.getFrameId(), '')
        self.assertEqual(motion.getTransformType(), ttype)
        self.assertNotEqual(motion.getOrigin(), origin)
        self.assertTrue(motion.getOrigin().isType(m3d.TransformType.kQuaternion))
        self.assertTrue(motion.getOrigin().isEqual(origin))
        self.assertEqual(motion.getTransforms().size(), 2)

        # setter
        motion.setFrameId('frame')
        self.assertEqual(motion.getFrameId(), 'frame')

        motion.setTransformType(m3d.TransformType.kEuler)
        self.assertEqual(motion.getTransformType(), m3d.TransformType.kEuler)

        motion.setOrigin(m3d.EulerTransform())
        self.assertTrue(motion.getOrigin().isType(m3d.TransformType.kEuler))

        motion.setTransforms(m3d.TransformContainer([], True))
        self.assertTrue(motion.getTransforms().hasPoses())

        # description
        self.assertTrue(str(motion).startswith('MotionData'))

    def test_read_interface(self):
        # data as argument
        motion1 = m3d.MotionData()
        return_code1 = m3d.M3DReader.read(get_data_file('motion1.m3d'), motion1, unsafe=True)
        self.assertEqual(return_code1, m3d.M3DIOStatus.kSuccess)

        return_code2 = m3d.M3DReader.read(get_data_file('motion_invalid.m3d'), motion1, unsafe=True)
        self.assertNotEqual(return_code2, m3d.M3DIOStatus.kSuccess)

        # data as return
        motion3, return_code3 = m3d.M3DReader.read(get_data_file('motion3.m3d'), unsafe=True)
        self.assertEqual(return_code3, m3d.M3DIOStatus.kSuccess)
        self.assertIsInstance(motion3, m3d.MotionData)

        motion4, return_code4 = m3d.M3DReader.read(get_data_file('motion_invalid.m3d'), unsafe=True)
        self.assertNotEqual(return_code4, m3d.M3DIOStatus.kSuccess)
        self.assertIsNone(motion4)

    def test_read_ascii_data(self):
        # read data
        motion1, return_code1 = m3d.M3DReader.read(get_data_file('motion1.m3d'), unsafe=True)
        self.assertEqual(return_code1, m3d.M3DIOStatus.kSuccess)
        transforms1 = motion1.getTransforms()

        # check types
        self.assertEqual(motion1.getFrameId(), "camera1")
        self.assertEqual(motion1.getTransformType(), m3d.TransformType.kAxisAngle)
        self.assertTrue(motion1.getOrigin().isType(m3d.TransformType.kAxisAngle))
        self.assertFalse(transforms1.hasStamps())
        self.assertFalse(transforms1.hasPoses())
        for transform in transforms1:
            self.assertTrue(transform.isType(m3d.TransformType.kAxisAngle))

        # check origin
        for v_idx, value in enumerate(motion1.getOrigin().toList()):
            np.testing.assert_almost_equal(value, v_idx + 1)

        # check data
        self.assertEqual(transforms1.size(), 3)
        for t_idx, transform in enumerate(transforms1):
            for v_idx, value in enumerate(transform.toList()):
                np.testing.assert_almost_equal(value, (v_idx + 1.0) * (10.0 ** (t_idx - 1)) + 1.0)

        # read data with stamps
        motion3, return_code3 = m3d.M3DReader.read(get_data_file('motion3.m3d'), unsafe=True)
        self.assertEqual(return_code3, m3d.M3DIOStatus.kSuccess)
        transforms3 = motion3.getTransforms()

        # check types
        self.assertEqual(motion3.getFrameId(), "camera3")
        self.assertEqual(motion3.getTransformType(), m3d.TransformType.kAxisAngle)
        self.assertTrue(motion3.getOrigin().isType(m3d.TransformType.kAxisAngle))
        self.assertTrue(transforms3.hasStamps())
        self.assertFalse(transforms3.hasPoses())
        for transform in transforms3:
            self.assertTrue(transform.isType(m3d.TransformType.kAxisAngle))

        # check data with stamps
        self.assertEqual(transforms3.size(), 3)
        for t_idx in range(transforms3.size()):
            self.assertEqual(transforms3.stamp_at(t_idx).toNSec(), (t_idx + 1) * 100)

            for v_idx, value in enumerate(transforms3.at(t_idx).toList()):
                np.testing.assert_almost_equal(value, (v_idx + 1.0) * (10.0 ** (t_idx - 1)))

    def test_write_and_read(self):
        # output files
        output_file_ascii = get_output_file('motion1_ascii.m3d')
        output_file_binary = get_output_file('motion1_binary.m3d')

        # read data
        motion, _ = m3d.M3DReader.read(get_data_file('motion1.m3d'), unsafe=True)
        self.assertIsNotNone(motion)

        # write data
        return_code_writer_ascii = m3d.M3DWriter.writeASCII(output_file_ascii, motion)
        self.assertEqual(return_code_writer_ascii, m3d.M3DIOStatus.kSuccess)

        return_code_writer_binary = m3d.M3DWriter.writeBinary(output_file_binary, motion)
        self.assertEqual(return_code_writer_binary, m3d.M3DIOStatus.kSuccess)

        # read data for checking
        motion_check_ascii, _ = m3d.M3DReader.read(output_file_ascii, unsafe=True)
        self.assertIsNotNone(motion_check_ascii)
        motion_check_binary, _ = m3d.M3DReader.read(output_file_binary, unsafe=True)
        self.assertIsNotNone(motion_check_binary)

        # check origin
        np.testing.assert_almost_equal(motion.getOrigin().toArray(), motion_check_ascii.getOrigin().toArray())
        np.testing.assert_almost_equal(motion.getOrigin().toArray(), motion_check_binary.getOrigin().toArray())

        # check transforms
        self.assertEqual(motion.getTransforms().size(), motion_check_ascii.getTransforms().size())
        self.assertEqual(motion.getTransforms().size(), motion_check_binary.getTransforms().size())

        for transform, transform_ascii, transform_binary in zip(motion.getTransforms(),
                                                                motion_check_ascii.getTransforms(),
                                                                motion_check_binary.getTransforms()):
            np.testing.assert_almost_equal(transform.toArray(), transform_ascii.toArray())
            np.testing.assert_almost_equal(transform.toArray(), transform_binary.toArray())

    def test_write_and_read_with_stamps(self):
        # output files
        output_file_ascii = get_output_file('motion3_ascii.m3d')
        output_file_binary = get_output_file('motion3_binary.m3d')

        # read data
        motion, _ = m3d.M3DReader.read(get_data_file('motion3.m3d'), unsafe=True)
        self.assertIsNotNone(motion)

        # write data
        return_code_writer_ascii = m3d.M3DWriter.writeASCII(output_file_ascii, motion)
        self.assertEqual(return_code_writer_ascii, m3d.M3DIOStatus.kSuccess)

        return_code_writer_binary = m3d.M3DWriter.writeBinary(output_file_binary, motion)
        self.assertEqual(return_code_writer_binary, m3d.M3DIOStatus.kSuccess)

        # read data for checking
        motion_check_ascii, _ = m3d.M3DReader.read(output_file_ascii, unsafe=True)
        self.assertIsNotNone(motion_check_ascii)
        motion_check_binary, _ = m3d.M3DReader.read(output_file_binary, unsafe=True)
        self.assertIsNotNone(motion_check_binary)

        # check origin
        np.testing.assert_almost_equal(motion.getOrigin().toArray(), motion_check_ascii.getOrigin().toArray())
        np.testing.assert_almost_equal(motion.getOrigin().toArray(), motion_check_binary.getOrigin().toArray())

        # check stamps and transforms
        self.assertEqual(motion.getTransforms().size(), motion_check_ascii.getTransforms().size())
        self.assertEqual(motion.getTransforms().size(), motion_check_binary.getTransforms().size())

        for t_idx, (transform, transform_ascii, transform_binary) in \
                enumerate(zip(motion.getTransforms(), motion_check_ascii.getTransforms(),
                              motion_check_binary.getTransforms())):
            # stamps
            self.assertEqual(motion.getTransforms().stamp_at(t_idx),
                             motion_check_ascii.getTransforms().stamp_at(t_idx))
            self.assertEqual(motion.getTransforms().stamp_at(t_idx),
                             motion_check_binary.getTransforms().stamp_at(t_idx))

            # transforms
            np.testing.assert_almost_equal(transform.toArray(), transform_ascii.toArray())
            np.testing.assert_almost_equal(transform.toArray(), transform_binary.toArray())

    def test_write_and_read_origin(self):
        # output files
        output_file_origin_ascii = get_output_file('motion_origin_ascii.m3d')
        output_file_origin_binary = get_output_file('motion_origin_binary.m3d')
        output_file_no_origin_ascii = get_output_file('motion_no_origin_ascii.m3d')
        output_file_no_origin_binary = get_output_file('motion_no_origin_binary.m3d')

        # read data
        motion1, _ = m3d.M3DReader.read(get_data_file('motion1.m3d'), unsafe=True)
        self.assertIsNotNone(motion1)

        motion2, _ = m3d.M3DReader.read(get_data_file('motion2.m3d'), unsafe=True)
        self.assertIsNotNone(motion2)

        # check origins
        self.assertIsNotNone(motion1.getOrigin())
        self.assertIsNone(motion2.getOrigin())

        # write data
        return_code_writer_origin_ascii = m3d.M3DWriter.writeASCII(output_file_origin_ascii, motion1)
        self.assertEqual(return_code_writer_origin_ascii, m3d.M3DIOStatus.kSuccess)

        return_code_writer_origin_binary = m3d.M3DWriter.writeBinary(output_file_origin_binary, motion1)
        self.assertEqual(return_code_writer_origin_binary, m3d.M3DIOStatus.kSuccess)

        return_code_writer_no_origin_ascii = m3d.M3DWriter.writeASCII(output_file_no_origin_ascii, motion2)
        self.assertEqual(return_code_writer_no_origin_ascii, m3d.M3DIOStatus.kSuccess)

        return_code_writer_no_origin_binary = m3d.M3DWriter.writeBinary(output_file_no_origin_binary, motion2)
        self.assertEqual(return_code_writer_no_origin_binary, m3d.M3DIOStatus.kSuccess)

        # read data for checking
        motion_check_origin_ascii, _ = m3d.M3DReader.read(output_file_origin_ascii, unsafe=True)
        self.assertIsNotNone(motion_check_origin_ascii)
        motion_check_origin_binary, _ = m3d.M3DReader.read(output_file_origin_binary, unsafe=True)
        self.assertIsNotNone(motion_check_origin_binary)
        motion_check_no_origin_ascii, _ = m3d.M3DReader.read(output_file_no_origin_ascii, unsafe=True)
        self.assertIsNotNone(motion_check_no_origin_ascii)
        motion_check_no_origin_binary, _ = m3d.M3DReader.read(output_file_no_origin_binary, unsafe=True)
        self.assertIsNotNone(motion_check_no_origin_binary)

        # check origins
        self.assertIsNotNone(motion_check_origin_ascii.getOrigin())
        self.assertIsNotNone(motion_check_origin_binary.getOrigin())
        self.assertIsNone(motion_check_no_origin_ascii.getOrigin())
        self.assertIsNone(motion_check_no_origin_binary.getOrigin())

    def test_write_genereric(self):
        # output files
        output_file_ascii1 = get_output_file('motion3_ascii1.m3d')
        output_file_ascii2 = get_output_file('motion3_ascii2.m3d')
        output_file_binary1 = get_output_file('motion3_binary1.m3d')
        output_file_binary2 = get_output_file('motion3_binary2.m3d')

        # read data
        motion, _ = m3d.M3DReader.read(get_data_file('motion3.m3d'), unsafe=True)
        self.assertIsNotNone(motion)

        # ascii
        return_code_writer_ascii1 = m3d.M3DWriter.writeASCII(output_file_ascii1, motion, precision=12)
        self.assertEqual(return_code_writer_ascii1, m3d.M3DIOStatus.kSuccess)
        return_code_writer_ascii2 = m3d.M3DWriter.write(output_file_ascii2, motion, m3d.M3DFileType.kASCII,
                                                        precision=12)
        self.assertEqual(return_code_writer_ascii2, m3d.M3DIOStatus.kSuccess)
        self.assertTrue(filecmp.cmp(output_file_ascii1, output_file_ascii2, shallow=False))

        # binary
        return_code_writer_binary1 = m3d.M3DWriter.writeBinary(output_file_binary1, motion)
        self.assertEqual(return_code_writer_binary1, m3d.M3DIOStatus.kSuccess)
        return_code_writer_binary2 = m3d.M3DWriter.write(output_file_binary2, motion, m3d.M3DFileType.kBinary,
                                                         precision=12)
        self.assertEqual(return_code_writer_binary2, m3d.M3DIOStatus.kSuccess)
        self.assertTrue(filecmp.cmp(output_file_binary1, output_file_binary2, shallow=False))

    def test_write_error(self):
        # output files
        error_file = get_output_file('motion_error.m3d')

        # no transforms
        motion = m3d.MotionData()

        return_code1a = m3d.M3DWriter.writeASCII(error_file, motion)
        self.assertEqual(return_code1a, m3d.M3DIOStatus.kNoTransforms)
        return_code1b = m3d.M3DWriter.writeBinary(error_file, motion)
        self.assertEqual(return_code1b, m3d.M3DIOStatus.kNoTransforms)

        # no transform type
        motion.setTransforms(m3d.TransformContainer([m3d.AxisAngleTransform()], False))

        return_code2a = m3d.M3DWriter.writeASCII(error_file, motion)
        self.assertEqual(return_code2a, m3d.M3DIOStatus.kNoTransformType)
        return_code2b = m3d.M3DWriter.writeBinary(error_file, motion)
        self.assertEqual(return_code2b, m3d.M3DIOStatus.kNoTransformType)


if __name__ == '__main__':
    unittest.main()
