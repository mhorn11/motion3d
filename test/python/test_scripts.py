import itertools
import subprocess
import unittest

import motion3d as m3d

from data import get_data_file, get_output_file


class TestScripts(unittest.TestCase):
    @staticmethod
    def _run_command(command):
        process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        std_out, _ = process.communicate()
        return process.returncode, std_out

    def _convert_kitti_odometry(self, kitti_poses, *args):
        # filenames
        kitti_output = get_output_file('kitti_', '.m3d')

        # convert
        command = ['m3d_convert_kitti_odometry', kitti_poses, kitti_output, *args]
        returncode, msg = self._run_command(command)
        self.assertEqual(returncode, 0)

        # read and check
        motion_data, status = m3d.M3DReader.read(kitti_output)
        self.assertEqual(status, m3d.M3DIOStatus.kSuccess)
        self.assertEqual(motion_data.getTransformType(), m3d.TransformType.kMatrix)
        self.assertEqual(motion_data.getTransforms().size(), 10)

    def test_convert_kitti_odometry(self):
        # filenames
        kitti_calib = get_data_file('kitti', 'calib.txt')
        kitti_poses = get_data_file('kitti', 'poses.txt')
        kitti_times = get_data_file('kitti', 'times.txt')

        # argument combinations
        kitti_args = [
            [],
            ['--calib', kitti_calib],
            ['--calib', kitti_calib, '--frame', 'CAM0'],
            ['--times', kitti_times],
            ['--calib', kitti_calib, '--times', kitti_times, '--frame', 'CAM0'],
            ['--binary'],
        ]

        # iterate
        for args in kitti_args:
            self._convert_kitti_odometry(kitti_poses, *args)

    def _convert_rosbag(self, src, topic, *args):
        # filenames
        rosbag_output = get_output_file('rosbag_', '.m3d')

        # convert
        command = ['m3d_convert_rosbag', src, topic, rosbag_output, *args,
                   '--base-frame', 'frame1', '--child-frame', 'frame2']
        returncode, msg = self._run_command(command)
        self.assertEqual(returncode, 0)

        # read and check
        motion_data, status = m3d.M3DReader.read(rosbag_output)
        self.assertEqual(status, m3d.M3DIOStatus.kSuccess)
        self.assertEqual(motion_data.getTransformType(), m3d.TransformType.kQuaternion)
        self.assertEqual(motion_data.getTransforms().size(), 10)

    def test_convert_rosbag(self):
        # filenames
        rosbag_sources = [get_data_file('rosbag', 'rosbag1.bag'), get_data_file('rosbag', 'rosbag2')]

        # topics
        rosbag_topics = ["/poses1", "/poses2", "/poses3"]

        # argument combinations
        rosbag_args = [
            [],
            ['--msg-stamp'],
            ['--binary'],
        ]

        # iterate
        for src, topic, args in itertools.product(rosbag_sources, rosbag_topics, rosbag_args):
            self._convert_rosbag(src, topic, *args)

    def test_plot_poses(self, *args):
        # filenames
        filenames = [get_data_file('motion1.m3d'), get_data_file('motion2.m3d')]

        # argument combinations
        plot_args = [
            [],
            ['--pos-step', '2'],
            ['--axes-step', '2'],
            ['--axes-length', '2'],
            ['--origin'],
        ]

        # iterate
        for filename, args in itertools.product(filenames, plot_args):
            command = ['m3d_plot_poses', filename, *args, '--skip-show']
            returncode, msg = self._run_command(command)
            self.assertEqual(returncode, 0)
