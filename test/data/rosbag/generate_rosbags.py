#!/usr/bin/env python3
from rosbags.rosbag1 import Writer as ROS1Writer
from rosbags.rosbag2 import Writer as ROS2Writer
from rosbags.serde import cdr_to_ros1, serialize_cdr
from rosbags.typesys.types import \
    builtin_interfaces__msg__Time as Time, \
    geometry_msgs__msg__Point as Point, \
    geometry_msgs__msg__Pose as Pose, \
    geometry_msgs__msg__PoseStamped as PoseStamped, \
    geometry_msgs__msg__PoseWithCovariance as PoseWithCovariance, \
    geometry_msgs__msg__PoseWithCovarianceStamped as PoseWithCovarianceStamped, \
    geometry_msgs__msg__Quaternion as Quaternion, \
    geometry_msgs__msg__Transform as Transform, \
    geometry_msgs__msg__TransformStamped as TransformStamped, \
    geometry_msgs__msg__Vector3 as Vector3, \
    std_msgs__msg__Header as Header
import numpy as np
import transforms3d as t3d


class TopicData:
    def __init__(self, name, datatype):
        self.name = name
        self.datatype = datatype
        self.msgtype = datatype.__msgtype__
        self.connection1 = None
        self.connection2 = None


class PoseData:
    def __init__(self, translation, quaternion):
        self.translation = translation
        self.quaternion = quaternion

    def to_pose(self):
        return Pose(
            position=Point(self.translation[0], self.translation[1], self.translation[2]),
            orientation=Quaternion(self.quaternion[0], self.quaternion[1], self.quaternion[2], self.quaternion[3]))

    def to_transform(self):
        return Transform(
            translation=Vector3(self.translation[0], self.translation[1], self.translation[2]),
            rotation=Quaternion(self.quaternion[0], self.quaternion[1], self.quaternion[2], self.quaternion[3]))


def random_pose():
    # random euler transform
    translation = np.random.rand(3) * 10.0 - 5.0
    euler_angles = (np.random.rand(3) * 2 * np.pi - np.pi).tolist()
    euler_axes = 'sxyz'

    # quaternion
    quaternion = t3d.euler.euler2quat(*euler_angles, euler_axes)

    return PoseData(translation, quaternion)


def pose2message(datatype, stamp, pose_data):
    header = Header(Time(sec=int(stamp // 10**9), nanosec=int(stamp % 10**9)),
                    frame_id='frame1')

    if datatype == PoseStamped:
        return PoseStamped(header=header, pose=pose_data.to_pose())

    if datatype == PoseWithCovarianceStamped:
        return PoseWithCovarianceStamped(header=header, pose=PoseWithCovariance(
            pose=pose_data.to_pose(), covariance=np.zeros(36)))

    if datatype == TransformStamped:
        return TransformStamped(header=header, child_frame_id='frame2', transform=pose_data.to_transform())


def main():
    # generate topics
    topics = [
        TopicData('/poses1', PoseStamped),
        TopicData('/poses2', PoseWithCovarianceStamped),
        TopicData('/poses3', TransformStamped),
    ]

    # poses
    stamped_poses = [(int(stamp * 1e9), random_pose()) for stamp in range(10)]

    # write
    with ROS1Writer('rosbag1.bag') as writer1, ROS2Writer('rosbag2') as writer2:
        # connections
        for topic in topics:
            topic.connection1 = writer1.add_connection(topic.name, topic.msgtype)
            topic.connection2 = writer2.add_connection(topic.name, topic.msgtype)

        # serialize and write message
        for (stamp, pose) in stamped_poses:
            for topic in topics:
                # get message
                message = pose2message(topic.datatype, stamp, pose)

                # write ROS1
                writer1.write(topic.connection1, stamp, cdr_to_ros1(serialize_cdr(message, topic.msgtype),
                              topic.msgtype))

                # write ROS2
                writer2.write(topic.connection2, stamp, serialize_cdr(message, topic.msgtype))


if __name__ == '__main__':
    main()
