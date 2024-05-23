#!/usr/bin/env python
import rospy
import rosbag
import os
from geometry_msgs.msg import PoseStamped
import argparse
import numpy as np


def convert_bag_to_tum(bag_file, topic, tum_file):
    with rosbag.Bag(bag_file, 'r') as bag, open(tum_file, 'w') as tum_file:
        # (topic, msg, t) = bag.read_messages(topics=topic)
        for topic, msg, t in bag.read_messages(topics=topic):
            pass
        for pose in msg.poses:
            timestamp = pose.header.stamp.to_sec()
            position = pose.pose.position
            orientation = pose.pose.orientation
            tum_file.write(
                f"{timestamp} {position.x} {position.y} {position.z} {orientation.x} {orientation.y} {orientation.z} {orientation.w}\n")


def quaternion_to_rotation_matrix(q):
    """
    Convert a quaternion into a 3x3 rotation matrix.
    """
    q = np.array([q.x, q.y, q.z, q.w])
    n = np.dot(q, q)
    if n < np.finfo(q.dtype).eps:
        return np.identity(3)
    q *= np.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0 - q[1][1] - q[2][2], q[0][1] - q[2][3], q[0][2] + q[1][3]],
        [q[0][1] + q[2][3], 1.0 - q[0][0] - q[2][2], q[1][2] - q[0][3]],
        [q[0][2] - q[1][3], q[1][2] + q[0][3], 1.0 - q[0][0] - q[1][1]]
    ])


def convert_bag_to_kitti(bag_file, topic, kitti_file):
    with rosbag.Bag(bag_file, 'r') as bag, open(kitti_file, 'w') as kitti:
        for topic, msg, t in bag.read_messages(topics=topic):
            pass
        for pose in msg.poses:
            position = pose.pose.position
            orientation = pose.pose.orientation
            rotation_matrix = quaternion_to_rotation_matrix(orientation)
            pose_matrix = np.hstack((rotation_matrix, [[position.x], [position.y], [position.z]]))
            # 将3x4的位姿矩阵（pose）转换为一个单行的字符串，其中元素之间用空格分隔
            pose_str = ' '.join(f'{num:.6e}' for num in pose_matrix.flatten())
            kitti.write(pose_str + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert ROS bag to TUM format')

    path = "/home/myx/develop/data/kitti_odometry/comparison/08/"

    bag_file = path + "tra_kitti08_path.bag"
    output_file = path + "disco_tra2.txt"

    topic = "/robot_2/lio_sam/mapping/path"

    parser.add_argument('--bag_file', default=bag_file, help='Input ROS bag file')
    parser.add_argument('--output_file', default=output_file, help='Output TUM format file')
    parser.add_argument('--topic', default=topic, help='ROS topic to extract')
    args = parser.parse_args()

    convert_bag_to_tum(args.bag_file, args.topic, args.output_file)
    # convert_bag_to_kitti(args.bag_file, args.topic, args.output_file)
