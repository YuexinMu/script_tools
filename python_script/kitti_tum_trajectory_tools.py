import rospy
import rosbag
from scipy.spatial.transform import Rotation as R
import os
from geometry_msgs.msg import PoseStamped
import argparse
import numpy as np
from datetime import datetime


class KittiTumTrajectory():
    def __init__(self):
        # different kitti dataset sequence correspond
        seq_map = {
            "00": {"name": "2011_10_03_drive_0027", "start": 0, "end": 4540},
            "01": {"name": "2011_10_03_drive_0042", "start": 0, "end": 1100},
            "02": {"name": "2011_10_03_drive_0034", "start": 0, "end": 4660},
            "03": {"name": "2011_09_26_drive_0067", "start": 0, "end": 800},
            "04": {"name": "2011_09_30_drive_0016", "start": 0, "end": 270},
            "05": {"name": "2011_09_30_drive_0018", "start": 0, "end": 2760},
            "06": {"name": "2011_09_30_drive_0020", "start": 0, "end": 1100},
            "07": {"name": "2011_09_30_drive_0027", "start": 0, "end": 1100},
            "08": {"name": "2011_09_30_drive_0028", "start": 1100, "end": 5170},
            "09": {"name": "2011_09_30_drive_0033", "start": 0, "end": 1590},
            "10": {"name": "2011_09_30_drive_0034", "start": 0, "end": 1200}
        }
        sequence = "08"
        method = "disco"
        path = "/home/myx/develop/data/kitti_odometry/comparison/{}".format(sequence)

        self.operation_seq = seq_map[sequence]

        # input files
        self.raw_kitti_pose_file = "{}/{}.txt".format(path, sequence)
        self.raw_timestamp_file = "{}/timestamps.txt".format(path)
        self.kitti_trajectory_bag = "{}/tra_kitti08_path.bag".format(path)

        # output files
        self.gt_tum_file_name = "{}/ground_truth_{}.txt".format(path, sequence)
        self.split_infos = {
            "01": {"ref_fn": "{}/ref_tra1.txt".format(path),
                   "tum_fn": "{}/{}_tra1.txt".format(path, method),
                   "bag_topic": "/robot_0/lio_sam/mapping/path",
                   "start": 0, "end": 1614
                   },
            "02": {"ref_fn": "{}/ref_tra2.txt".format(path),
                   "tum_fn": "{}/{}_tra2.txt".format(path, method),
                   "bag_topic": "/robot_1/lio_sam/mapping/path",
                   "start": 1515, "end": 3328
                   },
            "03": {"ref_fn": "{}/ref_tra3.txt".format(path),
                   "tum_fn": "{}/{}_tra3.txt".format(path, method),
                   "bag_topic": "/robot_2/lio_sam/mapping/path",
                   "start": 3029, "end": 5176
                   },
        }

        # init
        self.timestamps = self.read_timestamps()
        self.poses = self.read_raw_poses()

    def convert_bag_to_tum(self):
        with rosbag.Bag(self.kitti_trajectory_bag, 'r') as bag:
            for split_info in self.split_infos.values():
                for topic, msg, t in bag.read_messages(topics=split_info["bag_topic"]):
                    pass
                with open(split_info["tum_fn"], 'w') as tum_file:
                    for pose in msg.poses:
                        timestamp = pose.header.stamp.to_sec()
                        position = pose.pose.position
                        orientation = pose.pose.orientation
                        tum_file.write(
                            f"{timestamp} {position.x} {position.y} {position.z} {orientation.x} {orientation.y} {orientation.z} {orientation.w}\n")

    def read_raw_poses(self):
        raw_kitti_poses = []
        with open(self.raw_kitti_pose_file, 'r') as f:
            for line in f:
                values = list(map(float, line.strip().split()))
                pose = np.array(values).reshape(3, 4)
                raw_kitti_poses.append(pose)
        return raw_kitti_poses

    @staticmethod
    def convert_timestamp(timestamp):
        # 解析时间戳字符串
        dt = datetime.strptime(timestamp[:-3], '%Y-%m-%d %H:%M:%S.%f')

        # 计算时间戳的浮点表示（秒）
        seconds = datetime.timestamp(dt)
        # 将浮点表示转换为科学计数法形式
        # scientific_notation = f'{seconds:.6e}'
        return str(seconds)

    def read_timestamps(self):
        timestamps = []
        with open(self.raw_timestamp_file, 'r') as f:
            lines = f.readlines()
            for line in lines[:]:
                # 将时间戳转换为科学计数法形式
                scientific_notation = self.convert_timestamp(line.strip())
                timestamps.append(scientific_notation)
        return timestamps

    @staticmethod
    def kitti_pose_to_quaternion(pose):
        pose_rotation = pose[:3, :3]
        pose_transform = pose[:3, 3]

        q_x, q_y, q_z, q_w = R.from_matrix(pose_rotation).as_quat()

        return [pose_transform[0], pose_transform[1], pose_transform[2], q_x, q_y, q_z, q_w]

    def convert_gt_pose_to_tum(self):
        start_line = self.operation_seq["start"]
        end_line = self.operation_seq["end"]
        split_timestamps = self.timestamps[start_line - 1:end_line]

        error_msg = ("timestamp file must have same number of rows as the KITTI poses file")
        if len(split_timestamps) != len(self.poses):
            print(error_msg)
            return

        with open(self.gt_tum_file_name, 'w') as tum_file:
            for timestamp, pose in zip(self.timestamps, self.poses):
                pose_quat = self.kitti_pose_to_quaternion(pose)
                tum_line = [timestamp] + pose_quat
                tum_file.write(' '.join(f'{value}' for value in tum_line) + "\n")

    def convert_split_pose_to_tum(self):
        pose_start_line = self.operation_seq["start"]
        pose_end_line = self.operation_seq["end"]
        self.timestamps = self.timestamps[pose_start_line - 1:pose_end_line]

        for split_info in self.split_infos.values():
            split_start = split_info["start"]
            split_end = split_info["end"]
            # 选取不同数据段的索引
            start_line = split_start - pose_start_line if split_start > pose_start_line else split_start
            end_line = split_end - pose_start_line if split_end < pose_end_line else pose_end_line - pose_start_line
            with open(split_info["ref_fn"], 'w') as tum_file:
                for timestamp, pose in zip(self.timestamps[start_line:end_line],
                                           self.poses[start_line:end_line]):
                    pose_quat = self.kitti_pose_to_quaternion(pose)
                    tum_line = [timestamp] + pose_quat
                    tum_file.write(' '.join(f'{value}' for value in tum_line) + "\n")


if __name__ == "__main__":
    kitti_tum_trajectory = KittiTumTrajectory()
    # kitti_tum_trajectory.convert_bag_to_tum()
    kitti_tum_trajectory.convert_gt_pose_to_tum()
    kitti_tum_trajectory.convert_split_pose_to_tum()
