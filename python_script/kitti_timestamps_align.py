import numpy as np
# import datetime as dt
from datetime import datetime
import os


def convert_timestamp(timestamp):
    # 解析时间戳字符串
    dt = datetime.strptime(timestamp[:-3], '%Y-%m-%d %H:%M:%S.%f')

    # 计算时间戳的浮点表示（秒）
    seconds = datetime.timestamp(dt)

    # 将浮点表示转换为科学计数法形式
    # scientific_notation = f'{seconds:.6e}'

    return str(seconds)


if __name__ == "__main__":
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
    path = "/home/myx/develop/data/kitti_odometry/comparison/08/"

    raw_pose_name = path + "timestamps.txt"
    sync_time_name = path + "sync_time.txt"

    seq = seq_map["08"]

    with open(raw_pose_name, 'r') as src_file, open(sync_time_name, 'w') as dest_file:
        # 读取原始文件内容
        lines = src_file.readlines()

        # 复制指定范围的行到目标文件
        start_line = seq["start"]
        end_line = seq["end"]
        for line in lines[start_line - 1:end_line]:
            # 将时间戳转换为科学计数法形式
            scientific_notation = convert_timestamp(line.strip())
            dest_file.write(scientific_notation + '\n')
