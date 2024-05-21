import numpy as np


def transfrom_TR2xyzrpy(T, R):
    # 计算roll、yaw和pitch
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(-R[2, 0], np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
    pitch = np.arctan2(R[1, 0], R[0, 0])

    # 输出旋转角度
    print("Roll (x): ", roll)
    print("Yaw (y): ", yaw)
    print("Pitch (z): ", pitch)

    # 输出平移向量
    print("Translation (x, y, z): ", tuple(T))


def transfrom_xyzrpy2TR(roll, pitch, yaw):
    """
        将roll、yaw和pitch转换为旋转矩阵
        """
    # 计算旋转矩阵的元素
    sr, sp, sy = np.sin(roll), np.sin(pitch), np.sin(yaw)
    cr, cp, cy = np.cos(roll), np.cos(pitch), np.cos(yaw)

    # 计算旋转矩阵
    R_roll = np.array([[1, 0, 0],
                       [0, cr, -sr],
                       [0, sr, cr]])

    R_pitch = np.array([[cp, 0, sp],
                        [0, 1, 0],
                        [-sp, 0, cp]])

    R_yaw = np.array([[cy, -sy, 0],
                      [sy, cy, 0],
                      [0, 0, 1]])

    # 组合旋转矩阵
    R = np.dot(R_yaw, np.dot(R_pitch, R_roll))

    print(R)


if __name__ == '__main__':
    # 定义旋转矩阵R和平移向量T
    R = np.array([[9.999976e-01, 7.553071e-04, -2.035826e-03],
                  [-7.854027e-04, 9.998898e-01, -1.482298e-02],
                  [2.024406e-03, 1.482454e-02, 9.998881e-01]])
    T = np.array([-8.086759e-01, 3.195559e-01, -7.997231e-01])

    roll = -3.14 / 2
    yaw = 3.14 / 2
    pitch = 0.0
    transfrom_xyzrpy2TR(roll, pitch, yaw)

    # transfrom_TR2xyzrpy(T, R)
