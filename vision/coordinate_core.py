"""坐标转换核心库"""
import cv2
import numpy as np
import json
import os

class CoordinateProcessor:
    def __init__(self, matrix_file=None):
        if matrix_file is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.matrix_file = os.path.join(current_dir, "transform_matrix.json")
        else:
            self.matrix_file = matrix_file
            
        self.transform_matrix = None
        self.is_calibrated = False
        self.load_matrix()

    def load_matrix(self):
        """加载矩阵"""
        if os.path.exists(self.matrix_file):
            try:
                with open(self.matrix_file, 'r') as f:
                    data = json.load(f)
                    self.transform_matrix = np.array(data["transform_matrix"])
                    self.is_calibrated = True
                    print("坐标变换矩阵(Homography)加载成功")
            except Exception as e:
                print(f"矩阵加载失败: {e}")
        else:
            print("未找到坐标矩阵文件，请运行标定程序。")

    def save_matrix(self, matrix):
        """保存矩阵"""
        data = {
            "transform_matrix": matrix.tolist()
        }
        with open(self.matrix_file, 'w') as f:
            json.dump(data, f, indent=4)
        print(f"矩阵已保存至 {self.matrix_file}")
        self.load_matrix()

    def calibrate_affine(self, img_points, robot_points):
        """
        升级为计算透视变换矩阵 (Homography)
        比仿射变换更抗倾斜
        """
        if len(img_points) < 4:
            print("错误：透视变换至少需要4组点")
            return False, None

        pts_img = np.array(img_points, dtype=np.float32)
        pts_robot = np.array(robot_points, dtype=np.float32)

        # 使用 findHomography 替代 estimateAffine2D
        # 它可以处理梯形畸变（即摄像头不垂直的情况）
        matrix, status = cv2.findHomography(pts_img, pts_robot)
        
        if matrix is not None:
            self.save_matrix(matrix)
            return True, matrix
        else:
            return False, None

    def pixel_to_robot(self, u, v):
        """
        透视变换坐标转换
        """
        if self.transform_matrix is None:
            return 0, 0

        # 透视变换需要 (x, y, 1)
        point = np.array([[[u, v]]], dtype=np.float32)
        
        # 使用 perspectiveTransform
        transformed_point = cv2.perspectiveTransform(point, self.transform_matrix)
        
        x = transformed_point[0][0][0]
        y = transformed_point[0][0][1]
        return float(x), float(y)
