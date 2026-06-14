import cv2
import numpy as np
import json
import os

class CameraProcessor:
    def __init__(self, params_file=None):
        # 自动定位参数文件路径
        if params_file is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.params_file = os.path.join(current_dir, "camera_params.json")
        else:
            self.params_file = params_file
            
        self.camera_matrix = None
        self.dist_coeffs = None
        self.is_calibrated = False
        
        # 记录标定时的图像尺寸 (width, height)
        self.calib_size = None 
        
        self.load_params()

    def load_params(self):
        """加载标定参数"""
        if os.path.exists(self.params_file):
            try:
                with open(self.params_file, 'r') as f:
                    data = json.load(f)
                    self.camera_matrix = np.array(data["camera_matrix"])
                    self.dist_coeffs = np.array(data["dist_coeffs"])
                    # 尝试加载标定时的分辨率，如果没有则默认为None
                    self.calib_size = tuple(data.get("image_size", [])) if "image_size" in data else None
                    
                    self.is_calibrated = True
                    print(f"标定参数加载成功: {self.params_file}")
            except Exception as e:
                print(f"参数加载失败: {e}")
        else:
            print("未找到标定文件，请先运行标定程序。")

    def save_params(self, mtx, dist, image_size):
        """保存标定参数到JSON"""
        data = {
            "camera_matrix": mtx.tolist(),
            "dist_coeffs": dist.tolist(),
            "image_size": image_size # 保存标定时的分辨率 (w, h)
        }
        with open(self.params_file, 'w') as f:
            json.dump(data, f, indent=4)
        print(f"参数已保存至 {self.params_file}")
        self.load_params() 

    def undistort_image(self, frame):
        """对单帧图像进行除畸变（支持分辨率自动适配）"""
        if not self.is_calibrated:
            return frame 
        
        h, w = frame.shape[:2]
        
        # === 核心修复：检测分辨率是否匹配，不匹配则自动缩放矩阵 ===
        matrix_to_use = self.camera_matrix.copy()
        
        # 我们通过主点 (cx, cy) 来推测标定时的分辨率
        # cx 通常在图像宽度的一半附近
        calib_cx = self.camera_matrix[0, 2]
        calib_cy = self.camera_matrix[1, 2]
        
        # 计算缩放比例 (当前宽度 / 标定时的估算宽度)
        # 标定时的估算宽度 ≈ cx * 2
        estimated_calib_w = calib_cx * 2
        scale_factor = w / estimated_calib_w
        
        # 如果比例偏差超过 5%，说明分辨率变了，需要缩放矩阵
        if abs(scale_factor - 1.0) > 0.05:
            # print(f"检测到分辨率变化 (缩放 x{scale_factor:.2f})，正在自动调整内参...")
            matrix_to_use[0, 0] *= scale_factor # fx
            matrix_to_use[1, 1] *= scale_factor # fy
            matrix_to_use[0, 2] *= scale_factor # cx
            matrix_to_use[1, 2] *= scale_factor # cy
        
        # 计算新的优化相机矩阵
        # alpha=0: 裁剪掉所有黑色无效像素 (画面会放大)
        # alpha=1: 保留所有像素 (可能有黑边)
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(matrix_to_use, self.dist_coeffs, (w,h), 0, (w,h))
        
        # 执行除畸变
        dst = cv2.undistort(frame, matrix_to_use, self.dist_coeffs, None, newcameramtx)
        
        # 安全裁剪
        x, y, w_roi, h_roi = roi
        # 只有当 ROI 合理时才裁剪，防止出现 (26, 324) 这种异常
        if w_roi > w * 0.5 and h_roi > h * 0.5:
            dst = dst[y:y+h_roi, x:x+w_roi]
            # 可选：如果你希望输出图像尺寸和原图一致，可以 resize 回去
            dst = cv2.resize(dst, (w, h))
        else:
            # ROI 异常，说明畸变参数和当前画面极度不匹配，放弃裁剪
            # print("警告: 除畸变 ROI 异常，跳过裁剪步骤")
            pass
            
        return dst

    @staticmethod
    def run_calibration(images, pattern_size=(9, 6), square_size=20.0):
        # ... (这部分保持你原有的逻辑不变，或者复制下面的) ...
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
        objp = objp * square_size 

        objpoints = [] 
        imgpoints = [] 

        h, w = 0, 0
        
        print(f"开始标定，图片数量: {len(images)}")

        for img in images:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            h, w = gray.shape[:2]
            ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
            if ret:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)

        if len(objpoints) > 0:
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (w, h), None, None)
            return ret, mtx, dist, (w, h) # 返回图片尺寸
        else:
            return False, None, None, None

