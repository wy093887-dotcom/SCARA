import sys
import cv2
import time

# ================= 路径修复 =================
# 确保能引用到同目录下的 camera_core
from PySide6.QtWidgets import (QApplication, QMainWindow, QLabel, QVBoxLayout, 
                               QWidget, QPushButton, QHBoxLayout, QMessageBox, QProgressBar)
from PySide6.QtCore import QTimer, Qt, QThread, Signal
from PySide6.QtGui import QImage, QPixmap
from .camera_core import CameraProcessor

# =========================================================
#  后台计算线程 (防止界面卡死)
# =========================================================
class CalibrationWorker(QThread):
    # 信号：(是否成功, 内参矩阵, 畸变系数, 图片尺寸, 错误信息/误差值)
    finished_signal = Signal(bool, object, object, object, object)

    def __init__(self, processor, images, pattern_size, square_size):
        super().__init__()
        self.processor = processor
        self.images = images
        self.pattern_size = pattern_size
        self.square_size = square_size

    def run(self):
        try:
            # 【关键修改】接收 5 个返回值，匹配最新的 camera_core.py
            # 如果你的 camera_core 只返回 4 个值，这里会由 except 捕获，但我们假设是兼容高精度版的 5 个值
            result = self.processor.run_calibration(
                self.images, 
                pattern_size=self.pattern_size,
                square_size=self.square_size
            )
            
            # 兼容性处理：判断返回值的数量
            if len(result) == 5:
                success, mtx, dist, img_size, error = result
            elif len(result) == 4:
                success, mtx, dist, img_size = result
                error = 0.0 # 旧版本没有误差计算
            elif len(result) == 3:
                success, mtx, dist = result
                img_size = (640, 480) # 默认
                error = 0.0
            else:
                raise ValueError(f"Unexpected return values: {len(result)}")

            self.finished_signal.emit(success, mtx, dist, img_size, error)
        except Exception as e:
            # 发送失败信号
            self.finished_signal.emit(False, None, None, None, str(e))

# =========================================================
#  主界面
# =========================================================
class CalibrationApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("摄像头标定工具 (兼容修复版)")
        self.setGeometry(100, 100, 900, 750)

        self.processor = CameraProcessor()
        self.captured_images = []
        self.pattern_size = (10, 7) # 棋盘格角点数 (列-1, 行-1)
        self.square_size = 22.0    # 棋盘格边长 (mm)
        self.worker = None         # 线程对象占位
        
        # UI 组件
        self.image_label = QLabel("摄像头画面初始化中...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(640, 480)
        self.image_label.setStyleSheet("background-color: #000;")
        
        # 进度条
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 0) # 忙碌模式
        self.progress_bar.hide()
        self.status_label = QLabel("就绪")
        self.status_label.setAlignment(Qt.AlignCenter)
        
        # 按钮
        self.btn_capture = QPushButton(f"采集图片 (当前: 0)")
        self.btn_calibrate = QPushButton("开始计算")
        self.btn_clear = QPushButton("清空")
        
        # 样式
        self.btn_capture.setStyleSheet("padding: 12px; font-weight: bold; font-size: 14px;")
        self.btn_calibrate.setStyleSheet("padding: 12px; background-color: #d1c4e9; font-weight: bold;")
        self.btn_clear.setStyleSheet("padding: 12px;")
        
        self.btn_capture.clicked.connect(self.capture_frame)
        self.btn_calibrate.clicked.connect(self.start_calibration)
        self.btn_clear.clicked.connect(self.clear_data)

        # 布局
        control_layout = QHBoxLayout()
        control_layout.addWidget(self.btn_capture)
        control_layout.addWidget(self.btn_calibrate)
        control_layout.addWidget(self.btn_clear)

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.image_label, stretch=1)
        main_layout.addWidget(self.status_label)
        main_layout.addWidget(self.progress_bar)
        main_layout.addLayout(control_layout)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        # 摄像头初始化 (使用 DSHOW)
        self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

    def update_frame(self):
        ret, self.current_frame = self.cap.read()
        if ret:
            display_frame = self.current_frame.copy()
            # 实时画出角点辅助预览
            gray = cv2.cvtColor(display_frame, cv2.COLOR_BGR2GRAY)
            try:
                ret_corn, corners = cv2.findChessboardCorners(gray, self.pattern_size, 
                                                            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK)
                if ret_corn:
                    cv2.drawChessboardCorners(display_frame, self.pattern_size, corners, ret_corn)
                    cv2.rectangle(display_frame, (0,0), (display_frame.shape[1], display_frame.shape[0]), (0,255,0), 3)
            except:
                pass
            
            self.display_image(display_frame)

    def display_image(self, frame):
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        qt_image = QImage(rgb_image.data, w, h, ch * w, QImage.Format_RGB888)
        self.image_label.setPixmap(QPixmap.fromImage(qt_image).scaled(
            self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def capture_frame(self):
        if hasattr(self, 'current_frame') and self.current_frame is not None:
            self.captured_images.append(self.current_frame.copy())
            count = len(self.captured_images)
            self.btn_capture.setText(f"采集图片 (当前: {count})")
            self.status_label.setText(f"已采集 {count} 张图片")
            
            self.image_label.setStyleSheet("background-color: #fff;")
            QApplication.processEvents()
            QThread.msleep(50)
            self.image_label.setStyleSheet("background-color: #000;")

    def start_calibration(self):
        if len(self.captured_images) < 5:
            QMessageBox.warning(self, "数量不足", "至少需要 5 张图片，建议采集 15 张以上。")
            return

        # 冻结界面
        self.timer.stop() 
        self.btn_capture.setEnabled(False)
        self.btn_calibrate.setEnabled(False)
        self.btn_clear.setEnabled(False)
        
        self.progress_bar.show()
        self.status_label.setText("正在计算 (可能需要几十秒)...")
        self.status_label.setStyleSheet("color: blue; font-weight: bold;")

        # 启动后台线程
        self.worker = CalibrationWorker(
            self.processor, 
            self.captured_images, 
            self.pattern_size, 
            self.square_size
        )
        self.worker.finished_signal.connect(self.on_calibration_finished)
        self.worker.start()

    def on_calibration_finished(self, success, mtx, dist, img_size, error):
        """计算完成回调"""
        self.progress_bar.hide()
        
        if success:
            # 保存参数 (这里传入 img_size)
            self.processor.save_params(mtx, dist, img_size)
            
            msg = f"标定成功！\n分辨率: {img_size}\n误差: {error:.4f}"
            self.status_label.setText(f"标定成功 (误差 {error:.4f})")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            QMessageBox.information(self, "结果", msg)
        else:
            self.status_label.setText("标定失败")
            self.status_label.setStyleSheet("color: red;")
            QMessageBox.critical(self, "失败", f"标定失败: {error}")

        # 恢复界面
        self.btn_capture.setEnabled(True)
        self.btn_calibrate.setEnabled(True)
        self.btn_clear.setEnabled(True)
        self.captured_images = []
        self.btn_capture.setText("采集图片 (当前: 0)")
        
        self.timer.start()

    def clear_data(self):
        self.captured_images = []
        self.btn_capture.setText("采集图片 (当前: 0)")
        self.status_label.setText("数据已清空")

    def closeEvent(self, event):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        # 安全停止线程
        if hasattr(self, 'worker') and self.worker is not None:
            if self.worker.isRunning():
                self.worker.terminate()
                self.worker.wait()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CalibrationApp()
    window.show()
    sys.exit(app.exec())






