import sys
import cv2
from PySide6.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QImage, QPixmap
from .camera_core import CameraProcessor

class UndistortionApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("摄像头除畸变预览")
        self.setGeometry(100, 100, 1000, 500)

        # 加载核心处理模块
        self.processor = CameraProcessor()

        # UI: 左边原图，右边矫正图
        self.label_raw = QLabel("原图")
        self.label_undistort = QLabel("矫正后")
        for lbl in [self.label_raw, self.label_undistort]:
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("border: 1px solid gray;")
        
        layout = QVBoxLayout()
        layout.addWidget(self.label_raw)
        layout.addWidget(self.label_undistort)
        
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.cap = cv2.VideoCapture(0)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # 1. 显示原图
            self.display_image(frame, self.label_raw)
            
            # 2. 调用核心库进行除畸变
            undistorted_frame = self.processor.undistort_image(frame)
            
            # 3. 显示结果
            self.display_image(undistorted_frame, self.label_undistort)

    def display_image(self, frame, label_widget):
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        # 自适应缩放
        label_widget.setPixmap(QPixmap.fromImage(qt_image).scaled(
            label_widget.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def closeEvent(self, event):
        self.cap.release()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = UndistortionApp()
    window.show()
    sys.exit(app.exec())


