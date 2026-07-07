#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from PyQt5.QtCore import QThread, Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QLabel, QSizePolicy, QVBoxLayout, QWidget


class CameraThread(QThread):
    """独立的摄像头读取线程，直接显示端侧视频流。"""

    change_pixmap_signal = pyqtSignal(np.ndarray)
    fall_alarm_signal = pyqtSignal(str, str)

    def __init__(self, source=0):
        super().__init__()
        self.source = source
        self.running = True
        self._cap = None

    def run(self):
        print(f"[Camera] 子线程开始拉取视频源: {self.source}")

        cap = self._open_capture()
        self._cap = cap

        if not cap.isOpened():
            print(f"[Camera] 无法打开视频源: {self.source}")
            return

        while self.running and cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                self.msleep(10)
                continue

            self.change_pixmap_signal.emit(frame)

        if cap is not None:
            cap.release()
            print("[Camera] 摄像头已安全释放")

    def _open_capture(self):
        if isinstance(self.source, int):
            return cv2.VideoCapture(self.source, cv2.CAP_DSHOW)
        return cv2.VideoCapture(self.source)

    def stop(self):
        self.running = False
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
        if self.isRunning() and not self.wait(2000):
            print("[Camera] 线程未在 2 秒内退出，强制终止")
            self.terminate()
            self.wait(500)


class CameraWidget(QWidget):
    """支持 USB 摄像头或 RTSP 流的视频监控组件。"""

    fall_alarm_signal = pyqtSignal(str, str)

    def __init__(self, source=0, parent=None):
        super().__init__(parent)
        self.source = source
        self.thread = None
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.video_label = QLabel("📷  摄像头未开启\nCamera Offline")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setScaledContents(True)
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.video_label.setMinimumHeight(180)
        self.video_label.setMinimumWidth(200)
        self.video_label.setStyleSheet("""
            background-color: #0a0f1e;
            color: #445566;
            border-radius: 6px;
            border: 1px dashed #1e3a5f;
            font-size: 14px;
            font-family: 'Consolas', 'Courier New', monospace;
        """)

        layout.addWidget(self.video_label)

    def start(self):
        if self.thread is None:
            self.thread = CameraThread(self.source)
            self.thread.change_pixmap_signal.connect(self.update_image)
            self.thread.fall_alarm_signal.connect(self.fall_alarm_signal.emit)
            self.thread.start()

    def stop(self):
        if self.thread is not None:
            self.thread.stop()
            self.thread = None
        self.video_label.setText("📷  摄像头已关闭")

    def update_image(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image.copy())
        self.video_label.setPixmap(pixmap)
