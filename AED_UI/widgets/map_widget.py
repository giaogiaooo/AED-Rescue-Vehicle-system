#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地图渲染组件 — 基于 PyQt QPainter 原生绘制

坐标系说明：
  ROS 世界坐标系:  Y 轴向上
  Qt 像素坐标系:   Y 轴向下
  paintEvent 通过 QTransform.scale(1, -1) 翻转 Y 轴，使视觉一致

坐标变换公式：
  world → pixel:
    px = (world_x - origin_x) / resolution
    py = (world_y - origin_y) / resolution

LaserScan → 地图坐标:
  laser_x = range * cos(angle)
  laser_y = range * sin(angle)
  map_x = robot_x + laser_x * cos(robot_yaw) - laser_y * sin(robot_yaw)
  map_y = robot_y + laser_x * sin(robot_yaw) + laser_y * cos(robot_yaw)

Nav2 Path → 像素:
  px = (pose.x - origin_x) / resolution
  py = (pose.y - origin_y) / resolution
"""

import math
import numpy as np
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QColor, QImage, QPixmap, QPen, QTransform, QFont
from PyQt5.QtCore import Qt, QPointF


class MapWidget(QWidget):
    """基于 PyQt 原生绘制的 OccupancyGrid + LaserScan + Path 地图组件"""

    def __init__(self, parent=None):
        super().__init__(parent)

        # 地图底图
        self.map_img = None       # QPixmap
        self.map_info = None      # OccupancyGrid.info 字典

        # 机器人位姿（来自 AMCL，全局 map 坐标）
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0      # 弧度

        # 激光点云（世界坐标列表）
        self.laser_points = []    # [(map_x, map_y), ...]

        # Nav2 路径
        self.path_points = []     # [(x, y), ...]

        # 视图控制
        self.scale_factor = 1.0
        self.offset = QPointF(0, 0)
        self.last_mouse_pos = None

        # 自动居中标志（首次收到地图时自动适配）
        self._auto_fit_done = False

        # 调试标志
        self._debug_printed = False

        self.init_ui()

    # ======================== 初始化 ========================

    def init_ui(self):
        self.setStyleSheet(
            "background-color: #0a0f1e; border: none; border-radius: 4px;"
        )
        self.setMouseTracking(True)

    # ======================== 更新地图底图 ========================

    def update_map(self, map_msg):
        """解析 roslibpy 传来的 OccupancyGrid 字典并生成 QPixmap"""
        try:
            info = map_msg['info']
            width = info['width']
            height = info['height']
            resolution = info['resolution']
            self.map_info = info

            # 验证数据长度
            data = np.array(map_msg['data'], dtype=np.int16)
            expected_length = width * height
            if len(data) != expected_length:
                print(
                    f"[MapWidget] ⚠ 地图数据长度 ({len(data)}) "
                    f"与尺寸 ({width}x{height}={expected_length}) 不匹配，跳过渲染！"
                )
                return

            # 调试输出（首次）
            if not self._debug_printed:
                origin = info['origin']['position']
                print(
                    f"[MapWidget] 📐 地图已加载: {width}x{height} cells, "
                    f"分辨率={resolution:.4f} m/cell, "
                    f"origin=({origin['x']:.3f}, {origin['y']:.3f}), "
                    f"覆盖范围: x=[{origin['x']:.2f}, {origin['x'] + width * resolution:.2f}], "
                    f"y=[{origin['y']:.2f}, {origin['y'] + height * resolution:.2f}]"
                )
                self._debug_printed = True

            # 重组为 2D 矩阵（height 行, width 列）
            data = data.reshape((height, width))

            # RGB 图像矩阵
            img_array = np.zeros((height, width, 3), dtype=np.uint8)

            # 颜色映射
            img_array[data == -1] = [200, 200, 200]        # 未知 → 浅灰
            img_array[data == 0] = [255, 255, 255]          # 空闲 → 白
            img_array[data == 100] = [0, 0, 0]              # 障碍 → 黑

            # 概率值 1~99 → 灰度渐变
            mask_prob = (data > 0) & (data < 100)
            if np.any(mask_prob):
                gray_vals = (255 - (data[mask_prob] * 2.55).astype(np.uint8))
                img_array[mask_prob, 0] = gray_vals
                img_array[mask_prob, 1] = gray_vals
                img_array[mask_prob, 2] = gray_vals

            # 生成 QPixmap
            qimg = QImage(
                img_array.data, width, height, width * 3, QImage.Format_RGB888
            )
            self.map_img = QPixmap.fromImage(qimg)

            # 首次收到地图时自动适配缩放和居中
            if not self._auto_fit_done:
                self._fit_map_to_view()
                self._auto_fit_done = True

            self.update()

        except Exception as e:
            print(f"[MapWidget] ❌ 地图更新异常: {e}")

    # ======================== 自动适配视图 ========================

    def _fit_map_to_view(self):
        """根据 widget 大小和地图尺寸，自动计算缩放因子和居中偏移"""
        if self.map_info is None:
            return

        map_w = self.map_info['width']    # 像素宽
        map_h = self.map_info['height']   # 像素高

        view_w = self.width()
        view_h = self.height()

        if view_w <= 0 or view_h <= 0 or map_w <= 0 or map_h <= 0:
            return

        # 留 8% 边距，让地图不要贴边
        margin = 0.92
        scale_x = (view_w / map_w) * margin
        scale_y = (view_h / map_h) * margin
        self.scale_factor = min(scale_x, scale_y)

        # 偏移清零，地图以 widget 中心为基准
        self.offset = QPointF(0, 0)

        print(
            f"[MapWidget] 📐 自动适配: scale={self.scale_factor:.3f}, "
            f"map={map_w}x{map_h}px, view={view_w}x{view_h}px"
        )

    def resizeEvent(self, event):
        """窗口大小变化时重新适配"""
        super().resizeEvent(event)
        if self.map_info is not None and self._auto_fit_done:
            self._fit_map_to_view()

    def update_robot_pose(self, x, y, yaw_deg):
        """从 AMCL 接收全局位姿更新"""
        self.robot_x = x
        self.robot_y = y
        self.robot_yaw = math.radians(yaw_deg)
        self.update()

    # ======================== 更新激光点云 ========================

    def update_laser_scan(self, scan_msg):
        """
        将 LaserScan 消息转换为世界坐标系下的点云。
        变换链: base_laser → robot pose → map coordinate
        """
        if self.map_info is None:
            return  # 地图尚未加载，无法转换

        try:
            ranges = scan_msg.get('ranges', [])
            angle_min = scan_msg.get('angle_min', 0.0)
            angle_increment = scan_msg.get('angle_increment', 0.0)
            angle_max = scan_msg.get('angle_max', 0.0)
            range_min = scan_msg.get('range_min', 0.0)
            range_max = scan_msg.get('range_max', 10.0)

            if not ranges or not self.map_info:
                self.laser_points = []
                self.update()
                return

            points = []
            robot_yaw = self.robot_yaw
            cos_yaw = math.cos(robot_yaw)
            sin_yaw = math.sin(robot_yaw)

            for i, r in enumerate(ranges):
                if r is None or r < range_min or r > range_max:
                    continue

                # 步 1: 激光坐标系下的点
                angle = angle_min + i * angle_increment
                laser_x = r * math.cos(angle)
                laser_y = r * math.sin(angle)

                # 步 2: 旋转平移到 map 坐标系
                # map = robot + R(θ) * laser
                map_x = self.robot_x + laser_x * cos_yaw - laser_y * sin_yaw
                map_y = self.robot_y + laser_x * sin_yaw + laser_y * cos_yaw

                points.append((map_x, map_y))

            self.laser_points = points

            # 调试输出（每 60 帧输出一次，约 6Hz 时 ≈10秒一次）
            if not hasattr(self, '_laser_debug_count'):
                self._laser_debug_count = 0
            self._laser_debug_count += 1
            if self._laser_debug_count % 60 == 0:
                print(
                    f"[MapWidget] 🔴 激光点云: 原始 {len(ranges)} 点, "
                    f"有效 {len(points)} 点, "
                    f"机器人位姿=({self.robot_x:.2f}, {self.robot_y:.2f}), "
                    f"yaw={math.degrees(self.robot_yaw):.1f}°"
                )

            self.update()

        except Exception as e:
            print(f"[MapWidget] ⚠ 激光扫描更新异常: {e}")

    # ======================== 更新 Nav2 路径 ========================

    def update_path(self, path_msg):
        """解析 nav_msgs/Path，提取路径点坐标（世界坐标系）"""
        try:
            poses = path_msg.get('poses', [])
            points = []

            for pose_entry in poses:
                pos = pose_entry.get('pose', {}).get('position', {})
                points.append((pos.get('x', 0.0), pos.get('y', 0.0)))

            self.path_points = points

            # 调试输出
            if not hasattr(self, '_path_debug_count'):
                self._path_debug_count = 0
            self._path_debug_count += 1
            if self._path_debug_count % 10 == 0:
                print(
                    f"[MapWidget] 🛤️ 路径已更新: {len(points)} 个路径点"
                )

            self.update()

        except Exception as e:
            print(f"[MapWidget] ⚠ 路径更新异常: {e}")

    # ======================== 坐标转换工具 ========================

    def _world_to_pixel(self, wx, wy):
        """世界坐标 → 像素坐标（相对于 OccupancyGrid origin）"""
        if self.map_info is None:
            return wx, wy  # fallback
        origin_x = self.map_info['origin']['position']['x']
        origin_y = self.map_info['origin']['position']['y']
        resolution = self.map_info['resolution']
        px = (wx - origin_x) / resolution
        py = (wy - origin_y) / resolution
        return px, py

    # ======================== PaintEvent — 核心渲染 ========================

    def paintEvent(self, event):
        """所有绘制在此完成"""

        # 无地图时的占位提示
        if self.map_img is None or self.map_info is None:
            painter = QPainter(self)
            painter.setPen(QColor("#445566"))
            painter.setFont(QFont("Microsoft YaHei", 13))
            painter.drawText(self.rect(), Qt.AlignCenter, "等待接收 /map 地图数据...")
            painter.end()
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)

        # 获取地图参数
        resolution = self.map_info['resolution']
        origin_x = self.map_info['origin']['position']['x']
        origin_y = self.map_info['origin']['position']['y']
        map_width = self.map_info['width']
        map_height = self.map_info['height']

        # ========== 坐标系设置 ==========
        center_x = self.width() / 2
        center_y = self.height() / 2

        transform = QTransform()
        transform.translate(center_x + self.offset.x(), center_y + self.offset.y())
        transform.scale(self.scale_factor, self.scale_factor)
        # 关键：翻转 Y 轴，使 ROS (Y↑) → Qt (Y↓)
        transform.scale(1, -1)
        painter.setTransform(transform)

        # ========== 1. 绘制地图底图 ==========
        # 地图从 (0, 0) 开始绘制，origin 偏移已通过世界→像素公式处理
        painter.drawPixmap(0, 0, self.map_img)

        # ========== 2. 绘制 Nav2 全局路径（蓝色折线） ==========
        if self.path_points:
            pen = QPen(QColor(0, 140, 255, 200), 2.5 / self.scale_factor)
            # 虚线样式更醒目
            pen.setStyle(Qt.DashLine)
            painter.setPen(pen)

            prev_px, prev_py = None, None
            for wx, wy in self.path_points:
                px, py = self._world_to_pixel(wx, wy)
                if prev_px is not None:
                    painter.drawLine(
                        QPointF(prev_px, prev_py), QPointF(px, py)
                    )
                prev_px, prev_py = px, py

        # ========== 3. 绘制激光点云（红色散点） ==========
        if self.laser_points:
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(255, 50, 50, 180))
            dot_radius = max(2.0, 3.0 / self.scale_factor)

            for wx, wy in self.laser_points:
                px, py = self._world_to_pixel(wx, wy)
                painter.drawEllipse(QPointF(px, py), dot_radius, dot_radius)

        # ========== 4. 绘制机器人（蓝色圆形 + 黄色方向箭头） ==========
        rx_pixel, ry_pixel = self._world_to_pixel(self.robot_x, self.robot_y)

        painter.save()
        painter.translate(rx_pixel, ry_pixel)
        painter.rotate(math.degrees(self.robot_yaw))

        # 蓝色圆形机器人图标
        painter.setBrush(QColor(0, 140, 255, 200))
        painter.setPen(Qt.NoPen)
        radius = 8 / self.scale_factor
        painter.drawEllipse(QPointF(0, 0), radius, radius)

        # 黄色方向箭头
        painter.setPen(QPen(Qt.yellow, 2 / self.scale_factor))
        painter.drawLine(QPointF(0, 0), QPointF(radius * 1.5, 0))

        painter.restore()

        # ========== 5. 图例标注（屏幕空间，不受 Y 翻转影响） ==========
        painter.resetTransform()

        # 右上角图例
        legend_x = self.width() - 160
        legend_y = 12
        font = QFont("Consolas", 10)
        painter.setFont(font)

        # 背景
        painter.setBrush(QColor(10, 15, 30, 200))
        painter.setPen(QPen(QColor("#1e3a5f"), 1))
        painter.drawRoundedRect(legend_x - 8, legend_y - 4, 152, 78, 6, 6)

        # 路径图例（蓝色虚线）
        painter.setPen(QPen(QColor(0, 140, 255), 2))
        painter.drawLine(legend_x, legend_y + 11, legend_x + 24, legend_y + 11)
        painter.setPen(QPen(QColor("#aabbcc")))
        painter.drawText(legend_x + 30, legend_y + 15, "Nav2 路径")

        # 激光图例（红点）
        painter.setBrush(QColor(255, 50, 50))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(QPointF(legend_x + 12, legend_y + 32), 4, 4)
        painter.setPen(QPen(QColor("#aabbcc")))
        painter.drawText(legend_x + 30, legend_y + 37, "激光点云")

        # 机器人图例（蓝圆）
        painter.setBrush(QColor(0, 140, 255))
        painter.drawEllipse(QPointF(legend_x + 12, legend_y + 54), 5, 5)
        painter.setPen(QPen(QColor("#aabbcc")))
        painter.drawText(legend_x + 30, legend_y + 59, "机器人")

        painter.end()

    # ======================== 交互：滚轮缩放 ========================

    def wheelEvent(self, event):
        angle = event.angleDelta().y()
        if angle > 0:
            self.scale_factor *= 1.1
        else:
            self.scale_factor *= 0.9
        # 限制缩放范围
        self.scale_factor = max(0.1, min(10.0, self.scale_factor))
        self.update()

    # ======================== 交互：鼠标拖动平移 ========================

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self.last_mouse_pos is not None:
            delta = event.pos() - self.last_mouse_pos
            self.offset += QPointF(delta.x(), delta.y())
            self.last_mouse_pos = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        self.last_mouse_pos = None
