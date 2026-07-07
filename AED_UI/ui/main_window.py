#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QPushButton, QListWidget, QMessageBox, QFrame, QLineEdit)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QPixmap
from PyQt5.QtWidgets import QGraphicsDropShadowEffect

from widgets.map_widget import MapWidget
from widgets.camera_widget import CameraWidget
from ui.alarm_window import AlarmHistoryWindow
from database.db import DatabaseManager

# 【关键修改】：这里导入的是新的 ros_client，而不是旧的 ros_node
from ros.ros_client import ROS2ClientThread

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.db = DatabaseManager()
        self.ros_thread = None
        
        # 报警状态
        self.is_alarm_active = False
        self.normal_style = ""
        self.alarm_style = ""
        
        # 报警自动解除定时器（10秒无新红色报警后恢复）
        self.alarm_dismiss_timer = QTimer()
        self.alarm_dismiss_timer.setSingleShot(True)
        self.alarm_dismiss_timer.timeout.connect(self.dismiss_alarm_visual)
        
        self.init_ui()
        self.apply_styles()
        # 启动摄像头
        self.camera_widget.start()

    def init_ui(self):
        self.setWindowTitle("智能AED救援车监控平台 v2.0")
        self.resize(1280, 860)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(12, 12, 12, 12)

        # ===== 顶部标题与连接栏 =====
        top_frame = QFrame()
        top_frame.setObjectName("TopBar")
        top_layout = QHBoxLayout(top_frame)
        top_layout.setContentsMargins(16, 10, 16, 10)
        
        # 左侧：Logo + 标题
        header_left = QHBoxLayout()
        self.status_dot = QLabel()
        self.status_dot.setObjectName("StatusDot")
        self.status_dot.setFixedSize(28, 28)
        self.status_dot.setScaledContents(True)
        # 加载 logo 图片
        logo_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "logo.jpg")
        if os.path.exists(logo_path):
            self.logo_pixmap = QPixmap(logo_path)
            self.status_dot.setPixmap(self.logo_pixmap)
        else:
            self.status_dot.setText("●")
            self.status_dot.setStyleSheet("color: #ff4444; font-size: 18px;")
        # 发光效果用于状态指示
        self.logo_glow = QGraphicsDropShadowEffect()
        self.logo_glow.setBlurRadius(12)
        self.logo_glow.setOffset(0, 0)
        self.logo_glow.setColor(Qt.red)
        self.status_dot.setGraphicsEffect(self.logo_glow)
        header_left.addWidget(self.status_dot)
        
        title_label = QLabel("智能AED救援车监控平台")
        title_label.setObjectName("TitleLabel")
        header_left.addWidget(title_label)
        top_layout.addLayout(header_left)
        
        top_layout.addStretch()
        
        # 右侧：IP输入 + 连接按钮
        lbl_ip = QLabel("ROS2 IP")
        lbl_ip.setObjectName("TopLabel")
        top_layout.addWidget(lbl_ip)
        
        self.txt_ip = QLineEdit("192.168.1.103")
        self.txt_ip.setPlaceholderText("输入机器人 IP")
        self.txt_ip.setFixedWidth(160)
        self.txt_ip.setObjectName("AddrInput")
        top_layout.addWidget(self.txt_ip)
        
        self.btn_connect = QPushButton("⚡ 连接 ROS")
        self.btn_connect.setObjectName("ConnectBtn")
        self.btn_connect.setFixedHeight(34)
        self.btn_connect.clicked.connect(self.toggle_connection)
        top_layout.addWidget(self.btn_connect)
        
        self.lbl_conn_status = QLabel("●  未连接")
        self.lbl_conn_status.setObjectName("ConnStatus")
        top_layout.addWidget(self.lbl_conn_status)
        
        main_layout.addWidget(top_frame)

        # ===== 红色警报横幅（默认隐藏）=====
        self.alarm_banner = QLabel("⚠️ 红色警报 — 机器人检测到危险障碍物，正在紧急响应！")
        self.alarm_banner.setObjectName("AlarmBanner")
        self.alarm_banner.setVisible(False)
        self.alarm_banner.setAlignment(Qt.AlignCenter)
        self.alarm_banner.setFixedHeight(50)
        main_layout.addWidget(self.alarm_banner)

        # ===== 中间主体 (左中右三栏布局) =====
        body_layout = QHBoxLayout()
        body_layout.setSpacing(10)
        
        # [左侧] 摄像头
        cam_card = QFrame()
        cam_card.setObjectName("CamCard")
        cam_layout = QVBoxLayout(cam_card)
        cam_layout.setContentsMargins(12, 10, 12, 12)
        
        cam_header = QHBoxLayout()
        cam_icon = QLabel("📷")
        cam_icon.setObjectName("CardIcon")
        cam_header.addWidget(cam_icon)
        cam_header.addWidget(QLabel("现场画面"))
        cam_header.addStretch()
        cam_layout.addLayout(cam_header)
        
        self.camera_widget = CameraWidget(source="rtsp://admin:@192.168.1.86")
        self.camera_widget.fall_alarm_signal.connect(self.handle_fall_alarm)
        cam_layout.addWidget(self.camera_widget)
        body_layout.addWidget(cam_card, stretch=5)
        
        # [中间] 地图（主区域）
        map_card = QFrame()
        map_card.setObjectName("MapCard")
        map_layout = QVBoxLayout(map_card)
        map_layout.setContentsMargins(12, 10, 12, 12)
        
        map_header = QHBoxLayout()
        map_icon = QLabel("🗺️")
        map_icon.setObjectName("CardIcon")
        map_header.addWidget(map_icon)
        map_header.addWidget(QLabel("实时地图与轨迹"))
        map_header.addStretch()
        self.lbl_map_coord = QLabel("—")
        self.lbl_map_coord.setObjectName("SubInfo")
        map_header.addWidget(self.lbl_map_coord)
        map_layout.addLayout(map_header)
        
        self.map_widget = MapWidget()
        self.map_widget.setObjectName("MapArea")
        map_layout.addWidget(self.map_widget, stretch=1)
        body_layout.addWidget(map_card, stretch=3)
        
        # [右侧] 机器人状态
        status_card = QFrame()
        status_card.setObjectName("StatusCard")
        status_layout = QVBoxLayout(status_card)
        status_layout.setContentsMargins(12, 10, 12, 12)
        
        status_header = QHBoxLayout()
        status_icon = QLabel("📊")
        status_icon.setObjectName("CardIcon")
        status_header.addWidget(status_icon)
        status_header.addWidget(QLabel("机器人实时状态"))
        status_header.addStretch()
        status_layout.addLayout(status_header)
        
        # 数据行
        self.lbl_x = self._make_data_row("X 坐标", "0.00 m")
        self.lbl_y = self._make_data_row("Y 坐标", "0.00 m")
        self.lbl_yaw = self._make_data_row("偏航角", "0.00 °")
        self.lbl_vlin = self._make_data_row("线速度", "0.00 m/s")
        self.lbl_vang = self._make_data_row("角速度", "0.00 rad/s")
        self.lbl_nav_status = self._make_data_row("导航状态", "等待中")
        
        for widget in [self.lbl_x, self.lbl_y, self.lbl_yaw, self.lbl_vlin, self.lbl_vang, self.lbl_nav_status]:
            status_layout.addWidget(widget)
        status_layout.addStretch()
        body_layout.addWidget(status_card, stretch=2)

        main_layout.addLayout(body_layout, stretch=3)

        # ===== 下方报警区域 =====
        alarm_card = QFrame()
        alarm_card.setObjectName("AlarmCard")
        alarm_layout = QVBoxLayout(alarm_card)
        alarm_layout.setContentsMargins(12, 10, 12, 12)
        
        alarm_top = QHBoxLayout()
        alarm_icon = QLabel("🚨")
        alarm_icon.setObjectName("CardIcon")
        alarm_top.addWidget(alarm_icon)
        alarm_top.addWidget(QLabel("实时报警信息"))
        alarm_top.addStretch()
        
        self.btn_history = QPushButton("📋 历史报警记录")
        self.btn_history.setObjectName("HistoryBtn")
        self.btn_history.setFixedHeight(30)
        self.btn_history.clicked.connect(self.show_history_window)
        alarm_top.addWidget(self.btn_history)
        alarm_layout.addLayout(alarm_top)
        
        self.alarm_list = QListWidget()
        self.alarm_list.setObjectName("AlarmList")
        self.alarm_list.setMaximumHeight(130)
        self.alarm_list.setMinimumHeight(100)
        alarm_layout.addWidget(self.alarm_list)
        
        main_layout.addWidget(alarm_card)

    def _make_data_row(self, label_text, value_text):
        """创建统一格式的数据行：标签 + 数值"""
        frame = QFrame()
        frame.setObjectName("DataRow")
        row = QHBoxLayout(frame)
        row.setContentsMargins(6, 4, 6, 4)
        lbl = QLabel(label_text)
        lbl.setObjectName("DataLabel")
        val = QLabel(value_text)
        val.setObjectName("DataValue")
        row.addWidget(lbl)
        row.addStretch()
        row.addWidget(val)
        frame.val_label = val
        return frame

    def _update_data_row(self, row, value):
        """更新数据行中的数值"""
        row.val_label.setText(value)

    def apply_styles(self):
        # ---- 普通模式（深蓝科技风） ----
        self.normal_style = """
        /* ===== 全局背景 ===== */
        QMainWindow {
            background-color: #0a0e1a;
        }
        
        /* ===== 顶部栏 ===== */
        #TopBar {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                stop:0 #0d1328, stop:0.5 #111d3a, stop:1 #0d1328);
            border: 1px solid #1e3a5f;
            border-radius: 10px;
        }
        #TitleLabel {
            font-size: 22px;
            font-weight: bold;
            color: #00e5ff;
            padding: 0px 6px;
            font-family: "Microsoft YaHei", "Segoe UI", sans-serif;
        }
        #StatusDot {
            font-size: 18px;
        }
        #TopLabel {
            color: #8899bb;
            font-size: 13px;
            font-weight: bold;
            padding-right: 4px;
        }
        #AddrInput {
            background-color: #0d1628;
            color: #00e5ff;
            border: 1px solid #1e4a6e;
            border-radius: 6px;
            padding: 6px 10px;
            font-size: 13px;
            font-family: "Consolas", "Courier New", monospace;
        }
        #AddrInput:focus {
            border: 1px solid #00e5ff;
        }
        #ConnectBtn {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                stop:0 #006699, stop:1 #0088cc);
            color: #ffffff;
            border: 1px solid #00aaff;
            border-radius: 6px;
            padding: 6px 18px;
            font-weight: bold;
            font-size: 13px;
        }
        #ConnectBtn:hover {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                stop:0 #0088cc, stop:1 #00aaff);
            border: 1px solid #00ddff;
        }
        #ConnStatus {
            color: #ff4444;
            font-weight: bold;
            font-size: 13px;
            padding-left: 8px;
        }
        
        /* ===== 卡片通用 ===== */
        #MapCard, #CamCard, #StatusCard, #AlarmCard {
            background-color: rgba(13, 20, 40, 0.85);
            border: 1px solid #1e3a5f;
            border-radius: 10px;
        }
        #MapCard QLabel, #CamCard QLabel, #StatusCard QLabel, #ControlCard QLabel, #AlarmCard QLabel {
            color: #c8d6f0;
            font-size: 13px;
            font-family: "Microsoft YaHei", "Segoe UI", sans-serif;
        }
        #CardIcon {
            font-size: 16px;
            padding-right: 4px;
        }
        #SubInfo {
            color: #556688;
            font-size: 11px;
        }
        
        /* ===== 数据行 ===== */
        #DataRow {
            background-color: rgba(16, 30, 56, 0.6);
            border: 1px solid #162d50;
            border-radius: 6px;
            margin: 1px 0px;
        }
        #DataLabel {
            color: #7799cc;
            font-size: 12px;
            font-weight: bold;
        }
        #DataValue {
            color: #00e5ff;
            font-size: 13px;
            font-weight: bold;
            font-family: "Consolas", "Courier New", monospace;
        }
        
        /* ===== 报警列表 ===== */
        #AlarmList {
            background-color: #0a0f1e;
            color: #c8d6f0;
            border: 1px solid #1e3a5f;
            border-radius: 6px;
            font-size: 12px;
            font-family: "Consolas", "Courier New", monospace;
        }
        #AlarmList::item {
            padding: 4px 8px;
            border-bottom: 1px solid #111d38;
        }
        #AlarmList::item:selected {
            background-color: #112244;
            color: #00e5ff;
        }
        
        /* ===== 历史按钮 ===== */
        #HistoryBtn {
            background-color: #162040;
            color: #00e5ff;
            border: 1px solid #1e3a5f;
            border-radius: 6px;
            padding: 4px 14px;
            font-size: 12px;
            font-weight: bold;
        }
        #HistoryBtn:hover {
            background-color: #1e3058;
            border: 1px solid #00ccff;
        }
        
        /* ===== 滚动条 ===== */
        QScrollBar:vertical {
            background: #0a0f1e;
            width: 6px;
            border-radius: 3px;
        }
        QScrollBar::handle:vertical {
            background: #1e3a5f;
            border-radius: 3px;
            min-height: 30px;
        }
        QScrollBar::handle:vertical:hover {
            background: #00e5ff;
        }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
            height: 0px;
        }
        """

        # ---- 红色警报模式（全界面泛红） ----
        self.alarm_style = """
        /* ===== 全局背景 ===== */
        QMainWindow {
            background-color: #1a0808;
        }
        
        /* ===== 顶部栏 ===== */
        #TopBar {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                stop:0 #2a0d0d, stop:0.5 #3d1118, stop:1 #2a0d0d);
            border: 2px solid #aa2222;
            border-radius: 10px;
        }
        #TitleLabel {
            font-size: 22px;
            font-weight: bold;
            color: #ff4444;
            padding: 0px 6px;
            font-family: "Microsoft YaHei", "Segoe UI", sans-serif;
        }
        #StatusDot {
            font-size: 18px;
        }
        #TopLabel {
            color: #cc9988;
            font-size: 13px;
            font-weight: bold;
            padding-right: 4px;
        }
        #AddrInput {
            background-color: #2a0d16;
            color: #ff6644;
            border: 1px solid #6e1e3a;
            border-radius: 6px;
            padding: 6px 10px;
            font-size: 13px;
            font-family: "Consolas", "Courier New", monospace;
        }
        #AddrInput:focus {
            border: 1px solid #ff4444;
        }
        #ConnectBtn {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                stop:0 #992200, stop:1 #cc3300);
            color: #ffffff;
            border: 1px solid #ff4400;
            border-radius: 6px;
            padding: 6px 18px;
            font-weight: bold;
            font-size: 13px;
        }
        #ConnectBtn:hover {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                stop:0 #cc3300, stop:1 #ff4400);
            border: 1px solid #ff6622;
        }
        #ConnStatus {
            color: #ff4444;
            font-weight: bold;
            font-size: 13px;
            padding-left: 8px;
        }
        
        /* ===== 卡片通用 ===== */
        #MapCard, #CamCard, #StatusCard, #AlarmCard {
            background-color: rgba(30, 12, 12, 0.92);
            border: 2px solid #aa2222;
            border-radius: 10px;
        }
        #MapCard QLabel, #CamCard QLabel, #StatusCard QLabel, #ControlCard QLabel, #AlarmCard QLabel {
            color: #f0c8c0;
            font-size: 13px;
            font-family: "Microsoft YaHei", "Segoe UI", sans-serif;
        }
        #CardIcon {
            font-size: 16px;
            padding-right: 4px;
        }
        #SubInfo {
            color: #886655;
            font-size: 11px;
        }
        
        /* ===== 数据行 ===== */
        #DataRow {
            background-color: rgba(40, 14, 14, 0.8);
            border: 1px solid #661111;
            border-radius: 6px;
            margin: 1px 0px;
        }
        #DataLabel {
            color: #cc9988;
            font-size: 12px;
            font-weight: bold;
        }
        #DataValue {
            color: #ff5533;
            font-size: 13px;
            font-weight: bold;
            font-family: "Consolas", "Courier New", monospace;
        }
        
        /* ===== 报警列表 ===== */
        #AlarmList {
            background-color: #1a0a08;
            color: #f0c8c0;
            border: 1px solid #aa2222;
            border-radius: 6px;
            font-size: 12px;
            font-family: "Consolas", "Courier New", monospace;
        }
        #AlarmList::item {
            padding: 4px 8px;
            border-bottom: 1px solid #381111;
        }
        #AlarmList::item:selected {
            background-color: #441111;
            color: #ff5533;
        }
        
        /* ===== 历史按钮 ===== */
        #HistoryBtn {
            background-color: #301010;
            color: #ff5533;
            border: 1px solid #aa2222;
            border-radius: 6px;
            padding: 4px 14px;
            font-size: 12px;
            font-weight: bold;
        }
        #HistoryBtn:hover {
            background-color: #441818;
            border: 1px solid #ff4444;
        }
        
        /* ===== 滚动条 ===== */
        QScrollBar:vertical {
            background: #1a0808;
            width: 6px;
            border-radius: 3px;
        }
        QScrollBar::handle:vertical {
            background: #661111;
            border-radius: 3px;
            min-height: 30px;
        }
        QScrollBar::handle:vertical:hover {
            background: #ff4444;
        }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
            height: 0px;
        }

        /* ===== 报警横幅 ===== */
        #AlarmBanner {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                stop:0 #881111, stop:0.5 #cc2222, stop:1 #881111);
            color: #ffffff;
            border: 2px solid #ff2222;
            border-radius: 8px;
            padding: 10px 20px;
            font-size: 16px;
            font-weight: bold;
            font-family: "Microsoft YaHei", "Segoe UI", sans-serif;
        }
        """

        self.setStyleSheet(self.normal_style)

    def _set_logo_glow(self, color_hex):
        """设置 logo 发光颜色以指示连接/报警状态"""
        from PyQt5.QtGui import QColor
        self.logo_glow.setColor(QColor(color_hex))

    # ======================== 红色警报视觉切换 ========================

    def trigger_alarm_visual(self, alarm_msg=""):
        """触发红色警报界面"""
        if self.is_alarm_active:
            # 已在报警模式，只刷新定时器
            self.alarm_dismiss_timer.stop()
            self.alarm_dismiss_timer.start(10000)
            return

        self.is_alarm_active = True
        
        # 切换样式表
        self.setStyleSheet(self.alarm_style)

        # 更新顶部状态指示器
        self._set_logo_glow("#ff2222")

        # 更新报警横幅
        msg = alarm_msg if alarm_msg else "⚠️ 红色警报 — 机器人检测到危险障碍物，正在紧急响应！"
        self.alarm_banner.setText(msg)
        self.alarm_banner.setVisible(True)

        # 10 秒后自动恢复（期间有新红色报警会重置计时）
        self.alarm_dismiss_timer.start(10000)

    def dismiss_alarm_visual(self):
        """解除红色警报，恢复普通界面"""
        if not self.is_alarm_active:
            return

        self.is_alarm_active = False
        self.alarm_dismiss_timer.stop()
        self.alarm_banner.setVisible(False)

        # 恢复普通样式
        self.setStyleSheet(self.normal_style)

        # 恢复顶部颜色（根据连接状态）
        if self.ros_thread and self.ros_thread.isRunning():
            self._set_logo_glow("#00aaff")
        else:
            self._set_logo_glow("#ff4444")

    def toggle_connection(self):
        if self.ros_thread is None or not self.ros_thread.isRunning():
            ip = self.txt_ip.text().strip()
            self.lbl_conn_status.setText("●  连接中...")
            self.lbl_conn_status.setStyleSheet("color: #ffaa00; font-weight: bold; font-size: 13px;")
            self._set_logo_glow("#ffaa00")
            
            self.ros_thread = ROS2ClientThread(ip)
            signals = self.ros_thread.signals

            # 连接状态
            signals.connected_signal.connect(self.handle_connection)

            # AMCL 全局位姿 → 地图显示 + 状态面板坐标
            signals.amcl_pose_signal.connect(self.update_amcl_pose)

            # 里程计 → 仅更新速度信息（线速度、角速度）
            signals.odom_signal.connect(self.update_odom_speed)

            # 地图底图
            signals.map_signal.connect(self.map_widget.update_map)

            # 激光扫描 → 点云渲染（障碍物报警已禁用）
            signals.scan_signal.connect(self.map_widget.update_laser_scan)

            # 跌倒检测报警
            signals.fall_alarm_signal.connect(self.handle_fall_alarm)

            # Nav2 路径
            signals.path_signal.connect(self.map_widget.update_path)

            # 导航状态
            signals.nav_status_signal.connect(self.update_nav_status)
            signals.nav_failed_signal.connect(self.handle_nav_failed)
            
            self.ros_thread.start()
            self.btn_connect.setText("⚡ 断开连接")
            self.btn_connect.setObjectName("ConnectBtn")
        else:
            self.ros_thread.stop()
            self.ros_thread = None
            self.lbl_conn_status.setText("●  未连接")
            self.lbl_conn_status.setStyleSheet("color: #ff4444; font-weight: bold; font-size: 13px;")
            if not self.is_alarm_active:
                self._set_logo_glow("#ff4444")
            self.btn_connect.setText("⚡ 连接 ROS")
            self.btn_connect.setObjectName("ConnectBtn")

    def handle_connection(self, success):
        if success:
            self.lbl_conn_status.setText("●  已连接")
            self.lbl_conn_status.setStyleSheet("color: #00ff88; font-weight: bold; font-size: 13px;")
            if not self.is_alarm_active:
                self._set_logo_glow("#00aaff")
        else:
            self.lbl_conn_status.setText("●  连接失败")
            self.lbl_conn_status.setStyleSheet("color: #ff4444; font-weight: bold; font-size: 13px;")
            if not self.is_alarm_active:
                self._set_logo_glow("#ff4444")
            self.toggle_connection()

    def update_amcl_pose(self, x, y, yaw_deg):
        """AMCL 全局位姿更新 — 地图显示 + 状态面板坐标"""
        self._update_data_row(self.lbl_x, f"{x:.2f} m")
        self._update_data_row(self.lbl_y, f"{y:.2f} m")
        self._update_data_row(self.lbl_yaw, f"{yaw_deg:.2f} °")
        self.lbl_map_coord.setText(f"({x:.2f}, {y:.2f})")
        self.map_widget.update_robot_pose(x, y, yaw_deg)

    def update_odom_speed(self, x, y, yaw, v_lin, v_ang):
        """里程计更新 — 仅更新速度信息"""
        self._update_data_row(self.lbl_vlin, f"{v_lin:.2f} m/s")
        self._update_data_row(self.lbl_vang, f"{v_ang:.2f} rad/s")

    def update_nav_status(self, status_text):
        self._update_data_row(self.lbl_nav_status, status_text)
        # 根据状态改变颜色
        if "SUCCEEDED" in status_text or "已完成" in status_text:
            self.lbl_nav_status.val_label.setStyleSheet("color: #00ff88; font-size: 13px; font-weight: bold; font-family: 'Consolas';")
        elif "EXECUTING" in status_text or "执行中" in status_text:
            self.lbl_nav_status.val_label.setStyleSheet("color: #00e5ff; font-size: 13px; font-weight: bold; font-family: 'Consolas';")
        elif "ABORTED" in status_text or "失败" in status_text:
            self.lbl_nav_status.val_label.setStyleSheet("color: #ff4444; font-size: 13px; font-weight: bold; font-family: 'Consolas';")
        else:
            self.lbl_nav_status.val_label.setStyleSheet("color: #8899bb; font-size: 13px; font-weight: bold; font-family: 'Consolas';")

    def handle_scan_alarm(self, level, message):
        time_str = self.db.insert_alarm(level, message)
        self.alarm_list.insertItem(0, f"[{time_str}] 【{level}】 {message}")
        if self.alarm_list.count() > 50: self.alarm_list.takeItem(self.alarm_list.count() - 1)

        # 红色报警 → 触发全界面红色警报
        if "红色" in level:
            self.trigger_alarm_visual(message)
        # 橙色报警 → 闪烁提醒但不变全红（可后续扩展）
        elif "橙色" in level and not self.is_alarm_active:
            # 橙色时也重置红色报警定时器（延长时间）
            pass

    def handle_fall_alarm(self, level, message):
        """跌倒检测报警处理 — 记录报警 + 触发全界面红色警报"""
        time_str = self.db.insert_alarm(level, f"[跌倒] {message}")
        self.alarm_list.insertItem(0, f"[{time_str}] 【{level}】 [跌倒] {message}")
        if self.alarm_list.count() > 50:
            self.alarm_list.takeItem(self.alarm_list.count() - 1)

        # 跌倒 → 立即触发全界面红色警报
        if "红色" in level:
            self.trigger_alarm_visual(message)

    def handle_fall_cleared(self):
        """跌倒报警解除 — 立即恢复正常界面"""
        time_str = self.db.insert_alarm("报警解除", "[跌倒] 跌倒报警已解除")
        self.alarm_list.insertItem(0, f"[{time_str}] 【报警解除】 [跌倒] 跌倒报警已解除")
        if self.alarm_list.count() > 50:
            self.alarm_list.takeItem(self.alarm_list.count() - 1)
        self.dismiss_alarm_visual()

    def handle_nav_failed(self):
        msg = "导航任务失败（ABORTED）\n机器人无法到达指定目标点！"
        self._update_data_row(self.lbl_nav_status, "失败 (ABORTED)")
        self.lbl_nav_status.val_label.setStyleSheet("color: #ff4444; font-size: 13px; font-weight: bold; font-family: 'Consolas';")
        time_str = self.db.insert_alarm("系统错误", msg)
        self.alarm_list.insertItem(0, f"[{time_str}] 【系统错误】 {msg}")
        # 触发红色警报
        self.trigger_alarm_visual("⚠️ 导航失败 — 机器人无法到达指定目标点！")
        QMessageBox.critical(self, "导航失败报警", msg)

    def show_history_window(self):
        self.history_win = AlarmHistoryWindow(self)
        self.history_win.show()

    def closeEvent(self, event):
        # 立即隐藏窗口，给用户即时反馈（避免看到"未响应"）
        self.hide()
        # 先停止摄像头（RTSP 可能阻塞，有 2 秒超时保护）
        self.camera_widget.stop()
        # 再断开 ROS（有 1 秒超时保护）
        if self.ros_thread:
            self.ros_thread.stop()
        event.accept()
