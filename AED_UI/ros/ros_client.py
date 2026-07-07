#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 通信线程 — 基于 roslibpy WebSocket 连接 rosbridge_server
完整支持：
  /amcl_pose      → AMCL 全局定位（用于地图显示）
  /odom           → 里程计（线速度、角速度，仅用于状态面板）
  /scan           → LaserScan（点云渲染，避障报警已禁用）
  /fall_detection → 跌倒检测报警（Bool/String）
  /map            → OccupancyGrid（地图底图）
  /plan           → Nav2 全局路径
  /navigate_to_pose/_action/status → Nav2 导航状态
"""

import math
import os
import time
from PyQt5.QtCore import QThread, pyqtSignal, QObject

os.environ["AUTOBAHN_USE_NVX"] = "0"
import roslibpy


def euler_from_quaternion(x, y, z, w):
    """四元数 → 欧拉角，返回 Yaw（弧度）"""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


class ROS2Signals(QObject):
    """PyQt 信号集，所有信号在此统一定义，供跨线程通信"""

    # 连接状态
    connected_signal = pyqtSignal(bool)

    # AMCL 全局位姿 — 用于地图显示（替代 /odom）
    # x, y（世界坐标）, yaw_deg（偏航角/度）
    amcl_pose_signal = pyqtSignal(float, float, float)

    # 里程计 — 仅用于状态面板（速度信息）
    # x, y, yaw_deg, v_lin, v_ang
    odom_signal = pyqtSignal(float, float, float, float, float)

    # 激光扫描 — 完整 LaserScan 消息发给 MapWidget 渲染点云
    scan_signal = pyqtSignal(dict)

    # 激光避障报警（距离 < 阈值）- 已禁用
    scan_alarm_signal = pyqtSignal(str, str)  # level, message

    # 跌倒检测报警
    fall_alarm_signal = pyqtSignal(str, str)  # level, message

    # Nav2 全局路径
    path_signal = pyqtSignal(dict)

    # Nav2 导航状态
    nav_status_signal = pyqtSignal(str)
    nav_failed_signal = pyqtSignal()

    # 地图
    map_signal = pyqtSignal(dict)


class ROS2ClientThread(QThread):
    """基于 roslibpy 的 WebSocket 通信线程"""

    def __init__(self, ip, port=9090):
        super().__init__()
        self.ip = ip
        self.port = port
        self.signals = ROS2Signals()
        self.client = None
        self.is_running = True

        # 报警节流
        self.last_alarm_time = 0.0
        # 导航状态去重
        self.last_nav_status = -1
        # 调试计数
        self._scan_count = 0
        self._path_count = 0
        self._amcl_count = 0

    # ======================== 主循环 ========================

    def run(self):
        try:
            print(f"[ROS2] 正在连接 ws://{self.ip}:{self.port} ...")
            self.client = roslibpy.Ros(host=self.ip, port=self.port)
            self.client.on_ready(lambda: self.signals.connected_signal.emit(True))
            self.client.run()

            # ---- 1. 订阅 AMCL 全局位姿（用于地图显示） ----
            self.sub_amcl = roslibpy.Topic(
                self.client, '/amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped'
            )
            self.sub_amcl.subscribe(self.amcl_callback)
            print("[ROS2] ✓ 已订阅 /amcl_pose")

            # ---- 2. 订阅里程计（只用于状态面板速度显示） ----
            self.sub_odom = roslibpy.Topic(
                self.client, '/odom', 'nav_msgs/Odometry'
            )
            self.sub_odom.subscribe(self.odom_callback)
            print("[ROS2] ✓ 已订阅 /odom（仅用于速度）")

            # ---- 3. 订阅激光扫描 ----
            self.sub_scan = roslibpy.Topic(
                self.client, '/scan', 'sensor_msgs/LaserScan'
            )
            self.sub_scan.subscribe(self.scan_callback)
            print("[ROS2] ✓ 已订阅 /scan")

            # ---- 4. 订阅地图 ----
            self.sub_map = roslibpy.Topic(
                self.client, '/map', 'nav_msgs/OccupancyGrid'
            )
            self.sub_map.subscribe(self.map_callback)
            print("[ROS2] ✓ 已订阅 /map")

            # ---- 5. 订阅 Nav2 全局路径 ----
            self.sub_plan = roslibpy.Topic(
                self.client, '/plan', 'nav_msgs/Path'
            )
            self.sub_plan.subscribe(self.plan_callback)
            print("[ROS2] ✓ 已订阅 /plan（Nav2 全局路径）")

            # ---- 6. 订阅 Nav2 导航状态 ----
            self.sub_nav_status = roslibpy.Topic(
                self.client, '/navigate_to_pose/_action/status',
                'action_msgs/GoalStatusArray'
            )
            self.sub_nav_status.subscribe(self.nav_status_callback)
            print("[ROS2] ✓ 已订阅导航状态")

            # ---- 7. 发布导航目标 ----
            self.pub_goal = roslibpy.Topic(
                self.client, '/goal_pose', 'geometry_msgs/PoseStamped'
            )
            print("[ROS2] ✓ 导航目标发布器就绪")

            # ---- 8. 订阅跌倒检测（双 topic 兜底） ----
            self.sub_fall = roslibpy.Topic(
                self.client, '/fall_detection', 'std_msgs/String'
            )
            self.sub_fall.subscribe(self.fall_callback)
            print("[ROS2] ✓ 已订阅 /fall_detection（跌倒检测）")

            self.sub_fall2 = roslibpy.Topic(
                self.client, '/aed_fall_detection', 'std_msgs/String'
            )
            self.sub_fall2.subscribe(self.fall_callback)
            print("[ROS2] ✓ 已订阅 /aed_fall_detection（跌倒检测备用）")

            self.sub_fall_event = roslibpy.Topic(
                self.client, '/fall_event', 'std_msgs/Bool'
            )
            self.sub_fall_event.subscribe(self.fall_event_callback)
            print("[ROS2] ✓ 已订阅 /fall_event（报警开关）")

            print("[ROS2] 🚀 所有 Topic 已订阅，进入主循环...")

            # 保持线程存活
            while self.is_running and self.client.is_connected:
                time.sleep(0.5)

        except Exception as e:
            print(f"[ROS2] ❌ WebSocket 连接异常: {e}")
            if self.is_running:
                self.signals.connected_signal.emit(False)

    # ======================== 回调：AMCL 全局位姿 ========================

    def amcl_callback(self, msg):
        """geometry_msgs/PoseWithCovarianceStamped → 全局 map 坐标系下的位姿"""
        if not self.is_running:
            return
        try:
            pos = msg['pose']['pose']['position']
            ori = msg['pose']['pose']['orientation']
            x, y = pos['x'], pos['y']
            yaw = euler_from_quaternion(ori['x'], ori['y'], ori['z'], ori['w'])
            yaw_deg = math.degrees(yaw)

            self._amcl_count += 1
            if self._amcl_count % 60 == 0:
                print(f"[ROS2] AMCL 位姿: x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f}°  "
                      f"(累计 {self._amcl_count} 次)")

            self.signals.amcl_pose_signal.emit(x, y, yaw_deg)
        except Exception as e:
            if self.is_running:
                print(f"[ROS2] ⚠ AMCL 回调异常: {e}")

    # ======================== 回调：里程计（仅速度） ========================

    def odom_callback(self, msg):
        """nav_msgs/Odometry → 速度 + 局部坐标（仅用于状态面板）"""
        if not self.is_running:
            return
        try:
            pos = msg['pose']['pose']['position']
            ori = msg['pose']['pose']['orientation']
            twist = msg['twist']['twist']

            x, y = pos['x'], pos['y']
            yaw = euler_from_quaternion(ori['x'], ori['y'], ori['z'], ori['w'])
            v_lin = twist['linear']['x']
            v_ang = twist['angular']['z']

            self.signals.odom_signal.emit(x, y, math.degrees(yaw), v_lin, v_ang)
        except Exception:
            pass

    # ======================== 回调：激光扫描 ========================

    def scan_callback(self, msg):
        """sensor_msgs/LaserScan → 点云渲染 + 避障报警"""
        if not self.is_running:
            return
        try:
            # 1. 始终将完整消息发送给 MapWidget 渲染点云
            self.signals.scan_signal.emit(msg)

            # 2. 避障报警逻辑
            ranges = msg.get('ranges', [])
            r_min = msg.get('range_min', 0.0)
            r_max = msg.get('range_max', 10.0)

            valid_ranges = [
                r for r in ranges
                if r is not None and r_min < r < r_max
            ]
            if not valid_ranges:
                return

            # 周边障碍物报警已禁用，仅保留跌倒报警
            # min_dist = min(valid_ranges)
            # level, info = None, None
            #
            # if min_dist < 0.2:
            #     level = "红色报警"
            #     info = f"严重警告：距离障碍物极近 ({min_dist:.2f}m < 0.2m)"
            # elif min_dist < 0.3:
            #     level = "橙色警告"
            #     info = f"警告：距离障碍物过近 ({min_dist:.2f}m < 0.3m)"
            # elif min_dist < 0.5:
            #     level = "黄色警告"
            #     info = f"注意：前方有障碍物 ({min_dist:.2f}m < 0.5m)"
            #
            # current_time = time.time()
            # if level and (current_time - self.last_alarm_time > 2.0):
            #     self.last_alarm_time = current_time
            #     self.signals.scan_alarm_signal.emit(level, info)

        except Exception as e:
            if self.is_running:
                print(f"[ROS2] ⚠ Scan 回调异常: {e}")

    # ======================== 回调：跌倒检测 ========================

    def fall_callback(self, msg):
        """std_msgs/String → 跌倒检测报警"""
        if not self.is_running:
            return
        try:
            data = msg.get('data', '')
            # 调试：打印所有收到的跌倒 topic 消息
            print(f"[ROS2] 🔍 跌倒 topic 收到消息: data='{data}' (type={type(data).__name__})")

            # 支持多种消息格式
            if isinstance(data, bool):
                is_fall = data
                fall_info = "检测到人员跌倒！"
            elif isinstance(data, str):
                data_lower = data.strip().lower()
                # 匹配：包含 fall / 跌倒 / [FALL] 关键字即触发
                is_fall = any(kw in data_lower for kw in ('fall', '跌倒', 'true', 'yes'))
                fall_info = data.strip() if is_fall else ""
            else:
                return

            if is_fall:
                current_time = time.time()
                if current_time - self.last_alarm_time > 2.0:
                    self.last_alarm_time = current_time
                    print(f"[ROS2] 🚨 跌倒检测报警: {fall_info}")
                    self.signals.fall_alarm_signal.emit("红色报警", fall_info)
                else:
                    print(f"[ROS2] ⏳ 跌倒报警节流中（2秒间隔）")

        except Exception as e:
            if self.is_running:
                print(f"[ROS2] ⚠ FallDetection 回调异常: {e}")

    def fall_event_callback(self, msg):
        """std_msgs/Bool → 报警开关，仅响应 true，忽略 false"""
        if not self.is_running:
            return
        try:
            data = msg.get('data', False)
            if not data:
                return  # 忽略 false（解除由 UI 定时器负责）

            current_time = time.time()
            if current_time - self.last_alarm_time > 2.0:
                self.last_alarm_time = current_time
                print(f"[ROS2] 🚨 跌倒报警触发 (来自 /fall_event)")
                self.signals.fall_alarm_signal.emit("红色报警", "检测到人员跌倒！")

        except Exception as e:
            if self.is_running:
                print(f"[ROS2] ⚠ FallEvent 回调异常: {e}")

    def fall_info_callback(self, msg):
        """std_msgs/String → 跌倒详情日志"""
        if not self.is_running:
            return
        try:
            print(f"[ROS2] ℹ️ /fall_info: {msg.get('data', '')}")
        except Exception:
            pass

    # ======================== 回调：地图 ========================

    def map_callback(self, msg):
        """nav_msgs/OccupancyGrid → GUI 渲染地图底图"""
        if not self.is_running:
            return
        try:
            # 首次收到地图时输出调试信息
            info = msg.get('info', {})
            if self._scan_count == 0 or True:
                w, h = info.get('width', 0), info.get('height', 0)
                res = info.get('resolution', 0.0)
                origin = info.get('origin', {}).get('position', {})
                ox, oy = origin.get('x', 0), origin.get('y', 0)
                print(f"[ROS2] 📐 地图信息: {w}x{h} cells, "
                      f"分辨率={res:.4f} m/cell, "
                      f"origin=({ox:.3f}, {oy:.3f})")

            self.signals.map_signal.emit(msg)
        except Exception as e:
            if self.is_running:
                print(f"[ROS2] ⚠ Map 回调异常: {e}")

    # ======================== 回调：Nav2 全局路径 ========================

    def plan_callback(self, msg):
        """nav_msgs/Path → 全局路径点序列"""
        if not self.is_running:
            return
        try:
            poses = msg.get('poses', [])
            self._path_count += 1

            if self._path_count % 20 == 0:
                print(f"[ROS2] 🛤️ 收到全局路径: {len(poses)} 个路径点 "
                      f"(累计 {self._path_count} 次)")

            self.signals.path_signal.emit(msg)
        except Exception as e:
            if self.is_running:
                print(f"[ROS2] ⚠ Path 回调异常: {e}")

    # ======================== 回调：Nav2 导航状态 ========================

    def nav_status_callback(self, msg):
        """action_msgs/GoalStatusArray → 导航状态变更"""
        if not self.is_running:
            return
        try:
            status_list = msg.get('status_list', [])
            if not status_list:
                return

            latest_status = status_list[-1].get('status', 0)

            if latest_status != self.last_nav_status:
                self.last_nav_status = latest_status
                status_dict = {
                    0: "未知 (UNKNOWN)",
                    1: "已接收 (ACCEPTED)",
                    2: "执行中 (EXECUTING)",
                    3: "取消中 (CANCELING)",
                    4: "已完成 (SUCCEEDED)",
                    5: "已取消 (CANCELED)",
                    6: "已中止 (ABORTED)",
                }
                status_text = status_dict.get(latest_status, f"状态码: {latest_status}")
                print(f"[ROS2] 📡 导航状态变更: {status_text}")
                self.signals.nav_status_signal.emit(status_text)

                if latest_status == 6:
                    self.signals.nav_failed_signal.emit()

        except Exception as e:
            if self.is_running:
                print(f"[ROS2] ⚠ NavStatus 回调异常: {e}")

    # ======================== 导航指令 ========================

    def start_nav(self, x, y, yaw):
        """发送导航目标点到 /goal_pose"""
        if self.client and self.client.is_connected:
            goal_msg = {
                "header": {
                    "frame_id": "map",
                    "stamp": {"secs": int(time.time()), "nsecs": 0},
                },
                "pose": {
                    "position": {"x": x, "y": y, "z": 0.0},
                    "orientation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": math.sin(math.radians(yaw) / 2.0),
                        "w": math.cos(math.radians(yaw) / 2.0),
                    },
                },
            }
            self.pub_goal.publish(roslibpy.Message(goal_msg))
            print(f"[ROS2] 🎯 导航目标已发送: ({x:.2f}, {y:.2f}), yaw={yaw}°")
            return True
        else:
            print("[ROS2] ⚠ 无法发送导航目标（未连接）")
            return False

    # ======================== 停止线程 ========================

    def stop(self):
        """
        安全停止 ROS 通信线程（非阻塞版）。
        不在主线程中调用 client.terminate()（可能阻塞），让线程自然退出。
        """
        print("[ROS2] 🛑 正在断开连接...")
        self.is_running = False

        # 第一步：取消所有订阅，阻止更多回调触发
        sub_names = ['sub_amcl', 'sub_odom', 'sub_scan', 'sub_map',
                     'sub_plan', 'sub_nav_status', 'sub_fall', 'sub_fall2',
                     'sub_fall_event', 'sub_fall_info']
        for name in sub_names:
            sub = getattr(self, name, None)
            if sub:
                try:
                    sub.unsubscribe()
                except Exception:
                    pass

        # 第二步：通知退出（run() 循环检查 is_running，最多 0.5 秒内跳出）
        self.quit()

        # 第三步：等线程自然退出，最多 2 秒
        if not self.wait(2000):
            # 超时才强制断开 WebSocket 并终止线程
            print("[ROS2] ⚠ 线程未在规定时间内退出，强制终止")
            if self.client:
                try:
                    self.client.terminate()
                except Exception:
                    pass
            self.terminate()

        # 线程已退出，安全清理 WebSocket（不在主线程阻塞）
        if self.client:
            try:
                self.client.close()
            except Exception:
                pass

        print("[ROS2] ✓ 线程已停止")
