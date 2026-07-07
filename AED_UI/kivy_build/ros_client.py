#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 通信线程 — Kivy 版，基于 roslibpy WebSocket
用 threading.Thread + kivy.clock.mainthread 替代 QThread + pyqtSignal
"""

import math
import os
import time
import threading

os.environ["AUTOBAHN_USE_NVX"] = "0"
import roslibpy

try:
    from kivy.clock import mainthread
except ImportError:
    # 回退：允许在更简单的环境中使用（如命令行测试）
    def mainthread(f):
        return f


def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


class ROS2Client(threading.Thread):
    """基于 roslibpy 的 WebSocket 通信线程（Kivy 兼容版）"""

    def __init__(self, ip, port=9090, callbacks=None):
        super().__init__(daemon=True)
        self.ip = ip
        self.port = port
        self.client = None
        self.is_running = True

        # 回调字典，由主界面设置
        # callbacks = {
        #   'connected': func(bool),
        #   'amcl_pose': func(float, float, float),
        #   'odom': func(float, float, float, float, float),
        #   'scan': func(dict),
        #   'scan_alarm': func(str, str),
        #   'map': func(dict),
        #   'path': func(dict),
        #   'nav_status': func(str),
        #   'nav_failed': func(),
        # }
        self.cb = callbacks or {}

        # 内部状态
        self.last_alarm_time = 0.0
        self.last_nav_status = -1
        self._scan_count = 0
        self._path_count = 0
        self._amcl_count = 0

    # ---------- 安全地从子线程回调主线程 ----------
    def _safe_cb(self, key, *args):
        """确保回调在主线程执行"""
        cb = self.cb.get(key)
        if cb:
            try:
                mainthread(cb)(*args)
            except Exception:
                pass

    # ---------- 主循环 ----------
    def run(self):
        try:
            print(f"[ROS2] 正在连接 ws://{self.ip}:{self.port} ...")
            self.client = roslibpy.Ros(host=self.ip, port=self.port)
            self.client.on_ready(lambda: self._safe_cb('connected', True))
            self.client.run()

            self.sub_amcl = roslibpy.Topic(
                self.client, '/amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped')
            self.sub_amcl.subscribe(self._amcl_cb)
            print("[ROS2] OK 已订阅 /amcl_pose")

            self.sub_odom = roslibpy.Topic(
                self.client, '/odom', 'nav_msgs/Odometry')
            self.sub_odom.subscribe(self._odom_cb)
            print("[ROS2] OK 已订阅 /odom")

            self.sub_scan = roslibpy.Topic(
                self.client, '/scan', 'sensor_msgs/LaserScan')
            self.sub_scan.subscribe(self._scan_cb)
            print("[ROS2] OK 已订阅 /scan")

            self.sub_map = roslibpy.Topic(
                self.client, '/map', 'nav_msgs/OccupancyGrid')
            self.sub_map.subscribe(self._map_cb)
            print("[ROS2] OK 已订阅 /map")

            self.sub_plan = roslibpy.Topic(
                self.client, '/plan', 'nav_msgs/Path')
            self.sub_plan.subscribe(self._plan_cb)
            print("[ROS2] OK 已订阅 /plan")

            self.sub_nav_status = roslibpy.Topic(
                self.client, '/navigate_to_pose/_action/status',
                'action_msgs/GoalStatusArray')
            self.sub_nav_status.subscribe(self._nav_status_cb)
            print("[ROS2] OK 已订阅导航状态")

            self.pub_goal = roslibpy.Topic(
                self.client, '/goal_pose', 'geometry_msgs/PoseStamped')
            print("[ROS2] OK 导航目标发布器就绪")

            print("[ROS2] >>> 所有 Topic 已订阅，进入主循环...")

            while self.is_running and self.client.is_connected:
                time.sleep(0.5)

        except Exception as e:
            print(f"[ROS2] ERR WebSocket 连接异常: {e}")
            if self.is_running:
                self._safe_cb('connected', False)

    # ---------- 回调 ----------
    def _amcl_cb(self, msg):
        if not self.is_running:
            return
        try:
            pos = msg['pose']['pose']['position']
            ori = msg['pose']['pose']['orientation']
            x, y = pos['x'], pos['y']
            yaw = euler_from_quaternion(ori['x'], ori['y'], ori['z'], ori['w'])
            self._amcl_count += 1
            if self._amcl_count % 60 == 0:
                print(f"[ROS2] AMCL: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}  (x{self._amcl_count})")
            self._safe_cb('amcl_pose', x, y, math.degrees(yaw))
        except Exception as e:
            if self.is_running:
                print(f"[ROS2] WARN AMCL 回调异常: {e}")

    def _odom_cb(self, msg):
        if not self.is_running:
            return
        try:
            pos = msg['pose']['pose']['position']
            ori = msg['pose']['pose']['orientation']
            twist = msg['twist']['twist']
            yaw = euler_from_quaternion(ori['x'], ori['y'], ori['z'], ori['w'])
            v_lin = twist['linear']['x']
            v_ang = twist['angular']['z']
            self._safe_cb('odom', pos['x'], pos['y'], math.degrees(yaw), v_lin, v_ang)
        except Exception:
            pass

    def _scan_cb(self, msg):
        if not self.is_running:
            return
        try:
            self._safe_cb('scan', msg)
            ranges = msg.get('ranges', [])
            r_min = msg.get('range_min', 0.0)
            r_max = msg.get('range_max', 10.0)
            valid = [r for r in ranges if r is not None and r_min < r < r_max]
            if not valid:
                return
            min_dist = min(valid)
            level, info = None, None
            if min_dist < 0.2:
                level, info = "红色报警", f"严重警告：距离障碍物极近 ({min_dist:.2f}m < 0.2m)"
            elif min_dist < 0.3:
                level, info = "橙色警告", f"警告：距离障碍物过近 ({min_dist:.2f}m < 0.3m)"
            elif min_dist < 0.5:
                level, info = "黄色警告", f"注意：前方有障碍物 ({min_dist:.2f}m < 0.5m)"
            now = time.time()
            if level and (now - self.last_alarm_time > 2.0):
                self.last_alarm_time = now
                self._safe_cb('scan_alarm', level, info)
        except Exception as e:
            if self.is_running:
                print(f"[ROS2] WARN Scan 回调异常: {e}")

    def _map_cb(self, msg):
        if not self.is_running:
            return
        try:
            info = msg.get('info', {})
            w, h = info.get('width', 0), info.get('height', 0)
            res = info.get('resolution', 0.0)
            origin = info.get('origin', {}).get('position', {})
            print(f"[ROS2] MAP: {w}x{h} cells, res={res:.4f}, origin=({origin.get('x',0):.3f},{origin.get('y',0):.3f})")
            self._safe_cb('map', msg)
        except Exception as e:
            if self.is_running:
                print(f"[ROS2] WARN Map 回调异常: {e}")

    def _plan_cb(self, msg):
        if not self.is_running:
            return
        try:
            self._path_count += 1
            poses = msg.get('poses', [])
            if self._path_count % 20 == 0:
                print(f"[ROS2] PATH: {len(poses)} points (x{self._path_count})")
            self._safe_cb('path', msg)
        except Exception as e:
            if self.is_running:
                print(f"[ROS2] WARN Path 回调异常: {e}")

    def _nav_status_cb(self, msg):
        if not self.is_running:
            return
        try:
            status_list = msg.get('status_list', [])
            if not status_list:
                return
            latest = status_list[-1].get('status', 0)
            if latest != self.last_nav_status:
                self.last_nav_status = latest
                status_dict = {
                    0: "未知 (UNKNOWN)", 1: "已接收 (ACCEPTED)", 2: "执行中 (EXECUTING)",
                    3: "取消中 (CANCELING)", 4: "已完成 (SUCCEEDED)",
                    5: "已取消 (CANCELED)", 6: "已中止 (ABORTED)",
                }
                text = status_dict.get(latest, f"状态码: {latest}")
                print(f"[ROS2] NAV: {text}")
                self._safe_cb('nav_status', text)
                if latest == 6:
                    self._safe_cb('nav_failed')
        except Exception as e:
            if self.is_running:
                print(f"[ROS2] WARN NavStatus 回调异常: {e}")

    # ---------- 导航指令 ----------
    def start_nav(self, x, y, yaw):
        if self.client and self.client.is_connected:
            goal_msg = {
                "header": {"frame_id": "map", "stamp": {"secs": int(time.time()), "nsecs": 0}},
                "pose": {
                    "position": {"x": x, "y": y, "z": 0.0},
                    "orientation": {
                        "x": 0.0, "y": 0.0,
                        "z": math.sin(math.radians(yaw) / 2.0),
                        "w": math.cos(math.radians(yaw) / 2.0),
                    },
                },
            }
            self.pub_goal.publish(roslibpy.Message(goal_msg))
            print(f"[ROS2] GOAL sent: ({x:.2f}, {y:.2f}) yaw={yaw}")
            return True
        else:
            print("[ROS2] WARN 无法发送目标（未连接）")
            return False

    # ---------- 停止 ----------
    def stop(self, timeout=3.0):
        print("[ROS2] STOP 正在断开连接...")
        self.is_running = False
        for name in ['sub_amcl', 'sub_odom', 'sub_scan', 'sub_map', 'sub_plan', 'sub_nav_status']:
            sub = getattr(self, name, None)
            if sub:
                try:
                    sub.unsubscribe()
                except Exception:
                    pass
        if self.client:
            try:
                self.client.terminate()
            except Exception:
                pass
        self.join(timeout)
        if self.is_alive():
            print("[ROS2] WARN 线程未在超时内退出")
        print("[ROS2] OK 线程已停止")
