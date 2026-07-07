#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能AED救援车监控平台 — Kivy/Android 版
运行: pip install kivy roslibpy numpy opencv-python && python main.py
打包: buildozer android debug
"""

import math
import numpy as np
import threading
import sys
import os

os.environ["AUTOBAHN_USE_NVX"] = "0"

from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.scrollview import ScrollView
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.uix.widget import Widget
from kivy.uix.popup import Popup
from kivy.uix.recycleview import RecycleView
from kivy.uix.recycleboxlayout import RecycleBoxLayout
from kivy.uix.recycleview.views import RecycleDataViewBehavior
from kivy.properties import (StringProperty, NumericProperty, BooleanProperty,
                              ObjectProperty, ListProperty, ColorProperty)
from kivy.clock import Clock, mainthread
from kivy.graphics import (Color, Rectangle, Ellipse, Line, PushMatrix,
                           PopMatrix, Rotate, Translate, Scale)
from kivy.graphics.texture import Texture
from kivy.core.window import Window
from kivy.metrics import dp, sp

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False
    print("[WARN] opencv-python 未安装，摄像头功能不可用")

from database.db import DatabaseManager
from ros_client import ROS2Client

# ============================================================
# 配色（与原 PyQt 版一致）
# ============================================================
C_BG        = (0.039, 0.055, 0.102, 1)    # #0a0e1a
C_BG_CARD   = (0.051, 0.078, 0.157, 0.85) # #0d1428
C_BORDER    = (0.118, 0.227, 0.373, 1)    # #1e3a5f
C_CYAN      = (0.0,  0.898, 1.0,   1)     # #00e5ff
C_GREEN     = (0.0,  1.0,   0.533, 1)     # #00ff88
C_RED       = (1.0,  0.267, 0.267, 1)     # #ff4444
C_ORANGE    = (1.0,  0.533, 0.0,   1)     # #ff8800
C_YELLOW    = (1.0,  0.8,   0.0,   1)     # #ffcc00
C_TEXT      = (0.784, 0.839, 0.941, 1)    # #c8d6f0
C_TEXT_DIM  = (0.467, 0.6,   0.8,   1)    # #7799cc
C_TEXT_MUTE = (0.267, 0.333, 0.4,   1)    # #445566
C_BTN_BG    = (0.0,   0.4,   0.6,   1)    # #006699
C_BLUE      = (0.0,   0.549, 1.0,   0.8)  # robot blue


# ============================================================
# 地图渲染 Widget
# ============================================================
class MapWidget(Widget):
    """基于 Kivy Canvas 的 OccupancyGrid + LaserScan + Path 地图"""

    scale = NumericProperty(1.0)
    offset_x = NumericProperty(0)
    offset_y = NumericProperty(0)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.map_data = None       # (width, height, resolution, origin_x, origin_y)
        self.map_texture = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0       # 弧度
        self.laser_points = []
        self.path_points = []
        self._auto_fit = False
        self._touch_start = None
        self._touch_last = None
        self._last_dist = 0

    def update_map(self, map_msg):
        """解析 OccupancyGrid，生成 Texture"""
        try:
            info = map_msg['info']
            w, h, res = info['width'], info['height'], info['resolution']
            ox = info['origin']['position']['x']
            oy = info['origin']['position']['y']
            self.map_data = (w, h, res, ox, oy)

            data = np.array(map_msg['data'], dtype=np.int16)
            if len(data) != w * h:
                print(f"[Map] 数据长度不匹配: {len(data)} vs {w}*{h}")
                return
            data = data.reshape((h, w))

            # RGBA 纹理
            img = np.zeros((h, w, 4), dtype=np.uint8)
            img[:, :, 3] = 255  # 不透明

            img[data == -1] = [200, 200, 200, 255]     # 未知 → 浅灰
            img[data == 0]  = [255, 255, 255, 255]     # 空闲 → 白
            img[data == 100]= [0,   0,   0,   255]     # 障碍 → 黑
            mask = (data > 0) & (data < 100)
            if np.any(mask):
                g = (255 - data[mask] * 2.55).astype(np.uint8)
                img[mask, 0] = g
                img[mask, 1] = g
                img[mask, 2] = g

            # 创建 Kivy Texture
            tex = Texture.create(size=(w, h), colorfmt='rgba')
            tex.blit_buffer(img.tobytes(), colorfmt='rgba', bufferfmt='ubyte')
            tex.flip_vertical()  # Kivy 纹理 Y 轴翻转
            self.map_texture = tex

            print(f"[Map] 地图加载: {w}x{h} cells, res={res:.4f}")
            if not self._auto_fit:
                Clock.schedule_once(lambda dt: self._fit_to_view(), 0.1)
                self._auto_fit = True
            self._redraw()
        except Exception as e:
            print(f"[Map] ERR: {e}")

    def _fit_to_view(self):
        if not self.map_data:
            return
        w, h = self.map_data[0], self.map_data[1]
        vw, vh = self.width, self.height
        if vw <= 0 or vh <= 0:
            return
        self.scale = min(vw / w, vh / h) * 0.9
        self.offset_x = (vw - w * self.scale) / 2
        self.offset_y = (vh - h * self.scale) / 2

    def update_robot_pose(self, x, y, yaw_deg):
        self.robot_x = x
        self.robot_y = y
        self.robot_yaw = math.radians(yaw_deg)
        self._redraw()

    def update_scan(self, scan_msg):
        if not self.map_data:
            return
        try:
            ranges = scan_msg.get('ranges', [])
            amin = scan_msg.get('angle_min', 0.0)
            ainc = scan_msg.get('angle_increment', 0.0)
            rmin = scan_msg.get('range_min', 0.0)
            rmax = scan_msg.get('range_max', 10.0)
            cy = math.cos(self.robot_yaw)
            sy = math.sin(self.robot_yaw)
            pts = []
            for i, r in enumerate(ranges):
                if r is None or r < rmin or r > rmax:
                    continue
                angle = amin + i * ainc
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                mx = self.robot_x + lx * cy - ly * sy
                my = self.robot_y + lx * sy + ly * cy
                pts.append((mx, my))
            self.laser_points = pts
            self._redraw()
        except Exception as e:
            print(f"[Map] Scan ERR: {e}")

    def update_path(self, path_msg):
        try:
            pts = []
            for pe in path_msg.get('poses', []):
                pos = pe.get('pose', {}).get('position', {})
                pts.append((pos.get('x', 0), pos.get('y', 0)))
            self.path_points = pts
            self._redraw()
        except Exception as e:
            print(f"[Map] Path ERR: {e}")

    def _w2p(self, wx, wy):
        """世界坐标 → 像素坐标"""
        if not self.map_data:
            return 0, 0
        w, h, res, ox, oy = self.map_data
        px = (wx - ox) / res
        py = (wy - oy) / res
        return px, py

    def _redraw(self, *args):
        self.canvas.after.clear()
        with self.canvas.after:
            # 地图底图
            if self.map_texture:
                tex_w, tex_h = self.map_texture.size
                Color(1, 1, 1, 1)
                Rectangle(
                    pos=(self.offset_x, self.offset_y),
                    size=(tex_w * self.scale, tex_h * self.scale),
                    texture=self.map_texture,
                )

            # 路径（蓝色虚线）
            if self.path_points:
                Color(*C_CYAN[:-1], 0.8)
                pp = []
                for wx, wy in self.path_points:
                    px, py = self._w2p(wx, wy)
                    pp.extend([
                        self.offset_x + px * self.scale,
                        self.offset_y + py * self.scale,
                    ])
                if len(pp) >= 4:
                    Line(points=pp, width=dp(1.5), dash_offset=4)

            # 激光点云（红色散点）
            if self.laser_points:
                Color(*C_RED[:-1], 0.7)
                r = self.scale * 2
                for wx, wy in self.laser_points:
                    px, py = self._w2p(wx, wy)
                    cx = self.offset_x + px * self.scale
                    cy = self.offset_y + py * self.scale
                    Ellipse(pos=(cx - r, cy - r), size=(r * 2, r * 2))

            # 机器人（蓝色圆 + 黄色方向）
            if self.map_data:
                Color(*C_BLUE)
                rx, ry = self._w2p(self.robot_x, self.robot_y)
                cx = self.offset_x + rx * self.scale
                cy = self.offset_y + ry * self.scale
                rr = max(dp(4), self.scale * 4)
                PushMatrix()
                Translate(cx, cy)
                Rotate(angle=math.degrees(self.robot_yaw))
                Ellipse(pos=(-rr, -rr), size=(rr * 2, rr * 2))
                Color(*C_YELLOW)
                Line(points=[0, 0, rr * 1.8, 0], width=dp(1.5))
                PopMatrix()

    def on_size(self, *args):
        if self._auto_fit and self.map_data:
            self._fit_to_view()
        self._redraw()

    def on_touch_down(self, touch):
        if self.collide_point(*touch.pos):
            if touch.is_mouse_scrolling:
                # 双指缩放
                pass
            elif 'button' in touch.profile and touch.button == 'scrolldown':
                self.scale = max(0.1, min(10, self.scale * 0.9))
                self._redraw()
                return True
            elif 'button' in touch.profile and touch.button == 'scrollup':
                self.scale = max(0.1, min(10, self.scale * 1.1))
                self._redraw()
                return True
            else:
                self._touch_start = touch.pos
                self._touch_last = touch.pos
                return True
        return super().on_touch_down(touch)

    def on_touch_move(self, touch):
        if self._touch_last and self.collide_point(*touch.pos):
            dx = touch.x - self._touch_last[0]
            dy = touch.y - self._touch_last[1]
            self.offset_x += dx
            self.offset_y += dy
            self._touch_last = touch.pos
            self._redraw()
            return True
        return super().on_touch_move(touch)

    def on_touch_up(self, touch):
        self._touch_start = None
        self._touch_last = None
        return super().on_touch_up(touch)


# ============================================================
# 报警列表行
# ============================================================
class AlarmRow(RecycleDataViewBehavior, BoxLayout):
    time_text = StringProperty('')
    level_text = StringProperty('')
    msg_text = StringProperty('')
    level_color = ColorProperty(C_TEXT)

    def refresh_view_attrs(self, rv, index, data):
        self.time_text = data.get('time', '')
        self.level_text = data.get('level', '')
        self.msg_text = data.get('message', '')
        lv = data.get('level', '')
        if '红' in lv:
            self.level_color = C_RED
        elif '橙' in lv:
            self.level_color = C_ORANGE
        elif '黄' in lv:
            self.level_color = C_YELLOW
        else:
            self.level_color = C_TEXT_DIM
        return super().refresh_view_attrs(rv, index, data)


# ============================================================
# 数据行组件
# ============================================================
class DataRow(BoxLayout):
    label = StringProperty('')
    value = StringProperty('')


# ============================================================
# 主屏幕
# ============================================================
class MainScreen(BoxLayout):
    conn_status = StringProperty('未连接')
    conn_color = ColorProperty(C_RED)
    btn_text = StringProperty('连接 ROS')

    # 状态数据
    v_x = StringProperty('0.00 m')
    v_y = StringProperty('0.00 m')
    v_yaw = StringProperty('0.00')
    v_vlin = StringProperty('0.00 m/s')
    v_vang = StringProperty('0.00 rad/s')
    v_nav = StringProperty('等待中')
    v_coord = StringProperty('—')
    nav_color = ColorProperty(C_TEXT_DIM)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.db = DatabaseManager()
        self.ros = None
        self.camera_running = False
        self.cap = None
        self.alarm_items = []

        # 启动摄像头
        Clock.schedule_once(lambda dt: self._start_camera(), 1.0)

    # ---------- ROS 连接 ----------
    def toggle_connection(self):
        if self.ros and self.ros.is_alive():
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        ip = self.ids.txt_ip.text.strip() or '192.168.1.103'
        self.conn_status = '连接中...'
        self.conn_color = C_ORANGE

        callbacks = {
            'connected':    self._on_connected,
            'amcl_pose':    self._on_amcl_pose,
            'odom':         self._on_odom,
            'scan':         self._on_scan,
            'scan_alarm':   self._on_scan_alarm,
            'map':          self._on_map,
            'path':         self._on_path,
            'nav_status':   self._on_nav_status,
            'nav_failed':   self._on_nav_failed,
        }
        self.ros = ROS2Client(ip, callbacks=callbacks)
        self.ros.start()
        self.btn_text = '断开连接'

    def _disconnect(self):
        if self.ros:
            self.ros.stop()
            self.ros = None
        self.conn_status = '未连接'
        self.conn_color = C_RED
        self.btn_text = '连接 ROS'

    @mainthread
    def _on_connected(self, ok):
        if ok:
            self.conn_status = '已连接'
            self.conn_color = C_GREEN
        else:
            self.conn_status = '连接失败'
            self.conn_color = C_RED
            self._disconnect()

    @mainthread
    def _on_amcl_pose(self, x, y, yaw_deg):
        self.v_x = f'{x:.2f} m'
        self.v_y = f'{y:.2f} m'
        self.v_yaw = f'{yaw_deg:.2f}'
        self.v_coord = f'({x:.2f}, {y:.2f})'
        self.ids.map_widget.update_robot_pose(x, y, yaw_deg)

    @mainthread
    def _on_odom(self, x, y, yaw, v_lin, v_ang):
        self.v_vlin = f'{v_lin:.2f} m/s'
        self.v_vang = f'{v_ang:.2f} rad/s'

    @mainthread
    def _on_scan(self, msg):
        self.ids.map_widget.update_scan(msg)

    @mainthread
    def _on_scan_alarm(self, level, message):
        ts = self.db.insert_alarm(level, message)
        self.alarm_items.insert(0, {
            'time': ts, 'level': level, 'message': message
        })
        if len(self.alarm_items) > 50:
            self.alarm_items.pop()
        if hasattr(self.ids, 'alarm_rv'):
            self.ids.alarm_rv.data = self.alarm_items

    @mainthread
    def _on_map(self, msg):
        self.ids.map_widget.update_map(msg)

    @mainthread
    def _on_path(self, msg):
        self.ids.map_widget.update_path(msg)

    @mainthread
    def _on_nav_status(self, text):
        self.v_nav = text
        if 'SUCCEEDED' in text or '完成' in text:
            self.nav_color = C_GREEN
        elif 'EXECUTING' in text or '执行' in text:
            self.nav_color = C_CYAN
        elif 'ABORTED' in text or '失败' in text:
            self.nav_color = C_RED
        else:
            self.nav_color = C_TEXT_DIM

    @mainthread
    def _on_nav_failed(self):
        msg = '导航任务失败（ABORTED）\n机器人无法到达指定目标点！'
        self.v_nav = '失败 (ABORTED)'
        self.nav_color = C_RED
        ts = self.db.insert_alarm('系统错误', msg)
        self.alarm_items.insert(0, {
            'time': ts, 'level': '系统错误', 'message': msg
        })
        if hasattr(self.ids, 'alarm_rv'):
            self.ids.alarm_rv.data = self.alarm_items

    # ---------- 摄像头 ----------
    def _start_camera(self):
        if not HAS_CV2:
            return
        self.camera_running = True
        source = "rtsp://admin:@192.168.1.86"
        print(f"[Camera] 启动摄像头: {source}")
        t = threading.Thread(target=self._camera_loop, args=(source,), daemon=True)
        t.start()

    def _camera_loop(self, source):
        try:
            cap = cv2.VideoCapture(source)
            self.cap = cap
            while self.camera_running and cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    Clock.schedule_once(lambda dt, f=frame: self._show_frame(f), 0)
                else:
                    import time
                    time.sleep(0.01)
        except Exception as e:
            print(f"[Camera] ERR: {e}")
        finally:
            if self.cap:
                self.cap.release()
                self.cap = None

    @mainthread
    def _show_frame(self, frame):
        try:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w = frame.shape[:2]
            tex = Texture.create(size=(w, h), colorfmt='rgb')
            tex.blit_buffer(frame.tobytes(), colorfmt='rgb', bufferfmt='ubyte')
            tex.flip_vertical()
            self.ids.cam_image.texture = tex
        except Exception:
            pass

    # ---------- 历史报警 ----------
    def show_history(self):
        records = self.db.get_alarms(limit=100)
        total = self.db.get_total_count()
        content = '\n'.join(
            f"[{r[0]}] 【{r[1]}】 {r[2]}" for r in records
        ) or '(暂无报警记录)'
        popup = Popup(
            title=f'历史报警记录 (共 {total} 条)',
            content=ScrollView(
                do_scroll_x=False,
                bar_width=dp(8)
            ),
            size_hint=(0.92, 0.8),
            background_color=C_BG_CARD,
            title_color=C_CYAN,
            separator_color=C_BORDER,
        )
        label = Label(
            text=content,
            color=C_TEXT,
            font_size=sp(12),
            size_hint_y=None,
            text_size=(None, None),
            halign='left',
            valign='top',
            padding=(dp(12), dp(12)),
        )
        label.bind(texture_size=lambda *x: setattr(label, 'height', label.texture_size[1]))
        popup.content.add_widget(label)
        popup.open()

    # ---------- 关闭 ----------
    def on_stop(self):
        print("[App] 正在关闭...")
        self.camera_running = False
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
        if self.ros:
            self.ros.stop(timeout=2.0)


# ============================================================
# Kivy App
# ============================================================
class AEDApp(App):
    def build(self):
        Window.clearcolor = C_BG
        self.title = 'AED 监控平台'
        self.screen = MainScreen()
        return self.screen

    def on_stop(self):
        self.screen.on_stop()


if __name__ == '__main__':
    AEDApp().run()
