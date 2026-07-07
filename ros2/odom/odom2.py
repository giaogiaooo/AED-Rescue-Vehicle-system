#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import time
import struct
import threading
import sys
import math

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

CAR_WIDTH  = 0.205
CAR_LENGTH = 0.178
WHEEL_DIAMETER = 0.080
ENCODER_PPR = 1320.0
KINEMATIC_K = (CAR_WIDTH + CAR_LENGTH) / 2.0

class Elf2BaseNode(Node):
    def __init__(self):
        super().__init__('elf2_base_node')

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f"✅ 串口打开成功: {SERIAL_PORT}")
        except Exception as e:
            self.get_logger().error(f"❌ 串口打开失败: {e}")
            sys.exit(1)

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        
        # 保存用于发布的线速度和角速度
        self.vx = 0.0
        self.vy = 0.0
        self.vw = 0.0
        
        self.last_odom_time = self.get_clock().now()

        # ======================= 调试需求：新增累计计数器和时间戳 =======================
        self.total_lf = 0
        self.total_rf = 0
        self.total_lr = 0
        self.total_rr = 0
        self.last_debug_time = time.time()
        # ===========================================================================

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.running = True
        self.create_timer(0.05, self.publish_odom_and_tf)

        self.rx_thread = threading.Thread(target=self.serial_rx_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info("✅ 底盘驱动 + 里程计 + TF 已启动")

    def cmd_vel_callback(self, msg):
        vx = int(msg.linear.x * 100)
        vy = int(msg.linear.y * 100)
        w = int(msg.angular.z * 50)

        vx = max(-32767, min(32767, vx))
        vy = max(-32767, min(32767, vy))
        w = max(-32767, min(32767, w))

        vx_h = (vx >> 8) & 0xFF
        vx_l = vx & 0xFF
        vy_h = (vy >> 8) & 0xFF
        vy_l = vy & 0xFF
        w_h = (w >> 8) & 0xFF
        w_l = w & 0xFF

        checksum = (vx_h + vx_l + vy_h + vy_l + w_h + w_l) & 0xFF
        packet = bytearray([0xAA, 0x55, vx_h, vx_l, vy_h, vy_l, w_h, w_l, checksum, 0x0D])
        try:
            self.ser.write(packet)
        except:
            pass

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = [0,0,0,0]
        q[0] = sr * cp * cy - cr * sp * sy
        q[1] = cr * sp * cy + sr * cp * sy
        q[2] = cr * cp * sy - sr * sp * cy
        q[3] = cr * cp * cy + sr * sp * cy
        return q

    def publish_odom_and_tf(self):
        current_time = self.get_clock().now()
        q = self.quaternion_from_euler(0, 0, self.odom_theta)

        # 1. 发布 TF 变换
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # 2. 发布里程计消息
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # ======================= 调试需求：发布 Twist 速度信息 =======================
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vw
        # ===========================================================================

        # SLAM Toolbox 等算法要求协方差矩阵不能全为 0，否则拒绝初始化
        # 对角线元素分别代表 x, y, z, roll, pitch, yaw 的不确定性
        odom.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,   # x 的不确定性
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,   # y 的不确定性
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, # z 的不确定性（平面机器人设为极大值）
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, # roll 的不确定性
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, # pitch 的不确定性
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01     # yaw 的不确定性
        ]

        self.odom_pub.publish(odom)

    def serial_rx_loop(self):
        buffer = bytearray()
        while self.running and rclpy.ok():
            try:
                if self.ser.in_waiting:
                    buffer += self.ser.read(self.ser.in_waiting)

                while len(buffer) >= 12:
                    if buffer[0] == 0xAA and buffer[1] == 0x66:
                        calc_sum = sum(buffer[2:10]) & 0xFF
                        if calc_sum == buffer[10] and buffer[11] == 0x0D:
                            lf, rf, lr, rr = struct.unpack(">hhhh", buffer[2:10])
                            
                            # ======================= 调试需求：打印编码器与累计 =======================
                            print(
                                f"LF={lf:6d} RF={rf:6d} "
                                f"LR={lr:6d} RR={rr:6d}"
                            )

                            self.total_lf += lf
                            self.total_rf += rf
                            self.total_lr += lr
                            self.total_rr += rr

                            current_sys_time = time.time()
                            if current_sys_time - self.last_debug_time >= 1.0:
                                print(
                                    f"TOTAL -> "
                                    f"LF={self.total_lf} "
                                    f"RF={self.total_rf} "
                                    f"LR={self.total_lr} "
                                    f"RR={self.total_rr}"
                                )
                                self.last_debug_time = current_sys_time
                            # =========================================================================

                            current_time = self.get_clock().now()
                            dt = (current_time - self.last_odom_time).nanoseconds / 1e9
                            if dt < 0.001: dt = 0.02
                            self.last_odom_time = current_time

                            coeff = (math.pi * WHEEL_DIAMETER) / ENCODER_PPR
                            d_lf = lf * coeff
                            d_rf = rf * coeff
                            d_lr = lr * coeff
                            d_rr = rr * coeff

                            # ======================= 已将 x 和 y 的运动学解算恢复 =======================
                            dx_robot = (d_lf + d_rf + d_lr + d_rr) / 4.0
                            dy_robot = (-d_lf + d_rf + d_lr - d_rr) / 4.0  # 已将 Y 轴左右方向取反
                            dtheta = (-d_lf + d_rf - d_lr + d_rr) / (4.0 * KINEMATIC_K)
                            # ===================================================================================

                            # ======================= 调试需求：计算速度供发布 =======================
                            self.vx = dx_robot / dt
                            self.vy = dy_robot / dt
                            self.vw = dtheta / dt
                            # =========================================================================

                            c = math.cos(self.odom_theta)
                            s = math.sin(self.odom_theta)

                            self.odom_x += dx_robot * c - dy_robot * s
                            self.odom_y += dx_robot * s + dy_robot * c
                            self.odom_theta += dtheta

                            # ======================= 调试需求：角度归一化与 Odom 打印 =======================
                            while self.odom_theta > math.pi:
                                self.odom_theta -= 2.0 * math.pi

                            while self.odom_theta < -math.pi:
                                self.odom_theta += 2.0 * math.pi

                            print(
                                f"ODOM: "
                                f"x={self.odom_x:.6f} "
                                f"y={self.odom_y:.6f} "
                                f"theta={math.degrees(self.odom_theta):.3f}"
                            )
                            # =========================================================================

                            buffer = buffer[12:]
                            continue
                    buffer.pop(0)
                time.sleep(0.002)
            except Exception as e:
                time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = Elf2BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()