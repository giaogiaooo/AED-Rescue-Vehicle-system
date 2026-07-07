#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

def get_quaternion_from_euler(roll, pitch, yaw):
    """欧拉角转四元数"""
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class AEDInitializerNode(Node):
    def __init__(self):
        super().__init__('aed_initializer_node')
        
        # 待命点 (HOME) 坐标
        self.HOME_X = 3.7
        self.HOME_Y = -6.65
        self.HOME_YAW = 0.0

        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # 延迟2秒发布，确保 AMCL 节点已经完全启动完毕并能够接收消息
        self.timer = self.create_timer(2.0, self.publish_initial_pose)
        self.get_logger().info("AED Initializer Node started. Will publish initial pose in 2 seconds...")

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.pose.position.x = float(self.HOME_X)
        msg.pose.pose.position.y = float(self.HOME_Y)
        msg.pose.pose.position.z = 0.0

        q = get_quaternion_from_euler(0.0, 0.0, float(self.HOME_YAW))
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # 协方差矩阵设定 (给予较小的误差范围以便快速收敛)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25   # X方差
        msg.pose.covariance[7] = 0.25   # Y方差
        msg.pose.covariance[35] = 0.06  # Yaw方差

        self.publisher_.publish(msg)
        self.get_logger().info(f"==> Published Initial Pose (HOME): X={self.HOME_X}, Y={self.HOME_Y}, YAW={self.HOME_YAW}")
        
        # 发布一次后即销毁定时器并退出节点
        self.timer.cancel()
        self.get_logger().info("Initial pose published successfully. Exiting initializer node.")
        
        # 优雅地通知系统关闭此单次运行的节点
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = AEDInitializerNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        # 捕获 SystemExit 以静默退出，这是设计的正常行为
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()