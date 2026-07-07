#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import subprocess
import threading
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

# 定义任务状态枚举
class MissionState(Enum):
    IDLE = 0            # 空闲，待命点监听
    NAV_TO_TARGET = 1   # 前往AED服务点
    WAITING = 2         # 在服务点等待30秒
    NAV_TO_HOME = 3     # 返回待命点

def get_quaternion_from_euler(roll, pitch, yaw):
    """欧拉角转四元数 (避免额外依赖)"""
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class AudioPlayer:
    """ffplay 无限循环播放报警音"""
    def __init__(self, wav_file='aed.wav'):
        self.wav_file=wav_file
        self.process=None
    def start(self):
        if self.process is not None and self.process.poll() is None:
            return
        if not os.path.exists(self.wav_file):
            return
        self.process=subprocess.Popen(
            ["ffplay","-nodisp","-autoexit","-loop","0",self.wav_file],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
    def stop(self):
        if self.process is not None:
            try:
                self.process.terminate()
                self.process.wait(timeout=1)
            except Exception:
                try:self.process.kill()
                except Exception:pass
            self.process=None


class AEDDispatcherNode(Node):
    def __init__(self):
        super().__init__('aed_dispatcher_node')

        # === 目标坐标配置 (基于已知地图) ===
        # 待命点 (HOME)
        self.HOME_X = 3.7
        self.HOME_Y = -6.65
        self.HOME_YAW = 0.0
        # 服务点 (TARGET)
        self.TARGET_X = 0.46
        self.TARGET_Y = -1.14
        self.TARGET_YAW = 0.0
        
        # 停留时间
        self.WAIT_TIME_SEC = 30.0

        # === 状态机控制变量 ===
        self._last_fall_state = False  
        self.mission_state = MissionState.IDLE
        self.wait_timer = None
        
        # === 报警音效播放器初始化 ===
        # 确保 aed.wav 与运行该脚本时的当前工作目录一致，或者你可以在这里改为绝对路径，如 '/home/user/aed.wav'
        self.audio_player = AudioPlayer('aed.wav')

        # === Nav2 接口初始化 ===
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 订阅跌倒事件
        self._subscription = self.create_subscription(Bool, '/fall_event', self.fall_event_callback, 10)

        self.get_logger().info('AED Dispatcher Node ready. Monitoring /fall_event in IDLE state.')

    def fall_event_callback(self, msg: Bool):
        """处理跌倒触发事件"""
        current_state = msg.data

        # 仅在 False -> True 且 状态为空闲 时触发
        if current_state and not self._last_fall_state:
            if self.mission_state == MissionState.IDLE:
                self.get_logger().info('=====================================')
                self.get_logger().info('[DISPATCH] Fall confirmed! Dispatching AED vehicle!')
                self.get_logger().info('=====================================')
                self.mission_state = MissionState.NAV_TO_TARGET
                
                # ====== 【安全要求1】车子出发前往目标点时，开始报警 ======
                self.audio_player.start()
                
                self.send_nav_goal(self.TARGET_X, self.TARGET_Y, self.TARGET_YAW)
            else:
                # ====== 【安全要求2】由于此处只在 IDLE 时处理跌倒，这就保证了在 30 秒等待时再次扫到跌倒，不仅不会派车，也绝对不会触发音效 ======
                self.get_logger().warn(f'Fall event received but vehicle is busy (State: { self.mission_state.name }). Ignored.')
        
        self._last_fall_state = current_state

    def send_nav_goal(self, x, y, yaw):
        """通用导航发送方法"""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('[NAV2] Action server not available! Aborting mission.')
            
            # 异常情况，立刻静音
            self.audio_player.stop()
            self.mission_state = MissionState.IDLE
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        q = get_quaternion_from_euler(0.0, 0.0, float(yaw))
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.get_logger().info(f'[NAV2] Sending AED vehicle to ({x}, {y}, yaw={yaw})')

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('[NAV2] Goal rejected by server!')
            
            # 目标被拒，立刻静音
            self.audio_player.stop()
            self.mission_state = MissionState.IDLE
            return

        self.get_logger().info('[NAV2] Goal accepted by server.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """处理导航最终结果并执行后续流程"""
        status = future.result().status

        # ====== 【安全要求3】无论车辆是到达了 reached 还是中途异常中断，第一时间停止报警音效 ======
        self.audio_player.stop()

        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.mission_state == MissionState.NAV_TO_TARGET:
                self.get_logger().info('[NAV2] AED vehicle arrived at target.')
                self.get_logger().info(f'[DISPATCH] Waiting for {self.WAIT_TIME_SEC} seconds...')
                
                # 进入等待状态，启动 30 秒倒计时器
                self.mission_state = MissionState.WAITING
                self.wait_timer = self.create_timer(self.WAIT_TIME_SEC, self.wait_timeout_callback)

            elif self.mission_state == MissionState.NAV_TO_HOME:
                self.get_logger().info('[NAV2] AED vehicle returned HOME successfully.')
                self.get_logger().info('[DISPATCH] Mission complete. System is IDLE and ready for next event.')
                # 流程走通，重置为空闲
                self.mission_state = MissionState.IDLE

        elif status == GoalStatus.STATUS_ABORTED: # 状态码 6
            self.get_logger().error(f'[NAV2] Goal failed with status code: {status} (ABORTED)')
            
            # 判断当前处于什么任务阶段失败的
            if self.mission_state == MissionState.NAV_TO_TARGET:
                self.get_logger().warn('[DISPATCH] Navigation to target failed! Cancelling current mission and returning HOME.')
                # 状态切换为返回 HOME
                self.mission_state = MissionState.NAV_TO_HOME
                # 发送返回 HOME 的目标点
                self.send_nav_goal(self.HOME_X, self.HOME_Y, self.HOME_YAW)
            elif self.mission_state == MissionState.NAV_TO_HOME:
                self.get_logger().error('[DISPATCH] Navigation failed while returning HOME. System requires manual intervention.')
                # 如果回家的路也走不通，就进入空闲状态，防止死循环
                self.mission_state = MissionState.IDLE
        else:
            self.get_logger().error(f'[NAV2] Goal failed with status code: {status}')
            # 如果是其它状态码导致的失败，重置为空闲以便人工干预或重新触发
            self.mission_state = MissionState.IDLE

    def wait_timeout_callback(self):
        """30秒倒计时结束的回调"""
        self.get_logger().info('[DISPATCH] Wait time is over. Returning HOME.')
        # 销毁定时器
        self.wait_timer.cancel()
        self.wait_timer = None
        
        # ====== 【安全要求4】30秒到了派车回家，不会触发 audio_player.start()，保证返回途中也保持安静 ======
        self.mission_state = MissionState.NAV_TO_HOME
        self.send_nav_goal(self.HOME_X, self.HOME_Y, self.HOME_YAW)


def main(args=None):
    rclpy.init(args=args)
    node = AEDDispatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down dispatcher.')
    finally:
        node.audio_player.stop()  # 异常退出时也能保证终止音效播放
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()