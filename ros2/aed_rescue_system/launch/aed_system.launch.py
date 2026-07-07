from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 启动开机自定位节点 (等待2秒后自动向AMCL发布位姿)
        Node(
            package='aed_rescue_system',
            executable='aed_initializer_node',
            name='aed_initializer_node',
            output='screen',
            emulate_tty=True,
        ),

        # 2. 启动 AED 导航调度节点 (驻留内存等待调度)
        Node(
            package='aed_rescue_system',
            executable='aed_dispatcher_node',
            name='aed_dispatcher_node',
            output='screen',
            emulate_tty=True,
        ),

        # 3. 启动边缘端跌倒检测节点 (RK3588 NPU 实时推理)
        Node(
            package='aed_rescue_system',
            executable='aed_fall_detection_node',
            name='aed_fall_detection_node',
            output='screen',
            emulate_tty=True,
        )
    ])