# Launch三大组件：动作、条件和替换
# 动作：除了是一个节点外，还可以是一句打印，一段终端指令，甚至是另外一个launch文件
# 替换：使用launch的参数替换节点的参数值
# 条件：利用条件可以决定哪些动作启动，那些不启动，相当于if
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('decision')

    # 参数文件路径
    decision_params_file = os.path.join(pkg_share, 'param', 'settings_param.yaml')

    declare_decision_params_file = DeclareLaunchArgument(
        'decision_params_file',
        default_value=decision_params_file,
        description='Path to the parameters file'
    )

    decision_node = Node(
        package='decision',
        executable='main',
        name='decision_node',
        output='screen',
        parameters=[LaunchConfiguration('decision_params_file')],
        # 这个选项用于模拟一个终端（tty），使得节点的输出能够以行缓冲（line-buffered）的方式实时显示。
        # 这通常用于需要实时输出（例如，打印日志）的节点，以确保输出及时显示而不会因为缓冲而延迟。
        emulate_tty=True
    )

    return launch.LaunchDescription([
        # actions动作
        declare_decision_params_file,
        decision_node
    ])