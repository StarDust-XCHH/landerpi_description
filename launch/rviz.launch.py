import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch.events import Shutdown
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from launch.event_handlers import OnProcessExit
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler


def generate_launch_description():
    # === 自动确定 landerpi_description 包路径 ===
    try:
        # 优先：已安装的包（colcon build 后）
        landerpi_description_package_path = get_package_share_directory('landerpi_description')
    except Exception:
        # 回退：源码路径（相对于本 launch 文件）
        launch_file_dir = Path(__file__).parent
        landerpi_description_package_path = str(launch_file_dir.parent)
        # 验证关键文件是否存在
        rviz_test_path = Path(landerpi_description_package_path) / 'rviz' / 'view.rviz'
        if not rviz_test_path.exists():
            raise FileNotFoundError(
                f"RViz config 'view.rviz' not found in {landerpi_description_package_path}. "
                "Please ensure the package is built or the source structure is correct."
            )

    # === Launch 配置变量 ===
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    rviz_config_file = LaunchConfiguration('rviz_config')

    # === 声明 launch 参数 ===
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description=('Top-level namespace. The value will be used to replace the '
                     '<robot_namespace> keyword on the rviz config file.'))

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(landerpi_description_package_path, 'rviz', 'view.rviz'),
        description='Full path to the RVIZ config file to use')

    # === 启动 RViz（无命名空间）===
    start_rviz_cmd = Node(
        condition=UnlessCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', ReplaceString(
            source_file=rviz_config_file,
            replacements={'robot_1/': '', 'robot_1': ''})],
        output='screen')

    # === 启动 RViz（带命名空间）===
    start_namespaced_rviz_cmd = Node(
        condition=IfCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', ReplaceString(
            source_file=rviz_config_file,
            replacements={'<robot_namespace>': ('/', namespace), 'robot_1': namespace})],
        output='screen',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/goal_pose', 'goal_pose'),
            ('/clicked_point', 'clicked_point'),
            ('/initialpose', 'initialpose')
        ])

    # === 退出事件处理器（无命名空间）===
    exit_event_handler = RegisterEventHandler(
        condition=UnlessCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    # === 退出事件处理器（带命名空间）===
    exit_event_handler_namespaced = RegisterEventHandler(
        condition=IfCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_namespaced_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    # === 构建 LaunchDescription ===
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(start_rviz_cmd)
    ld.add_action(start_namespaced_rviz_cmd)

    ld.add_action(exit_event_handler)
    ld.add_action(exit_event_handler_namespaced)

    return ld


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
