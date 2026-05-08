import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    # 声明启动参数（可通过命令行覆盖，方便灵活配置）
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='uwb rader dev'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value=TextSubstitution(text='115200'),
        description='baud_rate'
    )
    

    # 定义uwb节点
    uwb_serial_node = Node(
        package='uwb_serial_pub',          # 功能包名
        executable='talker',  # 可执行文件名称（对应CMakeLists.txt中的target）
        name='uwb_serial',    # 节点名称（可覆盖代码中定义的节点名）
        output='screen',           # 输出日志到终端
        emulate_tty=True,          # 确保日志输出格式正常

        
        # 若需要向节点传递参数（代码中需读取ROS参数），可添加parameters配置
        parameters=[
            {'serial_port': LaunchConfiguration('serial_port')},
            {'baud_rate': LaunchConfiguration('baud_rate')},
        ]
    )

    # 组装启动描述
    ld = LaunchDescription()
    # 添加参数声明
    ld.add_action(serial_port_arg)
    ld.add_action(baud_rate_arg)
    # 添加节点
    ld.add_action(uwb_serial_node)

    return ld