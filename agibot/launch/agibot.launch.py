import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    # 声明启动参数（可通过命令行覆盖，方便灵活配置）
    local_ip_arg = DeclareLaunchArgument(
        'local_ip',
        default_value=TextSubstitution(text='192.168.1.136'),
        description='本地IP地址，用于和机器狗建立连接'
    )
    
    local_port_arg = DeclareLaunchArgument(
        'local_port',
        default_value=TextSubstitution(text='43988'),
        description='本地端口号，用于和机器狗建立连接'
    )
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value=TextSubstitution(text='192.168.1.136'),
        description='机器狗的IP地址'
    )

    # 定义agibot节点
    agibot_node = Node(
        package='agibot',          # 功能包名
        executable='agibot_move_sub',  # 可执行文件名称（对应CMakeLists.txt中的target）
        name='agibot_move_sub',    # 节点名称（可覆盖代码中定义的节点名）
        output='screen',           # 输出日志到终端
        emulate_tty=True,          # 确保日志输出格式正常

        
        # 若需要向节点传递参数（代码中需读取ROS参数），可添加parameters配置
        parameters=[
            {'local_ip': LaunchConfiguration('local_ip')},
            {'local_port': LaunchConfiguration('local_port')},
            {'robot_ip': LaunchConfiguration('robot_ip')}
        ]
    )

    # 组装启动描述
    ld = LaunchDescription()
    # 添加参数声明
    ld.add_action(local_ip_arg)
    ld.add_action(local_port_arg)
    ld.add_action(robot_ip_arg)
    # 添加节点
    ld.add_action(agibot_node)

    return ld