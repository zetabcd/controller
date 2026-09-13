from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

# sun: 启动仿真节点并加载仿真专用参数；控制器节点由 px4ctrl 的 launch 文件单独启动。

def generate_launch_description():
    # 获取参数文件的路径
    config = os.path.join(
        get_package_share_directory('quadsim_mujoco'),  # 替换为您的包名
        'config',
        'params.yaml'
    )  
    return LaunchDescription([          # 返回launch文件的描述信息
        Node(             
            package='quadsim_mujoco',       # 节点所在的功能包
            executable='quadsim_node',      # 节点的可执行文件名
            name='quadsim_node',            # 对节点重新命名
            parameters=[config],             # 加载参数文件
            output='screen'
        )
        # Node(             
        #     package='quadsim_mujoco',       # 节点所在的功能包
        #     executable='quadplot_node',      # 节点的可执行文件名
        #     name='quadplot_node',            # 对节点重新命名
        #     output='screen'
        # )
    ])

    
