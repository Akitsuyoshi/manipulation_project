from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("name",package_name="real_moveit_config").to_moveit_configs()

    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
        arguments=[
            "-d", str(moveit_config.package_path / "config" / "moveit.rviz"),
        ],
    )

    return LaunchDescription([rviz_node])
