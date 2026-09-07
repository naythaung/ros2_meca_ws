import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    scene_package = get_package_share_directory(
        "meca500_scene_description"
    )
    moveit_package = get_package_share_directory(
        "meca500_moveit_config"
    )

    scene_file = os.path.join(
        scene_package, "urdf", "scene.urdf.xacro"
    )

    with open(
        os.path.join(moveit_package, "config", "meca500.srdf")
    ) as file:
        semantic_description = file.read()

    with open(
        os.path.join(moveit_package, "config", "kinematics.yaml")
    ) as file:
        kinematics = yaml.safe_load(file)

    with open(
        os.path.join(moveit_package, "config", "joint_limits.yaml")
    ) as file:
        joint_limits = yaml.safe_load(file)

    return LaunchDescription([
        Node(
            package="meca500_tasks",
            executable="move_to_pose",
            output="screen",
            parameters=[
                {
                    "robot_description": ParameterValue(
                        Command([
                            FindExecutable(name="xacro"),
                            " ",
                            scene_file,
                        ]),
                        value_type=str,
                    ),
                    "robot_description_semantic": semantic_description,
                    "robot_description_kinematics": kinematics,
                    "robot_description_planning": joint_limits,
                    "use_sim_time": False,
                }
            ],
        )
    ])