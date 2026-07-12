import os
import re

import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def load_yaml(package_name, file_path):
    path = os.path.join(
        get_package_share_directory(package_name),
        file_path
    )

    with open(path) as f:
        return yaml.safe_load(f)


def build_descriptions(load_gripper):

    scene_cfg = os.path.join(
        get_package_share_directory(
            "franka_warehouse_world"
        ),
        "config",
        "scene.yaml",
    )

    with open(scene_cfg) as f:
        mount_height = yaml.safe_load(f)["table"]["height"]

    gazebo_xacro = os.path.join(
        get_package_share_directory(
            "franka_gazebo_bringup"
        ),
        "urdf",
        "franka_arm.gazebo.xacro",
    )

    urdf = xacro.process_file(
        gazebo_xacro,
        mappings={
            "robot_type": "fr3",
            "hand": load_gripper,
            "gazebo": "true",
            "ee_id": "franka_hand",
            "gazebo_effort": "true",
            "xyz": f"0 0 {mount_height}",
        },
    ).toxml()

    srdf_xacro = os.path.join(
        get_package_share_directory(
            "franka_description"
        ),
        "robots",
        "fr3",
        "fr3.srdf.xacro",
    )

    srdf = xacro.process_file(
        srdf_xacro,
        mappings={
            "hand": load_gripper,
            "ee_id": "franka_hand",
        },
    ).toxml()

    srdf = re.sub(
        r"<virtual_joint\b[^>]*/>",
        "",
        srdf,
    )

    return (
        {"robot_description": urdf},
        {"robot_description_semantic": srdf},
    )


def launch_setup(context: LaunchContext):

    load_gripper = context.perform_substitution(
        LaunchConfiguration("load_gripper")
    )

    robot_description, robot_description_semantic = \
        build_descriptions(load_gripper)

    kinematics_yaml = load_yaml(
        "franka_fr3_moveit_config",
        "config/kinematics.yaml",
    )

    executor = Node(
        package="franka_manipulation",
        executable="executor_node",
        name="executor",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {"use_sim_time": True},
        ],
    )

    return [executor]


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            "load_gripper",
            default_value="true",
        ),

        OpaqueFunction(
            function=launch_setup
        ),
    ])