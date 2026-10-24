"""moveit_master for the FR3 duo.

Needs move_group running with the same robot model (URDF + SRDF with the groups and links named in
config/moveit_master.yaml). MoveGroupInterface only needs robot_description and
robot_description_semantic on this node; planning, IK and execution happen in move_group.

  ros2 launch moveit_master moveit_sequence.launch.py
  ros2 launch moveit_master moveit_sequence.launch.py urdf:=<file.urdf.xacro> srdf:=<file.srdf.xacro> \
      joint_limits:=<joint_limits.yaml>

For the RViz mock test use fr3_duo_bringup/launch/rviz_mock.launch.py: it starts moveit_master itself.
"""
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robots = os.path.join(get_package_share_directory("franka_description"), "robots", "mobile_fr3_duo_v0_2")
    args = [
        DeclareLaunchArgument("urdf", default_value=os.path.join(robots, "mobile_fr3_duo_v0_2.urdf.xacro")),
        DeclareLaunchArgument("srdf", default_value=os.path.join(robots, "mobile_fr3_duo_v0_2.srdf.xacro")),
        DeclareLaunchArgument("xacro_args", default_value="",
                              description="extra xacro args for the URDF, e.g. 'ee_id:=robotiq_2f_85'"),
        DeclareLaunchArgument("params", default_value=os.path.join(
            get_package_share_directory("moveit_master"), "config", "moveit_master.yaml")),
        DeclareLaunchArgument("joint_limits", default_value="",
                              description="MoveIt joint_limits.yaml (acceleration limits): needed to "
                                          "re-time ARM_CARTESIAN moves"),
    ]
    xacro = FindExecutable(name="xacro")
    description = {
        "robot_description": ParameterValue(
            Command([xacro, " ", LaunchConfiguration("urdf"), " ", LaunchConfiguration("xacro_args")]),
            value_type=str),
        "robot_description_semantic": ParameterValue(
            Command([xacro, " ", LaunchConfiguration("srdf")]), value_type=str),
    }

    def master(context):
        limits_file = LaunchConfiguration("joint_limits").perform(context)
        limits = {}
        if limits_file:
            with open(limits_file) as f:
                limits = {"robot_description_planning": yaml.safe_load(f)}
        return [Node(package="moveit_master", executable="moveit_master", output="screen",
                     parameters=[description, limits, LaunchConfiguration("params")])]

    return LaunchDescription(args + [OpaqueFunction(function=master)])
