"""FR3 duo + Robotiq 2F-85 in RViz on mock hardware: test the moveit_master pipeline without Isaac.

Starts robot_state_publisher, ros2_control (mock_components/GenericSystem for every movable joint),
one trajectory controller per arm and per gripper, move_group, RViz and moveit_master. The arms
start at left_home / right_home, grippers open. Then, in another terminal:

  ros2 run articulation_orchestrator articulation_orchestrator        # smoke test
  ros2 run articulation_orchestrator run_placements --count 2         # packets -> tray

  ros2 launch fr3_duo_bringup rviz_mock.launch.py [rviz:=false] [moveit_master:=false] [spine:=0.15]
"""
import os
import xml.etree.ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "fr3_duo_bringup"
ARM_JOINTS = [f"{s}_fr3v2_joint{i}" for s in ("left", "right") for i in range(1, 8)]
GRIPPER_JOINTS = [f"{s}_robotiq_85_left_knuckle_joint" for s in ("left", "right")]


def share(*path):
    return os.path.join(get_package_share_directory(PKG), *path)


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def home_values(srdf):
    """{joint: value} of every <group_state name="*_home"> in the SRDF."""
    values = {}
    for gs in ET.fromstring(srdf).findall("group_state"):
        if gs.get("name").endswith("_home"):
            values.update({j.get("name"): float(j.get("value")) for j in gs.findall("joint")})
    return values


def add_mock_ros2_control(urdf, initial):
    """Append a mock_components ros2_control block with every movable, non-mimic joint.
    Arm and gripper joints get a position command interface; the rest (planar base, spine,
    wheels) only states, so TF and MoveIt see a complete robot state."""
    robot = ET.fromstring(urdf)
    rc = ET.SubElement(robot, "ros2_control", name="MockSystem", type="system")
    hw = ET.SubElement(rc, "hardware")
    ET.SubElement(hw, "plugin").text = "mock_components/GenericSystem"
    for j in robot.findall("joint"):
        if j.get("type") == "fixed" or j.find("mimic") is not None:
            continue
        name = j.get("name")
        je = ET.SubElement(rc, "joint", name=name)
        if name in ARM_JOINTS or name in GRIPPER_JOINTS:
            ET.SubElement(je, "command_interface", name="position")
        pos = ET.SubElement(je, "state_interface", name="position")
        ET.SubElement(pos, "param", name="initial_value").text = str(initial.get(name, 0.0))
        ET.SubElement(je, "state_interface", name="velocity")
    return ET.tostring(robot, encoding="unicode")


def setup(context):
    use_rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"
    use_master = LaunchConfiguration("moveit_master").perform(context).lower() == "true"
    spine = float(LaunchConfiguration("spine").perform(context))

    urdf = xacro.process_file(share("urdf", "fr3_duo_robotiq.urdf.xacro"),
                              mappings={"tcp_offset": LaunchConfiguration("tcp_offset").perform(context)}).toxml()
    srdf = xacro.process_file(share("srdf", "fr3_duo_robotiq.srdf.xacro")).toxml()
    initial = {**home_values(srdf), "franka_spine_vertical_joint": spine}
    urdf = add_mock_ros2_control(urdf, initial)

    description = {"robot_description": urdf, "robot_description_semantic": srdf}
    kinematics = {"robot_description_kinematics": load_yaml(share("config", "kinematics.yaml"))}
    limits = {"robot_description_planning": load_yaml(share("config", "joint_limits.yaml"))}
    with open(share("config", "moveit_defaults.json")) as f:
        import json
        defaults = json.load(f)
    pipelines = {k: defaults[k] for k in ("planning_pipelines", "default_planning_pipeline", "ompl")}

    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[description, kinematics, limits, defaults, load_yaml(share("config", "moveit_controllers.yaml")),
                    {"publish_robot_description_semantic": True, "publish_planning_scene": True,
                     "publish_geometry_updates": True, "publish_state_updates": True,
                     "publish_transforms_updates": True, "allow_trajectory_execution": True,
                     "capabilities": "", "disable_capabilities": ""}])

    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": urdf}]),
        # planning frame: the SRDF virtual joint world -> base
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["--frame-id", "world", "--child-frame-id", "base"]),
        Node(package="controller_manager", executable="ros2_control_node", output="screen",
             parameters=[share("config", "ros2_controllers.yaml")],
             remappings=[("~/robot_description", "/robot_description")]),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "left_arm_controller", "right_arm_controller",
                        "left_gripper_controller", "right_gripper_controller",
                        "--controller-manager-timeout", "60"]),
        move_group,
    ]
    if use_rviz:
        nodes.append(Node(package="rviz2", executable="rviz2", output="screen",
                          arguments=["-d", share("rviz", "moveit.rviz")],
                          parameters=[description, kinematics, limits, pipelines]))
    if use_master:
        master_params = os.path.join(get_package_share_directory("moveit_master"), "config", "moveit_master.yaml")
        nodes.append(TimerAction(period=5.0, actions=[Node(
            package="moveit_master", executable="moveit_master", output="screen",
            parameters=[description, kinematics, limits, master_params, {"use_sim_time": False}])]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("moveit_master", default_value="true"),
        DeclareLaunchArgument("spine", default_value="0.15",
                              description="spine height (m), 0 .. 0.85; the default scene is laid out for 0.15"),
        DeclareLaunchArgument("tcp_offset", default_value="0.13",
                              description="TCP above robotiq_85_base_link (m)"),
        OpaqueFunction(function=setup),
    ])
