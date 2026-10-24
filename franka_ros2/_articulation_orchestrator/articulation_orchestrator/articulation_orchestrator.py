#!/usr/bin/env python3
"""articulation_orchestrator: high-level logic.

Decides HOW an arm moves a packet into the tray: builds the list of SubMovements for one packet and
sends it to moveit_master (C++) via the action "moveit_master/execute_sequence". Which arm does a
packet is decided by ArmAllocator; which packets and where they go comes from the scene YAML
(run_placements.py ties them together).

Gripper: Robotiq 2F-85, top-down grasps. TCP frame (URDF link `{arm}_robotiq_85_tcp`): between the
fingertip pads, z = approach direction (out of the gripper), y = closing direction. If the URDF TCP
closes along x instead, set tcp_yaw_offset to -pi/2.

  ros2 run articulation_orchestrator articulation_orchestrator   # smoke test: home + open/close both grippers
"""
import json
import math
import os
import threading

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

from moveit_master.action import ExecuteSequence
from articulation_orchestrator.scene import Packet, Scene
from articulation_orchestrator.steps import (
    allow_collision, arm_named, arm_pose, attach, cartesian, detach, gripper_joint, make_pose,
    top_down_quat, wait, wrap_angle)

ROBOTIQ_CLOSED = 0.8        # rad, knuckle joint upper limit (0 = fully open, 85 mm)
ROBOTIQ_STROKE = 0.085      # m, opening at 0 rad
# Lowest fingertip point below robotiq_85_base_link (m) vs knuckle angle (rad), from the
# collision meshes: the tips move down while the gripper closes.
ROBOTIQ_TIP_Z = ((0.0, 0.1493), (0.1, 0.1528), (0.2, 0.1559), (0.3, 0.1584), (0.4, 0.1604),
                 (0.5, 0.1618), (0.6, 0.1627), (0.7, 0.1631), (0.8, 0.1631))


def _wait_future(future):
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    done.wait()
    return future.result()


class ArticulationOrchestrator(Node):
    def __init__(self):
        super().__init__("articulation_orchestrator")
        self._client = ActionClient(self, ExecuteSequence, "moveit_master/execute_sequence")
        # what is running: {"task", "arm", "object", "labels"}, for execution_logger
        self._stage_pub = self.create_publisher(String, "articulation_orchestrator/stage", 10)

        share = get_package_share_directory("articulation_orchestrator")
        self.declare_parameter("objects_dictionary", os.path.join(share, "assets", "objects_dictionary.yaml"))
        self.declare_parameter("home_pose_format", "{arm}_home")   # SRDF named state per arm group
        self.declare_parameter("return_home", True)                # arm home after every packet
        self.declare_parameter("tcp_yaw_offset", 0.0)              # rad, see module doc
        self.declare_parameter("tcp_offset", 0.13)                 # m, TCP above robotiq_85_base_link (URDF)
        self.declare_parameter("tcp_down_rpy", [math.pi, 0.0, 0.0])  # TCP pointing down, yaw 0
        # parallel gripper: TCP yaw and yaw + pi are the same grasp; prefer the one nearest this
        # per arm (less wrist turning). Defaults: the TCP yaw at {arm}_home (FR3 "ready").
        self.declare_parameter("preferred_tcp_yaw.left", -math.pi / 2)
        self.declare_parameter("preferred_tcp_yaw.right", math.pi / 2)

    def _param(self, name):
        return self.get_parameter(name).value

    # ---------- objects dictionary ----------
    def object_spec(self, obj_type) -> dict:
        """defaults + objects[obj_type] from objects_dictionary.yaml (re-read every call)."""
        with open(self._param("objects_dictionary")) as f:
            d = yaml.safe_load(f)
        if obj_type not in d["objects"]:
            raise KeyError(f"'{obj_type}' not in objects_dictionary.yaml")
        spec = {**d.get("defaults", {}), **(d["objects"][obj_type] or {})}
        width = spec["size"][1 if spec.get("grasp_axis", "y") == "y" else 0]
        if width > spec.get("max_opening", ROBOTIQ_STROKE) - 0.005:
            raise ValueError(f"'{obj_type}' is {width * 1000:.0f} mm across the grasp axis: "
                             "too wide for the Robotiq 2F-85")
        if spec.get("gripper_close") is None:
            spec["gripper_close"] = close_position(width, spec.get("squeeze", 0.15))
        if spec.get("gripper_release") is None:
            # open only a bit wider than the packet: fully open fingers hit the tray walls
            spec["gripper_release"] = close_position(min(width + spec.get("release_margin", 0.02),
                                                         ROBOTIQ_STROKE), 0.0)
        return spec

    # ---------- grasp geometry ----------
    def tcp_yaw(self, arm, closing_yaw, near=None):
        """TCP yaw whose closing axis (TCP y) lies along world direction `closing_yaw`.
        Of the two equivalent yaws, the one nearest `near` (default: the arm's preferred yaw)."""
        yaw = wrap_angle(closing_yaw + math.pi / 2 + self._param("tcp_yaw_offset"))
        ref = self._param(f"preferred_tcp_yaw.{arm}") if near is None else near
        alt = wrap_angle(yaw + math.pi)
        return yaw if abs(wrap_angle(yaw - ref)) <= abs(wrap_angle(alt - ref)) else alt

    def key_poses(self, arm, packet: Packet, scene: Scene, slot: int) -> dict:
        """TCP poses of one packet transfer (world): pre_grasp, grasp, place_hover, place, plus
        the heights and the spec they come from."""
        o = self.object_spec(packet.type)
        down = tuple(self._param("tcp_down_rpy"))
        lx, ly, h = packet.size
        top = packet.z + h / 2

        # pick: fingers close across the grasp axis of the packet
        closing = packet.yaw + (math.pi / 2 if o.get("grasp_axis", "y") == "y" else 0.0)
        pick_yaw = self.tcp_yaw(arm, closing)
        # keep the fingertips off the table, also once closed (they move down while closing)
        tip = tip_below_tcp(o["gripper_close"], self._param("tcp_offset"))
        grasp_z = max(top - o["grasp_depth"], scene.table.top_z + tip + o["min_table_clearance"])
        hang = grasp_z - (packet.z - h / 2)        # packet bottom below the TCP while carried

        # place: packet lying along the tray, on the lowest stack of `slot`
        tray = scene.tray
        sx, sy = tray.slot_xy(slot)
        place_yaw = tray.yaw + (math.pi / 2 if o.get("grasp_axis", "y") == "y" else 0.0)
        place_tcp_yaw = self.tcp_yaw(arm, place_yaw, near=pick_yaw)
        place_z = tray.floor_z + tray.stack[slot] + o["place_drop"] + hang
        hover_z = max(tray.rim_z, tray.floor_z + tray.stack[slot]) + hang + o["place_hover_clearance"]

        return {
            "spec": o,
            "pre_grasp": make_pose(packet.x, packet.y, top + o["approach_height"], top_down_quat(pick_yaw, down)),
            "grasp_z": grasp_z,
            "lift": o["lift_height"],
            "place_hover": make_pose(sx, sy, hover_z, top_down_quat(place_tcp_yaw, down)),
            "place_z": place_z,
            "place_xy": (sx, sy),
            "place_packet_yaw": tray.yaw,
            "place_packet_z": place_z - hang + h / 2,
        }

    # ---------- sequences ----------
    def pick_place_steps(self, arm, packet: Packet, scene: Scene, slot: int, k=None) -> list:
        """Pick `packet` from the table with `arm`, put it into tray slot `slot`.

        Collisions: the fingers may touch the packet from the approach on; while it is attached it
        may touch the table (pick) and the tray + packets already in the slot (place)."""
        k = k or self.key_poses(arm, packet, scene, slot)
        o, pid = k["spec"], packet.id
        pre = k["pre_grasp"]
        descend = pre.position.z - k["grasp_z"]
        below = [scene.tray.id, *scene.tray.contents[slot]]
        hover = k["place_hover"]
        lower = hover.position.z - k["place_z"]
        home = self._param("home_pose_format").format(arm=arm)
        settle = o.get("settle_time", 0.0)

        steps = [
            gripper_joint(arm, o["gripper_open"], label="pick: open"),
            arm_pose(arm, pre, label="pick: pre-grasp"),
            allow_collision(arm, pid, [scene.table.id], label="pick: allow packet"),
            cartesian(arm, dz=-descend, label="pick: descend"),
            gripper_joint(arm, o["gripper_close"], label="pick: close"),
        ]
        if settle:
            steps.append(wait(settle, label="pick: settle"))
        steps += [
            attach(arm, pid, label="pick: attach"),
            cartesian(arm, dz=o["lift_height"], label="pick: lift"),
            allow_collision(arm, pid, [scene.table.id], allow=False, label="pick: forbid table"),
            arm_pose(arm, hover, label="place: hover"),
            allow_collision(arm, pid, below, label="place: allow tray"),
            cartesian(arm, dz=-lower, label="place: lower"),
            gripper_joint(arm, o["gripper_release"], label="place: release"),
        ]
        if settle:
            steps.append(wait(settle, label="place: settle"))
        steps += [
            detach(arm, pid, label="place: detach"),
            cartesian(arm, dz=lower, label="place: retreat"),
            # the gripper is clear of the packet now; packet-tray contacts are world-world
            allow_collision(arm, pid, below, allow=False, label="place: forbid"),
        ]
        if self._param("return_home"):
            steps.append(arm_named(arm, home, label="home"))
        return steps

    def recover_steps(self, arm, spec=None) -> list:
        """After a failed packet (no attached objects remain: moveit_master rolled the scene back):
        open to the release width, move up, go home. Run them one by one (run_placements): the
        first two may fail where the gripper is stuck, only reaching home matters."""
        steps = []
        if spec is not None:
            steps.append(gripper_joint(arm, spec["gripper_release"], label="recover: open"))
            steps.append(cartesian(arm, dz=spec["lift_height"], label="recover: up"))
        return steps + [arm_named(arm, self._param("home_pose_format").format(arm=arm), label="recover: home")]

    # ---------- execution ----------
    def wait_for_master(self):
        self.get_logger().info("waiting for moveit_master ...")
        self._client.wait_for_server()

    def publish_stage(self, **stage):
        self._stage_pub.publish(String(data=json.dumps(stage)))

    def run_sequence(self, steps, velocity_scaling=0.0):
        """Blocking: returns (success, failed_step, message)."""
        goal = ExecuteSequence.Goal(steps=steps, velocity_scaling=velocity_scaling)
        handle = _wait_future(self._client.send_goal_async(goal))
        if not handle.accepted:
            return False, -1, "rejected (moveit_master busy)"
        res = _wait_future(handle.get_result_async()).result
        if not res.success:
            return False, int(res.failed_step), f"step {res.failed_step}: {res.message}"
        return True, -1, res.message


def close_position(width, squeeze=0.15):
    """Robotiq 2F-85 knuckle angle a bit past contact with an object `width` across.
    Approximation: opening linear in the knuckle angle (85 mm at 0, ~0 at 0.8 rad)."""
    contact = ROBOTIQ_CLOSED * (1.0 - width / ROBOTIQ_STROKE)
    return min(max(contact + squeeze, 0.0), ROBOTIQ_CLOSED)


def tip_below_tcp(knuckle, tcp_offset=0.13):
    """How far the Robotiq fingertips reach below the TCP at knuckle angle `knuckle` (m)."""
    q = min(max(knuckle, 0.0), ROBOTIQ_CLOSED)
    for (q0, z0), (q1, z1) in zip(ROBOTIQ_TIP_Z, ROBOTIQ_TIP_Z[1:]):
        if q <= q1:
            return z0 + (z1 - z0) * (q - q0) / (q1 - q0) - tcp_offset
    return ROBOTIQ_TIP_Z[-1][1] - tcp_offset


def main():
    """Smoke test (plan step 3): both arms home, each gripper opens and closes."""
    rclpy.init()
    node = ArticulationOrchestrator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    node.wait_for_master()
    steps = []
    for arm in ("left", "right"):
        steps += [arm_named(arm, node._param("home_pose_format").format(arm=arm)),
                  gripper_joint(arm, 0.0, label="open"), wait(0.5),
                  gripper_joint(arm, 0.6, label="close"), wait(0.5),
                  gripper_joint(arm, 0.0, label="open")]
    ok, _, msg = node.run_sequence(steps, velocity_scaling=0.2)
    node.get_logger().info(f"smoke test {'succeeded' if ok else 'failed'}: {msg}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
