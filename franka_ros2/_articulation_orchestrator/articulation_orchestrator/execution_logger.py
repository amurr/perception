#!/usr/bin/env python3
"""execution_logger: record both arms in the world frame while moveit_master sequences run.

One run per moveit_master/execute_sequence goal (= one packet, or a homing / recovery sequence).
The task comes from articulation_orchestrator/stage (published by run_placements just before the
goal). Each run is a directory <out_dir>/<YYYYmmdd_HHMMSS>_<packet>/ with:

- arm.jsonl    samples at rate_hz: joint positions + world-frame pose of every link in `links`,
               tagged with the moveit_master step and its label; plus events (goal start/end,
               step changes)
- scene.json   PlanningScene world objects (table, tray, packet_*) at run start
- steps.csv    one row per step, taken when it ended: arm, label, TCP position of that arm,
               gripper knuckle angle, the packet's planning-scene position at run start

  ros2 run articulation_orchestrator execution_logger
  ros2 run articulation_orchestrator execution_logger --ros-args -p out_dir:=/tmp/runs -p rate_hz:=50
"""
import csv
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from action_msgs.msg import GoalStatus, GoalStatusArray
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene

from moveit_master.action import ExecuteSequence

ARMS = ("left", "right")
DEFAULT_LINKS = [f"{a}_{l}" for a in ARMS for l in
                 ("fr3v2_1_link7", "fr3v2_1_link8", "robotiq_85_base_link", "robotiq_85_tcp")]
TERMINAL = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_CANCELED: "canceled",
            GoalStatus.STATUS_ABORTED: "aborted"}
STAGE_MAX_AGE = 10.0    # s: a stage message older than this does not belong to a new goal


def rnd(v, n=5):
    return [round(float(x), n) for x in v]


class ExecutionLogger(Node):
    def __init__(self):
        super().__init__("execution_logger")
        self.declare_parameter("out_dir", "/tmp/packet_exec_logs")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("links", DEFAULT_LINKS)
        self.declare_parameter("tcp_link_format", "{arm}_robotiq_85_tcp")
        self.declare_parameter("gripper_joint_format", "{arm}_robotiq_85_left_knuckle_joint")

        self.tf = Buffer()
        self._tf_listener = TransformListener(self.tf, self)
        self.joints = {}
        self.run = None
        self.stage, self.stage_t = None, 0.0

        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 50)
        self.create_subscription(String, "/articulation_orchestrator/stage", self._on_stage, 10)
        self.create_subscription(GoalStatusArray, "/moveit_master/execute_sequence/_action/status",
                                 self._on_status, status_qos)
        self.create_subscription(ExecuteSequence.Impl.FeedbackMessage,
                                 "/moveit_master/execute_sequence/_action/feedback", self._on_step, 10)
        self._scene_client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.create_timer(1.0 / max(self._p("rate_hz"), 1.0), self._sample)
        self.get_logger().info(f"execution_logger ready, runs go to {self._p('out_dir')}")

    def _p(self, name):
        return self.get_parameter(name).value

    # ---------- inputs ----------
    def _on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def _on_stage(self, msg):
        try:
            self.stage, self.stage_t = json.loads(msg.data), time.time()
        except ValueError:
            self.get_logger().warn(f"bad stage message: {msg.data!r}")

    def _on_status(self, msg):
        for st in msg.status_list:
            gid = bytes(st.goal_info.goal_id.uuid).hex()
            if self.run is None and st.status == GoalStatus.STATUS_EXECUTING:
                self._start(gid)
            elif self.run and self.run["goal"] == gid and st.status in TERMINAL:
                self._finish(TERMINAL[st.status])

    def _on_step(self, msg):
        if not self.run:
            return
        self._close_step()
        r = self.run
        r["step"], r["label"], r["step_start"] = int(msg.feedback.current_step), msg.feedback.current_label, time.time()
        self._event("step", step=r["step"], label=r["label"])

    # ---------- run lifecycle ----------
    def _start(self, gid):
        stage = self.stage if self.stage and time.time() - self.stage_t < STAGE_MAX_AGE else {}
        self.stage = None                     # one stage per goal
        name = time.strftime("%Y%m%d_%H%M%S") + (f"_{stage['packet']}" if stage.get("packet") else "")
        d = Path(self._p("out_dir")) / name
        d.mkdir(parents=True, exist_ok=True)
        self.run = {"goal": gid, "dir": d, "stage": stage, "step": None, "label": "",
                    "step_start": None, "rows": [], "scene": None, "arm_file": (d / "arm.jsonl").open("w")}
        self._event("goal_start", goal_id=gid, stage=stage)
        self._request_scene()
        self.get_logger().info(f"recording goal {gid[:8]} -> {d}")

    def _finish(self, outcome):
        self._close_step()
        self._event("goal_end", outcome=outcome)
        r, self.run = self.run, None
        r["arm_file"].close()
        self._write_steps(r)
        self.get_logger().info(f"goal {outcome}: {r['dir']} ({len(r['rows'])} steps)")

    # ---------- sampling ----------
    def _link_poses(self):
        out, world = {}, self._p("world_frame")
        for link in self._p("links"):
            try:
                t = self.tf.lookup_transform(world, link, Time()).transform
            except Exception:
                continue
            out[link] = {"xyz": rnd((t.translation.x, t.translation.y, t.translation.z)),
                         "wxyz": rnd((t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z))}
        return out

    def _sample(self):
        if not self.run:
            return
        r = self.run
        self._write({"type": "sample", "t": round(time.time(), 4),
                     "ros_t": self.get_clock().now().nanoseconds * 1e-9,
                     "step": r["step"], "label": r["label"],
                     "joints": {k: round(v, 5) for k, v in sorted(self.joints.items())},
                     "links": self._link_poses()})

    def _event(self, kind, **data):
        self._write({"type": "event", "event": kind, "t": round(time.time(), 4), **data})

    def _write(self, rec):
        self.run["arm_file"].write(json.dumps(rec) + "\n")
        self.run["arm_file"].flush()

    def _close_step(self):
        """Store where the step's arm is at the end of the running step (for steps.csv)."""
        r = self.run
        if not r or r["step"] is None:
            return
        arm = r["stage"].get("arm", "")
        links = self._link_poses()
        r["rows"].append({
            "t_start": r["step_start"], "t_end": time.time(), "step": r["step"], "label": r["label"],
            "arm": arm,
            "tcp": links.get(self._p("tcp_link_format").format(arm=arm), {}).get("xyz") if arm else None,
            "gripper": self.joints.get(self._p("gripper_joint_format").format(arm=arm)) if arm else None,
        })
        r["step"] = None

    # ---------- PlanningScene ----------
    def _request_scene(self):
        if not self._scene_client.service_is_ready():
            self.get_logger().warn("/get_planning_scene not available, scene.json skipped")
            return
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES
                                     | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
        run = self.run
        self._scene_client.call_async(req).add_done_callback(lambda f: self._on_scene(run, f))

    def _on_scene(self, run, future):
        try:
            objects = future.result().scene.world.collision_objects
        except Exception as e:
            self.get_logger().warn(f"get_planning_scene failed: {e}")
            return
        scene = {o.id: {"frame": o.header.frame_id,
                        "xyz": rnd((o.pose.position.x, o.pose.position.y, o.pose.position.z)),
                        "xyzw": rnd((o.pose.orientation.x, o.pose.orientation.y,
                                     o.pose.orientation.z, o.pose.orientation.w)),
                        "boxes": [rnd(p.dimensions) for p in o.primitives]}
                 for o in objects}
        run["scene"] = scene
        (run["dir"] / "scene.json").write_text(json.dumps(scene, indent=1))

    # ---------- summary ----------
    def _write_steps(self, r):
        packet = r["stage"].get("packet")
        start = ((r["scene"] or {}).get(packet) or {}).get("xyz") or [None] * 3
        cols = ["task", "packet", "arm", "slot", "step", "label", "t_start", "t_end", "gripper",
                "tcp_x", "tcp_y", "tcp_z", "packet_start_x", "packet_start_y", "packet_start_z"]
        with (r["dir"] / "steps.csv").open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(cols)
            for row in r["rows"]:
                w.writerow([r["stage"].get("task"), packet, row["arm"], r["stage"].get("slot"),
                            row["step"], row["label"], round(row["t_start"] or 0.0, 3), round(row["t_end"], 3),
                            None if row["gripper"] is None else round(row["gripper"], 4),
                            *(row["tcp"] or [None] * 3), *start])

    def destroy_node(self):
        if self.run:
            self._finish("logger stopped")
        super().destroy_node()


def main():
    rclpy.init()
    node = ExecutionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
