#!/usr/bin/env python3
"""planning_scene_updater: applies PlanningScene changes made NOT by the
MoveGroupInterface-controlled robot (e.g. the table, tray and packets of a scene,
a human removed an object, perception detected something).

Uses the /apply_planning_scene service, so every call returns only after
move_group has applied the change -> a plan started afterwards sees it.
"""
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene


def _wait_future(future, timeout=None):
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout):
        return None
    return future.result()


def _identity_pose() -> Pose:
    p = Pose()
    p.orientation.w = 1.0
    return p


def _resolve_uri(uri: str) -> str:
    """'package://pkg/path/file.stl' -> absolute file path."""
    if uri.startswith("package://"):
        pkg, rel = uri[len("package://"):].split("/", 1)
        return f"{get_package_share_directory(pkg)}/{rel}"
    if uri.startswith("file://"):
        return uri[len("file://"):]
    return uri


class PlanningSceneUpdater(Node):
    def __init__(self, frame_id: str = "world"):
        super().__init__("planning_scene_updater")
        self.declare_parameter("frame_id", frame_id)
        self.frame_id = self.get_parameter("frame_id").value

        self._client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self._get_client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self._mesh_cache: dict[str, Mesh] = {}

    # ---------- connection ----------
    def wait_for_move_group(self, timeout_sec: float | None = None) -> bool:
        self.get_logger().info("waiting for /apply_planning_scene ...")
        return self._client.wait_for_service(timeout_sec=timeout_sec)

    # ---------- read ----------
    def object_ids(self, timeout_sec: float = 5.0) -> set[str] | None:
        """Ids of all world collision objects currently in move_group's scene (None = timeout)."""
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        res = _wait_future(self._get_client.call_async(req), timeout_sec)
        if res is None:
            self.get_logger().error("get_planning_scene timed out")
            return None
        return {co.id for co in res.scene.world.collision_objects}

    # ---------- object builders ----------
    def make_box(self, obj_id: str, pose: Pose, size: tuple[float, float, float]) -> CollisionObject:
        co = self._base(obj_id, pose, CollisionObject.ADD)
        co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))]
        co.primitive_poses = [_identity_pose()]
        return co

    def make_cylinder(self, obj_id: str, pose: Pose, height: float, radius: float) -> CollisionObject:
        co = self._base(obj_id, pose, CollisionObject.ADD)
        co.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[height, radius])]
        co.primitive_poses = [_identity_pose()]
        return co

    def make_mesh(self, obj_id: str, pose: Pose, mesh_uri: str, scale: float = 1.0) -> CollisionObject:
        co = self._base(obj_id, pose, CollisionObject.ADD)
        co.meshes = [self._load_mesh(mesh_uri, scale)]
        co.mesh_poses = [_identity_pose()]
        return co

    def make_move(self, obj_id: str, pose: Pose) -> CollisionObject:
        """Change the pose of an existing object (geometry is kept)."""
        return self._base(obj_id, pose, CollisionObject.MOVE)

    def make_remove(self, obj_id: str) -> CollisionObject:
        return self._base(obj_id, _identity_pose(), CollisionObject.REMOVE)

    # ---------- apply (blocking) ----------
    def apply(self, objects: list[CollisionObject], timeout_sec: float = 5.0) -> bool:
        """Send all objects as ONE diff. Returns True once move_group applied it."""
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = objects
        scene.robot_state.is_diff = True

        res = _wait_future(self._client.call_async(ApplyPlanningScene.Request(scene=scene)),
                           timeout_sec)
        if res is None:
            self.get_logger().error("apply_planning_scene timed out")
            return False
        if not res.success:
            self.get_logger().error("move_group rejected the scene diff (see move_group log)")
        return res.success

    # convenience one-liners
    def add_box(self, obj_id, pose, size):              return self.apply([self.make_box(obj_id, pose, size)])
    def add_cylinder(self, obj_id, pose, h, r):         return self.apply([self.make_cylinder(obj_id, pose, h, r)])
    def add_mesh(self, obj_id, pose, uri, scale=1.0):   return self.apply([self.make_mesh(obj_id, pose, uri, scale)])
    def move_object(self, obj_id, pose):                return self.apply([self.make_move(obj_id, pose)])
    def remove_object(self, obj_id):                    return self.apply([self.make_remove(obj_id)])

    # ---------- internals ----------
    def _base(self, obj_id: str, pose: Pose, operation: int) -> CollisionObject:
        co = CollisionObject()
        co.header.frame_id = self.frame_id
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = obj_id
        co.pose = pose
        co.operation = operation
        return co

    def _load_mesh(self, uri: str, scale: float) -> Mesh:
        key = f"{uri}@{scale}"
        if key not in self._mesh_cache:
            import trimesh  # lazy import: only needed for meshes
            tm = trimesh.load(_resolve_uri(uri), force="mesh")
            msg = Mesh()
            msg.vertices = [Point(x=float(v[0]) * scale, y=float(v[1]) * scale, z=float(v[2]) * scale)
                            for v in tm.vertices]
            msg.triangles = [MeshTriangle(vertex_indices=[int(f[0]), int(f[1]), int(f[2])])
                             for f in tm.faces]
            self._mesh_cache[key] = msg
        return self._mesh_cache[key]


def make_pose(x: float, y: float, z: float) -> Pose:
    p = _identity_pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    return p


def main():
    """Standalone demo: add a table + packet, move the packet, remove both."""
    rclpy.init()
    node = PlanningSceneUpdater()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    node.wait_for_move_group()

    table = node.make_box("demo_table", make_pose(0.85, 0.0, 0.43), (0.5, 1.2, 0.04))
    packet = node.make_box("demo_packet", make_pose(0.70, 0.3, 0.4675), (0.16, 0.06, 0.035))
    node.get_logger().info(f"add table+packet: {node.apply([table, packet])}")

    node.get_logger().info(f"move packet: {node.move_object('demo_packet', make_pose(0.8, 0.0, 0.4675))}")
    node.get_logger().info(f"remove: {node.apply([node.make_remove('demo_packet'), node.make_remove('demo_table')])}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()