"""Offline tests of the packet logic: tray slots, arm allocation, grasp geometry, sequences."""
import math
import os

import pytest
import rclpy

from articulation_orchestrator.arm_allocator import ArmAllocator
from articulation_orchestrator.articulation_orchestrator import (
    ArticulationOrchestrator, close_position, tip_below_tcp)
from articulation_orchestrator.scene import load_scene
from articulation_orchestrator.steps import wrap_angle
from moveit_master.msg import SubMovement

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, "..", "assets")
SCENE = os.path.join(ASSETS, "scenes", "packets_tray.yaml")


@pytest.fixture(scope="module")
def orch():
    rclpy.init()
    node = ArticulationOrchestrator()
    node.set_parameters([rclpy.parameter.Parameter(
        "objects_dictionary", value=os.path.join(ASSETS, "objects_dictionary.yaml"))])
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def scene(orch):
    return load_scene(SCENE, orch.object_spec)


def tcp_axes(q):
    """World x, y, z axes of quaternion (x, y, z, w)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return ((1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)),
            (2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)),
            (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)))


def test_close_position():
    assert close_position(0.085, 0.0) == pytest.approx(0.0)
    assert close_position(0.0, 0.0) == pytest.approx(0.8)
    assert close_position(0.06, 0.15) == pytest.approx(0.8 * (1 - 0.06 / 0.085) + 0.15)
    assert close_position(0.01, 0.5) == 0.8


def test_scene_loads(scene):
    assert len(scene.packets) == 10 and len(scene.tasks) == 10
    p = scene.packets["packet_01"]
    assert p.z == pytest.approx(scene.table.top_z + p.size[2] / 2)


def test_tray_slots_fill_floor_then_stack(scene):
    tray = scene.tray
    n = tray.slots[0] * tray.slots[1]
    used = []
    for pid in scene.tasks:
        k = tray.next_slot(scene.packets[pid].size[:2])
        used.append(k)
        tray.place(k, scene.packets[pid])
    assert used[:n] == list(range(n))           # one per slot first
    layers = math.ceil(len(scene.tasks) / n)
    assert max(tray.stack) <= layers * 0.04 + 1e-9   # then evenly stacked layers
    xs = [tray.slot_xy(k) for k in range(n)]
    assert all(abs(x - tray.center[0]) < tray.size[0] / 2 and abs(y - tray.center[1]) < tray.size[1] / 2
               for x, y in xs)


def test_slot_on_the_packets_side(scene):
    tray = scene.tray
    for pid in ("packet_01", "packet_02"):
        p = scene.packets[pid]
        k = tray.next_slot(p.size[:2], near=(p.x, p.y))
        assert tray.slot_xy(k)[1] * p.y > 0             # same side of the tray as the packet
        tray.place(k, p)


def test_too_big_for_slot(scene):
    with pytest.raises(ValueError):
        scene.tray.next_slot((0.5, 0.1))


def test_allocator_sides_and_balance():
    a = ArmAllocator()
    assert a.choose(0.3) == "left"
    assert a.choose(-0.3) == "right"
    a.finished("left")
    assert a.choose(0.0) == "right"             # middle: the less busy arm
    blocked = ArmAllocator(ik_check=lambda arm, poses: arm == "right")
    assert blocked.choose(0.3, lambda arm: []) == "right"   # side arm cannot reach
    none = ArmAllocator(ik_check=lambda arm, poses: False)
    assert none.choose(0.3, lambda arm: []) is None


def test_grasp_pose_points_down_and_closes_across(orch, scene):
    p = scene.packets["packet_07"]                     # yaw -15 deg, grasp across y
    k = orch.key_poses("left", p, scene, 0)
    x_ax, y_ax, z_ax = tcp_axes(k["pre_grasp"].orientation)
    assert z_ax[2] == pytest.approx(-1.0)              # approach straight down
    # closing axis (TCP y) parallel to the packet's y axis
    packet_y = (-math.sin(p.yaw), math.cos(p.yaw))
    assert abs(y_ax[0] * packet_y[0] + y_ax[1] * packet_y[1]) == pytest.approx(1.0)
    spec = orch.object_spec(p.type)
    tip = tip_below_tcp(spec["gripper_close"])
    assert k["grasp_z"] == pytest.approx(max(p.z + p.size[2] / 2 - spec["grasp_depth"],
                                             scene.table.top_z + tip + spec["min_table_clearance"]))
    assert k["grasp_z"] - tip > scene.table.top_z                       # closed fingertips above the table
    assert k["grasp_z"] - tip < p.z + p.size[2] / 2                     # ... and below the packet top
    assert k["pre_grasp"].position.z > k["grasp_z"]


def test_tip_below_tcp():
    assert tip_below_tcp(0.0) == pytest.approx(0.0193)
    assert tip_below_tcp(0.8) == pytest.approx(0.0331)
    assert tip_below_tcp(0.25) == pytest.approx((0.1559 + 0.1584) / 2 - 0.13)


def test_place_heights(orch, scene):
    p = scene.packets["packet_01"]
    k = orch.key_poses("left", p, scene, 0)
    hang = k["grasp_z"] - (p.z - p.size[2] / 2)
    assert k["place_z"] - hang == pytest.approx(scene.tray.floor_z + 0.01)       # bottom: floor + drop
    assert k["place_hover"].position.z - hang > scene.tray.rim_z                 # clears the rim
    # the packet ends lying along the tray
    assert wrap_angle(k["place_packet_yaw"] - scene.tray.yaw) == pytest.approx(0.0)


def test_sequence(orch, scene):
    p = scene.packets["packet_02"]
    steps = orch.pick_place_steps("right", p, scene, 1)
    assert all(s.arm == "right" for s in steps if s.type != SubMovement.WAIT)
    types = [s.type for s in steps]
    assert types.index(SubMovement.ATTACH) < types.index(SubMovement.DETACH)
    assert steps[-1].type == SubMovement.ARM_NAMED and steps[-1].named_target == "right_home"
    # every allow is undone
    allowed = [(s.object_id, tuple(s.touch_links)) for s in steps if s.type == SubMovement.ALLOW_COLLISION and s.allow]
    forbidden = [(s.object_id, tuple(s.touch_links)) for s in steps if s.type == SubMovement.ALLOW_COLLISION and not s.allow]
    assert sorted(allowed) == sorted(forbidden)
    # descend and lower are straight down, retreat mirrors lower
    cart = {s.label: s.offset.z for s in steps if s.type == SubMovement.ARM_CARTESIAN}
    assert cart["pick: descend"] < 0 and cart["place: lower"] < 0
    assert cart["place: retreat"] == pytest.approx(-cart["place: lower"])
