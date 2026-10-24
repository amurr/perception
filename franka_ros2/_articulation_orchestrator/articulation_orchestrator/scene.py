"""Scene description for the packets -> tray task: table, tray and packets, loaded from one YAML
(assets/scenes/*.yaml). The same file is meant to place the objects in Isaac, so the planning
scene and the simulation agree.

Frames: everything is in the planning frame (`frame`, default world). Sizes are full lengths in
metres, yaws in degrees in the file and radians in the objects.

  frame: world
  table: {id: table, center: [x, y], top_z: z, size: [lx, ly], thickness: t}
  tray:  {id: tray, center: [x, y], yaw_deg: a, size: [lx, ly, h], wall: w, floor: f,
          slots: [nx, ny]}
  packets:
    - {id: packet_01, type: packet_small, xy: [x, y], yaw_deg: a}   # resting on the table
  tasks: [packet_01, packet_03, ...]                                # optional, default: all
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import yaml

from articulation_orchestrator.steps import make_pose, quat_from_rpy


@dataclass
class Table:
    id: str
    center: tuple[float, float]
    top_z: float
    size: tuple[float, float]
    thickness: float = 0.04


@dataclass
class Packet:
    id: str
    type: str
    x: float
    y: float
    yaw: float
    size: tuple[float, float, float] = (0.0, 0.0, 0.0)   # filled from objects_dictionary
    z: float = 0.0                                       # centre, filled from the table height


@dataclass
class Tray:
    id: str
    center: tuple[float, float]
    yaw: float
    size: tuple[float, float, float]      # outer lx, ly, height
    wall: float
    floor: float
    base_z: float                         # bottom of the tray (= table top)
    slots: tuple[int, int] = (2, 2)
    # per slot: height of what is already stacked on the floor, and the packet ids there
    stack: list[float] = field(default_factory=list)
    contents: list[list[str]] = field(default_factory=list)

    def __post_init__(self):
        n = self.slots[0] * self.slots[1]
        self.stack = self.stack or [0.0] * n
        self.contents = self.contents or [[] for _ in range(n)]

    @property
    def floor_z(self) -> float:
        return self.base_z + self.floor

    @property
    def rim_z(self) -> float:
        return self.base_z + self.size[2]

    def slot_xy(self, k) -> tuple[float, float]:
        """World xy of slot k (row-major over the inner area, x along the tray's length)."""
        nx, ny = self.slots
        ix, iy = k % nx, k // nx
        inner_x, inner_y = self.size[0] - 2 * self.wall, self.size[1] - 2 * self.wall
        lx = -inner_x / 2 + inner_x * (ix + 0.5) / nx
        ly = -inner_y / 2 + inner_y * (iy + 0.5) / ny
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.center[0] + c * lx - s * ly, self.center[1] + s * lx + c * ly

    def slot_size(self) -> tuple[float, float]:
        nx, ny = self.slots
        return (self.size[0] - 2 * self.wall) / nx, (self.size[1] - 2 * self.wall) / ny

    def next_slot(self, footprint=None, near=None) -> int:
        """A slot with the lowest stack: fills the floor, then layers. Of those, the one nearest
        `near` (x, y), e.g. the packet, so an arm stays on its side of the tray; else the first.
        footprint (lx, ly): the packet must fit the slot (lying along the tray)."""
        if footprint is not None:
            sx, sy = self.slot_size()
            if footprint[0] > sx + 1e-6 or footprint[1] > sy + 1e-6:
                raise ValueError(f"packet {footprint[0]:.3f} x {footprint[1]:.3f} m does not fit a "
                                 f"tray slot {sx:.3f} x {sy:.3f} m: change tray.slots")

        def dist(k):
            if near is None:
                return 0.0
            x, y = self.slot_xy(k)
            return round(math.hypot(x - near[0], y - near[1]), 4)
        return min(range(len(self.stack)), key=lambda k: (round(self.stack[k], 4), dist(k), k))

    def place(self, k, packet: Packet):
        self.stack[k] += packet.size[2]
        self.contents[k].append(packet.id)


@dataclass
class Scene:
    frame: str
    table: Table
    tray: Tray
    packets: dict[str, Packet]
    tasks: list[str]

    # ---------- planning scene ----------
    def collision_objects(self, updater) -> list:
        """Table, tray (floor + 4 walls) and every packet as PlanningScene collision objects."""
        t = self.table
        table = updater.make_box(
            t.id, make_pose(t.center[0], t.center[1], t.top_z - t.thickness / 2, (0, 0, 0, 1)),
            (t.size[0], t.size[1], t.thickness))
        return [table, self.tray_object(updater)] + [self.packet_object(updater, p)
                                                      for p in self.packets.values()]

    def tray_object(self, updater):
        from shape_msgs.msg import SolidPrimitive
        tr = self.tray
        lx, ly, h = tr.size
        co = updater.make_box(tr.id, make_pose(tr.center[0], tr.center[1], tr.base_z,
                                               quat_from_rpy(0, 0, tr.yaw)),
                              (lx, ly, tr.floor))
        # tray frame: origin at the bottom centre; floor + 4 walls
        parts = [((0, 0, tr.floor / 2), (lx, ly, tr.floor)),
                 ((+(lx - tr.wall) / 2, 0, h / 2), (tr.wall, ly, h)),
                 ((-(lx - tr.wall) / 2, 0, h / 2), (tr.wall, ly, h)),
                 ((0, +(ly - tr.wall) / 2, h / 2), (lx, tr.wall, h)),
                 ((0, -(ly - tr.wall) / 2, h / 2), (lx, tr.wall, h))]
        co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size)) for _, size in parts]
        co.primitive_poses = [make_pose(*xyz, (0, 0, 0, 1)) for xyz, _ in parts]
        return co

    @staticmethod
    def packet_object(updater, p: Packet, x=None, y=None, z=None, yaw=None):
        return updater.make_box(
            p.id, make_pose(p.x if x is None else x, p.y if y is None else y, p.z if z is None else z,
                            quat_from_rpy(0, 0, p.yaw if yaw is None else yaw)),
            tuple(p.size))


def load_scene(path, object_spec) -> Scene:
    """object_spec(type) -> dict with `size` [lx, ly, h] (ArticulationOrchestrator.object_spec)."""
    with open(path) as f:
        d = yaml.safe_load(f)
    t = d["table"]
    table = Table(id=t.get("id", "table"), center=tuple(t["center"]), top_z=float(t["top_z"]),
                  size=tuple(t["size"]), thickness=float(t.get("thickness", 0.04)))
    r = d["tray"]
    tray = Tray(id=r.get("id", "tray"), center=tuple(r["center"]),
                yaw=math.radians(r.get("yaw_deg", 0.0)), size=tuple(r["size"]),
                wall=float(r.get("wall", 0.008)), floor=float(r.get("floor", 0.005)),
                base_z=float(r.get("base_z", table.top_z)), slots=tuple(r.get("slots", (2, 2))))
    packets = {}
    for p in d["packets"]:
        spec = object_spec(p["type"])
        size = tuple(float(v) for v in spec["size"])
        z = float(p.get("z", table.top_z + size[2] / 2))
        packets[p["id"]] = Packet(id=p["id"], type=p["type"], x=float(p["xy"][0]), y=float(p["xy"][1]),
                                  yaw=math.radians(p.get("yaw_deg", 0.0)), size=size, z=z)
    tasks = list(d.get("tasks") or packets)
    unknown = [i for i in tasks if i not in packets]
    if unknown:
        raise KeyError(f"tasks name unknown packets {unknown}")
    return Scene(frame=d.get("frame", "world"), table=table, tray=tray, packets=packets, tasks=tasks)
