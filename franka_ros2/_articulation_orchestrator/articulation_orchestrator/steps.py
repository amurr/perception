"""SubMovement builders and the small pose math the sequences need.

Every arm step names its arm ("left" / "right"); moveit_master runs it on that arm's groups.
`label` ends up in moveit_master's feedback and error messages and in the execution log.
"""
import math

from geometry_msgs.msg import Pose, Vector3

from moveit_master.msg import SubMovement


# ---------- pose math ----------
def quat_from_rpy(roll, pitch, yaw):
    """(x, y, z, w) of R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_mul(a, b):
    """a * b for (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def make_pose(x, y, z, quat) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(q) for q in quat)
    return p


def wrap_angle(a):
    """-> (-pi, pi]"""
    a = math.fmod(a + math.pi, 2 * math.pi)
    return a + math.pi if a <= 0 else a - math.pi


def top_down_quat(tcp_yaw, tcp_down_rpy=(math.pi, 0.0, 0.0)):
    """TCP orientation pointing straight down, turned by `tcp_yaw` about the world z axis.
    tcp_down_rpy: TCP orientation (world) that points down with yaw 0."""
    return quat_mul(quat_from_rpy(0.0, 0.0, tcp_yaw), quat_from_rpy(*tcp_down_rpy))


# ---------- step builders ----------
def arm_pose(arm, pose, label="") -> SubMovement:
    return SubMovement(type=SubMovement.ARM_POSE, arm=arm, pose=pose, label=label)


def arm_named(arm, name, label="") -> SubMovement:
    return SubMovement(type=SubMovement.ARM_NAMED, arm=arm, named_target=name, label=label or name)


def gripper_named(arm, name, label="") -> SubMovement:
    return SubMovement(type=SubMovement.GRIPPER_NAMED, arm=arm, named_target=name, label=label or name)


def gripper_joint(arm, value, label="") -> SubMovement:
    return SubMovement(type=SubMovement.GRIPPER_JOINT, arm=arm, joint_value=float(value), label=label)


def wait(seconds, label="wait") -> SubMovement:
    return SubMovement(type=SubMovement.WAIT, duration=float(seconds), label=label)


def cartesian(arm, dx=0.0, dy=0.0, dz=0.0, frame="", label="") -> SubMovement:
    """Straight-line TCP move by (dx, dy, dz) in the planning frame (world), or in the TCP
    axes with frame="tcp"."""
    return SubMovement(type=SubMovement.ARM_CARTESIAN, arm=arm, frame=frame, label=label,
                      offset=Vector3(x=float(dx), y=float(dy), z=float(dz)))


def attach(arm, obj_id, label="") -> SubMovement:
    return SubMovement(type=SubMovement.ATTACH, arm=arm, object_id=obj_id, label=label or f"attach {obj_id}")


def detach(arm, obj_id, label="") -> SubMovement:
    return SubMovement(type=SubMovement.DETACH, arm=arm, object_id=obj_id, label=label or f"detach {obj_id}")


def allow_collision(arm, obj_id, touch=(), allow=True, label="") -> SubMovement:
    """allow: obj may touch the arm's gripper links + `touch` (e.g. the table under it)."""
    return SubMovement(type=SubMovement.ALLOW_COLLISION, arm=arm, object_id=obj_id,
                      touch_links=list(touch), allow=allow,
                      label=label or f"{'allow' if allow else 'forbid'} {obj_id}")


STEP_NAMES = {v: k for k, v in vars(SubMovement).items() if k.isupper() and isinstance(v, int)}


def describe(step) -> str:
    """One line per step, for dry runs and logs."""
    kind = STEP_NAMES.get(step.type, str(step.type))
    extra = ""
    if step.type == SubMovement.ARM_POSE:
        p = step.pose.position
        extra = f"({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
    elif step.type == SubMovement.ARM_CARTESIAN:
        o = step.offset
        extra = f"d=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}) {step.frame or 'world'}"
    elif step.type in (SubMovement.ARM_NAMED, SubMovement.GRIPPER_NAMED):
        extra = step.named_target
    elif step.type == SubMovement.GRIPPER_JOINT:
        extra = f"{step.joint_value:.3f} rad"
    elif step.type in (SubMovement.ATTACH, SubMovement.DETACH, SubMovement.ALLOW_COLLISION):
        extra = " ".join([step.object_id, *step.touch_links])
    return f"[{step.arm or '-':5}] {kind:15} {extra:40} {step.label}"
