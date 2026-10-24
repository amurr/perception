"""ArmAllocator: decides which arm picks a packet.

1. Side rule: a packet clearly on one side of the robot (world y beyond split_y +- margin) goes to
   that arm (left arm = +y side).
2. Near the middle, or when the side arm cannot reach: the arm that passes the IK check, then the
   one that has done fewer packets (keeps both arms working), then the side preference.
3. IK check (optional): every key TCP pose of the task (pre-grasp, grasp, place hover, place)
   must have a collision-free IK solution for the arm (move_group /compute_ik).

The allocator is plain Python; the IK check is a callable, so the rules can be tested without ROS.
"""
from __future__ import annotations

import threading

ARMS = ("left", "right")


class ArmAllocator:
    def __init__(self, arms=ARMS, split_y=0.0, margin=0.05, ik_check=None):
        """ik_check(arm, poses) -> bool, or None to skip reachability checks."""
        self.arms = tuple(arms)
        self.split_y = split_y
        self.margin = margin
        self.ik_check = ik_check
        self.done = {a: 0 for a in self.arms}

    def side_arm(self, y) -> str | None:
        if y > self.split_y + self.margin:
            return "left"
        if y < self.split_y - self.margin:
            return "right"
        return None

    def choose(self, y, poses_for=None) -> str | None:
        """Arm for a packet at world `y`. poses_for(arm) -> key TCP poses for that arm (the grasp
        yaw may differ per arm); needed only with ik_check. None: no arm can do it."""
        side = self.side_arm(y)
        order = sorted(self.arms, key=lambda a: (a != side, self.done[a], self.arms.index(a)))
        if side is not None:
            order = [side] + [a for a in order if a != side]
        for arm in order:
            if self.ik_check is None or poses_for is None or self.ik_check(arm, poses_for(arm)):
                return arm
        return None

    def finished(self, arm):
        self.done[arm] += 1


class IkChecker:
    """ik_check for ArmAllocator through move_group's /compute_ik (collision-aware)."""

    def __init__(self, node, group_format="{arm}_arm", tcp_format="{arm}_robotiq_85_tcp",
                 frame="world", timeout=0.2, attempts=3):
        from moveit_msgs.srv import GetPositionIK
        self._srv = GetPositionIK
        self._node = node
        self._client = node.create_client(GetPositionIK, "/compute_ik")
        self.group_format, self.tcp_format, self.frame, self.timeout = group_format, tcp_format, frame, timeout
        self.attempts = attempts       # KDL starts from random seeds: one miss is not "unreachable"

    def __call__(self, arm, poses) -> bool:
        from builtin_interfaces.msg import Duration
        from geometry_msgs.msg import PoseStamped
        from moveit_msgs.msg import MoveItErrorCodes
        if not self._client.wait_for_service(timeout_sec=2.0):
            self._node.get_logger().warn("/compute_ik not available: IK check skipped")
            return True
        for pose in poses:
            req = self._srv.Request()
            ik = req.ik_request
            ik.group_name = self.group_format.format(arm=arm)
            ik.ik_link_name = self.tcp_format.format(arm=arm)
            ik.pose_stamped = PoseStamped(pose=pose)
            ik.pose_stamped.header.frame_id = self.frame
            ik.avoid_collisions = True
            ik.robot_state.is_diff = True          # seed: the current state
            ik.timeout = Duration(sec=int(self.timeout), nanosec=int((self.timeout % 1) * 1e9))
            for _ in range(self.attempts):
                res = _call(self._client, req, 5.0)
                if res is not None and res.error_code.val == MoveItErrorCodes.SUCCESS:
                    break
            else:
                return False
        return True


def _call(client, req, timeout):
    done = threading.Event()
    future = client.call_async(req)
    future.add_done_callback(lambda _: done.set())
    return future.result() if done.wait(timeout) else None
