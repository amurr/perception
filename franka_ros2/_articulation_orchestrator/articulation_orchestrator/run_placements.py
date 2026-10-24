#!/usr/bin/env python3
"""run_placements: move the packets of a scene YAML into the tray, one packet at a time.

For every task (packet id): ArmAllocator picks the arm (side of the table, IK check, balance),
the orchestrator builds the pick-and-place sequence, moveit_master runs it. The other arm waits
at home. Results go to <out_dir>/<timestamp>/results.csv.

  ros2 run articulation_orchestrator run_placements                         # default scene
  ros2 run articulation_orchestrator run_placements --scene my.yaml --count 10 --velocity 0.3
  ros2 run articulation_orchestrator run_placements --keep-going --retries 2              # robust batch
  ros2 run articulation_orchestrator run_placements --dry-run               # print steps only

Needs move_group + moveit_master running (moveit_master/launch/moveit_sequence.launch.py), and the
packets placed in Isaac where the scene YAML says.
"""
import argparse
import csv
import math
import os
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from articulation_orchestrator.arm_allocator import ArmAllocator, IkChecker
from articulation_orchestrator.articulation_orchestrator import ArticulationOrchestrator
from articulation_orchestrator.planning_scene_updater import PlanningSceneUpdater
from articulation_orchestrator.scene import load_scene
from articulation_orchestrator.steps import arm_named, describe
from moveit_master.msg import SubMovement


def parse_args(argv):
    share = get_package_share_directory("articulation_orchestrator")
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--scene", default=os.path.join(share, "assets", "scenes", "packets_tray.yaml"))
    p.add_argument("--count", type=int, default=0, help="first N tasks (0 = all)")
    p.add_argument("--velocity", type=float, default=0.2, help="velocity / acceleration scaling")
    p.add_argument("--arm", choices=["left", "right"], help="use only this arm")
    p.add_argument("--split-y", type=float, default=0.0, help="world y between the arms' sides")
    p.add_argument("--margin", type=float, default=0.05, help="middle band where IK/balance decide")
    p.add_argument("--no-ik", action="store_true", help="skip the /compute_ik reachability check")
    p.add_argument("--no-home", action="store_true", help="do not send both arms home first")
    p.add_argument("--keep-going", action="store_true", help="continue after a failed packet")
    p.add_argument("--retries", type=int, default=2,
                   help="retry a packet that failed before the grasp (nothing attached) this often")
    p.add_argument("--dry-run", action="store_true", help="print the sequences, move nothing")
    p.add_argument("--out-dir", default="/tmp/packet_runs")
    return p.parse_args(argv)


def main(argv=None):
    rclpy.init(args=argv)
    args = parse_args(rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:])

    orch, updater = ArticulationOrchestrator(), PlanningSceneUpdater()
    executor = MultiThreadedExecutor()
    for node in (orch, updater):
        executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    log = orch.get_logger()

    scene = load_scene(args.scene, orch.object_spec)
    tasks = scene.tasks[:args.count] if args.count else scene.tasks
    arms = (args.arm,) if args.arm else ("left", "right")
    ik = None if (args.no_ik or args.dry_run) else IkChecker(orch, frame=scene.frame)
    allocator = ArmAllocator(arms=arms, split_y=args.split_y, margin=args.margin, ik_check=ik)
    log.info(f"scene {args.scene}: {len(scene.packets)} packets, {len(tasks)} tasks, arms {arms}")

    if not args.dry_run:
        orch.wait_for_master()
        updater.wait_for_move_group()
        if not updater.apply(scene.collision_objects(updater)):
            log.error("could not add the scene to the planning scene")
            rclpy.shutdown()
            return 1
        if not args.no_home:
            home = [arm_named(a, orch._param("home_pose_format").format(arm=a)) for a in arms]
            ok, _, msg = orch.run_sequence(home, args.velocity)
            if not ok:
                log.error(f"homing failed: {msg}")
                rclpy.shutdown()
                return 1

    out = Path(args.out_dir) / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    rows, failures = [], 0
    for n, pid in enumerate(tasks, 1):
        packet = scene.packets[pid]
        spec = orch.object_spec(packet.type)
        slot = scene.tray.next_slot(footprint=packet.size[:2], near=(packet.x, packet.y))

        def poses_for(arm):
            k = orch.key_poses(arm, packet, scene, slot)
            return [k["pre_grasp"], k["place_hover"]]      # fingers clear of the packet there

        arm = allocator.choose(packet.y, poses_for)
        row = {"n": n, "packet": pid, "type": packet.type, "arm": arm, "slot": slot,
               "x": round(packet.x, 4), "y": round(packet.y, 4), "yaw_deg": round(math.degrees(packet.yaw), 1)}
        if arm is None:
            log.error(f"[{n}/{len(tasks)}] {pid}: no arm reaches it")
            rows.append({**row, "success": False, "message": "unreachable"})
            failures += 1
            if not args.keep_going:
                break
            continue

        k = orch.key_poses(arm, packet, scene, slot)
        steps = orch.pick_place_steps(arm, packet, scene, slot, k)
        attach_at = next(i for i, s in enumerate(steps) if s.type == SubMovement.ATTACH)
        detach_at = next(i for i, s in enumerate(steps) if s.type == SubMovement.DETACH)
        log.info(f"[{n}/{len(tasks)}] {pid} ({packet.type}) -> tray slot {slot} with the {arm} arm, "
                 f"{len(steps)} steps")
        stop = False
        attempts = 1 if args.dry_run else 1 + max(args.retries, 0)
        for attempt in range(1, attempts + 1):
            if args.dry_run:
                for i, s in enumerate(steps):
                    print(f"  {i:2d} {describe(s)}")
                ok, failed, msg, seconds = True, -1, "dry run", 0.0
                break
            orch.publish_stage(task=n, packet=pid, arm=arm, slot=slot, attempt=attempt,
                               labels=[s.label for s in steps])
            t0 = time.time()
            ok, failed, msg = orch.run_sequence(steps, args.velocity)
            seconds = time.time() - t0
            if ok:
                break
            log.error(f"  FAILED (attempt {attempt}/{attempts}): {msg}")
            if failed > detach_at:
                # released in the tray, only the retreat / homing failed: it occupies the slot
                scene.tray.place(slot, packet)
            elif failed >= attach_at:
                # the packet may be anywhere in Isaac now: take it out of the planning scene
                updater.remove_object(pid)
                log.warn(f"  {pid} removed from the planning scene (lost after the grasp)")
            for step in orch.recover_steps(arm, spec):        # one by one: see recover_steps
                ok_rec, _, msg_rec = orch.run_sequence([step], args.velocity)
            if not ok_rec:                                    # the last one: home
                log.error(f"  recovery failed: {msg_rec}; stopping")
                stop = True
                break
            if failed >= attach_at:
                break                                         # retry only if nothing was grasped
            if attempt < attempts:
                log.info("  nothing grasped: retrying (new pre-grasp IK solution)")
        rows.append({**row, "success": ok, "failed_step": failed, "attempts": attempt,
                     "seconds": round(seconds, 1), "message": msg})

        if ok:
            allocator.finished(arm)
            scene.tray.place(slot, packet)
            packet.x, packet.y = k["place_xy"]
            packet.z, packet.yaw = k["place_packet_z"], k["place_packet_yaw"]
            log.info(f"  done in {seconds:.1f} s" + (f" (attempt {attempt})" if attempt > 1 else ""))
            continue
        failures += 1
        if stop or not args.keep_going:
            break

    with (out / "results.csv").open("w", newline="") as f:
        cols = ["n", "packet", "type", "arm", "slot", "x", "y", "yaw_deg", "success", "failed_step",
                "attempts", "seconds", "message"]
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    done = sum(r["success"] for r in rows)
    log.info(f"{done}/{len(tasks)} packets in the tray, {failures} failed; results: {out / 'results.csv'}")
    rclpy.shutdown()
    return 0 if failures == 0 else 2


if __name__ == "__main__":
    sys.exit(main())
