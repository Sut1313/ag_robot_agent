"""Agent-controlled tabletop simulation using the real Piper perception stack."""
from __future__ import annotations

import csv
import json
import math
import os
import re
import signal
import subprocess
import time
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from langchain_core.tools import tool

from runtime.context import RuntimeContext
from runtime.registry import sanitized_env
from .common import audited

SCRIPT = Path(__file__).resolve().parent.parent / "scripts/tabletop_module.sh"
MODULES = ("sim", "moveit", "feedback", "yolo", "graspnet")


def _cfg(ctx: RuntimeContext) -> dict[str, Any]:
    return ctx.config.tabletop_pick


def _shared(ctx: RuntimeContext) -> Path:
    return Path(_cfg(ctx)["shared_dir"]).expanduser()


def _env(ctx: RuntimeContext) -> dict[str, str]:
    cfg = _cfg(ctx)
    return {
        "TABLETOP_ROS_DOMAIN_ID": str(int(cfg["ros_domain_id"])),
        "TABLETOP_GZ_PARTITION": str(cfg["gz_partition"]),
        "TABLETOP_SHARED_DIR": str(_shared(ctx)),
        "PIPER_TABLETOP_WS": cfg["tabletop_ws"],
        "PIPER_REAL_WS": cfg["real_ws"],
    }


def _run(ctx: RuntimeContext, *args: str, timeout: float = 12) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["bash", str(SCRIPT), *args],
        env=sanitized_env(_env(ctx)),
        text=True, capture_output=True, timeout=timeout,
    )


def _nodes(ctx: RuntimeContext) -> set[str]:
    result = _run(ctx, "ros2", "node", "list")
    return set(result.stdout.splitlines()) if result.returncode == 0 else set()


def _yolo_matches(ctx: RuntimeContext) -> tuple[bool, str]:
    """Require the live bridge to publish into this job's shared directory."""
    expected = {
        "shared_dir": str(_shared(ctx)),
        "target_class": str(_cfg(ctx)["object_name"]),
    }
    details = []
    matched = True
    for key, value in expected.items():
        result = _run(ctx, "ros2", "param", "get", "/d435i_yolo_graspnet_bridge", key)
        actual = result.stdout.strip().rsplit(": ", 1)[-1] if result.returncode == 0 else result.stderr.strip()
        details.append(f"{key}={actual!r}, expected={value!r}")
        matched &= result.returncode == 0 and actual == value
    return matched, "; ".join(details)


def _wait(ctx: RuntimeContext, predicate, label: str, job: Any, timeout: float | None = None) -> None:
    deadline = time.monotonic() + (timeout or float(_cfg(ctx)["startup_timeout_s"]))
    while time.monotonic() < deadline:
        if job.cancel_requested:
            raise RuntimeError(f"cancelled while waiting for {label}")
        if predicate():
            return
        time.sleep(1)
    raise TimeoutError(f"{label} not ready; check tabletop module logs")


def _start(ctx: RuntimeContext, name: str) -> None:
    ctx.registry.start(
        f"tabletop_{name}", ["bash", str(SCRIPT), name],
        cwd=str(Path(_cfg(ctx)["real_ws"] if name == "graspnet" else _cfg(ctx)["tabletop_ws"]).expanduser()),
        env=_env(ctx),
    )


def _fresh_csv(ctx: RuntimeContext, since: float = 0.0) -> bool:
    path = _shared(ctx) / "perception_grasps.csv"
    validation = _shared(ctx) / "last_validation.json"
    if not path.is_file() or not validation.is_file():
        return False
    age = time.time() - path.stat().st_mtime
    if not 0 <= age <= float(_cfg(ctx)["max_frame_age_s"]):
        return False
    if path.stat().st_mtime < since:
        return False
    if abs(path.stat().st_mtime - validation.stat().st_mtime) > 5:
        return False
    try:
        data = json.loads(validation.read_text())
        return bool(data.get("per_object_grasps"))
    except (OSError, json.JSONDecodeError):
        return False


def _ensure_stack(ctx: RuntimeContext, job: Any) -> None:
    nodes = _nodes(ctx)
    if "/controller_manager" not in nodes or "/gz_ros2_control" not in nodes:
        _start(ctx, "sim")
    _wait(ctx, lambda: {"/controller_manager", "/gz_ros2_control", "/arm_controller", "/gripper_controller"} <= _nodes(ctx),
          "Gazebo/controllers", job)

    if "/move_group" not in _nodes(ctx):
        _start(ctx, "moveit")
    _wait(ctx, lambda: "/move_group" in _nodes(ctx), "MoveIt", job)
    scene = _run(ctx, "scene", timeout=20)
    if scene.returncode != 0 or "success=True" not in scene.stdout:
        raise RuntimeError(f"failed to add tabletop collision scene: {scene.stdout[-300:]} {scene.stderr[-300:]}")

    if "/joint_state_feedback_relay" not in _nodes(ctx):
        _start(ctx, "feedback")
    _wait(ctx, lambda: "/joint_state_feedback_relay" in _nodes(ctx), "joint feedback", job)

    if "/d435i_yolo_graspnet_bridge" in _nodes(ctx):
        matched, details = _yolo_matches(ctx)
        if not matched:
            owned = "perception" if ctx.registry.is_running("perception") else "tabletop_yolo"
            if not ctx.registry.is_running(owned):
                raise RuntimeError(f"YOLO bridge uses wrong simulation source: {details}")
            if owned == "perception":
                ctx.registry.stop("perception_view", timeout_s=3)
            ctx.registry.stop(owned, timeout_s=10)
            _wait(ctx, lambda: "/d435i_yolo_graspnet_bridge" not in _nodes(ctx), "old YOLO bridge shutdown", job)
    if "/d435i_yolo_graspnet_bridge" not in _nodes(ctx):
        _start(ctx, "yolo")
    _wait(ctx, lambda: "/d435i_yolo_graspnet_bridge" in _nodes(ctx), "YOLO bridge", job)
    matched, details = _yolo_matches(ctx)
    if not matched:
        raise RuntimeError(f"YOLO bridge uses wrong simulation source: {details}")
    if not ctx.registry.is_running("tabletop_graspnet") and not _fresh_csv(ctx, job.started_at):
        _start(ctx, "graspnet")
    _wait(ctx, lambda: _fresh_csv(ctx, job.started_at), "fresh GraspNet candidates", job)


def _snapshot(ctx: RuntimeContext, job: Any, since: float = 0.0) -> dict[str, Any]:
    shared = _shared(ctx)
    source = shared / "perception_grasps.csv"
    validation = shared / "last_validation.json"
    target = _cfg(ctx)["object_name"]
    handeye = yaml.safe_load(Path(_cfg(ctx)["handeye_yaml"]).read_text())
    camera_to_base = np.asarray(handeye["R_cam2base"], dtype=float)
    grasp_to_tcp = np.asarray([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=float)
    deadline = time.monotonic() + float(_cfg(ctx)["candidate_wait_s"])
    previous_mtime = 0.0
    selected: list[dict[str, str]] = []
    fields: list[str] | None = None
    source_mtime = 0.0
    while time.monotonic() < deadline:
        if job.cancel_requested:
            raise RuntimeError("candidate search cancelled")
        if not _fresh_csv(ctx, max(job.started_at, since)):
            time.sleep(1)
            continue
        source_mtime = source.stat().st_mtime
        if source_mtime == previous_mtime:
            time.sleep(1)
            continue
        previous_mtime = source_mtime
        with source.open(newline="") as handle:
            reader = csv.DictReader(handle)
            fields = reader.fieldnames
            rows = list(reader)
        if not fields or not rows or source.stat().st_mtime != source_mtime:
            continue
        selected = []
        for row in rows:
            if not row["name"].startswith(target):
                continue
            try:
                grasp = [float(row[key]) for key in ("camera_x", "camera_y", "camera_z")]
                center = [float(row[key]) for key in ("center_camera_x", "center_camera_y", "center_camera_z")]
                rotation = np.asarray(
                    [[float(row[f"r{i}{j}"]) for j in range(3)] for i in range(3)], dtype=float)
            except (KeyError, ValueError):
                continue
            if not all(math.isfinite(v) for v in grasp + center):
                continue
            if math.dist(grasp, center) > float(_cfg(ctx)["center_distance_max_m"]):
                continue
            tcp = camera_to_base @ rotation @ grasp_to_tcp
            # PiPER link7's closing axis is -Y of the TCP frame. Its sign
            # does not matter for a square block, but its world XY angle does.
            opening_down, forward_z = float(tcp[2, 1]), float(tcp[2, 2])
            if not (_cfg(ctx)["opening_down_min"] <= opening_down <= _cfg(ctx)["opening_down_max"]
                    and _cfg(ctx)["forward_z_min"] <= forward_z <= _cfg(ctx)["forward_z_max"]):
                continue
            # A square block can be grasped along either pair of opposing
            # faces. An oblique closure needs a much wider opening and can
            # push this 40 mm block out of the retry area. The block yaw is
            # fixed in this simulation scene; real perception is untouched.
            opening_angle = math.atan2(float(tcp[1, 1]), float(tcp[0, 1]))
            yaw_error = abs((opening_angle - float(_cfg(ctx)["object_yaw_rad"])
                             + math.pi / 4) % (math.pi / 2) - math.pi / 4)
            if yaw_error > math.radians(float(_cfg(ctx)["opening_axis_max_error_deg"])):
                continue
            # GraspNet's width is 72-100 mm for the known 40 mm simulated block.
            # Use the segmented object centroid as the position estimate while
            # retaining GraspNet's 6D orientation. The raw CSV stays intact.
            pull = float(_cfg(ctx)["center_pull_xyz"])
            row["camera_x"] = f"{grasp[0] + pull * (center[0] - grasp[0]):.9f}"
            row["camera_y"] = f"{grasp[1] + pull * (center[1] - grasp[1]):.9f}"
            row["camera_z"] = f"{grasp[2] + pull * (center[2] - grasp[2]):.9f}"
            # Override only the snapshot used by the simulated ROS gripper.
            row["width"] = f"{float(_cfg(ctx)['object_width_m']):.9f}"
            selected.append(row)
        if selected:
            break
    if not selected:
        raise RuntimeError("no suitable live GraspNet candidate before timeout")
    destination = shared / "tabletop_candidates_sim.csv"
    temporary = destination.with_suffix(".tmp")
    with temporary.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(selected)
    os.replace(temporary, destination)
    provenance = {
        "source_csv": str(source), "source_mtime": source_mtime,
        "validation_mtime": validation.stat().st_mtime,
        "snapshot_csv": str(destination), "candidate_count": len(selected),
        "width_override_m": float(_cfg(ctx)["object_width_m"]),
        "opening_down_range": [_cfg(ctx)["opening_down_min"], _cfg(ctx)["opening_down_max"]],
        "center_distance_max_m": float(_cfg(ctx)["center_distance_max_m"]),
        "center_pull_xyz": float(_cfg(ctx)["center_pull_xyz"]),
        "object_yaw_rad": float(_cfg(ctx)["object_yaw_rad"]),
        "opening_axis_max_error_deg": float(_cfg(ctx)["opening_axis_max_error_deg"]),
    }
    (shared / "tabletop_candidate_provenance.json").write_text(json.dumps(provenance, indent=2))
    return provenance


def _pose(ctx: RuntimeContext) -> list[float]:
    result = _run(ctx, "gz", "model", "-m", _cfg(ctx)["object_name"], "-p", timeout=20)
    match = re.search(r"\[([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)\]", result.stdout)
    if result.returncode or not match:
        raise RuntimeError(f"cannot read Gazebo object pose: {result.stdout[-200:]} {result.stderr[-200:]}")
    return [float(part) for part in match.groups()]


def _executor(ctx: RuntimeContext, scan_only: bool, job: Any) -> None:
    ctx.registry.stop("tabletop_executor", timeout_s=5)
    shared = _shared(ctx)
    args = [
        "executor", f"scan_only:={'true' if scan_only else 'false'}",
        "gripper_backend:=ros", f"enable_sim_lift:={'false' if scan_only else 'true'}",
        "skip_final_home:=true", "sim_home_on_failure:=true",
        "sim_min_hold_width:=0.020", "sim_max_hold_width:=0.052",
        "sim_pregrasp_opening_m:=0.060",
        f"enable_sim_place_service:={'false' if scan_only else 'true'}",
        "sim_release_height_m:=0.030",
        f"csv_path:={shared / 'tabletop_candidates_sim.csv'}",
        f"shared_dir:={shared}", f"handeye_yaml:={Path(_cfg(ctx)['handeye_yaml']).expanduser()}",
        f"grasp_z_offset:={float(_cfg(ctx)['grasp_z_offset_m'])}",
        f"graspnet_6d_max_candidates:={int(_cfg(ctx)['max_candidates'])}",
        "velocity_scale:=0.02", "planning_time_precheck:=0.2",
        "pose_goal_box_m:=0.004",
        "precheck_lift:=true",
    ]
    ctx.registry.start("tabletop_executor", ["bash", str(SCRIPT), *args], env=_env(ctx))
    _wait(ctx, lambda: "/grasp_executor" in _nodes(ctx), "grasp executor", job)


def _trigger(ctx: RuntimeContext, job: Any,
             service: str = "/grasp_execute") -> str:
    proc = subprocess.Popen(
        ["bash", str(SCRIPT), "ros2", "service", "call", service, "std_srvs/srv/Trigger", "{}"],
        env=sanitized_env(_env(ctx)), text=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True,
    )
    deadline = time.monotonic() + float(_cfg(ctx)["execution_timeout_s"])
    try:
        while proc.poll() is None:
            if job.cancel_requested or time.monotonic() >= deadline:
                os.killpg(proc.pid, signal.SIGINT)
                raise TimeoutError("tabletop service call cancelled or timed out")
            time.sleep(0.5)
        output = proc.communicate(timeout=3)[0]
        if proc.returncode != 0:
            raise RuntimeError(f"{service} call failed: {output[-500:]}")
        return output
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=3)


def _attempt(ctx: RuntimeContext, job: Any, dry_run: bool,
             rehearsal: dict[str, Any] | None, since: float,
             retry_index: int = 0) -> dict[str, Any]:
    provenance_path = _shared(ctx) / "tabletop_candidate_provenance.json"
    before = _pose(ctx)
    if rehearsal is not None:
        rehearsal_pose = rehearsal.get("object_pose")
        if not rehearsal_pose:
            raise RuntimeError("rehearsal object pose missing; repeat dry_run")
        if retry_index == 0 and math.dist(before, rehearsal_pose) > 0.015:
            raise RuntimeError("wood_block moved since rehearsal; repeat dry_run")
        if retry_index > 0 and (math.dist(before[:2], rehearsal_pose[:2]) > 0.08
                                or abs(before[2] - rehearsal_pose[2]) > 0.015):
            raise RuntimeError("wood_block left the retry area or tabletop; stop and inspect")
    # Check the physical scene before waiting for a new frame. A failed
    # contact may have pushed the block beyond the reachable retry area.
    # Every job consumes its own live frame; execution never replays the
    # rehearsal CSV snapshot.
    provenance = _snapshot(ctx, job, since=since)

    if not dry_run:
        # The fresh execution-frame candidate may have a different pose from
        # the rehearsal frame, so it needs its own MoveIt precheck.
        frame_limit = int(_cfg(ctx).get("max_frame_prechecks", 4))
        for frame_index in range(frame_limit):
            provenance["object_pose"] = before
            provenance_path.write_text(json.dumps(provenance, indent=2))
            _executor(ctx, True, job)
            try:
                recheck = _trigger(ctx, job)
            finally:
                ctx.registry.stop("tabletop_executor", timeout_s=5)
            if "将执行抓取" in recheck and "success=True" in recheck:
                break
            if frame_index + 1 >= frame_limit:
                raise RuntimeError(f"no current frame passed lift precheck: {recheck[-800:]}")
            provenance = _snapshot(ctx, job,
                                   since=float(provenance["source_mtime"]) + 0.001)
    else:
        provenance["object_pose"] = before
        provenance_path.write_text(json.dumps(provenance, indent=2))

    _executor(ctx, dry_run, job)
    result_path = _shared(ctx) / "grasp_result.json"
    old_mtime = result_path.stat().st_mtime if result_path.exists() else 0
    try:
        output = _trigger(ctx, job)
        if dry_run:
            approved = "将执行抓取" in output and "success=True" in output
            if approved:
                ctx.tabletop_gate.issue(
                    mode="sim", candidate="wood_block", score=1.0,
                    csv_mtime=Path(provenance["snapshot_csv"]).stat().st_mtime,
                    dry_job_id=job.job_id,
                )
            return {"success": approved, "scan_response": output[-800:], "provenance": provenance,
                    "object_pose": before, "token_issued": approved}
        if not result_path.is_file() or result_path.stat().st_mtime <= old_mtime:
            raise RuntimeError("grasp service returned without a fresh grasp_result.json")
        result = json.loads(result_path.read_text())
        pose_samples = []
        checks = max(1, int(float(_cfg(ctx)["hold_check_s"]) / 5)) if result.get("success") else 1
        for _ in range(checks):
            time.sleep(5 if result.get("success") else 3)
            pose_samples.append(_pose(ctx))
            if (pose_samples[-1][2] - before[2] < float(_cfg(ctx)["min_lift_m"])
                    or math.dist(pose_samples[-1][:2], before[:2]) > 0.05):
                break
        after = pose_samples[-1]
        dz = after[2] - before[2]
        held = all(pose[2] - before[2] >= float(_cfg(ctx)["min_lift_m"])
                   and math.dist(pose[:2], before[:2]) <= 0.05
                   for pose in pose_samples)
        pick_success = bool(result.get("success") and result.get("status") == "success" and held)
        place_output = None
        placed_pose = None
        placed = False
        if pick_success:
            place_output = _trigger(ctx, job, "/tabletop_place")
            time.sleep(5)
            placed_pose = _pose(ctx)
            placed = ("success=True" in place_output
                      and abs(placed_pose[2] - before[2]) <= 0.012
                      and math.dist(placed_pose[:2], before[:2]) <= 0.05)
        return {"success": bool(pick_success and placed),
                "pick_success": pick_success, "placed": placed,
                "grasp_result": result, "before_xyz": before,
                "after_xyz": after, "lift_m": round(dz, 4),
                "held_samples_xyz": pose_samples, "placed_xyz": placed_pose,
                "place_response": place_output[-800:] if place_output else None,
                "provenance": provenance, "current_frame_precheck": recheck[-800:]}
    finally:
        ctx.registry.stop("tabletop_executor", timeout_s=5)


def _cycle(ctx: RuntimeContext, job: Any, dry_run: bool) -> dict[str, Any]:
    _ensure_stack(ctx, job)
    provenance_path = _shared(ctx) / "tabletop_candidate_provenance.json"
    rehearsal = json.loads(provenance_path.read_text()) if not dry_run else None
    attempts: list[dict[str, Any]] = []
    since = job.started_at
    count = 1 if dry_run else int(_cfg(ctx).get("max_execution_attempts", 2))
    for index in range(count):
        if job.cancel_requested:
            raise RuntimeError("tabletop pick cancelled")
        try:
            outcome = _attempt(ctx, job, dry_run, rehearsal, since, retry_index=index)
        except RuntimeError as exc:
            if not attempts:
                raise
            # Preserve the first failure and Gazebo evidence when the block
            # has moved too far to make another attempt safely.
            attempts[-1]["retry_blocked_reason"] = str(exc)
            break
        attempts.append(outcome)
        if dry_run or outcome["success"]:
            break
        # A failed executor run returns to zero before we take another live
        # frame. If the executor claimed success but Gazebo truth disagrees,
        # leave the arm in place for diagnosis instead of retrying blindly.
        if outcome["grasp_result"].get("success"):
            break
        since = float(outcome["provenance"]["source_mtime"]) + 0.001
        if index + 1 < count:
            time.sleep(4)
    final = dict(attempts[-1])
    final["attempt_count"] = len(attempts)
    final["attempts"] = [{"success": item["success"],
                          "stage": (item.get("grasp_result") or {}).get("stage"),
                          "source_mtime": item["provenance"]["source_mtime"]}
                         for item in attempts]
    return final


def build_tabletop_tools(ctx: RuntimeContext) -> list:
    @tool
    @audited(ctx, "tabletop_pick")
    def tabletop_pick(dry_run: bool = True) -> dict:
        """桌面 PiPER 仿真抓取循环：实时感知、彩排、抓取抬升、保持、放回及回零。
        dry_run=True 只规划预检，机械臂不动；先成功彩排，再 dry_run=False 执行。
        结果以 Gazebo 木块抬升和落回桌面验证。用 tabletop_pick_status 查询任务。"""
        if ctx.config.mode != "sim" or not _cfg(ctx):
            return {"ok": False, "error": "tabletop_pick requires configured simulation mode"}
        if dry_run:
            ctx.tabletop_gate.clear()
        else:
            snapshot = _shared(ctx) / "tabletop_candidates_sim.csv"
            if not snapshot.is_file():
                return {"ok": False, "error": "没有彩排候选快照"}
            valid, why = ctx.tabletop_gate.validate(
                mode="sim", candidate="wood_block", csv_mtime=snapshot.stat().st_mtime)
            if not valid:
                return {"ok": False, "error": why}
            approved, why = ctx.approval.require("桌面仿真木块抓取、放回与回零", {
                "object": "wood_block", "shared_dir": str(_shared(ctx)),
            })
            if not approved:
                return {"ok": False, "error": why}
            ctx.tabletop_gate.clear()
        job = ctx.jobs.submit("tabletop_pick", lambda item: _cycle(ctx, item, dry_run),
                              timeout_s=900, exclusive=True)
        if job.status == "rejected":
            return {"ok": False, "error": job.error}
        return {"ok": True, "job_id": job.job_id, "dry_run": dry_run,
                "note": "任务在后台运行；用 tabletop_pick_status(job_id) 查询最终结果"}

    @tool
    @audited(ctx, "tabletop_pick_status")
    def tabletop_pick_status(job_id: str) -> dict:
        """查询桌面仿真抓取任务；最终以 result.success、pick_success、placed 和 lift_m 判断。"""
        return ctx.jobs.poll(job_id)

    @tool
    @audited(ctx, "tabletop_stop")
    def tabletop_stop() -> dict:
        """停止 Agent 启动的桌面仿真抓取执行器、GraspNet、YOLO、MoveIt 和 Gazebo。"""
        for job in ctx.jobs.list_active():
            if job["name"] == "tabletop_pick":
                ctx.jobs.cancel(job["job_id"])
        stopped = []
        for name in ("executor", "graspnet", "yolo", "feedback", "moveit", "sim"):
            stopped.append({"name": name, **ctx.registry.stop(f"tabletop_{name}", timeout_s=5)})
        if ctx.config.mode == "sim" and ctx.config.shared_dir.resolve() == _shared(ctx).resolve():
            for name in ("perception_view", "perception", "moveit", "rviz"):
                stopped.append({"name": name, **ctx.registry.stop(name, timeout_s=5)})
        container = f"agrobot_tabletop_{int(_cfg(ctx)['ros_domain_id'])}"
        subprocess.run(["docker", "stop", container], capture_output=True, timeout=20)
        return {"ok": True, "stopped": stopped}

    return [tabletop_pick, tabletop_pick_status, tabletop_stop]
