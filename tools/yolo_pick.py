from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

from langchain_core.tools import tool

from runtime.context import RuntimeContext
from runtime.registry import sanitized_env

from .common import audited


def _cfg(ctx: RuntimeContext) -> dict[str, Any]:
    return ctx.config.yolo_pick or {}


def _path(ctx: RuntimeContext, key: str) -> str:
    return str(Path(_cfg(ctx)[key]).expanduser())


def _shared_dir(ctx: RuntimeContext) -> Path:
    return Path(_cfg(ctx)["shared_dir"]).expanduser()


def _tail(ctx: RuntimeContext, name: str, lines: int = 30) -> list[str]:
    result = ctx.registry.logs_tail(name, lines)
    if result.get("ok"):
        return result.get("tail", [])
    path = ctx.config.paths.logs_dir / f"{name}.log"
    try:
        return path.read_text(encoding="utf-8", errors="replace").splitlines()[-lines:]
    except OSError:
        return [result.get("error", "no log available")]


def _evidence(ctx: RuntimeContext) -> tuple[bool, dict[str, Any]]:
    """Simulation is up: /clock + controller_manager (+ ros_gz / gz nodes)."""
    snapshot = ctx.probes.snapshot()
    nodes = snapshot.get("nodes", [])
    topics = set(snapshot.get("topics", []))
    clock = "/clock" in topics
    controller_manager = any("controller_manager" in node for node in nodes)
    gazebo_bridge = any(
        "gz" in node.lower() or "gazebo" in node.lower() for node in nodes
    )
    return (
        clock and controller_manager,
        {
            "clock_topic": clock,
            "controller_manager": controller_manager,
            "gazebo_bridge": gazebo_bridge,
            "node_count": len(nodes),
        },
    )


def _controllers_ready(ctx: RuntimeContext) -> dict[str, Any]:
    result = ctx.probes.controller_states()
    states = result.get("controllers", {})
    arm = states.get(ctx.config.precheck.get("arm_controller", "arm_controller"))
    gripper = states.get(ctx.config.precheck.get("gripper_controller", "gripper_controller"))
    if result.get("ok") and arm == gripper == "active":
        return {"ok": True, "arm": arm, "gripper": gripper}
    return {
        "ok": False,
        "error": "机械臂或夹爪控制器未激活；急停后请先调用 recover 并确认，再重新彩排",
        "arm": arm,
        "gripper": gripper,
        "probe_error": result.get("error"),
    }


def _ensure_sim(ctx: RuntimeContext) -> dict[str, Any]:
    ok, evidence = _evidence(ctx)
    if ok and ctx.probes.echo_once("/joint_states").get("ok"):
        if _cfg(ctx).get("perception_mode") == "live":
            nodes = ctx.probes.snapshot().get("nodes", [])
            if not any("yolo_bridge_node" in node for node in nodes):
                return {"ok": False, "error": "现有仿真没有启动 YOLO 桥接，请在新 ROS 域重启"}
        return {"ok": True, "already_running": True, "evidence": evidence}

    started = ctx.registry.start(
        "yolo_sim", ["bash", _path(ctx, "sim_script")],
        env={"YOLO_LIVE_PERCEPTION": "1" if _cfg(ctx).get("perception_mode") == "live" else "0"},
    )
    deadline = time.time() + float(_cfg(ctx).get("sim_ready_timeout_s", 60))
    while time.time() < deadline:
        ok, evidence = _evidence(ctx)
        live_ready = (
            _cfg(ctx).get("perception_mode") != "live"
            or any("yolo_bridge_node" in node for node in ctx.probes.snapshot().get("nodes", []))
        )
        if ok and live_ready and ctx.probes.echo_once("/joint_states").get("ok"):
            return {
                "ok": True,
                "already_running": False,
                "pid": started.get("pid"),
                "evidence": evidence,
            }
        if ctx.registry.get("yolo_sim") is None:
            return {
                "ok": False,
                "error": "仿真启动后立即退出",
                "log_tail": _tail(ctx, "yolo_sim"),
            }
        time.sleep(1.0)
    log_tail = _tail(ctx, "yolo_sim")
    ctx.registry.stop("yolo_sim", timeout_s=10)
    return {
        "ok": False,
        "error": "仿真未在超时时间内就绪（/clock + controller_manager + /joint_states）",
        "evidence": _evidence(ctx)[1],
        "log_tail": log_tail,
    }


def _ensure_graspnet(ctx: RuntimeContext) -> dict[str, Any]:
    if _cfg(ctx).get("perception_mode") != "live":
        return {"ok": True, "needed": False}
    if ctx.registry.is_running("yolo_graspnet"):
        return {"ok": True, "already_running": True}

    log_path = ctx.config.paths.logs_dir / "yolo_graspnet.log"
    start_size = log_path.stat().st_size if log_path.exists() else 0
    started = ctx.registry.start(
        "yolo_graspnet", ["bash", _path(ctx, "graspnet_script"), "headless"],
        cwd=_path(ctx, "project_ws"),
    )
    deadline = time.time() + float(_cfg(ctx).get("graspnet_ready_timeout_s", 60))
    while time.time() < deadline:
        if not ctx.registry.is_running("yolo_graspnet"):
            return {"ok": False, "error": "GraspNet 容器已退出", "log_tail": _tail(ctx, "yolo_graspnet")}
        try:
            with log_path.open("rb") as handle:
                handle.seek(start_size)
                if b"Waiting for first frame" in handle.read():
                    return {"ok": True, "pid": started.get("pid")}
        except OSError:
            pass
        time.sleep(1)
    ctx.registry.stop("yolo_graspnet", timeout_s=10)
    return {"ok": False, "error": "GraspNet 未在超时内就绪", "log_tail": _tail(ctx, "yolo_graspnet")}


def _ensure_moveit(ctx: RuntimeContext) -> dict[str, Any]:
    nodes = ctx.probes.snapshot().get("nodes", [])
    if any(node == "move_group" or node.endswith("/move_group") for node in nodes):
        return {"ok": True, "already_running": True}

    started = ctx.registry.start("yolo_moveit", ["bash", _path(ctx, "moveit_script")])
    deadline = time.time() + float(_cfg(ctx).get("moveit_ready_timeout_s", 40))
    while time.time() < deadline:
        nodes = ctx.probes.snapshot().get("nodes", [])
        if any(node == "move_group" or node.endswith("/move_group") for node in nodes):
            time.sleep(float(_cfg(ctx).get("moveit_settle_s", 10)))
            return {"ok": True, "already_running": False, "pid": started.get("pid")}
        if ctx.registry.get("yolo_moveit") is None:
            return {
                "ok": False,
                "error": "MoveIt 启动后立即退出",
                "log_tail": _tail(ctx, "yolo_moveit"),
            }
        time.sleep(1.0)
    log_tail = _tail(ctx, "yolo_moveit")
    ctx.registry.stop("yolo_moveit", timeout_s=10)
    return {
        "ok": False,
        "error": "move_group 未在超时时间内出现",
        "log_tail": log_tail,
    }


def _read_state(ctx: RuntimeContext) -> dict[str, Any]:
    path = _shared_dir(ctx) / _cfg(ctx).get("result_file", "pick_state.json")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {"ok": False, "error": str(exc), "path": str(path)}
    return {"ok": True, "data": data, "path": str(path)}


def _stop_group(proc: subprocess.Popen, sig: int = signal.SIGINT) -> None:
    try:
        os.killpg(os.getpgid(proc.pid), sig)
    except OSError:
        pass


def _run_executor(
    ctx: RuntimeContext,
    job: Any,
    dry_run: bool,
    target: str,
    max_candidates: int,
) -> dict[str, Any]:
    shared = _shared_dir(ctx)
    shared.mkdir(parents=True, exist_ok=True)
    log_path = ctx.config.paths.logs_dir / "yolo_pick.log"
    state_path = shared / _cfg(ctx).get("result_file", "pick_state.json")

    candidate_script = _cfg(ctx).get("candidate_script")
    if candidate_script:
        try:
            import sys

            subprocess.run(
                [sys.executable, str(Path(candidate_script).expanduser())],
                check=True,
                capture_output=True,
                text=True,
                timeout=60,
            )
        except Exception as exc:  # noqa: BLE001
            return {
                "success": False,
                "error": f"candidate_script failed: {exc}",
                "pick_state": None,
                "log_tail": [str(exc)],
            }

    stop_refresh = threading.Event()
    refresh_thread: threading.Thread | None = None
    refresh_s = float(_cfg(ctx).get("candidate_refresh_s", 0) or 0)
    if candidate_script and refresh_s > 0:
        def _refresh() -> None:
            import sys

            while not stop_refresh.wait(refresh_s):
                subprocess.run(
                    [sys.executable, str(Path(candidate_script).expanduser())],
                    capture_output=True,
                    text=True,
                    timeout=60,
                )

        refresh_thread = threading.Thread(target=_refresh, daemon=True)
        refresh_thread.start()

    start_size = log_path.stat().st_size if log_path.exists() else 0
    start_mtime = state_path.stat().st_mtime if state_path.exists() else 0.0

    env = sanitized_env(
        {
            **ctx.config.workspace_env,
            "SHARED_DIR": str(shared),
            "DRY_RUN": "true" if dry_run else "false",
            "MAX_CANDIDATES": str(int(max_candidates)),
            "TARGET": target,
            "YOLO_CAMERA_PITCH": str(_cfg(ctx).get("camera_pitch_offset", 0.0)),
        }
    )
    with open(log_path, "ab", buffering=0) as log_file:
        log_file.write(
            f"\n===== {time.strftime('%F %T')} yolo_pick dry_run={dry_run} "
            f"target={target!r} =====\n".encode()
        )
        proc = subprocess.Popen(
            ["bash", _path(ctx, "executor_script")],
            cwd=str(Path(_cfg(ctx)["project_ws"]).expanduser()),
            env=env,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    timeout_s = float(_cfg(ctx).get("executor_timeout_s", 300))
    deadline = time.time() + timeout_s
    timed_out = False
    cancelled = False
    completed = False
    executor_failed = False
    text = ""
    while True:
        if job.cancel_requested:
            cancelled = True
            break
        if time.time() > deadline:
            timed_out = True
            break
        if proc.poll() is not None:
            break
        try:
            with open(log_path, "rb") as handle:
                handle.seek(start_size)
                text = handle.read().decode("utf-8", errors="replace")
        except OSError:
            text = ""
        if "task complete" in text or "task incomplete" in text:
            completed = True
            break
        if re.search(r"\[ERROR\] \[lightweight_grasp_executor(?:-\d+)?\]: process has died", text):
            executor_failed = True
            break
        # The executor writes its terminal state before moving the arm to its
        # final zero posture. Wait for its final log line or process exit.
        time.sleep(1.0)

    # The launch keeps MoveIt alive after the one-shot executor finishes, so
    # stop the whole process group once the result is known.
    _stop_group(proc)
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        _stop_group(proc, signal.SIGKILL)
        proc.wait(timeout=5)

    stop_refresh.set()
    if refresh_thread is not None:
        refresh_thread.join(timeout=65)
    state = _read_state(ctx)
    fresh_state = state_path.exists() and state_path.stat().st_mtime > start_mtime
    completed = completed or "task complete" in text or "task incomplete" in text
    if state.get("ok") and fresh_state:
        completed = completed or state["data"].get("phase") in {"complete", "incomplete"}
    success = not timed_out and not cancelled and not executor_failed and (
        "task complete" in text or (
            fresh_state and state.get("ok") and state["data"].get("phase") == "complete"
        )
    )
    result = {
        "success": bool(success),
        "completed": completed,
        "timed_out": timed_out,
        "cancelled": cancelled,
        "pick_state": state.get("data") if state.get("ok") else None,
        "pick_state_path": state.get("path"),
        "log": str(log_path),
        "log_tail": _tail(ctx, "yolo_pick", 30),
    }
    if executor_failed:
        lines = [line for line in text.splitlines() if
                 "cannot reach initial planning home" in line or
                 "lightweight_grasp_executor" in line and "process has died" in line]
        result["error"] = "执行器提前退出：" + (lines[-2] if len(lines) > 1 else lines[-1] if lines else "查看 yolo_pick.log")
    return result


def build_yolo_tools(ctx: RuntimeContext) -> list:
    @tool
    @audited(ctx, "yolo_pick")
    def yolo_pick(dry_run: bool = True, target: str = "cylinder", max_candidates: int = 10) -> dict:
        """在 piper_with_yolo_graspnet（Gazebo Harmonic）仿真里运行抓取。
        首次彩排会自动启动 Gazebo、YOLO 和 GraspNet；ROS 图为空也可直接调用。
        dry_run=True 会移动机械臂到观察/规划位置并彩排轨迹，但不夹取物体。
        dry_run=False 需要同一目标的成功彩排和人工确认。
        target 为空表示全部物体，也可指定 cube、cuboid 或 cylinder。
        用 yolo_pick_status 查询结果。"""
        if not _cfg(ctx):
            return {"ok": False, "error": "yolo_pick 配置缺失（config/yolo_pick.yaml）"}
        if ctx.config.mode != "sim":
            return {"ok": False, "error": "yolo_pick 仅允许 --mode sim"}
        if target not in {"", "cube", "cuboid", "cylinder"}:
            return {"ok": False, "error": "target 必须为 cube、cuboid、cylinder 或空"}
        if _cfg(ctx).get("candidate_script") and target != "cylinder":
            return {"ok": False, "error": "当前合成候选只支持 cylinder"}
        if not 1 <= max_candidates <= 60:
            return {"ok": False, "error": "max_candidates 必须在 1 到 60 之间"}

        if not dry_run:
            csv_path = _shared_dir(ctx) / "perception_grasps.csv"
            try:
                csv_mtime = csv_path.stat().st_mtime
            except OSError:
                return {"ok": False, "error": "找不到彩排候选文件，请先彩排"}
            valid, why = ctx.yolo_gate.validate(
                mode="sim", candidate=target or "all", csv_mtime=csv_mtime
            )
            if not valid:
                return {"ok": False, "error": why}
            controllers = _controllers_ready(ctx)
            if not controllers["ok"]:
                return controllers
            approved, why = ctx.approval.require(
                "piper_with_yolo_graspnet 真实抓取（会移动机械臂）",
                {
                    "target": target or "all",
                    "max_candidates": int(max_candidates),
                    "shared_dir": str(_shared_dir(ctx)),
                    "warning": "确认工作空间内无人、急停可达",
                },
            )
            if not approved:
                return {"ok": False, "approved": False, "error": why}
            sim_ok, evidence = _evidence(ctx)
            if not sim_ok or not ctx.probes.echo_once("/joint_states").get("ok"):
                return {"ok": False, "error": "彩排后的仿真已停止，请重新彩排", "evidence": evidence}
        else:
            ctx.yolo_gate.clear()
            sim = _ensure_sim(ctx)
            if not sim.get("ok"):
                return sim
            controllers = _controllers_ready(ctx)
            if not controllers["ok"]:
                return controllers
        graspnet = _ensure_graspnet(ctx)
        if not graspnet.get("ok"):
            return graspnet
        # lightweight_pick.launch.py starts MoveIt itself.
        if not dry_run:
            try:
                valid, why = ctx.yolo_gate.validate(
                    mode="sim", candidate=target or "all",
                    csv_mtime=csv_path.stat().st_mtime,
                )
            except OSError:
                return {"ok": False, "error": "彩排候选文件已消失，请重新彩排"}
            if not valid:
                return {"ok": False, "error": why}

        def _run(job: Any) -> dict[str, Any]:
            result = _run_executor(ctx, job, dry_run, target, max_candidates)
            if dry_run and result["success"] and not result["cancelled"]:
                csv_mtime = (_shared_dir(ctx) / "perception_grasps.csv").stat().st_mtime
                ctx.yolo_gate.issue(
                    mode="sim", candidate=target or "all", score=1.0,
                    csv_mtime=csv_mtime, dry_job_id=job.job_id,
                )
                result["token_issued"] = True
            return result

        job = ctx.jobs.submit(
            "yolo_pick",
            _run,
            timeout_s=float(_cfg(ctx).get("executor_timeout_s", 300)) + 30,
            exclusive=True,
        )
        if job.status == "rejected":
            return {"ok": False, "error": job.error}
        if not dry_run:
            ctx.yolo_gate.clear()
        return {
            "ok": True,
            "job_id": job.job_id,
            "dry_run": bool(dry_run),
            "target": target or "all",
            "shared_dir": str(_shared_dir(ctx)),
            "note": "用 yolo_pick_status(job_id) 查询；执行器是一次性节点，跑完会退出",
        }

    @tool
    @audited(ctx, "yolo_pick_status")
    def yolo_pick_status(job_id: str) -> dict:
        """查询 piper_with_yolo_graspnet 抓取任务的状态与 pick_state.json。
        输入 yolo_pick 返回的 job_id；最终以 result.success 判断抓取是否成功。"""
        state = ctx.jobs.poll(job_id)
        if not state.get("ok"):
            return state
        if isinstance(state.get("result"), dict):
            state["success"] = state["result"].get("success")
            state["result"] = {
                key: value for key, value in state["result"].items() if key != "log_tail"
            }
        job = ctx.jobs.get(job_id)
        state_path = _shared_dir(ctx) / _cfg(ctx).get("result_file", "pick_state.json")
        if job and state_path.exists() and state_path.stat().st_mtime >= job.started_at:
            state["pick_state"] = _read_state(ctx).get("data")
        else:
            state["pick_state"] = None
        return state

    return [yolo_pick, yolo_pick_status]
