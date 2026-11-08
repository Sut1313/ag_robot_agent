from __future__ import annotations

import os
import json
import re
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

from langchain_core.tools import tool

from runtime.context import RuntimeContext
from runtime.probes import as_float

from .common import audited
from .read import run_precheck

_client_lock = threading.Lock()
_client_counter = 0


def _cfg(ctx: RuntimeContext) -> dict[str, Any]:
    return ctx.config.modules.get("grasp_executor", {})


def _mode_guard(ctx: RuntimeContext) -> dict[str, Any] | None:
    ok, evidence = ctx.require_mode()
    if ok:
        return None
    return {
        "ok": False,
        "error": f"运行环境与 --mode {ctx.config.mode} 不匹配，拒绝执行",
        "mode": ctx.config.mode,
        "evidence": evidence.get("evidence"),
    }


def _script(ctx: RuntimeContext, key: str) -> str:
    return str(Path(_cfg(ctx)[key]).expanduser())


def _handeye(ctx: RuntimeContext) -> str:
    key = "handeye_sim" if ctx.config.mode == "sim" else "handeye_real"
    return str(Path(_cfg(ctx)[key]).expanduser())


def _service(ctx: RuntimeContext, dry: bool) -> str:
    return _cfg(ctx)["service_dry" if dry else "service_real"]


def _module(dry: bool) -> str:
    return "grasp_executor_dry" if dry else "grasp_executor_real"


def _tail(ctx: RuntimeContext, module: str, lines: int = 25) -> list[str]:
    result = ctx.registry.logs_tail(module, lines)
    if result.get("ok"):
        return result.get("tail", [])
    path = ctx.config.paths.logs_dir / f"{module}.log"
    try:
        return path.read_text(encoding="utf-8", errors="replace").splitlines()[-lines:]
    except OSError:
        return [result.get("error", "no log available")]


def _csv_path(ctx: RuntimeContext) -> Path:
    return ctx.config.shared_dir / ctx.config.precheck["files"].get(
        "grasps", "perception_grasps.csv"
    )


def _csv_mtime(ctx: RuntimeContext) -> float:
    try:
        return _csv_path(ctx).stat().st_mtime
    except OSError:
        return 0.0


def _top_candidate(ctx: RuntimeContext) -> tuple[dict[str, Any] | None, str | None]:
    result = ctx.probes.read_csv(
        ctx.config.precheck["files"].get("grasps", "perception_grasps.csv")
    )
    if not result.get("ok"):
        return None, result.get("error", "cannot read candidates")
    parsed = []
    for index, row in enumerate(result.get("rows", [])):
        score = as_float(row.get("score"))
        if score is None:
            continue
        parsed.append(
            {
                "index": index,
                "name": row.get("name"),
                "score": round(score, 4),
                "width_m": as_float(row.get("width")),
                "camera_xyz": [
                    as_float(row.get("camera_x")),
                    as_float(row.get("camera_y")),
                    as_float(row.get("camera_z")),
                ],
            }
        )
    if not parsed:
        return None, "候选文件为空"
    parsed.sort(key=lambda item: item["score"], reverse=True)
    return parsed[0], None


def _ensure_executor(ctx: RuntimeContext, dry: bool) -> dict[str, Any]:
    cfg = _cfg(ctx)
    name = _module(dry)
    service = _service(ctx, dry)

    if service in ctx.probes.service_names():
        return {"ok": True, "already_running": True, "service": service, "module": name}

    if dry and ctx.registry.is_running(_module(False)):
        return {
            "ok": False,
            "error": "正式执行器正在运行，拒绝启动彩排实例；请先停止正式执行器",
        }

    if not dry and ctx.registry.is_running(_module(True)):
        ctx.registry.stop(_module(True), timeout_s=8)

    if not dry and ctx.config.mode == "sim":
        bridge = ctx.config.modules.get("feedback_bridge", {})
        if bridge.get("script") and not ctx.registry.is_running("feedback_bridge"):
            ctx.registry.start(
                "feedback_bridge",
                ["/usr/bin/python3", str(Path(bridge["script"]).expanduser())],
            )
        for _ in range(10):
            count = ctx.probes.topic_publishers("/joint_states_feedback").get("count")
            if count and count > 0:
                break
            time.sleep(0.3)

    if not ctx.registry.is_running(name):
        started = ctx.registry.start(
            name,
            ["bash", _script(ctx, "script_dry" if dry else "script_real")],
            env={
                "SHARED_DIR": str(ctx.config.shared_dir),
                "HANDEYE_YAML": _handeye(ctx),
                "VELOCITY_SCALE": str(cfg.get("velocity_scale", 0.02)),
                "OPEN_GRIPPER_LIFT_HEIGHT": str(cfg.get("open_gripper_lift_height", 0.05)),
                "LIFT_Z": str(cfg.get("lift_z", 0.05)),
                "GRIPPER_BACKEND": "ros" if ctx.config.mode == "sim" else "sdk",
                "ENABLE_SIM_LIFT": "true" if ctx.config.mode == "sim" else "false",
                "ENABLE_REAL_HOLD_PLACE": str(
                    ctx.config.mode == "real" and bool(cfg.get("real_hold_place"))).lower(),
                "REAL_HOLD_PLACE_MOTION": str(cfg.get("real_hold_place_motion") is True).lower(),
                "REAL_GRASP_Z_OFFSET_M": str(cfg.get("real_grasp_z_offset_m", 0.0)),
                "GRASP_GRIPPER_SETTLE_S": "5.0" if ctx.config.mode == "sim" else "0.8",
            },
        )
    else:
        started = {"pid": ctx.registry.status(name).get("pid")}

    deadline = time.time() + float(cfg.get("ready_timeout_s", 30))
    while time.time() < deadline:
        if service in ctx.probes.service_names():
            return {
                "ok": True,
                "already_running": False,
                "service": service,
                "module": name,
                "pid": started.get("pid"),
            }
        if ctx.registry.get(name) is None:
            return {
                "ok": False,
                "error": "执行器启动后立即退出",
                "module": name,
                "log_tail": _tail(ctx, name),
            }
        time.sleep(0.5)

    return {
        "ok": False,
        "error": "执行器服务未在超时时间内出现",
        "module": name,
        "log_tail": _tail(ctx, name),
    }


def _kill_process(proc: subprocess.Popen) -> None:
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except OSError:
        pass


def _call_trigger(
    ctx: RuntimeContext, service: str, timeout_s: float, job: Any,
    *, hard_stop_on_timeout: bool = True, cancel_service: str | None = None,
) -> dict[str, Any]:
    """Call a std_srvs/Trigger service without blocking the ReAct loop."""
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    global _client_counter
    if not rclpy.ok():
        rclpy.init()

    with _client_lock:
        _client_counter += 1
        node_name = f"rosa_grasp_client_{_client_counter}"

    node = Node(node_name)
    try:
        client = node.create_client(Trigger, service)
        if not client.wait_for_service(timeout_sec=10.0):
            return {"ok": False, "error": f"服务不可用: {service}"}
        if job is not None and job.cancel_requested:
            return {"ok": False, "cancelled": True, "error": "调用已取消"}

        future = client.call_async(Trigger.Request())
        deadline = time.time() + timeout_s
        while not future.done():
            cancelled = job is not None and job.cancel_requested
            timed_out = time.time() > deadline
            if cancelled or timed_out:
                future.cancel()
                result = {"ok": False, "cancelled": cancelled,
                          "error": "调用已取消" if cancelled else "服务调用超时"}
                if cancel_service:
                    result["stop_request"] = _call_trigger(
                        ctx, cancel_service, 4, None, hard_stop_on_timeout=False)
                elif timed_out and hard_stop_on_timeout:
                    result["estop"] = _estop(ctx, "service_timeout")
                return result
            rclpy.spin_once(node, timeout_sec=0.1)

        response = future.result()
        return {
            "ok": True,
            "success": bool(response.success),
            "message": str(response.message),
        }
    finally:
        node.destroy_node()


def _estop(ctx: RuntimeContext, reason: str) -> dict[str, Any]:
    code, out, err = ctx.probes.run(
        [
            "ros2",
            "control",
            "switch_controllers",
            "--deactivate",
            "arm_controller",
            "gripper_controller",
            "--strict",
        ],
        timeout=10,
    )
    result: dict[str, Any] = {
        "deactivate_controllers": {
            "code": code,
            "out": (out or "").strip()[:300],
            "err": (err or "").strip()[:300],
        }
    }
    if ctx.config.mode == "real":
        code2, out2, err2 = ctx.probes.run(
            [
                "ros2",
                "service",
                "call",
                "/enable_srv",
                "piper_msgs/srv/Enable",
                "{enable_request: false}",
            ],
            timeout=10,
        )
        result["disable_arm"] = {
            "code": code2,
            "out": (out2 or "").strip()[:300],
            "err": (err2 or "").strip()[:300],
        }
    return result


def _activate(ctx: RuntimeContext) -> dict[str, Any]:
    code, out, err = ctx.probes.run(
        [
            "ros2",
            "control",
            "switch_controllers",
            "--activate",
            "arm_controller",
            "gripper_controller",
            "--strict",
        ],
        timeout=10,
    )
    result: dict[str, Any] = {
        "activate_controllers": {
            "code": code,
            "out": (out or "").strip()[:300],
            "err": (err or "").strip()[:300],
        }
    }
    if ctx.config.mode == "real":
        code2, out2, err2 = ctx.probes.run(
            [
                "ros2",
                "service",
                "call",
                "/enable_srv",
                "piper_msgs/srv/Enable",
                "{enable_request: true}",
            ],
            timeout=10,
        )
        result["enable_arm"] = {
            "code": code2,
            "out": (out2 or "").strip()[:300],
            "err": (err2 or "").strip()[:300],
        }
    return result


def parse_scan_report(message: str) -> tuple[bool, dict[str, int]]:
    """A Trigger success only means the scan completed.

    A usable rehearsal needs at least one candidate that passed every stage
    (lift_pass > 0). The dry-run message carries these counters; if the format
    changes and no counters are found, fail closed.
    """
    counts = {
        key: int(value)
        for key, value in re.findall(
            r"(candidates|tested|gate_rejected|approach_pass|grasp_pass|lift_pass)=(\d+)",
            message or "",
        )
    }
    return counts.get("lift_pass", 0) > 0, counts


def precheck_passed(call: dict[str, Any], report: dict[str, Any] | None) -> bool:
    """Did a candidate pass every precheck stage?

    The executor's ``lift_pass`` counter can stay 0 even when it selects a
    fully-passing candidate (observed in simulation: message says "全过" but
    lift_pass=0). The structured ``precheck_results.json`` selection is the
    primary signal; counters/message are a fallback.
    """
    if not call.get("ok") or not call.get("success"):
        return False
    message = str(call.get("message", ""))
    if "无全过候选" in message:
        return False
    if isinstance(report, dict):
        if report.get("selected") is not None:
            return True
        if report.get("selected_index") not in (None, -1):
            return True
    _, counts = parse_scan_report(message)
    if counts.get("lift_pass", 0) > 0:
        return True
    return "全过" in message


def build_grasp_tools(ctx: RuntimeContext) -> list:
    mode = ctx.config.mode
    cfg = _cfg(ctx)
    managed_real = mode == "real" and cfg.get("real_hold_place") is True
    real_job_names = {"real_hold_prepare", "grasp_and_hold", "place_held_object"}

    def cycle_call(service: str, timeout: float, job=None, motion: bool = False) -> dict:
        call = _call_trigger(ctx, service, timeout, job, hard_stop_on_timeout=False,
                             cancel_service="/real_hold_cancel" if motion else None)
        if not call.get("ok"):
            return call
        try:
            report = json.loads(call["message"])
            if not isinstance(report, dict):
                raise ValueError("expected an object")
        except (KeyError, ValueError):
            return {"ok": False, "error": "真机执行器返回了无效状态"}
        return {**report, "ok": bool(call.get("success") and report.get("success"))}

    def require_cycle():
        if not managed_real:
            return {"ok": False, "error": "此工具需要真机模式及保持放置配置"}
        return _mode_guard(ctx)

    def submit_cycle_job(name, work, timeout):
        job = ctx.jobs.submit(name, work, timeout_s=timeout, exclusive=True)
        if job.status == "rejected":
            return {"ok": False, "error": job.error}
        return {"ok": True, "job_id": job.job_id, "note": "用 grasp_job_status 查询执行结果"}

    @tool
    @audited(ctx, "real_hold_prepare")
    def real_hold_prepare() -> dict:
        """仅规划真机抓取、抬升和原位放回的完整路径。此工具不移动机械臂或夹爪。"""
        guard = require_cycle()
        if guard:
            return guard

        def work(job):
            if not run_precheck(ctx).get("ok_for_grasp"):
                return {"ok": False, "error": "系统预检未通过，请查询 precheck_all"}
            if job.cancel_requested:
                return {"ok": False, "error": "预演已取消"}
            started = _ensure_executor(ctx, dry=False)
            if not started.get("ok"):
                return started
            return cycle_call("/real_hold_prepare", 110, job)

        return submit_cycle_job("real_hold_prepare", work, 220)

    @tool
    @audited(ctx, "held_object_status")
    def held_object_status() -> dict:
        """查询真机预演、最近一次持物反馈、放置或待人工处理状态。查询不会发送动作。"""
        if not managed_real:
            return {"ok": False, "error": "此工具需要真机模式及保持放置配置"}
        return cycle_call("/real_hold_status", 4)

    def cycle_motion(name: str, expected: str, endpoint: str, label: str, target: str = ""):
        guard = require_cycle()
        if guard:
            return guard
        if cfg.get("real_hold_place_motion") is not True:
            return {"ok": False, "error": "真机保持放置当前只允许预演，动作开关未开启"}
        if ctx.jobs.list_active():
            return {"ok": False, "error": "已有任务在运行，请先查询任务状态"}
        status = cycle_call("/real_hold_status", 4)
        preview_id = status.get("preview_id")
        if (not status.get("ok") or status.get("state") != expected
                or not status.get("motion_enabled") or not isinstance(preview_id, str)
                or not re.fullmatch(r"[0-9a-f]{32}", preview_id)):
            return {"ok": False, "error": f"动作需要 {expected} 状态和有效的真机预演",
                    "cycle": status}
        candidate = status.get("candidate") or ""
        if target and target not in {candidate, re.sub(r"_\d+$", "", candidate)}:
            return {"ok": False, "error": "预演物体与请求类别不一致", "candidate": candidate}
        approved, why = ctx.approval.require(label, {
            "candidate": candidate,
            "preview_id": preview_id,
            "approval_remaining_s": status.get("approval_remaining_s"),
            "pick_position_m": status.get("pick_position_m"),
            "lift_height_m": status.get("lift_height_m"),
        }, force_confirm=True)
        if not approved:
            return {"ok": False, "error": why}
        return submit_cycle_job(name, lambda job: cycle_call(
            f"/{endpoint}/p{preview_id}", 300, job, motion=True), 320)

    @tool
    @audited(ctx, "grasp_and_hold")
    def grasp_and_hold(target: str = "") -> dict:
        """用户明确要求抓取并保持时使用。需要 real_hold_prepare 成功及终端人工确认。
        target 为用户指定的 YOLO 类别，如 block。成功后等待独立放回命令。"""
        return cycle_motion("grasp_and_hold", "prepared", "real_hold_pick", "真机抓取并保持", target)

    @tool
    @audited(ctx, "place_held_object")
    def place_held_object() -> dict:
        """用户明确要求放回当前物体时使用。需要 held 状态及终端人工确认。
        真机返回抓取点、松爪、退让并回零。"""
        return cycle_motion("place_held_object", "held", "real_hold_place", "真机原位放回并回零")

    @tool
    @audited(ctx, "executor_start")
    def executor_start(dry_run: bool = True) -> dict:
        """启动抓取执行器节点。dry_run=True 为彩排实例（不动臂），False 为正式实例。
        用户说"启动抓取执行器"时调用；一般不需要手动调用，彩排/执行会自动启动。"""
        guard = _mode_guard(ctx)
        if guard:
            return guard
        return _ensure_executor(ctx, dry_run)

    @tool
    @audited(ctx, "grasp_precheck")
    def grasp_precheck() -> dict:
        """彩排抓取：检查系统预检、读取最佳候选、启动彩排执行器并跑一次 dry-run
        （机械臂不动）。返回 job_id，用 grasp_job_status 查询结果。
        只有彩排成功才会签发执行令牌。用户说"彩排一下/预演抓取/检查能不能抓"时调用。"""
        if managed_real:
            return {"ok": False, "error": "此真机档位请使用 real_hold_prepare 彩排完整路径"}
        guard = _mode_guard(ctx)
        if guard:
            return guard
        pre = run_precheck(ctx)
        if not pre.get("ok_for_grasp") and set(pre.get("blocking", [])) <= {
            "controllers",
            "perception",
            "camera",
        }:
            # Transient CLI/discovery failures right after MoveIt starts.
            time.sleep(2)
            pre = run_precheck(ctx)
        if not pre.get("ok_for_grasp"):
            return {
                "ok": False,
                "error": "系统预检未通过",
                "blocking": pre.get("blocking"),
                "hint": "先修复 blocking 项（moveit/perception 等）再彩排",
            }

        best, error = _top_candidate(ctx)
        if best is None:
            return {"ok": False, "error": error, "hint": "先启动感知与 GraspNet 生成候选"}

        ensured = _ensure_executor(ctx, dry=True)
        if not ensured.get("ok"):
            return ensured

        service = _service(ctx, True)
        csv_mtime = _csv_mtime(ctx)
        candidate = best["name"]
        score = best["score"]

        def work(job):
            job.stage = "dry_run"
            call = _call_trigger(ctx, service, 180, job)
            job.stage = "collecting"
            precheck = ctx.probes.read_json("precheck_results.json")
            report = precheck.get("data") if precheck.get("ok") else None
            grasp_result = ctx.probes.read_json(
                ctx.config.precheck["files"].get("grasp_result", "grasp_result.json")
            )
            scan_ok, scan_counts = parse_scan_report(str(call.get("message", "")))
            issued = precheck_passed(call, report)
            selected_index = (
                report.get("selected_index") if isinstance(report, dict) else None
            )
            token_info = None
            if issued:
                preview = ctx.grasp_gate.issue(
                    mode=mode,
                    candidate=candidate,
                    score=score,
                    csv_mtime=csv_mtime,
                    dry_job_id=job.job_id,
                    message=(
                        f"{call.get('message', '')} | selected_index={selected_index}"
                    ),
                )
                token_info = {
                    "candidate": preview.candidate,
                    "score": preview.score,
                    "selected_index": selected_index,
                    "valid_s": ctx.grasp_gate.valid_s,
                }
            return {
                "call": call,
                "report": report,
                "grasp_result": grasp_result.get("data")
                if grasp_result.get("ok")
                else None,
                "candidate": best,
                "scan_report": scan_counts,
                "selected_index": selected_index,
                "token_issued": issued,
                "token": token_info,
            }

        job = ctx.jobs.submit("grasp_precheck", work, timeout_s=190, exclusive=True)
        if job.status == "rejected":
            return {"ok": False, "error": job.error}
        return {
            "ok": True,
            "job_id": job.job_id,
            "candidate": best,
            "note": "用 grasp_job_status 查询彩排结果；成功后才签发执行令牌",
        }

    @tool
    @audited(ctx, "grasp_execute")
    def grasp_execute(candidate: str = "") -> dict:
        """执行真实抓取（会移动机械臂）。必须先完成 grasp_precheck，且令牌未过期、
        候选未变化。执行前会要求人工输入 yes 确认。用户明确说"执行抓取/开始抓"时调用。"""
        if managed_real:
            return {"ok": False, "error": "此真机档位请使用 grasp_and_hold"}
        guard = _mode_guard(ctx)
        if guard:
            return guard
        preview = ctx.grasp_gate.token()
        if preview is None:
            return {
                "ok": False,
                "error": "没有有效的彩排令牌",
                "hint": "先运行 grasp_precheck 并确保成功",
            }

        name = candidate or preview.candidate
        ok, reason = ctx.grasp_gate.validate(
            mode=mode, candidate=name, csv_mtime=_csv_mtime(ctx)
        )
        if not ok:
            return {"ok": False, "error": reason}

        approved, why = ctx.approval.require(
            "真实抓取（会移动机械臂）",
            {
                "candidate": name,
                "score": preview.score,
                "mode": mode,
                "velocity_scale": cfg.get("velocity_scale", 0.02),
                "handeye_yaml": _handeye(ctx),
                "dry_job": preview.dry_job_id,
                "warning": "确认工作空间内无人、急停按钮可达",
            },
        )
        if not approved:
            return {"ok": False, "approved": False, "error": why}

        def work(job):
            job.stage = "starting_executor"
            ensured = _ensure_executor(ctx, dry=False)
            if not ensured.get("ok"):
                raise RuntimeError(ensured.get("error", "executor start failed"))
            job.stage = "executing"
            call = _call_trigger(ctx, _service(ctx, False), 120, job)
            job.stage = "collecting"
            latest = ctx.probes.read_json(
                ctx.config.precheck["files"].get("grasp_result", "grasp_result.json")
            )
            report = latest.get("data") if latest.get("ok") else None
            if call.get("success"):
                ctx.grasp_gate.clear()
            return {"call": call, "report": report}

        job = ctx.jobs.submit("grasp_execute", work, timeout_s=150, exclusive=True)
        if job.status == "rejected":
            return {"ok": False, "error": job.error}
        return {
            "ok": True,
            "approved": True,
            "job_id": job.job_id,
            "candidate": name,
            "note": "用 grasp_job_status 查询进度；急停用 estop 工具或另一终端 scripts/estop.sh",
        }

    @tool
    @audited(ctx, "grasp_job_status")
    def grasp_job_status(job_id: str) -> dict:
        """查询彩排或执行任务的进度/结果。输入 job_id（如 job_3）。"""
        state = ctx.jobs.poll(job_id)
        if not state.get("ok"):
            return state
        if state.get("name") in real_job_names:
            return state
        if state["status"] in {"succeeded", "failed", "cancelled", "timeout"}:
            latest = ctx.probes.read_json(
                ctx.config.precheck["files"].get("grasp_result", "grasp_result.json")
            )
            state["grasp_result"] = latest.get("data") if latest.get("ok") else None
        return state

    @tool
    @audited(ctx, "grasp_cancel")
    def grasp_cancel(job_id: str) -> dict:
        """取消抓取任务并立即停止机械臂：取消任务、停用控制器、（真机）失能机械臂、
        停止正式执行器。用户说"停下/取消抓取"时调用。"""
        job = ctx.jobs.get(job_id)
        if job is not None and job.name in real_job_names:
            return ctx.jobs.cancel(job_id)
        if managed_real:
            return {"ok": False, "error": "请提供正在运行的真机保持放置任务编号"}
        cancelled = ctx.jobs.cancel(job_id)
        estop = _estop(ctx, "grasp_cancel")
        executor = ctx.registry.stop(_module(False), timeout_s=8)
        return {
            "ok": True,
            "cancel": cancelled,
            "estop": estop,
            "executor": executor,
        }

    @tool
    @audited(ctx, "estop")
    def estop() -> dict:
        """立即急停：取消所有任务、停用 arm/gripper 控制器、（真机）失能机械臂。
        不需要确认。用户说"急停/马上停"时调用。"""
        for job in ctx.jobs.list_active():
            ctx.jobs.cancel(job["job_id"])
        return {"ok": True, "estop": _estop(ctx, "estop_tool")}

    @tool
    @audited(ctx, "recover")
    def recover() -> dict:
        """急停后恢复：重新激活控制器，（真机）重新使能机械臂。需要人工确认。
        用户说"恢复/重新使能"时调用。"""
        guard = _mode_guard(ctx)
        if guard:
            return guard
        approved, why = ctx.approval.require(
            "恢复控制器与机械臂使能（机械臂会重新上电/受力）",
            {"mode": mode},
        )
        if not approved:
            return {"ok": False, "approved": False, "error": why}
        return {"ok": True, "approved": True, "result": _activate(ctx)}

    return [
        executor_start,
        grasp_precheck,
        grasp_execute,
        real_hold_prepare,
        grasp_and_hold,
        held_object_status,
        place_held_object,
        grasp_job_status,
        grasp_cancel,
        estop,
        recover,
    ]
