from __future__ import annotations

import time
import subprocess
from pathlib import Path
from typing import Any

from langchain_core.tools import tool

from runtime.context import RuntimeContext

from .common import audited
from .source import yolo_shared_dir


def _cfg(ctx: RuntimeContext, name: str) -> dict[str, Any]:
    return ctx.config.modules.get(name, {})


def _script(ctx: RuntimeContext, value: str) -> str:
    return str(Path(value).expanduser())


def _tail(ctx: RuntimeContext, module: str, lines: int = 20) -> list[str]:
    result = ctx.registry.logs_tail(module, lines)
    if result.get("ok"):
        return result.get("tail", [])
    # The process may already have exited and been pruned; read its log file.
    path = ctx.config.paths.logs_dir / f"{module}.log"
    try:
        content = path.read_text(encoding="utf-8", errors="replace").splitlines()
        return content[-lines:]
    except OSError:
        return [result.get("error", "no log available")]


def _mode_guard(ctx: RuntimeContext) -> dict[str, Any] | None:
    """Refuse start actions when runtime evidence contradicts --mode."""
    ok, evidence = ctx.require_mode()
    if ok:
        return None
    return {
        "ok": False,
        "error": f"运行环境与 --mode {ctx.config.mode} 不匹配，拒绝启动模块",
        "mode": ctx.config.mode,
        "evidence": evidence.get("evidence"),
    }


def _wait_file_newer(
    ctx: RuntimeContext,
    module: str,
    filename: str,
    since: float,
    timeout_s: float,
) -> tuple[bool, float | None]:
    path = ctx.config.shared_dir / filename
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            mtime = path.stat().st_mtime
            if mtime >= since - 1.0:
                return True, mtime
        except OSError:
            pass
        if ctx.registry.get(module) is None:
            return False, None
        time.sleep(0.5)
    return False, None


def _log_contains(ctx: RuntimeContext, module: str, needle: str) -> bool:
    path = ctx.config.paths.logs_dir / f"{module}.log"
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return False
    # Only look at the current run: the log is appended across restarts.
    marker = text.rfind("start: ")
    current = text[marker:] if marker >= 0 else text
    return needle.lower() in current.lower()


def _wait_perception(
    ctx: RuntimeContext, since: float, timeout_s: float
) -> tuple[str, float | None]:
    """Return one of: frame / started / dead / timeout.

    A synchronized RGB-D frame is the strong signal, but a bridge that is alive
    and has logged "started" is still a valid module even if the simulation is
    paused or the camera stream is not producing pairable frames.
    """
    path = ctx.config.shared_dir / "detections.json"
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            mtime = path.stat().st_mtime
            if mtime >= since - 1.0:
                return "frame", mtime
        except OSError:
            pass
        if ctx.registry.get("perception") is None:
            return "dead", None
        time.sleep(0.5)
    if _log_contains(ctx, "perception", "bridge started"):
        return "started", None
    return "timeout", None


def _wait_node(
    ctx: RuntimeContext, node_name: str, timeout_s: float
) -> tuple[bool, list[str]]:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        nodes = ctx.probes.snapshot().get("nodes", [])
        if any(node == node_name or node.endswith("/" + node_name) for node in nodes):
            return True, nodes
        if ctx.registry.get("moveit") is None:
            return False, nodes
        time.sleep(1.0)
    return False, ctx.probes.snapshot().get("nodes", [])


def _open_viewer(ctx: RuntimeContext) -> dict[str, Any]:
    if not ctx.probes.display_available():
        return {"ok": False, "error": "无图形会话（DISPLAY/WAYLAND_DISPLAY 未设置）"}
    existing = ctx.registry.get("perception_view")
    if existing is not None:
        return {"ok": True, "already_running": True, "pid": existing.pid}
    cfg = _cfg(ctx, "perception_view")
    script = _script(ctx, cfg["script"])
    yolo_dir = yolo_shared_dir(ctx)
    shared_dir = yolo_dir or ctx.config.shared_dir
    overlay_args = (
        ["--overlay-name", "yolo_overlay_live.png", "--manifest-name", "detections_live.json"]
        if yolo_dir else []
    )
    result = ctx.registry.start(
        "perception_view",
        [
            "/usr/bin/python3",
            script,
            "--shared-dir",
            str(shared_dir),
            *overlay_args,
        ],
    )
    time.sleep(0.8)
    if ctx.registry.get("perception_view") is None:
        return {
            "ok": False,
            "error": "窗口进程启动后立即退出",
            "log_tail": _tail(ctx, "perception_view"),
        }
    return {"ok": True, **result}


def _open_rviz(ctx: RuntimeContext, mode: str) -> dict[str, Any]:
    if not ctx.probes.display_available():
        return {"ok": False, "error": "无图形会话（DISPLAY/WAYLAND_DISPLAY 未设置）"}
    if ctx.registry.get("rviz") is not None:
        return {"ok": True, "already_running": True}
    cfg = _cfg(ctx, "rviz")
    key = "sim_script" if mode == "sim" else "real_script"
    started = ctx.registry.start("rviz", ["bash", _script(ctx, cfg[key])])
    time.sleep(1.0)
    if ctx.registry.get("rviz") is None:
        return {
            "ok": False,
            "error": "RViz 启动后立即退出",
            "log_tail": _tail(ctx, "rviz"),
        }
    return {"ok": True, **started}


def build_module_tools(ctx: RuntimeContext) -> list:
    mode = ctx.config.mode

    @tool
    @audited(ctx, "perception_start")
    def perception_start(target_class: str = "", show_window: bool = True) -> dict:
        """启动 YOLO 感知模块。仿真：只启动识别节点读取 Gazebo 相机；真机：启动 RealSense +
        识别 + 手眼 TF。启动需要 10-40 秒。show_window=True 时同时打开识别窗口。
        用户说"启动视觉/打开相机/开始检测"时调用。"""
        yolo_dir = yolo_shared_dir(ctx)
        if yolo_dir:
            result = {
                "ok": True,
                "already_running": True,
                "source": "yolo_bridge_node",
                "shared_dir": str(yolo_dir),
            }
            if show_window:
                result["window"] = _open_viewer(ctx)
            return result
        guard = _mode_guard(ctx)
        if guard:
            return guard
        if mode == "sim":
            if target_class and target_class not in {"block", "wood_block"}:
                return {"ok": False, "error": "当前桌面仿真只支持 wood_block 目标"}
            from .tabletop_pick import _nodes, _yolo_matches
            if "/d435i_yolo_graspnet_bridge" in _nodes(ctx):
                matched, details = _yolo_matches(ctx)
                if matched:
                    result = {"ok": True, "already_running": True, "shared_dir": str(ctx.config.shared_dir), "target_class": "wood_block"}
                    if show_window:
                        result["window"] = _open_viewer(ctx)
                    return result
                if ctx.registry.is_running("perception"):
                    ctx.registry.stop("perception_view", timeout_s=3)
                    ctx.registry.stop("perception", timeout_s=10)
                else:
                    return {"ok": False, "error": f"现有 YOLO 节点使用错误的数据源：{details}"}
        elif ctx.registry.is_running("perception"):
            status = ctx.registry.status("perception")
            return {"ok": True, "already_running": True, "pid": status.get("pid")}

        cfg = _cfg(ctx, "perception")
        key = "script_sim" if mode == "sim" else "script_real"
        if key not in cfg:
            return {"ok": False, "error": f"module config missing {key}"}

        default_key = "default_target_class_sim" if mode == "sim" else "default_target_class"
        target = "wood_block" if mode == "sim" else target_class or cfg.get(default_key, "block")
        since = time.time()
        started = ctx.registry.start(
            "perception",
            ["bash", _script(ctx, cfg[key])],
            env={
                "TARGET_CLASS": str(target),
                "SHARED_DIR": str(ctx.config.shared_dir),
            },
        )
        ready, mtime = _wait_perception(
            ctx,
            since,
            float(cfg.get("ready_timeout_s", 40)),
        )
        if ready in {"dead", "timeout"}:
            log_tail = _tail(ctx, "perception")
            stopped = ctx.registry.stop("perception", timeout_s=5)
            return {
                "ok": False,
                "error": "感知进程启动失败或未进入就绪状态",
                "reason": ready,
                "target_class": target,
                "log_tail": log_tail,
                "stopped": stopped,
            }

        result: dict[str, Any] = {
            "ok": True,
            "pid": started.get("pid"),
            "log": started.get("log"),
            "target_class": target,
            "frame_committed": ready == "frame",
            "ready_file": str(ctx.config.shared_dir / cfg.get("ready_file", "")),
        }
        if ready == "started":
            result["warning"] = (
                "模块已启动，但在超时时间内没有完成 RGB-D 帧同步，因此还没有检测结果。"
                "请检查相机话题是否在发布、仿真是否处于暂停状态。"
            )
            result["log_tail"] = _tail(ctx, "perception", 10)
        if show_window:
            result["window"] = _open_viewer(ctx)
        return result

    @tool
    @audited(ctx, "perception_stop")
    def perception_stop() -> dict:
        """停止 YOLO 感知模块并关闭识别窗口。用户说"关掉视觉/停止检测"时调用。"""
        view = ctx.registry.stop("perception_view", timeout_s=3)
        perception = ctx.registry.stop("perception", timeout_s=10)
        return {"ok": True, "perception": perception, "view": view}

    @tool
    @audited(ctx, "perception_view_open")
    def perception_view_open() -> dict:
        """打开 YOLO 识别窗口（显示画好框的相机画面）。用户说"给我看识别画面"时调用。"""
        return _open_viewer(ctx)

    @tool
    @audited(ctx, "perception_view_close")
    def perception_view_close() -> dict:
        """关闭 YOLO 识别窗口。"""
        return {"ok": True, **ctx.registry.stop("perception_view", timeout_s=3)}

    @tool
    @audited(ctx, "graspnet_start")
    def graspnet_start() -> dict:
        """启动 GraspNet 抓取推理（Docker）。仿真使用桌面共享目录，就绪约需 30-120 秒。
        用户说"启动抓取推理/生成抓取候选"时调用。"""
        guard = _mode_guard(ctx)
        if guard:
            return guard
        if mode == "sim":
            from .tabletop_pick import _nodes, _yolo_matches
            if "/d435i_yolo_graspnet_bridge" not in _nodes(ctx):
                return {"ok": False, "error": "请先启动仿真 YOLO 感知节点"}
            matched, details = _yolo_matches(ctx)
            if not matched:
                return {"ok": False, "error": f"YOLO 节点与 GraspNet 的仿真数据源不一致：{details}"}
        module = "tabletop_graspnet" if mode == "sim" else "graspnet"
        if ctx.registry.is_running(module):
            return {"ok": True, "already_running": True, "shared_dir": str(ctx.config.shared_dir)}
        cfg = _cfg(ctx, "graspnet")
        since = time.time()
        if mode == "sim":
            tabletop = ctx.config.tabletop_pick
            command = ["bash", str(Path(__file__).resolve().parent.parent / "scripts/tabletop_module.sh"), "graspnet"]
            env = {
                "TABLETOP_SHARED_DIR": str(ctx.config.shared_dir),
                "TABLETOP_ROS_DOMAIN_ID": str(tabletop["ros_domain_id"]),
                "TABLETOP_GZ_PARTITION": str(tabletop["gz_partition"]),
            }
        else:
            command = ["bash", _script(ctx, cfg["script"])]
            env = {"SHARED_DIR": str(ctx.config.shared_dir)}
        started = ctx.registry.start(module, command, env=env)
        ready, _ = _wait_file_newer(
            ctx,
            module,
            cfg.get("ready_file", "perception_grasps.csv"),
            since,
            float(cfg.get("ready_timeout_s", 90)),
        )
        if not ready:
            log_tail = _tail(ctx, module)
            stopped = ctx.registry.stop(module, timeout_s=10)
            if mode == "sim":
                subprocess.run(["docker", "stop", f"agrobot_tabletop_{ctx.config.tabletop_pick['ros_domain_id']}"], capture_output=True, timeout=20)
            return {
                "ok": False,
                "error": "GraspNet 未在超时时间内产出候选",
                "log_tail": log_tail,
                "stopped": stopped,
            }
        return {"ok": True, "pid": started.get("pid"), "log": started.get("log"), "shared_dir": str(ctx.config.shared_dir)}

    @tool
    @audited(ctx, "graspnet_stop")
    def graspnet_stop() -> dict:
        """停止 GraspNet 推理容器。"""
        module = "tabletop_graspnet" if mode == "sim" else "graspnet"
        stopped = ctx.registry.stop(module, timeout_s=15)
        if mode == "sim":
            subprocess.run(["docker", "stop", f"agrobot_tabletop_{ctx.config.tabletop_pick['ros_domain_id']}"], capture_output=True, timeout=20)
        return {"ok": True, **stopped}

    @tool
    @audited(ctx, "moveit_start")
    def moveit_start(with_rviz: bool = False) -> dict:
        """启动 MoveIt move_group（运动规划服务），可选同时打开 RViz。
        用户说"启动运动规划/打开 RViz"时调用。"""
        guard = _mode_guard(ctx)
        if guard:
            return guard
        nodes = ctx.probes.snapshot().get("nodes", [])
        if any(node == "move_group" or node.endswith("/move_group") for node in nodes):
            return {"ok": True, "already_running": True}

        cfg = _cfg(ctx, "moveit")
        key = "sim_script" if mode == "sim" else "real_script"
        started = ctx.registry.start(
            "moveit",
            ["bash", _script(ctx, cfg[key])],
            env={"START_RVIZ": "true" if with_rviz and mode == "sim" else "false"},
        )

        ready, seen = _wait_node(
            ctx, cfg.get("ready_node", "move_group"), float(cfg.get("ready_timeout_s", 30))
        )
        if not ready:
            log_tail = _tail(ctx, "moveit")
            ctx.registry.stop("moveit", timeout_s=5)
            return {
                "ok": False,
                "error": "move_group 未在超时时间内出现",
                "log_tail": log_tail,
                "nodes_seen": seen,
            }
        # The node appears before the planning pipeline is ready; planning
        # requests sent immediately time out.
        time.sleep(float(cfg.get("settle_s", 8)))

        result: dict[str, Any] = {"ok": True, "pid": started.get("pid"), "log": started.get("log")}
        if with_rviz and mode == "real":
            result["rviz"] = _open_rviz(ctx, mode)
        return result

    @tool
    @audited(ctx, "moveit_stop")
    def moveit_stop() -> dict:
        """停止 MoveIt move_group 和 RViz。"""
        real_cfg = ctx.config.modules.get("grasp_executor", {})
        if (mode == "real" and real_cfg.get("real_hold_place") is True
                and real_cfg.get("real_hold_place_motion") is True
                and "/real_hold_status" in ctx.probes.service_names()):
            return {"ok": False, "error": "真机保持放置执行器仍在线，请现场检查持物状态后停机"}
        rviz = ctx.registry.stop("rviz", timeout_s=5)
        moveit = ctx.registry.stop("moveit", timeout_s=20)
        note = None
        if moveit.get("forced"):
            note = (
                "move_group 收到 SIGINT 后未在超时内退出，已强制终止；"
                "MoveIt 2.5 在关闭时可能打印段错误，属已知现象，进程已结束"
            )
        return {"ok": True, "rviz": rviz, "moveit": moveit, "note": note}

    @tool
    @audited(ctx, "rviz_open")
    def rviz_open() -> dict:
        """打开 RViz 运动规划界面（仿真用 piper_tabletop_sim 的 moveit.rviz）。"""
        guard = _mode_guard(ctx)
        if guard:
            return guard
        return _open_rviz(ctx, mode)

    @tool
    @audited(ctx, "rviz_close")
    def rviz_close() -> dict:
        """关闭 RViz。"""
        return {"ok": True, **ctx.registry.stop("rviz", timeout_s=5)}

    return [
        perception_start,
        perception_stop,
        perception_view_open,
        perception_view_close,
        graspnet_start,
        graspnet_stop,
        moveit_start,
        moveit_stop,
        rviz_open,
        rviz_close,
    ]
