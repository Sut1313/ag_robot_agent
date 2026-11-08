from __future__ import annotations

import time
from datetime import datetime
from typing import Any

from langchain_core.tools import tool

from runtime.context import RuntimeContext
from runtime.probes import as_float

from .common import audited
from .source import yolo_shared_dir

REQUIRED_FOR_GRASP = (
    "mode",
    "joint_states",
    "tf",
    "controllers",
    "camera",
    "moveit",
    "shared_dir",
    "perception",
    "docker",
)


def _item(
    name: str,
    status: str,
    evidence: dict[str, Any] | None = None,
    hint: str | None = None,
) -> dict[str, Any]:
    entry: dict[str, Any] = {"item": name, "status": status}
    if evidence is not None:
        entry["evidence"] = evidence
    if hint:
        entry["hint"] = hint
    return entry


def run_precheck(ctx: RuntimeContext) -> dict[str, Any]:
    probes = ctx.probes
    cfg = ctx.config
    pre = cfg.precheck
    snapshot = probes.snapshot()
    nodes = snapshot.get("nodes", [])
    topics = set(snapshot.get("topics", []))
    services = set(snapshot.get("services", []))
    items: list[dict[str, Any]] = []

    # 1. mode evidence
    mode = ctx.mode_evidence(snapshot)
    items.append(
        _item(
            "mode",
            "pass" if mode["ok"] else "fail",
            mode["evidence"],
            None
            if mode["ok"]
            else f"当前声明为 {cfg.mode} 模式，但运行环境证据不匹配；请检查是否 source 了正确的工作空间",
        )
    )

    # 2. joint states
    joint_topic = pre["joint_states_topic"]
    joint = probes.echo_once(joint_topic)
    if joint.get("ok") and isinstance(joint.get("value"), dict):
        value = joint["value"]
        position = value.get("position")
        names = value.get("name")
        position_list = position if isinstance(position, list) else []
        name_list = names if isinstance(names, list) else []
        by_name = (
            dict(zip(name_list, position_list)) if name_list else {}
        )
        required_joints = [f"joint{index}" for index in range(1, 7)]
        present = [name for name in required_joints if name in by_name]
        ok = len(present) == len(required_joints)
        evidence = {
            "topic": joint_topic,
            "found_joints": present,
            "published_names": name_list,
            "gripper": by_name.get("gripper"),
        }
        stamp = value.get("header", {}).get("stamp", {})
        if cfg.mode == "real" and isinstance(stamp, dict):
            try:
                stamp_s = float(stamp.get("sec", 0)) + float(
                    stamp.get("nanosec", 0)
                ) / 1e9
                evidence["age_s"] = round(max(0.0, time.time() - stamp_s), 3)
            except (TypeError, ValueError):
                pass
        items.append(
            _item(
                "joint_states",
                "pass" if ok else "fail",
                evidence,
                None if ok else "未收到有效关节状态，检查驱动或仿真是否启动",
            )
        )
    else:
        items.append(
            _item(
                "joint_states",
                "fail",
                {"topic": joint_topic, "error": joint.get("error")},
                "无法读取 /joint_states",
            )
        )

    # 3. TF chain
    tf_cfg = pre.get("tf", {})
    if "parent" in tf_cfg and "child" in tf_cfg:
        tf_parent, tf_child = tf_cfg["parent"], tf_cfg["child"]
    else:
        mode_cfg = tf_cfg.get(cfg.mode, {})
        tf_parent = mode_cfg.get("parent", "base_link")
        tf_child = mode_cfg.get("child", "camera_link")
    tf = probes.tf_ok(tf_parent, tf_child)
    items.append(
        _item(
            "tf",
            "pass" if tf.get("ok") else "fail",
            {
                "parent": tf_parent,
                "child": tf_child,
                "translation": tf.get("translation"),
                "error": tf.get("error"),
            },
            None if tf.get("ok") else "TF 链不存在或未刷新；检查 robot_state_publisher / 手眼 TF",
        )
    )

    # 4. controllers
    controllers = probes.controller_states()
    arm_name = pre.get("arm_controller", "arm_controller")
    gripper_name = pre.get("gripper_controller", "gripper_controller")
    states = controllers.get("controllers", {})
    arm_state = states.get(arm_name)
    gripper_state = states.get(gripper_name)
    controllers_ok = bool(controllers.get("ok")) and arm_state == "active" and gripper_state == "active"
    items.append(
        _item(
            "controllers",
            "pass" if controllers_ok else "fail",
            {
                "arm": arm_state,
                "gripper": gripper_state,
                "error": controllers.get("error"),
            },
            None if controllers_ok else "arm/gripper 控制器未同时 active",
        )
    )

    # 5. camera publishers
    camera_topic = pre.get("camera_topic")
    camera = probes.topic_publishers(camera_topic)
    camera_ok = bool(camera.get("ok")) and (camera.get("count") or 0) > 0
    items.append(
        _item(
            "camera",
            "pass" if camera_ok else "fail",
            {"topic": camera_topic, "publishers": camera.get("count")},
            None if camera_ok else "相机话题没有发布者；检查 D435i 驱动或仿真相机桥接",
        )
    )

    # 6. MoveIt
    moveit_node = pre.get("moveit_node", "move_group")
    moveit_service = pre.get("moveit_service", "/get_planning_scene")
    node_ok = any(node == moveit_node or node.endswith("/" + moveit_node) for node in nodes)
    service_ok = moveit_service in services
    moveit_ok = node_ok and service_ok
    items.append(
        _item(
            "moveit",
            "pass" if moveit_ok else "fail",
            {"node": node_ok, "service": service_ok},
            None if moveit_ok else "move_group 未运行或规划服务不可用",
        )
    )

    # 7. shared directory
    shared_ok = probes.dir_writable(cfg.shared_dir)
    files_cfg = pre.get("files", {})
    file_ages = {
        key: probes.file_age_s(cfg.shared_dir / name)
        for key, name in files_cfg.items()
    }
    items.append(
        _item(
            "shared_dir",
            "pass" if shared_ok else "fail",
            {"path": str(cfg.shared_dir), "files": file_ages},
            None if shared_ok else "共享目录不存在或不可写",
        )
    )

    # 8. perception freshness
    det_name = files_cfg.get("detections", "detections.json")
    det_age = probes.file_age_s(cfg.shared_dir / det_name)
    det_topic = pre.get("detections_topic", "/real_piper_yolo/detections")
    det_pub = probes.topic_publishers(det_topic)
    fresh_s = float(pre.get("perception_fresh_s", 10))
    fresh = det_age is not None and det_age <= fresh_s
    topic_alive = bool(det_pub.get("ok")) and (det_pub.get("count") or 0) > 0
    perception_ok = fresh or topic_alive
    items.append(
        _item(
            "perception",
            "pass" if perception_ok else "fail",
            {
                "detections_age_s": round(det_age, 2) if det_age is not None else None,
                "topic_publishers": det_pub.get("count"),
            },
            None if perception_ok else "感知结果未更新；先启动 YOLO 感知模块",
        )
    )

    # 9. display (advisory)
    display_ok = probes.display_available()
    items.append(
        _item(
            "display",
            "pass" if display_ok else "warn",
            {"DISPLAY": display_ok},
            None if display_ok else "无图形会话，识别窗/RViz 无法在本机打开",
        )
    )

    # 10. docker (required for GraspNet)
    docker = probes.docker_available()
    items.append(
        _item(
            "docker",
            "pass" if docker.get("ok") else "fail",
            {"available": docker.get("ok"), "error": docker.get("error")},
            None if docker.get("ok") else "Docker 不可用，GraspNet 无法启动",
        )
    )

    blocking = [
        entry["item"]
        for entry in items
        if entry["item"] in REQUIRED_FOR_GRASP and entry["status"] != "pass"
    ]
    ok_for_read = (
        next(e for e in items if e["item"] == "mode")["status"] == "pass"
        and next(e for e in items if e["item"] == "shared_dir")["status"] == "pass"
    )
    return {
        "ok_for_read": ok_for_read,
        "ok_for_grasp": not blocking,
        "mode": cfg.mode,
        "checked_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "blocking": blocking,
        "items": items,
    }


def build_read_tools(ctx: RuntimeContext) -> list:
    cfg = ctx.config
    pre = cfg.precheck

    @tool
    @audited(ctx, "session_status")
    def session_status() -> dict:
        """查看 agent 当前状态：仿真/真机模式、权限档位、ROS 图规模、已启动模块、运行中的任务。
        用户问"现在什么状态/什么模式/有哪些模块在跑"时调用。"""
        return ctx.status()

    @tool
    @audited(ctx, "precheck_all")
    def precheck_all() -> dict:
        """对当前系统做一次完整体检：模式证据、关节状态、TF 链、控制器、相机、MoveIt、
        共享目录、感知新鲜度、图形会话、Docker。返回逐项 pass/fail 和是否满足抓取条件。
        用户问"能不能抓/系统准备好了吗/哪里有问题"时调用。"""
        return run_precheck(ctx)

    @tool
    @audited(ctx, "perception_detections")
    def perception_detections() -> dict:
        """读取最近一帧 YOLO 检测结果：类别、置信度、包围框、数量、数据新鲜度。
        用户问"看到什么/检测到几个杯子/目标在哪"时调用。"""
        yolo_dir = yolo_shared_dir(ctx)
        source = "yolo_live" if yolo_dir else "default"
        filename = "detections_live.json" if yolo_dir else pre["files"].get("detections", "detections.json")
        result = ctx.probes.read_json(filename, directory=yolo_dir)
        if not result.get("ok"):
            return {
                "ok": False,
                "error": result.get("error"),
                "source": source,
                "path": result.get("path"),
                "hint": "当前 YOLO 尚未产出实时检测帧" if yolo_dir else "感知模块可能未启动，先用 precheck_all 查看 perception 项",
            }
        data = result.get("data", {})
        age = result.get("age_s")
        fresh_s = float(pre.get("perception_fresh_s", 10))
        if age is None or age > fresh_s:
            return {
                "ok": False,
                "source": source,
                "path": result.get("path"),
                "age_s": age,
                "stale": True,
                "frame_sequence": data.get("frame_sequence"),
                "count": 0,
                "detections": [],
                "error": "检测帧已过期，不能作为当前实时结果",
            }
        detections = []
        for entry in data.get("detections", []):
            detections.append(
                {
                    "name": entry.get("name", entry.get("label")),
                    "class_name": entry.get("class_name", entry.get("label")),
                    "confidence": entry.get("confidence", entry.get("score")),
                    "box_xyxy": entry.get("box_xyxy"),
                    "mask_pixels": entry.get("mask_pixels"),
                }
            )
        return {
            "ok": True,
            "source": source,
            "path": result.get("path"),
            "age_s": age,
            "stale": False,
            "frame_sequence": data.get("frame_sequence"),
            "count": len(detections),
            "detections": detections,
        }

    @tool
    @audited(ctx, "grasp_candidates")
    def grasp_candidates(top_k: int = 5) -> dict:
        """读取 GraspNet 生成的抓取候选，按分数从高到低返回前 top_k 个：
        候选名、分数、夹爪宽度、相机坐标系位置。用户问"有哪些抓法/最佳抓取点"时调用。"""
        top_k = max(1, min(int(top_k), 20))
        yolo_dir = yolo_shared_dir(ctx)
        result = ctx.probes.read_csv(
            pre["files"].get("grasps", "perception_grasps.csv"), directory=yolo_dir
        )
        if not result.get("ok"):
            return {
                "ok": False,
                "error": result.get("error"),
                "hint": "GraspNet 可能未运行或尚未产出结果",
            }
        rows = result.get("rows", [])
        if yolo_dir and result.get("age_s", float("inf")) > float(pre.get("perception_fresh_s", 10)):
            return {
                "ok": False,
                "source": "yolo_live",
                "age_s": result.get("age_s"),
                "error": "抓取候选已过期，请重新请求实时感知",
            }
        parsed = []
        for index, row in enumerate(rows):
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
                    "source_dist_xy": as_float(row.get("source_dist_xy")),
                }
            )
        parsed.sort(key=lambda item: item["score"], reverse=True)
        return {
            "ok": True,
            "source": "yolo_live" if yolo_dir else "default",
            "total": len(parsed),
            "age_s": result.get("age_s"),
            "candidates": parsed[:top_k],
        }

    @tool
    @audited(ctx, "grasp_result_latest")
    def grasp_result_latest() -> dict:
        """读取最近一次抓取执行结果：成功与否、状态、阶段、实际夹爪宽度、使用的候选。
        用户问"抓到了吗/上次为什么失败"时调用。"""
        yolo_dir = yolo_shared_dir(ctx)
        result = ctx.probes.read_json(
            "pick_state.json" if yolo_dir else pre["files"].get("grasp_result", "grasp_result.json"),
            directory=yolo_dir,
        )
        if not result.get("ok"):
            return {"ok": False, "error": result.get("error")}
        return {
            "ok": True,
            "source": "yolo_pick_state" if yolo_dir else "default",
            "age_s": result.get("age_s"),
            "result": result.get("data"),
        }

    @tool
    @audited(ctx, "module_logs_tail")
    def module_logs_tail(module: str, lines: int = 30) -> dict:
        """查看某个已启动模块的日志尾部。module 取模块名（如 perception、graspnet、rviz）。
        用户问"为什么启动失败/看下日志"时调用。"""
        return ctx.registry.logs_tail(module, lines)

    @tool
    @audited(ctx, "audit_tail")
    def audit_tail(lines: int = 20) -> dict:
        """查看最近的审计记录（每次工具调用的时间、参数、结果、耗时）。
        用户问"刚才做了什么/操作记录"时调用。"""
        return {"ok": True, "entries": ctx.audit.tail(lines)}

    return [
        session_status,
        precheck_all,
        perception_detections,
        grasp_candidates,
        grasp_result_latest,
        module_logs_tail,
        audit_tail,
    ]
