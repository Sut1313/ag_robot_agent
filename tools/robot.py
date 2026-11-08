from __future__ import annotations

from langchain_core.tools import tool

from runtime.context import RuntimeContext

from .common import audited


def shutdown_robot_tools() -> None:
    """Retained for compatibility; robot tools no longer own an rclpy node."""
    return None


def build_robot_tools(ctx: RuntimeContext) -> list:
    topic = ctx.config.precheck.get("joint_states_topic", "/joint_states")

    @tool
    @audited(ctx, "read_joint_states")
    def read_joint_states() -> dict:
        """读取机械臂六个关节的当前角度（弧度）和夹爪开度（毫米）。
        用户问"现在什么姿态/关节角多少/夹爪开没开"时调用。"""
        result = ctx.probes.echo_once(topic)
        if not result.get("ok") or not isinstance(result.get("value"), dict):
            return {
                "ok": False,
                "error": result.get("error")
                or f"无法读取 {topic}（仿真/驱动是否启动？）",
            }
        msg = result["value"]
        names = msg.get("name") or []
        positions = msg.get("position") or []
        by_name = dict(zip(names, positions)) if names else {}
        joints = {
            f"joint{index}": round(float(by_name[f"joint{index}"]), 4)
            for index in range(1, 7)
            if f"joint{index}" in by_name
        }
        missing = [
            f"joint{index}" for index in range(1, 7) if f"joint{index}" not in by_name
        ]
        gripper = by_name.get("gripper")
        return {
            "ok": not missing,
            "joints_rad": joints,
            "missing_joints": missing,
            "gripper_mm": round(float(gripper) * 1000, 1) if gripper is not None else None,
            "published_names": list(names),
            "stamp": msg.get("header", {}).get("stamp"),
        }

    return [read_joint_states]
