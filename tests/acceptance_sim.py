#!/usr/bin/env python3
"""P6 acceptance checks safe to run against a simulation.

Usage:
    export ROS_DOMAIN_ID=42 GZ_PARTITION=agent42
    source /opt/ros/humble/setup.bash
    python tests/acceptance_sim.py

The script starts and stops MoveIt/perception itself. It never commands arm
motion: it only exercises precheck, idempotency, refusals, estop and cleanup.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from agent import build_context, build_tools  # noqa: E402
from tools import shutdown_robot_tools  # noqa: E402

RESULTS: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    RESULTS.append((name, bool(ok), detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))


def main() -> int:
    args = argparse.Namespace(
        mode="sim", profile="grasp", config_dir=None, auto_confirm_sim=True
    )
    ctx = build_context(args)
    tools = {t.name: t for t in build_tools(ctx, "grasp")}

    ok, evidence = ctx.require_mode()
    if not ok:
        print(f"[ABORT] 仿真不可达或模式不匹配: {evidence}")
        print("        请先启动仿真，并确认 ROS_DOMAIN_ID / GZ_PARTITION 一致。")
        return 2

    try:
        # --- T01: precheck all green -------------------------------------
        r = tools["moveit_start"].invoke({"with_rviz": False})
        check("T01a moveit_start", r.get("ok") is True, str(r.get("error") or ""))
        r = tools["perception_start"].invoke({"show_window": False})
        check("T01b perception_start", r.get("ok") is True, str(r.get("error") or ""))

        pre = tools["precheck_all"].invoke({})
        items = {item["item"]: item["status"] for item in pre["items"]}
        required = ("mode", "joint_states", "tf", "controllers", "camera", "moveit", "perception")
        bad = [name for name in required if items.get(name) != "pass"]
        check("T01c precheck_all", not bad, f"failing={bad}" if bad else "all required pass")

        # --- T02: idempotent start ---------------------------------------
        status_before = ctx.registry.status("perception")
        again = tools["perception_start"].invoke({"show_window": False})
        check(
            "T02 idempotent perception start",
            again.get("already_running") is True
            and again.get("pid") == status_before.get("pid"),
            f"pid={status_before.get('pid')}",
        )

        # --- T03: detection contract -------------------------------------
        det = tools["perception_detections"].invoke({})
        check(
            "T03 detection contract",
            det.get("ok") is True and det.get("frame_sequence") is not None,
            f"count={det.get('count')} age={det.get('age_s')}",
        )

        # --- T04: execute without token refused --------------------------
        ex = tools["grasp_execute"].invoke({})
        check(
            "T04 execute refused without token",
            ex.get("ok") is False and "令牌" in str(ex.get("error", "")),
            str(ex.get("error", "")),
        )

        # --- T05: mode mismatch refused ----------------------------------
        real_args = argparse.Namespace(
            mode="real", profile="operate", config_dir=None, auto_confirm_sim=False
        )
        real_ctx = build_context(real_args)
        mode_ok, evidence = real_ctx.require_mode()
        check(
            "T05a real mode evidence rejected on sim",
            mode_ok is False,
            str(evidence.get("evidence")),
        )
        real_tools = {t.name: t for t in build_tools(real_ctx, "operate")}
        guarded = real_tools["perception_start"].invoke({"show_window": False})
        check(
            "T05b mode guard blocks start",
            guarded.get("ok") is False and "不匹配" in str(guarded.get("error", "")),
            str(guarded.get("error", "")),
        )

        # --- T06: estop / recover ----------------------------------------
        stop = tools["estop"].invoke({})
        states = ctx.probes.controller_states().get("controllers", {})
        check(
            "T06a estop deactivates controllers",
            stop.get("ok") is True and states.get("arm_controller") == "inactive",
            f"arm={states.get('arm_controller')}",
        )
        rec = tools["recover"].invoke({})
        states = ctx.probes.controller_states().get("controllers", {})
        check(
            "T06b recover reactivates controllers",
            rec.get("ok") is True and states.get("arm_controller") == "active",
            f"arm={states.get('arm_controller')}",
        )

        # --- T07: audit trail --------------------------------------------
        entries = ctx.audit.tail(50)
        names = {entry.get("tool") for entry in entries}
        check(
            "T07 audit trail",
            {"precheck_all", "perception_start", "estop"} <= names,
            f"{len(entries)} entries",
        )

        # --- T08: cleanup leaves no orphans ------------------------------
        tools["perception_stop"].invoke({})
        tools["moveit_stop"].invoke({})
        time.sleep(3)
        leftovers = subprocess.run(
            ["pgrep", "-af", f"{ctx.config.shared_dir}|viewer_yolo.py"],
            capture_output=True,
            text=True,
        ).stdout.strip()
        leftovers = "\n".join(
            line
            for line in leftovers.splitlines()
            if "acceptance_sim" not in line and "pgrep" not in line
        )
        check("T08 no orphan processes", not leftovers, leftovers[:200])
    finally:
        ctx.registry.stop_all(timeout_s=15)
        shutdown_robot_tools()

    failed = [name for name, ok, _ in RESULTS if not ok]
    print(f"\n{len(RESULTS) - len(failed)}/{len(RESULTS)} checks passed")
    if failed:
        print("failed:", ", ".join(failed))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
