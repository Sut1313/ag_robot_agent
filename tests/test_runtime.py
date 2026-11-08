#!/usr/bin/env python3
"""P1 runtime tests. Run with the agent venv:

    python tests/test_runtime.py
"""
from __future__ import annotations

import json
import os
import sys
import tempfile
import time
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from runtime.approval import ApprovalGate
from runtime.audit import AuditLog
from runtime.config import load_config
from runtime.jobs import JobManager
from runtime.registry import ProcessRegistry, sanitized_env


def test_env_sanitized() -> None:
    with patch.dict(os.environ, {
        "VIRTUAL_ENV": "/tmp/agent-env",
        "PATH": "/tmp/agent-env/bin:/usr/bin:/bin",
    }):
        env = sanitized_env()
    assert "VIRTUAL_ENV" not in env
    assert "/tmp/agent-env/bin" not in env["PATH"].split(":")
    assert "/usr/bin" in env["PATH"].split(":")
    print("ok  env sanitized")


def test_audit_redaction(tmp: Path) -> None:
    log = AuditLog(tmp / "audit.jsonl", mode="sim", profile="read")
    log.record("demo", args={"api_key": "sk-secret", "nested": {"token": "abc"}})
    entries = log.tail(5)
    assert len(entries) == 1
    assert entries[0]["args"]["api_key"] == "***"
    assert entries[0]["args"]["nested"]["token"] == "***"
    raw = (tmp / "audit.jsonl").read_text(encoding="utf-8")
    assert "sk-secret" not in raw
    print("ok  audit redaction")


def test_registry(tmp: Path) -> None:
    reg = ProcessRegistry(tmp)
    first = reg.start("demo", ["bash", "-c", "echo hello-from-child; sleep 30"])
    assert first["started"] is True
    second = reg.start("demo", ["bash", "-c", "sleep 30"])
    assert second["already_running"] is True
    assert second["pid"] == first["pid"]

    time.sleep(0.5)
    tail = reg.logs_tail("demo", 10)
    assert tail["ok"] is True
    assert any("hello-from-child" in line for line in tail["tail"])

    status = reg.status("demo")
    assert status["alive"] is True

    stopped = reg.stop("demo", timeout_s=3)
    assert stopped["stopped"] is True
    assert reg.is_running("demo") is False

    reg.start("a", ["sleep", "30"])
    reg.start("b", ["sleep", "30"])
    results = reg.stop_all(timeout_s=3)
    assert {item["name"] for item in results} == {"a", "b"}
    assert all(item["stopped"] for item in results)
    assert reg.list() == []
    print("ok  registry start/idempotent/log/stop/stop_all")


def test_jobs() -> None:
    manager = JobManager()

    def quick(job):
        job.stage = "working"
        return {"value": 42}

    job = manager.submit("quick", quick)
    for _ in range(50):
        state = manager.poll(job.job_id)
        if state["status"] in {"succeeded", "failed"}:
            break
        time.sleep(0.05)
    assert state["status"] == "succeeded", state
    assert state["result"] == {"value": 42}

    def slow(job):
        for _ in range(100):
            if job.cancel_requested:
                return {"cancelled": True}
            time.sleep(0.05)
        return {"cancelled": False}

    blocked = manager.submit("slow-exclusive", slow, exclusive=True)
    time.sleep(0.1)
    rejected = manager.submit("second-exclusive", quick, exclusive=True)
    assert rejected.status == "rejected"
    manager.cancel(blocked.job_id)
    for _ in range(60):
        state = manager.poll(blocked.job_id)
        if state["status"] in {"cancelled", "succeeded", "failed"}:
            break
        time.sleep(0.05)
    assert state["status"] == "cancelled", state
    print("ok  jobs submit/poll/exclusive/cancel")


def test_approval() -> None:
    yes = ApprovalGate(auto_confirm=False, reader=lambda _t: "yes")
    assert yes.require("move", {"x": 1})[0] is True

    no = ApprovalGate(auto_confirm=False, reader=lambda _t: "y")
    assert no.require("move")[0] is False

    timeout = ApprovalGate(auto_confirm=False, reader=lambda _t: None)
    approved, reason = timeout.require("move")
    assert approved is False and "超时" in reason

    auto = ApprovalGate(auto_confirm=True, reader=lambda _t: None)
    assert auto.require("move")[0] is True
    assert auto.require("move", force_confirm=True)[0] is False
    print("ok  approval yes/deny/timeout/auto-confirm")


def test_yaml_extraction() -> None:
    from runtime.probes import extract_yaml_document

    chatter_before = (
        "A message was lost!!!\n"
        "\ttotal count change:1\n"
        "\ttotal count: 1---\n"
        "header:\n"
        "  sec: 1\n"
        "---\n"
    )
    assert extract_yaml_document(chatter_before).startswith("header:")

    chatter_after_separator = (
        "---\n"
        "A message was lost!!!\n"
        "\ttotal count: 1\n"
        "- 0.1\n"
        "- 0.2\n"
        "---\n"
    )
    assert extract_yaml_document(chatter_after_separator) == "- 0.1\n- 0.2"
    print("ok  yaml extraction around DDS chatter")


def test_config() -> None:
    cfg = load_config("sim", "read")
    tools = cfg.tools_for_profile()
    assert "session_status" in tools
    assert "read_joint_states" in tools
    grasp_tools = cfg.tools_for_profile("grasp")
    assert "grasp_execute" in grasp_tools and "estop" in grasp_tools
    print("ok  config load and profile expansion")


def test_safe_rosa_filters_builtin_writes() -> None:
    """ROSA injects ros2_service_call/ros2_param_set by default; they must be gone."""
    import argparse

    from agent import SafeROSA, build_context, build_prompts, build_tools
    from langchain_core.language_models.fake_chat_models import FakeListChatModel

    class ToolCallingFake(FakeListChatModel):
        def bind_tools(self, tools, **kwargs):
            return self

    args = argparse.Namespace(
        mode="sim", profile="read", config_dir=None, auto_confirm_sim=False
    )
    ctx = build_context(args)
    agent = SafeROSA(
        ros_version=2,
        llm=ToolCallingFake(responses=["OK"]),
        tools=build_tools(ctx, "read"),
        prompts=build_prompts(ctx),
        streaming=True,
    )
    names = {tool.name for tool in agent._ROSA__tools.get_tools()}
    assert "ros2_service_call" not in names
    assert "ros2_param_set" not in names
    assert "ros2_node_list" in names
    print("ok  generic ROS write tools filtered")


def test_scan_report_parser() -> None:
    from tools.grasp import parse_scan_report

    ok, counts = parse_scan_report(
        "scan_only report: candidates=300 tested=3, gate_rejected=147, "
        "approach_pass=0, grasp_pass=0, lift_pass=0 -> 无全过候选"
    )
    assert ok is False and counts["lift_pass"] == 0

    ok2, counts2 = parse_scan_report("approach_pass=2, grasp_pass=1, lift_pass=1")
    assert ok2 is True and counts2["grasp_pass"] == 1

    ok3, _ = parse_scan_report("")
    assert ok3 is False
    print("ok  scan report parser (fails closed)")


def test_grasp_gate() -> None:
    from runtime.grasp_gate import GraspGate

    gate = GraspGate(valid_s=180)
    assert gate.validate(mode="sim", candidate="block_0", csv_mtime=1.0)[0] is False

    preview = gate.issue(
        mode="sim", candidate="block_0", score=1.2, csv_mtime=1.0, dry_job_id="job_1"
    )
    assert preview.candidate == "block_0"
    assert gate.validate(mode="sim", candidate="block_0", csv_mtime=1.0)[0] is True
    assert gate.validate(mode="sim", candidate="other", csv_mtime=1.0)[0] is False
    assert gate.validate(mode="sim", candidate="block_0", csv_mtime=2.0)[0] is False
    assert gate.validate(mode="real", candidate="block_0", csv_mtime=1.0)[0] is False

    gate.clear()
    assert gate.validate(mode="sim", candidate="block_0", csv_mtime=1.0)[0] is False

    expired = GraspGate(valid_s=0.01)
    expired.issue(
        mode="sim", candidate="block_0", score=1.0, csv_mtime=1.0, dry_job_id="job_2"
    )
    time.sleep(0.05)
    ok, reason = expired.validate(mode="sim", candidate="block_0", csv_mtime=1.0)
    assert ok is False and "过期" in reason
    print("ok  grasp gate token lifecycle")


def test_precheck_pass_detection() -> None:
    from tools.grasp import precheck_passed

    call = {
        "ok": True,
        "success": True,
        "message": "scan_only report: ... lift_pass=0 -> 候选 [1] top_a 全过",
    }
    report = {"selected": {"xyz": [0, 0, 0]}, "approved": [{}], "selected_index": 1}
    assert precheck_passed(call, report) is True

    no = {
        "ok": True,
        "success": True,
        "message": "scan_only report: ... lift_pass=0 -> 无全过候选",
    }
    assert precheck_passed(no, {"selected": None, "approved": []}) is False

    counted = {"ok": True, "success": True, "message": "approach_pass=2, lift_pass=1"}
    assert precheck_passed(counted, None) is True
    print("ok  precheck pass detection")


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="rosa-test-") as raw:
        tmp = Path(raw)
        test_env_sanitized()
        test_audit_redaction(tmp)
        test_registry(tmp)
        test_jobs()
        test_approval()
        test_yaml_extraction()
        test_config()
        test_safe_rosa_filters_builtin_writes()
        test_scan_report_parser()
        test_grasp_gate()
        test_precheck_pass_detection()
    print("\nall runtime tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
