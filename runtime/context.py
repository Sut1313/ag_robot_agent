from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any

from .approval import ApprovalGate
from .audit import AuditLog
from .config import Config
from .grasp_gate import GraspGate
from .jobs import JobManager
from .probes import Probes
from .registry import ProcessRegistry


@dataclass
class RuntimeContext:
    config: Config
    audit: AuditLog
    approval: ApprovalGate
    registry: ProcessRegistry
    jobs: JobManager
    probes: Probes
    grasp_gate: GraspGate = field(default_factory=GraspGate)
    yolo_gate: GraspGate = field(default_factory=GraspGate)
    tabletop_gate: GraspGate = field(default_factory=GraspGate)
    started_at: float = field(default_factory=time.time)

    # ----------------------------------------------------------------- checks

    def mode_evidence(self, snapshot: dict[str, Any] | None = None) -> dict[str, Any]:
        snap = snapshot or self.probes.snapshot()
        topics = set(snap.get("topics", []))
        nodes = snap.get("nodes", [])

        if self.config.mode == "sim":
            has_clock = "/clock" in topics
            has_gz = any("gz_ros2_control" in node for node in nodes)
            return {
                "mode": "sim",
                "ok": has_clock and has_gz,
                "evidence": {
                    "clock_topic": has_clock,
                    "gz_ros2_control_node": has_gz,
                    "node_count": len(nodes),
                },
            }

        has_piper = any(node.rsplit("/", 1)[-1] in {"piper_ctrl_single_node", "piper_single_ctrl"}
                        for node in nodes)
        has_clock = "/clock" in topics
        return {
            "mode": "real",
            "ok": has_piper and not has_clock,
            "evidence": {
                "piper_single_ctrl_node": has_piper,
                "clock_topic_present": has_clock,
                "node_count": len(nodes),
            },
        }

    def require_mode(self) -> tuple[bool, dict[str, Any]]:
        """Hard gate for write/motion tools: runtime evidence must match --mode.

        DDS discovery is occasionally incomplete for a moment (especially just
        after a simulator starts), so retry briefly before refusing.
        """
        evidence = self.mode_evidence()
        for _ in range(3):
            if evidence.get("ok"):
                return True, evidence
            time.sleep(1.0)
            evidence = self.mode_evidence()
        return bool(evidence.get("ok")), evidence

    def status(self) -> dict[str, Any]:
        snapshot = self.probes.snapshot()
        mode = self.mode_evidence(snapshot)
        return {
            "mode": self.config.mode,
            "profile": self.config.profile,
            "mode_evidence": mode,
            "uptime_s": round(time.time() - self.started_at, 1),
            "ros": {
                "nodes": len(snapshot.get("nodes", [])),
                "topics": len(snapshot.get("topics", [])),
                "services": len(snapshot.get("services", [])),
            },
            "modules": self.registry.list(),
            "jobs": self.jobs.list_active(),
            "paths": {
                "shared_dir": str(self.config.shared_dir),
                "logs_dir": str(self.config.paths.logs_dir),
            },
            "shared_dir_writable": self.probes.dir_writable(self.config.shared_dir),
        }
