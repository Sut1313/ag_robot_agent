from __future__ import annotations

import csv
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import yaml

from .registry import sanitized_env

_NUM_RE = re.compile(r"^\s*-?\d+(\.\d+)?([eE][+-]?\d+)?\s*$")
_ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")


def strip_ansi(text: str) -> str:
    return _ANSI_RE.sub("", text)


def extract_yaml_document(text: str) -> str:
    """Drop CLI chatter (e.g. 'A message was lost!!!') around the YAML body.

    ``ros2 topic echo`` separates documents with ``---`` and some DDS
    implementations print warnings to stdout before the first separator, often
    with leading tabs and sometimes glued to the separator.
    """
    cleaned = strip_ansi(text).strip()
    cleaned = re.sub(r"---\s*$", "", cleaned).strip()
    lines = cleaned.splitlines()

    start = 0
    for index, line in enumerate(lines):
        if line.strip() == "---":
            continue
        stripped = line.lstrip()
        if (
            re.match(r"^[A-Za-z_][A-Za-z0-9_]*:", line)
            or stripped.startswith("- ")
            or stripped.startswith("[")
            or stripped.startswith("{")
        ):
            start = index
            break

    body = lines[start:]
    for index, line in enumerate(body):
        if line.strip() == "---":
            body = body[:index]
            break
    return "\n".join(body).strip()


class Probes:
    """Read-only observations of the ROS graph and the on-disk contracts."""

    def __init__(self, config: Any) -> None:
        self.config = config
        self.cli_timeout = float(config.limits.get("timeouts", {}).get("ros_cli_s", 5))
        self.tf_timeout = float(config.limits.get("timeouts", {}).get("tf_s", 2.5))

    # ------------------------------------------------------------- low level

    def run(
        self, cmd: list[str], timeout: float | None = None
    ) -> tuple[int, str, str]:
        try:
            proc = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout or self.cli_timeout,
                env=sanitized_env(),
            )
            return proc.returncode, proc.stdout, proc.stderr
        except subprocess.TimeoutExpired:
            return 124, "", f"timeout after {timeout or self.cli_timeout}s: {' '.join(cmd)}"
        except FileNotFoundError as exc:
            return 127, "", str(exc)

    def _list(self, cmd: list[str]) -> list[str]:
        code, out, _ = self.run(cmd)
        if code != 0:
            return []
        return [line.strip() for line in out.splitlines() if line.strip()]

    def service_names(self) -> list[str]:
        return self._list(["ros2", "service", "list"])

    def node_names(self) -> list[str]:
        return self._list(["ros2", "node", "list"])

    def snapshot(self) -> dict[str, Any]:
        return {
            "nodes": self._list(["ros2", "node", "list"]),
            "topics": self._list(["ros2", "topic", "list"]),
            "services": self._list(["ros2", "service", "list"]),
        }

    # ------------------------------------------------------------- ROS probes

    def echo_once(self, topic: str, field: str | None = None) -> dict[str, Any]:
        # Always fetch the full message: `--once --field` can stream several
        # messages for high-rate topics and is not reliably terminated.
        cmd = ["ros2", "topic", "echo", topic, "--once"]
        code, out, err = self.run(cmd)
        if code != 0:
            return {"ok": False, "error": strip_ansi((err or out)).strip()[:500]}
        text = extract_yaml_document(out)
        if not text:
            return {"ok": False, "error": "empty YAML document", "raw": out[:300]}
        try:
            value = yaml.safe_load(text)
        except yaml.YAMLError as exc:
            return {"ok": False, "error": f"yaml: {exc}", "raw": text[:500]}
        if field:
            current = value
            for part in field.split("."):
                if isinstance(current, dict) and part in current:
                    current = current[part]
                else:
                    return {"ok": False, "error": f"field not found: {field}"}
            value = current
        return {"ok": True, "value": value}

    def topic_publishers(self, topic: str) -> dict[str, Any]:
        code, out, err = self.run(["ros2", "topic", "info", topic])
        if code != 0:
            return {"ok": False, "error": (err or out).strip()[:300], "count": None}
        match = re.search(r"Publisher count:\s*(\d+)", out)
        return {
            "ok": True,
            "count": int(match.group(1)) if match else None,
            "raw": out.strip()[:500],
        }

    def tf_ok(self, parent: str, child: str) -> dict[str, Any]:
        """Look up a TF chain in a separate process.

        Running tf2 inside the agent process races with the joint-state
        subscriber's executor (rclpy wait-set invalidation), so isolate it.
        """
        script = Path(__file__).resolve().parent.parent / "scripts" / "tf_probe.py"
        use_sim = "1" if self.config.mode == "sim" else "0"
        code, out, err = self.run(
            [sys.executable, str(script), parent, child, use_sim],
            timeout=self.tf_timeout + 3.0,
        )
        line = ""
        for candidate in reversed(out.splitlines()):
            if candidate.strip().startswith("{"):
                line = candidate.strip()
                break
        if line:
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                payload = None
            if payload is not None:
                payload.setdefault("parent", parent)
                payload.setdefault("child", child)
                if payload.get("ok") and payload.get("translation"):
                    payload["translation"] = [
                        round(float(v), 4) for v in payload["translation"]
                    ]
                return payload
        return {
            "ok": False,
            "parent": parent,
            "child": child,
            "error": (err or out).strip()[:300] or f"tf probe failed (code {code})",
        }

    def controller_states(self) -> dict[str, Any]:
        last_error: str | None = None
        for _ in range(5):
            code, out, err = self.run(["ros2", "control", "list_controllers"])
            if code == 0:
                controllers: dict[str, str] = {}
                for line in strip_ansi(out).splitlines():
                    parts = line.split()
                    if len(parts) < 2:
                        continue
                    name = parts[0].split("[", 1)[0].strip()
                    controllers[name] = parts[-1].strip()
                if controllers:
                    return {"ok": True, "controllers": controllers}
                last_error = "empty controller list"
            else:
                last_error = (err or out).strip()[:300]
            time.sleep(0.4)
        return {
            "ok": False,
            "error": last_error or "controller list unavailable",
            "controllers": {},
        }

    # ------------------------------------------------------------ host probes

    def docker_available(self) -> dict[str, Any]:
        code, out, err = self.run(["docker", "info"], timeout=5)
        return {"ok": code == 0, "error": None if code == 0 else (err or out)[:300]}

    @staticmethod
    def display_available() -> bool:
        return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))

    @staticmethod
    def dir_writable(path: str | Path) -> bool:
        target = Path(path)
        return target.is_dir() and os.access(target, os.W_OK)

    @staticmethod
    def file_age_s(path: str | Path) -> float | None:
        try:
            return max(0.0, time.time() - Path(path).stat().st_mtime)
        except OSError:
            return None

    # ------------------------------------------------------------ file probes

    def read_json(self, name: str, directory: str | Path | None = None) -> dict[str, Any]:
        path = (Path(directory) if directory is not None else self.config.shared_dir) / name
        age = self.file_age_s(path)
        if age is None:
            return {"ok": False, "error": f"missing: {path}", "path": str(path)}
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            return {"ok": False, "error": str(exc), "path": str(path)}
        return {"ok": True, "data": data, "age_s": round(age, 2), "path": str(path)}

    def read_csv(self, name: str, directory: str | Path | None = None) -> dict[str, Any]:
        path = (Path(directory) if directory is not None else self.config.shared_dir) / name
        age = self.file_age_s(path)
        if age is None:
            return {"ok": False, "error": f"missing: {path}", "path": str(path)}
        try:
            with open(path, "r", encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
        except OSError as exc:
            return {"ok": False, "error": str(exc), "path": str(path)}
        return {"ok": True, "rows": rows, "age_s": round(age, 2), "path": str(path)}


def as_float(value: Any, default: float | None = None) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default
