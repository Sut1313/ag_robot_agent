from __future__ import annotations

import json
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

_SENSITIVE = ("api_key", "apikey", "authorization", "token", "password", "secret")


def _redact(value: Any) -> Any:
    if isinstance(value, dict):
        cleaned = {}
        for key, item in value.items():
            if any(marker in str(key).lower() for marker in _SENSITIVE):
                cleaned[key] = "***"
            else:
                cleaned[key] = _redact(item)
        return cleaned
    if isinstance(value, (list, tuple)):
        return [_redact(item) for item in value]
    return value


def _truncate(value: Any, limit: int) -> Any:
    if isinstance(value, str) and len(value) > limit:
        return value[:limit] + f"...({len(value)} chars)"
    if isinstance(value, dict):
        return {k: _truncate(v, limit) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_truncate(item, limit) for item in value]
    return value


class AuditLog:
    """Append-only JSONL audit trail. Never raises into tool code."""

    def __init__(
        self,
        path: str | Path,
        mode: str,
        profile: str,
        max_field_chars: int = 2000,
    ) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.mode = mode
        self.profile = profile
        self.max_field_chars = max_field_chars
        self._lock = threading.Lock()

    def record(
        self,
        tool: str,
        args: dict[str, Any] | None = None,
        result: Any = None,
        approved: bool | None = None,
        job_id: str | None = None,
        duration_ms: float | None = None,
        evidence: dict[str, Any] | None = None,
        error: str | None = None,
    ) -> dict[str, Any]:
        entry: dict[str, Any] = {
            "ts": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "mode": self.mode,
            "profile": self.profile,
            "tool": tool,
        }
        if args is not None:
            entry["args"] = _truncate(_redact(args), self.max_field_chars)
        if result is not None:
            entry["result"] = _truncate(_redact(result), self.max_field_chars)
        if approved is not None:
            entry["approved"] = approved
        if job_id is not None:
            entry["job_id"] = job_id
        if duration_ms is not None:
            entry["duration_ms"] = round(duration_ms, 1)
        if evidence:
            entry["evidence"] = _truncate(_redact(evidence), self.max_field_chars)
        if error:
            entry["error"] = _truncate(error, self.max_field_chars)

        line = json.dumps(entry, ensure_ascii=False)
        try:
            with self._lock:
                with open(self.path, "a", encoding="utf-8") as handle:
                    handle.write(line + "\n")
        except OSError:
            # Auditing must never take the robot down.
            pass
        return entry

    def tail(self, lines: int = 20) -> list[dict[str, Any]]:
        lines = max(1, min(int(lines), 200))
        try:
            content = self.path.read_text(encoding="utf-8").splitlines()
        except OSError:
            return []
        out: list[dict[str, Any]] = []
        for raw in content[-lines:]:
            try:
                out.append(json.loads(raw))
            except json.JSONDecodeError:
                continue
        return out


class Timed:
    """Context manager used by tool wrappers to record duration."""

    def __init__(self) -> None:
        self.start = time.perf_counter()
        self.elapsed_ms = 0.0

    def __enter__(self) -> "Timed":
        return self

    def __exit__(self, *exc: object) -> None:
        self.elapsed_ms = (time.perf_counter() - self.start) * 1000.0
