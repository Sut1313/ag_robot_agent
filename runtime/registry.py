from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


def sanitized_env(extra: dict[str, str] | None = None) -> dict[str, str]:
    """Return an environment for child processes.

    The agent itself runs inside a venv that carries numpy 2.x. ROS nodes and
    the perception stack must not inherit that interpreter or its PATH, so we
    strip the venv markers. ROS/ament variables are left untouched.
    """
    env = {
        key: value
        for key, value in os.environ.items()
        if key not in {"VIRTUAL_ENV", "PYTHONHOME", "PYTHONUSERBASE"}
    }
    venv = os.environ.get("VIRTUAL_ENV")
    prefixes = {Path(path) / "bin" for path in (venv, sys.prefix if sys.prefix != sys.base_prefix else None) if path}
    path = env.get("PATH", "")
    env["PATH"] = ":".join(
        part
        for part in path.split(":")
        if part and Path(part) not in prefixes
    )
    if extra:
        env.update({str(k): str(v) for k, v in extra.items()})
    return env


def _proc_stat(pid: int) -> tuple[str, str] | None:
    """Return (starttime, state) from /proc/<pid>/stat.

    ``state == 'Z'`` means the process has exited but has not been reaped yet;
    it must count as dead, otherwise a stopped child never looks stopped.
    """
    try:
        raw = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    except OSError:
        return None
    try:
        after_comm = raw.rsplit(")", 1)[1].split()
        return after_comm[19], after_comm[0]
    except (IndexError, ValueError):
        return None


def _proc_starttime(pid: int) -> str | None:
    info = _proc_stat(pid)
    return info[0] if info else None


@dataclass
class ProcessRecord:
    name: str
    pid: int
    pgid: int
    cmd: list[str]
    cwd: str | None
    log_path: str
    started_at: float
    starttime: str

    @property
    def alive(self) -> bool:
        info = _proc_stat(self.pid)
        if info is None:
            return False
        starttime, state = info
        return starttime == self.starttime and state not in {"Z", "X"}


class ProcessRegistry:
    """Book-keeping for every long-running process the agent starts.

    Records are persisted so a restarted agent can still see (and stop)
    processes started by an earlier agent instance. Dead or PID-reused records
    are pruned on load.
    """

    def __init__(self, logs_dir: str | Path, *, workspace_env: dict[str, str] | None = None) -> None:
        self.logs_dir = Path(logs_dir)
        self.workspace_env = workspace_env or {}
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        self.state_path = self.logs_dir / "registry.json"
        self._lock = threading.RLock()
        self._records: dict[str, ProcessRecord] = {}
        self._popen: dict[str, subprocess.Popen] = {}
        self._load()

    # ------------------------------------------------------------------ state

    def _load(self) -> None:
        try:
            raw = json.loads(self.state_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return
        if not isinstance(raw, dict):
            return
        for name, item in raw.items():
            try:
                record = ProcessRecord(
                    name=item["name"],
                    pid=int(item["pid"]),
                    pgid=int(item["pgid"]),
                    cmd=list(item["cmd"]),
                    cwd=item.get("cwd"),
                    log_path=item["log_path"],
                    started_at=float(item["started_at"]),
                    starttime=str(item["starttime"]),
                )
            except (KeyError, TypeError, ValueError):
                continue
            if record.alive:
                self._records[name] = record

    def _persist(self) -> None:
        data = {
            name: asdict(record)
            for name, record in self._records.items()
            if record.alive
        }
        tmp = self.state_path.with_suffix(".tmp")
        try:
            tmp.write_text(
                json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            os.replace(tmp, self.state_path)
        except OSError:
            pass

    # ---------------------------------------------------------------- queries

    def get(self, name: str) -> ProcessRecord | None:
        with self._lock:
            proc = self._popen.get(name)
            if proc is not None and proc.poll() is not None:
                self._popen.pop(name, None)
                self._records.pop(name, None)
                self._persist()
                return None
            record = self._records.get(name)
            if record is not None and not record.alive:
                self._records.pop(name, None)
                self._popen.pop(name, None)
                return None
            return record

    def is_running(self, name: str) -> bool:
        record = self.get(name)
        return record is not None

    def list(self) -> list[dict[str, Any]]:
        with self._lock:
            self._prune()
            return [self._status(record) for record in self._records.values()]

    def _prune(self) -> None:
        dead = [name for name, rec in self._records.items() if not rec.alive]
        for name in dead:
            self._records.pop(name, None)
        if dead:
            self._persist()

    def _status(self, record: ProcessRecord) -> dict[str, Any]:
        return {
            "name": record.name,
            "alive": record.alive,
            "pid": record.pid,
            "pgid": record.pgid,
            "log": record.log_path,
            "started_at": datetime.fromtimestamp(record.started_at).isoformat(
                timespec="seconds"
            ),
            "cmd": record.cmd,
        }

    def status(self, name: str) -> dict[str, Any]:
        record = self.get(name)
        if record is None:
            return {"name": name, "alive": False, "pid": None, "log": None}
        return self._status(record)

    # --------------------------------------------------------------- lifecycle

    def start(
        self,
        name: str,
        cmd: list[str],
        cwd: str | Path | None = None,
        env: dict[str, str] | None = None,
    ) -> dict[str, Any]:
        if not cmd or not all(isinstance(part, str) for part in cmd):
            raise ValueError("cmd must be a non-empty list of strings")

        with self._lock:
            existing = self.get(name)
            if existing is not None:
                return {
                    "started": False,
                    "already_running": True,
                    "pid": existing.pid,
                    "log": existing.log_path,
                }

            log_path = self.logs_dir / f"{name}.log"
            header = (
                f"\n===== {datetime.now().isoformat(timespec='seconds')} "
                f"start: {' '.join(cmd)} =====\n"
            )
            with open(log_path, "ab", buffering=0) as log_file:
                log_file.write(header.encode("utf-8"))
                proc = subprocess.Popen(
                    cmd,
                    cwd=str(cwd) if cwd else None,
                    env=sanitized_env({**self.workspace_env, **(env or {})}),
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )

            starttime = _proc_starttime(proc.pid)
            if starttime is None:
                raise RuntimeError(
                    f"process {name} started but /proc/{proc.pid} is unreadable"
                )

            record = ProcessRecord(
                name=name,
                pid=proc.pid,
                pgid=os.getpgid(proc.pid),
                cmd=cmd,
                cwd=str(cwd) if cwd else None,
                log_path=str(log_path),
                started_at=time.time(),
                starttime=starttime,
            )
            self._records[name] = record
            self._popen[name] = proc
            self._persist()
            return {
                "started": True,
                "already_running": False,
                "pid": record.pid,
                "log": record.log_path,
            }

    def stop(self, name: str, timeout_s: float = 10.0) -> dict[str, Any]:
        with self._lock:
            record = self.get(name)
            if record is None:
                return {"stopped": True, "was_running": False, "pid": None}

            proc = self._popen.get(name)
            _signal_group(record.pgid, signal.SIGINT)
            deadline = time.time() + max(0.5, timeout_s)
            while time.time() < deadline:
                if not record.alive:
                    break
                if proc is not None and proc.poll() is not None:
                    break
                time.sleep(0.1)

            forced = False
            if record.alive:
                forced = True
                _signal_group(record.pgid, signal.SIGKILL)
                for _ in range(20):
                    if not record.alive:
                        break
                    time.sleep(0.1)

            self._records.pop(name, None)
            self._popen.pop(name, None)
            if proc is not None:
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    forced = True
            self._persist()
            return {
                "stopped": not record.alive,
                "was_running": True,
                "pid": record.pid,
                "forced": forced,
            }

    def stop_all(self, timeout_s: float = 10.0, *, exclude=()) -> list[dict[str, Any]]:
        names = [item["name"] for item in self.list() if item["name"] not in exclude]
        return [{"name": name, **self.stop(name, timeout_s)} for name in names]

    def logs_tail(self, name: str, lines: int = 30) -> dict[str, Any]:
        record = self.get(name)
        if record is None:
            known = [item["name"] for item in self.list()]
            return {"ok": False, "error": f"unknown module: {name}", "known": known}
        lines = max(1, min(int(lines), 500))
        try:
            content = Path(record.log_path).read_text(
                encoding="utf-8", errors="replace"
            ).splitlines()
        except OSError as exc:
            return {"ok": False, "error": str(exc), "log": record.log_path}
        return {
            "ok": True,
            "name": name,
            "log": record.log_path,
            "tail": content[-lines:],
        }


def _signal_group(pgid: int, sig: int) -> None:
    try:
        os.killpg(pgid, sig)
    except ProcessLookupError:
        return
    except PermissionError:
        try:
            os.kill(pgid, sig)
        except OSError:
            return
