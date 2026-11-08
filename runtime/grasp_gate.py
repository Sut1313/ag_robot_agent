from __future__ import annotations

import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class PrecheckToken:
    created_at: float
    mode: str
    candidate: str
    score: float
    csv_mtime: float
    dry_job_id: str
    message: str


class GraspGate:
    """Single-use token issued only after a successful dry run.

    Execution requires a fresh token whose candidate and CSV mtime still match.
    The token is cleared after a real execution to prevent replay.
    """

    def __init__(self, valid_s: float = 180.0) -> None:
        self.valid_s = float(valid_s)
        self._token: PrecheckToken | None = None
        self._lock = threading.Lock()

    def issue(
        self,
        *,
        mode: str,
        candidate: str,
        score: float,
        csv_mtime: float,
        dry_job_id: str,
        message: str = "",
    ) -> PrecheckToken:
        preview = PrecheckToken(
            created_at=time.time(),
            mode=mode,
            candidate=candidate,
            score=float(score),
            csv_mtime=float(csv_mtime),
            dry_job_id=dry_job_id,
            message=message,
        )
        with self._lock:
            self._token = preview
        return preview

    def token(self) -> PrecheckToken | None:
        with self._lock:
            return self._token

    def validate(
        self, *, mode: str, candidate: str, csv_mtime: float
    ) -> tuple[bool, str]:
        preview = self.token()
        if preview is None:
            return False, "没有有效的彩排令牌，请先运行 grasp_precheck"
        age = time.time() - preview.created_at
        if age > self.valid_s:
            return False, f"彩排已过期（{age:.0f}s > {self.valid_s:.0f}s），请重新彩排"
        if preview.mode != mode:
            return False, f"模式已变化（{preview.mode} -> {mode}），请重新彩排"
        if abs(float(csv_mtime) - preview.csv_mtime) > 1e-6:
            return False, "候选文件在彩排后发生了变化，请重新彩排"
        if candidate and candidate != preview.candidate:
            return False, f"候选不匹配：彩排的是 {preview.candidate}"
        return True, "ok"

    def clear(self) -> None:
        with self._lock:
            self._token = None
