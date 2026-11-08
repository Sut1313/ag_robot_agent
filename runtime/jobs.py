from __future__ import annotations

import threading
import time
import traceback
from dataclasses import dataclass, field
from typing import Any, Callable


@dataclass
class Job:
    job_id: str
    name: str
    status: str = "pending"  # pending|running|succeeded|failed|cancelled|timeout|rejected
    stage: str = "queued"
    result: Any = None
    error: str | None = None
    started_at: float = field(default_factory=time.time)
    finished_at: float | None = None
    timed_out: bool = False
    cancel_event: threading.Event = field(default_factory=threading.Event)
    deadline: float | None = None

    @property
    def cancel_requested(self) -> bool:
        return self.cancel_event.is_set()

    def elapsed_s(self) -> float:
        end = self.finished_at or time.time()
        return end - self.started_at


class JobManager:
    """Background jobs for blocking ROS service calls and long operations.

    Tools must return a job id immediately instead of blocking the ReAct loop.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._jobs: dict[str, Job] = {}
        self._counter = 0
        self._exclusive = threading.Lock()

    def submit(
        self,
        name: str,
        fn: Callable[[Job], Any],
        timeout_s: float | None = None,
        exclusive: bool = False,
    ) -> Job:
        with self._lock:
            self._counter += 1
            job_id = f"job_{self._counter}"
            job = Job(job_id=job_id, name=name)
            if timeout_s is not None:
                job.deadline = time.time() + float(timeout_s)
            self._jobs[job_id] = job

        if exclusive and not self._exclusive.acquire(blocking=False):
            job.status = "rejected"
            job.stage = "rejected"
            job.error = "another exclusive job is already running"
            job.finished_at = time.time()
            return job

        def worker() -> None:
            acquired = exclusive
            try:
                job.status = "running"
                job.stage = "running"
                result = fn(job)
                job.result = result
                if job.timed_out:
                    job.status = "timeout"
                    job.stage = "timeout"
                elif job.cancel_requested:
                    job.status = "cancelled"
                    job.stage = "cancelled"
                else:
                    job.status = "succeeded"
                    job.stage = "done"
            except Exception as exc:  # noqa: BLE001 - surfaced via job status
                job.status = "failed"
                job.stage = "failed"
                job.error = f"{type(exc).__name__}: {exc}\n{traceback.format_exc(limit=3)}"
            finally:
                job.finished_at = time.time()
                if acquired:
                    try:
                        self._exclusive.release()
                    except RuntimeError:
                        pass

        thread = threading.Thread(target=worker, name=f"job-{job_id}", daemon=True)
        thread.start()

        if timeout_s is not None:
            timer = threading.Timer(float(timeout_s), self._expire, args=(job_id,))
            timer.daemon = True
            timer.start()

        return job

    def _expire(self, job_id: str) -> None:
        with self._lock:
            job = self._jobs.get(job_id)
        if job is None or job.finished_at is not None:
            return
        job.timed_out = True
        job.stage = "timeout-requested"
        job.cancel_event.set()

    def get(self, job_id: str) -> Job | None:
        with self._lock:
            return self._jobs.get(job_id)

    def latest(self, name: str | None = None) -> Job | None:
        with self._lock:
            jobs = [job for job in self._jobs.values() if name is None or job.name == name]
        return max(jobs, key=lambda job: job.started_at) if jobs else None

    def poll(self, job_id: str) -> dict[str, Any]:
        job = self.get(job_id)
        if job is None:
            return {"ok": False, "error": f"unknown job: {job_id}"}
        return {
            "ok": True,
            "job_id": job.job_id,
            "name": job.name,
            "status": job.status,
            "stage": job.stage,
            "elapsed_s": round(job.elapsed_s(), 1),
            "cancel_requested": job.cancel_requested,
            "result": job.result,
            "error": job.error,
        }

    def cancel(self, job_id: str) -> dict[str, Any]:
        job = self.get(job_id)
        if job is None:
            return {"ok": False, "error": f"unknown job: {job_id}"}
        if job.finished_at is not None:
            return {"ok": True, "job_id": job_id, "already_finished": True,
                    "status": job.status}
        job.cancel_event.set()
        job.stage = "cancel-requested"
        return {"ok": True, "job_id": job_id, "cancel_requested": True}

    def list_active(self) -> list[dict[str, Any]]:
        with self._lock:
            jobs = list(self._jobs.values())
        return [
            {
                "job_id": job.job_id,
                "name": job.name,
                "status": job.status,
                "stage": job.stage,
                "elapsed_s": round(job.elapsed_s(), 1),
            }
            for job in jobs
            if job.finished_at is None
        ]
