from __future__ import annotations

import json
import select
import sys
import textwrap
from typing import Any, Callable


class ApprovalGate:
    """Human-in-the-loop gate for motion and other high-risk actions.

    The gate fails closed: timeout, EOF, non-interactive stdin and anything
    other than the literal word ``yes`` all deny the action.
    """

    def __init__(
        self,
        auto_confirm: bool = False,
        timeout_s: float = 60.0,
        reader: Callable[[float], str | None] | None = None,
        stream: Any = None,
    ) -> None:
        self.auto_confirm = bool(auto_confirm)
        self.timeout_s = float(timeout_s)
        self._reader = reader or self._read_stdin
        self._stream = stream or sys.stdout

    def _say(self, text: str) -> None:
        print(text, file=self._stream, flush=True)

    @staticmethod
    def _read_stdin(timeout: float) -> str | None:
        try:
            ready, _, _ = select.select([sys.stdin], [], [], timeout)
        except (OSError, ValueError):
            return None
        if not ready:
            return None
        line = sys.stdin.readline()
        if line == "":
            return None
        return line.strip()

    def require(
        self,
        action: str,
        details: dict[str, Any] | None = None,
        force_confirm: bool = False,
    ) -> tuple[bool, str]:
        """Return (approved, reason). Never raises."""
        if self.auto_confirm and not force_confirm:
            return True, "auto-confirmed (simulation mode)"

        self._say("")
        self._say("=" * 62)
        self._say(f"需要确认: {action}")
        if details:
            rendered = json.dumps(details, ensure_ascii=False, indent=2, default=str)
            self._say(textwrap.indent(rendered, "  "))
        self._say("-" * 62)
        self._say(
            f"输入 yes 确认，其他任何输入或 {self.timeout_s:.0f} 秒超时都会取消。"
        )

        answer = self._reader(self.timeout_s)
        if answer is None:
            return False, "未收到确认（超时或输入结束），已取消"
        if answer.strip().lower() != "yes":
            return False, f"输入不是 yes（收到 {answer!r}），已取消"
        return True, "已确认"
