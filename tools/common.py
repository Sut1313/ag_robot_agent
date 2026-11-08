from __future__ import annotations

import inspect
import time
from functools import wraps
from typing import Any, Callable


def audited(ctx: Any, name: str | None = None) -> Callable:
    """Record every tool invocation, including exceptions.

    Applied *inside* LangChain's @tool decorator so the schema still comes from
    the original function signature and docstring.
    """

    def decorator(fn: Callable) -> Callable:
        tool_name = name or fn.__name__
        signature = inspect.signature(fn)

        @wraps(fn)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            start = time.perf_counter()
            try:
                bound = signature.bind_partial(*args, **kwargs)
                call_args = dict(bound.arguments)
            except TypeError:
                call_args = {"args": args, "kwargs": kwargs}
            try:
                result = fn(*args, **kwargs)
            except Exception as exc:  # noqa: BLE001 - re-raised after audit
                ctx.audit.record(
                    tool_name,
                    args=call_args,
                    error=f"{type(exc).__name__}: {exc}",
                    duration_ms=(time.perf_counter() - start) * 1000.0,
                )
                raise
            ctx.audit.record(
                tool_name,
                args=call_args,
                result=result,
                duration_ms=(time.perf_counter() - start) * 1000.0,
            )
            return result

        return wrapper

    return decorator
