"""Select the active perception backend for read-only Agent tools."""

from pathlib import Path

from runtime.context import RuntimeContext


def yolo_shared_dir(ctx: RuntimeContext) -> Path | None:
    if ctx.config.mode != "sim" or not ctx.config.yolo_pick.get("shared_dir"):
        return None
    nodes = ctx.probes.snapshot().get("nodes", [])
    if any("yolo_bridge_node" in node for node in nodes):
        return Path(ctx.config.yolo_pick["shared_dir"]).expanduser()
    return None
