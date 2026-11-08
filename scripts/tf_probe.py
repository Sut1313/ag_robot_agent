#!/usr/bin/env python3
"""One-shot TF lookup in a separate process.

Usage: tf_probe.py <parent> <child> <use_sim_time:0|1>
Prints one JSON line: {"ok": true, "translation": [x,y,z]} or {"ok": false, "error": ...}
"""
from __future__ import annotations

import json
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def main() -> int:
    parent, child = sys.argv[1], sys.argv[2]
    use_sim = len(sys.argv) > 3 and sys.argv[3] == "1"

    rclpy.init()
    node = Node("rosa_tf_probe")
    if use_sim:
        node.set_parameters(
            [
                rclpy.parameter.Parameter(
                    "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True
                )
            ]
        )
    buffer = Buffer()
    TransformListener(buffer, node)
    deadline = time.time() + 6.0
    last_error = None
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                transform = buffer.lookup_transform(
                    parent, child, rclpy.time.Time()
                )
                t = transform.transform.translation
                print(
                    json.dumps(
                        {"ok": True, "translation": [t.x, t.y, t.z]},
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
                return 0
            except Exception as exc:  # noqa: BLE001
                last_error = str(exc)
        print(
            json.dumps({"ok": False, "error": last_error or "unavailable"}),
            flush=True,
        )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
