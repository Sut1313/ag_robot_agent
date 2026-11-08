#!/usr/bin/env python3
"""Show the YOLO overlay produced by the perception bridge.

The bridge writes yolo_overlay.png atomically and .ready last, so polling the
file mtime is safe: a reader never sees a half-written image.

Must run with the system Python (not the agent venv): OpenCV and numpy in the
venv are ABI-incompatible with the system ROS build.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--shared-dir", required=True)
    parser.add_argument("--window", default="YOLO - Perception")
    parser.add_argument("--overlay-name", default="yolo_overlay.png")
    parser.add_argument("--manifest-name", default="frame_manifest.json")
    parser.add_argument(
        "--timeout", type=float, default=0.0, help="0 means run until q/ESC"
    )
    args = parser.parse_args()

    shared = Path(args.shared_dir).expanduser()
    overlay = shared / args.overlay_name
    manifest = shared / args.manifest_name

    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
    last_mtime: float | None = None
    start = time.time()

    try:
        while True:
            if args.timeout and time.time() - start > args.timeout:
                break
            if overlay.exists():
                mtime = overlay.stat().st_mtime
                if mtime != last_mtime:
                    image = cv2.imread(str(overlay))
                    if image is not None:
                        sequence = None
                        try:
                            sequence = json.loads(
                                manifest.read_text(encoding="utf-8")
                            ).get("frame_sequence")
                        except (OSError, json.JSONDecodeError):
                            pass
                        if sequence is not None:
                            cv2.putText(
                                image,
                                f"frame {sequence}",
                                (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.8,
                                (0, 255, 0),
                                2,
                            )
                        cv2.imshow(args.window, image)
                        last_mtime = mtime
            key = cv2.waitKey(100) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
