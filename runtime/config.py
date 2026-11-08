from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from string import Template
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_DIR = ROOT / "config"


def _expand(value: str) -> Path:
    return Path(os.path.expanduser(str(value)))


def _load_yaml(path: Path) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a mapping")
    return data


def _resolve_paths(data: dict, variables: dict[str, str]) -> dict:
    resolved = {}
    for key, value in data.items():
        if isinstance(value, dict):
            value = _resolve_paths(value, variables)
        elif isinstance(value, str):
            value = os.path.expanduser(Template(value).safe_substitute(variables))
        resolved[key] = value
    return resolved


@dataclass(frozen=True)
class Paths:
    shared_dir: Path
    sim_shared_dir: Path
    real_ws: Path
    sim_ws: Path
    logs_dir: Path


@dataclass(frozen=True)
class Config:
    mode: str
    profile: str
    config_dir: Path
    paths: Paths
    limits: dict[str, Any]
    precheck: dict[str, Any]
    modules: dict[str, Any]
    yolo_pick: dict[str, Any]
    tabletop_pick: dict[str, Any]
    groups: dict[str, list[str]]
    profiles: dict[str, list[str]]

    @property
    def shared_dir(self) -> Path:
        return self.paths.sim_shared_dir if self.mode == "sim" else self.paths.shared_dir

    @property
    def workspace_env(self) -> dict[str, str]:
        env = {"PIPER_REAL_WS": str(self.paths.real_ws),
               "PIPER_TABLETOP_WS": str(self.paths.sim_ws)}
        if self.yolo_pick.get("project_ws"):
            env["PIPER_YOLO_SIM_PROJECT"] = self.yolo_pick["project_ws"]
        return env

    def tools_for_profile(self, profile: str | None = None) -> list[str]:
        name = profile or self.profile
        if name not in self.profiles:
            raise ValueError(f"unknown profile: {name}")
        ordered: list[str] = []
        for group in self.profiles[name]:
            for tool in self.groups.get(group, []):
                if tool not in ordered:
                    ordered.append(tool)
        return ordered


def load_config(
    mode: str,
    profile: str,
    config_dir: str | Path | None = None,
) -> Config:
    directory = Path(config_dir) if config_dir else DEFAULT_CONFIG_DIR
    variables = {
        **os.environ,
        "AGENT_ROOT": str(ROOT),
        "PIPER_REAL_WS": os.getenv("PIPER_REAL_WS", str(ROOT.parent / "real_piper_yolo_graspnet")),
        "PIPER_TABLETOP_WS": os.getenv("PIPER_TABLETOP_WS", str(ROOT.parent / "piper_tabletop_sim_ws")),
        "PIPER_YOLO_SIM_PROJECT": os.getenv(
            "PIPER_YOLO_SIM_PROJECT", str(ROOT.parent / "piper_with_yolo_graspnet")),
    }
    raw = _load_yaml(directory / "paths.yaml")["paths"]
    resolved = _resolve_paths(raw, variables)
    variables.update(PIPER_REAL_WS=str(_expand(resolved["real_ws"]).resolve()),
                     PIPER_TABLETOP_WS=str(_expand(resolved["sim_ws"]).resolve()))
    paths_raw = _resolve_paths(raw, variables)
    limits = _load_yaml(directory / "limits.yaml")
    precheck = _load_yaml(directory / "precheck.yaml")
    modules = _resolve_paths(_load_yaml(directory / "modules.yaml"), variables)
    yolo_file = directory / "yolo_pick.yaml"
    yolo_pick = _load_yaml(yolo_file).get("yolo_pick", {}) if yolo_file.exists() else {}
    tabletop_file = directory / "tabletop_pick.yaml"
    tabletop_pick = _load_yaml(tabletop_file).get("tabletop_pick", {}) if tabletop_file.exists() else {}
    yolo_pick = _resolve_paths(yolo_pick, variables)
    tabletop_pick = _resolve_paths(tabletop_pick, variables)
    profile_data = _load_yaml(directory / "profiles.yaml")

    paths = Paths(
        shared_dir=_expand(paths_raw["shared_dir"]),
        sim_shared_dir=_expand(paths_raw["sim_shared_dir"]),
        real_ws=_expand(paths_raw["real_ws"]),
        sim_ws=_expand(paths_raw["sim_ws"]),
        logs_dir=_expand(paths_raw["logs_dir"]),
    )
    paths.logs_dir.mkdir(parents=True, exist_ok=True)
    paths.sim_shared_dir.mkdir(parents=True, exist_ok=True)

    config = Config(
        mode=mode,
        profile=profile,
        config_dir=directory,
        paths=paths,
        limits=limits,
        precheck=precheck,
        modules=modules,
        yolo_pick=yolo_pick,
        tabletop_pick=tabletop_pick,
        groups=profile_data.get("groups", {}),
        profiles=profile_data.get("profiles", {}),
    )
    if profile not in config.profiles:
        raise ValueError(f"unknown profile: {profile}")
    return config
