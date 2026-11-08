#!/usr/bin/env python3
"""ROSA agent entry point for AG_robot (ROS2 Humble)."""
from __future__ import annotations

import argparse
import json
import os
import re
import sys

from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from rosa import ROSA, RobotSystemPrompts

from runtime import (
    ApprovalGate,
    AuditLog,
    JobManager,
    ProcessRegistry,
    Probes,
    RuntimeContext,
    load_config,
)
from tools import (
    build_grasp_tools,
    build_module_tools,
    build_read_tools,
    build_robot_tools,
    build_yolo_tools,
    build_tabletop_tools,
    shutdown_robot_tools,
)

load_dotenv(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env"))

# ROSA always injects its own ROS2 toolset, including generic write tools.
# Those bypass the profile model: `ros2_service_call` alone could invoke
# /grasp_execute or controller_manager. Remove them from the tool list that the
# model sees. Dedicated, validated tools are the only write path.
BANNED_BUILTIN_TOOLS = {"ros2_service_call", "ros2_param_set"}


class _FilteredTools:
    def __init__(self, inner, banned: set[str]) -> None:
        self._inner = inner
        self._banned = banned

    def get_tools(self) -> list:
        return [
            tool
            for tool in self._inner.get_tools()
            if getattr(tool, "name", "") not in self._banned
        ]


class SafeROSA(ROSA):
    def _get_tools(self, ros_version, packages=None, tools=None, blacklist=None):
        inner = super()._get_tools(
            ros_version, packages=packages, tools=tools, blacklist=blacklist
        )
        return _FilteredTools(inner, BANNED_BUILTIN_TOOLS)


def get_llm():
    provider = os.getenv("LLM_PROVIDER", "deepseek").lower()
    if provider == "deepseek":
        return ChatOpenAI(
            model=os.getenv("DEEPSEEK_MODEL", "deepseek-flash"),
            api_key=os.getenv("DEEPSEEK_API_KEY"),
            base_url="https://api.deepseek.com",
            temperature=0,
            timeout=60,
            max_retries=2,
            extra_body={"thinking": {"type": "disabled"}},
        )
    if provider == "openai":
        return ChatOpenAI(model=os.getenv("OPENAI_MODEL", "gpt-4o-mini"), temperature=0)
    if provider == "ollama":
        from langchain_ollama import ChatOllama

        return ChatOllama(
            model=os.getenv("OLLAMA_MODEL", "qwen2.5:7b-instruct"),
            temperature=0,
            num_ctx=8192,
        )
    raise ValueError(f"unknown LLM_PROVIDER: {provider}")


def build_context(args: argparse.Namespace) -> RuntimeContext:
    config = load_config(args.mode, args.profile, args.config_dir)
    audit = AuditLog(
        config.paths.logs_dir / "audit.jsonl",
        mode=args.mode,
        profile=args.profile,
        max_field_chars=int(config.limits.get("audit_max_field_chars", 2000)),
    )
    approval = ApprovalGate(
        auto_confirm=(args.mode == "sim" and args.auto_confirm_sim),
        timeout_s=float(config.limits.get("approval_timeout_s", 60)),
    )
    registry = ProcessRegistry(config.paths.logs_dir, workspace_env=config.workspace_env)
    return RuntimeContext(
        config=config,
        audit=audit,
        approval=approval,
        registry=registry,
        jobs=JobManager(),
        probes=Probes(config),
    )


def build_tools(ctx: RuntimeContext, profile: str) -> list:
    available = {}
    built = (
        build_read_tools(ctx)
        + build_robot_tools(ctx)
        + build_module_tools(ctx)
        + build_grasp_tools(ctx)
        + build_yolo_tools(ctx)
        + build_tabletop_tools(ctx)
    )
    for item in built:
        available[item.name] = item

    wanted = ctx.config.tools_for_profile(profile)
    missing = [name for name in wanted if name not in available]
    if missing:
        raise SystemExit(
            f"档位 '{profile}' 需要以下工具，但当前版本（P1–P3）尚未实现：\n"
            f"  {', '.join(missing)}\n"
            "请使用 --profile read 或 --profile operate；"
            "抓取工具将在 P4/P5 加入。"
        )
    if ctx.config.mode == "real":
        sim_tools = set(ctx.config.groups.get("yolo", []) + ctx.config.groups.get("tabletop", []))
        wanted = [name for name in wanted if name not in sim_tools]
        if ctx.config.modules["grasp_executor"].get("real_hold_place") is True:
            wanted = [name for name in wanted if name not in {"grasp_precheck", "grasp_execute"}]
    return [available[name] for name in wanted]


def build_prompts(ctx: RuntimeContext) -> RobotSystemPrompts:
    mode = ctx.config.mode
    profile = ctx.config.profile
    mode_text = "仿真（Gazebo）" if mode == "sim" else "真机"
    approval_note = (
        "当前仿真启用了 auto-confirm-sim；用户要求正式执行且彩排成功后，"
        "直接调用 tabletop_pick(dry_run=false)，不要再在对话中索要 yes。"
        if mode == "sim" and ctx.approval.auto_confirm else
        "执行抓取前必须先确认工作空间内无人、急停按钮可达。"
    )
    if profile == "read":
        capabilities = (
            "当前只提供查询类工具：系统状态、系统预检、YOLO 检测结果、抓取候选、"
            "抓取结果、机器人关节状态、模块日志、审计记录。"
            "你没有启动/停止模块、移动机械臂或执行抓取的权限。"
        )
    elif profile == "operate":
        capabilities = (
            "你可以查询系统状态与预检，也可以启动/停止 YOLO 感知、识别窗口、"
            "GraspNet、MoveIt 与 RViz。"
            "你不能移动机械臂，也不能执行抓取。"
            "启动感知或规划模块需要 10-90 秒，工具会等到就绪后才返回。"
        )
    elif mode == "real" and ctx.config.modules["grasp_executor"].get("real_hold_place") is True:
        capabilities = (
            "真机流程先启动感知、GraspNet 和 MoveIt，确认识别到用户指定的物体。"
            "用 real_hold_prepare 仅规划抓取、抬升、原位放回和回零；用 grasp_job_status 查询预演。"
            "用户明确要求抓取并保持且预演完成后，调用 grasp_and_hold 并等待终端人工确认。"
            "用户指定物体时，把已确认的 YOLO 类别传入 grasp_and_hold 的 target。"
            "根据任务结果中的 state=held 和夹爪开度报告最近一次持物判定。"
            "用户明确要求放回后才调用 place_held_object，由操作者独立确认。"
            "held_object_status 查询状态。grasp_cancel 请求停止当前任务并保留夹爪状态，"
            "estop 用于紧急情况。needs_recovery 时交由现场操作者处理。"
            "放置结果报告松爪与回零，物体实际落稳须现场观察。"
        )
    else:
        capabilities = (
            "你可以查询、启停模块，还可以彩排抓取和执行真实抓取。"
            "后端 A 彩排用 grasp_precheck（机械臂不动）；执行用 grasp_execute（会移动机械臂），"
            "必须先有有效的彩排令牌，并且会要求人工输入 yes 确认。"
            "急停用 estop，恢复用 recover。"
            "如果当前场景是 piper_with_yolo_graspnet（Gazebo Harmonic），"
            "perception_detections 读取该场景的实时 YOLO 预览；"
            "空闲时预览持续更新，过期帧不能当作当前检测。"
            "对 cylinder 先调用 yolo_pick(dry_run=true)，确认 result.success 和 token_issued，"
            "再调用 yolo_pick(dry_run=false) 并查询 yolo_pick_status。"
            "后端 B 的 dry_run=true 仍会移动机械臂到观察和规划姿态，只是不夹取和放置物体；"
            "向用户解释时必须准确说明这一点。"
            "yolo_pick 会自行启动 Gazebo、YOLO 和 GraspNet，首次调用前允许 ROS 图为空；"
            "precheck_all 是另一个仿真后端的检查，不能阻止 yolo_pick 启动。"
            "它是一次性执行器，没有 /grasp_execute 服务。"
            f"{approval_note}"
            "桌面木块场景 piper_tabletop_sim 使用 tabletop_pick：先 dry_run=true 彩排，"
            "再 dry_run=false 执行抓取、抬升保持、原位放回和回零；它自动启动 MoveIt、YOLO、GraspNet，"
            "只接受本次实时候选。正式结果须查询 tabletop_pick_status，检查 result.success、"
            "result.pick_success、result.placed 与 Gazebo 木块位姿。"
        )
    sim_instructions = (
        "yolo_pick 彩排会自动启动它需要的后端 B 仿真和感知；"
        "该场景尚未启动时直接调用 yolo_pick(dry_run=true)，不要等待后端 A 预检通过。"
        "回答 yolo_pick 任务结果前必须重新调用 yolo_pick_status，使用最新结果。"
        "仿真桌面木块任务使用 tabletop_pick 与 tabletop_pick_status。"
        "停止桌面仿真模块时调用 tabletop_stop。"
        if mode == "sim" else ""
    )
    return RobotSystemPrompts(
        embodiment_and_persona=(
            f"你是 AG_robot 的机械臂操作助手，控制一台 AgileX Piper 六轴机械臂。"
            f"当前运行模式：{mode_text}；权限档位：{profile}。"
        ),
        about_your_environment=(
            "ROS2 Humble。长度单位毫米，角度单位度。"
            "关节状态使用弧度。仿真与真机使用不同的启动方式，不要混用。"
        ),
        about_your_capabilities=capabilities,
        critical_instructions=(
            "任何数值必须来自工具返回，禁止编造话题名、关节角、坐标、置信度或错误码。"
            "查不到就明确说查不到。"
            "当用户要求当前档位不具备的能力时，明确说明缺少哪些权限，"
            "并建议用更高级档位重启 agent；不要假装已经执行。"
            "启动普通模块前先用 precheck_all 或 session_status 确认当前状态，"
            "已在运行就不要重复启动。"
            f"{sim_instructions}"
            "后台任务返回 job_id 后先告知用户，后续查询状态时读取工具结果。"
            "启动失败时把日志末尾的关键错误告诉用户。"
            "收到急停请求时立即调用 estop。"
            "执行真实抓取前必须确认工作空间内无人、急停可达；"
            "没有对应流程的有效预演时不得尝试执行。"
            "回答保持简洁，先结论后依据。"
        ),
        constraints_and_guardrails=(
            "不确定是否安全时拒绝执行并说明缺少什么信息。"
            "不要假设仿真和真机环境相同。"
        ),
    )


def print_json(data: object) -> None:
    print(json.dumps(data, ensure_ascii=False, indent=2, default=str), flush=True)


def direct_yolo_status(ctx: RuntimeContext, query: str) -> bool:
    """Read job results directly when the user asks for their status."""
    match = re.search(r"\bjob_\d+\b", query)
    lowered = query.lower()
    if not match and not any(word in lowered for word in (
        "任务", "抓取", "彩排", "放置", "cylinder", "yolo_pick"
    )):
        return False
    explicit_status = any(word in lowered for word in (
        "状态", "结果", "查询", "查一下", "进度", "进展", "status", "result"
    ))
    outcome_question = (
        ("是否" in query or "了吗" in query or "怎么样" in query or "？" in query or "?" in query)
        and any(word in query for word in ("成功", "完成", "放置", "抓取"))
    )
    if not (explicit_status or outcome_question or query.strip() == (match.group() if match else "")):
        return False
    if any(word in query for word in ("取消", "急停")):
        return False
    job = ctx.jobs.get(match.group()) if match else ctx.jobs.latest("yolo_pick")
    if job is None or job.name != "yolo_pick":
        return False
    status_tool = build_yolo_tools(ctx)[1]
    state = status_tool.invoke({"job_id": job.job_id})
    result = state.get("result") or {}
    pick_state = state.get("pick_state") or {}
    print_json({
        "job_id": job.job_id,
        "status": state.get("status"),
        "success": state.get("success"),
        "completed": result.get("completed"),
        "token_issued": result.get("token_issued"),
        "phase": pick_state.get("phase"),
        "placed": pick_state.get("placed"),
        "pending": pick_state.get("pending"),
        "error": state.get("error"),
        "log": result.get("log"),
    })
    return True


def direct_tabletop_status(ctx: RuntimeContext, query: str) -> bool:
    """Bypass ReAct polling for explicit tabletop job status questions."""
    match = re.search(r"\bjob_\d+\b", query)
    explicit = any(word in query.lower() for word in ("状态", "结果", "进度", "进展", "status", "result"))
    named = any(word in query.lower() for word in ("桌面", "木块", "tabletop"))
    job = ctx.jobs.get(match.group()) if match else ctx.jobs.latest("tabletop_pick")
    if job is None or job.name != "tabletop_pick":
        return False
    if not (named and explicit or match and explicit or query.strip() == job.job_id):
        return False
    state = build_tabletop_tools(ctx)[1].invoke({"job_id": job.job_id})
    print_json(state)
    return True


def repl(ctx: RuntimeContext, agent: ROSA) -> None:
    print("ROSA ready.")
    print(f"  模式: {ctx.config.mode}   档位: {ctx.config.profile}")
    print(f"  共享目录: {ctx.config.shared_dir}")
    print(f"  日志目录: {ctx.config.paths.logs_dir}")
    print("命令: exit / clear / status / audit [n] / modules / help")

    while True:
        try:
            query = input("\nyou> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not query:
            continue
        if query in {"exit", "quit"}:
            break
        if query == "clear":
            agent.clear_chat()
            print("[对话已清空]")
            continue
        if query == "status":
            print_json(ctx.status())
            continue
        if query == "modules":
            print_json(ctx.registry.list())
            continue
        if query.startswith("audit"):
            parts = query.split()
            count = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 10
            print_json(ctx.audit.tail(count))
            continue
        if query == "help":
            print("exit/quit 退出，clear 清空对话，status 系统状态，")
            print("audit [n] 最近 n 条审计，modules 已启动模块。")
            continue
        if direct_tabletop_status(ctx, query) or direct_yolo_status(ctx, query):
            continue
        print("rosa>", agent.invoke(query), flush=True)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="AG_robot ROSA agent")
    parser.add_argument(
        "--mode", choices=["sim", "real"], default="sim", help="运行模式（启动时锁定）"
    )
    parser.add_argument(
        "--profile",
        choices=["read", "operate", "grasp"],
        default="read",
        help="工具权限档位（默认 read）",
    )
    parser.add_argument("--config-dir", default=None, help="配置目录")
    parser.add_argument(
        "--auto-confirm-sim",
        action="store_true",
        help="仿真模式下跳过人工确认（真机模式无效）",
    )
    parser.add_argument(
        "--keep-modules",
        action="store_true",
        help="退出时不停止由本次 agent 启动的模块",
    )
    parser.add_argument(
        "--verbose", action="store_true", help="打印 ROSA 的中间工具调用"
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    ctx = build_context(args)
    try:
        tools = build_tools(ctx, args.profile)
    except SystemExit as exc:
        print(exc, file=sys.stderr)
        return 2

    agent = SafeROSA(
        ros_version=2,
        llm=get_llm(),
        tools=tools,
        prompts=build_prompts(ctx),
        blacklist=["/rosout", "parameter_events"],
        verbose=args.verbose or os.getenv("ROSA_VERBOSE", "0") == "1",
        max_iterations=15,
        streaming=True,
    )

    ctx.audit.record(
        "session_start",
        args={
            "mode": args.mode,
            "profile": args.profile,
            "tools": [t.name for t in tools],
            "banned_builtin_tools": sorted(BANNED_BUILTIN_TOOLS),
        },
    )
    try:
        repl(ctx, agent)
    finally:
        if not args.keep_modules:
            real = ctx.config.modules["grasp_executor"]
            preserve_motion_stack = (
                args.mode == "real" and real.get("real_hold_place") is True
                and real.get("real_hold_place_motion") is True
                and ctx.registry.is_running("grasp_executor_real")
            )
            if preserve_motion_stack:
                print("保留真机执行器和 MoveIt，现场确认持物状态后再停机。", flush=True)
            stopped = ctx.registry.stop_all(
                exclude={"grasp_executor_real", "moveit"} if preserve_motion_stack else ())
            if stopped:
                print_json({"stopped_modules": stopped})
        ctx.audit.record("session_end")
        shutdown_robot_tools()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
