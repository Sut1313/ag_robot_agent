# AG Robot ROSA

面向 AgileX PiPER 的 ROS 2 Humble 自然语言操作终端。ROSA 通过专用工具启动感知、读取 YOLO 和 GraspNet 结果、预演路径并查询任务。真机抓取保持与原位放回使用两条独立命令和两次终端确认。

## 环境

- Ubuntu 22.04、Python 3.10、ROS 2 Humble 和 MoveIt 2。打开感知画面时，系统 Python 还需要 OpenCV。
- 真机和桌面木块仿真均依赖 [real_piper_yolo_graspnet](https://github.com/Sut1313/real_piper_yolo_graspnet) 的 ROS 包和 GraspNet Docker 镜像。感知模型、推理权重、相机与手眼标定由对应工作区提供。
- 木块仿真依赖 [piper_tabletop_sim](https://github.com/Sut1313/piper_tabletop_sim) 的 Gazebo Harmonic 场景、仿真手眼参数和木块模型。另一套圆柱体仿真使用可选的 [piper_with_yolo_graspnet](https://github.com/Sut1313/piper_with_yolo_graspnet)。

## 安装

在本仓库根目录创建 Python 环境。

```bash
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements.txt
```

### 接入自己的大模型 API

仓库提供 [空白模板](env.example)。克隆后，每位操作者用自己在模型服务商处创建的 API Key 配置本机。ROSA 启动时从仓库根目录的 `.env` 读取配置。

```bash
if [ ! -e .env ]; then
  (umask 077; cp env.example .env)
fi
chmod 600 .env
nano .env
```

**DeepSeek** 使用以下配置。将密钥填在 `DEEPSEEK_API_KEY=` 后面，模型名填服务商提供的可用模型 ID。

```dotenv
LLM_PROVIDER=deepseek
DEEPSEEK_API_KEY=
DEEPSEEK_MODEL=deepseek-flash
```

**OpenAI** 将 `LLM_PROVIDER` 改为 `openai`，填写自己的 `OPENAI_API_KEY` 和模型 ID。

```dotenv
LLM_PROVIDER=openai
OPENAI_API_KEY=
OPENAI_MODEL=gpt-4o-mini
```

使用自建服务或其他 OpenAI 兼容网关时，沿用第二种配置，并在本机 `.env` 中设置 `OPENAI_BASE_URL` 为该服务给出的 API 基址。密钥和模型 ID 使用该服务自己的值，模型需支持工具调用。Shell 已导出的同名环境变量优先于 `.env`。修改后重启 ROSA。

首次接入可用只读档位验证。出现 `you>` 后输入“请回复模型连接成功”，能收到模型回复即表示请求已到达所配置的服务。

```bash
source /opt/ros/humble/setup.bash
.venv/bin/python agent.py --mode sim --profile read
```

发布时提交 `env.example`，保持密钥字段为空。实际密钥留在本机的 `.env`，旧凭据副本 `env.txt` 也由 Git 忽略。上传使用 Git 的暂存区，避免把整个本机目录打包上传。提交前可以检查暂存文件列表和凭据文件的忽略状态。

```bash
git diff --cached --name-only
git check-ignore .env env.txt
```

默认从同级目录寻找以下工作区。工作区名称或位置不同时，可在启动前设置对应的环境变量。

| 环境变量 | 默认位置 |
| --- | --- |
| `PIPER_REAL_WS` | 同级的 `real_piper_yolo_graspnet` |
| `PIPER_TABLETOP_WS` | 同级的 `piper_tabletop_sim_ws` |
| `PIPER_YOLO_SIM_PROJECT` | 同级的 `piper_with_yolo_graspnet` |
| `PIPER_GZ_CONTROL_WS` | 用户目录中的 `gz_ros2_control_ws`，适用于使用源码 overlay 的 Gazebo 安装 |

### 配套执行器补丁

ROSA 的真机保持与放回和桌面木块流程需要 [配套补丁](patches/real_piper_yolo_graspnet-c613640.patch)。补丁以 `real_piper_yolo_graspnet` 的 `c613640399aaa535b90f12f039d0d6eeec4b8629` 为基线，包含 SDK 与仿真夹爪执行接口、MoveIt 适配和共享目录支持。请在基线工作区检查并应用补丁。

```bash
git -C "${PIPER_REAL_WS:-../real_piper_yolo_graspnet}" apply --check "$PWD/patches/real_piper_yolo_graspnet-c613640.patch"
git -C "${PIPER_REAL_WS:-../real_piper_yolo_graspnet}" apply "$PWD/patches/real_piper_yolo_graspnet-c613640.patch"
```

按后端 README 构建 ROS 包及 GraspNet 镜像。已包含对应改动的工作区可直接构建。GitHub 上已发布的后端基线需要此补丁才能提供这里使用的真机保持放回接口。

## 用 ROSA 控制真机抓取和放回

流程为 **D435i 采集 → YOLO 识别 → GraspNet 生成抓取候选 → MoveIt 预演 → 抓取并抬升 → 保持等待 → 原位下降并松爪 → 退让和回零**。以下以积木 `block` 为例。原位放回使用本轮预演的抓取位置。

### 1. 接线并启动仅规划模式

接好 PiPER、USB-CAN 和 D435i。先确认机械臂在零位、夹爪空载、工作空间清空、现场急停可达。核对后端的 `camera_data/handeye_eye_to_hand.yaml` 来自当前相机和工位。首次联调保持 [模块配置](config/modules.yaml) 中的 `real_hold_place_motion: false`。

在 ROSA 仓库根目录激活 CAN。这里以 `can0` 为例。

```bash
(cd "${PIPER_REAL_WS:-../real_piper_yolo_graspnet}/src/piper_ros" && bash can_activate.sh can0 1000000)
ip -details link show can0
```

打开三个终端，均进入 ROSA 仓库根目录并加载相同环境。`68` 是示例真机 ROS 域，使用与仿真隔离的值。

```bash
source /opt/ros/humble/setup.bash
source "${PIPER_REAL_WS:-../real_piper_yolo_graspnet}/install/setup.bash"
export ROS_DOMAIN_ID=68
```

**终端 A · PiPER 驱动**

```bash
ros2 launch piper start_single_piper.launch.py \
  can_port:=can0 auto_enable:=false \
  gripper_exist:=true gripper_forward:=false
```

`gripper_forward:=false` 将夹爪控制留给执行器的 Piper SDK。另开一个加载了相同环境的终端，核对真机关节反馈持续更新，机械臂状态 `err_code` 为 `0`。

```bash
ros2 topic hz /joint_states_feedback
# 结束频率检查后执行下一条
ros2 topic echo /arm_status --once
```

**终端 B · MoveIt 和 RViz**

```bash
ros2 launch piper_with_gripper_moveit demo.launch.py \
  use_rviz:=true allow_trajectory_execution:=false
```

**终端 C · ROSA 对话**

```bash
.venv/bin/python agent.py --mode real --profile grasp
```

### 2. 通过自然语言启动感知并预演

在终端 C 出现 `you>` 后，依次输入下列指令。

| 对 ROSA 说 | 确认结果 |
| --- | --- |
| “启动积木感知和 GraspNet，告诉我当前识别到了什么。” | YOLO 检测到 `block`，GraspNet 产生了本轮的新鲜候选。ROSA 负责启动这两个感知模块。 |
| “检查真机抓取准备状态。” | 真机模式、相机、手眼 TF、MoveIt 和控制器检查通过。 |
| “为积木预演抓取、保持和原位放回。” | 收到 `job_id`。ROSA 启动执行器，并从实测零位规划抓取、抬升、返回、退让和回零。 |
| “查询任务 job_1 的结果。” | 换成实际任务编号。检查 `result.state=prepared`，并核对候选名、TCP 抓取坐标和抬升高度。 |

预演有效期为 **90 秒**。结合 RViz 中的机器人和碰撞场景，核对返回的抓取坐标及抬升高度与现场桌面相符。目标或机械臂位置变化后，等当前预演失效并重新预演。

### 3. 启用实动并完成一次抓放

按真机测试手册先核验夹爪全开和全闭反馈、手眼标定、抓取高度以及桌面和障碍模型，再进入有人值守的低速单次测试。[PiPER 真机测试手册](https://github.com/Sut1313/real_piper_yolo_graspnet/blob/main/docs/REAL_ROBOT_TEST_RUNBOOK.md)

1. 退出仅预演的 ROSA，结束旧执行器。在 [模块配置](config/modules.yaml) 中设为 `real_hold_place_motion: true`。
2. 重启终端 A 和 B。将 A 的 `auto_enable` 改为 `true`，将 B 的 `allow_trajectory_execution` 改为 `true`。启动驱动时保持夹爪空载。
3. 重启终端 C 的 ROSA。重新启动感知，完成一次新预演并查询到 `state=prepared`。
4. 对 ROSA 说 **“抓取积木并保持。”** 核对终端提示的候选和抓取坐标，输入 `yes`。查询此次抓取的任务编号，等待 `result.state=held`，目视确认物体已抬离桌面并稳定夹持。
5. 对 ROSA 说 **“放回物体。”** 再输入一次 `yes`。查询此次放置的任务编号，等待 `result.state=placed` 和 `result.released=true`，目视确认物体已落稳且机械臂回零。

抓取任务和放置任务各有自己的 `job_id`。任务结果中的 `held` 表示最近一次持物开度复核通过，`placed` 表示松爪反馈和回零检查通过。将现场目视结果一同记录后再判定本轮成功。

执行中可以说 **“取消任务 job_编号。”** 如出现意外运动，使用现场急停。收到 `needs_recovery` 时，请现场检查夹爪、物体和关节状态后再继续。执行器重启会清空本轮持物状态。退出 ROSA 前先确认物体已放回，实动模式下再由现场人员按手册关闭仍运行的执行器和 MoveIt。

目前这套真机流程已完成构建、离线测试与隔离 ROS 接口测试，实体持物和落稳待现场验收。[真机操作说明](docs/真机保持与原位放回.md)

## 用 ROSA 在仿真中跑完整循环

桌面木块后端会自动启动 Gazebo、MoveIt、YOLO 和 GraspNet。先按环境章节准备仿真工作区、模型和配套执行器，从本仓库根目录运行。

```bash
source /opt/ros/humble/setup.bash
.venv/bin/python agent.py --mode sim --profile grasp
```

1. 说 **“请在桌面木块仿真中彩排抓取。”** 查询任务状态，确认 `result.success=true` 且 `token_issued=true`。
2. 说 **“请正式抓取桌面木块，保持后放回桌面并回零。”** 在终端确认，等待完整循环结束。
3. 查询正式任务状态，检查 `result.pick_success=true`、`result.placed=true` 和 `result.success=true`，并查看抬升、保持采样及落回桌面的位姿。
4. 说 **“请停止桌面木块仿真及其感知模块。”**

需要看画面时，可在 ROS 域 `67` 和 Gazebo 分区 `tabletop67` 打开 Gazebo GUI。圆柱体仿真使用独立后端，它的彩排会将仿真机械臂移到观察和规划姿态。[桌面木块仿真](docs/桌面木块Agent抓取.md) · [圆柱体仿真闭环](docs/仿真抓取闭环验证.md)

## 离线自检

```bash
source /opt/ros/humble/setup.bash
.venv/bin/python tests/test_runtime.py
```

运行时数据与日志分别写入 `runtime_data/` 和 `logs/`。凭据、日志、捕获数据及本机运行环境通过 Git 忽略规则保留在本机。
