"""Grasp executor (dry) for the AG_robot agent.

默认 scan_only:=true —— 只做 CSV 解析 + 坐标变换 + 三段预检 dry-run（臂不动），
返回可达性统计。
⚠ 切换 scan_only:=false 会触发真机抓取执行（GraspExecutor.execute_pick），
必须现场二次确认后方可开启。

参数与节点声明一一对应（brief 参数清单）；handeye_yaml 默认指向
camera_data/ 真实标定文件（用户指令）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 包内 ssik 脚本绝对路径（launch 文件位于 <pkg_root>/launch/）。
# M6（fixed 2026-08-07）：abspath 不解析 symlink（symlink-install 下 __file__
# 仍是 install 路径 → scripts 解析不到）；改用 realpath 解析到源码目录。
# 同时 setup.py data_files 已把 scripts/ 打包进 install（常规安装同样可用）。
_DEFAULT_SSIK_SCRIPT = os.path.join(
    get_package_share_directory('real_piper_perception'),
    'scripts', 'piper_ssik_ik.py')


def generate_launch_description():
    scan_only = LaunchConfiguration('scan_only')
    dry_run_max_stage = LaunchConfiguration('dry_run_max_stage')
    velocity_scale = LaunchConfiguration('velocity_scale')
    handeye_yaml = LaunchConfiguration('handeye_yaml')
    shared_dir = LaunchConfiguration('shared_dir')
    csv_path = LaunchConfiguration('csv_path')
    approach_clearance = LaunchConfiguration('approach_clearance')
    lift_z = LaunchConfiguration('lift_z')
    open_gripper_lift_height = LaunchConfiguration('open_gripper_lift_height')
    grasp_z_offset = LaunchConfiguration('grasp_z_offset')
    gripper_margin = LaunchConfiguration('gripper_margin')
    gripper_detect_threshold = LaunchConfiguration('gripper_detect_threshold')
    gripper_backend = LaunchConfiguration('gripper_backend')
    graspnet_6d_max_candidates = LaunchConfiguration('graspnet_6d_max_candidates')
    candidate_frames = LaunchConfiguration('candidate_frames')
    grasp_score_min = LaunchConfiguration('grasp_score_min')
    min_approach_down = LaunchConfiguration('min_approach_down')
    enable_self_consistency = LaunchConfiguration('enable_self_consistency')
    self_consistency_max_center_dist = LaunchConfiguration(
        'self_consistency_max_center_dist')
    max_orientation_variants = LaunchConfiguration('max_orientation_variants')
    enable_piper_analytic_ik_planning = LaunchConfiguration(
        'enable_piper_analytic_ik_planning')
    graspnet_6d_analytic_only_precheck = LaunchConfiguration(
        'graspnet_6d_analytic_only_precheck')
    piper_analytic_ik_script = LaunchConfiguration('piper_analytic_ik_script')
    piper_analytic_ik_max_solutions = LaunchConfiguration(
        'piper_analytic_ik_max_solutions')
    piper_analytic_ik_max_solution_plans = LaunchConfiguration(
        'piper_analytic_ik_max_solution_plans')
    planning_time_precheck = LaunchConfiguration('planning_time_precheck')
    planning_time_exec = LaunchConfiguration('planning_time_exec')
    enable_sim_lift = LaunchConfiguration('enable_sim_lift')
    skip_final_home = LaunchConfiguration('skip_final_home')

    return LaunchDescription([
        # ⚠ scan_only 默认 true（安全默认：只预检不执行）。切 false 需现场二次确认！
        DeclareLaunchArgument(
            'scan_only', default_value='true',
            description='true=只预检 dry-run（臂不动）；false=真机抓取执行（需现场二次确认）'),
        DeclareLaunchArgument(
            'dry_run_max_stage', default_value='',
            description='Task 10: only execute up to this stage '
                        '(open/pre_pose/approach/grasp/gripper/lift)'),
        DeclareLaunchArgument('velocity_scale', default_value='0.02'),
        DeclareLaunchArgument(
            'handeye_yaml',
            default_value='camera_data/handeye_eye_to_hand.yaml',
            description='eye-to-hand 标定 yaml（camera_data/ 真实文件）'),
        DeclareLaunchArgument(
            'shared_dir',
            default_value='shared_camera_data',
            description='感知/结果共享目录'),
        DeclareLaunchArgument(
            'csv_path', default_value='',
            description='感知候选 CSV；空则用 <shared_dir>/perception_grasps.csv'),
        DeclareLaunchArgument('approach_clearance', default_value='0.05'),
        DeclareLaunchArgument('lift_z', default_value='0.10'),
        DeclareLaunchArgument('open_gripper_lift_height', default_value='0.10'),
        DeclareLaunchArgument('grasp_z_offset', default_value='0.02'),
        DeclareLaunchArgument('gripper_margin', default_value='0.005'),
        DeclareLaunchArgument('gripper_detect_threshold', default_value='0.01'),
        DeclareLaunchArgument(
            'gripper_backend', default_value='sdk',
            description="'sdk' = Piper SDK (real), 'ros' = /gripper_controller (sim)"),
        # I2 修复：默认候选数 1 → 50（真实 CSV 37 候选下 default=1 曾导致
        # rank[0] 门控漂移即 tested=0、流程永不执行；节点侧另有上限 300 钳制）
        DeclareLaunchArgument('graspnet_6d_max_candidates', default_value='150'),
        DeclareLaunchArgument(
            'candidate_frames', default_value='1',
            description='每次触发最多等待的新鲜候选帧数'),
        # 2026-08-07：候选质量阈值（默认 0 = 不过滤；用户可传 0.33/0.45 等）
        DeclareLaunchArgument('grasp_score_min', default_value='0.0'),
        DeclareLaunchArgument(
            'min_approach_down', default_value='0.70',
            description='TCP +Z 接近轴相对 base -Z 的最小对齐值'),
        DeclareLaunchArgument('enable_self_consistency', default_value='true'),
        DeclareLaunchArgument(
            'self_consistency_max_center_dist', default_value='0.08',
            description='物体中心与抓取中心横向最大距离（米）'),
        DeclareLaunchArgument('max_orientation_variants', default_value='2'),
        DeclareLaunchArgument(
            'enable_piper_analytic_ik_planning', default_value='false'),
        DeclareLaunchArgument(
            # 2026-08-10 用户裁决：预检只走 KDL（ssik 实测 58 次求解 0 成功），
            # 门控必须关——门控检查在 ssik 块之外，开着会让 KDL 路径被短路
            'graspnet_6d_analytic_only_precheck', default_value='false'),
        DeclareLaunchArgument(
            'piper_analytic_ik_script', default_value=_DEFAULT_SSIK_SCRIPT),
        DeclareLaunchArgument(
            'piper_analytic_ik_max_solutions', default_value='8'),
        DeclareLaunchArgument(
            'piper_analytic_ik_max_solution_plans', default_value='6'),
        # 2026-08-10 用户裁决：每段预检求解预算 0.5s（失败候选由 ~1.0s 减半）
        DeclareLaunchArgument('planning_time_precheck', default_value='0.5'),
        DeclareLaunchArgument('planning_time_exec', default_value='10.0'),
        DeclareLaunchArgument(
            'enable_sim_lift', default_value='false',
            description='Simulation only: lift 10 cm after a successful grasp'),
        DeclareLaunchArgument(
            'skip_final_home', default_value='false',
            description='Debug only: leave the arm at the final pose'),

        Node(
            package='real_piper_perception',
            executable='grasp_executor',
            name='grasp_executor_dry',
            parameters=[{
                'scan_only': scan_only,
                'dry_run_max_stage': dry_run_max_stage,
                'velocity_scale': velocity_scale,
                'handeye_yaml': handeye_yaml,
                'shared_dir': shared_dir,
                'csv_path': csv_path,
                'approach_clearance': approach_clearance,
                'lift_z': lift_z,
                'open_gripper_lift_height': open_gripper_lift_height,
                'grasp_z_offset': grasp_z_offset,
                'gripper_margin': gripper_margin,
                'gripper_detect_threshold': gripper_detect_threshold,
                'gripper_backend': gripper_backend,
                'graspnet_6d_max_candidates': graspnet_6d_max_candidates,
                'candidate_frames': candidate_frames,
                'grasp_score_min': grasp_score_min,
                'min_approach_down': min_approach_down,
                'enable_self_consistency': enable_self_consistency,
                'self_consistency_max_center_dist': self_consistency_max_center_dist,
                'max_orientation_variants': max_orientation_variants,
                'enable_piper_analytic_ik_planning': enable_piper_analytic_ik_planning,
                'graspnet_6d_analytic_only_precheck': graspnet_6d_analytic_only_precheck,
                'piper_analytic_ik_script': piper_analytic_ik_script,
                'piper_analytic_ik_max_solutions': piper_analytic_ik_max_solutions,
                'piper_analytic_ik_max_solution_plans': piper_analytic_ik_max_solution_plans,
                'planning_time_precheck': planning_time_precheck,
                'planning_time_exec': planning_time_exec,
                'enable_sim_lift': ParameterValue(
                    enable_sim_lift, value_type=bool),
                'skip_final_home': ParameterValue(
                    skip_final_home, value_type=bool),
            }],
            remappings=[
                ('grasp_execute', '/grasp_execute_dry'),
                ('grasp_result', '/grasp_result_dry'),
            ],
            output='screen',
        ),
    ])
