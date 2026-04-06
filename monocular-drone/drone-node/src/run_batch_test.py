import subprocess
import os
import time
import math
import sys

"""
Batch test runner aligned with paper (Gaigalas et al., Drones 2025, 9, 236).

Features:
  - Run EVERYTHING mode: A* + RRT* x all scenarios x all voxels, then auto-analyze
  - Depth source selection: monocular estimation vs AirSim ground truth
  - Overwrite option: re-run and overwrite existing trial JSONs
  - Paper-aligned parameters for fair comparison
"""

SCENARIOS = {
    "AirSimNH": {
        1: {"goal": [-130, -115, 3], "distance": 173.59},
        2: {"goal": [130, 130, 3],   "distance": 183.87},
        3: {"goal": [-115, 130, 3],  "distance": 173.59},
    },
    "Blocks": {
        1: {"goal": [70, 150, 3],    "distance": 165.56},
        2: {"goal": [70, -135, 3],   "distance": 152.10},
        3: {"goal": [-105, -150, 3], "distance": 183.12},
    },
}

ALL_PLANNERS = ["a_star", "rrt_star"]
ALL_VOXELS = [0.5, 1.0, 2.0]
ALL_SCENARIOS = [1, 2, 3]
TRIALS_PER_COMBO = 10
SPEED = 2
MAX_A_STAR_ITERS = 200
COLLISION_WATCHDOG = 20
NO_PATH_WATCHDOG = 50          # Paper default (stuck detector, not path failure)
GOAL_ERR = 5
TIMEOUT_PER_TRIAL = 600

# Paper Section 3.1: fixed map 1800x1800x100, fixed resolution 2
# Voxel size V is map_resolution. Map dimensions do NOT change per V.
MAP_WIDTH = 1800
MAP_DEPTH = 1800
MAP_HEIGHT = 100
DEFAULT_RESOLUTION = 2         # Paper default, used when voxel_size not overridden

MAPPER_SCRIPT = "/catkin_ws/src/drone-node/src/mapper_nav_ros.py"
ANALYZE_SCRIPT = "/catkin_ws/src/drone-node/src/analyze_metrics.py"
BASE_OUTPUT_DIR = "/catkin_ws/src/results"


def build_command(planner_type, goal, voxel_size, logfile, use_airsim_depth):
    cmd = [
        "python3", MAPPER_SCRIPT,
        "--planner_type", planner_type,
        "--goal_off", str(goal[0]), str(goal[1]), str(goal[2]),
        "--exit_on_goal",
        "--logfile", logfile,
        "--speed", str(SPEED),
        "--map_resolution", str(voxel_size),
        "--map_depth", str(MAP_DEPTH),
        "--map_width", str(MAP_WIDTH),
        "--map_heigth", str(MAP_HEIGHT),
        "--max_a_star_iters", str(MAX_A_STAR_ITERS),
        "--collision_watchdog_cnt", str(COLLISION_WATCHDOG),
        "--no_path_watchdog_cnt", str(NO_PATH_WATCHDOG),
        "--goal_err", str(GOAL_ERR),
    ]
    if use_airsim_depth:
        cmd.append("--use_airsim_depth")
    else:
        cmd.append("--use_rgb_imaging")
    return cmd


def prompt_choice(prompt, options):
    while True:
        print(prompt)
        for i, (label, _) in enumerate(options, 1):
            print(f"  {i}. {label}")
        choice = input(f"Enter choice (1-{len(options)}): ").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(options):
            return options[int(choice) - 1][1]


def run_trials(env, planner, depth_label, use_airsim_depth,
               scenarios, voxels, overwrite):
    """Run a set of trials for one planner. Returns number of runs executed."""
    run_count = 0
    total = len(scenarios) * len(voxels) * TRIALS_PER_COMBO

    for scenario in scenarios:
        goal = SCENARIOS[env][scenario]["goal"]

        for voxel_size in voxels:
            output_dir = os.path.join(
                BASE_OUTPUT_DIR, env, f"{planner}_{depth_label}",
                f"S{scenario}_V{voxel_size}"
            )
            os.makedirs(output_dir, exist_ok=True)

            print(f"\n>>> [{planner}] S{scenario} V={voxel_size} | "
                  f"Goal={goal} | Map={MAP_WIDTH}x{MAP_DEPTH}")

            for trial in range(1, TRIALS_PER_COMBO + 1):
                run_count += 1
                logfile = os.path.join(output_dir, f"trial_{trial:02d}.json")

                if os.path.exists(logfile) and not overwrite:
                    print(f"  [{run_count}/{total}] Trial {trial} exists, skipping.")
                    continue

                cmd = build_command(planner, goal, voxel_size, logfile,
                                    use_airsim_depth)

                print(f"  [{run_count}/{total}] S{scenario} V{voxel_size} "
                      f"Trial {trial}/{TRIALS_PER_COMBO}...")

                try:
                    subprocess.run(cmd, timeout=TIMEOUT_PER_TRIAL)
                    print(f"  [OK] Trial {trial} finished.")
                except subprocess.TimeoutExpired:
                    print(f"  [TIMEOUT] Trial {trial} > {TIMEOUT_PER_TRIAL}s.")
                except Exception as e:
                    print(f"  [ERROR] Trial {trial}: {e}")

                time.sleep(3)

    return run_count


def run_analysis(env):
    """Run analyze_metrics.py and export CSV."""
    csv_path = os.path.join(BASE_OUTPUT_DIR, env, "results_summary.csv")
    print(f"\n{'=' * 60}")
    print(f"  Running analysis for {env}...")
    print(f"{'=' * 60}")

    analyze_cmd = [
        "python3", ANALYZE_SCRIPT,
        "--dir", BASE_OUTPUT_DIR,
        "--csv", csv_path,
    ]

    try:
        subprocess.run(analyze_cmd, timeout=120)
        print(f"\n  Analysis complete! CSV saved to: {csv_path}")
    except Exception as e:
        print(f"\n  [ERROR] Analysis failed: {e}")
        print(f"  You can run manually: python3 {ANALYZE_SCRIPT} --dir {BASE_OUTPUT_DIR}")


def run_batch():
    print("=" * 60)
    print("  Drone Batch Test (Paper-Aligned)")
    print("  github.com/pukyle/Depth-Vision-UAV-Autopilot")
    print("=" * 60)

    # 1. Environment
    ENV = prompt_choice("\nSelect Environment:", [
        ("Blocks", "Blocks"),
        ("AirSimNH", "AirSimNH"),
    ])

    # 2. Mode selection
    MODE = prompt_choice("\nSelect Mode:", [
        ("Custom (pick planner/scenario/voxel)", "custom"),
        ("Run EVERYTHING - single depth source (A* + RRT*, all scenarios, all voxels)", "everything"),
        ("Run FULL EXPERIMENT - BOTH depth sources (AirSim + estimated, then analyze)", "full"),
    ])

    # 3. Depth source (skip for full mode — it runs both)
    if MODE == "full":
        USE_AIRSIM_DEPTH = None  # placeholder, will run both
        depth_label = "both"
    else:
        USE_AIRSIM_DEPTH = prompt_choice("\nSelect Depth Source:", [
            ("Monocular depth estimation (Depth Anything V2)", False),
            ("AirSim ground-truth depth (no GPU cost)", True),
        ])
        depth_label = "airsim_depth" if USE_AIRSIM_DEPTH else "estimated_depth"

    # 4. Overwrite
    OVERWRITE = prompt_choice("\nExisting trial files:", [
        ("Skip existing (resume interrupted batch)", False),
        ("Overwrite all (re-run from scratch)", True),
    ])

    if MODE == "full":
        # ══════════════════════════════════════
        #  FULL EXPERIMENT: both depth sources
        # ══════════════════════════════════════
        planners = ALL_PLANNERS
        scenarios = ALL_SCENARIOS
        voxels = ALL_VOXELS
        depth_configs = [
            ("airsim_depth", True),
            ("estimated_depth", False),
        ]
        total_all = len(planners) * len(scenarios) * len(voxels) * TRIALS_PER_COMBO * len(depth_configs)

        print(f"\n{'=' * 60}")
        print(f"  === FULL EXPERIMENT MODE ===")
        print(f"  Environment:    {ENV}")
        print(f"  Planners:       {planners}")
        print(f"  Depth sources:  AirSim ground-truth -> Depth estimation")
        print(f"  Scenarios:      {scenarios}")
        print(f"  Voxel Sizes:    {voxels}")
        print(f"  Trials/combo:   {TRIALS_PER_COMBO}")
        print(f"  Total runs:     {total_all}")
        print(f"  Overwrite:      {OVERWRITE}")
        print(f"  Map:            {MAP_WIDTH}x{MAP_DEPTH}x{MAP_HEIGHT} (fixed)")
        print(f"  Estimated time: ~{total_all * 3} min")
        print(f"{'=' * 60}")

        confirm = input("\nThis will take a VERY long time. Proceed? (y/n): ").strip().lower()
        if confirm != "y":
            print("Aborted.")
            return

        t_start = time.time()

        for d_label, use_airsim in depth_configs:
            print(f"\n{'#' * 60}")
            print(f"  DEPTH SOURCE: {d_label.upper()}")
            print(f"{'#' * 60}")

            for planner in planners:
                print(f"\n{'=' * 60}")
                print(f"  [{d_label}] PLANNER: {planner.upper()}")
                print(f"{'=' * 60}")

                run_trials(ENV, planner, d_label, use_airsim,
                           scenarios, voxels, OVERWRITE)

        elapsed = time.time() - t_start
        hours = int(elapsed // 3600)
        mins = int((elapsed % 3600) // 60)

        print(f"\n{'=' * 60}")
        print(f"  FULL EXPERIMENT completed in {hours}h {mins}m")
        print(f"  Results: {BASE_OUTPUT_DIR}/{ENV}/")
        print(f"{'=' * 60}")

        # Auto-run analysis
        run_analysis(ENV)

    elif MODE == "everything":
        # ══════════════════════════════════════
        #  RUN EVERYTHING MODE
        # ══════════════════════════════════════
        planners = ALL_PLANNERS
        scenarios = ALL_SCENARIOS
        voxels = ALL_VOXELS
        total_all = len(planners) * len(scenarios) * len(voxels) * TRIALS_PER_COMBO

        print(f"\n{'=' * 60}")
        print(f"  === RUN EVERYTHING MODE ===")
        print(f"  Environment:    {ENV}")
        print(f"  Planners:       {planners}")
        print(f"  Depth source:   {depth_label}")
        print(f"  Scenarios:      {scenarios}")
        print(f"  Voxel Sizes:    {voxels}")
        print(f"  Trials/combo:   {TRIALS_PER_COMBO}")
        print(f"  Total runs:     {total_all}")
        print(f"  Overwrite:      {OVERWRITE}")
        print(f"  Estimated time: ~{total_all * 5} min (assuming ~5 min/trial)")
        print(f"{'=' * 60}")

        confirm = input("\nThis will take a LONG time. Proceed? (y/n): ").strip().lower()
        if confirm != "y":
            print("Aborted.")
            return

        t_start = time.time()

        for planner in planners:
            print(f"\n{'#' * 60}")
            print(f"  PLANNER: {planner.upper()}")
            print(f"{'#' * 60}")

            run_trials(ENV, planner, depth_label, USE_AIRSIM_DEPTH,
                       scenarios, voxels, OVERWRITE)

        elapsed = time.time() - t_start
        hours = int(elapsed // 3600)
        mins = int((elapsed % 3600) // 60)

        print(f"\n{'=' * 60}")
        print(f"  All trials completed in {hours}h {mins}m")
        print(f"  Results: {BASE_OUTPUT_DIR}/{ENV}/")
        print(f"{'=' * 60}")

        # Auto-run analysis
        run_analysis(ENV)

    else:
        # ══════════════════════════════════════
        #  CUSTOM MODE (original behavior)
        # ══════════════════════════════════════
        PLANNER = prompt_choice("\nSelect Planner:", [
            ("A* (original paper method)", "a_star"),
            ("Informed-RRT* (extended method)", "rrt_star"),
        ])

        scenarios = prompt_choice("\nSelect Scenario:", [
            ("Scenario 1", [1]),
            ("Scenario 2", [2]),
            ("Scenario 3", [3]),
            ("ALL Scenarios (1, 2, 3)", ALL_SCENARIOS),
        ])

        voxels = prompt_choice("\nSelect Voxel Size:", [
            ("V = 0.5", [0.5]),
            ("V = 1.0", [1.0]),
            ("V = 2.0", [2.0]),
            ("ALL Voxel Sizes (0.5, 1.0, 2.0)", ALL_VOXELS),
        ])

        total = len(scenarios) * len(voxels) * TRIALS_PER_COMBO

        print(f"\n{'=' * 60}")
        print(f"  Environment:    {ENV}")
        print(f"  Planner:        {PLANNER}")
        print(f"  Depth source:   {depth_label}")
        print(f"  Scenarios:      {scenarios}")
        print(f"  Voxel Sizes:    {voxels}")
        print(f"  Trials/combo:   {TRIALS_PER_COMBO}")
        print(f"  Total runs:     {total}")
        print(f"  Overwrite:      {OVERWRITE}")
        print(f"{'=' * 60}")

        confirm = input("\nProceed? (y/n): ").strip().lower()
        if confirm != "y":
            print("Aborted.")
            return

        run_trials(ENV, PLANNER, depth_label, USE_AIRSIM_DEPTH,
                   scenarios, voxels, OVERWRITE)

        print(f"\n{'=' * 60}")
        print(f"  All {total} trials completed!")
        print(f"  Results: {BASE_OUTPUT_DIR}/{ENV}/{PLANNER}_{depth_label}/")
        print(f"{'=' * 60}")

        # Ask to run analysis
        do_analyze = input("\nRun analysis now? (y/n): ").strip().lower()
        if do_analyze == "y":
            run_analysis(ENV)


if __name__ == "__main__":
    run_batch()