import subprocess
import os
import time
import math

"""
Batch test runner aligned with paper (Gaigalas et al., Drones 2025, 9, 236).

Paper's experimental setup (Section 4.2.1):
  - 3 environments × 3 scenarios × 3 voxel sizes × 10 trials = 810 flights total
  - Voxel sizes: 0.5, 1.0, 2.0
  - Speed: 2 m/s
  - Max A* iterations: 200
  - Collision watchdog: 20 consecutive collision steps → quit
  - No-path watchdog: 200 consecutive failures → quit (from paper's path planner limit)
  - Goal reached threshold: 5 m
  - Map size: 1800 × 1800 × 100 (at V=0.5; scaled proportionally for other V)
"""

# --- Paper Table 2: Start and end coordinates ---
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

# --- Paper parameters ---
VOXEL_SIZES = [0.5, 1.0, 2.0]
TRIALS_PER_COMBO = 10       # Paper: 10 flights per (scenario, voxel_size) pair
SPEED = 2                   # Paper: 2 m/s
MAX_A_STAR_ITERS = 200      # Paper: max 200 iterations
COLLISION_WATCHDOG = 20     # Paper: 20 consecutive collision steps → quit
NO_PATH_WATCHDOG = 200      # Paper: 200 consecutive no-path failures → quit
GOAL_ERR = 5                # Paper: 5 m threshold for "reached"
TIMEOUT_PER_TRIAL = 600     # 10 minutes timeout per trial

# Map coverage: paper uses 1800×1800×100 at V=0.5
# At V=0.5, 1800 voxels × 0.5 = 900m coverage
# We scale map_size = int(900 / voxel_size) to keep same physical coverage
MAP_COVERAGE_METERS = 900   # Physical coverage in meters
MAP_HEIGHT = 100            # Z dimension stays fixed

MAPPER_SCRIPT = "/catkin_ws/src/drone-node/src/mapper_nav_ros.py"
BASE_OUTPUT_DIR = "/catkin_ws/src/results"


def compute_map_size(voxel_size):
    """Compute map dimensions to maintain consistent physical coverage."""
    xy_size = int(math.ceil(MAP_COVERAGE_METERS / voxel_size))
    return xy_size, xy_size, MAP_HEIGHT


def build_command(planner_type, goal, voxel_size, logfile):
    """Build the subprocess command with all paper-aligned parameters."""
    map_w, map_d, map_h = compute_map_size(voxel_size)

    cmd = [
        "python3", MAPPER_SCRIPT,
        "--planner_type", planner_type,
        "--goal_off", str(goal[0]), str(goal[1]), str(goal[2]),
        "--exit_on_goal",
        "--logfile", logfile,
        # Paper parameters
        "--speed", str(SPEED),
        "--map_resolution", str(voxel_size),
        "--map_depth", str(map_d),
        "--map_width", str(map_w),
        "--map_heigth", str(map_h),
        "--max_a_star_iters", str(MAX_A_STAR_ITERS),
        "--collision_watchdog_cnt", str(COLLISION_WATCHDOG),
        "--no_path_watchdog_cnt", str(NO_PATH_WATCHDOG),
        "--goal_err", str(GOAL_ERR),
    ]
    return cmd


def run_batch():
    print("=" * 50)
    print("  Drone Batch Test (Paper-Aligned)")
    print("=" * 50)

    # --- 1. Environment Selection ---
    env_choice = ""
    while env_choice not in ["1", "2"]:
        print("\nSelect Environment:")
        print("  1. Blocks")
        print("  2. AirSimNH")
        env_choice = input("Enter your choice (1 or 2): ").strip()
    ENV = "Blocks" if env_choice == "1" else "AirSimNH"

    # --- 2. Planner Selection ---
    planner_choice = ""
    while planner_choice not in ["1", "2"]:
        print("\nSelect Planner Type:")
        print("  1. A*")
        print("  2. Informed-RRT*")
        planner_choice = input("Enter your choice (1 or 2): ").strip()
    PLANNER_TYPE = "a_star" if planner_choice == "1" else "rrt_star"

    # --- 3. Scenario Selection ---
    scenario_choice = ""
    while scenario_choice not in ["1", "2", "3", "4"]:
        print("\nSelect Scenario:")
        print("  1. Scenario 1")
        print("  2. Scenario 2")
        print("  3. Scenario 3")
        print("  4. Run ALL Scenarios (1, 2, and 3)")
        scenario_choice = input("Enter your choice (1/2/3/4): ").strip()

    scenarios_to_run = [1, 2, 3] if scenario_choice == "4" else [int(scenario_choice)]

    # --- 4. Voxel Size Selection ---
    voxel_choice = ""
    while voxel_choice not in ["1", "2", "3", "4"]:
        print("\nSelect Voxel Size:")
        print("  1. V = 0.5")
        print("  2. V = 1.0")
        print("  3. V = 2.0")
        print("  4. Run ALL Voxel Sizes (0.5, 1.0, 2.0)")
        voxel_choice = input("Enter your choice (1/2/3/4): ").strip()

    if voxel_choice == "4":
        voxels_to_run = VOXEL_SIZES
    else:
        voxels_to_run = [VOXEL_SIZES[int(voxel_choice) - 1]]

    # --- Summary ---
    total_runs = len(scenarios_to_run) * len(voxels_to_run) * TRIALS_PER_COMBO
    print(f"\n{'=' * 50}")
    print(f"  Environment:    {ENV}")
    print(f"  Planner:        {PLANNER_TYPE}")
    print(f"  Scenarios:      {scenarios_to_run}")
    print(f"  Voxel Sizes:    {voxels_to_run}")
    print(f"  Trials/combo:   {TRIALS_PER_COMBO}")
    print(f"  Total runs:     {total_runs}")
    print(f"  Speed:          {SPEED} m/s")
    print(f"  Max A* iters:   {MAX_A_STAR_ITERS}")
    print(f"  Collision stop: {COLLISION_WATCHDOG} steps")
    print(f"  Goal threshold: {GOAL_ERR} m")
    print(f"{'=' * 50}\n")

    confirm = input("Proceed? (y/n): ").strip().lower()
    if confirm != "y":
        print("Aborted.")
        return

    # --- Execution ---
    run_count = 0
    for scenario in scenarios_to_run:
        goal_info = SCENARIOS[ENV][scenario]
        goal = goal_info["goal"]

        for voxel_size in voxels_to_run:
            map_w, map_d, _ = compute_map_size(voxel_size)

            # Output directory: results/{ENV}/{PLANNER}/S{n}_V{v}/
            output_dir = os.path.join(
                BASE_OUTPUT_DIR, ENV, PLANNER_TYPE,
                f"S{scenario}_V{voxel_size}"
            )
            os.makedirs(output_dir, exist_ok=True)

            print(f"\n>>> Scenario {scenario} | V={voxel_size} | "
                  f"Goal={goal} | Map={map_w}x{map_d} | "
                  f"Dir: {output_dir}")

            for trial in range(1, TRIALS_PER_COMBO + 1):
                run_count += 1
                logfile = os.path.join(output_dir, f"trial_{trial:02d}.json")

                # Skip if already completed
                if os.path.exists(logfile):
                    print(f"  [{run_count}/{total_runs}] Trial {trial} "
                          f"already exists, skipping.")
                    continue

                cmd = build_command(PLANNER_TYPE, goal, voxel_size, logfile)

                print(f"  [{run_count}/{total_runs}] S{scenario} V{voxel_size} "
                      f"Trial {trial}/{TRIALS_PER_COMBO}...")

                try:
                    subprocess.run(cmd, timeout=TIMEOUT_PER_TRIAL)
                    print(f"  [OK] Trial {trial} finished.")
                except subprocess.TimeoutExpired:
                    print(f"  [TIMEOUT] Trial {trial} exceeded {TIMEOUT_PER_TRIAL}s.")
                except Exception as e:
                    print(f"  [ERROR] Trial {trial}: {e}")

                # Buffer for AirSim reset
                time.sleep(3)

    print(f"\n{'=' * 50}")
    print(f"  All {total_runs} trials completed!")
    print(f"  Results in: {BASE_OUTPUT_DIR}/{ENV}/{PLANNER_TYPE}/")
    print(f"{'=' * 50}")


if __name__ == "__main__":
    run_batch()