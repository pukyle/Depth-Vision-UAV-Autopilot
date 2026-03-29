import json
import os
import sys
import numpy as np

"""
Analysis script aligned with paper (Gaigalas et al., Drones 2025, 9, 236).

Reads results from run_batch_test.py output directory structure:
  results/{ENV}/{PLANNER}/S{n}_V{v}/trial_XX.json

Produces paper Table 5-7 style output:
  Per (voxel_size, scenario): reached_goal, avg_collisions, avg_distance,
                              avg_time, avg_distance_to_goal (failed only)
"""

# Paper Table 2
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

VOXEL_SIZES = [0.5, 1.0, 2.0]
GOAL_THRESHOLD = 5.0  # Paper: within 5m counts as reached

BASE_OUTPUT_DIR = "/catkin_ws/src/results"


def analyze_single_trial(filepath, target_pos):
    """Analyze a single trial JSON file. Returns a dict of metrics or None."""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except (json.JSONDecodeError, FileNotFoundError) as e:
        print(f"  Warning: Cannot read {filepath}: {e}")
        return None

    logs = data.get("log", [])
    if not logs:
        return None

    target = np.array(target_pos)

    # --- Goal reached? ---
    last_entry = logs[-1]
    last_pos = np.array(last_entry["real_pos"])
    dist_to_goal = np.linalg.norm(last_pos - target)
    reached = last_entry.get("gloal_reached", False) and dist_to_goal < GOAL_THRESHOLD

    # --- Collision count (total steps with collision) ---
    collision_steps = sum(1 for entry in logs if entry.get("has_col", False))

    # --- Actual flight distance ---
    positions = [np.array(entry["real_pos"]) for entry in logs]
    flight_distance = sum(
        np.linalg.norm(positions[i] - positions[i - 1])
        for i in range(1, len(positions))
    )

    # --- Flight time ---
    if "time_start" in logs[0] and "time_end" in logs[-1]:
        flight_time = logs[-1]["time_end"] - logs[0]["time_start"]
    else:
        flight_time = 0.0

    # --- Computational time per frame (FPS) ---
    frame_times = []
    for entry in logs:
        t_start = entry.get("time_start")
        t_end = entry.get("time_end")
        if t_start is not None and t_end is not None:
            frame_times.append(t_end - t_start)
    avg_frame_time = np.mean(frame_times) if frame_times else 0.0
    fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0.0

    # --- Path length ratio ---
    # (actual / theoretical Euclidean distance)

    # --- Smoothness (jerk = sum of acceleration changes) ---
    acc_values = [np.array(entry["acc"]) for entry in logs if "acc" in entry]
    jerk = 0.0
    if len(acc_values) > 1:
        jerk = sum(
            np.linalg.norm(acc_values[i] - acc_values[i - 1])
            for i in range(1, len(acc_values))
        )

    return {
        "reached": reached,
        "dist_to_goal": dist_to_goal,
        "collisions": collision_steps,
        "flight_distance": flight_distance,
        "flight_time": flight_time,
        "fps": fps,
        "jerk": jerk,
        "total_frames": len(logs),
    }


def analyze_combo(result_dir, target_pos):
    """Analyze all trials in a single (scenario, voxel_size) directory."""
    if not os.path.isdir(result_dir):
        return None

    files = sorted([f for f in os.listdir(result_dir) if f.endswith('.json')])
    if not files:
        return None

    results = []
    for f in files:
        r = analyze_single_trial(os.path.join(result_dir, f), target_pos)
        if r is not None:
            results.append(r)

    if not results:
        return None

    n = len(results)
    reached_count = sum(1 for r in results if r["reached"])
    failed = [r for r in results if not r["reached"]]
    succeeded = [r for r in results if r["reached"]]

    # Paper columns: Reached, Collisions (avg of succeeded), Distance, Time, Distance to Goal (avg of failed)
    avg_collisions = np.mean([r["collisions"] for r in succeeded]) if succeeded else None
    avg_distance = np.mean([r["flight_distance"] for r in succeeded]) if succeeded else None
    avg_time = np.mean([r["flight_time"] for r in succeeded]) if succeeded else None
    avg_dist_to_goal = np.mean([r["dist_to_goal"] for r in failed]) if failed else None
    avg_fps = np.mean([r["fps"] for r in results])
    avg_jerk = np.mean([r["jerk"] for r in results])

    # Additional: path ratio for succeeded flights
    avg_all_distance = np.mean([r["flight_distance"] for r in results])

    return {
        "n_trials": n,
        "reached": reached_count,
        "avg_collisions": avg_collisions,
        "avg_distance": avg_distance,
        "avg_time": avg_time,
        "avg_dist_to_goal_failed": avg_dist_to_goal,
        "avg_fps": avg_fps,
        "avg_jerk": avg_jerk,
        "avg_all_distance": avg_all_distance,
    }


def fmt(val, decimals=2):
    """Format a value for display, returning '-' for None."""
    if val is None:
        return "-"
    return f"{val:.{decimals}f}"


def print_paper_table(env, planner, all_results):
    """Print results in paper Table 5-7 format."""
    print(f"\n{'=' * 90}")
    print(f"  {env} — {planner}")
    print(f"{'=' * 90}")
    header = (f"{'Voxel':>5} {'Scen':>4} | {'Reached':>8} | {'Collisions':>10} | "
              f"{'Distance':>10} | {'Time':>8} | {'Dist2Goal':>10} | {'FPS':>6}")
    print(header)
    print("-" * 90)

    total_reached = 0
    total_trials = 0

    for voxel_size in VOXEL_SIZES:
        for scenario in [1, 2, 3]:
            key = (voxel_size, scenario)
            r = all_results.get(key)
            if r is None:
                print(f"{voxel_size:>5} {scenario:>4} |     (no data)")
                continue

            total_reached += r["reached"]
            total_trials += r["n_trials"]

            reached_str = f"{r['reached']}/{r['n_trials']}"
            row = (f"{voxel_size:>5} {scenario:>4} | {reached_str:>8} | "
                   f"{fmt(r['avg_collisions']):>10} | "
                   f"{fmt(r['avg_distance']):>10} | "
                   f"{fmt(r['avg_time']):>8} | "
                   f"{fmt(r['avg_dist_to_goal_failed']):>10} | "
                   f"{fmt(r['avg_fps']):>6}")
            print(row)

    print("-" * 90)
    if total_trials > 0:
        print(f"  Total: {total_reached}/{total_trials} reached "
              f"({total_reached / total_trials * 100:.1f}%)")
    print(f"{'=' * 90}\n")


def main():
    # --- Select what to analyze ---
    print("=" * 50)
    print("  Experiment Results Analyzer (Paper-Aligned)")
    print("=" * 50)

    # Auto-detect available results
    available = []
    for env in ["AirSimNH", "Blocks"]:
        for planner in ["a_star", "rrt_star"]:
            check_dir = os.path.join(BASE_OUTPUT_DIR, env, planner)
            if os.path.isdir(check_dir):
                available.append((env, planner))

    if not available:
        print(f"\nNo results found in {BASE_OUTPUT_DIR}/")
        print("Expected structure: {BASE_OUTPUT_DIR}/{{ENV}}/{{PLANNER}}/S{{n}}_V{{v}}/")
        print("\nYou can also specify a custom path:")
        custom = input("Enter results base directory (or press Enter to quit): ").strip()
        if not custom:
            return
        # Re-scan with custom path
        global BASE_OUTPUT_DIR
        BASE_OUTPUT_DIR = custom
        for env in ["AirSimNH", "Blocks"]:
            for planner in ["a_star", "rrt_star"]:
                check_dir = os.path.join(BASE_OUTPUT_DIR, env, planner)
                if os.path.isdir(check_dir):
                    available.append((env, planner))

    if not available:
        print("Still no results found. Exiting.")
        return

    print(f"\nFound results for:")
    for i, (env, planner) in enumerate(available):
        print(f"  {i + 1}. {env} / {planner}")
    print(f"  {len(available) + 1}. Analyze ALL")

    choice = input(f"\nSelect (1-{len(available) + 1}): ").strip()

    if choice == str(len(available) + 1):
        to_analyze = available
    elif choice.isdigit() and 1 <= int(choice) <= len(available):
        to_analyze = [available[int(choice) - 1]]
    else:
        to_analyze = available

    # --- Run analysis ---
    for env, planner in to_analyze:
        scenarios_info = SCENARIOS.get(env)
        if not scenarios_info:
            print(f"Warning: No scenario info for {env}, skipping.")
            continue

        all_results = {}

        for voxel_size in VOXEL_SIZES:
            for scenario in [1, 2, 3]:
                result_dir = os.path.join(
                    BASE_OUTPUT_DIR, env, planner,
                    f"S{scenario}_V{voxel_size}"
                )
                target_pos = scenarios_info[scenario]["goal"]
                r = analyze_combo(result_dir, target_pos)
                if r is not None:
                    all_results[(voxel_size, scenario)] = r

        if all_results:
            print_paper_table(env, planner, all_results)
        else:
            print(f"\n  No data found for {env}/{planner}")


if __name__ == "__main__":
    main()