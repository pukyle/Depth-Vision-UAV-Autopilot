"""
Flight Experiment Analyzer — Report-ready tables.

Output matches the exact format from the project report:
  TABLE 1: Start/end coordinates (fixed)
  TABLE 2: Hardware specs (fixed)
  TABLE 3: Overall navigation results
  TABLE 4-11: Detailed results per method x environment
"""

import json, os, csv, math, argparse, glob
import numpy as np

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
VOXELS = [0.5, 1.0, 2.0]
GOAL_THR = 5

# Table numbering: method_key -> (Blocks table#, NH table#)
TABLE_MAP = {
    "a_star_airsim_depth":       ("A* (Ground Truth Depth)",    4, 8),
    "a_star_estimated_depth":    ("A* (Estimated Depth)",       5, 9),
    "rrt_star_airsim_depth":     ("RRT* (Ground Truth Depth)",  6, 10),
    "rrt_star_estimated_depth":  ("RRT* (Estimated Depth)",     7, 11),
}


# ══════════════════════════════════════════════════════
#  Trial analysis
# ══════════════════════════════════════════════════════
def analyze_trial(fp, target, theo_dist):
    try:
        with open(fp, 'r') as f:
            data = json.load(f)
    except:
        return None
    logs = data.get("log", [])
    cfg = data.get("cfg", {})
    if not logs:
        return None

    target = np.array(target)
    n = len(logs)

    last_pos = np.array(logs[-1]["real_pos"])
    d2g_raw = np.linalg.norm(last_pos - target)
    d2g = math.floor(d2g_raw)
    reached = d2g <= GOAL_THR

    col_frames = sum(1 for l in logs if l.get("has_col", False))
    positions = [np.array(l["real_pos"]) for l in logs]
    flight_dist = sum(np.linalg.norm(positions[i] - positions[i-1])
                      for i in range(1, len(positions)))
    flight_time = logs[-1]["time_end"] - logs[0]["time_start"]
    path_ratio = flight_dist / theo_dist if theo_dist > 0 else 0

    acc_vals = [np.array(l["acc"]) for l in logs if "acc" in l]
    jerk = 0.0
    if len(acc_vals) > 1:
        jerk = sum(np.linalg.norm(acc_vals[i] - acc_vals[i-1])
                   for i in range(1, len(acc_vals)))
    jerk_per_frame = jerk / max(n - 1, 1)

    frame_times = [l["time_end"] - l["time_start"] for l in logs
                   if l.get("time_end") and l.get("time_start")
                   and l["time_end"] > l["time_start"]]
    avg_comp = np.mean(frame_times) if frame_times else 0
    fps = 1.0 / avg_comp if avg_comp > 0 else 0

    return {
        "reached": reached, "d2g": round(d2g_raw, 2),
        "collisions": col_frames, "distance": round(flight_dist, 2),
        "time": round(flight_time, 2), "path_ratio": round(path_ratio, 4),
        "jerk": round(jerk, 2), "jerk_per_frame": round(jerk_per_frame, 4),
        "fps": round(fps, 2), "n_frames": n,
        "planner": cfg.get("planner_type", "?"),
        "resolution": cfg.get("map_resolution", "?"),
    }


def scan_results(base_dir):
    """Scan all results and return nested dict: results[env][method][(vs,sc)] = [trials]"""
    results = {}
    for env_name, scenarios in SCENARIOS.items():
        env_path = os.path.join(base_dir, env_name)
        if not os.path.isdir(env_path):
            continue
        results[env_name] = {}
        for method_dir in sorted(os.listdir(env_path)):
            method_path = os.path.join(env_path, method_dir)
            if not os.path.isdir(method_path) or method_dir == "results_summary.csv":
                continue
            grouped = {}
            for vs in VOXELS:
                for sc in [1, 2, 3]:
                    combo = os.path.join(method_path, f"S{sc}_V{vs}")
                    if not os.path.isdir(combo):
                        continue
                    goal = scenarios[sc]["goal"]
                    dist = scenarios[sc]["distance"]
                    trials = []
                    for f in sorted(os.listdir(combo)):
                        if f.endswith('.json'):
                            r = analyze_trial(os.path.join(combo, f), goal, dist)
                            if r:
                                r["file"] = f
                                r["scenario"] = sc
                                trials.append(r)
                    if trials:
                        grouped[(vs, sc)] = trials
            if grouped:
                results[env_name][method_dir] = grouped
    return results


def f(v, d=2):
    if v is None:
        return "-"
    return f"{v:.{d}f}" if isinstance(v, float) else str(v)


# ══════════════════════════════════════════════════════
#  TABLE 1: Fixed — coordinates
# ══════════════════════════════════════════════════════
def print_table1():
    print(f"\n{'=' * 90}")
    print(f"TABLE 1. Start and end coordinates of different scenarios in chosen environments.")
    print(f"{'=' * 90}")
    print(f"{'Environment':<14s} {'Scenario':>8s} {'Start (X,Y,Z)':>16s} {'End (X,Y,Z)':>20s} {'Distance (m)':>14s}")
    print(f"{'-' * 85}")

    for env in ["AirSimNH", "Blocks"]:
        for sc in [1, 2, 3]:
            info = SCENARIOS[env][sc]
            g = info["goal"]
            label = env if sc == 1 else ""
            print(f"{label:<14s} {sc:>8d} {'(0, 0, 0)':>16s} {'({}, {}, {})'.format(*g):>20s} {info['distance']:>14.2f}")
        if env == "AirSimNH":
            print()

    print(f"{'=' * 90}")


# ══════════════════════════════════════════════════════
#  TABLE 2: Fixed — hardware
# ══════════════════════════════════════════════════════
def print_table2():
    print(f"\n{'=' * 70}")
    print(f"TABLE 2. Hardware specifications and depth estimation performance.")
    print(f"{'=' * 70}")
    print(f"{'Component / Metric':<40s} {'Specification / Value':>28s}")
    print(f"{'-' * 70}")
    print(f"{'CPU':<40s} {'13th Gen Intel i7-13620H (12C, 2.4GHz)':>28s}")
    print(f"{'GPU':<40s} {'NVIDIA RTX 5060 Laptop (8GB GDDR6)':>28s}")
    print(f"{'RAM':<40s} {'32.0 GB':>28s}")
    print(f"{'MAPE (AirSimNH, Fine-Tuned)':<40s} {'23.3 %':>28s}")
    print(f"{'MAPE (Blocks, Fine-Tuned)':<40s} {'39.4 %':>28s}")
    print(f"{'=' * 70}")


# ══════════════════════════════════════════════════════
#  TABLE 3: Overall results
# ══════════════════════════════════════════════════════
def print_table3(results):
    print(f"\n{'=' * 90}")
    print(f"TABLE 3. Overall navigation results across environments.")
    print(f"{'=' * 90}")
    print(f"{'Method':<30s} {'Environment':>12s} {'Reached Goal':>18s} {'Distance to Goal':>18s}")
    print(f"{'-' * 85}")

    method_order = [
        "a_star_airsim_depth", "a_star_estimated_depth",
        "rrt_star_airsim_depth", "rrt_star_estimated_depth",
    ]

    for method_key in method_order:
        if method_key not in TABLE_MAP:
            continue
        label = TABLE_MAP[method_key][0]

        for env in ["AirSimNH", "Blocks"]:
            if env not in results or method_key not in results[env]:
                env_label = env if method_key == method_order[0] or True else ""
                print(f"{label:<30s} {env:>12s} {'-':>18s} {'-':>18s}")
                continue

            grouped = results[env][method_key]
            all_trials = [t for trials in grouped.values() for t in trials]
            n = len(all_trials)
            reached = sum(1 for t in all_trials if t["reached"])
            failed = [t for t in all_trials if not t["reached"]]
            avg_d2g = np.mean([t["d2g"] for t in failed]) if failed else 0

            r_str = f"{reached}/{n} ({reached/n*100:.1f}%)"
            d_str = f"{avg_d2g:.2f}" if failed else "-"

            print(f"{label:<30s} {env:>12s} {r_str:>18s} {d_str:>18s}")

        print()  # blank line between methods

    print(f"{'=' * 90}")


# ══════════════════════════════════════════════════════
#  TABLE 4-11: Detailed per method x environment
# ══════════════════════════════════════════════════════
def print_detail_table(table_num, method_label, env, grouped):
    print(f"\n{'=' * 110}")
    print(f"TABLE {table_num}. Detailed results for {method_label} in the {env} environment.")
    print(f"{'=' * 110}")
    print(f"{'Scen.':>5s} {'Voxel Size (m)':>14s} | {'Reached':>8s} {'Cols':>6s} {'Dist (m)':>10s} "
          f"{'Time (s)':>9s} {'Dg (m)':>8s} | {'PathRatio':>10s} {'Jerk/f':>8s} {'FPS':>6s}")
    print(f"{'-' * 105}")

    overall_reached = 0
    overall_n = 0
    overall_d2g_list = []
    overall_pr_list = []
    overall_jf_list = []
    overall_fps_list = []

    for sc in [1, 2, 3]:
        for vs in VOXELS:
            trials = grouped.get((vs, sc), [])

            if not trials:
                sc_label = str(sc) if vs == VOXELS[0] else ""
                print(f"{sc_label:>5s} {vs:>14} | {'-':>8s} {'-':>6s} {'-':>10s} "
                      f"{'-':>9s} {'-':>8s} | {'-':>10s} {'-':>8s} {'-':>6s}")
                continue

            n = len(trials)
            reached = [t for t in trials if t["reached"]]
            failed = [t for t in trials if not t["reached"]]
            n_r = len(reached)

            overall_reached += n_r
            overall_n += n

            # Cols, Distance, Time: average over reached only
            avg_col = np.mean([t["collisions"] for t in reached]) if reached else None
            avg_dist = np.mean([t["distance"] for t in reached]) if reached else None
            avg_time = np.mean([t["time"] for t in reached]) if reached else None

            # Dg: average over failed only
            avg_d2g = np.mean([t["d2g"] for t in failed]) if failed else None
            if failed:
                overall_d2g_list.extend([t["d2g"] for t in failed])

            # Path Ratio: average over reached
            avg_pr = np.mean([t["path_ratio"] for t in reached]) if reached else None
            if reached:
                overall_pr_list.extend([t["path_ratio"] for t in reached])

            # Jerk/f: average over ALL trials
            avg_jf = np.mean([t["jerk_per_frame"] for t in trials])
            overall_jf_list.extend([t["jerk_per_frame"] for t in trials])

            # FPS: average over ALL trials
            avg_fps = np.mean([t["fps"] for t in trials])
            overall_fps_list.extend([t["fps"] for t in trials])

            sc_label = str(sc) if vs == VOXELS[0] else ""
            r_str = f"{n_r}/{n}"

            print(f"{sc_label:>5s} {vs:>14} | {r_str:>8s} {f(avg_col):>6s} {f(avg_dist):>10s} "
                  f"{f(avg_time):>9s} {f(avg_d2g):>8s} | {f(avg_pr, 3):>10s} {f(avg_jf, 4):>8s} {f(avg_fps):>6s}")

    # Overall row
    print(f"{'-' * 105}")
    overall_d2g = np.mean(overall_d2g_list) if overall_d2g_list else None
    overall_pr = np.mean(overall_pr_list) if overall_pr_list else None
    overall_jf = np.mean(overall_jf_list) if overall_jf_list else None
    overall_fps = np.mean(overall_fps_list) if overall_fps_list else None

    r_str = f"{overall_reached}/{overall_n}"
    print(f"{'':>5s} {'Overall Avg':>14s} | {r_str:>8s} {'':>6s} {'':>10s} "
          f"{'':>9s} {f(overall_d2g):>8s} | {f(overall_pr, 3):>10s} {f(overall_jf, 4):>8s} {f(overall_fps):>6s}")

    print(f"{'=' * 110}")
    print(f"* Dg = distance to goal (failed trials only). Path Ratio = trajectory / Euclidean distance. Jerk/f = avg jerk per frame.")


# ══════════════════════════════════════════════════════
#  CSV export
# ══════════════════════════════════════════════════════
def export_csv(results, base_dir):
    fields = ["env", "method", "scenario", "voxel_size", "trial",
              "reached", "d2g", "collisions", "distance", "time",
              "path_ratio", "jerk", "jerk_per_frame", "fps", "n_frames"]

    for env in results:
        csv_path = os.path.join(base_dir, env, "results_summary.csv")
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for method in sorted(results[env].keys()):
                for (vs, sc), trials in sorted(results[env][method].items()):
                    for t in trials:
                        writer.writerow({
                            "env": env, "method": method,
                            "scenario": sc, "voxel_size": vs,
                            "trial": t["file"], "reached": t["reached"],
                            "d2g": t["d2g"], "collisions": t["collisions"],
                            "distance": t["distance"], "time": t["time"],
                            "path_ratio": t["path_ratio"], "jerk": t["jerk"],
                            "jerk_per_frame": t["jerk_per_frame"],
                            "fps": t["fps"], "n_frames": t["n_frames"],
                        })
        print(f"\n  CSV: {csv_path}")


# ══════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", type=str, default="/catkin_ws/src/results")
    args = parser.parse_args()

    print("=" * 60)
    print("  Flight Experiment Analyzer")
    print("  Output: TABLE 1-11 (report format)")
    print("=" * 60)

    results = scan_results(args.dir)

    if not results:
        print("  No results found. Check --dir path.")
        return

    # Fixed tables
    print_table1()
    print_table2()

    # Table 3: Overall
    print_table3(results)

    # Tables 4-11: Detailed
    env_order = ["Blocks", "AirSimNH"]
    method_order = [
        "a_star_airsim_depth", "a_star_estimated_depth",
        "rrt_star_airsim_depth", "rrt_star_estimated_depth",
    ]

    for method_key in method_order:
        if method_key not in TABLE_MAP:
            continue
        label, blocks_num, nh_num = TABLE_MAP[method_key]

        for env, tnum in [("Blocks", blocks_num), ("AirSimNH", nh_num)]:
            if env in results and method_key in results[env]:
                print_detail_table(tnum, label, env, results[env][method_key])
            else:
                print(f"\n  TABLE {tnum}: {label} / {env} — no data")

    # CSV export
    export_csv(results, args.dir)


if __name__ == "__main__":
    main()