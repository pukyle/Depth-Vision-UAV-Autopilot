"""
JSON + CSV Fixer + Analyzer

Step 1: Scan all trial JSON files, fix gloal_reached based on floor(d2g) <= 5
Step 2: Fix results_summary.csv (floor d2g, update reached)
Step 3: Re-analyze and print paper-style tables
"""

import json, csv, os, sys, math, argparse, glob
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
VOXELS = ['0.5', '1.0', '2.0']
GOAL_THR = 5


# ══════════════════════════════════════════════════════
#  Step 1: Fix all JSON files
# ══════════════════════════════════════════════════════
def fix_json_files(results_dir):
    """Walk through all trial JSON files and fix gloal_reached."""
    print(f"\n  === Step 1: Fixing JSON files ===")
    fixed_total = 0
    scanned_total = 0

    for env_name, scenarios in SCENARIOS.items():
        env_path = os.path.join(results_dir, env_name)
        if not os.path.isdir(env_path):
            continue

        for method_dir in sorted(os.listdir(env_path)):
            method_path = os.path.join(env_path, method_dir)
            if not os.path.isdir(method_path):
                continue

            for combo_dir in sorted(os.listdir(method_path)):
                combo_path = os.path.join(method_path, combo_dir)
                if not os.path.isdir(combo_path):
                    continue

                # Parse scenario from directory name: S1_V0.5 -> scenario=1
                try:
                    parts = combo_dir.split('_')
                    sc = int(parts[0][1:])  # S1 -> 1
                except:
                    continue

                if sc not in scenarios:
                    continue

                goal = np.array(scenarios[sc]["goal"])

                # Process each trial JSON
                for json_file in sorted(glob.glob(os.path.join(combo_path, "*.json"))):
                    scanned_total += 1
                    fixed = fix_single_json(json_file, goal)
                    if fixed:
                        fixed_total += 1

    print(f"  Scanned: {scanned_total} JSON files")
    print(f"  Fixed:   {fixed_total} (gloal_reached False -> True)")


def fix_single_json(filepath, goal):
    """Fix a single trial JSON. Returns True if modified."""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except:
        return False

    logs = data.get("log", [])
    if not logs:
        return False

    modified = False

    # Check the last entry
    last = logs[-1]
    last_pos = np.array(last.get("real_pos", [0, 0, 0]))
    d2g_raw = np.linalg.norm(last_pos - goal)
    d2g_floored = math.floor(d2g_raw)

    # If floor(d2g) <= 5 but gloal_reached is False, fix it
    if d2g_floored <= GOAL_THR and not last.get("gloal_reached", False):
        # Find the last entry that has gloal_reached=False and fix it
        # The JSON might have gloal_reached set in multiple entries
        for entry in logs:
            if "gloal_reached" in entry:
                pass  # Don't change intermediate entries

        # Fix the last entry
        logs[-1]["gloal_reached"] = True
        modified = True

    # Also check: some JSONs have gloal_reached=True followed by gloal_reached=False
    # because of the code structure in mapper_nav_ros.py (line 426 always writes False)
    # Fix: if ANY entry has d2g <= 5 from goal, the last entry should be True
    for i, entry in enumerate(logs):
        pos = np.array(entry.get("real_pos", [0, 0, 0]))
        d = math.floor(np.linalg.norm(pos - goal))
        if d <= GOAL_THR and i == len(logs) - 1:
            if not entry.get("gloal_reached", False):
                entry["gloal_reached"] = True
                modified = True

    if modified:
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f)
        basename = os.path.basename(filepath)
        parent = os.path.basename(os.path.dirname(filepath))
        print(f"    Fixed: {parent}/{basename} (d2g={d2g_raw:.2f} -> floor={d2g_floored})")

    return modified


# ══════════════════════════════════════════════════════
#  Step 2: Fix CSV
# ══════════════════════════════════════════════════════
def fix_csv(csv_path):
    """Fix d2g and reached in CSV."""
    print(f"\n  === Step 2: Fixing CSV ===")

    if not os.path.exists(csv_path):
        print(f"  [SKIP] CSV not found: {csv_path}")
        return None

    with open(csv_path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        rows = list(reader)

    if not rows:
        print(f"  [SKIP] CSV is empty")
        return None

    fixed_count = 0
    for row in rows:
        try:
            d2g_raw = float(row['d2g'])
        except (ValueError, KeyError):
            continue

        d2g_floored = math.floor(d2g_raw)
        row['d2g'] = str(d2g_floored)

        if d2g_floored <= GOAL_THR and row['reached'] == 'False':
            row['reached'] = 'True'
            fixed_count += 1

    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"  Fixed {fixed_count} rows (False -> True)")
    print(f"  Saved: {csv_path}")
    return rows


# ══════════════════════════════════════════════════════
#  Step 3: Analyze from corrected CSV
# ══════════════════════════════════════════════════════
def analyze(rows):
    """Print paper-style tables from corrected data."""
    methods = {}
    for row in rows:
        m = row.get('method', '?')
        if m not in methods:
            methods[m] = []
        methods[m].append(row)

    env = rows[0].get('env', '?')

    # Table 4
    print(f"\n{'=' * 80}")
    print(f"  Table 4 — Overall Results: {env}")
    print(f"{'=' * 80}")
    print(f"  {'Method':<35s} {'Reached Goal':>15s} {'Distance to Goal':>22s}")
    print(f"  {'-' * 75}")

    for m in sorted(methods.keys()):
        t = methods[m]
        n = len(t)
        r = sum(1 for x in t if x['reached'] == 'True')
        f = [x for x in t if x['reached'] == 'False']
        d = np.mean([float(x['d2g']) for x in f]) if f else 0
        rs = f"{r} of {n}"
        ds = f"{d:.2f} ({len(f)} unsuccessful)" if f else "-"
        print(f"  {m:<35s} {rs:>15s} {ds:>22s}")
    print(f"{'=' * 80}")

    # Table 5/6 per method
    for m in sorted(methods.keys()):
        t = methods[m]
        print(f"\n{'=' * 95}")
        print(f"  Table 5/6 — {env}: {m}")
        print(f"{'=' * 95}")
        print(f"  {'V':>5s} {'S':>2s} | {'Reached':>8s} | {'Cols':>6s} | "
              f"{'Distance':>10s} {'Time':>8s} | {'Dist2Goal':>10s}")
        print(f"  {'-' * 85}")

        for vs in VOXELS:
            for sc in ['1', '2', '3']:
                g = [x for x in t if str(x.get('voxel_size',''))==vs and str(x.get('scenario',''))==sc]
                if not g:
                    print(f"  {vs:>5} {sc:>2} | {'(no data)':>8s}")
                    continue
                reached = [x for x in g if x['reached']=='True']
                failed = [x for x in g if x['reached']=='False']
                nr = len(reached)
                ac = f"{np.mean([float(x['collisions']) for x in reached]):.2f}" if reached else "-"
                ad = f"{np.mean([float(x['distance']) for x in reached]):.2f}" if reached else "-"
                at = f"{np.mean([float(x['time']) for x in reached]):.2f}" if reached else "-"
                ag = f"{np.mean([float(x['d2g']) for x in failed]):.2f}" if failed else "-"
                print(f"  {vs:>5} {sc:>2} | {nr:>8} | {ac:>6s} | {ad:>10s} {at:>8s} | {ag:>10s}")
        print(f"{'=' * 95}")

    # Table E per method
    for m in sorted(methods.keys()):
        t = methods[m]
        print(f"\n{'=' * 100}")
        print(f"  Table E — Extended Metrics: {env}: {m}")
        print(f"{'=' * 100}")
        print(f"  {'V':>5s} {'S':>2s} | {'Reached':>4s}/{'N':>2s} {'Rate':>6s} | "
              f"{'PathRatio':>10s} | {'MAPE(%)':>8s} | "
              f"{'Jerk':>8s} {'Jerk/f':>8s} | {'CompTime':>8s} {'FPS':>6s}")
        print(f"  {'-' * 95}")

        all_t = []
        for vs in VOXELS:
            for sc in ['1', '2', '3']:
                g = [x for x in t if str(x.get('voxel_size',''))==vs and str(x.get('scenario',''))==sc]
                if not g:
                    print(f"  {vs:>5} {sc:>2} | {'(no data)':>12s}")
                    continue
                all_t.extend(g)
                n = len(g)
                reached = [x for x in g if x['reached']=='True']
                nr = len(reached)
                rate = nr/n*100
                pr = [float(x['path_ratio']) for x in reached if float(x.get('path_ratio',0))>0]
                avg_pr = np.mean(pr) if pr else None
                mp = [float(x['mape']) for x in g if x.get('mape') and x['mape']!='']
                avg_mp = np.mean(mp) if mp else None
                aj = np.mean([float(x['jerk']) for x in g])
                ajf = np.mean([float(x['jerk_per_frame']) for x in g])
                act = np.mean([float(x['comp_time']) for x in g])
                afps = np.mean([float(x['fps']) for x in g])
                def f(v,d=2):
                    return "-" if v is None else f"{v:.{d}f}"
                print(f"  {vs:>5} {sc:>2} | {nr:>4}/{n:>2} {rate:>5.1f}% | "
                      f"{f(avg_pr,3):>10s} | {f(avg_mp,2):>8s} | "
                      f"{f(aj,2):>8s} {f(ajf,4):>8s} | {f(act,3):>8s} {f(afps,2):>6s}")

        if all_t:
            n=len(all_t); reached=[x for x in all_t if x['reached']=='True']; nr=len(reached)
            rate=nr/n*100
            pr=[float(x['path_ratio']) for x in reached if float(x.get('path_ratio',0))>0]
            avg_pr=np.mean(pr) if pr else None
            mp=[float(x['mape']) for x in all_t if x.get('mape') and x['mape']!='']
            avg_mp=np.mean(mp) if mp else None
            aj=np.mean([float(x['jerk']) for x in all_t])
            ajf=np.mean([float(x['jerk_per_frame']) for x in all_t])
            act=np.mean([float(x['comp_time']) for x in all_t])
            afps=np.mean([float(x['fps']) for x in all_t])
            def f(v,d=2):
                return "-" if v is None else f"{v:.{d}f}"
            print(f"  {'-'*95}")
            print(f"  {'ALL':>5} {'':>2} | {nr:>4}/{n:>2} {rate:>5.1f}% | "
                  f"{f(avg_pr,3):>10s} | {f(avg_mp,2):>8s} | "
                  f"{f(aj,2):>8s} {f(ajf,4):>8s} | {f(act,3):>8s} {f(afps,2):>6s}")
        print(f"{'=' * 100}")

    # Table A1
    labels = sorted(methods.keys())
    print(f"\n{'=' * 85}")
    print(f"  Table A1 — FPS: {env}")
    print(f"{'=' * 85}")
    h = f"  {'Voxel':>8s}"
    for l in labels:
        s = l.replace("_airsim_depth","(AT)").replace("_estimated_depth","(Est)")
        h += f"  {s:>18s}"
    print(h)
    print(f"  {'-'*80}")
    for vs in VOXELS:
        row = f"  {vs:>8}"
        for l in labels:
            tt = [x for x in methods[l] if str(x.get('voxel_size',''))==vs]
            if tt:
                row += f"  {np.mean([float(x['fps']) for x in tt]):>18.2f}"
            else:
                row += f"  {'-':>18s}"
        print(row)
    row = f"  {'Overall':>8}"
    for l in labels:
        row += f"  {np.mean([float(x['fps']) for x in methods[l]]):>18.2f}"
    print(row)
    print(f"{'=' * 85}")


def main():
    parser = argparse.ArgumentParser(description="JSON + CSV Fixer + Analyzer")
    parser.add_argument("--dir", type=str, default="/catkin_ws/src/results",
                        help="Results base directory")
    parser.add_argument("--csv", type=str, default=None,
                        help="CSV path (default: {dir}/{env}/results_summary.csv)")
    args = parser.parse_args()

    print("=" * 60)
    print("  JSON + CSV Fixer + Analyzer")
    print(f"  Rule: floor(d2g) <= {GOAL_THR}m = success")
    print("=" * 60)

    # Step 1: Fix all JSON files
    fix_json_files(args.dir)

    # Step 2: Fix CSV and analyze
    # Find all CSV files
    for env_name in sorted(os.listdir(args.dir)):
        env_path = os.path.join(args.dir, env_name)
        if not os.path.isdir(env_path):
            continue

        csv_path = args.csv or os.path.join(env_path, "results_summary.csv")
        if os.path.exists(csv_path):
            rows = fix_csv(csv_path)
            if rows:
                print(f"\n  === Step 3: Analysis ===")
                analyze(rows)
        else:
            print(f"\n  [SKIP] No CSV for {env_name}")
            print(f"  Run analyze_metrics.py first to generate CSV")


if __name__ == "__main__":
    main()