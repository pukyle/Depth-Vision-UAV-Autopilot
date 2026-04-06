# Autonomous UAV Navigation with Informed-RRT* and Monocular Depth Estimation

[![License: BSD-3-Clause](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](LICENSE)
![Fork Status](https://img.shields.io/badge/Forked_from-jonasctrl%2Fmonocular--slam--drone-9cf)

> **Note:** This is a modified version of the [Monocular-SLAM-Drone](https://github.com/jonasctrl/monocular-slam-drone) framework.

## About the Project

This project extends the original **Monocular-SLAM-Drone** framework by integrating an **Informed-RRT*** (Rapidly-exploring Random Tree*) path planner with **APF (Artificial Potential Field) path smoothing**, replacing the original A* algorithm. All experiments are conducted in Microsoft AirSim on Unreal Engine.

While the original framework provides excellent real-time monocular depth estimation and 3D voxel occupancy grid generation, this fork focuses on optimizing path planning efficiency in complex, high-dimensional 3D environments.

### Key Modifications

* **Path Planning Algorithm:** Replaced the grid-based **A*** planner with sampling-based **Informed-RRT***.
    * Conservative design: A* always runs first as baseline; RRT* only optimizes if path is ≥5% shorter. **Never worse than A*.**
    * KD-Tree accelerated nearest-neighbor search (O(log n) vs O(n))
    * 0.5s time budget + 500 iteration cap to prevent FPS degradation
    * First 3 replans use RRT* optimization; subsequent replans use A* only for reactive speed
* **APF Path Smoothing:** Post-processing step that pushes waypoints away from obstacles using repulsive potential fields, improving obstacle clearance without interfering with AirSim's `moveOnPathAsync` movement control.
* **Depth Source Toggle:** Added `--use_airsim_depth` flag to bypass Depth Anything V2 and use AirSim ground-truth depth, enabling controlled ablation studies.
* **Automated Experiment Pipeline:** Batch testing (`run_batch_test.py`) and analysis (`analyze_metrics.py`) tools with paper-aligned parameters for reproducible experiments.

| Module | Description |
|--------|-------------|
| **Depth Estimation Module (DEM)** | (Same as original) Estimates depth images from the RGB camera feed using Depth Anything V2. |
| **Mapper Module (MM)** | (Same as original) Builds and iteratively updates the occupancy map-based 3D environment representation. |
| **Navigation Module (NM)** | **[UPDATED]** Supports both A* and Informed-RRT* via hybrid planner. APF smoothing applied to all paths. Depth source selectable via CLI. |

## Experimental Results

Full experiments conducted on **Blocks** and **AirSimNH** environments with 3 scenarios × 3 voxel sizes × 10 trials = 90 flights per method.

### Blocks Environment — Ground Truth Depth

| Method | Success Rate | Path Ratio | Jerk/f | FPS |
|--------|-------------|-----------|--------|-----|
| A* | 69/90 (76.7%) | 1.117 | 0.922 | 4.75 |
| **RRT*** | **73/90 (81.1%)** | **1.115** | 0.973 | 4.73 |

RRT* achieves **+4.4%** higher success rate than A* with comparable FPS, demonstrating that the path optimization improves navigation without computational overhead.

### AirSimNH Environment — Ground Truth Depth

| Method | Success Rate | Path Ratio | Jerk/f | FPS |
|--------|-------------|-----------|--------|-----|
| A* | 36/90 (40.0%) | 1.921 | 1.880 | 3.96 |
| **RRT*** | **42/90 (46.7%)** | 1.943 | 2.018 | 3.90 |

RRT* outperforms A* by **+6.7%** in the more complex AirSimNH environment.

### Impact of Depth Estimation

| Method | Blocks (GT) | Blocks (Est.) | AirSimNH (GT) | AirSimNH (Est.) |
|--------|------------|--------------|--------------|----------------|
| A* | 76.7% | 44.4% | 40.0% | 0.0% |
| RRT* | 81.1% | 46.7% | 46.7% | 0.0% |

Depth estimation reduces FPS from ~4.7 to ~1.4 (−70%), causing significant performance degradation due to stale occupancy maps and phantom obstacles from depth noise.

> **Full results with all 11 tables are available in the [Releases](../../releases) section.**

## Modified Files

| File | Change | Description |
|------|--------|-------------|
| `informed_rrt_star.py` | **New** | Informed-RRT* with A* baseline guarantee, KD-Tree, time budget |
| `apf_dwa_local_planner.py` | **Rewrite** | Changed from velocity controller to path smoother |
| `hybrid_planner.py` | **Rewrite** | Planner router + APF smoothing integration |
| `mapper_nav_ros.py` | **Modified** | Conditional DEM init, depth source toggle, replan counter reset |
| `nav_config.py` | **Modified** | Added `--use_airsim_depth` and `--planner_type` CLI flags |
| `run_batch_test.py` | **New** | Automated batch test runner with paper-aligned parameters |
| `analyze_metrics.py` | **New** | Paper-format analysis (TABLE 1-11) with CSV export |
| `fix_and_analyze.py` | **New** | JSON/CSV post-processing for goal threshold correction |

All other files (`a_star.py`, `mapper.py`, `depth.py`, `utils.py`, `data_logger.py`, `depth_train_v2_*.py`) remain **unchanged** from the original.

## Usage

### Running a Single Flight

```bash
# A* with AirSim ground-truth depth
python3 mapper_nav_ros.py --planner_type a_star --use_airsim_depth --goal_off 70 150 3 --exit_on_goal

# RRT* with depth estimation
python3 mapper_nav_ros.py --planner_type rrt_star --use_rgb_imaging --goal_off 70 150 3 --exit_on_goal
```

### Running Batch Experiments

```bash
python3 run_batch_test.py
# Select: Environment → Mode → Depth Source → Overwrite/Skip
# Modes:
#   1. Custom (pick planner/scenario/voxel)
#   2. Run EVERYTHING (single depth source, A* + RRT*)
#   3. Run FULL EXPERIMENT (both depth sources, then auto-analyze)
```

### Analyzing Results

```bash
# Generate TABLE 1-11 report + CSV
python3 analyze_metrics.py --dir /catkin_ws/src/results

# Fix JSON/CSV goal threshold and re-analyze
python3 fix_and_analyze.py --dir /catkin_ws/src/results
```

## Experimental Data

Raw experiment data (trial JSONs + summary CSVs) are available as zip archives in the [Releases](../../releases) section:

- `results_Blocks.zip` — 360 trial JSONs (4 methods × 90 trials)
- `results_AirSimNH.zip` — 360 trial JSONs (4 methods × 90 trials)

## Hardware

| Component | Specification |
|-----------|--------------|
| CPU | 13th Gen Intel Core i7-13620H (12 Cores, 2.40 GHz) |
| GPU | NVIDIA GeForce RTX 5060 Laptop GPU (8GB GDDR6) |
| RAM | 32.0 GB DDR5 |
| OS | Windows 11 + Docker (Ubuntu 20.04) |

## Acknowledgements & Citation

This project is built upon the excellent work by Jonas Gaigalas et al. If you use the core SLAM or Depth Estimation framework, please cite their original paper:

> **Original Paper:** [A Framework for Autonomous UAV Navigation Based on Monocular Depth Estimation](https://www.mdpi.com/2504-446X/9/4/236)

```bibtex
@article{monocular-slam-drone,
  title={A Framework for Autonomous UAV Navigation Based on Monocular Depth Estimation},
  author={Gaigalas Jonas and Perkauskas Linas and Gricius Henrikas and Kanapickas Tomas and Kriščiūnas Andrius},
  journal={Drones},
  year={2025},
  publisher={MDPI},
}
```
