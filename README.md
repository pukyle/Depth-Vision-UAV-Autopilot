# Autonomous UAV Navigation with Informed-RRT* and Monocular Depth Estimation

[![License: BSD-3-Clause](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](LICENSE)
![Fork Status](https://img.shields.io/badge/Forked_from-jonasctrl%2Fmonocular--slam--drone-9cf)

> This is a modified version of the [Monocular-SLAM-Drone](https://github.com/jonasctrl/monocular-slam-drone) framework.

## About the Project

This project extends the original **Monocular-SLAM-Drone** framework by integrating an **APF-guided Informed-RRT*** path planner with **Artificial Potential Field (APF) path smoothing**, replacing the original A* algorithm. All experiments are conducted in Microsoft AirSim on Unreal Engine.

---

### Why Informed-RRT* with APF?

Before deploying to AirSim, we conducted **algorithm-level simulations** comparing three RRT* variants integrated with DWA (Dynamic Window Approach) as a local controller:

| Variant | Description |
|---------|-------------|
| RRT* | Basic rapidly-exploring random tree with asymptotic optimality |
| Informed-RRT* | Limits sampling to an ellipsoidal subset, improving convergence |
| APF-guided Informed-RRT* | Adds potential field bias to concentrate samples in obstacle-free corridors |

<p align="center">
  <img src="https://github.com/user-attachments/assets/62193352-5682-4b0c-a840-9b127cc3745e" width="100%" alt="Three RRT* variants with DWA trajectory tracking"/>
</p>
<p align="center"><sub>Fig 1. Route comparison — APF-guided Informed-RRT* produces the most concentrated node distribution along feasible corridors.</sub></p>
<table>
  <tr>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/995d72ad-df47-4744-9a3c-e7bf7a115e86" width="100%" alt="Smoothness comparison"/>
      <br><sub>Fig 2. Mean turning angle — DWA reduces oscillation across all variants.</sub>
    </td>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/b5b06011-81b1-483e-b398-905e10ae1828" width="100%" alt="Path length comparison"/>
      <br><sub>Fig 3. Path length — Informed variants produce shorter paths than basic RRT*.</sub>
    </td>
  </tr>
</table>

<p align="center">
  <img src="https://github.com/user-attachments/assets/2967121e-4b0f-4e8c-ae0b-d9cf06c4e683" width="50%" alt="Planning time comparison"/>
</p>
<p align="center"><sub>Fig 4. Planning time — Informed-RRT* reduces planning time by limiting the sampling domain.</sub></p>

These results confirmed that:
- **Informed-RRT*** reduces sampling range and improves search efficiency over basic RRT*
- **APF guidance** concentrates nodes in feasible passages, reducing wasted exploration
- **DWA** provides smooth local trajectory tracking regardless of global planner choice
- Global path quality **directly impacts** local control stability

Based on these findings, we implemented APF-guided Informed-RRT* within the AirSim framework.

---

### Key Modifications

- **Informed RRT_star Planner** — A* runs first as baseline; RRT* only optimizes if path is ≥5% shorter. KD-Tree accelerated, 0.5s time budget, 500 iteration cap. First 3 replans use RRT*; subsequent replans fall back to A* for reactive speed.
- **APF Path Smoothing** — Pushes waypoints away from obstacles via repulsive potential fields. Does not interfere with `moveOnPathAsync`.
- **Depth Source Toggle** — `--use_airsim_depth` bypasses Depth Anything V2 for controlled ablation studies.
- **Automated Pipeline** — `run_batch_test.py` and `analyze_metrics.py` for reproducible batch experiments with paper-aligned parameters.

| Module | Description |
|--------|-------------|
| **DEM** | *(Unchanged)* Depth Anything V2 monocular depth estimation |
| **MM** | *(Unchanged)* 3D occupancy grid with Bresenham raycast |
| **NM** | **[Updated]** Hybrid A*/RRT* planner + APF smoothing + depth source toggle |

---

## Experimental Results

90 flights per method (3 scenarios × 3 voxel sizes × 10 trials) in each environment.

### Ground Truth Depth

<table>
  <tr>
    <th></th>
    <th colspan="4" align="center">Blocks</th>
    <th colspan="4" align="center">AirSimNH</th>
  </tr>
  <tr>
    <th>Method</th>
    <th>Success</th><th>Path Ratio</th><th>Jerk/f</th><th>FPS</th>
    <th>Success</th><th>Path Ratio</th><th>Jerk/f</th><th>FPS</th>
  </tr>
  <tr>
    <td>A*</td>
    <td>69/90 (76.7%)</td><td>1.117</td><td>0.922</td><td>4.75</td>
    <td>36/90 (40.0%)</td><td>1.921</td><td>1.880</td><td>3.96</td>
  </tr>
  <tr>
    <td><b>RRT*</b></td>
    <td><b>73/90 (81.1%)</b></td><td><b>1.115</b></td><td>0.973</td><td>4.73</td>
    <td><b>42/90 (46.7%)</b></td><td>1.943</td><td>2.018</td><td>3.90</td>
  </tr>
</table>

RRT* outperforms A* by **+4.4%** in Blocks and **+6.7%** in AirSimNH with no FPS loss.

### Impact of Depth Estimation

| Method | Blocks (GT) | Blocks (Est.) | AirSimNH (GT) | AirSimNH (Est.) |
|--------|:-----------:|:-------------:|:-------------:|:---------------:|
| A* | 76.7% | 44.4% | 40.0% | 0.0% |
| RRT* | 81.1% | 46.7% | 46.7% | 0.0% |

<sub>Depth estimation reduces FPS from ~4.7 to ~1.4 (−70%), causing stale occupancy maps and phantom obstacles.</sub>

> Full results (TABLE 1–11) and raw trial data are available in the [Releases](../../releases) section.

---

## Modified Files

| File | Status | Description |
|------|--------|-------------|
| `informed_rrt_star.py` | **New** | Informed-RRT* with A* baseline, KD-Tree, time budget |
| `apf_dwa_local_planner.py` | **Rewrite** | Velocity controller → path smoother |
| `hybrid_planner.py` | **Rewrite** | Planner router + APF integration |
| `mapper_nav_ros.py` | **Modified** | Depth toggle, replan counter reset |
| `nav_config.py` | **Modified** | `--use_airsim_depth`, `--planner_type` flags |
| `run_batch_test.py` | **New** | Automated batch test runner |
| `analyze_metrics.py` | **New** | TABLE 1–11 analysis + CSV export |
| `fix_and_analyze.py` | **New** | JSON/CSV goal threshold correction |

<sub>All other files (a_star.py, mapper.py, depth.py, utils.py, data_logger.py, depth_train_v2_*.py) remain unchanged.</sub>

---

## Usage

### Single Flight
```bash
# A* with ground-truth depth
python3 mapper_nav_ros.py --planner_type a_star --use_airsim_depth --goal_off 70 150 3 --exit_on_goal

# RRT* with depth estimation
python3 mapper_nav_ros.py --planner_type rrt_star --use_rgb_imaging --goal_off 70 150 3 --exit_on_goal
```

### Batch Experiments
```bash
python3 run_batch_test.py
# Modes: Custom | Run EVERYTHING | Run FULL EXPERIMENT (both depth sources)
```

### Analysis
```bash
python3 analyze_metrics.py --dir /catkin_ws/src/results
python3 fix_and_analyze.py --dir /catkin_ws/src/results
```

---

## Experimental Data

Raw data available in [Releases](../../releases):
- `results_Blocks.zip` — 360 trial JSONs
- `results_AirSimNH.zip` — 360 trial JSONs

## Hardware

| Component | Specification |
|-----------|--------------|
| CPU | 13th Gen Intel i7-13620H (12C, 2.40 GHz) |
| GPU | NVIDIA RTX 5060 Laptop (8GB GDDR6) |
| RAM | 32 GB DDR5 |
| OS | Windows 11 + Docker (Ubuntu 20.04) |

---

## Acknowledgements & Citation

Built upon the work by Jonas Gaigalas et al.:

> [A Framework for Autonomous UAV Navigation Based on Monocular Depth Estimation](https://www.mdpi.com/2504-446X/9/4/236)

```bibtex
@article{monocular-slam-drone,
  title={A Framework for Autonomous UAV Navigation Based on Monocular Depth Estimation},
  author={Gaigalas Jonas and Perkauskas Linas and Gricius Henrikas and Kanapickas Tomas and Kriščiūnas Andrius},
  journal={Drones},
  year={2025},
  publisher={MDPI},
}
```
