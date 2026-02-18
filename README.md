# Autonomous UAV Navigation with Informed-RRT* and Monocular Depth Estimation

[![License: BSD-3-Clause](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](LICENSE)
![Fork Status](https://img.shields.io/badge/Forked_from-jonasctrl%2Fmonocular--slam--drone-9cf)

> **Note:** This is a modified version of the [Monocular-SLAM-Drone](https://github.com/jonasctrl/monocular-slam-drone) framework.

## About the Project

This project extends the original **Monocular-SLAM-Drone** framework by integrating an **Informed-RRT*  **(Rapidly-exploring Random Tree*)** path planner, replacing the original A* algorithm.

While the original framework provides excellent real-time monocular depth estimation and 3D voxel occupancy grid generation, this fork focuses on optimizing path planning efficiency in complex, high-dimensional 3D environments.

### Key Modifications
* **Path Planning Algorithm:** Replaced the grid-based **A*** planner with sampling-based **Informed-RRT***.
    * *Why?* Informed-RRT* offers better performance in sparse environments and converges faster to an optimal solution by limiting the search to an ellipsoidal subset.
* **Navigation Module (NM):** Refactored the ROS node to support continuous space trajectory generation instead of discrete grid pathfinding.

| Module                          | Description                                                                                                                                                                                          |
| ------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Depth Estimation Module (DEM)** | (Same as original) Estimates depth images from the RGB camera feed...                                                                                    |
| **Mapper Module (MM)** | (Same as original) Builds and iteratively updates the occupancy map-based 3D environment...                                                              |
| **Navigation Module (NM)** | **[UPDATED]** Finds viable path trajectories using the **Informed-RRT*** algorithm. Optimizes path length and smoothness compared to the original A* implementation. |

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
