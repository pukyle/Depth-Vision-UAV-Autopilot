import numpy as np
import nav_config as cfg

from a_star import a_star_3d
from informed_rrt_star import informed_rrt_star_3d
from apf_dwa_local_planner import APFDWALocalPlanner


# Singleton local planner instance (shared across calls)
_local_planner = None

def get_local_planner():
    global _local_planner
    if _local_planner is None:
        _local_planner = APFDWALocalPlanner()
    return _local_planner


def hybrid_plan_3d(grid, start, goal):
    """
    Global path planner router.
    Returns (path, is_unf) — path is list of integer voxel tuples.
    """
    current_planner = getattr(cfg, 'planner_type', 'a_star')

    if current_planner == 'a_star':
        print(f"[Planner] Using A*: {start} -> {goal}")
        path, is_unf = a_star_3d(grid, start, goal)

    elif current_planner == 'rrt_star':
        print(f"[Planner] Using Informed-RRT*: {start} -> {goal}")
        try:
            path, is_unf = informed_rrt_star_3d(grid, start, goal,
                                                 max_iter=2500, step_size=2.5)
        except Exception as e:
            print(f"[Planner] RRT* error: {e}, falling back to A*")
            path, is_unf = a_star_3d(grid, start, goal)
    else:
        print(f"[Planner] Unknown '{current_planner}', using A*")
        path, is_unf = a_star_3d(grid, start, goal)

    return path, is_unf


def compute_local_velocity(grid, current_pos_vox, current_vel_vox,
                           global_path_vox, goal_pos_vox, is_colliding=False):
    """
    Compute a velocity command using APF+DWA local planner.
    
    Called from mapper_nav_ros.py each step when using RRT* planner.
    
    Args:
        grid: 3D voxel occupancy grid
        current_pos_vox: drone position as float array in voxel coords
        current_vel_vox: drone velocity as float array in voxel coords
        global_path_vox: global planned path (list of voxel tuples)
        goal_pos_vox: final goal in voxel coords
        is_colliding: True if drone is currently in collision
        
    Returns:
        velocity: np.array([vx, vy, vz]) in voxel coordinates,
                  or None if local planner cannot help (use global path directly)
    """
    lp = get_local_planner()

    if is_colliding:
        # Emergency: compute escape velocity, ignore global path
        print("[LocalPlanner] Collision detected — computing escape velocity")
        vel = lp.compute_escape_velocity(grid, current_pos_vox, current_vel_vox,
                                          goal_pos_vox)
        if np.linalg.norm(vel) > 0.1:
            return vel
        return None

    if not global_path_vox or len(global_path_vox) < 2:
        return None

    # Select sub-goal from global path
    subgoal = lp.select_subgoal(global_path_vox, current_pos_vox)
    if subgoal is None:
        return None

    vel = lp.compute_velocity(grid, current_pos_vox, current_vel_vox,
                               subgoal, global_path_vox)

    if np.linalg.norm(vel) < 0.05:
        return None  # No good velocity found, fall back to path following

    return vel