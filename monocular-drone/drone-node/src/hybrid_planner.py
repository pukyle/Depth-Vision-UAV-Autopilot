"""
Hybrid planner: routes between A* and Informed-RRT*.
After global planning, applies APF path smoothing.
"""

import nav_config as cfg
from a_star import a_star_3d
from informed_rrt_star import informed_rrt_star_3d, reset_replan_counter
from apf_dwa_local_planner import get_smoother


def _get_height_restriction():
    from a_star import tol as a_star_tol
    return a_star_tol


def hybrid_plan_3d(grid, start, goal):
    """
    1. Run global planner (A* or RRT*)
    2. Apply APF smoothing
    3. Return (path, is_unf)
    """
    planner = getattr(cfg, 'planner_type', 'a_star')

    if planner == 'a_star':
        path, is_unf = a_star_3d(grid, start, goal)

    elif planner == 'rrt_star':
        try:
            path, is_unf = informed_rrt_star_3d(grid, start, goal,
                                                 max_iter=500, step_size=2.5)
        except Exception as e:
            print(f"[Planner] RRT* error: {e}, falling back to A*")
            path, is_unf = a_star_3d(grid, start, goal)
    else:
        print(f"[Planner] Unknown '{planner}', using A*")
        path, is_unf = a_star_3d(grid, start, goal)

    # Apply APF smoothing if we have a path
    if path and len(path) > 2:
        try:
            smoother = get_smoother()
            height_restr = _get_height_restriction()
            smoothed = smoother.smooth_path(grid, path, height_restr)
            if smoothed and len(smoothed) >= 2:
                path = smoothed
        except Exception as e:
            print(f"[Smoother] APF failed: {e}, using raw path")

    return path, is_unf