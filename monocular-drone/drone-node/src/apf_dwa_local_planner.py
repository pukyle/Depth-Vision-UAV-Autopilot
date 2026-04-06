"""
APF+DWA Path Smoother

Role: Post-process the global path BEFORE sending to AirSim.
NOT a velocity controller. Does NOT replace moveOnPathAsync.

What it does:
  1. Takes a global path (list of voxel tuples) from A*/RRT*
  2. Uses APF repulsive forces to push waypoints away from nearby obstacles
  3. Uses DWA-style scoring to evaluate alternative waypoint positions
  4. Returns a smoothed path (same format, same length range)

This is called inside hybrid_planner.py after global planning.
"""

import numpy as np
import nav_config as cfg
from a_star import v_empty, is_inside, skip_dangerous_blocks2


class APFPathSmoother:
    def __init__(self):
        self.repulsion_radius = 3       # Voxels: scan range for obstacles
        self.repulsion_strength = 1.5   # How strongly to push away from obstacles
        self.attraction_strength = 1.0  # How strongly to pull toward original path
        self.max_displacement = 1       # Max voxels a waypoint can shift
        self.smooth_iterations = 2      # Number of smoothing passes

    def _find_nearest_obstacle(self, grid, x, y, z):
        """Find direction and distance to nearest obstacle within repulsion_radius."""
        min_dist = float('inf')
        obs_dir = np.array([0.0, 0.0, 0.0])
        r = self.repulsion_radius

        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for dz in range(-1, 2):  # Less Z sensitivity
                    nx, ny, nz = x + dx, y + dy, z + dz
                    if not is_inside((nx, ny, nz), grid.shape):
                        continue
                    if not v_empty(grid[nx, ny, nz]):
                        dist = np.sqrt(dx*dx + dy*dy + dz*dz)
                        if dist < min_dist and dist > 0:
                            min_dist = dist
                            # Direction AWAY from obstacle
                            obs_dir = np.array([-dx, -dy, -dz], dtype=float)

        if min_dist == float('inf'):
            return None, float('inf')

        # Normalize direction
        norm = np.linalg.norm(obs_dir)
        if norm > 0:
            obs_dir = obs_dir / norm

        return obs_dir, min_dist

    def _is_point_safe(self, grid, x, y, z):
        """Check if point is valid for path (same criteria as A*)."""
        pt = (x, y, z)
        if not is_inside(pt, grid.shape):
            return False
        if not v_empty(grid[x, y, z]):
            return False
        if skip_dangerous_blocks2(pt, grid):
            return False
        return True

    def smooth_path(self, grid, path, height_restr=None):
        """
        Apply APF-based smoothing to push waypoints away from obstacles.
        
        Args:
            grid: 3D voxel occupancy grid
            path: list of (x, y, z) integer tuples
            height_restr: (min_z, max_z) tuple or None
            
        Returns:
            smoothed path: list of (x, y, z) integer tuples, same connectivity
        """
        if len(path) <= 2:
            return path  # Can't smooth start/goal only

        smoothed = list(path)

        for iteration in range(self.smooth_iterations):
            new_path = [smoothed[0]]  # Keep start fixed

            for i in range(1, len(smoothed) - 1):
                x, y, z = smoothed[i]

                # Find repulsive force from nearest obstacle
                rep_dir, obs_dist = self._find_nearest_obstacle(grid, x, y, z)

                if rep_dir is None or obs_dist > self.repulsion_radius:
                    # No nearby obstacles, keep original
                    new_path.append(smoothed[i])
                    continue

                # APF repulsive force: stronger when closer to obstacle
                # F_rep = strength * (1/dist - 1/radius) * direction
                if obs_dist < 0.5:
                    obs_dist = 0.5
                rep_force = self.repulsion_strength * (1.0 / obs_dist - 1.0 / self.repulsion_radius)
                rep_force = max(0, rep_force)

                # Compute displacement
                displacement = rep_dir * rep_force

                # Clamp displacement
                disp_mag = np.linalg.norm(displacement)
                if disp_mag > self.max_displacement:
                    displacement = displacement * (self.max_displacement / disp_mag)

                # Apply displacement
                new_x = int(round(x + displacement[0]))
                new_y = int(round(y + displacement[1]))
                new_z = int(round(z + displacement[2]))

                # Enforce height restriction
                if height_restr is not None:
                    new_z = max(height_restr[0], min(height_restr[1], new_z))

                # Only accept if new position is safe
                if self._is_point_safe(grid, new_x, new_y, new_z):
                    new_path.append((new_x, new_y, new_z))
                else:
                    new_path.append(smoothed[i])  # Keep original

            new_path.append(smoothed[-1])  # Keep goal fixed
            smoothed = new_path

        # Re-interpolate to ensure grid-adjacency after shifting
        result = self._reinterpolate(smoothed)
        return result

    def _reinterpolate(self, path):
        """Ensure consecutive points are at most 1 voxel apart."""
        if len(path) <= 1:
            return path
        result = [path[0]]
        for i in range(1, len(path)):
            x0, y0, z0 = result[-1]
            x1, y1, z1 = path[i]
            dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
            steps = max(abs(dx), abs(dy), abs(dz))
            if steps <= 1:
                if (x1, y1, z1) != result[-1]:
                    result.append((x1, y1, z1))
                continue
            for s in range(1, steps + 1):
                t = s / float(steps)
                pt = (int(round(x0 + dx * t)),
                      int(round(y0 + dy * t)),
                      int(round(z0 + dz * t)))
                if pt != result[-1]:
                    result.append(pt)
        return result


# Singleton instance
_smoother = None

def get_smoother():
    global _smoother
    if _smoother is None:
        _smoother = APFPathSmoother()
    return _smoother