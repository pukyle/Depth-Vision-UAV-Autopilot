import numpy as np
import nav_config as cfg
from a_star import v_empty, is_inside, skip_dangerous_blocks2


class APFDWALocalPlanner:
    """
    Local planner using Artificial Potential Field + Dynamic Window Approach.
    
    Role in the system:
      - Global planner (A*/RRT*) produces a voxel-grid path
      - This local planner picks a sub-goal from that path and computes
        a safe velocity command that avoids nearby obstacles in real-time
    
    Integration with mapper_nav_ros.py:
      - Called every step with the current voxel grid, drone position, 
        velocity, and the global path
      - Returns a velocity vector (vx, vy, vz) in MAP coordinates
      - The caller converts this to AirSim world coordinates for execution
    """

    def __init__(self):
        # --- Kinematic Constraints ---
        self.max_speed_xy = cfg.speed
        self.max_speed_z = 0.8          # Strict Z limit to prevent climbing
        self.max_accel = 2.5            # Conservative acceleration

        # --- DWA Parameters ---
        self.dt = 0.4                   # Time step for velocity sampling
        self.predict_time = 0.6         # Prediction horizon (seconds)
        self.v_samples = 7              # Samples per XY axis
        self.vz_samples = 3             # Samples for Z axis

        # --- Scoring Weights ---
        self.weight_heading = 4.0       # Alignment with sub-goal direction
        self.weight_clearance = 2.5     # Obstacle avoidance (INCREASED for NH)
        self.weight_velocity = 1.0      # Forward progress reward
        self.weight_path = 2.0          # Staying close to global path

        # --- Safety ---
        self.safe_radius = 3            # Voxel search radius for obstacles
        self.lookahead_voxels = 8       # How far ahead on global path to pick sub-goal

    def select_subgoal(self, global_path_vox, current_pos_vox):
        """
        Pick a sub-goal from the global path that is ahead of the drone.
        Returns a voxel coordinate (x, y, z).
        """
        if not global_path_vox or len(global_path_vox) == 0:
            return None

        cur = np.array(current_pos_vox, dtype=float)

        # Find closest point on path
        min_dist = float('inf')
        min_idx = 0
        for i, p in enumerate(global_path_vox):
            d = np.linalg.norm(cur - np.array(p))
            if d < min_dist:
                min_dist = d
                min_idx = i

        # Pick lookahead point ahead of closest
        target_idx = min(min_idx + self.lookahead_voxels, len(global_path_vox) - 1)
        return global_path_vox[target_idx]

    def get_dynamic_window(self, current_vel):
        """Calculate allowable velocity range from current velocity."""
        dw = []
        for i, (max_s, samples) in enumerate([
            (self.max_speed_xy, self.v_samples),
            (self.max_speed_xy, self.v_samples),
            (self.max_speed_z, self.vz_samples)
        ]):
            vmin = max(-max_s, current_vel[i] - self.max_accel * self.dt)
            vmax = min(max_s, current_vel[i] + self.max_accel * self.dt)
            dw.append((vmin, vmax))
        return dw

    def calculate_clearance(self, grid, predict_pos):
        """
        APF repulsion: scan neighborhood of predicted position.
        Returns clearance score (higher = safer).
        """
        px = int(round(predict_pos[0]))
        py = int(round(predict_pos[1]))
        pz = int(round(predict_pos[2]))

        # Fatal: outside grid or occupied
        if not is_inside((px, py, pz), grid.shape) or not v_empty(grid[px, py, pz]):
            return -float('inf')

        # Danger zone: adjacent to obstacle
        if skip_dangerous_blocks2((px, py, pz), grid):
            return -50.0

        # Scan for nearest obstacle
        min_dist = float('inf')
        sr = self.safe_radius
        for dx in range(-sr, sr + 1):
            for dy in range(-sr, sr + 1):
                for dz in range(-1, 2):
                    nx, ny, nz = px + dx, py + dy, pz + dz
                    if is_inside((nx, ny, nz), grid.shape) and not v_empty(grid[nx, ny, nz]):
                        dist = np.sqrt(dx*dx + dy*dy + dz*dz)
                        if dist < min_dist:
                            min_dist = dist

        return min(min_dist, float(self.safe_radius))

    def compute_velocity(self, grid, current_pos_vox, current_vel_vox,
                         target_pos_vox, global_path_vox=None):
        """
        Core DWA+APF computation.
        
        Args:
            grid: 3D voxel occupancy grid
            current_pos_vox: drone position in voxel coordinates (float array)
            current_vel_vox: drone velocity in voxel coordinates (float array)
            target_pos_vox: sub-goal position in voxel coordinates
            global_path_vox: full global path for path-following score (optional)
            
        Returns:
            best_velocity: (vx, vy, vz) in voxel coordinates
        """
        dw = self.get_dynamic_window(current_vel_vox)
        current_pos = np.array(current_pos_vox, dtype=float)
        target_pos = np.array(target_pos_vox, dtype=float)

        vx_space = np.linspace(dw[0][0], dw[0][1], self.v_samples)
        vy_space = np.linspace(dw[1][0], dw[1][1], self.v_samples)
        vz_space = np.linspace(dw[2][0], dw[2][1], self.vz_samples)

        # Target direction (APF attractive force)
        target_vec = target_pos - current_pos
        target_dist = np.linalg.norm(target_vec)
        target_dir = target_vec / target_dist if target_dist > 0 else np.zeros(3)

        # Build path set for proximity scoring
        path_set = None
        if global_path_vox and len(global_path_vox) > 2:
            path_set = np.array(global_path_vox[:50], dtype=float)  # Cap for performance

        best_velocity = np.array([0.0, 0.0, 0.0])
        max_score = -float('inf')

        for vx in vx_space:
            for vy in vy_space:
                for vz in vz_space:
                    v_sample = np.array([vx, vy, vz])
                    v_mag = np.linalg.norm(v_sample)

                    # Predict future position
                    predict_pos = current_pos + v_sample * self.predict_time

                    # 1. Clearance score (APF repulsion)
                    clearance = self.calculate_clearance(grid, predict_pos)
                    if clearance == -float('inf'):
                        continue

                    # 2. Heading score (APF attraction)
                    if v_mag > 0.1 and target_dist > 0:
                        heading = np.dot(v_sample / v_mag, target_dir)
                    else:
                        heading = 0.0

                    # 3. Path proximity score
                    path_score = 0.0
                    if path_set is not None:
                        dists = np.linalg.norm(path_set - predict_pos, axis=1)
                        path_score = -np.min(dists)  # Penalize distance from path

                    # 4. Total score
                    total = (self.weight_heading * heading +
                             self.weight_clearance * clearance +
                             self.weight_velocity * v_mag +
                             self.weight_path * path_score)

                    if total > max_score:
                        max_score = total
                        best_velocity = v_sample

        return best_velocity

    def compute_escape_velocity(self, grid, current_pos_vox, current_vel_vox,
                                 goal_pos_vox):
        """
        Emergency velocity computation when drone is colliding.
        Uses wider search, more aggressive avoidance, lower speed.
        """
        # Temporarily boost clearance weight and reduce speed
        old_clearance = self.weight_clearance
        old_heading = self.weight_heading
        old_speed = self.max_speed_xy

        self.weight_clearance = 5.0
        self.weight_heading = 2.0
        self.max_speed_xy = cfg.speed * 0.6  # Slower for safety

        vel = self.compute_velocity(grid, current_pos_vox, current_vel_vox,
                                    goal_pos_vox, global_path_vox=None)

        # Restore
        self.weight_clearance = old_clearance
        self.weight_heading = old_heading
        self.max_speed_xy = old_speed

        return vel