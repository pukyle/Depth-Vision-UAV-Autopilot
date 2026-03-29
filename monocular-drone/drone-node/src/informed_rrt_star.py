import numpy as np
import random
import nav_config as cfg
from a_star import a_star_3d, v_empty, skip_dangerous_blocks2, is_inside


class Node:
    __slots__ = ['x', 'y', 'z', 'parent', 'cost']

    def __init__(self, x, y, z):
        self.x = int(round(x))
        self.y = int(round(y))
        self.z = int(round(z))
        self.parent = None
        self.cost = 0.0


def get_dist(n1, n2):
    return np.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2 + (n1.z - n2.z)**2)


def _is_valid_point(grid, x, y, z, safety='strict'):
    """
    Check if a point is safe to occupy.
    safety='strict':  full 3x3x3 check (same as A*, safest)
    safety='medium':  check the point + 4 cardinal XY neighbors (avoids walls, allows tight spaces)
    safety='relaxed': only check the voxel itself (last resort for escaping)
    """
    pt = (x, y, z)
    if not is_inside(pt, grid.shape):
        return False
    if not v_empty(grid[x, y, z]):
        return False

    if safety == 'strict':
        if skip_dangerous_blocks2(pt, grid):
            return False
    elif safety == 'medium':
        # Check cardinal XY neighbors (no diagonals, no Z) — wall proximity check
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if is_inside((nx, ny, z), grid.shape) and not v_empty(grid[nx, ny, z]):
                return False

    return True


def check_collision_3d(grid, n1, n2, height_restr=None, safety='strict'):
    """Check collision along line from n1 to n2. Returns True if BLOCKED."""
    dist = get_dist(n1, n2)
    if dist < 0.5:
        return False

    steps = max(int(dist * 2), 2)
    for i in range(1, steps + 1):
        t = i / float(steps)
        px = int(round(n1.x + t * (n2.x - n1.x)))
        py = int(round(n1.y + t * (n2.y - n1.y)))
        pz = int(round(n1.z + t * (n2.z - n1.z)))

        if height_restr is not None:
            if pz < height_restr[0] or pz > height_restr[1]:
                return True

        if not _is_valid_point(grid, px, py, pz, safety=safety):
            return True

    return False


def _interpolate_path(raw_path):
    """Interpolate path to max 1 voxel step (26-connectivity)."""
    if len(raw_path) <= 1:
        return raw_path

    interpolated = [raw_path[0]]
    for i in range(1, len(raw_path)):
        x0, y0, z0 = interpolated[-1]
        x1, y1, z1 = raw_path[i]
        dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
        steps = max(abs(dx), abs(dy), abs(dz))

        if steps <= 1:
            if (x1, y1, z1) != interpolated[-1]:
                interpolated.append((x1, y1, z1))
            continue

        for s in range(1, steps + 1):
            t = s / float(steps)
            pt = (int(round(x0 + dx * t)), int(round(y0 + dy * t)), int(round(z0 + dz * t)))
            if pt != interpolated[-1]:
                interpolated.append(pt)

    return interpolated


def _get_height_restriction():
    from a_star import tol as a_star_tol
    return a_star_tol


def _generate_escape_path(grid, start_tuple, goal_tuple, height_restr, max_steps=15):
    """
    Emergency escape with MEDIUM safety (cardinal XY check only).
    Keeps 1-voxel wall clearance in XY while allowing tighter spaces than 3x3x3.
    Uses visited set to prevent loops.
    """
    sx, sy, sz = start_tuple
    gx, gy, gz = goal_tuple

    if not is_inside((sx, sy, sz), grid.shape):
        return []

    path = [start_tuple]
    visited = {start_tuple}
    cx, cy, cz = sx, sy, sz

    for _ in range(max_steps):
        best_pt = None
        best_dist = float('inf')

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    nx, ny, nz = cx + dx, cy + dy, cz + dz

                    if height_restr is not None:
                        if nz < height_restr[0] or nz > height_restr[1]:
                            continue

                    pt = (nx, ny, nz)
                    if pt in visited:
                        continue

                    # Medium safety: cardinal XY wall check
                    if not _is_valid_point(grid, nx, ny, nz, safety='medium'):
                        continue

                    dist = np.sqrt((nx - gx)**2 + (ny - gy)**2 + (nz - gz)**2)
                    if dist < best_dist:
                        best_dist = dist
                        best_pt = pt

        if best_pt is None:
            # Medium failed, try relaxed for 1 step to unblock
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny, nz = cx + dx, cy + dy, cz
                    pt = (nx, ny, nz)
                    if pt in visited:
                        continue
                    if _is_valid_point(grid, nx, ny, nz, safety='relaxed'):
                        dist = np.sqrt((nx - gx)**2 + (ny - gy)**2 + (nz - gz)**2)
                        if dist < best_dist:
                            best_dist = dist
                            best_pt = pt
            if best_pt is None:
                break

        path.append(best_pt)
        visited.add(best_pt)
        cx, cy, cz = best_pt

    return path if len(path) > 1 else []


def _rrt_independent_search(grid, start_tuple, goal_tuple, height_restr,
                            max_iter=1500, step_size=3.0):
    """Standalone RRT* without A* seed. Goal-biased cone sampling."""
    start_node = Node(*start_tuple)
    goal_node = Node(*goal_tuple)

    if not _is_valid_point(grid, start_node.x, start_node.y, start_node.z, safety='relaxed'):
        return [], True

    nodes = [start_node]
    goal_vec = np.array([goal_tuple[0] - start_tuple[0],
                         goal_tuple[1] - start_tuple[1],
                         goal_tuple[2] - start_tuple[2]], dtype=float)
    goal_dist_total = np.linalg.norm(goal_vec)
    if goal_dist_total < 1:
        return [start_tuple, goal_tuple], False

    for i in range(max_iter):
        r = random.random()
        if r < 0.15:
            rx, ry, rz = goal_tuple
        elif r < 0.70:
            t = random.uniform(0.05, 1.0)
            spread = min(t * 15.0, 20.0)
            base = np.array(start_tuple, dtype=float) + goal_vec * t
            rx = base[0] + random.uniform(-spread, spread)
            ry = base[1] + random.uniform(-spread, spread)
            rz = base[2] + random.uniform(-1.5, 1.5)
        else:
            rx = random.uniform(1, grid.shape[0] - 2)
            ry = random.uniform(1, grid.shape[1] - 2)
            rz = random.uniform(max(1, height_restr[0]), min(grid.shape[2] - 2, height_restr[1]))

        rz = max(height_restr[0], min(height_restr[1], rz))
        rnd_node = Node(rx, ry, rz)

        nearest = min(nodes, key=lambda n: get_dist(n, rnd_node))
        dist = get_dist(nearest, rnd_node)
        if dist < 0.5:
            continue

        move_dist = min(step_size, dist)
        ratio = move_dist / dist
        new_node = Node(
            nearest.x + (rnd_node.x - nearest.x) * ratio,
            nearest.y + (rnd_node.y - nearest.y) * ratio,
            max(height_restr[0], min(height_restr[1],
                nearest.z + (rnd_node.z - nearest.z) * ratio))
        )

        if not _is_valid_point(grid, new_node.x, new_node.y, new_node.z):
            continue
        if check_collision_3d(grid, nearest, new_node, height_restr):
            continue

        search_radius = min(step_size * 1.5, 5.0)
        neighbors = [n for n in nodes if get_dist(n, new_node) < search_radius]
        best_parent, min_cost = nearest, nearest.cost + get_dist(nearest, new_node)

        for nb in neighbors:
            c = nb.cost + get_dist(nb, new_node)
            if c < min_cost and not check_collision_3d(grid, nb, new_node, height_restr):
                best_parent, min_cost = nb, c

        new_node.parent = best_parent
        new_node.cost = min_cost
        nodes.append(new_node)

        for nb in neighbors:
            rc = new_node.cost + get_dist(new_node, nb)
            if rc < nb.cost and not check_collision_3d(grid, new_node, nb, height_restr):
                nb.parent, nb.cost = new_node, rc

        if get_dist(new_node, goal_node) < step_size * 2:
            if not check_collision_3d(grid, new_node, goal_node, height_restr):
                goal_node.parent = new_node
                goal_node.cost = new_node.cost + get_dist(new_node, goal_node)
                raw = []
                curr = goal_node
                while curr:
                    raw.append((curr.x, curr.y, curr.z))
                    curr = curr.parent
                return _interpolate_path(raw[::-1]), False

    last_node = min(nodes, key=lambda n: get_dist(n, goal_node))
    raw = []
    curr = last_node
    while curr:
        raw.append((curr.x, curr.y, curr.z))
        curr = curr.parent
    raw = raw[::-1]
    return (_interpolate_path(raw), True) if len(raw) > 1 else ([], True)


def _rrt_optimize_astar(grid, start_tuple, goal_tuple, astar_path, astar_unf,
                         height_restr, max_iter, step_size):
    """Standard Informed-RRT*: optimize an A* seed path."""
    c_best = sum(
        np.sqrt(sum((a - b)**2 for a, b in zip(astar_path[i], astar_path[i+1])))
        for i in range(len(astar_path) - 1)
    )

    start_node = Node(*start_tuple)
    goal_node = Node(*goal_tuple)
    nodes = [start_node]

    for idx in range(1, len(astar_path), 2):
        p = astar_path[idx]
        new_node = Node(*p)
        nearest = min(nodes, key=lambda n: get_dist(n, new_node))
        if not check_collision_3d(grid, nearest, new_node, height_restr):
            new_node.parent = nearest
            new_node.cost = nearest.cost + get_dist(nearest, new_node)
            nodes.append(new_node)

    for i in range(max_iter):
        if random.random() < 0.75:
            base = random.choice(astar_path)
            rx = base[0] + random.uniform(-4, 4)
            ry = base[1] + random.uniform(-4, 4)
            rz = base[2] + random.uniform(-1, 1)
        else:
            rx = random.uniform(1, grid.shape[0] - 2)
            ry = random.uniform(1, grid.shape[1] - 2)
            rz = random.uniform(max(1, height_restr[0]), min(grid.shape[2] - 2, height_restr[1]))

        rz = max(height_restr[0], min(height_restr[1], rz))
        rnd_node = Node(rx, ry, rz)
        nearest = min(nodes, key=lambda n: get_dist(n, rnd_node))
        dist = get_dist(nearest, rnd_node)
        if dist < 0.5:
            continue

        ratio = min(step_size, dist) / dist
        new_node = Node(
            nearest.x + (rnd_node.x - nearest.x) * ratio,
            nearest.y + (rnd_node.y - nearest.y) * ratio,
            max(height_restr[0], min(height_restr[1],
                nearest.z + (rnd_node.z - nearest.z) * ratio))
        )

        if not _is_valid_point(grid, new_node.x, new_node.y, new_node.z):
            continue
        if check_collision_3d(grid, nearest, new_node, height_restr):
            continue

        search_radius = min(step_size * 1.5, 5.0)
        neighbors = [n for n in nodes if get_dist(n, new_node) < search_radius]
        best_parent, min_cost = nearest, nearest.cost + get_dist(nearest, new_node)
        for nb in neighbors:
            c = nb.cost + get_dist(nb, new_node)
            if c < min_cost and not check_collision_3d(grid, nb, new_node, height_restr):
                best_parent, min_cost = nb, c

        new_node.parent = best_parent
        new_node.cost = min_cost
        nodes.append(new_node)

        for nb in neighbors:
            rc = new_node.cost + get_dist(new_node, nb)
            if rc < nb.cost and not check_collision_3d(grid, new_node, nb, height_restr):
                nb.parent, nb.cost = new_node, rc

    last_node = min(nodes, key=lambda n: get_dist(n, goal_node))
    rrt_reached = False
    if get_dist(last_node, goal_node) < step_size * 2:
        if not check_collision_3d(grid, last_node, goal_node, height_restr):
            goal_node.parent = last_node
            goal_node.cost = last_node.cost + get_dist(last_node, goal_node)
            last_node = goal_node
            rrt_reached = True

    raw = []
    curr = last_node
    while curr:
        raw.append((curr.x, curr.y, curr.z))
        curr = curr.parent
    raw = raw[::-1]

    if len(raw) < 2:
        return astar_path, astar_unf

    rrt_len = sum(np.sqrt(sum((raw[i][d] - raw[i+1][d])**2 for d in range(3)))
                  for i in range(len(raw) - 1))
    if c_best > 0 and rrt_len > c_best * 1.5:
        return astar_path, astar_unf

    final = _interpolate_path(raw)
    print(f"[RRT*] A*={len(astar_path)}, RRT*={len(raw)}, interp={len(final)}, goal={rrt_reached}")
    return final, (not rrt_reached) and astar_unf


def informed_rrt_star_3d(grid, start_tuple, goal_tuple, max_iter=2500, step_size=2.5):
    """
    3-level fallback planner:
      Level 1: A* + RRT* optimization
      Level 2: Independent RRT*
      Level 3: Escape path (medium safety → relaxed fallback)
    """
    height_restr = _get_height_restriction()

    # Level 1
    astar_path, astar_unf = a_star_3d(grid, start_tuple, goal_tuple)
    if astar_path and len(astar_path) > 1:
        result = _rrt_optimize_astar(grid, start_tuple, goal_tuple,
                                      astar_path, astar_unf, height_restr,
                                      max_iter, step_size)
        if result[0]:
            return result

    # Level 2
    print(f"[RRT*] A* failed, trying independent RRT*...")
    rrt_path, rrt_unf = _rrt_independent_search(
        grid, start_tuple, goal_tuple, height_restr,
        max_iter=min(max_iter, 2000), step_size=step_size)
    if rrt_path and len(rrt_path) > 1:
        print(f"[RRT*] Independent found {len(rrt_path)} pts")
        return rrt_path, rrt_unf

    # Level 3
    print(f"[RRT*] All global planners failed, generating escape path (medium safety)...")
    escape = _generate_escape_path(grid, start_tuple, goal_tuple, height_restr, max_steps=12)
    if escape and len(escape) > 1:
        print(f"[RRT*] Escape: {len(escape)} steps")
        return escape, True

    print(f"[RRT*] WARNING: All levels failed")
    return [], True