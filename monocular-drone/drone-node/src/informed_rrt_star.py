"""
Informed-RRT* path planner — Performance-optimized version.

Design philosophy: NEVER be worse than A*.

Performance optimizations:
  1. Reduced max iterations (500 instead of 2500) + 0.5s time budget
  2. KD-Tree for nearest neighbor search — O(log n) instead of O(n)
  3. First plan vs replan — RRT* only on first 3 plans, A* only after that
"""

import numpy as np
import random
import time
from a_star import a_star_3d, v_empty, skip_dangerous_blocks2, is_inside

try:
    from scipy.spatial import cKDTree
    HAS_KDTREE = True
except ImportError:
    HAS_KDTREE = False


class Node:
    __slots__ = ['x', 'y', 'z', 'parent', 'cost', 'idx']
    def __init__(self, x, y, z):
        self.x = int(round(x))
        self.y = int(round(y))
        self.z = int(round(z))
        self.parent = None
        self.cost = 0.0
        self.idx = -1


class NodeTree:
    """KD-Tree accelerated node storage."""
    def __init__(self):
        self.nodes = []
        self._coords = []
        self._tree = None
        self._rebuild_every = 50

    def add(self, node):
        node.idx = len(self.nodes)
        self.nodes.append(node)
        self._coords.append([node.x, node.y, node.z])
        if HAS_KDTREE and len(self.nodes) % self._rebuild_every == 0:
            self._tree = cKDTree(np.array(self._coords))

    def nearest(self, target):
        if HAS_KDTREE and self._tree is not None and self._tree.n == len(self.nodes):
            _, idx = self._tree.query([target.x, target.y, target.z])
            return self.nodes[idx]
        return min(self.nodes, key=lambda n: _dist_sq(n, target))

    def neighbors_within(self, target, radius):
        if HAS_KDTREE and self._tree is not None and self._tree.n == len(self.nodes):
            indices = self._tree.query_ball_point([target.x, target.y, target.z], radius)
            return [self.nodes[i] for i in indices]
        r_sq = radius * radius
        return [n for n in self.nodes if _dist_sq(n, target) < r_sq]

    def __len__(self):
        return len(self.nodes)


def _dist_sq(n1, n2):
    return (n1.x - n2.x)**2 + (n1.y - n2.y)**2 + (n1.z - n2.z)**2

def _dist(n1, n2):
    return np.sqrt(_dist_sq(n1, n2))

def _get_height_restriction():
    from a_star import tol as a_star_tol
    return a_star_tol

def _is_safe(grid, x, y, z):
    pt = (x, y, z)
    if not is_inside(pt, grid.shape):
        return False
    if not v_empty(grid[x, y, z]):
        return False
    if skip_dangerous_blocks2(pt, grid):
        return False
    return True

def _collision_free(grid, n1, n2, height_restr):
    d = _dist(n1, n2)
    if d < 0.5:
        return True
    steps = max(int(d * 2), 2)
    for i in range(1, steps + 1):
        t = i / float(steps)
        px = int(round(n1.x + t * (n2.x - n1.x)))
        py = int(round(n1.y + t * (n2.y - n1.y)))
        pz = int(round(n1.z + t * (n2.z - n1.z)))
        if pz < height_restr[0] or pz > height_restr[1]:
            return False
        if not _is_safe(grid, px, py, pz):
            return False
    return True

def _interpolate(path):
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
            pt = (int(round(x0 + dx * t)), int(round(y0 + dy * t)), int(round(z0 + dz * t)))
            if pt != result[-1]:
                result.append(pt)
    return result

def _path_length(path):
    if len(path) < 2:
        return 0
    return sum(np.sqrt(sum((path[i][d] - path[i+1][d])**2 for d in range(3)))
               for i in range(len(path) - 1))


def _rrt_optimize(grid, astar_path, height_restr, max_iter=500, step_size=2.5):
    """
    RRT* optimization with time budget and KD-Tree.
    max_iter=500 (down from 2500), plus 0.5s hard time limit.
    """
    if len(astar_path) < 3:
        return None

    start_tuple = astar_path[0]
    goal_tuple = astar_path[-1]
    astar_len = _path_length(astar_path)

    start_node = Node(*start_tuple)
    goal_node = Node(*goal_tuple)

    tree = NodeTree()
    tree.add(start_node)

    # Seed with A* waypoints
    for idx in range(1, len(astar_path), 3):
        p = astar_path[idx]
        nn = Node(*p)
        nearest = tree.nearest(nn)
        if _collision_free(grid, nearest, nn, height_restr):
            nn.parent = nearest
            nn.cost = nearest.cost + _dist(nearest, nn)
            tree.add(nn)

    t_start = time.time()
    time_budget = 0.5  # Hard time limit

    for iteration in range(max_iter):
        if iteration % 50 == 0 and iteration > 0:
            if time.time() - t_start > time_budget:
                break

        if random.random() < 0.8:
            base = random.choice(astar_path)
            rx = base[0] + random.uniform(-3, 3)
            ry = base[1] + random.uniform(-3, 3)
            rz = base[2] + random.uniform(-1, 1)
        else:
            rx = random.uniform(1, grid.shape[0] - 2)
            ry = random.uniform(1, grid.shape[1] - 2)
            rz = random.uniform(max(1, height_restr[0]),
                                min(grid.shape[2] - 2, height_restr[1]))

        rz = max(height_restr[0], min(height_restr[1], rz))
        rnd = Node(rx, ry, rz)

        nearest = tree.nearest(rnd)
        d = _dist(nearest, rnd)
        if d < 0.5:
            continue

        ratio = min(step_size, d) / d
        nn = Node(nearest.x + (rnd.x - nearest.x) * ratio,
                  nearest.y + (rnd.y - nearest.y) * ratio,
                  max(height_restr[0], min(height_restr[1],
                      nearest.z + (rnd.z - nearest.z) * ratio)))

        if not _is_safe(grid, nn.x, nn.y, nn.z):
            continue
        if not _collision_free(grid, nearest, nn, height_restr):
            continue

        radius = min(step_size * 1.5, 5.0)
        neighbors = tree.neighbors_within(nn, radius)
        best, best_cost = nearest, nearest.cost + _dist(nearest, nn)
        for nb in neighbors:
            c = nb.cost + _dist(nb, nn)
            if c < best_cost and _collision_free(grid, nb, nn, height_restr):
                best, best_cost = nb, c
        nn.parent = best
        nn.cost = best_cost
        tree.add(nn)

        for nb in neighbors:
            rc = nn.cost + _dist(nn, nb)
            if rc < nb.cost and _collision_free(grid, nn, nb, height_restr):
                nb.parent = nn
                nb.cost = rc

    elapsed = time.time() - t_start

    # Try to connect to goal
    closest = tree.nearest(goal_node)
    if _dist(closest, goal_node) < step_size * 2 and \
       _collision_free(grid, closest, goal_node, height_restr):
        goal_node.parent = closest
        goal_node.cost = closest.cost + _dist(closest, goal_node)
        raw = []
        curr = goal_node
        while curr:
            raw.append((curr.x, curr.y, curr.z))
            curr = curr.parent
        raw = raw[::-1]
        opt_path = _interpolate(raw)
        opt_len = _path_length(opt_path)
        if opt_len < astar_len * 0.95:
            print(f"[RRT*] Optimized: A*={astar_len:.0f} -> RRT*={opt_len:.0f} "
                  f"({len(tree)}nodes, {elapsed:.2f}s)")
            return opt_path

    print(f"[RRT*] No improvement ({len(tree)}nodes, {elapsed:.2f}s)")
    return None


# Replan counter: distinguishes first plan from reactive replans
_replan_count = 0


def informed_rrt_star_3d(grid, start, goal, max_iter=500, step_size=2.5):
    """
    Informed-RRT* with first-plan vs replan distinction.
    
    First 3 plans: A* + RRT* optimization (find good initial path)
    After that: A* only (reactive replans need speed, not optimality)
    """
    global _replan_count
    _replan_count += 1

    # Step 1: Always run A* as baseline
    astar_path, astar_unf = a_star_3d(grid, start, goal)

    # Step 2: RRT* optimization only on first 3 plans
    if _replan_count <= 3 and astar_path and len(astar_path) > 2:
        height_restr = _get_height_restriction()
        try:
            optimized = _rrt_optimize(grid, astar_path, height_restr,
                                       max_iter=max_iter, step_size=step_size)
            if optimized:
                return optimized, astar_unf
        except Exception as e:
            print(f"[RRT*] Error: {e}, using A*")

    # Step 3: Return A* result (NEVER worse than A*)
    return astar_path, astar_unf


def reset_replan_counter():
    """Call at the start of each trial."""
    global _replan_count
    _replan_count = 0