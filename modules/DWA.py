import os
import numpy as np
import matplotlib.pyplot as plt

# ======================================================
# 全域參數設定
# ======================================================
POINTCLOUD_DIR = "POINTCLOUD"

DRONE_RADIUS = 0.3
OBSTACLE_RADIUS = 0.3
TIME_HORIZON = 2.0

MAX_RANGE = 8.0
ANGLE_BINS = 181

V_MAX = 3.0
V_RES = 0.05

A_MAX = 300.0       # 最大加速度
DT = 0.1          # 控制週期

# DWA 權重
W_HEADING = 1.0
W_VEL = 0.5
W_CLEAR = 1.0
W_SMOOTH = 0.2


# ======================================================
# 1. 點雲 → 水平投影（AirSim）
# ======================================================
def project_horizontal_airsim(points,
                               x_min=0.3,
                               x_max=8.0,
                               z_min=-1.0,
                               z_max=1.0):
    mask = (
        (points[:, 0] > x_min) &
        (points[:, 0] < x_max) &
        (points[:, 2] > z_min) &
        (points[:, 2] < z_max)
    )
    return points[mask][:, [0, 1]]


# ======================================================
# 2. 點雲 → 距離 ARRAY
# ======================================================
def points_to_ranges(points,
                     angle_bins=ANGLE_BINS,
                     max_range=MAX_RANGE):
    angles = np.linspace(-np.pi / 2, np.pi / 2, angle_bins)
    ranges = np.ones(angle_bins) * max_range

    for x, y in points:
        r = np.hypot(x, y)
        if r > max_range:
            continue
        a = np.arctan2(y, x)
        idx = np.argmin(np.abs(angles - a))
        ranges[idx] = min(ranges[idx], r)

    return ranges, angles


# ======================================================
# 3. Velocity Obstacle
# ======================================================
def compute_vo(p, R, T):
    d = np.linalg.norm(p)
    if d <= R:
        return None
    theta = np.arctan2(p[1], p[0])
    alpha = np.arcsin(R / d)
    return theta, alpha, d / T


def build_vo_map(ranges, angles):
    vx = np.arange(-V_MAX, V_MAX, V_RES)
    vy = np.arange(0, V_MAX, V_RES)
    VO = np.zeros((len(vx), len(vy)))

    for r, a in zip(ranges, angles):
        if r >= V_MAX * TIME_HORIZON:
            continue

        p = np.array([r * np.cos(a), r * np.sin(a)])
        vo = compute_vo(
            p,
            DRONE_RADIUS + OBSTACLE_RADIUS,
            TIME_HORIZON
        )
        if vo is None:
            continue

        theta, alpha, radius = vo

        for i, vxi in enumerate(vx):
            for j, vyj in enumerate(vy):
                v = np.array([vxi, vyj])
                if np.linalg.norm(v) < radius:
                    ang = np.arctan2(vyj, vxi)
                    if abs(ang - theta) < alpha:
                        VO[i, j] = 1

    return vx, vy, VO


# ======================================================
# 4. Dynamic Window
# ======================================================
def dynamic_window_mask(vx, vy, current_v):
    mask = np.zeros((len(vx), len(vy)))
    for i, vxi in enumerate(vx):
        for j, vyj in enumerate(vy):
            dv = np.array([vxi, vyj]) - current_v
            if np.linalg.norm(dv) <= A_MAX * DT:
                mask[i, j] = 1
    return mask


# ======================================================
# 5. DWA cost functions
# ======================================================
def heading_cost(v, goal_dir):
    n = np.linalg.norm(v)
    if n < 1e-3:
        return 0.0
    return np.dot(v / n, goal_dir)


def velocity_cost(v):
    return np.linalg.norm(v) / V_MAX


def smoothness_cost(v, current_v):
    return -np.linalg.norm(v - current_v)


# ======================================================
# 6. 建立 DWA cost map
# ======================================================
def build_dwa_cost_map(vx, vy, VO, dyn_mask,
                       current_v, goal_dir):

    feasible = (VO == 0) & (dyn_mask == 1)
    cost_map = np.full((len(vx), len(vy)), -np.inf)

    for i, vxi in enumerate(vx):
        for j, vyj in enumerate(vy):
            if not feasible[i, j]:
                continue

            v = np.array([vxi, vyj])
            cost = (
                W_HEADING * heading_cost(v, goal_dir) +
                W_VEL * velocity_cost(v) +
                W_CLEAR * 1.0 +   # VO 已經是硬約束
                W_SMOOTH * smoothness_cost(v, current_v)
            )
            cost_map[i, j] = cost

    return cost_map


# ======================================================
# 7. 選擇最佳速度
# ======================================================
def select_best_velocity(vx, vy, cost_map):
    idx = np.unravel_index(np.argmax(cost_map), cost_map.shape)
    return np.array([vx[idx[0]], vy[idx[1]]])


# ======================================================
# 8. 主 DWA Controller Loop
# ======================================================
def dwa_controller_loop():
    files = sorted(f for f in os.listdir(POINTCLOUD_DIR) if f.endswith(".npy"))

    current_v = np.array([0.0, 0.0])
    goal_dir = np.array([1.0, 0.0])   # 假設目標在正前方

    for step, fname in enumerate(files):
        print(f"[STEP {step}] {fname}")

        points = np.load(os.path.join(POINTCLOUD_DIR, fname))
        h_points = project_horizontal_airsim(points)
        ranges, angles = points_to_ranges(h_points)

        vx, vy, VO = build_vo_map(ranges, angles)
        dyn_mask = dynamic_window_mask(vx, vy, current_v)
        cost_map = build_dwa_cost_map(
            vx, vy, VO, dyn_mask,
            current_v, goal_dir
        )

        best_v = select_best_velocity(vx, vy, cost_map)
        print("Selected velocity:", best_v)

        # 更新狀態（簡化模型）
        current_v = best_v

        # ---- 視覺化（debug 用）----
        plt.figure(figsize=(5, 5))
        plt.imshow(
            cost_map.T,
            origin="lower",
            extent=[vx[0], vx[-1], vy[0], vy[-1]],
            cmap="viridis"
        )
        plt.title("DWA Cost Map")
        plt.xlabel("vx")
        plt.ylabel("vy")
        plt.colorbar()
        plt.show()

        # 模擬即時
        # break  # 若只看一 frame，取消註解


# ======================================================
# Entry
# ======================================================
if __name__ == "__main__":
    dwa_controller_loop()
