import os
import numpy as np
import matplotlib.pyplot as plt

# =========================
# 使用者設定
# =========================
POINTCLOUD_DIR = "POINTCLOUD"
DRONE_RADIUS = 0.3
OBSTACLE_RADIUS = 0.3
TIME_HORIZON = 2.0

MAX_RANGE = 8.0
ANGLE_BINS = 181

V_MAX = 3.0
V_RES = 0.05


# =========================
# 1. 點雲 → 水平投影
# =========================
def project_horizontal_airsim(points,
                               x_min=0.3,
                               x_max=8.0,
                               z_min=-1.0,
                               z_max=1.0):
    """
    AirSim 相機座標:
    X forward, Y right, Z down
    """
    mask = (
        (points[:, 0] > x_min) &
        (points[:, 0] < x_max) &
        (points[:, 2] > z_min) &
        (points[:, 2] < z_max)
    )
    return points[mask][:, [0, 1]]  # (X, Y)


# =========================
# 2. 點雲 → 距離 ARRAY
# =========================
def points_to_ranges(points,
                     angle_bins=ANGLE_BINS,
                     max_range=MAX_RANGE):
    angles = np.linspace(-np.pi / 2, np.pi / 2, angle_bins)
    ranges = np.ones(angle_bins) * max_range

    for x, y in points:
        r = np.sqrt(x**2 + y**2)
        if r > max_range:
            continue
        a = np.arctan2(y, x)
        idx = np.argmin(np.abs(angles - a))
        ranges[idx] = min(ranges[idx], r)

    return ranges, angles


# =========================
# 3. VO 幾何
# =========================
def compute_vo(p, R, T):
    d = np.linalg.norm(p)
    if d <= R:
        return None

    theta = np.arctan2(p[1], p[0])
    alpha = np.arcsin(R / d)
    return theta, alpha, d / T


# =========================
# 4. 建立 VO Map
# =========================
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


# =========================
# 5. VO 視覺化
# =========================
def plot_vo(vx, vy, VO, title="Velocity Obstacle"):
    plt.figure(figsize=(6, 6))
    plt.imshow(
        VO.T,
        origin="lower",
        extent=[vx[0], vx[-1], vy[0], vy[-1]],
        cmap="Reds",
        alpha=0.8
    )
    plt.xlabel("vx (m/s)")
    plt.ylabel("vy (m/s)")
    plt.title(title)
    plt.grid(True)
    plt.show()


# =========================
# 6. 主流程
# =========================
def main():
    files = sorted([
        f for f in os.listdir(POINTCLOUD_DIR)
        if f.endswith(".npy")
    ])

    if not files:
        print("No point cloud files found.")
        return

    for fname in files:
        print(f"Processing {fname}")
        points = np.load(os.path.join(POINTCLOUD_DIR, fname))

        h_points = project_horizontal_airsim(points)
        ranges, angles = points_to_ranges(h_points)
        vx, vy, VO = build_vo_map(ranges, angles)

        plot_vo(vx, vy, VO, title=f"VO - {fname}")
        break   # 若要即時跑，先看單張，移除此行即可


if __name__ == "__main__":
    main()
