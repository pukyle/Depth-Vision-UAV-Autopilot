# config.py
import numpy as np

class Config:
    # --- AirSim 連線設定 ---
    IP_ADDRESS = "127.0.0.1"
    PORT = 41451

    # --- 無人機物理限制 (根據 Tello 或模擬器調整) ---
    MAX_SPEED = 2.0           # m/s
    MAX_YAW_RATE = 45.0       # deg/s
    MAX_ACCEL = 0.5           # m/s^2
    MAX_DYAW = 30.0           # deg/s^2

    # --- DWA 演算法權重 (依據報告公式調整) ---
    # G(v, w) = alpha*heading + beta*dist + gamma*vel 
    DWA_ALPHA = 0.15          # Heading (朝向目標)
    DWA_BETA = 1.0            # Distance (避障安全度)
    DWA_GAMMA = 0.1           # Velocity (速度)
    
    # --- 感知設定 ---
    LIDAR_MAX_RANGE = 10.0    # 公尺 (模擬深度圖的最大距離)
    COLLISION_DIST = 0.5      # 安全半徑