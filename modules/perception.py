import airsim
import numpy as np
import cv2
import math
import msgpackrpc # 我們需要直接用底層通訊，避開版本地獄
from config import Config

class Perception:
    def __init__(self, client):
        self.client = client
        
        # --- 萬能相容模式設定 ---
        # 不管外面的 airsim 套件是哪一版，我們自己建立一條「乾淨」的連線
        # 這能解決 "invalid map key" (編碼問題) 和 "Expected 2 got 3" (參數問題)
        print(f"[Perception] 初始化相容模式連線: {Config.IP_ADDRESS}:{Config.PORT}")
        self.clean_client = msgpackrpc.Client(
            msgpackrpc.Address(Config.IP_ADDRESS, Config.PORT), 
            timeout=3
        )
        
        self.width = 0
        self.height = 0
        self.fov_degrees = 90.0
        self.focal_length = None
        self.c_x = None
        self.c_y = None

    def update_camera_intrinsics(self, width, height):
        self.width = width
        self.height = height
        self.c_x = width / 2.0
        self.c_y = height / 2.0
        self.focal_length = width / (2 * math.tan(math.radians(self.fov_degrees / 2)))

    def get_local_obstacles(self):
        """
        [CV 核心] 取得深度圖 -> 轉換為虛擬雷達數據 -> 輸出障礙物座標
        """
        # 1. 準備請求 (標準字典格式)
        requests_data = [{
            'camera_name': "front",
            'image_type': 1,         # DepthPlanar
            'pixels_as_float': True,
            'compress': False
        }]

        responses = []

        try:
            # 2. 強制發送「2 個參數」 (requests, vehicle_name)
            # 這能完美避開 "Expected 2, got 3" 的錯誤
            # 使用空字串 "" 當作無人機名稱，避開崩潰問題
            responses_raw = self.clean_client.call('simGetImages', requests_data, "")
            
            # 手動將回應包裝成 AirSim 物件，讓後面的 CV 程式碼可以通用
            responses = [airsim.ImageResponse(response_raw) for response_raw in responses_raw]

        except Exception as e:
            print(f"[Perception] Error: {e}")
            return []

        if not responses:
            return []

        response = responses[0]
        
        # 3. [CV 處理] 以下邏輯與標準 CV 方法完全一致
        img_1d = np.array(response.image_data_float, dtype=np.float32)
        
        if img_1d.size == 0:
            return []
            
        if self.width != response.width:
            self.update_camera_intrinsics(response.width, response.height)
            
        img_2d = img_1d.reshape(response.height, response.width)

        # 4. [CV 技巧] 虛擬雷達轉換 (Pseudo-LiDAR)
        # 取中間 10% 範圍
        mid_h = self.height // 2
        roi_h = int(self.height * 0.1)
        y1 = max(0, mid_h - roi_h)
        y2 = min(self.height, mid_h + roi_h)
        
        scan_strip = img_2d[y1:y2, :]
        
        # 取得每一列的最小值 (單線雷達模擬)
        lidar_scan = np.min(scan_strip, axis=0)

        # 5. 投影回 2D 平面
        obs_x_list = []
        obs_y_list = []
        
        angles = np.linspace(-self.fov_degrees/2, self.fov_degrees/2, self.width)
        
        for i in range(self.width):
            dist = lidar_scan[i]
            # 過濾條件: 0.1m ~ 5.0m
            if dist > 0.1 and dist < 5.0:
                angle_rad = math.radians(angles[i])
                ox = dist * math.cos(angle_rad)
                oy = dist * math.sin(angle_rad)
                obs_x_list.append(ox)
                obs_y_list.append(oy)

        if not obs_x_list:
            return []

        obstacles = np.column_stack((obs_x_list, obs_y_list))
        
        # 降採樣
        if len(obstacles) > 50:
            obstacles = obstacles[::3]

        # 診斷輸出
        min_d = np.min(lidar_scan)
        if min_d < 4.0:
            print(f"[Vision] 深度掃描: 最近障礙物 {min_d:.2f}m")

        return obstacles