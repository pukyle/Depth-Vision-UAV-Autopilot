import airsim
import cv2
import numpy as np

class Visualizer:
    def __init__(self, client):
        self.client = client

    def draw_path_3d(self, path_points, z=-1.5, color=[1.0, 0.0, 0.0, 1.0], duration=5.0):
        """
        在 AirSim 3D 場景中畫出路徑
        :param path_points: [[x, y], ...]
        :param z: 畫線的高度 (建議設為負值，例如 -1.5)
        :param color: RGBA [R, G, B, Alpha] (0~1)
        :param duration: 線條滯留時間 (秒)
        """
        if not path_points or len(path_points) < 2:
            return

        points_3d = []
        for p in path_points:
            # 將 2D 點轉為 AirSim 3D 向量
            points_3d.append(airsim.Vector3r(p[0], p[1], z))

        # 呼叫 API 畫線
        self.client.simPlotLineList(points_3d, 
                                    color_rgba=color, 
                                    thickness=10.0, 
                                    duration=duration, 
                                    is_persistent=False)

    def draw_obstacles(self, obstacles, z=-1.5, duration=0.1):
        """
        在障礙物位置畫出標記 (例如畫一個小的十字或點)
        :param obstacles: [[x, y, r], ...] 全域座標
        """
        # TODO: 可以改成畫圓柱體 (simPlotCylinder) 會更清楚
        points = []
        for obs in obstacles:
            x, y = obs[0], obs[1]
            # 畫一個垂直線代表障礙物
            p_start = airsim.Vector3r(x, y, z)
            p_end = airsim.Vector3r(x, y, z - 2.0)
            self.client.simPlotLineList([p_start, p_end], 
                                        color_rgba=[0.0, 1.0, 0.0, 1.0], 
                                        thickness=5.0, 
                                        duration=duration,
                                        is_persistent=False)

    @staticmethod
    def show_depth_image(depth_img_float, window_name="Depth View"):
        """
        使用 OpenCV 顯示深度圖 (除錯用)
        :param depth_img_float: 來自 AirSim 的原始 float 陣列
        """
        if depth_img_float is None or depth_img_float.size == 0:
            return

        # 正規化深度圖以便顯示 (0~255)
        # 假設 0m 是黑，10m 是白
        depth_vis = np.clip(depth_img_float, 0, 10) / 10.0 * 255.0
        depth_vis = depth_vis.astype(np.uint8)
        
        # 轉成偽彩色 (Heatmap) 會比較容易看清距離
        depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        cv2.imshow(window_name, depth_color)
        cv2.waitKey(1) # 讓視窗刷新，設定為 1ms 避免卡住