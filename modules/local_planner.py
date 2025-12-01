import numpy as np
import math
from config import Config
from utils.math_utils import normalize_angle

class DWAPlanner:
    def __init__(self):
        # --- 基礎參數 ---
        self.dt = 0.1  # [s] 預測軌跡的時間間隔
        self.predict_time = 2.0  # [s] 預測總時間
        
        # --- 採樣解析度 ---
        self.v_res = 0.1       # [m/s] 線速度解析度
        self.yaw_rate_res = 5.0 * math.pi / 180.0  # [rad/s] 角速度解析度

        # --- 評分權重 (Cost Weights) ---
        self.to_goal_cost_gain = Config.DWA_ALPHA      # Heading: 朝向目標
        self.speed_cost_gain = Config.DWA_GAMMA        # Velocity: 速度越快越好
        self.obstacle_cost_gain = Config.DWA_BETA      # Obstacle: 軌跡距離障礙物越遠越好
        
        # --- VO 相關設定 ---
        self.use_vo = True  # 是否啟用 VO
        self.vo_cost_gain = 2.0 # VO 的權重 (通常設很高，視為硬約束)
        
        self.robot_radius = 0.8  # 無人機半徑 (含安全邊際)

    def plan(self, current_state, goal, obstacles):
        """
        DWA 主入口
        :param current_state: [x, y, yaw, v, w]
        :param goal: [gx, gy]
        :param obstacles: [[ox, oy], ...] (全域座標)
        :return: best_v, best_w
        """
        
        # 1. 計算動態視窗 (Dynamic Window) - 限制速度範圍
        dw = self.calc_dynamic_window(current_state)

        # 2. 搜尋最佳控制量 (整合 DWA + VO)
        best_u, best_traj = self.calc_control_and_trajectory(current_state, dw, goal, obstacles)

        return best_u[0], best_u[1]

    def calc_dynamic_window(self, x):
        """
        計算下一個瞬間能達到的速度範圍 (考慮物理極限與馬達性能)
        """
        # 1. 硬體極限
        Vs = [0.0, Config.MAX_SPEED, 
              -Config.MAX_YAW_RATE, Config.MAX_YAW_RATE]

        # 2. 加減速極限
        Vd = [x[3] - Config.MAX_ACCEL * self.dt,
              x[3] + Config.MAX_ACCEL * self.dt,
              x[4] - Config.MAX_DYAW * self.dt,
              x[4] + Config.MAX_DYAW * self.dt]

        # 3. 取交集
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        return dw

    def calc_control_and_trajectory(self, x, dw, goal, ob):
        """
        核心迴圈：遍歷所有可能的速度組合，計算分數
        """
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        # 遍歷線速度 (v) 與 角速度 (w)
        for v in np.arange(dw[0], dw[1], self.v_res):
            for w in np.arange(dw[2], dw[3], self.yaw_rate_res):

                # 1. 預測軌跡 (Trajectory Prediction)
                trajectory = self.predict_trajectory(x, v, w)

                # 2. 計算各項成本
                to_goal_cost = self.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, goal)
                speed_cost = self.speed_cost_gain * (Config.MAX_SPEED - trajectory[-1, 3])
                ob_cost = self.obstacle_cost_gain * self.calc_obstacle_cost(trajectory, ob)
                
                # 3. 計算 Velocity Obstacle 成本 (VO Check)
                vo_cost = 0.0
                if self.use_vo:
                    vo_cost = self.vo_cost_gain * self.calc_vo_cost(v, w, x, ob)

                final_cost = to_goal_cost + speed_cost + ob_cost + vo_cost

                # 4. 找最小成本
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, w]
                    best_trajectory = trajectory

        return best_u, best_trajectory

    def calc_vo_cost(self, v, w, current_state, obstacles):
        """
        【VO 核心邏輯】
        檢查速度向量是否落在障礙物的「碰撞錐 (Collision Cone)」內。
        如果是，回傳無限大的 Cost。
        """
        # 如果速度很慢，或是原地旋轉，VO 的影響較小 (簡化處理)
        if v < 0.1:
            return 0.0
            
        robot_x, robot_y, robot_yaw = current_state[0], current_state[1], current_state[2]
        
        # 計算當前的瞬時速度向量 (考慮無人機 Heading)
        # 注意：DWA 採樣的是 (v, w)，這裡我們檢查線速度 v 的方向
        # 假設 v 沿著機頭方向 (或預測下一時刻的機頭方向)
        # 為了更精確，我們可以用「預測 0.5 秒後的航向」來做 VO 檢查
        predict_yaw = robot_yaw + w * 0.5 
        vx = v * math.cos(predict_yaw)
        vy = v * math.sin(predict_yaw)
        
        # 障礙物半徑 (假設障礙物是點，半徑主要由機器人大小決定)
        # VO 半徑 = Robot Radius + Obstacle Radius (假設 0.5)
        vo_radius = self.robot_radius + 0.5 
        
        for obs in obstacles:
            ox, oy = obs[0], obs[1]
            
            # 1. 計算相對位置向量 (Robot -> Obstacle)
            rx = ox - robot_x
            ry = oy - robot_y
            dist_sq = rx**2 + ry**2
            dist = math.sqrt(dist_sq)
            
            # 2. 距離篩選：太遠的障礙物不構成 VO 威脅
            if dist > v * self.predict_time * 2.0:
                continue
                
            # 3. 方向篩選 (Dot Product)：障礙物是否在後方？
            # 如果 (rx, ry) dot (vx, vy) < 0，表示障礙物在背後，不必避
            if rx * vx + ry * vy < 0:
                continue

            # 4. 碰撞錐檢查 (幾何法)
            # 計算圓心到「速度向量直線」的垂直距離
            # 投影長度 proj = (R . V) / |V|
            # 因為 |V| = v，所以 proj = (rx*vx + ry*vy) / v
            proj_length = (rx * vx + ry * vy) / v
            
            # 垂直距離平方 = 距離平方 - 投影長度平方
            perp_dist_sq = dist_sq - proj_length**2
            
            # 如果垂直距離 < (機器人半徑 + 障礙物半徑)，表示速度向量穿過障礙物
            if perp_dist_sq < vo_radius**2:
                return float("inf") # 碰撞！這個速度不可行
                
        return 0.0

    def predict_trajectory(self, x_init, v, w):
        """
        使用運動學模型預測軌跡 (Unicycle Model)
        """
        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= self.predict_time:
            x[2] += w * self.dt              # yaw
            x[0] += v * math.cos(x[2]) * self.dt  # x
            x[1] += v * math.sin(x[2]) * self.dt  # y
            x[3] = v
            x[4] = w
            
            trajectory = np.vstack((trajectory, x))
            time += self.dt
        return trajectory

    def calc_obstacle_cost(self, trajectory, ob):
        """
        DWA 原生的避障檢查：檢查「整條軌跡」是否碰到障礙物
        """
        if len(ob) == 0:
            return 0.0

        traj_xy = trajectory[:, 0:2] 
        min_r = float("inf")

        for i in range(len(traj_xy)):
            for j in range(len(ob)):
                ox = ob[j][0]
                oy = ob[j][1]
                dx = traj_xy[i][0] - ox
                dy = traj_xy[i][1] - oy
                # 這裡使用平方距離比較快，最後再開根號
                r_sq = dx**2 + dy**2
                
                # 撞擊檢查
                if r_sq <= self.robot_radius**2:
                    return float("Inf")
                
                if r_sq < min_r:
                    min_r = r_sq

        # 將平方距離轉回真實距離
        min_r = math.sqrt(min_r)
        return 1.0 / min_r if min_r > 0 else float("Inf")

    def calc_to_goal_cost(self, trajectory, goal):
        """
        計算軌跡終點與目標的角度誤差
        """
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        current_yaw = trajectory[-1, 2]
        
        # 使用 math_utils 的 normalize_angle 確保角度在 -pi ~ pi
        cost_angle = normalize_angle(error_angle - current_yaw)
        
        return abs(cost_angle)