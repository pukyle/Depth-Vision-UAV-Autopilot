import time
import numpy as np
import airsim

# --- 引入我們建立的模組 ---
from modules.drone_control import DroneController
from modules.perception import Perception
from modules.local_planner import DWAPlanner
from modules.global_planner import RRTPlanner

# --- 引入工具庫 (上一段討論的 Utils) ---
from utils.visualization import Visualizer
from utils.math_utils import local_to_global, check_collision_with_margin
from config import Config

def main():
    # ==========================================
    # 1. 初始化系統模組
    # ==========================================
    print("[Main] Initializing modules...")
    drone = DroneController()
    perception = Perception(drone.client)
    dwa = DWAPlanner()
    vis = Visualizer(drone.client)
    
    # 初始化 RRT 全域規劃器
    # rand_area: 搜尋範圍 (根據你的場景大小調整，例如 x: -20~60)
    rrt = RRTPlanner(rand_area=[-20, 60], expand_dis=3.0, robot_radius=1.0)

    # ==========================================
    # 2. 設定任務與初始規劃
    # ==========================================
    start_pos = [0.0, 0.0]      # 假設起點
    global_goal = [30.0, 0.0]   # 設定終點 (前方 30 米)
    
    known_obstacles = []        # 用來記憶所有發現過的障礙物 (全域座標)
    
    print(f"[Main] Planning initial path to {global_goal}...")
    
    # 初次規劃 (假設一開始沒有障礙物，RRT 會給出一條直線)
    path = rrt.plan(start_pos, global_goal, known_obstacles)
    
    if path is None:
        print("[Main] Critical Error: Initial RRT failed!")
        return

    # 視覺化：畫出初始路徑 (紅色線)
    vis.draw_path_3d(path, z=-1.5, duration=20.0)
    current_wp_index = 0

    try:
        # ==========================================
        # 3. 起飛與執行
        # ==========================================
        drone.takeoff()
        time.sleep(1) # 等待穩定
        print("[Main] Mission Start!")

        while True:
            # --- A. 狀態更新 (State Update) ---
            # state: [x, y, yaw, v, w]
            state = drone.get_state()
            current_pos = state[0:2]

            # --- B. 感知與地圖構建 (Perception & Mapping) ---
            # 1. 取得局部障礙物 (相對於無人機)
            local_obs = perception.get_local_obstacles()
            
            # 2. 轉換為全域座標 (World Frame)
            new_global_obs = local_to_global(local_obs, state)
            
            # 3. 視覺化障礙物 (綠色標記) - 讓你確認感知是對的
            vis.draw_obstacles(new_global_obs, duration=0.1)

            # 4. 更新已知障礙物列表
            # (簡單實作：直接加入。進階實作可用 Grid Map 避免重複)
            if new_global_obs:
                known_obstacles.extend(new_global_obs)

                # --- C. 動態重規劃 (Online Re-planning) ---
                # 只有當「新發現的障礙物」擋住了「剩餘的路徑」時，才重算
                remaining_path = path[current_wp_index:]
                
                if check_collision_with_margin(remaining_path, new_global_obs, margin=1.5):
                    print("[Main] ⚠️ Path blocked by new obstacle! Re-planning...")
                    
                    # 1. 安全懸停 (給 RRT 計算時間)
                    drone.send_velocity(0, 0, 0)
                    
                    # 2. 重新執行 RRT
                    # 以當前位置為起點，考慮所有已知障礙物
                    new_path = rrt.plan(current_pos, global_goal, known_obstacles)
                    
                    if new_path:
                        print("[Main] ✅ New path found!")
                        path = new_path
                        current_wp_index = 0 # 重置索引，從新路徑開始走
                        
                        # 畫出新路徑
                        vis.draw_path_3d(path, z=-1.5, duration=10.0)
                    else:
                        print("[Main] ❌ RRT stuck! Hovering and retrying...")
                        time.sleep(1)
                        continue

            # --- D. 路徑跟隨邏輯 (Path Following) ---
            # 1. 判斷是否抵達終點
            dist_to_goal = np.linalg.norm(current_pos - np.array(global_goal))
            if dist_to_goal < 1.0:
                print("[Main] 🏆 Goal Reached!")
                break
            
            # 2. 決定當前的局部目標 (Current Waypoint)
            if current_wp_index < len(path):
                target_wp = path[current_wp_index]
                dist_to_wp = np.linalg.norm(current_pos - np.array(target_wp))
                
                # 如果夠接近路徑點 (例如 < 2米)，就切換下一個點
                if dist_to_wp < 2.0:
                    # print(f"[Main] Reached waypoint {current_wp_index}")
                    current_wp_index += 1
            else:
                target_wp = global_goal

            # --- E. 局部規劃與控制 (DWA & Control) ---
            # DWA 負責微觀避障：在前往 target_wp 的同時，避開附近的障礙
            # 我們傳入 known_obstacles 讓 DWA 知道全域障礙物分佈
            best_v, best_w = dwa.plan(state, target_wp, known_obstacles)

            # 發送指令
            drone.send_velocity(best_v, 0, best_w)
            
            # 控制迴圈頻率 (10Hz)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("[Main] Interrupted by user.")
    except Exception as e:
        print(f"[Main] Error: {e}")
    finally:
        print("[Main] Landing...")
        drone.land()

if __name__ == "__main__":
    main()