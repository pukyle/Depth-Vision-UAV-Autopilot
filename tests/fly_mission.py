import airsim
import sys
import os
import time
import threading
import math

# --- 路徑設定 ---
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
modules_path = os.path.join(project_root, 'modules')
sys.path.append(modules_path)

from global_planner import RRTPlanner

# --- 輔助函式：找出離目前位置最近的路徑點索引 ---
def find_closest_point_index(current_pos, path_points):
    min_dist = float('inf')
    closest_index = 0
    for i, point in enumerate(path_points):
        dist = math.sqrt((current_pos.x_val - point.x_val)**2 + 
                         (current_pos.y_val - point.y_val)**2 + 
                         (current_pos.z_val - point.z_val)**2)
        if dist < min_dist:
            min_dist = dist
            closest_index = i
    return closest_index

# ==========================================
# 核心飛行函式：包含飛行、監控、脫困
# ==========================================
def execute_flight_task(client_fly, client_monitor, path_3d, task_name="任務"):
    print(f"\n>>> 啟動【{task_name}】 (總路徑點: {len(path_3d)}) <<<")
    
    # 畫出這次的路徑 (不同任務用不同顏色，去程紅，回程藍)
    color = [1.0, 0.0, 0.0, 1.0] if "去程" in task_name else [0.0, 0.0, 1.0, 1.0]
    client_fly.simPlotLineList(path_3d, color_rgba=color, is_persistent=False, duration=120.0, thickness=10.0)

    remaining_path = path_3d
    retry_count = 0
    max_retries = 5

    while retry_count < max_retries:
        if len(remaining_path) < 2:
            print(f">>> 【{task_name}】即將抵達，停止導航 <<<")
            break

        print(f"[{task_name}] 執行中... 剩餘 {len(remaining_path)} 個點")

        is_flying = True
        
        # 飛行執行緒
        def flight_worker():
            nonlocal is_flying
            try:
                client_fly.moveOnPathAsync(
                    remaining_path, 
                    velocity=3.0, 
                    timeout_sec=120, 
                    drivetrain=airsim.DrivetrainType.ForwardOnly, 
                    yaw_mode=airsim.YawMode(False, 0)
                ).join()
            except Exception:
                pass
            finally:
                is_flying = False

        t = threading.Thread(target=flight_worker)
        t.start()

        hit_obstacle = False
        
        # 監控迴圈
        while is_flying:
            collision_info = client_monitor.simGetCollisionInfo()
            
            if collision_info.has_collided:
                z_current = client_monitor.getMultirotorState().kinematics_estimated.position.z_val
                obj_name = collision_info.object_name
                
                # 過濾地板
                if z_current < -0.5 and ("ground" in obj_name.lower() or "ramp" in obj_name.lower()):
                        pass
                else:
                    print(f"\n[警報] {task_name}途中發生撞擊！物件: {obj_name}")
                    hit_obstacle = True
                    
                    # 1. 煞車 (Fire and Forget)
                    client_fly.moveByVelocityAsync(0, 0, 0, 1) 
                    time.sleep(0.5)
                    
                    # 2. 脫困 (後退+上升)
                    print("[系統] 執行脫困動作...")
                    client_fly.moveByVelocityBodyFrameAsync(-2.0, 0, -1.0, 1.5)
                    time.sleep(1.5)
                    
                    # 3. 再次煞車
                    client_fly.moveByVelocityAsync(0, 0, 0, 1)
                    time.sleep(0.5)

                    is_flying = False 
                    break 

            time.sleep(0.1)
        
        t.join(timeout=1.0)

        if hit_obstacle:
            retry_count += 1
            print(f"[系統] 重新規劃路線 (嘗試 {retry_count}/{max_retries})")
            
            # 找到最近的點接關
            curr_pos = client_monitor.getMultirotorState().kinematics_estimated.position
            idx = find_closest_point_index(curr_pos, remaining_path)
            
            if idx < len(remaining_path) - 1:
                remaining_path = remaining_path[idx+1:]
            else:
                print(f"[{task_name}] 已在終點附近。")
                break
        else:
            print(f">>> 【{task_name}】順利完成！ <<<")
            return True # 任務成功

    return False # 任務失敗 (重試次數過多)

# ==========================================
# 主程式
# ==========================================
def main():
    client_fly = airsim.MultirotorClient()
    client_fly.confirmConnection()
    client_fly.enableApiControl(True)
    client_fly.armDisarm(True)

    client_monitor = airsim.MultirotorClient()
    client_monitor.confirmConnection()

    try:
        print(">>> 準備起飛... <<<")
        client_fly.takeoffAsync().join()
        
        print("正在調整高度至 3 公尺...")
        client_fly.moveToZAsync(-3.0, 1).join()
        time.sleep(1)

        # 取得起點與終點
        state = client_fly.getMultirotorState()
        pos = state.kinematics_estimated.position
        start_node = [pos.x_val, pos.y_val]
        fly_height = -3.0

        goal_node = [pos.x_val + 15, pos.y_val + 5]
        obstacle_list = [(pos.x_val + 7, pos.y_val + 1, 2.5)]

        print(f"起點: {start_node}")
        print(f"終點: {goal_node}")
        
        # RRT 規劃路徑
        rrt = RRTPlanner(
            rand_area=[min(start_node[0], goal_node[0])-10, max(start_node[0], goal_node[0])+10],
            expand_dis=1.0,
            robot_radius=1.0,
            max_iter=3000
        )
        path = rrt.plan(start_node, goal_node, obstacle_list)

        if not path:
            print("XXX 路徑規劃失敗 XXX")
            return

        print("路徑規劃完成，準備執行任務。")
        
        # 轉換為 3D 座標點 (去程路徑)
        path_forward = []
        for p in path:
            path_forward.append(airsim.Vector3r(float(p[0]), float(p[1]), fly_height))
        
        # --- 階段一：去程 ---
        success = execute_flight_task(client_fly, client_monitor, path_forward, task_name="去程")
        
        if success:
            print("\n>>> 抵達目的地，懸停 3 秒準備返航... <<<")
            client_fly.hoverAsync().join()
            time.sleep(3)

            # --- 階段二：回程 (原路折返) ---
            # 利用 Python 的 List 切片 [::-1] 來反轉路徑
            path_backward = path_forward[::-1]
            
            print(">>> 開始原路折返 <<<")
            execute_flight_task(client_fly, client_monitor, path_backward, task_name="返程")

        else:
            print("去程任務失敗，取消返航。")

    except Exception as e:
        print(f"發生錯誤: {e}")

    finally:
        print("\n正在執行降落與資源釋放...")
        try:
            client_fly.landAsync().join()
            client_fly.armDisarm(False)
            client_fly.enableApiControl(False)
        except:
            pass
        print("程式結束。")

if __name__ == '__main__':
    main()