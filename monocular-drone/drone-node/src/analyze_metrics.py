import json
import os
import numpy as np

def analyze_json_logs(log_dir, target_pos, env_dist):
    success_rate_count = 0
    all_path_ratios = []
    all_mapes = []
    all_smoothness = []
    all_comp_times = []

    # 取得資料夾內所有 JSON 檔案
    files = [f for f in os.listdir(log_dir) if f.endswith('.json')]
    total_files = len(files)

    if total_files == 0:
        print("找不到任何 JSON 檔案，請檢查路徑。")
        return

    for file in files:
        with open(os.path.join(log_dir, file), 'r', encoding='utf-8') as f:
            data = json.load(f)
            logs = data.get("log", [])
            if not logs: continue

            # 1. 導航成功率 (Success Rate, %)
            # 判斷最後一幀是否抵達且誤差 < 2m
            last_entry = logs[-1]
            last_pos = np.array(last_entry["real_pos"])
            dist_to_goal = np.linalg.norm(last_pos - np.array(target_pos))
            
            if last_entry.get("gloal_reached", False) and dist_to_goal < 2.0:
                success_rate_count += 1

            # 2. 路徑長度比 (Path Length Ratio)
            path_points = [np.array(entry["real_pos"]) for entry in logs]
            actual_path_length = sum(np.linalg.norm(path_points[i] - path_points[i-1]) 
                                    for i in range(1, len(path_points)))
            all_path_ratios.append(actual_path_length / env_dist)

            # 3. 平均絕對百分比誤差 (MAPE)
            # 如果您的 JSON 目前沒記 mape 欄位，這裡會預設為 0
            mapes = [entry.get("mape", 0) for entry in logs]
            all_mapes.append(np.mean(mapes))

            # 4. 平滑度 (Smoothness - Jerk)
            # 計算加速度 (acc) 的變化量
            acc_values = [np.array(entry["acc"]) for entry in logs]
            jerk = sum(np.linalg.norm(acc_values[i] - acc_values[i-1]) 
                       for i in range(1, len(acc_values)))
            all_smoothness.append(jerk)

            # 5. 計算時間 (Computational Time)
            # time_end - time_start 的平均值
            times = [entry["time_end"] - entry["time_start"] for entry in logs]
            all_comp_times.append(np.mean(times))

    # 輸出統計結果報表
    print(f"\n================ 實驗指標分析報告 (n={total_files}) ================")
    print(f"1. 導航成功率 (Success Rate): {success_rate_count/total_files*100:.2f} %")
    print(f"2. 平均路徑長度比 (Path Ratio): {np.mean(all_path_ratios):.4f} (越接近 1 越佳)")
    print(f"3. 平均深度估計誤差 (MAPE): {np.mean(all_mapes):.2f} %")
    print(f"4. 平均平滑度 (Smoothness/Jerk): {np.mean(all_smoothness):.4f} (越小越平穩)")
    print(f"5. 平均計算耗時 (Comp. Time): {np.mean(all_comp_times):.4f} s / 頻率: {1/np.mean(all_comp_times):.2f} Hz")
    print(f"=================================================================\n")

# 設定 AirSimNH Scenario 1 的參數
LOG_DIR = "./results_test"
TARGET_POS = [-130, -115, 3] # End (X, Y, Z)
THEORETICAL_DIST = 173.59     # 理論最短距離

analyze_json_logs(LOG_DIR, TARGET_POS, THEORETICAL_DIST)