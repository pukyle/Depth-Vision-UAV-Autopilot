import subprocess
import os
import time

# --- 設定為 10 次進行初步驗證 ---
TOTAL_TRIALS = 10 
ENV = "AirSimNH"
SCENARIO = 1
GOAL = [-130, -115, 3] # 論文 AirSimNH Scenario 1 終點
OUTPUT_DIR = f"/catkin_ws/src/results_test"

os.makedirs(OUTPUT_DIR, exist_ok=True)

for i in range(1, TOTAL_TRIALS + 1):
    logfile = os.path.join(OUTPUT_DIR, f"test_trial_{i:02d}.json")
    
    # 組合指令，確保包含 --exit_on_goal
    cmd = [
        "python3", "/catkin_ws/src/drone-node/src/mapper_nav_ros.py",
        "--goal_off", str(GOAL[0]), str(GOAL[1]), str(GOAL[2]),
        "--exit_on_goal",
        "--logfile", logfile
    ]
    
    print(f"🚀 正在執行第 {i}/{TOTAL_TRIALS} 次實驗...")
    
    try:
        # 執行並等待程式結束
        subprocess.run(cmd, timeout=600) 
        print(f"✅ 第 {i} 次已結束，準備下一場。")
    except Exception as e:
        print(f"⚠️ 第 {i} 次發生異常: {e}")

    # 緩衝時間，確保 AirSim 完全重置
    time.sleep(3)

print(f"\n🎉 10 次初步驗證完成！請檢查 {OUTPUT_DIR} 資料夾。")