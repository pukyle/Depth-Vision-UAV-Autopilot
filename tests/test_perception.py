import airsim
import cv2
import numpy as np
import time
import threading

# --- 建立雙連線 ---
client_fly = airsim.MultirotorClient()
client_fly.confirmConnection()
client_fly.enableApiControl(True)
client_fly.armDisarm(True)

client_cam = airsim.MultirotorClient()
client_cam.confirmConnection()

print("已連線，準備執行飛行任務...")

# --- 飛行任務 (保持不變) ---
def flight_mission():
    try:
        print("[飛行員] 起飛中...")
        client_fly.takeoffAsync().join()

        print("[飛行員] 上升至 5 公尺...")
        client_fly.moveToZAsync(-5, 1).join()

        print("[飛行員] 懸停 3 秒...")
        time.sleep(3)

        print("[飛行員] 下降...")
        client_fly.moveToZAsync(-1, 1).join() 

        print("[飛行員] 降落...")
        client_fly.landAsync().join()
        print("[飛行員] 任務完成。")
    except Exception as e:
        print(f"[飛行員] 錯誤: {e}")

mission_thread = threading.Thread(target=flight_mission)
mission_thread.start()

print(">>> 監控視窗已開啟 (視窗已放大 3 倍) <<<")

try:
    while mission_thread.is_alive():
        # 1. 請求影像
        # compress=False 對於 Float 數據來說其實沒差，因為 Float 不會被壓縮
        requests = [airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, True)]
        
        try:
            responses = client_cam.simGetImages(requests)
        except:
            continue

        if not responses:
            continue

        response = responses[0]
        
        # 2. 數據處理
        img_float = np.array(response.image_data_float, dtype=np.float32)
        img_float = img_float.reshape(response.height, response.width)

        # 3. 視覺化 (0-10米)
        max_depth = 10.0
        img_display = np.clip(img_float, 0, max_depth) 
        img_display = (img_display / max_depth * 255.0).astype(np.uint8)
        img_color = cv2.applyColorMap(img_display, cv2.COLORMAP_JET)
        
        # --- 關鍵修改：放大視窗 (Upscale) ---
        # 取得目前寬高
        h, w = img_color.shape[:2]
        # 放大 3 倍 (INTER_NEAREST 速度最快，INTER_LINEAR 比較平滑)
        scale_factor = 3
        img_resized = cv2.resize(img_color, (w * scale_factor, h * scale_factor), interpolation=cv2.INTER_LINEAR)

        # 加上文字
        cv2.putText(img_resized, "Live Depth (3x Zoom)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

        # 4. 顯示
        cv2.imshow("AirSim Monitor", img_resized)

        # 這裡 waitKey(1) 是必須的，否則視窗不會更新
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

finally:
    cv2.destroyAllWindows()
    try:
        client_fly.landAsync().join()
        client_fly.armDisarm(False)
        client_fly.enableApiControl(False)
    except:
        pass
    print("程式結束。")