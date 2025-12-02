import airsim
import cv2
import numpy as np
import time
import threading

# 術語修正
if not hasattr(airsim.ImageType, 'DepthPlanar'):
    airsim.ImageType.DepthPlanar = airsim.ImageType.DepthPlanner

# --- 全域變數 ---
latest_depth_img = None
latest_rgb_img = None
img_lock = threading.Lock()
keep_running = True

# --- 連線 ---
client_fly = airsim.MultirotorClient()
client_fly.confirmConnection()
client_fly.enableApiControl(True)
client_fly.armDisarm(True)

client_cam = airsim.MultirotorClient()
client_cam.confirmConnection()

# 相機調整
print("調整相機位置...")
camera_pose = airsim.Pose(airsim.Vector3r(0.5, 0, 0), airsim.to_quaternion(0, 0, 0))
client_cam.simSetCameraPose("0", camera_pose)

print(">>> 系統就緒 (指令監控版) <<<")
print("------------------------------------------------")
print("  [T] 起飛      |  [Space] 降落")
print("  [W] 前進      |  [S] 後退")
print("  [A] 左移      |  [D] 右移")
print("  [I] 上升      |  [K] 下降")
print("  [J] 左轉      |  [L] 右轉")
print("  [Q] 離開")
print("------------------------------------------------")

xy_speed = 3.0 
z_speed = 2.0
yaw_rate = 30.0
cmd_duration = 0.1 

# --- 背景抓圖 ---
def image_updater_thread():
    global latest_depth_img, latest_rgb_img, keep_running
    while keep_running:
        try:
            requests = [
                airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, True),
                airsim.ImageRequest("0", airsim.ImageType.Scene, False, True)
            ]
            responses = client_cam.simGetImages(requests)
            if responses:
                img_depth = np.array(responses[0].image_data_float, dtype=np.float32)
                img_depth = img_depth.reshape(responses[0].height, responses[0].width)
                
                img_rgb_1d = np.frombuffer(responses[1].image_data_uint8, dtype=np.uint8)
                img_rgb_decoded = cv2.imdecode(img_rgb_1d, cv2.IMREAD_UNCHANGED)

                with img_lock:
                    latest_depth_img = img_depth
                    if img_rgb_decoded is not None:
                        latest_rgb_img = img_rgb_decoded
            time.sleep(0.002) # 極速抓圖
        except:
            pass

t = threading.Thread(target=image_updater_thread)
t.daemon = True
t.start()

# --- 主迴圈 ---
is_flying = False

try:
    while True:
        with img_lock:
            curr_depth = latest_depth_img.copy() if latest_depth_img is not None else None
            curr_rgb = latest_rgb_img.copy() if latest_rgb_img is not None else None

        # 顯示倍率
        display_scale = 4

        if curr_depth is not None:
            max_depth = 15.0
            img_depth_display = np.clip(curr_depth, 0, max_depth) 
            img_depth_display = (img_depth_display / max_depth * 255.0).astype(np.uint8)
            img_depth_color = cv2.applyColorMap(img_depth_display, cv2.COLORMAP_JET)
            
            h, w = img_depth_color.shape[:2]
            img_depth_resized = cv2.resize(img_depth_color, (w * display_scale, h * display_scale), interpolation=cv2.INTER_NEAREST)
            cv2.putText(img_depth_resized, f"Depth ({w}x{h})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
            cv2.imshow("Depth Monitor", img_depth_resized)

        if curr_rgb is not None:
            h, w = curr_rgb.shape[:2]
            img_rgb_resized = cv2.resize(curr_rgb, (w * display_scale, h * display_scale), interpolation=cv2.INTER_LINEAR)
            cv2.putText(img_rgb_resized, f"RGB ({w}x{h})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
            cv2.imshow("RGB View", img_rgb_resized)

        # 控制邏輯
        key = cv2.waitKey(10) & 0xFF 
        if key == ord('q'):
            print("[系統] 正在關閉程式...")
            keep_running = False
            break
        elif key == ord('t'): 
            if not is_flying:
                print(f"[指令] 起飛 (Takeoff)")
                client_fly.takeoffAsync()
                is_flying = True
        elif key == 32: 
            print(f"[指令] 降落 (Land)")
            client_fly.landAsync()
            is_flying = False

        if is_flying:
            vx, vy, vz, yaw = 0.0, 0.0, 0.0, 0.0
            action_name = ""

            if key == ord('w'): 
                vx = xy_speed
                action_name = "前進"
            if key == ord('s'): 
                vx = -xy_speed
                action_name = "後退"
            if key == ord('d'): 
                vy = xy_speed
                action_name = "往右"
            if key == ord('a'): 
                vy = -xy_speed
                action_name = "往左"
            if key == ord('i'): 
                vz = -z_speed 
                action_name = "上升"
            if key == ord('k'): 
                vz = z_speed   
                action_name = "下降"
            if key == ord('j'): 
                yaw = -yaw_rate
                action_name = "左轉"
            if key == ord('l'): 
                yaw = yaw_rate 
                action_name = "右轉"

            if vx == 0 and vy == 0 and vz == 0 and yaw == 0:
                client_fly.hoverAsync()
                # 懸停時不打印，保持畫面乾淨
            else:
                # 有動作時才打印
                print(f"[指令] {action_name}")
                client_fly.moveByVelocityBodyFrameAsync(vx, vy, vz, cmd_duration, 
                    airsim.DrivetrainType.MaxDegreeOfFreedom, 
                    airsim.YawMode(True, yaw))

except KeyboardInterrupt:
    keep_running = False

finally:
    cv2.destroyAllWindows()
    try:
        print("[系統] 正在執行最後降落與斷線...")
        client_fly.hoverAsync().join()
        client_fly.armDisarm(False)
        client_fly.enableApiControl(False)
    except:
        pass
    print("程式結束。")