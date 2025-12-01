import airsim
import numpy as np
from config import Config

class DroneController:
    def __init__(self):
        self.client = airsim.MultirotorClient(ip=Config.IP_ADDRESS)
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        print("[System] AirSim connected & Armed.")

    def takeoff(self):
        print("[System] Taking off...")
        self.client.takeoffAsync().join()
        # 起飛後先升高一點，避免撞地
        self.client.moveToZAsync(-1.5, 1).join()

    def land(self):
        print("[System] Landing...")
        self.client.landAsync().join()
        self.client.armDisarm(False)
        self.client.enableApiControl(False)

    def get_state(self):
        """
        取得目前無人機的狀態，供 DWA 使用
        Returns: [x, y, yaw, v, w]
        """
        # TODO: 這裡目前只回傳簡單數據，之後需精確取得慣性座標系的資料
        kinematics = self.client.getMultirotorState().kinematics_estimated
        pos = kinematics.position
        vel = kinematics.linear_velocity
        # 取得 Yaw (這邊需要四元數轉歐拉角，暫時省略)
        yaw = 0.0 
        return np.array([pos.x_val, pos.y_val, yaw, vel.x_val, 0.0])

    def send_velocity(self, vx, vy, yaw_rate):
        """
        發送速度指令 (DWA 算出來的結果)
        """
        self.client.moveByVelocityAsync(
            vx, vy, 0, # z 軸暫時固定
            duration=0.1, # 指令持續時間短，模擬即時控制
            yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate)
        ).join()