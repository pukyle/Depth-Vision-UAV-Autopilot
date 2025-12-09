# Depth-Vision-UAV-Autopilot

### 基於電腦視覺的無人機自動避障與路徑規劃系統
**(Autonomous UAV obstacle avoidance and path planning based on computer vision)**

本專案基於 Microsoft AirSim 模擬器，旨在實現無人機在未知環境下的全自動導航。系統結合深度相機 (Depth Camera) 進行環境感知，利用 **RRT 演算法** 進行全域路徑規劃，並具備碰撞自動脫困機制。

---

## 📅 開發日誌 (Dev Log)

> 記錄關鍵里程碑與 Notion 筆記連結

* **2025/12/09**: [Notion - RRT Path Planning & Auto-Recovery](https://www.notion.so/2025-12-9-AirSim-rapidly-exploring-random-tree-RRT-2c4acd6c1d5380239c02f652ef4d2b61?source=copy_link)
    * 完成 RRT 全域路徑規劃整合。
    * 實作碰撞自動脫困 (後退+爬升+重規劃)。
    * 實作原路折返 (Return to Home) 功能。
    * 解決 AirSim 多執行緒 IOLoop 衝突 (Dual-Client 架構)。
* **2025/12/02**: [Notion - AirSim Setup & Tests](https://www.notion.so/2025-12-2-AirSim-2bdacd6c1d5380a8ae18f24a1d8d7ea4?source=copy_link)
* **2025/12/01**: [Notion - Project Initiation](https://www.notion.so/2025-12-1-AirSim-2bcacd6c1d5380b992d4daf31d6f74f0?source=copy_link)

---

## ✅ 功能清單與待辦 (Roadmap)

### 已完成功能 (Completed)
- [x] **環境建置**：解決 Python Client 與 Binary 版本衝突。
- [x] **深度感知**：實現即時 Depth Planar 影像獲取與熱力圖顯示。
- [x] **系統架構**：建立雙客戶端 (Dual-Client) 架構，分離控制與感知執行緒。
- [x] **全域規劃**：實作 RRT (Rapidly-exploring Random Tree) 演算法。
- [x] **自動脫困**：實作撞擊後的自動復原 (Back-off & Recovery) 機制。
- [x] **自動返航**：任務完成後原路折返。

### 待開發功能 (To-Do)
- [ ] **局部避障 (Local Planner)**：
    - [ ] 整合 VFH 或 DWA 演算法，針對動態障礙物進行閃避。
- [ ] **路徑優化**：
    - [ ] 使用 B-Spline 或 PID 控制平滑 RRT 的折線路徑。
- [ ] **地圖建置**：
    - [ ] 將深度圖轉換為 2D Grid Map 或 3D Point Cloud (OctoMap)。
- [ ] **真實無人機部署**：
    - [ ] 將演算法移植至 PX4/ROS2/Tello 環境 (預計)。

---

## 📂 專案結構 (Project Structure)

```text
Depth-Vision-UAV-Autopilot/
├── modules/                       # 核心功能模組 (演算法封裝)
│   ├── global_planner.py          # [RRT] 全域路徑規劃器
│   ├── perception.py              # [Vision] 深度圖處理與視覺化
│   └── drone_control.py           # [Control] 飛行指令封裝
├── tests/                         # 測試與執行腳本
│   ├── fly_mission.py             # [MAIN] ⭐ 主程式：整合飛行、導航、避障
│   ├── test_rrt.py                # [Unit Test] RRT 演算法邏輯驗證
│   ├── test_perception.py         # [Tool] 純深度視覺監控工具
│   └── test_perception_control.py # (舊版) 控制測試備份
├── utils/                         # 通用工具函式
└── README.md                      # 專案說明文件
```
## 🛠️ 安裝與執行 (Setup & Usage)
1. 依賴安裝
```bash
pip install airsim numpy opencv-python matplotlib
```
2. AirSim 設定 (必要!)
修改 Documents\AirSim\settings.json，確保 API 伺服器開啟：
```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ApiServerEnabled": true,
  "ViewMode": "FlyWithMe",
  "Vehicles": {
    "SimpleFlight": {
      "VehicleType": "SimpleFlight",
      "DefaultVehicleState": "Armed",
      "AllowAPIAlways": true
    }
  }
}
```
