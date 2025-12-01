# 🚁 Depth-Vision-UAV-Autopilot

**基於深度視覺的無人機自動避障與路徑規劃系統**
*(Autonomous UAV obstacle avoidance and path planning based on depth vision)*

這是一個基於 Microsoft AirSim 的無人機自動駕駛專案。我們利用深度相機（Depth Camera）獲取環境資訊，結合全域路徑規劃（Global Planner）與局部避障演算法（Local Planner），實現無人機在未知環境中的自主導航。

---

## ⚡ 目前進度 (Current Status)

> **最後更新**：2025/12/01
> **狀態**：✅ 感知模組與多執行緒控制測試通過 (`tests/test_perception.py`)

目前我們成功解決了 AirSim Python API 在「同時控制飛行」與「獲取高解析度深度圖」時會發生的 **RPC 阻塞與卡死問題**。

**已實現功能：**
* **雙客戶端架構 (Dual-Client)**：分離「飛行控制」與「影像接收」通道，確保傳輸順暢。
* **多執行緒任務 (Multi-threading)**：背景執行緒處理飛行劇本，主執行緒處理即時影像渲染。
* **即時深度監控**：將 Float32 深度數據轉換為熱力圖 (Heatmap) 並即時顯示。

---

## 🛠️ 環境安裝 (Installation)

### 1. 系統需求
* **OS**: Windows 10/11
* **Simulator**: Microsoft AirSim (建議使用 Blocks 環境進行測試)
* **Python**: 3.8+

### 2. 安裝依賴
請確保安裝了以下 Python 套件（特別注意 AirSim 版本需與模擬器匹配）：

```bash
pip install airsim numpy opencv-python
```
### 3. AirSim 設定 (關鍵！)
執行專案前，請務必修改 Documents\AirSim\settings.json。 我們必須使用 Multirotor 模式來啟用物理引擎，並設定適當的解析度以避免延遲。
```bash
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ViewMode": "FlyWithMe",
  "CameraDefaults": {
      "CaptureSettings": [
        {
          "ImageType": 0,
          "Width": 320,
          "Height": 240,
          "FOV_Degrees": 90
        }
      ]
  }
}
```
## 🚀 如何執行 Demo (Quick Start)
目前最完整的測試腳本位於 tests/ 資料夾中。這個腳本會演示無人機自動起飛、懸停，並同時彈出即時深度圖監控視窗。

### 1. 啟動 AirSim (Blocks 環境)。

### 2. 執行測試腳本：

```Bash
# 請確保在專案根目錄，並進入虛擬環境 (.venv)
python tests/test_perception.py
```
### 3. 預期結果
* 無人機自動解鎖並起飛至 5 公尺高度。

* 電腦跳出 "AirSim Depth Monitor" 視窗，顯示彩色熱力圖（紅色近、藍色遠）。

* 視窗畫面流暢，不會因為無人機移動而卡住。

* 按鍵盤 q 可隨時強制降落並結束程式。

## 📂 專案結構 (Project Structure)
```Plaintext
Depth-Vision-UAV-Autopilot/
├── modules/                 # 核心功能模組 (開發中)
│   ├── perception.py        # 感知：處理深度圖、點雲轉換
│   ├── drone_control.py     # 控制：封裝 AirSim API
│   ├── global_planner.py    # 規劃：RRT/A* 演算法
│   └── local_planner.py     # 避障：DWA/VO 演算法
├── tests/                   # 測試腳本
│   └── test_perception.py   # ✅ 目前的主要 Demo (雙 Client 測試)
├── utils/                   # 工具函式
│   └── visualization.py     # 影像視覺化處理
├── output/                  # 執行結果輸出 (圖片、pfm 檔)
└── README.md                # 專案說明文件
```
## 💡 關鍵技術筆記 (Technical Notes)
為了讓隊友理解為什麼 test_perception.py 能跑，請注意以下機制：

### Dual-Client Strategy：

我們建立了兩個 MultirotorClient 物件。

* Client A (Pilot)：負責 takeoff, moveToPosition。

* Client B (Photographer)：負責 simGetImages。

這避免了單一 Socket 通道被影像數據塞爆導致控制指令發不出去的問題。

### Depth Map Processing：

* 使用 DepthPlanner (Float32) 獲取真實距離。

* 為了視覺化，我們將距離 Clip 在 0~10米 區間，並放大 3 倍顯示。

## 📝 待辦事項 (Roadmap)
[x] 環境建置：解決 Python Client 與 Binary 版本衝突問題。

[x] 感知測試：實現多執行緒即時深度圖獲取 (test_perception.py)。

[ ] 模組重構：將 Demo 中的雙 Client 邏輯封裝回 modules/ 中的類別。

[ ] 全域規劃：實作 RRT (Rapidly-exploring Random Tree) 演算法。

[ ] 局部避障：整合 VFH 或 DWA 演算法進行動態閃避。

[ ] PID 控制：優化飛行軌跡的平滑度。
