# AirSim Drone Auto-Pilot Project

基於 AirSim 的無人機自動避障與路徑規劃系統。
整合了 RRT 全域規劃、DWA 局部避障與 Velocity Obstacle (VO) 演算法。

## 環境安裝

1. 安裝 Python 3.8+
2. 安裝依賴套件：
   ```bash
   pip install -r requirements.txt
   ```
3. 下載 AirSim (推薦使用 Blocks 環境進行測試)

## 專案結構
* modules/: 核心功能模組 (感知、控制、規劃)
* main.py: 主程式入口
* test/: 測試腳本(先從這裡開始練習)
* utils/: 工具函式 (數學計算、視覺化)

## 如何測試
1. 啟動 AirSim (Blocks 環境)
2. 先執行 test_perception.py (Static Perception Test)
   ```bash
   python tests/test_perception.py
   ```
3. 無人機起飛後，手動在 AirSim 中確認：如果你前方有個方塊，畫面上是否出現對應的綠色線條？
4. 如果綠線位置跟方塊重疊，代表深度圖 -> 3D座標 -> 全域座標的轉換成功。
5. 再進行test_rrt.py (RRT Path Test)
   ```bash
   python tests/test_rrt.py
   ```
6. 觀察 AirSim：
    * 你應該會看到幾個綠色柱子（虛擬障礙物）。
    * 應該會有一條紅色折線從起點繞過障礙物連到終點。
7. 如果路徑直接穿過綠色柱子，代表 RRT 的check_collision 邏輯或半徑設定有問題。
8. 這兩步測試通過後，就能進行下一步的飛行避障整合測試了。
