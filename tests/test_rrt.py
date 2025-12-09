import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
modules_path = os.path.join(current_dir, '..', 'modules')
sys.path.append(modules_path)

import matplotlib.pyplot as plt
from global_planner import RRTPlanner


def main():
    print("開始 RRT 路徑規劃測試...")

    # --- 1. 設定測試環境 ---
    # 起點 (x, y)
    start = [0.0, 0.0]
    # 終點 (x, y)
    goal = [6.0, 10.0]
    # 障礙物列表 [x, y, radius]
    # 這裡模擬幾個圓形障礙物擋在起點和終點中間
    obstacle_list = [
        (3.0, 3.0, 1.5),
        (3.0, 6.0, 1.5),
        (3.0, 8.0, 1.0),
        (5.0, 5.0, 1.0)
    ]
    # 隨機採樣範圍 [min, max]
    rand_area = [-2.0, 15.0]

    # --- 2. 初始化 RRT ---
    rrt = RRTPlanner(
        rand_area=rand_area,
        expand_dis=1.0,        # 每次延伸 1 公尺
        path_resolution=0.1,   # 碰撞檢測精度
        goal_sample_rate=10,   # 10% 機率直接取終點
        max_iter=500,          # 最大嘗試次數
        robot_radius=0.5       # 無人機半徑
    )

    # --- 3. 執行規劃 ---
    path = rrt.plan(start, goal, obstacle_list)

    if path is None:
        print("路徑規劃失敗！找不到路徑。")
    else:
        print("路徑規劃成功！")
        print(f"路徑點數量: {len(path)}")

    # --- 4. 繪圖視覺化 (Matplotlib) ---
    plt.figure(figsize=(8, 8))
    
    # 畫出起點與終點
    plt.plot(start[0], start[1], "xr", markersize=10, label="Start")
    plt.plot(goal[0], goal[1], "xm", markersize=10, label="Goal")

    # 畫出障礙物
    for (ox, oy, size) in obstacle_list:
        circle = plt.Circle((ox, oy), size, color='black', alpha=0.5)
        plt.gcf().gca().add_artist(circle)

    # (選用) 畫出 RRT 探索過的樹枝 (node_list)
    # 這能讓你看懂 RRT 是怎麼「搜尋」的
    if True: 
        for node in rrt.node_list:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g", alpha=0.2)

    # 畫出最終路徑
    if path is not None:
        # path 是 [[x,y], [x,y]...]，我們把它拆開成 X 和 Y 列表方便繪圖
        path_x = [point[0] for point in path]
        path_y = [point[1] for point in path]
        plt.plot(path_x, path_y, '-r', linewidth=2, label="RRT Path")

    plt.grid(True)
    plt.axis("equal")
    plt.legend()
    plt.title("RRT Path Planning Test")
    plt.show()

if __name__ == '__main__':
    main()