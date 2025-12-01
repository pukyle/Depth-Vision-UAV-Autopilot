import math
import numpy as np

def normalize_angle(angle):
    """
    將角度正規化到 [-pi, pi] 之間
    這對於計算角度誤差 (Heading Cost) 非常重要
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def local_to_global(local_points, drone_state):
    """
    將局部座標 (Body Frame) 轉換為全域座標 (World Frame)
    
    :param local_points: [[x, y], ...] 或 [[x, y, r], ...] (相對於無人機)
    :param drone_state: [x_global, y_global, yaw, ...]
    :return: global_points (與輸入格式相同，但座標已轉換)
    """
    if len(local_points) == 0:
        return []

    drone_x = drone_state[0]
    drone_y = drone_state[1]
    yaw = drone_state[2]

    # 旋轉矩陣 (2D Rotation Matrix)
    c, s = math.cos(yaw), math.sin(yaw)
    rotation_matrix = np.array([[c, -s], [s, c]])

    global_points = []
    
    for point in local_points:
        # 取出 x, y (假設前兩個元素是座標)
        local_xy = np.array(point[0:2])
        
        # 旋轉 + 平移
        global_xy = np.dot(rotation_matrix, local_xy) + np.array([drone_x, drone_y])
        
        # 如果原始資料有第三個維度 (例如半徑)，把它接回去
        if len(point) > 2:
            global_pt = list(global_xy) + list(point[2:])
        else:
            global_pt = list(global_xy)
            
        global_points.append(global_pt)

    return global_points

def calc_distance(p1, p2):
    """ 計算兩點歐幾里得距離 """
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def check_collision_with_margin(path, obstacles, margin=1.0):
    """
    通用碰撞檢查函式
    :param path: [[x, y], ...]
    :param obstacles: [[x, y, radius], ...]
    :param margin: 安全邊際
    """
    if not path or not obstacles:
        return False
        
    for wx, wy in path:
        for ox, oy, size in obstacles:
            dist = math.hypot(wx - ox, wy - oy)
            if dist < (size + margin):
                return True
    return False