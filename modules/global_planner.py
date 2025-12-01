import math
import random
import numpy as np

class RRTPlanner:
    """
    基於 RRT (Rapidly-Exploring Random Trees) 的全域路徑規劃器
    改編自 PythonRobotics (Atsushi Sakai)
    """

    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self, rand_area, expand_dis=2.0, path_resolution=0.5, 
                 goal_sample_rate=10, max_iter=500, robot_radius=1.0):
        """
        初始化參數 (這些參數在任務執行過程中通常不會變)
        :param rand_area: [min, max] 隨機採樣的座標範圍 (例如 [-20, 60])
        :param expand_dis: 每次樹生長的步長 [m]
        :param path_resolution: 碰撞檢測的解析度 [m]
        :param goal_sample_rate: 直接取目標點的機率 [%]
        :param max_iter: 最大嘗試次數
        :param robot_radius: 機器人半徑 (用於碰撞判斷)
        """
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.robot_radius = robot_radius
        self.node_list = []

    def plan(self, start, goal, obstacle_list):
        """
        執行 RRT 路徑規劃
        :param start: [x, y] 起點
        :param goal: [x, y] 終點
        :param obstacle_list: [[x, y, radius], ...] 障礙物列表
        :return: path [[x, y], ...] 由起點到終點的路徑點列表，若失敗回傳 None
        """
        self.start = self.Node(start[0], start[1])
        self.goal = self.Node(goal[0], goal[1])
        self.obstacle_list = obstacle_list
        self.node_list = [self.start]

        for i in range(self.max_iter):
            # 1. 產生隨機點
            rnd_node = self.get_random_node()
            
            # 2. 找最近的節點
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            # 3. 向隨機點延伸 (Steer)
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            # 4. 碰撞檢測 (若安全則加入樹中)
            if self.check_collision(new_node, self.obstacle_list, self.robot_radius):
                self.node_list.append(new_node)

            # 5. 判斷是否接近目標
            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.goal, self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list, self.robot_radius):
                    return self.generate_final_course(len(self.node_list) - 1)

        return None  # 找不到路徑

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node
        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.goal.x, self.goal.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        
        # 反轉路徑 (Start -> Goal)
        return path[::-1]

    def calc_dist_to_goal(self, x, y):
        dx = x - self.goal.x
        dy = y - self.goal.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand))
        else:  # goal bias
            rnd = self.Node(self.goal.x, self.goal.y)
        return rnd

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))
        return minind

    @staticmethod
    def check_collision(node, obstacleList, robot_radius):
        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            # 檢查路徑上的每一個點與該障礙物的距離
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            # 如果這段路徑中任何一點進入了障礙物範圍 (包含機器人自身半徑)
            if min(d_list) <= (size + robot_radius)**2:
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta