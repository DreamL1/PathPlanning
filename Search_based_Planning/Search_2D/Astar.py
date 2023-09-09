"""
A_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import heapq

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")  # 添加目标文件夹

from Search_2D import plotting, env


class AStar:
    """AStar set the cost + heuristics as the priority
    """

    def __init__(self, s_start, s_goal, heuristic_type):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()  # class Env 环境文件，这个很重要

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.OPEN = []  # priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / VISITED order
        self.PARENT = dict()  # recorded parent 创建父节点的字典，key为当前节点，value为其父节点
        self.g = dict()  # cost to come

    def searching(self):
        """
        A_star Searching. 算法核心部分
        :return: path, visited order
        """

        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (self.f_value(self.s_start), self.s_start))

        """
        当OPEN集合不为空，则执行循环；或当前节点为目标点，跳出循环
        这里建议只要OPEN集合不为空就执行循环，会计算更多父节点信息
        """
        while self.OPEN:  # 只要OPEN里有节点就进行循环
            _, s = heapq.heappop(self.OPEN)  # 小顶堆，从堆栈中弹出并返回最小值（python中没有大顶堆，若实现大顶堆，push(A,-a)、-pop(A)）
            self.CLOSED.append(s)  # 从OPEN中选出最小的s放入CLOSED中

            if s == self.s_goal:  # stop condition 当前节点为目标点，跳出循环
                break

            for s_n in self.get_neighbor(s):  # 遍历s的8个自由度的s_n
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost  # 更新代价
                    self.PARENT[s_n] = s  # 更新父节点
                    heapq.heappush(self.OPEN, (self.f_value(s_n), s_n))  # 将自由度中的节点的f值压入OPEN堆中

        return self.extract_path(self.PARENT), self.CLOSED, self.g[self.s_goal]

    def searching_repeated_astar(self, e):
        """
        repeated A*.
        :param e: weight of A*
        :return: path and visited order
        """

        path, visited = [], []

        while e >= 1:
            p_k, v_k = self.repeated_searching(self.s_start, self.s_goal, e)
            path.append(p_k)
            visited.append(v_k)
            e -= 0.5

        return path, visited

    def repeated_searching(self, s_start, s_goal, e):
        """
        run A* with weight e.
        :param s_start: starting state
        :param s_goal: goal state
        :param e: weight of a*
        :return: path and visited order.
        """

        g = {s_start: 0, s_goal: float("inf")}
        PARENT = {s_start: s_start}
        OPEN = []
        CLOSED = []
        heapq.heappush(OPEN,
                       (g[s_start] + e * self.heuristic(s_start), s_start))

        while OPEN:
            _, s = heapq.heappop(OPEN)
            CLOSED.append(s)

            if s == s_goal:
                break

            for s_n in self.get_neighbor(s):
                new_cost = g[s] + self.cost(s, s_n)

                if s_n not in g:
                    g[s_n] = math.inf

                if new_cost < g[s_n]:  # conditions for updating Cost
                    g[s_n] = new_cost
                    PARENT[s_n] = s
                    heapq.heappush(OPEN, (g[s_n] + e * self.heuristic(s_n), s_n))

        return self.extract_path(PARENT), CLOSED

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles. 当前节点邻节点
        :param s: state
        :return: neighbors
        """

        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion 计算两个节点的移动代价
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):  # 如果是障碍，则返回inf
            return math.inf

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])  # 计算两节点移动代价，采用欧氏距离

    def is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision. 判断两个节点间是否障碍
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """

        # 如果两个节点有在预设障碍里，返回 True，表示有障碍
        if s_start in self.obs or s_end in self.obs:
            return True

        # 用来判断节点周围是否有障碍物，如果有，则不走这条路；这里不需要这个判断
        # if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
        #     if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
        #         s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
        #         s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
        #     else:
        #         s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
        #         s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
        #
        #     if s1 in self.obs or s2 in self.obs:
        #         return True

        return False

    def f_value(self, s):
        """
        f = g + h. (g: Cost to come, h: heuristic value) 计算f值
        :param s: current state
        :return: f
        """

        return self.g[s] + self.heuristic(s)

    def extract_path(self, PARENT):
        """
        Extract the path based on the PARENT set. 获取路径，这部一般放在最后
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = PARENT[s]  # 更替s的父节点是s
            path.append(s)  # 将s放入名为path的字典

            if s == self.s_start:  # 当s为起点时，寻得路径，跳出循环
                break

        return list(path)  # list强制转换为列表

    def heuristic(self, s):
        """
        Calculate heuristic. 计算估计代价
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type  # heuristic type
        goal = self.s_goal  # goal node

        if heuristic_type == "manhattan":  # 曼哈顿距离
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        if heuristic_type == "euclidean":  # 欧几里得距离
            return math.hypot(goal[0] - s[0], goal[1] - s[1])
        if heuristic_type == "diagonal":  # 对角线距离
            return (math.sqrt(2) - 2) * min(abs(goal[0] - s[0]), abs(goal[1] - s[1])) + \
                   abs(goal[0] - s[0]) + abs(goal[1] - s[1])


def main():
    # s_start = (5, 5)  # 起点
    # s_goal = (45, 25)  # 目标点
    s_start = (2, 1)
    s_goal = (7, 6)
    # 障碍设置在env里改

    astar = AStar(s_start, s_goal, "manhattan")  # 初始化
    plot = plotting.Plotting(s_start, s_goal)

    path, visited, dist = astar.searching()  # A*核心步骤

    print("The path distance is %8.5f" % dist)
    plot.animation(path, visited, "A*")  # animation

    # path, visited = astar.searching_repeated_astar(2.5)  # initial weight e = 2.5
    # plot.animation_ara_star(path, visited, "Repeated A*")


if __name__ == '__main__':
    main()







