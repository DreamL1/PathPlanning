"""
Dijkstra 2D
@author: huiming zhou
"""

import os
import sys
import math
import heapq

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting, env

from Search_2D.Astar import AStar


class Dijkstra(AStar):
    """Dijkstra set the cost as the priority
       相比于A*，Dijkstra代价函数不包括h
    """
    def searching(self):
        """
        Breadth-first Searching.
        :return: path, visited order
        """

        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (0, self.s_start))  # 将s压入堆中

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)  # 小顶堆，优先队列OPEN中弹出s
            self.CLOSED.append(s)  # 将s加入CLOSE队列

            if s == self.s_goal:  # 从Dijkstra定义出发，应该当OPEN为空时结束搜索
                break

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s

                    # best first set the heuristics as the priority 
                    heapq.heappush(self.OPEN, (new_cost, s_n))

        return self.extract_path(self.PARENT), self.CLOSED, self.g[self.s_goal]


def main():
    s_start = (5, 5)
    s_goal = (45, 25)
    # s_start = (2, 1)
    # s_goal = (7, 6)

    dijkstra = Dijkstra(s_start, s_goal, 'None')
    plot = plotting.Plotting(s_start, s_goal)

    path, visited, dist = dijkstra.searching()
    print("The path distance is %8.5f" % dist)
    plot.animation(path, visited, "Dijkstra's")  # animation generate


if __name__ == '__main__':
    main()
