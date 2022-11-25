"""
Env 2D
@author: huiming zhou
"""


class Env:
    def __init__(self):
        # self.x_range = 51  # size of background
        # self.y_range = 31
        self.x_range = 9  # size of background
        self.y_range = 8

        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]  # 8个自由度

        self.obs = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """

        x = self.x_range
        y = self.y_range
        obs = set()

        # 定义x向边缘
        for i in range(x):
            obs.add((i, 0))  # 下边缘
        for i in range(x):
            obs.add((i, y - 1))  # 上边缘

        # 定义y向边缘
        for i in range(y):
            obs.add((0, i))  # 左边缘
        for i in range(y):
            obs.add((x - 1, i))  # 右边缘

        # 设置障碍物
        # for i in range(10, 21):
        #     obs.add((i, 15))
        # for i in range(15):
        #     obs.add((20, i))
        #
        # for i in range(10, 30):
        #     obs.add((30, i))
        # for i in range(16):
        #     obs.add((40, i))

        obs.add((2, 6))
        obs.add((2, 5))
        obs.add((2, 4))
        obs.add((2, 3))
        obs.add((3, 6))
        obs.add((3, 5))
        obs.add((3, 4))
        obs.add((3, 3))
        obs.add((4, 2))

        return obs
