import sys
import os
import numpy as np
import matplotlib.pyplot as plt

MAP_PATH = os.path.join(os.path.dirname(
    os.path.abspath(__file__)), '3-map/map.npy')


### START CODE HERE ###
# This code block is optional. You can define your utility function and class in this block if necessary.
# 启发函数部分
from queue import PriorityQueue
SQRT_2 = pow(2, 0.5)
def heuristic_estimate(id_x, id_y):
    diff_x = abs(100-id_x)
    diff_y = abs(100-id_y)
    # 类似于 pi/4 方向上的两个最邻近的距离
    return SQRT_2*min(diff_x, diff_y)+max(diff_x, diff_y)-min(diff_x, diff_y)

###  END CODE HERE  ###


def Improved_A_star(world_map, start_pos, goal_pos):
    """
    Given map of the world, start position of the robot and the position of the goal, 
    plan a path from start position to the goal using improved A* algorithm.

    Arguments:
    world_map -- A 120*120 array indicating map, where 0 indicating traversable and 1 indicating obstacles.
    start_pos -- A 2D vector indicating the start position of the robot.
    goal_pos -- A 2D vector indicating the position of the goal.

    Return:
    path -- A N*2 array representing the planned path by improved A* algorithm.
    """

    ### START CODE HERE ###
    # came_from用来记录新扩展边界的由来方向
    came_from = np.zeros_like(world_map)
    # g是已知代价函数
    g = np.zeros_like(world_map)
    direction_x = np.array(
        [-1, 0, 0, 1,
         1, 1, -1, -1,
         -2, 0, 2, 0,
         1, 1, -1, -1,
         -2, -2, -2, 2,
         2, -2, 2, -2])
    direction_y = np.array(
        [0, -1, 1, 0,
         1,  -1, -1, 1,
         0, 2, 0, -2,
         -2, 2, -2, 2,
         -1, -1, 1, 1,
         -2, 2, 2, -2])
    world_map_copy = np.zeros_like(world_map)
    # 与障碍物相邻（八个方向）的位置，增加大额代价
    for i in range(120):
        for j in range(120):
            if world_map[i][j] == 1:
                for k in range(0, 20):
                    if i+direction_x[k] <= 119 and i+direction_x[k] >= 0 and j+direction_y[k] <= 119 and j+direction_y[k] >= 0:
                        world_map_copy[i+direction_x[k]
                                       ][j+direction_y[k]] = 1

    frontier = PriorityQueue()
    frontier.put((0, start_pos))

        # core process
    while (not frontier.empty()):
        frontier_node = frontier.get()[1]
        # 从边缘优先级队列里，依据评价函数，扩展一个代价最小的点

        # 搜索到终点以后，开始建立路线
        if (frontier_node[0] == goal_pos[0] and frontier_node[1] == goal_pos[1]):
            expand_node = goal_pos
            path = [goal_pos]
            # 以下2行代码与以上2行等效
            # expand_node = frontier_node
            # path = [frontier_node]
            while (expand_node[0] != start_pos[0]) or (expand_node[1] != start_pos[1]):
                expand_node_i = came_from[expand_node[0]][expand_node[1]]
                next_node_x = expand_node[0] - direction_x[expand_node_i]
                next_node_y = expand_node[1] - direction_y[expand_node_i]
                next_node = [next_node_x, next_node_y]
                # 更新path
                path.append(next_node)
                # 更新expand_node
                expand_node = next_node
            return path

        for i in range(0, 8):
            temp_node = [frontier_node[0] + direction_x[i],
                         frontier_node[1] + direction_y[i]]
            # 必须无障碍物，才有可能扩展
            if (not world_map_copy[temp_node[0]][temp_node[1]]):
                if came_from[frontier_node[0]][frontier_node[1]] == i:
                    steer_cost = 0
                else:
                    steer_cost = 0.6
                # 已知代价g,自变量是坐标x,y
                # 从frontier_node走出去的代价是frontier_node加上两点间路程代价
                if i >= 4:
                    new_cost = g[frontier_node[0]
                                  ][frontier_node[1]]+SQRT_2+steer_cost
                else:
                    new_cost = g[frontier_node[0]
                                  ][frontier_node[1]]+1+steer_cost
                # 若没有留存过代价，或者新探索代价比留存的代价更小，则更新边缘
                if (g[temp_node[0]][temp_node[1]] == 0) or (new_cost < g[temp_node[0]][temp_node[1]]):

                    # 评价函数=已知代价+启发函数
                    g[temp_node[0]][temp_node[1]] = new_cost
                    priority = new_cost +\
                        heuristic_estimate(
                            temp_node[0], temp_node[1])
                    # 加入优先队列
                    frontier.put((priority, temp_node))
                    came_from[temp_node[0]][temp_node[1]] = i

    ###  END CODE HERE  ###
    return path


if __name__ == '__main__':

    # Get the map of the world representing in a 120*120 array, where 0 indicating traversable and 1 indicating obstacles.
    map = np.load(MAP_PATH)

    # Define goal position of the exploration
    goal_pos = [100, 100]

    # Define start position of the robot.
    start_pos = [10, 10]

    # Plan a path based on map from start position of the robot to the goal.
    path = Improved_A_star(map, start_pos, goal_pos)

    # Visualize the map and path.
    obstacles_x, obstacles_y = [], []
    for i in range(120):
        for j in range(120):
            if map[i][j] == 1:
                obstacles_x.append(i)
                obstacles_y.append(j)

    path_x, path_y = [], []
    for path_node in path:
        path_x.append(path_node[0])
        path_y.append(path_node[1])

    plt.plot(path_x, path_y, "-r")
    plt.plot(start_pos[0], start_pos[1], "xr")
    plt.plot(goal_pos[0], goal_pos[1], "xb")
    plt.plot(obstacles_x, obstacles_y, ".k")
    plt.grid(True)
    plt.axis("equal")
    plt.show()
