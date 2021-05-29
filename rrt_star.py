#!/usr/bin/env python3
import rospy as rp
from grid_map import GridMap
import numpy as np
import random
import math

np.random.seed(444)

class RRT(GridMap):
    global distance_list
    distance_list = []
    global points_list
    points_list = []

    def __init__(self):
        super(RRT, self).__init__()
        self.step = 0.05

    def check_if_valid(self, a, b):
        global in_free_space
        x = np.linspace(a[0], b[0], num=100, endpoint=True)
        y = np.linspace(a[1], b[1], num=100, endpoint=True)
        skale = np.shape(self.map)
        skale_x = skale[1] / self.width
        skale_y = skale[0] / self.height
        for i in range(0, 100):
            if self.map[int(y[i] * skale_y), int(x[i] * skale_x)] == 100:
                in_free_space = False
                break
            in_free_space = True
        return in_free_space

    def random_point(self):
        x = random.uniform(0, self.width)
        y = random.uniform(0, self.height)
        return (x, y)

    def find_closest(self, pos, list_of_points):
        distance = 10000
        global closest
        for point in list_of_points:
            temp = math.sqrt((pos[0] - point[0]) ** 2 + (pos[1] - point[1]) ** 2)
            distance = min(temp, distance)
            if distance == temp:
                closest = point
        return closest

    def new_pt(self, pt, closest):
        alfa = math.atan2(pt[1] - closest[1], pt[0] - closest[0])
        pt = (round(closest[0] + self.step * math.cos(alfa), 3), round(closest[1] + self.step * math.sin(alfa), 3))
        return pt

    def make_path(self, start, current):
        path = [current]
        while current != self.start:
            current = start[current]
            path.append(current)
        return path

    def calculate_path(self, point):
        dist = 0
        while True:
            dist += math.sqrt((point[0] - self.parent[point][0]) ** 2 + (point[1] - self.parent[point][1]) ** 2)
            point = self.parent[point]
            if point == self.start:
                break
        distance_list.append(dist)

    def update_path(self, point, list_of_points, distance_list):
        for i in range(0, len(distance_list)):
            dist = math.sqrt((point[0] - self.parent[point][0]) ** 2 + (point[1] - self.parent[point][1]) ** 2)
            if i > 0:
                if (dist + distance_list[i] < dist + distance_list[i-1]):
                    if self.check_if_valid(point, list_of_points[0]):
                        self.parent[point] = list_of_points[0]
                        print(i)
                        print(distance_list[i])
                        print(distance_list[i-1])
                        print(distance_list)

    def search(self):
        self.parent[self.start] = None
        # points_list = []
        points_list.append(self.start)
        while True:
            p = self.random_point()
            temp = self.find_closest(p, points_list)
            p_new = self.new_pt(p, temp)
            if self.check_if_valid(temp, p_new):
                if self.check_if_valid(p_new, self.start):
                    self.parent[p_new] = self.start
                else:
                    self.parent[p_new] = temp
                points_list.append(p_new)
                self.publish_search()
                self.calculate_path(p_new)
                self.update_path(p_new, points_list, distance_list)
                if self.check_if_valid(p_new, self.end):
                    break
            rp.sleep(0.05)
        self.parent[self.end] = p_new
        path = self.make_path(self.parent, self.end)
        self.publish_path(path)


if __name__ == '__main__':
    rrt = RRT()
    rrt.search()