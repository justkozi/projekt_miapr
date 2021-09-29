#!/usr/bin/env python3
import rospy as rp
from grid_map import GridMap
import numpy as np
import random
import math

np.random.seed(444)

class RRT(GridMap):
    def __init__(self):
        super(RRT, self).__init__()
        self.step = 0.05

    def check_if_valid(self, a, b):
        """
        Checks if the segment connecting a and b lies in the free space.
        :param a: point in 2D
        :param b: point in 2D
        :return: boolean
        """
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
        """
        Draws random point in 2D
        :return: point in 2D
        """
        x = random.uniform(0, self.width)
        y = random.uniform(0, self.height)
        return (x, y)

    def find_closest(self, pos, list_of_points):
        """
        Finds the closest vertex in the graph to the pos argument

        :param pos: point id 2D
        :return: vertex from graph in 2D closest to the pos
        """
        distance = 10000
        global closest
        for point in list_of_points:
            temp = math.sqrt((pos[0] - point[0]) ** 2 + (pos[1] - point[1]) ** 2)
            distance = min(temp, distance)
            if distance == temp:
                closest = point
        return closest


    def new_pt(self, pt, closest):
        """
        Finds the point on the segment connecting closest with pt, which lies self.step from the closest (vertex in graph)

        :param pt: point in 2D
        :param closest: vertex in the tree (point in 2D)
        :return: point in 2D
        """
        alfa = math.atan2(pt[1] - closest[1], pt[0] - closest[0])
        pt = (round(closest[0] + self.step * math.cos(alfa), 3), round(closest[1] + self.step * math.sin(alfa), 3))
        return pt

    def make_path(self, start, current):
        path = [current]
        while current != self.start:
            current = start[current]
            path.append(current)
        return path

    def search(self):
        """
        RRT search algorithm for start point self.start and desired state self.end.
        Saves the search tree in the self.parent dictionary, with key value pairs representing segments
        (key is the child vertex, and value is its parent vertex).
        Uses self.publish_search() and self.publish_path(path) to publish the search tree and the final path respectively.
        """
        self.parent[self.start] = None
        points_list = []
        points_list.append(self.start)
        while True:
            p = self.random_point()
            np.random.seed(444)
            temp = self.find_closest(p, points_list)
            p_new = self.new_pt(p, temp)
            if self.check_if_valid(temp, p_new):
                self.parent[p_new] = temp
                points_list.append(p_new)
                self.publish_search()
                if self.check_if_valid(p_new, self.end):
                    break
            rp.sleep(0.02)
        self.parent[self.end] = p_new
        path = self.make_path(self.parent, self.end)
        self.publish_path(path)


if __name__ == '__main__':
    rrt = RRT()
    rrt.search()
