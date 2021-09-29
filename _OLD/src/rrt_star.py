#!/usr/bin/env python3
import rospy as rp
from grid_map import GridMap
import numpy as np
import random
import math

seeds = 1000 #tutaj wpisujemy ile punktów na mapie bierzemy pod uwagę
#nie polecam testowania seed>1500, strasznie długo to trwa
np.random.seed(444)

class RRT(GridMap):

    def __init__(self):
        super(RRT, self).__init__()
        self.step = 0.05 #tutaj wpisujemy odległość pomiędzy punktami, im mniejsza tym większa dokładność
        #z drugiej strony w przypadku małej odległości, trzeba dać więcej punktów (seeds), aby algorytm znalazł ścieżkę

    #ta funkcja odpowiada za sprawdzanie, czy pomiędzy dwoma punktami na mapie występuje przeszkoda
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

    #wybór losowego punktu
    def random_point(self):
        x = random.uniform(0, self.width)
        y = random.uniform(0, self.height)
        return (x, y)

    #ta funkcja szuka najbliższego sąsiada
    def find_closest(self, pos, list_of_points):
        distance = 10000
        global closest
        for point in list_of_points:
            temp = math.sqrt((pos[0] - point[0]) ** 2 + (pos[1] - point[1]) ** 2)
            distance = min(temp, distance)
            if distance == temp:
                closest = point
        return closest

    #ta funkcja zwraca nowy punkt w przeszukiwaniu, zgodnie z ideą drzewa
    def new_pt(self, pt, closest):
        alfa = math.atan2(pt[1] - closest[1], pt[0] - closest[0])
        pt = (round(closest[0] + self.step * math.cos(alfa), 3), round(closest[1] + self.step * math.sin(alfa), 3))
        return pt

    #funkcja do tworzenia finalnej ścieżki
    def make_path(self, start, current):
        path = [current]
        while current != self.start:
            current = start[current]
            path.append(current)
        return path

    #funkcja obliczająca odległość poszczególnych punktów od punktu startowego
    def calculate_path(self, point, distance_l):
        dist = 0
        while True:
            dist += math.sqrt((point[0] - self.parent[point][0]) ** 2 + (point[1] - self.parent[point][1]) ** 2)
            point = self.parent[point]
            if point == self.start:
                break
        distance_l.append(dist)

    #funkcja optymalizująca połączenia w drzewie
    def update(self, distance_l, p1, p2, i, j):
        while True:
            if self.parent[p1] is None:
                break
            elif self.parent[p1] == self.start:
                break
            elif p1 == p2:
                break
            elif self.check_if_valid(p1, p2):
                dist = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
                if dist + distance_l[j-1] < distance_l[i-1]:
                    self.parent[p1] = p2
                    distance_l[i-1] = dist + distance_l[j-1]
                self.publish_search()
                break
            else:
                break

    #funkcja szukająca najkrótszej ścieżki pomiędzy początkiem i końcem
    def find_shortest(self, point, distance_l, i, distance_global):
        if self.check_if_valid(point, self.end):
            dist = math.sqrt((point[0] - self.end[0]) ** 2 + (point[1] - self.end[1]) ** 2)
            if dist + distance_l[i-1] < distance_global:
                distance_global = dist + distance_l[i-1]
                self.parent[self.end] = point
        return distance_global

    #przeszukiwanie
    def search(self):
        self.parent[self.start] = None
        points_list = []
        distance_list = []
        points_list.append(self.start)
        for i in range(seeds):
            p = self.random_point()
            temp = self.find_closest(p, points_list)
            p_new = self.new_pt(p, temp)
            if self.check_if_valid(temp, p_new):
                #ta wersja algorytmu jest szybsza (4 poniższe linijki)
                if self.check_if_valid(p_new, self.start):
                    self.parent[p_new] = self.start
                else:
                    self.parent[p_new] = temp
                #a ta zdecydowanie wolniejsza (1 linijka niżej)
                # self.parent[p_new] = temp
                self.calculate_path(p_new, distance_list)
                points_list.append(p_new)
                self.publish_search()
                #jak update jest tutaj to kod działa zdeeeeecydowanie dłużej, ale aktualizuje na bieżąco
                # for i in range(len(points_list)):
                #     for j in range(len(points_list)):
                #         self.update(distance_list, points_list[i], points_list[j], i, j)

            rp.sleep(0.01) #tą wartość można zwiększyć, jeśli chcemy mieć lepszą widoczność przy dodawaniu punktów

        #żeby kod działał szybciej jest 1 update na końcu
        print('Szukam optymalnej ścieżki. To może chwilę potrwać.')
        for i in range(len(points_list)):
            for j in range(len(points_list)):
                self.update(distance_list, points_list[i], points_list[j], i, j)
        distance = 100
        for i in range(len(points_list)):
            distance = self.find_shortest(points_list[i], distance_list, i, distance)
        path = self.make_path(self.parent, self.end)
        self.publish_path(path)
        print('Długość znalezionej ścieżki to ' + str(distance))

if __name__ == '__main__':
    rrt = RRT()
    rrt.search()