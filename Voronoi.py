import numpy as np
import matplotlib.pyplot as plt
from math import sqrt, cos, sin, pi
from scipy.spatial import Voronoi, voronoi_plot_2d, Delaunay
from matplotlib.collections import PolyCollection
from shapely.geometry import Polygon, Point
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

class VoronoiDiag(object):
    def __init__(self):
        self.points = list()
        with open("C:/Users/DELL/Desktop/ltl2/voronoi_points", 'r') as f:
            for line in f.readlines():
                line = line.strip()
                if not len(line):
                    continue
                line = line.split()
                x = float(line[0])
                y = float(line[1])
                self.points.append([x, y])
        self.vd = Voronoi(points=self.points)
        self.tri = Delaunay(self.points)
        self.poly = list()
        self.poly_regions = [None] * self.points.__len__()
        self.poly.append(Polygon([tuple(p) for p in self.points[:4]]))
        self.poly.append(Polygon([tuple(p) for p in self.points[4:7]]))
        self.poly.append(Polygon([tuple(p) for p in self.points[7:10]]))
        self.poly.append(Polygon([tuple(p) for p in self.points[10:13]]))
        self.poly.append(Polygon([tuple(p) for p in self.points[13:16]]))

    def getvd(self):
        colors = ['r', 'b', 'c', 'g', 'y']
        axes = plt.axes()
        v = voronoi_plot_2d(self.vd, axes)
        verts = list()
        for p in self.poly:
            verts.append(p.exterior.coords)
        pc = PolyCollection(verts=verts, facecolors=colors)
        axes.add_collection(pc)
        plt.gca().set_aspect(1)
        plt.savefig('vd.eps', bbox_inches = 'tight')
        # plt.show()
        return axes

    def tt(self):
        regions = self.vd.regions
        vertices = self.vd.vertices
        centroids = [None] * regions.__len__()
        mass = [None] * regions.__len__()
        for i in range(1, regions.__len__()):
            poly_point = list()
            if i == 1:
                poly_point.append(tuple(vertices[4]))
                poly_point.append(tuple(vertices[3]))
                k = (self.points[11][1] - self.points[10][1]) / (self.points[11][0] - self.points[10][0])
                poly_point.append((vertices[3][0] - k * vertices[3][1], 0))
                poly_point.append((100, 0))
                k = (self.points[11][1] - self.points[8][1]) / (self.points[11][0] - self.points[8][0])
                poly_point.append((100, vertices[4][1] + 1.0 / k * (100 - vertices[4][0])))
            elif i == 4:
                poly_point.append(tuple(vertices[4]))
                poly_point.append(tuple(vertices[9]))
                poly_point.append(tuple(vertices[10]))
                poly_point.append(tuple(vertices[6]))
                k = (self.points[9][1] - self.points[8][1]) / (self.points[9][0] - self.points[8][0])
                poly_point.append((100, vertices[6][1] + 1.0 / k * (100 - vertices[6][0])))
                k = (self.points[11][1] - self.points[8][1]) / (self.points[11][0] - self.points[8][0])
                poly_point.append((100, vertices[4][1] + 1.0 / k * (100 - vertices[4][0])))
            elif i == 2:
                poly_point.append(tuple(vertices[6]))
                k = (self.points[9][1] - self.points[8][1]) / (self.points[9][0] - self.points[8][0])
                poly_point.append((100, vertices[6][1] + 1.0 / k * (100 - vertices[6][0])))
                poly_point.append((100, 100))
                k = (self.points[7][1] - self.points[9][1]) / (self.points[7][0] - self.points[9][0])
                poly_point.append((vertices[6][0] + k * (100 - vertices[6][1]), 100))
            elif i == 5:
                poly_point.append(tuple(vertices[6]))
                poly_point.append(tuple(vertices[10]))
                poly_point.append(tuple(vertices[12]))
                k = (self.points[7][1] - self.points[5][1]) / (self.points[7][0] - self.points[5][0])
                poly_point.append((vertices[12][0] + k * (100 - vertices[12][1]), 100))
                k = (self.points[7][1] - self.points[9][1]) / (self.points[7][0] - self.points[9][0])
                poly_point.append((vertices[6][0] + k * (100 - vertices[6][1]), 100))
            elif i == 6:
                poly_point.append(tuple(vertices[12]))
                poly_point.append(tuple(vertices[11]))
                poly_point.append(tuple(vertices[1]))
                poly_point.append(tuple(vertices[2]))
                k = (self.points[6][1] - self.points[5][1]) / (self.points[6][0] - self.points[5][0])
                poly_point.append((vertices[2][0] + k * (100 - vertices[2][1]), 100))
                k = (self.points[7][1] - self.points[5][1]) / (self.points[7][0] - self.points[5][0])
                poly_point.append((vertices[12][0] + k * (100 - vertices[12][1]), 100))
            elif i == 3:
                poly_point.append(tuple(vertices[2]))
                k = (self.points[5][1] - self.points[6][1]) / (self.points[5][0] - self.points[6][0])
                poly_point.append((vertices[2][0] + k * (100 - vertices[2][1]), 100))
                poly_point.append((0, 100))
                k = (self.points[4][1] - self.points[6][1]) / (self.points[4][0] - self.points[6][0])
                poly_point.append((0, vertices[2][1] - 1.0 / k * vertices[2][0]))
            elif i == 16:
                poly_point.append(tuple(vertices[2]))
                poly_point.append(tuple(vertices[1]))
                poly_point.append(tuple(vertices[22]))
                poly_point.append(tuple(vertices[23]))
                k = (self.points[4][1] - self.points[15][1]) / (self.points[4][0] - self.points[15][0])
                poly_point.append((0, vertices[23][1] - 1.0 / k * vertices[23][0]))
                k = (self.points[4][1] - self.points[6][1]) / (self.points[4][0] - self.points[6][0])
                poly_point.append((0, vertices[2][1] - 1.0 / k * vertices[2][0]))
            elif i == 12:
                poly_point.append(tuple(vertices[23]))
                poly_point.append(tuple(vertices[19]))
                poly_point.append(tuple(vertices[14]))
                k = (self.points[13][1] - self.points[15][1]) / (self.points[13][0] - self.points[15][0])
                poly_point.append((0, vertices[14][1] - 1.0 / k * vertices[14][0]))
                k = (self.points[4][1] - self.points[15][1]) / (self.points[4][0] - self.points[15][0])
                poly_point.append((0, vertices[23][1] - 1.0 / k * vertices[23][0]))
            elif i == 7:
                poly_point.append(tuple(vertices[14]))
                k = (self.points[13][1] - self.points[15][1]) / (self.points[13][0] - self.points[15][0])
                poly_point.append((0, vertices[14][1] - 1.0 / k * vertices[14][0]))
                poly_point.append((0, 0))
                k = (self.points[13][1] - self.points[14][1]) / (self.points[13][0] - self.points[14][0])
                poly_point.append((vertices[14][0] - k * vertices[14][1], 0))
            elif i == 13:
                poly_point.append(tuple(vertices[14]))
                poly_point.append(tuple(vertices[19]))
                poly_point.append(tuple(vertices[20]))
                poly_point.append(tuple(vertices[18]))
                k = (self.points[14][1] - self.points[10][1]) / (self.points[14][0] - self.points[10][0])
                poly_point.append((vertices[18][0] - k * vertices[18][1], 0))
                k = (self.points[14][1] - self.points[13][1]) / (self.points[14][0] - self.points[13][0])
                poly_point.append((vertices[14][0] - k * vertices[14][1], 0))
            elif i == 10:
                poly_point.append(tuple(vertices[3]))
                poly_point.append(tuple(vertices[17]))
                poly_point.append(tuple(vertices[18]))
                poly_point.append(tuple(vertices[5]))
                k = (self.points[11][1] - self.points[10][1]) / (self.points[11][0] - self.points[10][0])
                poly_point.append((vertices[3][0] - k * vertices[3][1], 0))
            else:
                for index in regions[i]:
                    poly_point.append(tuple(vertices[index]))
            poly_region = Polygon(poly_point)
            self.poly_regions[i - 1] = poly_region
            for p in self.poly:
                inter = p.intersection(poly_region)
                if not inter.is_empty:
                    centroids[i] = inter.centroid
                    mass[i] = inter.area * 0.001
                    break
        return centroids[1:], mass[1:]

    def graph(self):
        edges = list()
        regions = self.vd.regions
        vertices = self.vd.vertices
        label = [None] * regions.__len__()
        for i in range(1, regions.__len__()):
            for j in range(i + 1, regions.__len__()):
                temp = [x for x in regions[i] if x in regions[j]]
                if temp.__len__() != 0:
                    for index in temp:
                        if index == -1:
                            continue
                        elif 0 <= vertices[index, 0] <= 100 and 0 <= vertices[index, 1] <= 100:
                            edges.append([i, j])
                            break
        for i in range(self.points.__len__()):
            if i in range(0, 4):
                label[self.vd.point_region[i]] = 1
            elif i in range(4, 7):
                label[self.vd.point_region[i]] = 2
            elif i in range(7, 10):
                label[self.vd.point_region[i]] = 3
            elif i in range(10, 13):
                label[self.vd.point_region[i]] = 4
            else:
                label[self.vd.point_region[i]] = 5
        return edges, label[1:]


if __name__ == '__main__':
    vd = VoronoiDiag()
    # plt.triplot([p[0] for p in vd.points], [p[1] for p in vd.points], vd.tri.simplices.copy())
    # plt.plot([p[0] for p in vd.points], [p[1] for p in vd.points], 'o')
    axes = vd.getvd()
    c, m = vd.tt()
    for i in range(0, 16):
        print c[i], m[i]
    print vd.graph()
    p = Point(30, 60)
    theta = 0
    d1 = c[13]
    d2 = c[15]
    order = 1
    b = 0.1
    dt = 0.1
    trace = [[p.x, p.y]]
    while True:
        if order == 1:
            if p.distance(d1) > 0.1:
                for i in range(vd.poly_regions.__len__()):
                    if p.within(vd.poly_regions[i]):
                        r = i
                        break
                u1 = m[13] * (d1.x - p.x)
                u2 = m[13] * (d1.y - p.y)
                v1 = u1 * cos(theta) + u2 * sin(theta)
                w1 = -1.0 / b * u1 * sin(theta) + 1.0 / b * u2 * cos(theta)
                if w1 > 30 * pi / 180:
                    w1 = 30 * pi / 180
                if w1 < -30 * pi / 180:
                    w1 = -30 * pi / 180
                theta = theta + w1 * dt
                p = Point(p.x + v1 * cos(theta) * dt, p.y + v1 * sin(theta) * dt)
                trace.append([p.x, p.y])
            else:
                order = 2
                # break
        elif order == 2:
            if p.distance(d2) > 0.1:
                for i in range(vd.poly_regions.__len__()):
                    if p.within(vd.poly_regions[i]):
                        r = i
                        break
                u1 = m[15] * (d2.x - p.x)
                u2 = m[15] * (d2.y - p.y)
                v1 = u1 * cos(theta) + u2 * sin(theta)
                w1 = -1.0 / b * u1 * sin(theta) + 1.0 / b * u2 * cos(theta)
                if w1 > 30 * pi / 180:
                    w1 = 30 * pi / 180
                if w1 < -30 * pi / 180:
                    w1 = -30 * pi / 180
                theta = theta + w1 * dt
                p = Point(p.x + v1 * cos(theta) * dt, p.y + v1 * sin(theta) * dt)
                trace.append([p.x, p.y])
            else:
                break
    axes.plot([p[0] for p in trace], [p[1] for p in trace], color='r')
    plt.show()


