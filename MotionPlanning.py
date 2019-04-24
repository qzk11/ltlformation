import numpy as np
from scipy import optimize
import sympy
from math import tan, cos, sin, pi, sqrt
import matplotlib.pyplot as plt


class UAV(object):
    def __init__(self, q0, qf, tf, a4):
        self.obstacles = []
        self.pos = (q0[0], q0[1])
        self.r = 0.5
        self.q0 = q0
        self.qf = qf
        self.a = np.mat([0.0, 0.0, 0.0, 0.0, 0.0]).T
        self.a[4, 0] = a4
        self.b = np.mat([0.0, 0.0, 0.0, 0.0]).T
        self.trajectory(q0, qf, 0.0, tf)

    def trajectory(self, q0, qf, t0, tf):
        self.B1 = np.mat([[1.0, q0[0], q0[0] ** 2, q0[0] ** 3],
                          [0.0, 1.0, 2 * q0[0], 3 * q0[0] ** 2],
                          [1.0, qf[0], qf[0] ** 2, qf[0] ** 3],
                          [0.0, 1.0, 2 * qf[0], 3 * qf[0] ** 2]])
        self.Y1 = np.mat([q0[1], tan(q0[2]), qf[1], tan(qf[2])]).T
        self.A1 = np.mat([q0[0] ** 4, 4 * q0[0] ** 3, qf[0] ** 4, 4 * qf[0] ** 3]).T
        temp = self.B1.I * (self.Y1 - self.A1 * self.a[4, 0])
        self.a[0:4, 0] = temp
        self.B2 = np.mat([[1.0, t0, t0 ** 2, t0 ** 3],
                          [0.0, 1.0, 2 * t0, 3 * t0 ** 2],
                          [1.0, tf, tf ** 2, tf ** 3],
                          [0.0, 1.0, 2 * tf, 3 * tf ** 2]
                          ])
        self.Y2 = np.mat([q0[0], q0[3] * cos(q0[2]), qf[0], qf[3] * cos(qf[2])]).T
        self.b = self.B2.I * self.Y2

    def update(self, q0):
        self.trajectory(q0, self.qf, 0, 10)

    def addObstacle(self, q, r, v, theta):
        obs = {"pos": q, "radius": r, "velocity": v, "angle": theta}
        self.obstacles.append(obs)

    def getCoe(self, t):
        x = self.pos[0]
        y = self.pos[1]
        h = np.mat([1.0, x, x ** 2, x ** 3])
        t = 0
        for obs in self.obstacles:
            g0 = (h * self.B1.I * self.Y1 - obs["pos"][1] - obs["velocity"] * sin(obs["angle"]) * t) ** 2 + \
                 (x - obs["pos"][0] - obs["velocity"] * cos(obs["angle"]) * t) ** 2 - (self.r + obs["radius"]) ** 2
            g1 = 2 * (x ** 4 - h * self.B1.I * self.A1) * \
                 (h * self.B1.I * self.Y1 - obs["pos"][0] - obs["velocity"] * sin(obs["angle"]) * t)
            g2 = (x ** 4 - h * self.B1.I * self.A1) ** 2


    def getCoordinate(self, t):
        time = np.mat([1.0, t, t ** 2, t ** 3]).T
        x = self.b.T * time
        xx = np.mat([1.0, x, x ** 2, x ** 3, x ** 4]).T
        y = self.a.T * xx
        return x[0, 0], y[0, 0]

    def getCoordinate_(self, x):
        xx = np.mat([1.0, x, x ** 2, x ** 3, x ** 4]).T
        y = self.a.T * xx
        return y[0, 0]

    def getControl(self, t):
        x = (self.b.T * np.mat([1, t, t ** 2, t ** 3]).T)[0, 0]
        x1 = (self.b[1:4].T * np.mat([1.0, 2.0 * t, 3.0 * t ** 2]).T)[0, 0]
        x2 = (self.b[2:4].T * np.mat([2.0, 6.0 * t]).T)[0, 0]
        y1 = (self.a[1:5].T * np.mat([x1, 2.0 * x * x1, 3.0 * x ** 2 * x1, 4.0 * x ** 3 * x1]).T)[0, 0]
        y2 = (self.a[1:5].T * np.mat([x2, 2.0 * x1 ** 2 + 2.0 * x * x2, 6.0 * x * x1 ** 2 + 3.0 * x ** 2 * x2, 12.0 * x ** 2 * x1 ** 2 + 4.0 * x ** 3 * x2]).T)[0, 0]
        v = sqrt(x1 ** 2 + y1 ** 2)
        w = (y2 * x1 - y1 * x2) / (x1 ** 2 + y1 ** 2)
        return v, w

if __name__ == "__main__":
    uav1 = UAV([5.0, 5.0, pi / 4, 0.0], [30.0, 25.0, pi / 12, 0.0], 1.0, 0.0001)
    uav2 = UAV([5.0, 5.0, pi / 4, 0.0], [30.0, 25.0, pi / 12, 0.0], 20, 0.0001)
    p1 = []
    p2 = []
    p3 = []
    p4 = [[5.0, 5.0, pi / 4]]
    for t in np.arange(0, 1.0, 0.01):
        p1.append(uav1.getCoordinate(t))
    for t in np.arange(0.01, 1.0, 0.01):
        v, w = uav1.getControl(t)
        theta = p4[-1][2] + w * 0.01
        x = p4[-1][0] + v * cos(theta) * 0.01
        y = p4[-1][1] + v * sin(theta) * 0.01
        p4.append([x, y, theta])
    for t in np.arange(0, 20, 0.01):
        p2.append(uav2.getCoordinate(t))
    for x in np.arange(5.0, 30.0, 0.1):
        p3.append([x, uav1.getCoordinate_(x)])
    figure, axes = plt.subplots()
    axes.plot([n[0] for n in p1], [n[1] for n in p1], color='g', label='uav1')
    axes.plot([n[0] for n in p2], [n[1] for n in p2], color='r', label='uav2')
    axes.plot([n[0] for n in p3], [n[1] for n in p3], color='b', label='uav2_')
    axes.legend()
    plt.show()
    plt.plot([n[0] for n in p4], [n[1] for n in p4])
    plt.show()