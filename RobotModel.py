import numpy as np
from scipy import optimize
from math import tan, cos, sin, atan2, pi, sqrt
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def dist(x, y):
    return sqrt((x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2)


class Robot(object):
    Pos = dict()

    def __init__(self, q0, index, a4, mu, mu_):
        self.q = list(q0)          # the robot's real-time position
        self.obstacles = []
        Robot.Pos[index] = list(q0)
        self.neighborhood = []
        self.distance = dict()
        self.a_f = dict()
        self.mu = mu
        self.mu_ = mu_
        self.index = index
        self.a = np.mat([0.0, 0.0, 0.0, 0.0, 0.0]).T
        self.a[4, 0] = a4
        self.b = np.mat([0.0, 0.0, 0.0, 0.0]).T
        self.traj = [list(q0)]
        self.t0 = 0.0          # t0 and tf are two vars to compute the cruise trajectory
        self.tf = 0.0
        self.priority = {'time': 0, 'id': index, 'reach': True, 'count': 0}

    def setT0(self, t0):
        self.t0 = t0

    def setTf(self, tf):
        self.tf = tf

    def update_traj(self, q0, qf, t0=0.0):
        tf = self.tf
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
        traj = [[], []]
        for t in np.arange(t0, tf, 1.0):
            x = (self.b.T * np.mat([1, t, t ** 2, t ** 3]).T)[0, 0]
            y = (self.a.T * np.mat([1, x, x ** 2, x ** 3, x ** 4]).T)[0, 0]
            traj[0].append(x)
            traj[1].append(y)
        return traj

    def cruise(self, t):
        x = (self.b.T * np.mat([1, t, t ** 2, t ** 3]).T)[0, 0]
        x1 = (self.b[1:4].T * np.mat([1.0, 2.0 * t, 3.0 * t ** 2]).T)[0, 0]
        x2 = (self.b[2:4].T * np.mat([2.0, 6.0 * t]).T)[0, 0]
        y1 = (self.a[1:5].T * np.mat([x1, 2.0 * x * x1, 3.0 * x ** 2 * x1, 4.0 * x ** 3 * x1]).T)[0, 0]
        y2 = (self.a[1:5].T * np.mat([x2, 2.0 * x1 ** 2 + 2.0 * x * x2, 6.0 * x * x1 ** 2 + 3.0 * x ** 2 * x2, 12.0 * x ** 2 * x1 ** 2 + 4.0 * x ** 3 * x2]).T)[0, 0]
        v = sqrt(x1 ** 2 + y1 ** 2)
        w = (y2 * x1 - y1 * x2) / (x1 ** 2 + y1 ** 2)
        return v, w

    def cruise2(self, qf):
        d = dist(self.q, qf)
        theta = atan2(qf[1] - self.q[1], qf[0] - self.q[0])
        v = 0.2 * d / (d + 0.01)
        w = 0.03 * (theta - self.q[2])
        return v, w

    def formation(self):
        xi = self.q[0]
        yi = self.q[1]
        theta_i = self.q[2]
        v = 0.0
        omega = 0.0
        for i in self.neighborhood:
            xj = Robot.Pos[i][0]
            yj = Robot.Pos[i][1]
            dist = sqrt((xi - xj) ** 2 + (yi - yj) ** 2)
            psi = atan2(yj - yi, xj - xi)
            v = v + self.a_f[i] * (dist - self.distance[i]) * cos(psi - theta_i)
            omega = omega + self.a_f[i] * (dist - self.distance[i]) * sin(psi - theta_i)
        v = v * self.mu
        omega = omega * self.mu_
        return v, omega
        # if omega > 30 * pi / 180:
        #     omega = 30 * pi / 180
        # if omega < -30 * pi / 180:
        #     omega = -30 * pi / 180
        # self.q[0] += v * cos(self.q[2]) * 0.1
        # self.q[1] += v * sin(self.q[2]) * 0.1
        # self.q[2] += omega * 0.1
        # self.traj.append(self.q)

    # def collisionFree(self):

    def changeForCoe(self, mu, mu_):
        self.mu = mu
        self.mu_ = mu_

    def changePos(self, p):
        self.q = p

    def addNeighbor(self, index, r):
        """Add a neighborhood to the robot:
        Args:
            index: The label number of the robot to be added
            r: The desired distance between the robots
        """
        self.neighborhood.append(index)
        self.distance[index] = r
        t = [dist(self.q, Robot.Pos[key]) for key in self.neighborhood]
        j = self.neighborhood[t.index(max(t))]
        count = self.neighborhood.__len__()
        for i in self.neighborhood:
            if i == j:
                self.a_f[i] = 0.5
            else:
                self.a_f[i] = 1.0 / 2 / (count - 1)

    def removeAllNeighbor(self):
        self.neighborhood = []
        self.distance = dict()
        self.a_f = dict()

    def addObstacle(self, q, r, v=0, theta=0):
        """Add an obstacle to the robot:
        Args:
            q: The position of obstacle
            r: The radius of obstacle
            v: The velocity of obstacle
            theta: The heading angle of obstacle
        """
        obs = {"pos": q, "radius": r, "velocity": v, "angle": theta}
        self.obstacles.append(obs)


def main():
    goal = (6.0, 6.5)
    start_angle = -0.785 #-1.248
    angle = atan2(goal[1] - 42.5, goal[0] - 40.5)
    angle = -2.313
    r1 = Robot((40.5, 42.5, start_angle, 0.0), 1, 0.00001, 0.5, 4.0)
    r2 = Robot((43.5, 45.5, 0.0, 0.0), 2, 0.00001, 0.5, 4.0)
    r3 = Robot((45.5, 45.5, 0.0, 0.0), 3, 0.00001, 0.5, 4.0)
    r1.addNeighbor(2, 5)
    r1.addNeighbor(3, 5)
    r2.addNeighbor(1, 5)
    r2.addNeighbor(3, 5)
    r3.addNeighbor(1, 5)
    r3.addNeighbor(2, 5)
    robot = [r1, r2, r3]
    r1.setTf(100)
    traj = r1.update_traj((40.5, 42.5, start_angle, 0.0), (goal[0], goal[1], angle, 0.0))
    # plt.plot(traj[0], traj[1])
    # plt.show()
    dt = 0.5
    for t in np.arange(dt, 1000, dt):
        # v1, w1 = r1.cruise(t)
        v1, w1 = r1.cruise2((goal[0], goal[1], angle, 0.0))
        v2 = 0
        w2 = 0
        # v2, w2 = r1.formation()
        v = v1 + v2
        w = w1 + w2
        if w > 30 * pi / 180:
            w = 30 * pi / 180
        if w < -30 * pi / 180:
            w = -30 * pi / 180
        if ((start_angle < pi / 2 and start_angle > -1 * pi / 2) and (angle < pi / 2 and angle > -1 * pi / 2)) or \
            (((start_angle > -1 * pi and start_angle < -1 * pi / 2) or (start_angle > pi / 2 and start_angle < pi)) and
             ((angle > -1 * pi and angle < -1 * pi / 2) or (angle > pi / 2 and angle < pi))):
            theta = r1.q[2] = w * dt
        else:
            theta = r1.q[2] + w * dt
        x = r1.q[0] + v * cos(theta) * dt
        y = r1.q[1] + v * sin(theta) * dt
        r1.changePos([x, y, theta, v])
        r1.traj.append(r1.q)
        # r1.update_traj(r1.q, (goal[0], goal[1], angle, 0.0), t0=t)
        for r in [r2, r3]:
            v, w = r.formation()
            v = v + v1
            w = w + w1
            if w > 30 * pi / 180:
                w = 30 * pi / 180
            if w < -30 * pi / 180:
                w = -30 * pi / 180
            theta = r.q[2] + w * dt
            x = r.q[0] + v * cos(theta) * dt
            y = r.q[1] + v * sin(theta) * dt
            r.changePos([x, y, theta, v])
            r.traj.append(r.q)
        for r in robot:
            Robot.Pos[r.index] = r.q
    fig, axes = plt.subplots()
    color = ['r', 'b', 'g']
    for i in xrange(0, 3):
        axes.plot([p[0] for p in robot[i].traj], [p[1] for p in robot[i].traj], color=color[i],
                  label='robot{0}'.format(i + 1))
    axes.legend()
    axis = plt.gca()
    axis.set_aspect(1)
    plt.show()
    d1 = []
    d2 = []
    d3 = []
    for i in xrange(r1.traj.__len__()):
        d1.append(dist(r1.traj[i], r2.traj[i]))
        d2.append(dist(r1.traj[i], r3.traj[i]))
        d3.append(dist(r2.traj[i], r3.traj[i]))
    plt.plot(d1, color='c', label='d12')
    plt.plot(d2, color='r', label='d13')
    plt.plot(d3, color='b', label='d23')
    plt.legend()
    plt.grid(True)
    plt.show()

    # robot = [r1, r2, r3]
    # for t in np.arange(0.1, 30, 0.1):
    #     for r in robot:
    #         r.formation()
    #     for r in robot:
    #         Robot.Pos[r.index] = r.q
    # fig, axes = plt.subplots()
    # color = ['r', 'b', 'g']
    # for i in xrange(0, 3):
    #     axes.plot([p[0] for p in robot[i].traj], [p[1] for p in robot[i].traj], color=color[i],
    #               label='robot{0}'.format(i + 1))
    # axes.legend()
    # axis = plt.gca()
    # axis.set_aspect(1)
    # plt.show()

if __name__ == "__main__":
    main()
    # r1 = Robot((8.5, 80.5, 0 * pi / 4, 0.0), 1, 0.000001, 6.5, 7.0)
    # r2 = Robot((6.5, 76.5, 0.0, 0.0), 2, 0.001, 6.5, 7.0)
    # r3 = Robot((6.5, 84.5, 0.0, 0.0), 3, 0.001, 6.5, 7.0)
    # r4 = Robot((4.0, 80.0, 0.0, 0.0), 4, 0.001, 1.5, 7.0)
    # radius = 5
    # r1.addNeighbor(2, radius)
    # r1.addNeighbor(3, radius)
    # r1.addNeighbor(4, radius)
    # r2.addNeighbor(1, radius)
    # r2.addNeighbor(4, radius)
    # r3.addNeighbor(1, radius)
    # r3.addNeighbor(4, radius)
    # r4.addNeighbor(1, radius)
    # r4.addNeighbor(2, radius)
    # r4.addNeighbor(3, radius)
    # robot = [r1, r2, r3, r4]
    # angle = atan2(90.0 - 80.5, 80.5 - 8.5)
    # r1.setTf(100)
    # r1.update_traj((8.5, 80.5, angle, 0.0), (80.5, 90.0, angle, 0.0))
    # dt = 0.5
    # for t in np.arange(dt, 100, dt):
    #     v1, w1 = r1.cruise(t)
    #     v2 = 0
    #     w2 = 0
    #     # v2, w2 = r1.formation()
    #     v = v1 + v2
    #     w = w1 + w2
    #     if w > 30 * pi / 180:
    #         w = 30 * pi / 180
    #     if w < -30 * pi / 180:
    #         w = -30 * pi / 180
    #     theta = r1.q[2] + w * dt
    #     x = r1.q[0] + v * cos(theta) * dt
    #     y = r1.q[1] + v * sin(theta) * dt
    #     r1.changePos([x, y, theta, v])
    #     r1.traj.append(r1.q)
    #     r1.update_traj(r1.q, (80.5, 90.0, angle, 0.0), t0=t)
    #     for r in [r2, r3, r4]:
    #         v, w = r.formation()
    #         v = v + v1
    #         w = w + w1
    #         if w > 30 * pi / 180:
    #             w = 30 * pi / 180
    #         if w < -30 * pi / 180:
    #             w = -30 * pi / 180
    #         theta = r.q[2] + w * dt
    #         x = r.q[0] + v * cos(theta) * dt
    #         y = r.q[1] + v * sin(theta) * dt
    #         r.changePos([x, y, theta, v])
    #         r.traj.append(r.q)
    #     for r in robot:
    #         Robot.Pos[r.index] = r.q
    # fig, axes = plt.subplots()
    # color = ['r', 'b', 'g', 'c']
    # for i in xrange(0, 4):
    #     axes.plot([p[0] for p in robot[i].traj], [p[1] for p in robot[i].traj], color=color[i], label='robot{0}'.format(i + 1))
    # axes.legend()
    # axis = plt.gca()
    # axis.set_aspect(1)
    # plt.show()
    # d1 = []
    # d2 = []
    # d3 = []
    # d4 = []
    # d5 = []
    # for i in xrange(r1.traj.__len__()):
    #     d1.append(dist(r1.traj[i], r2.traj[i]))
    #     d2.append(dist(r1.traj[i], r3.traj[i]))
    #     d3.append(dist(r1.traj[i], r4.traj[i]))
    #     d4.append(dist(r2.traj[i], r4.traj[i]))
    #     d5.append(dist(r3.traj[i], r4.traj[i]))
    # plt.plot(d1, color='c', label='d12')
    # plt.plot(d2, color='r', label='d13')
    # plt.plot(d3, color='b', label='d14')
    # plt.plot(d4, color='g', label='d24')
    # plt.plot(d5, color='y', label='d34')
    # plt.legend()
    # plt.grid(True)
    # plt.show()
    #
    # fig = plt.figure()
    # ax = plt.axes(xlim=(0, 90), ylim=(0, 70))
    # line1, = ax.plot([], [], lw=2, color='r')
    # line2, = ax.plot([], [], lw=2, color='b')
    # line3, = ax.plot([], [], lw=2, color='g')
    # line4, = ax.plot([], [], lw=2, color='c')
    # # initialization function: plot the background of each frame
    # def init():
    #     line1.set_data([], [])
    #     line2.set_data([], [])
    #     line3.set_data([], [])
    #     line4.set_data([], [])
    #     return line1, line2, line3, line4
    #
    #
    # # animation function.  this is called sequentially
    # def animate(i):
    #     line1.set_data([p[0] for p in robot[0].traj[0:i]], [p[1] for p in robot[0].traj[0:i]])
    #     line2.set_data([p[0] for p in robot[1].traj[0:i]], [p[1] for p in robot[1].traj[0:i]])
    #     line3.set_data([p[0] for p in robot[2].traj[0:i]], [p[1] for p in robot[2].traj[0:i]])
    #     line4.set_data([p[0] for p in robot[3].traj[0:i]], [p[1] for p in robot[3].traj[0:i]])
    #     return line1, line2, line3, line4


    # call the animator.  blit=true means only re-draw the parts that have changed.
    # anim = FuncAnimation(fig, animate, init_func=init, frames=r1.traj.__len__(), interval=30, blit=True)
    # anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])


