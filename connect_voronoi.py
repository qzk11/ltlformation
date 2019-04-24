from MotionModel import MotionFts
from ActionModel import ActionModel
from MotActModel import MotActModel
from planner import ltl_planner
from RobotModel import Robot
from Voronoi import VoronoiDiag
from matplotlib.lines import Line2D
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.collections import PolyCollection
from shapely.geometry import Point, LineString
import matplotlib.pyplot as plt
import numpy as np
from math import sqrt, pow, floor, ceil, exp, atan2, pi, cos, sin, fabs
import time
import Queue
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

class Test(object):
    def __init__(self):
        self.vd = VoronoiDiag()
        self.axes = self.vd.getvd()
        self.centroids, self.mass = self.vd.tt()
        edge, label = self.vd.graph()
        self.region_v = dict()
        Region_dict_v = dict()
        edges_v = list()
        symbol = [['fuel'], ['w', 'f'], ['w'], ['f'], ['']]
        for r in range(self.centroids.__len__()):
            self.region_v[r+1] = (self.centroids[r].x, self.centroids[r].y)
            Region_dict_v[self.region_v[r+1]] = {'reg' + label[r].__str__()}
            for s in symbol[label[r] - 1]:
                if s != '':
                    Region_dict_v[self.region_v[r+1]].add(s)
        Region_symbols = ['reg1', 'reg2', 'reg3', 'reg4', 'reg5', 'fuel', 'w', 'f']
        for e in edge:
            edges_v.append((self.region_v[e[0]], self.region_v[e[1]]))
        self.vd_motion = MotionFts(Region_dict_v, Region_symbols, "voronoi_motion")
        self.vd_motion.add_un_edges(edges_v, unit_cost=1)
        ######################################
        self.region = dict()
        self.region['reg1'] = (43.0, 42.0)
        self.region['reg2'] = (20.0, 75.0)
        self.region['reg3'] = (75.0, 83.0)
        self.region['reg4'] = (78.0, 18.0)
        self.region['reg5'] = (19.0, 17.0)
        Region_dict = {self.region['reg1']: {'reg1', 'fuel'},
                       self.region['reg2']: {'reg2', 'w', 'f'},
                       self.region['reg3']: {'reg3', 'w'},
                       self.region['reg4']: {'reg4', 'f'},
                       self.region['reg5']: {'reg5'}
                       }
        Region_symbols = ['reg1', 'reg2', 'reg3', 'reg4', 'reg5', 'fuel', 'w', 'f']
        self.edges = [(self.region['reg1'], self.region['reg2']),
                 (self.region['reg1'], self.region['reg3']),
                 (self.region['reg1'], self.region['reg4']),
                 (self.region['reg1'], self.region['reg5']),
                 (self.region['reg2'], self.region['reg3']),
                 (self.region['reg2'], self.region['reg5']),
                 (self.region['reg3'], self.region['reg1']),
                 (self.region['reg3'], self.region['reg2']),
                 (self.region['reg3'], self.region['reg4']),
                 (self.region['reg4'], self.region['reg3']),
                 (self.region['reg4'], self.region['reg5']),
                 (self.region['reg5'], self.region['reg1']),
                 (self.region['reg5'], self.region['reg2']),
                 (self.region['reg5'], self.region['reg4'])]
        ############### Robot1 ###############
        print("Robot 1")
        R1_init_region = (6.0, 6.5, 0.0, 0.0)
        R1_motion = MotionFts(Region_dict, Region_symbols, "R1_Region")
        R1_motion.set_initial(R1_init_region)
        R1_motion.add_di_edges(self.edges, unit_cost=1)

        R1_action_dict = dict()
        R1_action = ActionModel(R1_action_dict)
        R1_model = MotActModel(R1_motion, R1_action)
        # Soft task
        R1_task = '<>(reg5 && <>(reg1 && <>(reg3 && <>reg4)))'
        # Hard task
        R1_task = '[]<>(w && <>reg4) && []<>fuel'
        # Coverage
        # R1_task = '<>fm1 && <>reg1 && <>reg4 && []<>reg5 && [](fm1 -> X reg3)'
        self.R1_planner = ltl_planner(R1_model, R1_task, None)
        self.R1_planner.optimal()

        ############### Robot2 ###############
        print("Robot 2")
        R2_init_region = (8.5, 8.5, 0.0, 0.0)
        R2_motion = MotionFts(Region_dict, Region_symbols, "R2_Region")
        R2_motion.set_initial(R2_init_region)
        R2_motion.add_di_edges(self.edges, unit_cost=1)

        R2_action_dict = dict()
        R2_action = ActionModel(R2_action_dict)
        R2_model = MotActModel(R2_motion, R2_action)
        # Soft task
        R2_task = '[]<>reg3 && []<>reg4 && []!(reg1 && reg2)'
        # Hard task
        R2_task = '[]<>(f && <>reg3) && []<>fuel'
        # R2_task = '<>fm2 && [](fm2 -> X reg5)'
        self.R2_planner = ltl_planner(R2_model, R2_task, None)
        self.R2_planner.optimal()

        ############### Robot3 ###############
        print("Robot 3")
        R3_init_region = (5.5, 5.5, 0.0, 0.0)
        R3_motion = MotionFts(Region_dict, Region_symbols, "R3_Region")
        R3_motion.set_initial(R3_init_region)
        R3_motion.add_di_edges(self.edges, unit_cost=1)

        R3_action_dict = dict()
        R3_action = ActionModel(R3_action_dict)
        R3_model = MotActModel(R3_motion, R3_action)
        R3_task = '[]<>(f && <>reg5) && []<>(w && <>reg5) && []<>fuel'
        self.R3_planner = ltl_planner(R3_model, R3_task, None)
        self.R3_planner.optimal()

        #####################################
        self.robot1 = Robot(R1_init_region, 1, 0.00001, 0.5, 4.0)
        self.robot2 = Robot(R2_init_region, 2, 0.00001, 0.5, 4.0) #5.5
        self.robot3 = Robot(R3_init_region, 3, 0.00001, 0.5, 4.0) #5.5

        radius = 5
        self.robot1.addNeighbor(2, radius)
        self.robot1.addNeighbor(3, radius)
        self.robot2.addNeighbor(1, radius)
        self.robot2.addNeighbor(3, radius)
        self.robot3.addNeighbor(1, radius)
        self.robot3.addNeighbor(2, radius)

    def reach(self, s, d):
        if (sqrt((s[0] - d[0]) ** 2 + (s[1] - d[1]) ** 2) < 2.0):
            return True
        return False

    def distance(self, pose1, pose2):
        return sqrt((pose1[0] - pose2[0]) ** 2 + (pose1[1] - pose2[1]) ** 2) + 0.001

    def elect(self, priority, N):
        index = []
        for p in priority:
            if p['count'] >= N:
                index.append(p)
        if len(index) == 0:
            for p in priority:
                index.append(p)
        leader = index[0]
        if len(index) > 1:
            for p in index[1:]:
                if leader['time'] == p['time']:
                    if leader['id'] < p['id']:
                        m = p
                        n = leader
                    else:
                        m = leader
                        n = p
                else:
                    if leader['time'] > p['time']:
                        m = p
                        n = leader
                    else:
                        m = leader
                        n = p
                if m['reach']:
                    leader = m
                else:
                    if n['reach']:
                        leader = n
                    else:
                        leader = m
        for p in index:
            if p['id'] != leader['id']:
                p['count'] = p['count'] + 1
            else:
                p['count'] = 0
        return leader['id']

    def get_region(self, point):
        p = Point(point[0], point[1])
        for poly in self.vd.poly_regions:
            if p.within(poly):
                index = self.vd.poly_regions.index(poly)
                return (self.centroids[index].x, self.centroids[index].y)
        return None

    def run(self):
        dt = 0.15
        count = 1
        finish = [0, 0, 1, 1]
        first = True   # the first loop towards the goal region
        dist = [[], [], []]
        robot = [self.robot1, self.robot2, self.robot3]
        start_angle = 0.0
        angle = 0.0
        t0 = 0.0      # compute the trajectory
        ration = [1.0, 1.0, 1.0]
        v_min = 0.1
        v_max = 1.0
        theta_max = 13
        planner = [self.R1_planner, self.R2_planner, self.R3_planner]
        start_elect = False              # signal to trigger a new round of election
        v1 = 0       # the control input of cruise
        w1 = 0
        leader_list = []
        goal_list = []
        #### compute the first leader #############
        l = self.elect([self.robot1.priority, self.robot2.priority, self.robot3.priority], 3)
        leader = robot[l - 1]
        goal = planner[l - 1].next_move
        last = goal
        ##########################################
        hole_label = False   #hole from reg2 to reg1
        leader_list.append(l - 1)
        goal_list.append([key for key, value in self.region.items() if cmp(value, goal) == 0])
        vd_reg = Queue.Queue()
        b = 0.1    #coef for voronoi control input
        t = 0
        while True:
            # start a new round of election
            if start_elect:
                while True:
                    leader.priority['time'] = t
                    planner[l - 1].find_next_move()
                    # Delete the repeated regions
                    while cmp(goal, planner[l - 1].next_move) == 0:
                        planner[l - 1].find_next_move()
                    if planner[l - 1].segment == 'loop':
                        finish[l - 1] = 1
                    for i in xrange(3):
                        next_region = planner[i].next_move
                        if (cmp(goal, next_region) == 0) or (goal, next_region) in self.edges:
                            robot[i].priority['reach'] = True
                        else:
                            robot[i].priority['reach'] = False
                    l = self.elect([self.robot1.priority, self.robot2.priority, self.robot3.priority], 3)
                    leader = robot[l - 1]
                    print("The leader is Robot: %s, and the goal region is %s\n" % (l - 1, planner[l - 1].next_move))
                    # next goal region equal to current region
                    leader_list.append(l - 1)
                    goal_list.append(
                        [key for key, value in self.region.items() if cmp(value, planner[l - 1].next_move) == 0])
                    if cmp(goal, planner[l - 1].next_move) == 0:
                        goal = planner[l - 1].next_move
                        print("Time %s: Robot%s has been in region %s\n" % (t, l - 1, planner[l - 1].next_move))
                    else:
                        goal = planner[l - 1].next_move
                        start_elect = False
                        break
            if cmp(last, goal) != 0 and (last, goal) not in self.edges:
                hole_label = True
                for r in self.vd_motion.dijkstra_path(self.get_region(leader.q), 'reg5'):
                    vd_reg.put(r)
            else:
                hole_label = False
                label = list(self.region.keys())[self.region.values().index(goal)]
                for r in self.vd_motion.dijkstra_path(self.get_region(leader.q), label):
                    vd_reg.put(r)
            while not vd_reg.empty():
                reg = vd_reg.get()
                vd_index = list(self.region_v.keys())[list(self.region_v.values()).index(reg)] - 1
                ##############get the closest point############
                line = LineString([(leader.q[0], leader.q[1]), (reg[0], reg[1])])
                vd_index_cur = list(self.region_v.keys())[list(self.region_v.values()).index(self.get_region((leader.q[0], leader.q[1])))] - 1
                p1 = self.vd.poly_regions[vd_index]
                p2 = self.vd.poly_regions[vd_index_cur]
                inter = p1.intersection(p2)
                mid_point = None
                if not inter.is_empty and line.intersection(inter).is_empty:
                    mid_point = min([p for p in list(inter.coords)], key=lambda p: Point([p]).distance(line))
                    another = [p for p in list(inter.coords) if cmp(mid_point, p) != 0][0]
                    another = (another[0] - mid_point[0], another[1] - mid_point[1])
                    mid_point = (mid_point[0] + another[0] * 0.2, mid_point[1] + another[1] * 0.2)
                ###############################################
                while not self.reach(leader.q, reg):
                    t = count * dt
                    if mid_point is not None and self.reach(leader.q, mid_point):
                        mid_point = None
                    if mid_point is not None:
                        u1 = self.mass[vd_index_cur] * (mid_point[0] - leader.q[0])
                        u2 = self.mass[vd_index_cur] * (mid_point[1] - leader.q[1])
                    else:
                        u1 = self.mass[vd_index] * (reg[0] - leader.q[0])
                        u2 = self.mass[vd_index] * (reg[1] - leader.q[1])
                    v1 = u1 * cos(leader.q[2]) + u2 * sin(leader.q[2])
                    w1 = -1.0 / b * u1 * sin(leader.q[2]) + 1.0 / b * u2 * cos(leader.q[2])
                    # v1, w1 = leader.cruise2(reg)
                    v2 = 0
                    w2 = 0
                    # v2, w2 = leader.formation()
                    v = v1 + v2
                    w = w1 + w2
                    if v < v_min:
                        v = v_min
                    if v > v_max:
                        v = v_max
                    if fabs(w) > theta_max * pi / 180:
                        w = w / fabs(w) * theta_max * pi / 180
                    if fabs(w) > v * 10:
                        w = w / fabs(w) * v * 10
                    theta = leader.q[2] + w * dt
                    x = leader.q[0] + v * cos(theta) * dt
                    y = leader.q[1] + v * sin(theta) * dt
                    leader.changePos([x, y, theta, v])
                    leader.traj.append(leader.q)
                    # compute for follower robots
                    for r in robot:
                        if r.index != leader.index:
                            v2, w2 = r.formation()
                            v = v1 + v2
                            w = w1 + w2
                            # v = v2
                            # w = w2
                            if v < v_min:
                                v = v_min
                            if v > v_max:
                                v = v_max
                            if fabs(w) > theta_max * pi / 180:
                                w = w / fabs(w) * theta_max * pi / 180
                            if fabs(w) > v * 10:
                                w = w / fabs(w) * v * 10
                            theta = r.q[2] + w * dt
                            x = r.q[0] + v * cos(theta) * dt
                            y = r.q[1] + v * sin(theta) * dt
                            r.changePos([x, y, theta, v])
                            r.traj.append(r.q)
                    count += 1
                    for r in robot:
                        Robot.Pos[r.index] = r.q
                    dist[0].append(self.distance(self.robot1.traj[-1], self.robot2.traj[-1]))
                    dist[1].append(self.distance(self.robot1.traj[-1], self.robot3.traj[-1]))
                    dist[2].append(self.distance(self.robot2.traj[-1], self.robot3.traj[-1]))
            print("Time %s: Robot%s finished its motion %s\n" % (t, l - 1, goal))
            if hole_label:
                hole_label = False
            else:
                start_elect = True
            last = goal
            # if all(item > 0 for item in finish):
            #     break
            if t > 2400:
                break
        print('The leader list: ' + leader_list.__str__())
        print('The goal list: ' + goal_list.__str__())
        # fig, axes = plt.subplots()
        # self.axes.set_xlim(-5, 105)
        # self.axes.set_ylim(-5, 105)
        # self.axes.set_xlabel('x(m)')
        # self.axes.set_ylabel('y(m)')
        plt.show()
        colors = ['r', 'b', 'c', 'g', 'y']
        axes = plt.axes()
        verts = list()
        for p in self.vd.poly:
            verts.append(p.exterior.coords)
        pc = PolyCollection(verts=verts, facecolors=colors)
        axes.add_collection(pc)
        axes.set_xlim(0, 100)
        axes.set_ylim(0, 100)
        axes.set_xlabel('x(m)')
        axes.set_ylabel('y(m)')
        num = -1
        # axes.plot([p[0] for p in self.robot1.traj[0:num]], [p[1] for p in self.robot1.traj[0:num]], label='Robot1')
        # axes.plot([p[0] for p in self.robot2.traj[0:num]], [p[1] for p in self.robot2.traj[0:num]], label='Robot2')
        # axes.plot([p[0] for p in self.robot3.traj[0:num]], [p[1] for p in self.robot3.traj[0:num]], label='Robot3')

        # axes.plot([p[0] for p in robot1.traj[5152:6559]], [p[1] for p in robot1.traj[5152:6559]], label = 'UAV1')
        # axes.plot([p[0] for p in robot2.traj[5152:6559]], [p[1] for p in robot1.traj[5152:6559]], label = 'UAV2')
        # axes.plot([p[0] for p in robot3.traj[5152:6559]], [p[1] for p in robot1.traj[5152:6559]], label = 'UAV3')

        axes.plot([p[0] for p in self.robot1.traj[0:7551]], [p[1] for p in self.robot1.traj[0:7551]], label='Robot1')
        axes.plot([p[0] for p in self.robot2.traj[0:7551]], [p[1] for p in self.robot2.traj[0:7551]], label='Robot2')
        axes.plot([p[0] for p in self.robot3.traj[0:7551]], [p[1] for p in self.robot3.traj[0:7551]], label='Robot3')

        axis = plt.gca()
        axis.set_aspect(1)
        # plt.title('The robots\' motion tracks')
        # plt.title('The robots\' motion tracks between 5586s and 6289.5s')
        plt.legend(loc='upper center', ncol=1, fancybox=True, shadow=True)
        plt.savefig('track1.eps', bbox_inches = 'tight')
        plt.show()

        # plot the distance between robots during formation1
        plt.plot(np.arange(0, dist[0].__len__(), 1) * 0.015, dist[0], color='c', label='d12')
        plt.plot(np.arange(0, dist[0].__len__(), 1) * 0.015, dist[1], color='r', label='d13')
        plt.plot(np.arange(0, dist[0].__len__(), 1) * 0.015, dist[2], color='b', label='d14')
        plt.legend(loc='lower right', ncol=1, fancybox=True, shadow=True)
        # plt.title('The distance between robots')
        plt.xlabel('t(s)')
        plt.ylabel('distance(m)')
        plt.grid(True)
        plt.savefig('distance.eps', bbox_inches = 'tight')
        plt.show()

        fig = plt.figure()
        ax = plt.axes(xlim=(0, 90), ylim=(0, 100))
        for r in self.region.keys():
            circle = plt.Circle(xy=(self.region[r][0], self.region[r][1]), radius=5)
            ax.add_artist(circle)
        line1, = ax.plot([], [], lw=2, color='b')
        line2, = ax.plot([], [], lw=2, color='r')
        line3, = ax.plot([], [], lw=2, color='g')

        # initialization function: plot the background of each frame
        def init():
            line1.set_data([], [])
            line2.set_data([], [])
            line3.set_data([], [])
            return line1, line2, line3

        # animation function.  this is called sequentially
        def animate(i):
            line1.set_data([p[0] for p in robot[0].traj[0:i]], [p[1] for p in robot[0].traj[0:i]])
            line2.set_data([p[0] for p in robot[1].traj[0:i]], [p[1] for p in robot[1].traj[0:i]])
            line3.set_data([p[0] for p in robot[2].traj[0:i]], [p[1] for p in robot[2].traj[0:i]])
            return line1, line2, line3

        # call the animator.  blit=true means only re-draw the parts that have changed.
        # anim = FuncAnimation(fig, animate, init_func=init, frames=self.robot1.traj.__len__(), interval=30, blit=True)
        # anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])


if __name__ == "__main__":
    test = Test()
    test.run()